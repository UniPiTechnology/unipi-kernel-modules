/*
 * UniPi PLC device driver - Copyright (C) 2018 UniPi Technology
 * Author: Tomas Knot <tomasknot@gmail.com>
 *
 *  Based on the SC16IS7xx driver by Jon Ringle <jringle@gridpoint.com>,
 *  which was in turn based on max310x.c, by Alexander Shiyan <shc_work@mail.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

/************
 * Includes *
 ************/

#include "unipi_common.h"
#include "unipi_sysfs.h"
#include "unipi_uart.h"
#include "unipi_platform.h"
#include "unipi_gpio.h"
#include "unipi_iio.h"
#include "unipi_misc.h"
#include "unipi_spi.h"
#include "unipi_tty.h"

/* using trace_printk or printk ???*/

#if NEURONSPI_DETAILED_DEBUG > 1
# define unipi_spi_trace_1(f, args...)	printk(f, ##args)
#else
# define unipi_spi_trace_1(f, args...)
#endif

#if NEURONSPI_DETAILED_DEBUG > 0
# define unipi_spi_trace(f, args...)	printk(f, ##args)
#else
# define unipi_spi_trace(f, args...)
#endif


/********************
 * Data Definitions *
 ********************/

MODULE_DEVICE_TABLE(of, neuronspi_id_match);

struct spi_driver neuronspi_spi_driver =
{
	.driver =
	{
		.name			= NEURON_DRIVER_NAME,
		.of_match_table	= of_match_ptr(neuronspi_id_match)
	},
	.probe				= neuronspi_spi_probe,
	.remove				= neuronspi_spi_remove,
};

struct file_operations file_ops =
{
	.open 				= neuronspi_open,
	.read 				= neuronspi_read,
	.write 				= neuronspi_write,
	.release 			= neuronspi_release,
	.owner				= THIS_MODULE
};

struct neuronspi_char_driver neuronspi_cdrv =
{
	.dev = NULL
};

struct mutex neuronspi_master_mutex;
struct mutex unipi_inv_speed_mutex;
int neuronspi_model_id = -1;
struct spi_device *neuronspi_s_dev[NEURONSPI_MAX_DEVS];  // global list of neuron spi devices
struct task_struct *neuronspi_invalidate_thread;

static u8 neuronspi_probe_count = 0;
static struct spinlock *neuronspi_probe_spinlock;
static struct sched_param neuronspi_sched_param = { .sched_priority = MAX_RT_PRIO / 2 };

/************************
 * Non-static Functions *
 ************************/
int _neuronspi_spi_send_op(struct spi_device* spi_dev, s32 trans_count, const struct neuronspi_op_buffer* send_buf,
                            struct neuronspi_op_buffer* recv_buf, s32 len, 
                            s32 freq, s32 delay, s32 send_header, u8 lock_val, u16 crc2);


int neuronspi_spi_send_const_op(struct spi_device* spi_dev, const struct neuronspi_op_buffer* send_buf,
                            struct neuronspi_op_buffer* recv_buf, s32 len, 
                            s32 freq, s32 delay)
{
	s32 trans_count = (len-1) / NEURONSPI_MAX_TX + 3; // number of transmissions
    return _neuronspi_spi_send_op(spi_dev,trans_count, send_buf,recv_buf,len,freq, delay, UNIPISPI_OP_MODE_SEND_HEADER, 0, 0);
}


/*
 *  send_header: bits of UNIPISPI_OP_MODE_SEND_HEADER | UNIPISPI_OP_MODE_DO_CRC | UNIPISPI_OP_MODE_HAVE_CRC_SPACE
 *  len:         length of second message. If DO_CRC is not set, len includes crc
 *  buffers:     must be long enough if HAVE_CRC_SPACE is set  
 *  returns:     0 if success
 */
int neuronspi_spi_send_op(struct spi_device* spi_dev, struct neuronspi_op_buffer* send_buf, struct neuronspi_op_buffer* recv_buf, s32 len, 
                          s32 freq, s32 delay, s32 send_header, u8 lock_val)
{
	u16 packet_crc = 0;
	s32 trans_count = 2; // number of transmissions

    if (send_header & UNIPISPI_OP_MODE_SEND_HEADER) {
		unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI Master Write(op1) %8ph\n", send_buf->first_message);
        if (send_header & UNIPISPI_OP_MODE_DO_CRC) {
            packet_crc = neuronspi_spi_crc(send_buf->first_message, 4, 0);
            *((u16*)(send_buf->first_message+4)) = packet_crc;
        }
    }

    if (len > 0) {
        if (send_header & UNIPISPI_OP_MODE_DO_CRC) {
            packet_crc = neuronspi_spi_crc(send_buf->second_message, len, packet_crc);
            if (send_header & UNIPISPI_OP_MODE_HAVE_CRC_SPACE) {
                // crc can be placed into data buffer
                send_buf->second_message[len] = packet_crc & 0xff;
                send_buf->second_message[len+1] = packet_crc >> 8;
                len += 2;
            } else {
                // crc needs own buffer and own chunk
                //crc_send_buf[0] = packet_crc;
                trans_count++;
            }
        }
		trans_count += (len-1) / NEURONSPI_MAX_TX + 1;
		unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI Master Write(%3d) %32ph\n", len, send_buf->second_message);
    }
    return _neuronspi_spi_send_op(spi_dev,trans_count, send_buf,recv_buf,len,freq, delay, send_header, lock_val, packet_crc);
}

int _neuronspi_spi_send_op(struct spi_device* spi_dev, s32 trans_count, const struct neuronspi_op_buffer* send_buf,
                            struct neuronspi_op_buffer* recv_buf, s32 len, 
                            s32 freq, s32 delay, s32 send_header, u8 lock_val, u16 crc2)
{
	s32 i = 0;
    s32 remain;
	int ret_code = 0;
	u16 recv_crc1 = 0;
	u16 recv_crc2;
	u16 packet_crc;
    u16 crc_send_buf[2];
    u16 crc_recv_buf[2];
    u8 opcode;
    int portindex;
    struct neuronspi_port* port;


	struct neuronspi_driver_data *d_data;
    struct spi_transfer* s_trans;

	mutex_lock(&neuronspi_master_mutex);
    d_data = spi_get_drvdata(spi_dev);

    // Check if there are running reserved operations
	if (d_data != NULL && d_data->reserved_device && lock_val != d_data->reserved_device) {
		recv_buf->first_message[0] = 0;
        if ((len > 0) && (recv_buf->second_message)) recv_buf->second_message[0] = 0;
        mutex_unlock(&neuronspi_master_mutex);
        return -1; // blocked by reservation
	}

	s_trans = kzalloc(sizeof(struct spi_transfer) * trans_count, GFP_ATOMIC);
	s_trans[0].delay_usecs = NEURONSPI_EDGE_DELAY;
	s_trans[0].bits_per_word = 8;
	s_trans[0].speed_hz = freq;

	s_trans[1].bits_per_word = 8;
	s_trans[1].speed_hz = freq;
    if (send_header) {
        s_trans[1].delay_usecs = delay;
        s_trans[1].len = NEURONSPI_FIRST_MESSAGE_LENGTH;
        s_trans[1].tx_buf = send_buf->first_message;
        s_trans[1].rx_buf = recv_buf->first_message;
    }
    if (len > 0) {
		remain = len;
		for (i = 2; i < trans_count; i++) {
			memset(&(s_trans[i]), 0, sizeof(s_trans[i]));
			s_trans[i].delay_usecs = 0;
			s_trans[i].bits_per_word = 8;
			s_trans[i].speed_hz = freq;
			s_trans[i].tx_buf = send_buf->second_message + (NEURONSPI_MAX_TX * (i - 2));
			s_trans[i].rx_buf = recv_buf->second_message + (NEURONSPI_MAX_TX * (i - 2));
			s_trans[i].len = (remain > NEURONSPI_MAX_TX) ? NEURONSPI_MAX_TX : remain;
			remain -= NEURONSPI_MAX_TX;
		}
        // corrent last chunk 
        s_trans[trans_count-1].delay_usecs = NEURONSPI_LAST_TRANSFER_DELAY;
        if ((send_header & (UNIPISPI_OP_MODE_HAVE_CRC_SPACE | UNIPISPI_OP_MODE_DO_CRC)) == (UNIPISPI_OP_MODE_DO_CRC)) {
            // crc2 is placed into own chunk
            crc_recv_buf[0] = crc2;
			s_trans[trans_count-1].tx_buf = crc_send_buf;
			s_trans[trans_count-1].rx_buf = crc_recv_buf;
			s_trans[trans_count-1].len = 2;
        } else {
            // crc is in part of data chunk
            // len is size of second message WITHOUT crc
            len -= 2;
        }
	}

    // Call SPI transaction
	spi_sync_transfer(spi_dev, s_trans, trans_count);
	kfree(s_trans);

    if (send_header & UNIPISPI_OP_MODE_SEND_HEADER) {
        unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI Master Read (op1) %8ph\n", recv_buf->first_message);
    }

    // skip next processing if reserved operation is running
    if (send_header & UNIPISPI_OP_MODE_SEND_HEADER) {
		recv_crc1 = neuronspi_spi_crc(recv_buf->first_message, 4, 0);
		packet_crc = *((u16*)(recv_buf->first_message+4));

		if (recv_crc1 == packet_crc) {
            unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI CRC1 Correct (=%04x)", packet_crc);
            opcode = recv_buf->first_message[0];
            if (d_data != NULL  && (opcode >= 0x41)&&(opcode <= 0x44)) {
                // Signal the UART to issue character reads
                portindex = (opcode==0x43) ? 0 : (opcode - 0x41);
                if (d_data->uart_count && (portindex < d_data->uart_count)) {
				    unipi_spi_trace(KERN_INFO "UNIPISPI: Reading UART data for device %d, opcode=%02x\n", d_data->neuron_index, opcode);
                    port = neuronspi_uart_data_global->p + d_data->uart_pindex + portindex;
                    if (opcode != 0x43) {
                        // read one incomming character from UART
                        neuronspi_rx_queue_add(port, recv_buf->first_message[3]);
                        //neuronspi_uart_handle_rx(port, 1, 1);
                    }
                    // read queue length
                    port->rx_remain = recv_buf->first_message[2];
                    //if (!d_data->uart_read) {
                    //    d_data->uart_read = recv_buf->first_message[2];
                    unipi_spi_trace(KERN_INFO "UNIPISPI: UART Buffer:%d, ttyNS%d(%d:%d)\n", port->rx_remain, port->port.line, port->dev_index, port->dev_port);
                    kthread_queue_work(&neuronspi_uart_data_global->kworker, &port->rx_work);
                    
                }
			}
		} else {
            recv_buf->first_message[0] = 0;
			unipi_spi_trace(KERN_INFO "UNIPISPI: SPI CRC1 Not Correct (Received: %04x Calculated: %04x)\n", packet_crc, recv_crc1);
		}
    }
    if (len > 0) {
        // Check second message crc
		recv_crc2 = neuronspi_spi_crc(recv_buf->second_message, len, recv_crc1);
        if ((send_header & (UNIPISPI_OP_MODE_HAVE_CRC_SPACE | UNIPISPI_OP_MODE_DO_CRC)) == (UNIPISPI_OP_MODE_DO_CRC)) {
            packet_crc = crc_recv_buf[0];
        } else {
            packet_crc = recv_buf->second_message[len] | (recv_buf->second_message[len+1] << 8);
        }
		unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI Master Read - %d:\n\t%64ph\n\t%64ph\n\t%64ph\n\t%64ph\n", len, 
                                        recv_buf->second_message, recv_buf->second_message + 64,
                                        recv_buf->second_message+128, recv_buf->second_message+192);

		if (recv_crc2 != packet_crc) {
            unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI CRC2 Not Correct: %04x COMPUTED: %04x\n", packet_crc, recv_crc2);
            if (send_header & UNIPISPI_OP_MODE_SEND_HEADER) 
                recv_buf->second_message[0] = 0;
            ret_code = 1;

		} else if (recv_buf->second_message[0] == 0x65) {
            
            // this op can be invoked only from kernel_work rx_proc
            portindex = 0; // second_message[2]; Overit ve firmware
            if (d_data->uart_count && (portindex < d_data->uart_count)) {
                port = neuronspi_uart_data_global->p + d_data->uart_pindex + portindex;
                port->rx_remain = recv_buf->second_message[3];
                neuronspi_rx_queue_swap(port);
            }
        }
    }
    mutex_unlock(&neuronspi_master_mutex);
    return ret_code;
}

/*****************************************************************
 *  Modbus like Interface via /dev/unipispi 
 */
int neuronspi_open (struct inode *inode_p, struct file *file_p)
{
	struct neuronspi_file_data *f_internal_data;
	if (neuronspi_s_dev == NULL || file_p == NULL || inode_p == NULL) {
		return -1;
	}
	neuronspi_cdrv.open_counter += 1;
	f_internal_data = kzalloc(sizeof(*f_internal_data), GFP_ATOMIC);
	f_internal_data->recv_buf.second_message = kzalloc(NEURONSPI_BUFFER_MAX, GFP_ATOMIC);
	f_internal_data->send_buf.second_message = kzalloc(NEURONSPI_BUFFER_MAX, GFP_ATOMIC);
	f_internal_data->spi_device = neuronspi_s_dev;
	mutex_init(&f_internal_data->lock);
	file_p->private_data = f_internal_data;
	return 0;
}

int neuronspi_release (struct inode *inode_p, struct file *file_p)
{
	struct neuronspi_file_data *f_internal_data;
	if (file_p == NULL) {
		return -1;
	}
	f_internal_data = (struct neuronspi_file_data*)file_p->private_data;
	f_internal_data->spi_device = NULL;
	kfree(f_internal_data->recv_buf.second_message);
	f_internal_data->recv_buf.second_message = NULL;
	kfree(f_internal_data->send_buf.second_message);
	f_internal_data->send_buf.second_message = NULL;
	kfree(f_internal_data);
	file_p->private_data = NULL;
	neuronspi_cdrv.open_counter -= 1;
	return 0;
}

ssize_t neuronspi_read (struct file *file_p, char *buffer, size_t len, loff_t *offset)
{

	s32 result = 0;
    loff_t dummy_offset = 0;
	struct neuronspi_file_data* private_data;
	struct spi_device* spi_driver_data;
	struct neuronspi_driver_data* driver_data;
	// Sanity checking
	if (neuronspi_cdrv.open_counter == 0) {
		neuronspi_cdrv.open_counter = 1;
	}
	if (buffer == NULL) return -7; // Invalid read buffer
    if (len == 0) return result; // Empty read
    if (len > 4095) return -EMSGSIZE;
    if (file_p == NULL) {
    	printk(KERN_DEBUG "UNIPISPI: CDEV File Pointer is NULL\n");
    	return -8;
    }
    if (file_p->private_data == NULL) {
    	printk(KERN_DEBUG "UNIPISPI: CDEV No Private Data\n");
    	return -1;	// No private data
    }
    private_data = (struct neuronspi_file_data*) file_p->private_data;
    if (private_data == NULL) return -4;
    spi_driver_data = private_data->spi_device[private_data->device_index];	// Get private (driver) data from FP
    if (spi_driver_data == NULL) return -2;

    driver_data = spi_get_drvdata(spi_driver_data);
    if (driver_data == NULL) return -2;
    if (driver_data->spi_driver == NULL) return -2;	// Invalid private data
    if ((driver_data->first_probe_reply[0] == 0) && !(driver_data->probe_always_succeeds) ) return -3; // Device not present

    mutex_lock(&(private_data->lock));
    if (private_data->recv_buf.second_message == NULL) {
    	mutex_unlock(&(private_data->lock));
    	return -10;
    }
    unipi_spi_trace(KERN_INFO "UNIPISPI: CDEV Read %d, DEV:%s%d DRV:%s%d msglen=%d offset=%d\n", len, (spi_driver_data->dev.of_node->name),
    		(spi_driver_data->chip_select), (driver_data->spi_driver->driver.name), (private_data->device_index),private_data->message_len, (int)*offset);
            
	if (private_data->has_first_message & UNIPISPI_OP_MODE_SEND_HEADER) {
   		result = simple_read_from_buffer(buffer, len, &dummy_offset, private_data->recv_buf.first_message, NEURONSPI_FIRST_MESSAGE_LENGTH);
		if (result >=0) {
            dummy_offset = 0;
			result = simple_read_from_buffer(buffer+result, len - result, &dummy_offset,
									private_data->recv_buf.second_message, private_data->message_len);
        }
	} else {
   		result = simple_read_from_buffer(buffer, len, &dummy_offset, private_data->recv_buf.second_message, private_data->message_len);
	}
	if (result >= 0) {
    	if (result + NEURONSPI_HEADER_LENGTH <= len) result = len;
	}
/*
    } else if (private_data->message_len == 0) {
    	mutex_unlock(&(private_data->lock));
    	return -9;
    }
*/
	mutex_unlock(&(private_data->lock));
	return result;
}


ssize_t neuronspi_write (struct file *file_p, const char *buffer, size_t len, loff_t *w_offset)
{
	u8 device_index = 0;
	u32 frequency;
	u8  reservation, send_header;
	s32 delay = 25;
    loff_t dummy_offset = 0;
    size_t datalen;
	//unsigned long flags;
	struct neuronspi_file_data* private_data;
	struct spi_device* spi_driver_data;
	struct neuronspi_driver_data* driver_data;
	// Sanity checking
	if (neuronspi_cdrv.open_counter == 0) {
		neuronspi_cdrv.open_counter = 1;
	}

	unipi_spi_trace(KERN_INFO "UNIPISPI: CDEV Write len:%d\n", len);
	if ((buffer == NULL) || (len == 0)) {
		return 0; // Void write
	}

    if ((len > 4095) || (len < NEURONSPI_HEADER_LENGTH + NEURONSPI_FIRST_MESSAGE_LENGTH)) return -EMSGSIZE;
    if (file_p == NULL) {
    	return -12;
    }
    if (file_p->private_data == NULL) {
    	printk(KERN_DEBUG "UNIPISPI: CDEV No Private Data\n");
    	return -1;	// No private data
    }
    // Read packet header and initialise private data (dependent on each other)
    device_index = buffer[0];
    send_header = buffer[3];
    frequency = (buffer[4] << 8 | buffer[5]) * 1000;
   	delay = buffer[6];
    reservation = buffer[7]; // Device reservation

    buffer += NEURONSPI_HEADER_LENGTH;
    datalen = len - NEURONSPI_HEADER_LENGTH;
    
    if (device_index > NEURONSPI_MAX_DEVS - 1) return -2;
    private_data = (struct neuronspi_file_data*) file_p->private_data;
    spi_driver_data = private_data->spi_device[device_index];	// Get private (driver) data from FP
    if (spi_driver_data == NULL) return -2;

    driver_data = spi_get_drvdata(spi_driver_data);
    if (driver_data == NULL) return -2;
    if (driver_data->spi_driver == NULL) return -2;	// Invalid private data
    if ((driver_data->first_probe_reply[0] == 0) && !(driver_data->probe_always_succeeds) ) 
        return -3; // Device not present

    if (delay == 0) { delay = 25; }	// Delay setting

    if (reservation == 255) { 
        // Unlock device
   		driver_data->reserved_device = 0;
   	
    } else if (driver_data->reserved_device == 0) {
        // Reserve the device
   		driver_data->reserved_device = reservation;

    } else if (reservation != driver_data->reserved_device) {
        // Another device reserved
   		return -4;				
   	}

    if (!frequency || (frequency > driver_data->ideal_frequency)) 
        frequency = driver_data->ideal_frequency;

    mutex_lock(&(private_data->lock));
	private_data->device_index = device_index;
    private_data->has_first_message = send_header;
	if (send_header & UNIPISPI_OP_MODE_SEND_HEADER) {
    	private_data->message_len = datalen - NEURONSPI_FIRST_MESSAGE_LENGTH;
		simple_write_to_buffer(private_data->send_buf.first_message, NEURONSPI_FIRST_MESSAGE_LENGTH, &dummy_offset, buffer, datalen);
        dummy_offset = 0;
		simple_write_to_buffer(private_data->send_buf.second_message, NEURONSPI_BUFFER_MAX, &dummy_offset, buffer+NEURONSPI_FIRST_MESSAGE_LENGTH, private_data->message_len);
        //memcpy(private_data->send_buf.first_message, buffer + NEURONSPI_HEADER_LENGTH, NEURONSPI_FIRST_MESSAGE_LENGTH);
		//memcpy(private_data->send_buf.second_message, buffer + NEURONSPI_HEADER_LENGTH + NEURONSPI_FIRST_MESSAGE_LENGTH, private_data->message_len);
	} else {
    	private_data->message_len = datalen;
		simple_write_to_buffer(private_data->send_buf.second_message, NEURONSPI_BUFFER_MAX, &dummy_offset, buffer, datalen);
		//memcpy(private_data->send_buf.second_message, buffer + NEURONSPI_HEADER_LENGTH, private_data->message_len);
	}
    // clear receive buffer content
#if NEURONSPI_DETAILED_DEBUG > 1
    memset(private_data->recv_buf.first_message, 0, sizeof(private_data->recv_buf.first_message));
    memset(private_data->recv_buf.second_message, 0, NEURONSPI_BUFFER_MAX );
#else
    private_data->recv_buf.first_message[0] = 0;
    private_data->recv_buf.second_message[0] = 0;
#endif

    if (private_data->message_len) {
        if (private_data->send_buf.second_message[0] == 0x65) {
            // op read string is not allowed here
            mutex_unlock(&private_data->lock);
            return len;
        }
    }
    neuronspi_spi_send_op(spi_driver_data, &private_data->send_buf, &private_data->recv_buf, private_data->message_len,
								frequency, delay, send_header, reservation);
    mutex_unlock(&private_data->lock);
    return len;
}

/********************************************************
 *  Interface used by kernel - UnipiSpi operations 
 */

s32 neuronspi_spi_uart_write(struct spi_device *spi, u8 *send_buf, int length, u8 uart_index)
{
	u8 *message_buf;
	u8 *recv_buf;
	s32 transmit_len;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);
	struct neuronspi_op_buffer send_op;
	struct neuronspi_op_buffer recv_op;
	s32 frequency = NEURONSPI_DEFAULT_FREQ;
	if (d_data) {
		frequency = d_data->ideal_frequency;
	}

	unipi_spi_trace(KERN_INFO "UNIPISPI: SPI uart write, dev:%d, len:%d\n", uart_index, length);
	if ((length == 0) || (length > 256)) {
		return -1;
	}
	if (d_data->reserved_device) {
		return 0;
	}

	if (length == 1) {
		transmit_len = 0;
		send_op.first_message[0] = 0x41;
		send_op.first_message[1] = send_buf[0];
		send_op.first_message[2] = 0;
		send_op.first_message[3] = uart_index;
		//neuronspi___spi_send_message_crc(spi, &send_op, &recv_op, transmit_len, frequency, 65);
		neuronspi_spi_send_op(spi, &send_op, &recv_op, transmit_len, frequency, 65, UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC|UNIPISPI_OP_MODE_HAVE_CRC_SPACE, 0);
	} else {
		transmit_len = length & 1 ? length+1 : length;  // transmit_length must be even
		send_op.first_message[0] = 0x64;                //NEURONSPI_SPI_UART_LONG_MESSAGE[0];
		send_op.first_message[1] = length == 256 ? 0 : length;           // special case length==256
		send_op.first_message[2] = 0;
		send_op.first_message[3] = uart_index;
		message_buf = kzalloc(transmit_len+16, GFP_ATOMIC);
		memcpy(message_buf, send_buf, length);
		recv_buf = kzalloc(transmit_len+16, GFP_ATOMIC);
		memset(recv_buf+transmit_len, 0xff, 16);
        
        recv_op.second_message = recv_buf;
        send_op.second_message = message_buf;
		neuronspi_spi_send_op(spi, &send_op, &recv_op, transmit_len, frequency, 65, UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC|UNIPISPI_OP_MODE_HAVE_CRC_SPACE, 0);
		//neuronspi___spi_send_message_crc(spi, &send_op, &recv_op, transmit_len, frequency, 65);

		kfree(message_buf);
		kfree(recv_buf);
	}
	return 0;
}

void neuronspi_spi_uart_read(struct spi_device* spi, u8 *recv_buf, s32 len, u8 uart_index)
{
	s32 transmit_len;
	struct neuronspi_op_buffer send_op;
	struct neuronspi_op_buffer recv_op;
    u8 *send_buf;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);
	s32 frequency = NEURONSPI_DEFAULT_FREQ;
	if (d_data) {
		frequency = d_data->ideal_frequency;
	}

	unipi_spi_trace(KERN_INFO "UNIPISPI: SPI uart read, dev:%d, len:%d\n", uart_index, len);
	if (len < 246) {
		len = (len+4);
	} else {
		len = 250;
	}
	transmit_len = (len & 1 ? len+1 : len) + 4;  // transmit_length must be even

	send_op.first_message[1] = transmit_len;     //
	send_op.first_message[2] = 0;
	send_op.first_message[3] = uart_index;

	send_buf = kzalloc(transmit_len, GFP_ATOMIC);

#if 1
	send_op.first_message[0] = 0x65;
#else
	send_op.first_message[0] =  (NEURON_FIRMWARE_VERSION(d_data) < 0x518) ? 0x65 : 0x68;
#endif
	send_buf[0] = 0x65;

    send_op.second_message = send_buf;
    recv_op.second_message = recv_buf;

	unipi_spi_trace(KERN_INFO "UNIPISPI: UART Device Read len:%d\n", transmit_len);

	if (!d_data->reserved_device) {
		neuronspi_spi_send_op(spi, &send_op, &recv_op, transmit_len, frequency, 65, 
                                UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC|UNIPISPI_OP_MODE_HAVE_CRC_SPACE, 0);
	}
    kfree(send_buf);
}

int unipispi_modbus_read_register(struct spi_device* spi_dev, u16 reg, u16* value)
{
    struct neuronspi_op_buffer send_buf;
    struct neuronspi_op_buffer recv_buf;
    u8 send_data[4+2+2]; // cmd: 4B, value 2B, crc 2B
    u8 recv_data[4+2+2];    
    int ret_code;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	s32 frequency = NEURONSPI_DEFAULT_FREQ;
	if (d_data) {
		frequency = d_data->ideal_frequency;
	}
    send_buf.second_message = send_data;
    recv_buf.second_message = recv_data;

    send_buf.first_message[0] = 0x03;  // OP_READ_REGISTERS
    send_buf.first_message[1] = 4+2;
    *((u16*)(send_buf.first_message + 2)) = reg;
	memcpy(send_data, send_buf.first_message, 4);
	send_data[1] = 1;
    
	ret_code = neuronspi_spi_send_op(spi_dev, &send_buf, &recv_buf, 4+2, frequency, 35,
                                    UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC|UNIPISPI_OP_MODE_HAVE_CRC_SPACE, 0);
	if (ret_code == 0) {
        if ((recv_data[0] == 0x03) && (recv_data[1]==1)) {
            *value = *((u16*)(recv_data + 4));
        } else {
            ret_code = 2;
        }
    }
    return ret_code;
}

int unipispi_modbus_read_u32(struct spi_device* spi_dev, u16 reg, u32* value)
{
    struct neuronspi_op_buffer send_buf;
    struct neuronspi_op_buffer recv_buf;
    u8 send_data[4+4+2]; // cmd: 4B, value 4B, crc 2B
    u8 recv_data[4+4+2];    
    int ret_code;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	s32 frequency = NEURONSPI_DEFAULT_FREQ;
	if (d_data) {
		frequency = d_data->ideal_frequency;
	}
    send_buf.second_message = send_data;
    recv_buf.second_message = recv_data;

    send_buf.first_message[0] = 0x03;  // OP_READ_REGISTERS
    send_buf.first_message[1] = 4+4;
    *((u16*)(send_buf.first_message + 2)) = reg;
	memcpy(send_data, send_buf.first_message, 4);
	send_data[1] = 2;
    
	ret_code = neuronspi_spi_send_op(spi_dev, &send_buf, &recv_buf, 4+4, frequency, 35,
                                    UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC|UNIPISPI_OP_MODE_HAVE_CRC_SPACE, 0);
	if (ret_code == 0) {
        if ((recv_data[0] == 0x03) && (recv_data[1]==2)) {
            *value = *((u32*)(recv_data + 4));
        } else {
            ret_code = 2;
        }
    }
    return ret_code;
}

int unipispi_modbus_read_many(struct spi_device* spi_dev, u16 reg, u16* value, int register_count)
{
	printk(KERN_ERR "UNIPISPI: modbus read many(%d) UNIMPLENETED\n", register_count);
    //ToDo: 
    return 0;
}


int unipispi_modbus_write_register(struct spi_device* spi_dev, u16 reg, u16 value)
{
    struct neuronspi_op_buffer send_buf;
    struct neuronspi_op_buffer recv_buf;
    u8 send_data[4+2+2]; // cmd: 4B, value 2B, crc 2B
    u8 recv_data[4+2+2];    
    int ret_code;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	s32 frequency = NEURONSPI_DEFAULT_FREQ;
	if (d_data) {
		frequency = d_data->ideal_frequency;
	}
    send_buf.second_message = send_data;
    recv_buf.second_message = recv_data;

    send_buf.first_message[0] = 0x06;   // OP_WRITE_REGISTERS
    send_buf.first_message[1] = 4+2;
    *((u16*)(send_buf.first_message + 2)) = reg;
	memcpy(send_data, send_buf.first_message, 4);
	send_data[1] = 1;
    *((u16*)(send_data + 4)) = value;
    
	ret_code = neuronspi_spi_send_op(spi_dev, &send_buf, &recv_buf, 4+2, frequency, 35,
                                    UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC|UNIPISPI_OP_MODE_HAVE_CRC_SPACE, 0);
	if (ret_code == 0) {
        if ((recv_data[0] != 0x06) || (recv_data[1]!=1)) {
            ret_code = 2;
        }
    }
    return ret_code;
}

int unipispi_modbus_write_u32(struct spi_device* spi_dev, u16 reg, u32 value)
{
    struct neuronspi_op_buffer send_buf;
    struct neuronspi_op_buffer recv_buf;
    u8 send_data[4+4+2]; // cmd: 4B, value 4B, crc 2B
    u8 recv_data[4+4+2];    
    int ret_code;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	s32 frequency = NEURONSPI_DEFAULT_FREQ;
	if (d_data) {
		frequency = d_data->ideal_frequency;
	}
    send_buf.second_message = send_data;
    recv_buf.second_message = recv_data;

    send_buf.first_message[0] = 0x06;    // OP_WRITE_REGISTERS
    send_buf.first_message[1] = 4+4;
    *((u16*)(send_buf.first_message + 2)) = reg;
	memcpy(send_data, send_buf.first_message, 4);
	send_data[1] = 2;
    *((u32*)(send_data + 4)) = value;
    
	ret_code = neuronspi_spi_send_op(spi_dev, &send_buf, &recv_buf, 4+4, frequency, 35,
                                    UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC|UNIPISPI_OP_MODE_HAVE_CRC_SPACE, 0);
	if (ret_code == 0) {
        if ((recv_data[0] != 0x06) || (recv_data[1]!=2)) {
            ret_code = 2;
        }
    }
    return ret_code;
}

int unipispi_modbus_write_many(struct spi_device* spi_dev, u16 reg, u16* value, int register_count)
{
	printk(KERN_ERR "UNIPISPI: modbus write many(%d) UNIMPLENETED\n", register_count);
    //ToDo:
    return 0;
}

int unipispi_modbus_write_coil(struct spi_device* spi_dev, u16 coil, int value)
{
    struct neuronspi_op_buffer send_buf;
    struct neuronspi_op_buffer recv_buf;
    int ret_code;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	s32 frequency = NEURONSPI_DEFAULT_FREQ;
	if (d_data) {
		frequency = d_data->ideal_frequency;
	}

    send_buf.first_message[0] = 0x05;    // OP_WRITE_SINGLE_COIL
    send_buf.first_message[1] = value ? 1 : 0;
    *((u16*)(send_buf.first_message + 2)) = coil;
    
	ret_code = neuronspi_spi_send_op(spi_dev, &send_buf, &recv_buf, 0, frequency, 25,
                                    UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC|UNIPISPI_OP_MODE_HAVE_CRC_SPACE, 0);
    return ret_code;
}


void neuronspi_spi_set_irqs(struct spi_device* spi_dev, u16 to)
{
    unipispi_modbus_write_register(spi_dev, 1007, to);
	unipi_spi_trace(KERN_INFO "UNIPISPI: SPI IRQ Set, Dev-CS:%d, to:%d\n", spi_dev->chip_select, to);
}



/*
int neuronspi_spi_send_message(struct spi_device* spi_dev, u8 *send_buf, u8 *recv_buf, s32 len, s32 freq, s32 delay, s32 send_header, u8 lock_val)
{
	s32 i = 0;
	int ret_code = 0;
	u16 recv_crc1 = 0;
	u16 recv_crc2 = 0;
	u16 packet_crc = 0;
	s32 trans_count = (len<=0) ? 2 : ((len-1) / NEURONSPI_MAX_TX) + 3;	// number of transmissions
	//struct spi_message *s_msg;
	struct neuronspi_driver_data *d_data;
    struct spi_transfer* s_trans;
	mutex_lock(&neuronspi_master_mutex);
	//if ((len % NEURONSPI_MAX_TX) == 0) trans_count--;
    d_data = spi_get_drvdata(spi_dev);
	if (d_data != NULL && d_data->reserved_device && lock_val != d_data->reserved_device) {
		memset(&recv_buf, 0, len);
	} else {
		s_trans = kzalloc(sizeof(struct spi_transfer) * trans_count, GFP_ATOMIC);
		unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI Master Write, len:%d,\n %100ph\n", len, send_buf);

		if (!send_header) {
			trans_count -= 1;	// one less transmission as the header is omitted
		}
		//s_msg = kmalloc(sizeof(struct spi_message), GFP_ATOMIC);
		//spi_message_init(s_msg);
		for (i = 0; i < trans_count; i++) {
			memset(&(s_trans[i]), 0, sizeof(s_trans[i]));
			s_trans[i].delay_usecs = 0;
			s_trans[i].bits_per_word = 8;
			s_trans[i].speed_hz = freq;
			if (i == 0) {
				s_trans[i].delay_usecs = NEURONSPI_EDGE_DELAY;
			} else if (i == 1) {
				s_trans[i].tx_buf = send_buf;
				s_trans[i].rx_buf = recv_buf;
				if (send_header) {
					s_trans[i].delay_usecs = delay;
					s_trans[i].len = NEURONSPI_FIRST_MESSAGE_LENGTH;
				} else {
					// If len is more than NEURONSPI_MAX_TX * i, then chunk len is NEURONSPI_MAX_TX, otherwise it's the remainder
					s_trans[i].len = (len - (NEURONSPI_MAX_TX * i)) > 0 ? NEURONSPI_MAX_TX : len;
				}
			} else if (i == trans_count - 1) {
				if (send_header) {
					s_trans[i].tx_buf = &(send_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + (NEURONSPI_MAX_TX * (i - 2))]);
					s_trans[i].rx_buf = &(recv_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + (NEURONSPI_MAX_TX * (i - 2))]);
					s_trans[i].len = (len - (NEURONSPI_MAX_TX * (i - 2)));
					//s_trans[i].len = ((NEURONSPI_MAX_TX * (i - 1)) - len) < 0 ? NEURONSPI_MAX_TX : (len - (NEURONSPI_MAX_TX * (i - 2)));
				} else {
					s_trans[i].tx_buf = &(send_buf[NEURONSPI_MAX_TX * (i - 1)]);
					s_trans[i].rx_buf = &(recv_buf[NEURONSPI_MAX_TX * (i - 1)]);
					s_trans[i].len = ((NEURONSPI_MAX_TX * i) - len) < 0 ? NEURONSPI_MAX_TX : (len - (NEURONSPI_MAX_TX * (i - 1)));
				}
				s_trans[i].delay_usecs = NEURONSPI_LAST_TRANSFER_DELAY;
				// If len is more than NEURONSPI_MAX_TX * i (+ optionally header), then chunk len is NEURONSPI_MAX_TX (+ optionally header),
				// otherwise it's the remainder
			} else {
				if (send_header) {
					s_trans[i].tx_buf = &(send_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + (NEURONSPI_MAX_TX * (i - 2))]);
					s_trans[i].rx_buf = &(recv_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + (NEURONSPI_MAX_TX * (i - 2))]);
					s_trans[i].len = NEURONSPI_MAX_TX;
					//s_trans[i].len = ((NEURONSPI_MAX_TX * (i - 1)) - len) < 0 ? NEURONSPI_MAX_TX : (len - (NEURONSPI_MAX_TX * (i - 2)));
				} else {
					s_trans[i].tx_buf = &(send_buf[NEURONSPI_MAX_TX * (i - 1)]);
					s_trans[i].rx_buf = &(recv_buf[NEURONSPI_MAX_TX * (i - 1)]);
					s_trans[i].len = ((NEURONSPI_MAX_TX * i) - len) < 0 ? NEURONSPI_MAX_TX : (len - (NEURONSPI_MAX_TX * (i - 1)));
				}
				// If len is more than NEURONSPI_MAX_TX * i (+ optionally header), then chunk len is NEURONSPI_MAX_TX (+ optionally header),
				// otherwise it's the remainder
			}
			//spi_message_add_tail(&(s_trans[i]), s_msg);
		}
		//spi_sync(spi_dev, s_msg);
		spi_sync_transfer(spi_dev, s_trans, trans_count);
		//for (i = 0; i < trans_count; i++) {
		//	spi_transfer_del(&(s_trans[i]));
		//}
	    kfree(s_trans);
	    //kfree(s_msg);
		unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI Master Read - %d:\n\t%100ph\n\t%100ph\n\t%100ph\n\t%100ph\n", len,recv_buf, &recv_buf[64],
				&recv_buf[128], &recv_buf[192]);

	}
    if (d_data == NULL || (d_data != NULL && !d_data->reserved_device)) {
		recv_crc1 = neuronspi_spi_crc(recv_buf, 4, 0);
		memcpy(&packet_crc, &recv_buf[4], 2);
		unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI CRC1: %x\t COMPUTED CRC1:%x\n", packet_crc, recv_crc1);

		if (recv_crc1 == packet_crc) {
		// Signal the UART to issue character reads
            unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI CRC1 Correct");

			if (d_data && ((recv_buf[0] & 0xfd) == 0x41)) {
				unipi_spi_trace(KERN_INFO "UNIPISPI: Reading UART data for device %d\n", d_data->neuron_index);

			    if (recv_buf[0] == 0x41) {
					d_data->uart_buf[0] = recv_buf[3];
					for (i = 0; i < d_data->uart_data->p_count; i++) {
						if (d_data->uart_data->p[i].dev_index == d_data->neuron_index) {
							neuronspi_uart_handle_rx(&d_data->uart_data->p[i], 1, 1);
						}
					}
				}
				if (!(d_data->uart_read) && (d_data->uart_count)) {
					d_data->uart_read = recv_buf[2];
					for (i = 0; i < d_data->uart_data->p_count; i++) {
                        unipi_spi_trace(KERN_INFO "UNIPISPI: UART Buffer:%d, UART Local Port Count:%d, UART Global Port Count:%d\n", d_data->uart_read,
							d_data->uart_count,  d_data->uart_data->p_count);

						if (d_data->uart_data->p[i].dev_index == d_data->neuron_index && !d_data->reserved_device) {
							kthread_queue_work(&d_data->uart_data->kworker, &d_data->uart_data->p[i].rx_work);
						}
					}
				}
			}
		} else {
			unipi_spi_trace(KERN_INFO "UNIPISPI: SPI CRC1 Not Correct");
		}
		recv_crc2 = neuronspi_spi_crc(&recv_buf[6], len - 8, recv_crc1);
		memcpy(&packet_crc, &recv_buf[len - 2], 2);
		unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI CRC2: %x\t COMPUTED CRC2:%x\n", packet_crc, recv_crc2);
		if (recv_crc2 != packet_crc) {
			unipi_spi_trace(KERN_INFO "UNIPISPI: SPI CRC2 Not Correct");

			recv_buf[0] = 0;
			ret_code = 1;
		}
    }
    mutex_unlock(&neuronspi_master_mutex);
    return ret_code;
}
*/


irqreturn_t neuronspi_spi_irq(s32 irq, void *dev_id)
{
	s32 i;
	struct spi_device *spi;
	struct neuronspi_driver_data *d_data;
	//struct neuronspi_uart_data *u_data;
	spi = (struct spi_device *)dev_id;
	d_data = spi_get_drvdata(spi);
	unipi_spi_trace(KERN_INFO "UNIPISPI: SPI IRQ\n");

	if (d_data->uart_count) {
		for (i = 0; i < neuronspi_uart_data_global->p_count; i++) {
			if (neuronspi_uart_data_global->p[i].dev_index == d_data->neuron_index) {
				kthread_queue_work(&neuronspi_uart_data_global->kworker, &neuronspi_uart_data_global->p[i].irq_work);
			}

		}
	}
	return IRQ_HANDLED;
}


s32 neuronspi_spi_probe(struct spi_device *spi)
{
	const struct neuronspi_devtype *devtype;
	struct neuronspi_driver_data *n_spi;
    struct neuronspi_op_buffer recv_op;
	s32 ret, i, no_irq = 0;
	u8 uart_count = 0;
	unsigned long flags;
    
	n_spi = kzalloc(sizeof *n_spi, GFP_ATOMIC);
	spin_lock_irqsave(neuronspi_probe_spinlock, flags);
	neuronspi_probe_count++;
	spin_unlock_irqrestore(neuronspi_probe_spinlock, flags);
	if (!n_spi)
		return -ENOMEM;
	unipi_spi_trace(KERN_INFO "UNIPISPI: Probe Started\n");
	if (spi == NULL) {
        kfree(n_spi);
		return -8;
	}

	/* Setup SPI bus */
	spi->bits_per_word	= 8;
	spi->mode		    = spi->mode ? spi->mode : SPI_MODE_0;
	spi->max_speed_hz	= spi->max_speed_hz ? spi->max_speed_hz : 12000000;
	ret = spi_setup(spi);
	if (ret) {
        kfree(n_spi);
		return ret;
    }

	n_spi->neuron_index = spi->chip_select - 1;
	n_spi->reserved_device = 0;

	if (neuron_plc_dev == NULL) {
		neuron_plc_dev = platform_device_alloc("unipi_plc", -1);
		neuron_plc_dev->dev.groups = neuron_plc_attr_groups;
		platform_device_add(neuron_plc_dev);
	}


	unipi_spi_trace(KERN_DEBUG "UNIPISPI: CS: %d, Chip Max Hz-%d %d\n", spi->chip_select, spi->master->max_speed_hz, spi->max_speed_hz);
	if (spi->dev.of_node) {
		const struct of_device_id *of_id =
			of_match_device(neuronspi_id_match, &spi->dev);
		if (!of_id) {
			printk(KERN_DEBUG "UNIPISPI: Probe %s does not match a device!\n", *(&spi->dev.of_node->full_name));
			return -ENODEV;
		}
		of_property_read_u32_array(spi->dev.of_node, "neuron-board-index", &(n_spi->neuron_index), 1);
		of_property_read_u32_array(spi->dev.of_node, "neuron-probe-always-succeeds", &(n_spi->probe_always_succeeds), 1);
		of_property_read_u32_array(spi->dev.of_node, "neuron-always-create-tty", &(n_spi->always_create_uart), 1);
		devtype = (struct neuronspi_devtype *)of_id->data;
		unipi_spi_trace(KERN_INFO "UNIPISPI: DEVICE TREE NODE FOUND %d\n", n_spi->neuron_index);

	} else {
		const struct spi_device_id *id_entry = spi_get_device_id(spi);
		devtype = (struct neuronspi_devtype *)id_entry->driver_data;
	}


	kthread_init_worker(&n_spi->primary_worker);

	n_spi->primary_worker_task = kthread_run(kthread_worker_fn, &n_spi->primary_worker, "neuronspi");
	if (IS_ERR(n_spi->primary_worker_task )) {
		ret = PTR_ERR(n_spi->primary_worker_task);
		return ret;
	}
	sched_setscheduler(n_spi->primary_worker_task, SCHED_FIFO, &neuronspi_sched_param);

	// We perform an initial probe of registers 1000-1004 to identify the device, using a premade message
	n_spi->first_probe_reply = kzalloc(UNIPISPI_PROBE_MESSAGE_LEN, GFP_ATOMIC);	// allocate space for initial probe
	n_spi->lower_board_id = n_spi->upper_board_id = n_spi->combination_id = 0xFF;
	n_spi->spi_driver = &neuronspi_spi_driver;

	neuronspi_spi_send_const_op(spi, &UNIPISPI_IDLE_MESSAGE, &recv_op, 0, NEURONSPI_DEFAULT_FREQ, 25);
	// Throw away the first message - the associated SPI Master is sometimes not properly initialised at this point

    recv_op.second_message = n_spi->first_probe_reply;
	i = 0;
	do {
        neuronspi_spi_send_const_op(spi, &UNIPISPI_PROBE_MESSAGE, &recv_op, UNIPISPI_PROBE_MESSAGE_LEN, NEURONSPI_DEFAULT_FREQ, 25);
		i++;
	} while (n_spi->first_probe_reply[0] == 0 && i < 5);

	if (n_spi->first_probe_reply[0] != 0) { 	// CRC error sets the first byte to 0
		uart_count = n_spi->first_probe_reply[14-6] & 0x0f;
		for (i = 0; i < NEURONSPI_BOARDTABLE_LEN; i++) {
			if (n_spi->first_probe_reply[19-6] == NEURONSPI_BOARDTABLE[i].lower_board_id) {
				if (n_spi->combination_id == 0xFF && NEURONSPI_BOARDTABLE[i].upper_board_id == 0) {
					n_spi->combination_id = NEURONSPI_BOARDTABLE[i].index;
				}
				if (n_spi->lower_board_id == 0xFF) {
					n_spi->lower_board_id = n_spi->first_probe_reply[17-6];
				}
				if (n_spi->first_probe_reply[17-6] == NEURONSPI_BOARDTABLE[i].index) {
					n_spi->combination_id = n_spi->first_probe_reply[17-6];
					n_spi->upper_board_id = NEURONSPI_BOARDTABLE[i].upper_board_id;
				}
			}
		}
	} else if (!n_spi->probe_always_succeeds) {
		ret = -ENODEV;
		kfree(n_spi);
		printk(KERN_INFO "UNIPISPI: Probe did not detect a valid UniPi device on CS %d\n", spi->chip_select);
		return ret;
	} else if (n_spi->always_create_uart) {
		uart_count = 1;
	}

	if (n_spi->lower_board_id != 0xFF && n_spi->combination_id != 0xFF) {
		n_spi->features = kzalloc(sizeof(struct neuronspi_board_features), GFP_ATOMIC);
		n_spi->features = &(NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->features);
	} else {
		n_spi->features = NULL;
	}

	n_spi->ideal_frequency = NEURONSPI_COMMON_FREQ;
	for (i = 0; i < NEURONSPI_SLOWER_MODELS_LEN; i++) {
		if (NEURONSPI_SLOWER_MODELS[i] == (n_spi->first_probe_reply[19-6] << 8 | n_spi->first_probe_reply[18-6])) {
			//n_spi->slower_model = 1;
			n_spi->ideal_frequency = NEURONSPI_SLOWER_FREQ;
		}
	}
	if (n_spi->lower_board_id != 0xFF && n_spi->combination_id != 0xFF) {
		printk(KERN_INFO "UNIPISPI: Probe detected UniPi Board %s (L:%x U:%x C:%x) Index: %d Fw: v%d.%d on CS %d, \
Uart count: %d - reg1000: %x, reg1001: %x, reg1002: %x, reg1003: %x, reg1004: %x\n",
				NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->combination_name,
				n_spi->lower_board_id, n_spi->upper_board_id, n_spi->combination_id,  n_spi->neuron_index,
				n_spi->first_probe_reply[11-6],  n_spi->first_probe_reply[10-6], spi->chip_select, uart_count,
				n_spi->first_probe_reply[11-6] << 8 | n_spi->first_probe_reply[10-6],
				n_spi->first_probe_reply[13-6] << 8 | n_spi->first_probe_reply[12-6], n_spi->first_probe_reply[15-6] << 8 | n_spi->first_probe_reply[14-6],
				n_spi->first_probe_reply[17-6] << 8 | n_spi->first_probe_reply[16-6], n_spi->first_probe_reply[19-6] << 8 | n_spi->first_probe_reply[18-6]);
	} else if (n_spi->lower_board_id != 0xFF) {
		printk(KERN_INFO "UNIPISPI: Probe detected UniPi Board L:%x C:??? Index: %d Fw: v%d.%d on CS %d, Uart count: %d - reg1000: %x, \
reg1001: %x, reg1002: %x, reg1003: %x, reg1004: %x\n",
				n_spi->lower_board_id, n_spi->neuron_index, n_spi->first_probe_reply[11-6],  n_spi->first_probe_reply[10-6],
				spi->chip_select, uart_count, n_spi->first_probe_reply[11-6] << 8 | n_spi->first_probe_reply[10-6],
				n_spi->first_probe_reply[13-6] << 8 | n_spi->first_probe_reply[12-6], n_spi->first_probe_reply[15-6] << 8 | n_spi->first_probe_reply[14-6],
				n_spi->first_probe_reply[17-6] << 8 | n_spi->first_probe_reply[16-6], n_spi->first_probe_reply[19-6] << 8 | n_spi->first_probe_reply[18-6]);
	} else {
		printk(KERN_INFO "UNIPISPI: Probe detected UniPi Board L:??? C:??? Index: %d Fw: v%d.%d on CS %d, Uart count: %d - reg1000: %x, \
reg1001: %x, reg1002: %x, reg1003: %x, reg1004: %x\n",
				n_spi->neuron_index, n_spi->first_probe_reply[11-6],  n_spi->first_probe_reply[10-6], spi->chip_select, uart_count,
				n_spi->first_probe_reply[11-6] << 8 | n_spi->first_probe_reply[10-6], n_spi->first_probe_reply[13-6] << 8 | n_spi->first_probe_reply[12-6],
				n_spi->first_probe_reply[15-6] << 8 | n_spi->first_probe_reply[14-6], n_spi->first_probe_reply[17-6] << 8 | n_spi->first_probe_reply[16-6],
				n_spi->first_probe_reply[19-6] << 8 | n_spi->first_probe_reply[18-6]);
	}
	if (n_spi->combination_id != 0xFF) {
		printk(KERN_INFO "UNIPISPI: UniPi device %s on CS %d uses SPI communication freq. %d Hz\n",
				NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->combination_name,
				spi->chip_select, n_spi->ideal_frequency);
	}
	n_spi->reg_map = regmap_init(&(spi->dev), &neuronspi_regmap_bus, spi, &neuronspi_regmap_config_default);
	spin_lock_init(&n_spi->sysfs_regmap_lock);
	if (n_spi->features) {
		n_spi->regstart_table = kzalloc(sizeof(struct neuronspi_board_regstart_table), 1);
		neuronspi_create_reg_starts(n_spi->regstart_table, NEURONSPI_BOARDTABLE[n_spi->combination_id].definition);
	} else {
		n_spi->regstart_table = NULL;
	}


	// Check for user-configurable LED devices
	if (n_spi->features && n_spi->features->led_count > 0) {
		printk(KERN_INFO "UNIPISPI: LED model %s with %d LEDs detected at CS: %d\n",
				NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->combination_name,
				n_spi->features->led_count, spi->chip_select);
		n_spi->led_driver = kzalloc(sizeof(struct neuronspi_led_driver) * n_spi->features->led_count, GFP_ATOMIC);
		for (i = 0; i < n_spi->features->led_count; i++) {
			kthread_init_work(&(n_spi->led_driver[i].led_work), neuronspi_led_proc);
		}
	}

	n_spi->char_driver = &neuronspi_cdrv;


	spin_lock_irqsave(neuronspi_probe_spinlock, flags);
	neuronspi_s_dev[n_spi->neuron_index] = spi;
	spi_set_drvdata(spi, n_spi);
    
	//spi_set_drvdata(neuronspi_s_dev[n_spi->neuron_index], n_spi);
	if (neuronspi_probe_count == NEURONSPI_MAX_DEVS) {
		neuronspi_model_id = neuronspi_find_model_id(neuronspi_probe_count);
	}
	spin_unlock_irqrestore(neuronspi_probe_spinlock, flags);
	if (neuronspi_model_id != -1) {
		printk(KERN_INFO "UNIPISPI: Detected UniPi board combination corresponding to %s\n", NEURONSPI_MODELTABLE[neuronspi_model_id].model_name);
	}

	unipi_spi_trace(KERN_DEBUG "UNIPISPI: SPI IRQ: %d", spi->irq);
	strcpy(n_spi->platform_name, "io_group0");
	n_spi->platform_name[8] = n_spi->neuron_index + '1';
	n_spi->board_device = platform_device_alloc(n_spi->platform_name, -1);
	n_spi->board_device->dev.parent = &(neuron_plc_dev->dev);
	if (n_spi->combination_id != 0xFF) {
		n_spi->board_device->dev.groups = neuron_board_attr_groups;
	}
	n_spi->board_device->dev.driver = &neuronspi_spi_driver.driver;
	platform_device_add(n_spi->board_device);
	platform_set_drvdata(n_spi->board_device, n_spi);

	if (!(neuronspi_cdrv.major_number)) { // Register character device if it doesn't exist
		ret = char_register_driver();
		if (ret) {
			printk(KERN_ERR "UNIPISPI: Failed to register the neuronspi character driver, ERR:%d\n", ret);
		}
	}

	if (n_spi->features) {
		if (n_spi->features->led_count) {
			for (i = 0; i < n_spi->features->led_count; i++) {
				strcpy(n_spi->led_driver[i].name, "unipi:green:uled-x1");
				if (i < 9) {
					n_spi->led_driver[i].name[18] = i + '1';
				} else {
					n_spi->led_driver[i].name[18] = i - 9 + 'a';
				}
				// Initialise the rest of the structure
				n_spi->led_driver[i].id = i;
				n_spi->led_driver[i].brightness = LED_OFF;
				n_spi->led_driver[i].spi = spi;
				spin_lock_init(&n_spi->led_driver[i].lock);
				n_spi->led_driver[i].ldev.name = n_spi->led_driver[i].name;
				n_spi->led_driver[i].ldev.brightness = n_spi->led_driver[i].brightness;
				n_spi->led_driver[i].ldev.brightness_set = neuronspi_led_set_brightness;
				led_classdev_register(&(n_spi->board_device->dev), &(n_spi->led_driver[i].ldev));
			}
		}
#ifdef CONFIG_GPIOLIB
		if (n_spi->features->di_count) {
			n_spi->di_driver = kzalloc(sizeof(struct neuronspi_di_driver*) * n_spi->features->di_count, GFP_ATOMIC);
			for (i = 0; i < n_spi->features->di_count; i++) {
				n_spi->di_driver[i] = kzalloc(sizeof(struct neuronspi_di_driver), GFP_ATOMIC);
				strcpy(n_spi->di_driver[i]->name, "di_0_00");
				n_spi->di_driver[i]->name[3] = n_spi->neuron_index + '1';
				n_spi->di_driver[i]->name[5] = ((i + 1) / 10) + '0';
				n_spi->di_driver[i]->name[6] = ((i + 1) % 10) + '0';
				n_spi->di_driver[i]->di_index = i;
				n_spi->di_driver[i]->spi = spi;
				n_spi->di_driver[i]->plat_dev = platform_device_alloc(n_spi->di_driver[i]->name, -1);
				n_spi->di_driver[i]->plat_dev->dev.parent = &(n_spi->board_device->dev);
				n_spi->di_driver[i]->plat_dev->dev.groups = neuron_gpio_di_attr_groups;
				n_spi->di_driver[i]->plat_dev->dev.driver = &neuronspi_spi_driver.driver;
				platform_device_add(n_spi->di_driver[i]->plat_dev);
				platform_set_drvdata(n_spi->di_driver[i]->plat_dev, n_spi->di_driver[i]);
				n_spi->di_driver[i]->gpio_c.owner = THIS_MODULE;
				n_spi->di_driver[i]->gpio_c.parent = &(n_spi->di_driver[i]->plat_dev->dev);
				n_spi->di_driver[i]->gpio_c.label = "neuron_di";
				n_spi->di_driver[i]->gpio_c.can_sleep = 1;
				n_spi->di_driver[i]->gpio_c.ngpio = 1;
				n_spi->di_driver[i]->gpio_c.base = -1;
				n_spi->di_driver[i]->gpio_c.direction_input = neuronspi_gpio_di_direction_input;
				n_spi->di_driver[i]->gpio_c.get = neuronspi_gpio_di_get;
				gpiochip_add_data(&n_spi->di_driver[i]->gpio_c, n_spi->di_driver[i]);
			}
		}

		if (n_spi->features->do_count) {
			n_spi->do_driver = kzalloc(sizeof(struct neuronspi_do_driver*) * n_spi->features->do_count, GFP_ATOMIC);
			for (i = 0; i < n_spi->features->do_count; i++) {
				n_spi->do_driver[i] = kzalloc(sizeof(struct neuronspi_do_driver), GFP_ATOMIC);
				strcpy(n_spi->do_driver[i]->name, "do_0_00");
				n_spi->do_driver[i]->name[3] = n_spi->neuron_index + '1';
				n_spi->do_driver[i]->name[5] = ((i + 1) / 10) + '0';
				n_spi->do_driver[i]->name[6] = ((i + 1) % 10) + '0';
				n_spi->do_driver[i]->do_index = i;
				n_spi->do_driver[i]->spi = spi;
				n_spi->do_driver[i]->plat_dev = platform_device_alloc(n_spi->do_driver[i]->name, -1);
				n_spi->do_driver[i]->plat_dev->dev.parent = &(n_spi->board_device->dev);
				n_spi->do_driver[i]->plat_dev->dev.groups = neuron_gpio_do_attr_groups;
				n_spi->do_driver[i]->plat_dev->dev.driver = &neuronspi_spi_driver.driver;
				platform_device_add(n_spi->do_driver[i]->plat_dev);
				platform_set_drvdata(n_spi->do_driver[i]->plat_dev, n_spi->do_driver[i]);
				n_spi->do_driver[i]->gpio_c.owner = THIS_MODULE;
				n_spi->do_driver[i]->gpio_c.parent = &(n_spi->do_driver[i]->plat_dev->dev);
				n_spi->do_driver[i]->gpio_c.label = "neuron_do";
				n_spi->do_driver[i]->gpio_c.can_sleep = 1;
				n_spi->do_driver[i]->gpio_c.ngpio = 1;
				n_spi->do_driver[i]->gpio_c.base = -1;
				n_spi->do_driver[i]->gpio_c.direction_output = neuronspi_gpio_do_direction_output;
				n_spi->do_driver[i]->gpio_c.set = neuronspi_gpio_do_set;
				gpiochip_add_data(&n_spi->do_driver[i]->gpio_c, n_spi->do_driver[i]);
			}
		}

		if (n_spi->features->ro_count) {
			n_spi->ro_driver = kzalloc(sizeof(struct neuronspi_ro_driver*) * n_spi->features->ro_count, GFP_ATOMIC);
			for (i = 0; i < n_spi->features->ro_count; i++) {
				n_spi->ro_driver[i] = kzalloc(sizeof(struct neuronspi_ro_driver), GFP_ATOMIC);
				strcpy(n_spi->ro_driver[i]->name, "ro_0_00");
				n_spi->ro_driver[i]->name[3] = n_spi->neuron_index + '1';
				n_spi->ro_driver[i]->name[5] = ((i + 1) / 10) + '0';
				n_spi->ro_driver[i]->name[6] = ((i + 1) % 10) + '0';
				n_spi->ro_driver[i]->ro_index = i;
				n_spi->ro_driver[i]->spi = spi;
				n_spi->ro_driver[i]->plat_dev = platform_device_alloc(n_spi->ro_driver[i]->name, -1);
				n_spi->ro_driver[i]->plat_dev->dev.parent = &(n_spi->board_device->dev);
				n_spi->ro_driver[i]->plat_dev->dev.groups = neuron_gpio_ro_attr_groups;
				n_spi->ro_driver[i]->plat_dev->dev.driver = &neuronspi_spi_driver.driver;
				platform_device_add(n_spi->ro_driver[i]->plat_dev);
				platform_set_drvdata(n_spi->ro_driver[i]->plat_dev, n_spi->ro_driver[i]);
				n_spi->ro_driver[i]->gpio_c.owner = THIS_MODULE;
				n_spi->ro_driver[i]->gpio_c.parent = &(n_spi->ro_driver[i]->plat_dev->dev);
				n_spi->ro_driver[i]->gpio_c.label = "neuron_ro";
				n_spi->ro_driver[i]->gpio_c.can_sleep = 1;
				n_spi->ro_driver[i]->gpio_c.ngpio = 1;
				n_spi->ro_driver[i]->gpio_c.base = -1;
				n_spi->ro_driver[i]->gpio_c.direction_output = neuronspi_gpio_ro_direction_output;
				n_spi->ro_driver[i]->gpio_c.set = neuronspi_gpio_ro_set;
				gpiochip_add_data(&n_spi->ro_driver[i]->gpio_c, n_spi->ro_driver[i]);
			}
		}
#endif
		if (n_spi->features->stm_ai_count) {
			n_spi->stm_ai_driver = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_analog_data));
			((struct neuronspi_analog_data*)iio_priv(n_spi->stm_ai_driver))->parent = spi;
			((struct neuronspi_analog_data*)iio_priv(n_spi->stm_ai_driver))->index = 0;
			n_spi->stm_ai_driver->modes = INDIO_DIRECT_MODE;
			n_spi->stm_ai_driver->currentmode = INDIO_DIRECT_MODE;
			n_spi->stm_ai_driver->name = "ai_type_a";
			n_spi->stm_ai_driver->dev.parent = &(n_spi->board_device->dev);
			dev_set_name(&n_spi->stm_ai_driver->dev, "ai_%d_1",	(int)n_spi->neuron_index + 1);
			n_spi->stm_ai_driver->num_channels = 2;
			n_spi->stm_ai_driver->channels = neuronspi_stm_ai_chan_spec;
			n_spi->stm_ai_driver->info = &neuronspi_stm_ai_info;
			iio_device_register(n_spi->stm_ai_driver);
		}
		if (n_spi->features->stm_ao_count) {
			n_spi->stm_ao_driver = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_analog_data));
			((struct neuronspi_analog_data*)iio_priv(n_spi->stm_ao_driver))->parent = spi;
			((struct neuronspi_analog_data*)iio_priv(n_spi->stm_ao_driver))->index = 0;
			n_spi->stm_ao_driver->modes = INDIO_DIRECT_MODE;
			n_spi->stm_ao_driver->currentmode = INDIO_DIRECT_MODE;
			n_spi->stm_ao_driver->name = "ao_type_a";
			n_spi->stm_ao_driver->dev.parent = &(n_spi->board_device->dev);
			dev_set_name(&n_spi->stm_ao_driver->dev, "ao_%d_1",	(int)n_spi->neuron_index + 1);
			n_spi->stm_ao_driver->num_channels = 3;
			n_spi->stm_ao_driver->channels = neuronspi_stm_ao_chan_spec;
			n_spi->stm_ao_driver->info = &neuronspi_stm_ao_info;
			iio_device_register(n_spi->stm_ao_driver);
		}
		if (n_spi->features->sec_ai_count) {
			n_spi->sec_ai_driver = kzalloc(sizeof(struct neuronspi_analog_data*) * n_spi->features->sec_ai_count, GFP_ATOMIC);
			for (i = 0; i < n_spi->features->sec_ai_count; i++) {
				n_spi->sec_ai_driver[i] = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_analog_data));
				((struct neuronspi_analog_data*)iio_priv(n_spi->sec_ai_driver[i]))->parent = spi;
				((struct neuronspi_analog_data*)iio_priv(n_spi->sec_ai_driver[i]))->index = i;
				n_spi->sec_ai_driver[i]->modes = INDIO_DIRECT_MODE;
				n_spi->sec_ai_driver[i]->currentmode = INDIO_DIRECT_MODE;
				n_spi->sec_ai_driver[i]->name = "ai_type_b";
				n_spi->sec_ai_driver[i]->dev.parent = &(n_spi->board_device->dev);
				dev_set_name(&n_spi->sec_ai_driver[i]->dev, "ai_%d_%d",	(int)n_spi->neuron_index + 1, (int)i + 1);
				n_spi->sec_ai_driver[i]->num_channels = 3;
				n_spi->sec_ai_driver[i]->channels = neuronspi_sec_ai_chan_spec;
				n_spi->sec_ai_driver[i]->info = &neuronspi_sec_ai_info;
				iio_device_register(n_spi->sec_ai_driver[i]);
			}
		}
		if (n_spi->features->sec_ao_count) {
			n_spi->sec_ao_driver = kzalloc(sizeof(struct neuronspi_analog_data*) * n_spi->features->sec_ao_count, GFP_ATOMIC);
			for (i = 0; i < n_spi->features->sec_ao_count; i++) {
				n_spi->sec_ao_driver[i] = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_analog_data));
				((struct neuronspi_analog_data*)iio_priv(n_spi->sec_ao_driver[i]))->parent = spi;
				((struct neuronspi_analog_data*)iio_priv(n_spi->sec_ao_driver[i]))->index = i;
				n_spi->sec_ao_driver[i]->modes = INDIO_DIRECT_MODE;
				n_spi->sec_ao_driver[i]->currentmode = INDIO_DIRECT_MODE;
				n_spi->sec_ao_driver[i]->name = "ao_type_b";
				n_spi->sec_ao_driver[i]->dev.parent = &(n_spi->board_device->dev);
				dev_set_name(&n_spi->sec_ao_driver[i]->dev, "ao_%d_%d",	(int)n_spi->neuron_index + 1, (int)i + 1);
				n_spi->sec_ao_driver[i]->num_channels = 1;
				n_spi->sec_ao_driver[i]->channels = neuronspi_sec_ao_chan_spec;
				n_spi->sec_ao_driver[i]->info = &neuronspi_sec_ao_info;
				iio_device_register(n_spi->sec_ao_driver[i]);
			}
		}
	}

    n_spi->uart_count_to_probe = uart_count;
	if (uart_count) {
        //n_spi->uart_buf = kzalloc(NEURONSPI_FIFO_SIZE, GFP_ATOMIC);
        if (neuronspi_uart_driver_global != NULL) {	
            // Normalne se registrace portu udela az po inicializaci unipispi driveru. (Na konci __init__)
            // Opravit proceduru probe !!! 
            unipi_spi_trace(KERN_DEBUG "UNIPISPI: UART registration\n");

            neuronspi_uart_probe(spi, n_spi);
            //n_spi->uart_count = uart_count;
        } else {
            unipi_spi_trace(KERN_DEBUG "UNIPISPI: Neuronspi uart driver not registered yet. Uart port add later.\n");
        }
	}

	neuronspi_spi_set_irqs(spi, 0x5);
	for (i = 0; i < NEURONSPI_NO_INTERRUPT_MODELS_LEN; i++) {
		if (NEURONSPI_NO_INTERRUPT_MODELS[i] == (n_spi->first_probe_reply[17-6] << 8 | n_spi->first_probe_reply[16-6])) {
			no_irq = 1;
		}
	}

	n_spi->poll_thread = NULL;
	if (!no_irq) {
		n_spi->no_irq = 0;
		ret = devm_request_irq(&(spi->dev), spi->irq, neuronspi_spi_irq, 0x81, dev_name(&(spi->dev)), spi);
		unipi_spi_trace(KERN_DEBUG "UNIPISPI: IRQ registration, ret:%d\n", ret);
	} else {
		n_spi->no_irq = 1;
		unipi_spi_trace(KERN_DEBUG "UNIPISPI: NO IRQ ON THIS MODEL !!\n");
	}

	return ret;
}

s32 neuronspi_spi_remove(struct spi_device *spi)
{
	int i;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	if (n_spi) {
		if (n_spi->led_driver) {
			for (i = 0; i < n_spi->features->led_count; i++) {
				led_classdev_unregister(&(n_spi->led_driver[i].ldev));
			}
			kthread_flush_worker(&n_spi->primary_worker);
			kfree(n_spi->led_driver);
			n_spi->led_driver = NULL;
		}
		printk(KERN_INFO "UNIPISPI: LED DRIVER UNREGISTERED\n");
		if (n_spi->di_driver) {
			for (i = 0; i < n_spi->features->di_count; i++) {
				gpiochip_remove(&n_spi->di_driver[i]->gpio_c);
				platform_set_drvdata(n_spi->di_driver[i]->plat_dev, 0);
				platform_device_unregister(n_spi->di_driver[i]->plat_dev);
				kfree(n_spi->di_driver[i]);
			}
			kfree(n_spi->di_driver);
		}
		if (n_spi->do_driver) {
			for (i = 0; i < n_spi->features->do_count; i++) {
				gpiochip_remove(&n_spi->do_driver[i]->gpio_c);
				platform_set_drvdata(n_spi->do_driver[i]->plat_dev, 0);
				platform_device_unregister(n_spi->do_driver[i]->plat_dev);
				kfree(n_spi->do_driver[i]);
			}
			kfree(n_spi->do_driver);
		}
		if (n_spi->ro_driver) {
			for (i = 0; i < n_spi->features->ro_count; i++) {
				gpiochip_remove(&n_spi->ro_driver[i]->gpio_c);
				platform_set_drvdata(n_spi->ro_driver[i]->plat_dev, 0);
				platform_device_unregister(n_spi->ro_driver[i]->plat_dev);
				kfree(n_spi->ro_driver[i]);
			}
			kfree(n_spi->ro_driver);
		}
		printk(KERN_INFO "UNIPISPI: GPIO DRIVER UNREGISTERED\n");
		if (n_spi->stm_ai_driver) {
			iio_device_unregister(n_spi->stm_ai_driver);
		}
		if (n_spi->stm_ao_driver) {
			iio_device_unregister(n_spi->stm_ao_driver);
		}
		if (n_spi->sec_ai_driver) {
			for (i = 0; i < n_spi->features->sec_ai_count; i++) {
				iio_device_unregister(n_spi->sec_ai_driver[i]);
			}
			kfree(n_spi->sec_ai_driver);
			n_spi->sec_ai_driver = NULL;
		}
		if (n_spi->sec_ao_driver) {
			for (i = 0; i < n_spi->features->sec_ao_count; i++) {
				iio_device_unregister(n_spi->sec_ao_driver[i]);
			}
			kfree(n_spi->sec_ao_driver);
			n_spi->sec_ao_driver = NULL;
		}
		printk(KERN_INFO "UNIPISPI: IIO DRIVER UNREGISTERED\n");
		//if (n_spi->uart_buf) {
		//	kfree(n_spi->uart_buf);
		//	n_spi->uart_buf = NULL;
		//}
		printk(KERN_INFO "UNIPISPI: SPI/UART DRIVER UNREGISTERED\n");
		if (n_spi->board_device) {
			platform_set_drvdata(n_spi->board_device, 0);
			platform_device_unregister(n_spi->board_device);
		}
		kfree(n_spi);
	}
	return 0;
}

s32 char_register_driver(void)
{
	s32 ret = 0;

	// Character device registration
	unipi_spi_trace(KERN_DEBUG "UNIPISPI: Initialising Character Device\n");

	neuronspi_cdrv.major_number = register_chrdev(0, NEURON_DEVICE_NAME, &file_ops);
	if (neuronspi_cdrv.major_number < 0){
	   printk(KERN_ALERT "NEURONSPI: failed to register a major number\n");
	   return neuronspi_cdrv.major_number;
	}
	unipi_spi_trace(KERN_DEBUG "UNIPISPI: registered correctly with major number %d\n", neuronspi_cdrv.major_number);

	// Character class registration
	neuronspi_cdrv.driver_class = class_create(THIS_MODULE, NEURON_DEVICE_CLASS);
	if (IS_ERR(neuronspi_cdrv.driver_class)) {
		unregister_chrdev(neuronspi_cdrv.major_number, NEURON_DEVICE_NAME);
		printk(KERN_ALERT "NEURONSPI: Failed to register device class\n");
		return PTR_ERR(neuronspi_cdrv.driver_class);
	}
	unipi_spi_trace(KERN_DEBUG "UNIPISPI: device class registered correctly\n");

	// Device driver registration
	neuronspi_cdrv.dev = device_create_with_groups(neuronspi_cdrv.driver_class, &(neuron_plc_dev->dev), MKDEV(neuronspi_cdrv.major_number, 0), NULL, neuron_plc_attr_groups, NEURON_DEVICE_NAME);
	if (IS_ERR(neuronspi_cdrv.dev)) {
		class_destroy(neuronspi_cdrv.driver_class);
	    	unregister_chrdev(neuronspi_cdrv.major_number, NEURON_DEVICE_NAME);
	    	printk(KERN_ALERT "NEURONSPI: Failed to create the device\n");
	    	return PTR_ERR(neuronspi_cdrv.dev);
	}
	unipi_spi_trace(KERN_DEBUG "UNIPISPI: device class created correctly\n");

	return ret;
}

s32 char_unregister_driver(void)
{
	device_destroy(neuronspi_cdrv.driver_class, MKDEV(neuronspi_cdrv.major_number, 0));     // Destroy the device
	class_unregister(neuronspi_cdrv.driver_class);                          				// Unregister the class
	class_destroy(neuronspi_cdrv.driver_class);                             				// Destroy the class
	unregister_chrdev(neuronspi_cdrv.major_number, NEURON_DEVICE_NAME);             			// Unregister the major number
	printk(KERN_INFO "UNIPISPI: Device unloaded successfully\n");
	return 0;
}

/*********************
 * Final definitions *
 *********************/

MODULE_ALIAS("spi:unipispi");

static s32 __init neuronspi_init(void)
{
	s32 ret = 0;

    //neuronspi_uart_driver_init();

	neuronspi_probe_spinlock = kzalloc(sizeof(struct spinlock), GFP_ATOMIC);
	spin_lock_init(neuronspi_probe_spinlock);
	mutex_init(&neuronspi_master_mutex);
	mutex_init(&unipi_inv_speed_mutex);
    // clear global neuron spi devices list
	memset(&neuronspi_s_dev, 0, sizeof(neuronspi_s_dev));
	ret = spi_register_driver(&neuronspi_spi_driver);
	if (ret < 0) {
		printk(KERN_ERR "UNIPISPI: Failed to init neuronspi spi --> %d\n", ret);
		return ret;
	} else {
#ifdef NEURONSPI_MAJOR_VERSIONSTRING
		printk(KERN_INFO "UNIPISPI: SPI Driver Registered, Major Version: %s\n", NEURONSPI_MAJOR_VERSIONSTRING);
#else
		printk(KERN_INFO "UNIPISPI: SPI Driver Registered\n");
#endif
	}
	neuronspi_invalidate_thread = kthread_create(neuronspi_regmap_invalidate, NULL, "unipispi_inv");
	if (neuronspi_invalidate_thread != NULL) {
		wake_up_process(neuronspi_invalidate_thread);
	}

    neuronspi_uart_driver_init();
    neuronspi_uart_probe_all();

	unipi_tty_init();
	return ret;
}

module_init(neuronspi_init);

static void __exit neuronspi_exit(void)
{
	unipi_spi_trace(KERN_INFO "UNIPISPI: Open Counter is %d\n", neuronspi_cdrv.open_counter);

	if (neuronspi_invalidate_thread) {
		kthread_stop(neuronspi_invalidate_thread);
	}
	char_unregister_driver();
    neuronspi_uart_driver_exit();
	spi_unregister_driver(&neuronspi_spi_driver);
	if (neuron_plc_dev) {
		platform_device_unregister(neuron_plc_dev);
	}
	printk(KERN_INFO "UNIPISPI: SPI Driver Unregistered\n");
}
module_exit(neuronspi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomas Knot <knot@faster.cz>");
MODULE_DESCRIPTION("UniPi PLC Driver");
