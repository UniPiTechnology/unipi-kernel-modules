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

struct mutex neuronspi_master_mutex;
struct mutex unipi_inv_speed_mutex;
int neuronspi_model_id = -1;
struct spi_device *neuronspi_s_dev[NEURONSPI_MAX_DEVS];  // global list of neuron spi devices
struct task_struct *neuronspi_invalidate_thread;

static u8 neuronspi_probe_count = 0;
static struct spinlock *neuronspi_probe_spinlock;
static struct sched_param neuronspi_sched_param = { .sched_priority = MAX_RT_PRIO / 2 };

struct neuronspi_char_driver neuronspi_cdrv =
{
	.dev = NULL
};

struct neuronspi_file_data
{
	//struct spi_device** spi_device;
	struct mutex 		lock;
    struct neuronspi_op_buffer send_buf;
    struct neuronspi_op_buffer recv_buf;
	u32			        message_len;
	u8					device_index;
	u8					has_first_message;
};

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
        if (d_data->poll_enabled) {
            // reschedule poll timer
            hrtimer_start_range_ns(&d_data->poll_timer, 2000000, 4000000, HRTIMER_MODE_REL);
        }
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
        if (d_data && d_data->poll_enabled) {
            // reschedule poll timer
            hrtimer_start_range_ns(&d_data->poll_timer, 2000000, 4000000, HRTIMER_MODE_REL);
        }
		recv_crc1 = neuronspi_spi_crc(recv_buf->first_message, 4, 0);
		packet_crc = *((u16*)(recv_buf->first_message+4));

		if (recv_crc1 == packet_crc) {
            unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI CRC1 Correct (=%04x)", packet_crc);
            opcode = recv_buf->first_message[0];
            if (d_data != NULL) {
                if ((opcode >= 0x41)&&(opcode <= 0x44)) {
                    // Signal the UART to issue character reads
                    portindex = (opcode==0x43) ? 0 : (opcode - 0x41);
                    if (d_data->uart_count && (portindex < d_data->uart_count)) {
                        unipi_spi_trace(KERN_INFO "UNIPISPI: Reading UART data for device %d, opcode=%02x\n", d_data->neuron_index, opcode);
                        port = neuronspi_uart_data_global->p + d_data->uart_pindex + portindex;
                        if (opcode != 0x43) {
                            // read one incomming character from UART
                            neuronspi_rx_queue_add(port, recv_buf->first_message[3]);
                        }
                        // read queue length
                        port->rx_remain = recv_buf->first_message[2];
                        unipi_spi_trace(KERN_INFO "UNIPISPI: UART Buffer:%d, ttyNS%d(%d:%d)\n", port->rx_remain, port->port.line, port->dev_index, port->dev_port);
                        kthread_queue_work(&neuronspi_uart_data_global->kworker, &port->rx_work);
                    }
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
	//f_internal_data->spi_device = neuronspi_s_dev;
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
	//f_internal_data->spi_device = NULL;
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
	struct spi_device* spi;
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
    spi = neuronspi_s_dev[private_data->device_index];
    if (spi == NULL) return -2;

    driver_data = spi_get_drvdata(spi);
    if (driver_data == NULL) return -2;
    //if (driver_data->spi_driver == NULL) return -2;	// Invalid private data
    //if ((driver_data->first_probe_reply[0] == 0) && !(driver_data->probe_always_succeeds) ) return -3; // couldnt happen

    mutex_lock(&(private_data->lock));
    if (private_data->recv_buf.second_message == NULL) {
    	mutex_unlock(&(private_data->lock));
    	return -10;
    }
    unipi_spi_trace(KERN_INFO "UNIPISPI: CDEV Read %d, nspi:%d msglen=%d offset=%d\n", len,
                    (private_data->device_index), private_data->message_len, (int)*offset);
            
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
	struct spi_device* spi;
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
    spi = neuronspi_s_dev[device_index];	
    if (spi == NULL) return -2;

    driver_data = spi_get_drvdata(spi);
    if (driver_data == NULL) return -2;
    //if (driver_data->spi_driver == NULL) return -2;	// Invalid private data
    //if ((driver_data->first_probe_reply[0] == 0) && !(driver_data->probe_always_succeeds) ) 
          // this couldnt happen, former condition is enough
    //    return -3; // Device not present

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
	} else {
    	private_data->message_len = datalen;
		simple_write_to_buffer(private_data->send_buf.second_message, NEURONSPI_BUFFER_MAX, &dummy_offset, buffer, datalen);
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
    neuronspi_spi_send_op(spi, &private_data->send_buf, &private_data->recv_buf, private_data->message_len,
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
	unipi_spi_trace(KERN_INFO "UNIPISPI: SPI IRQ Set, CS%d, to:0x%x\n", spi_dev->chip_select, to);
}


void neuronspi_irq_proc(struct kthread_work *ws)
{
	struct neuronspi_driver_data *n_spi = ((container_of((ws), struct neuronspi_driver_data, irq_work)));
	struct spi_device *spi = neuronspi_s_dev[n_spi->neuron_index];
    struct neuronspi_op_buffer recv_buf;

	neuronspi_spi_send_const_op(spi, &UNIPISPI_IDLE_MESSAGE, &recv_buf, 0, NEURONSPI_DEFAULT_FREQ, 25);
}


// callback of poll_timer. Schedule ->irq_work
static enum hrtimer_restart neuronspi_poll_timer_func(struct hrtimer *timer)
{
    struct neuronspi_driver_data* n_spi = ((container_of((timer), struct neuronspi_driver_data, poll_timer)));

	unipi_spi_trace_1(KERN_INFO "UNIPISPI: nspi%d POLL IRQ\n", n_spi->neuron_index);
	kthread_queue_work(&n_spi->primary_worker, &n_spi->irq_work);
	return HRTIMER_NORESTART;
}

/*
s32 neuronspi_uart_poll(void *data)
{
	struct neuronspi_driver_data *d_data = (struct neuronspi_driver_data*) data;
	//struct neuronspi_uart_data *u_data;
	s32 i;
	while (!kthread_should_stop()) {
		usleep_range(2000,8000);
		if (d_data->uart_count) {
			//u_data = d_data->uart_data;
			for (i = 0; i < neuronspi_uart_data_global->p_count; i++) {
				if (neuronspi_uart_data_global->p[i].dev_index == d_data->neuron_index) {
					kthread_queue_work(&neuronspi_uart_data_global->kworker, &neuronspi_uart_data_global->p[i].irq_work);
				}
			}
		}
	}
	return 0;
}
*/

irqreturn_t neuronspi_spi_irq(s32 irq, void *dev_id)
{
	// real irq handler - schedule idle op to read irq status (irq_work = neuronspi_irq_proc)
    struct spi_device *spi = (struct spi_device *)dev_id;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);

	unipi_spi_trace(KERN_INFO "UNIPISPI: nspi%d SPI IRQ\n", n_spi->neuron_index);
	kthread_queue_work(&n_spi->primary_worker, &n_spi->irq_work);

	return IRQ_HANDLED;
}

void neuronspi_enable_uart_interrupt(struct neuronspi_port* n_port)
{
    // ToDo: save, which port requested enable to support disabling
    struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
    neuronspi_spi_set_irqs(spi, 0x5);
    if (n_spi->no_irq) {
        // start polling
        n_spi->poll_enabled = 1;
        // invoke first probe -> which invokes hrtimer
        kthread_queue_work(&n_spi->primary_worker, &n_spi->irq_work);
    }
}

s32 neuronspi_spi_probe(struct spi_device *spi)
{
	const struct neuronspi_devtype *devtype;
	struct neuronspi_driver_data *n_spi;
    struct neuronspi_op_buffer recv_op;
	s32 ret, i, no_irq = 0;
	u8 uart_count = 0;
    u32 probe_always_succeeds;
	u32 always_create_uart;

	unsigned long flags;
    
	n_spi = kzalloc(sizeof *n_spi, GFP_ATOMIC);
	spin_lock_irqsave(neuronspi_probe_spinlock, flags);
	neuronspi_probe_count++;
	spin_unlock_irqrestore(neuronspi_probe_spinlock, flags);
	if (!n_spi)
		return -ENOMEM;
	unipi_spi_trace(KERN_INFO "UNIPISPI: CS%d Probe Started\n", spi->chip_select);
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

	unipi_spi_trace(KERN_DEBUG "UNIPISPI: CS%d, Chip Max Hz-%d %d\n", spi->chip_select, spi->master->max_speed_hz, spi->max_speed_hz);
	if (spi->dev.of_node) {
		const struct of_device_id *of_id =
			of_match_device(neuronspi_id_match, &spi->dev);
		if (!of_id) {
			printk(KERN_DEBUG "UNIPISPI: Probe %s does not match a device!\n", *(&spi->dev.of_node->full_name));
            kfree(n_spi);
			return -ENODEV;
		}
		of_property_read_u32_array(spi->dev.of_node, "neuron-board-index", &(n_spi->neuron_index), 1);
		of_property_read_u32_array(spi->dev.of_node, "neuron-probe-always-succeeds", &(probe_always_succeeds), 1);
		of_property_read_u32_array(spi->dev.of_node, "neuron-always-create-tty", &(always_create_uart), 1);
		devtype = (struct neuronspi_devtype *)of_id->data;
		unipi_spi_trace(KERN_INFO "UNIPISPI: CS%d, Device Tree Node defined index=%d\n", spi->chip_select, n_spi->neuron_index);

	} else {
		const struct spi_device_id *id_entry = spi_get_device_id(spi);
		devtype = (struct neuronspi_devtype *)id_entry->driver_data;
	}


	// We perform an initial probe of registers 1000-1004 to identify the device, using a premade message
	n_spi->first_probe_reply = kzalloc(UNIPISPI_PROBE_MESSAGE_LEN, GFP_ATOMIC);	// allocate space for initial probe
	n_spi->lower_board_id = n_spi->upper_board_id = n_spi->combination_id = 0xFF;

	neuronspi_spi_send_const_op(spi, &UNIPISPI_IDLE_MESSAGE, &recv_op, 0, NEURONSPI_DEFAULT_FREQ, 25);
	// Throw away the first message - the associated SPI Master is sometimes not properly initialised at this point

    recv_op.second_message = n_spi->first_probe_reply;
	i = 0;
	do {
        neuronspi_spi_send_const_op(spi, &UNIPISPI_PROBE_MESSAGE, &recv_op, UNIPISPI_PROBE_MESSAGE_LEN, NEURONSPI_DEFAULT_FREQ, 25);
		i++;
	} while (n_spi->first_probe_reply[0] == 0 && i < 5);

	if (n_spi->first_probe_reply[0] != 0) { 	// CRC error sets the first byte to 0
		uart_count = n_spi->first_probe_reply[8] & 0x0f;
        //lower_board_id = n_spi->first_probe_reply[13];
        
		for (i = 0; i < NEURONSPI_BOARDTABLE_LEN; i++) {
			if (n_spi->first_probe_reply[13] == NEURONSPI_BOARDTABLE[i].lower_board_id) {
				if (n_spi->combination_id == 0xFF && NEURONSPI_BOARDTABLE[i].upper_board_id == 0) {
					n_spi->combination_id = NEURONSPI_BOARDTABLE[i].index;
				}
				if (n_spi->lower_board_id == 0xFF) {
					n_spi->lower_board_id = n_spi->first_probe_reply[11];
				}
				if (n_spi->first_probe_reply[11] == NEURONSPI_BOARDTABLE[i].index) {
					n_spi->combination_id = n_spi->first_probe_reply[11];
					n_spi->upper_board_id = NEURONSPI_BOARDTABLE[i].upper_board_id;
				}
			}
		}

	} else if (!probe_always_succeeds) {
		ret = -ENODEV;
		kfree(n_spi);
		printk(KERN_INFO "UNIPISPI: Probe did not detect a valid UniPi device at CS%d\n", spi->chip_select);
		return ret;

	} else if (always_create_uart) {
		uart_count = 1;
	}

	if (n_spi->lower_board_id != 0xFF && n_spi->combination_id != 0xFF) {
		n_spi->features = &(NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->features);
	} else {
		n_spi->features = NULL;
	}

	n_spi->ideal_frequency = NEURONSPI_COMMON_FREQ;
	for (i = 0; i < NEURONSPI_SLOWER_MODELS_LEN; i++) {
		if (NEURONSPI_SLOWER_MODELS[i] == (n_spi->first_probe_reply[13] << 8 | n_spi->first_probe_reply[12])) {
			n_spi->ideal_frequency = NEURONSPI_SLOWER_FREQ;
		}
	}

    // Prepare worker for interrupt, LEDs, 
	kthread_init_worker(&n_spi->primary_worker);

	n_spi->primary_worker_task = kthread_run(kthread_worker_fn, &n_spi->primary_worker, "unipispi%d", n_spi->neuron_index);
	if (IS_ERR(n_spi->primary_worker_task )) {
		ret = PTR_ERR(n_spi->primary_worker_task);
        kfree(n_spi);
		return ret;
	}
	sched_setscheduler(n_spi->primary_worker_task, SCHED_FIFO, &neuronspi_sched_param);

	if (n_spi->lower_board_id != 0xFF && n_spi->combination_id != 0xFF) {
		printk(KERN_INFO "UNIPISPI: Probe detected UniPi Board %s (L:%x U:%x C:%x) Fw: v%d.%d at CS%d (id=%d)\n\t\tUarts:%d, reg1000-4: %10ph\n",
				NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->combination_name,
				n_spi->lower_board_id, n_spi->upper_board_id, n_spi->combination_id, 
				n_spi->first_probe_reply[5],  n_spi->first_probe_reply[4], 
				spi->chip_select,  n_spi->neuron_index, uart_count, n_spi->first_probe_reply +4) ;
	} else if (n_spi->lower_board_id != 0xFF) {
		printk(KERN_INFO "UNIPISPI: Probe detected UniPi Board (L:%x C:???) Fw: v%d.%d at CS%d (id=%d)\n\t\tUarts:%d, reg1000-4: %10ph\n",
				n_spi->lower_board_id, n_spi->first_probe_reply[5],  n_spi->first_probe_reply[4], 
				spi->chip_select,  n_spi->neuron_index, uart_count, n_spi->first_probe_reply +4) ;
	} else {
		printk(KERN_INFO "UNIPISPI: Probe detected UniPi Board (L:??? C:???) Fw: v%d.%d at CS%d (id=%d)\n\t\tUarts:%d, reg1000-4: %10ph\n",
				n_spi->first_probe_reply[5],  n_spi->first_probe_reply[4], 
				spi->chip_select,  n_spi->neuron_index, uart_count, n_spi->first_probe_reply +4) ;
	}
	if (n_spi->combination_id != 0xFF) {
		printk(KERN_INFO "UNIPISPI: UniPi device %s at CS%d uses SPI communication freq. %d Hz\n",
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

	//n_spi->char_driver = &neuronspi_cdrv;
	spin_lock_irqsave(neuronspi_probe_spinlock, flags);

	neuronspi_s_dev[n_spi->neuron_index] = spi;
	spi_set_drvdata(spi, n_spi);

	if (neuronspi_probe_count == NEURONSPI_MAX_DEVS) {
		neuronspi_model_id = neuronspi_find_model_id(neuronspi_probe_count);
	}
	spin_unlock_irqrestore(neuronspi_probe_spinlock, flags);
	if (neuronspi_model_id != -1) {
		printk(KERN_INFO "UNIPISPI: Detected UniPi board combination corresponding to %s\n", NEURONSPI_MODELTABLE[neuronspi_model_id].model_name);
	}

    /* Platform LED, GPIO, IIO sysfs subsystem init */
	if (neuron_plc_dev == NULL) {
        // global platform root
		neuron_plc_dev = platform_device_alloc("unipi_plc", -1);
		neuron_plc_dev->dev.groups = neuron_plc_attr_groups;
		platform_device_add(neuron_plc_dev);
	}
    
    n_spi->board_device = neuronspi_board_device_probe(n_spi);

	if (n_spi->features) {
		if (n_spi->features->led_count) {
			printk(KERN_INFO "UNIPISPI: LED: %d User leds detected at CS%d\n", n_spi->features->led_count, spi->chip_select);
			n_spi->led_driver = neuronspi_led_probe(n_spi->features->led_count, n_spi->neuron_index, n_spi->board_device);
		}
#ifdef CONFIG_GPIOLIB
		if (n_spi->features->di_count) {
			n_spi->di_driver = neuronspi_di_probe(n_spi->features->di_count, n_spi->neuron_index, n_spi->board_device);
		}
		if (n_spi->features->do_count) {
			n_spi->do_driver = neuronspi_do_probe(n_spi->features->do_count, n_spi->neuron_index, n_spi->board_device);
		}

		if (n_spi->features->ro_count) {
			n_spi->ro_driver = neuronspi_ro_probe(n_spi->features->ro_count, n_spi->neuron_index, n_spi->board_device);
		}
#endif
		if (n_spi->features->stm_ai_count) {
			n_spi->stm_ai_driver = neuronspi_stm_ai_probe(n_spi->features->stm_ai_count, n_spi->neuron_index, n_spi->board_device);
		}
		if (n_spi->features->stm_ao_count) {
			n_spi->stm_ao_driver = neuronspi_stm_ao_probe(n_spi->features->stm_ao_count, n_spi->neuron_index, n_spi->board_device);
		}
		if (n_spi->features->sec_ai_count) {
			n_spi->sec_ai_driver = neuronspi_sec_ai_probe(n_spi->features->sec_ai_count, n_spi->neuron_index, n_spi->board_device);
		}
		if (n_spi->features->sec_ao_count) {
			n_spi->sec_ao_driver = neuronspi_sec_ao_probe(n_spi->features->sec_ao_count, n_spi->neuron_index, n_spi->board_device);
		}
	}

    n_spi->uart_count_to_probe = uart_count;
	if (uart_count) {
        if (neuronspi_uart_driver_global != NULL) {	
            // Normally is port registration done after unipispi driver probe. (in the end of __init__)
            unipi_spi_trace(KERN_DEBUG "UNIPISPI: UART registration\n");
            neuronspi_uart_probe(spi, n_spi);

        } else {
            unipi_spi_trace(KERN_DEBUG "UNIPISPI: Uart driver not registered yet. Uart port add later.\n");
        }
	}

	neuronspi_spi_set_irqs(spi, 0x5);
    for (i = 0; i < NEURONSPI_NO_INTERRUPT_MODELS_LEN; i++) {
		if (NEURONSPI_NO_INTERRUPT_MODELS[i] == (n_spi->first_probe_reply[11] << 8 | n_spi->first_probe_reply[10])) {
			no_irq = 1;
			break;
		}
	}

    kthread_init_work(&(n_spi->irq_work), neuronspi_irq_proc); // prepare work function for interrupt status checking
	//n_spi->poll_thread = NULL;
	if (!no_irq) {
		n_spi->no_irq = 0;
		ret = devm_request_irq(&(spi->dev), spi->irq, neuronspi_spi_irq, 0x81, dev_name(&(spi->dev)), spi);
		unipi_spi_trace(KERN_DEBUG "UNIPISPI: SPI IRQ %d registration: ret=%d\n", spi->irq, ret);
	} else {
        n_spi->no_irq = 1;
        hrtimer_init(&n_spi->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        n_spi->poll_timer.function = neuronspi_poll_timer_func;
        unipi_spi_trace(KERN_DEBUG "UNIPISPI: NO IRQ ON THIS MODEL !!\n");
	}

	return ret;
}

s32 neuronspi_spi_remove(struct spi_device *spi)
{
	int i;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	if (n_spi) {

		if (n_spi->no_irq) {
            hrtimer_cancel(&n_spi->poll_timer);
        }
		kthread_flush_worker(&n_spi->primary_worker);

		if (n_spi->led_driver) {
			for (i = 0; i < n_spi->features->led_count; i++) {
				led_classdev_unregister(&(n_spi->led_driver[i].ldev));
			}
			kfree(n_spi->led_driver);
			n_spi->led_driver = NULL;
		}
		printk(KERN_INFO "UNIPISPI: LED DRIVER UNREGISTERED\n");
        
		if (n_spi->di_driver) { neuronspi_gpio_remove(n_spi->di_driver); }
		if (n_spi->do_driver) { neuronspi_gpio_remove(n_spi->do_driver); }
		if (n_spi->ro_driver) { neuronspi_gpio_remove(n_spi->ro_driver); }
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
		printk(KERN_INFO "UNIPISPI: SPI/UART DRIVER UNREGISTERED\n");
		if (n_spi->board_device) {
			platform_set_drvdata(n_spi->board_device, 0);
			platform_device_unregister(n_spi->board_device);
		}
		kfree(n_spi);
	}
	return 0;
}


struct file_operations file_ops =
{
	.open 				= neuronspi_open,
	.read 				= neuronspi_read,
	.write 				= neuronspi_write,
	.release 			= neuronspi_release,
	.owner				= THIS_MODULE
};


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


MODULE_ALIAS("spi:unipispi");

static s32 __init neuronspi_init(void)
{
	s32 ret = 0;

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
		printk(KERN_INFO "UNIPISPI: SPI Driver Registered, Major Version: %s\n", NEURONSPI_MAJOR_VERSIONSTRING);
	}

	neuronspi_invalidate_thread = kthread_create(neuronspi_regmap_invalidate, NULL, "unipispi_inv");
	if (neuronspi_invalidate_thread != NULL) {
		wake_up_process(neuronspi_invalidate_thread);
	}

	if (!(neuronspi_cdrv.major_number)) { // Register character device if it doesn't exist
		ret = char_register_driver();
		if (ret) {
			printk(KERN_ERR "UNIPISPI: Failed to register the neuronspi character driver, ERR:%d\n", ret);
		}
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
