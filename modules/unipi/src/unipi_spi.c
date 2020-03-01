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
#include <linux/completion.h>
#include <linux/cpufreq.h>

#include "unipi_common.h"
#include "unipi_sysfs.h"
#include "unipi_uart.h"
#include "unipi_platform.h"
#include "unipi_gpio.h"
#include "unipi_iio.h"
#include "unipi_misc.h"
#include "unipi_spi.h"
#include "unipi_uart.h"
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

// Instantiated once
struct neuronspi_char_driver
{
	int major_number;
	u32 open_counter;
	struct class* driver_class;
	struct device* dev;
};


//struct mutex neuronspi_master_mutex;
spinlock_t   unipi_spi_master_lock;
struct mutex unipi_inv_speed_mutex;
int neuronspi_model_id = -1;
struct spi_device *neuronspi_s_dev[NEURONSPI_MAX_DEVS];  // global list of neuron spi devices
struct task_struct *neuronspi_invalidate_thread;

static u8 neuronspi_probe_count = 0;
static struct spinlock *neuronspi_probe_spinlock;
static struct sched_param neuronspi_sched_param = { .sched_priority = MAX_RT_PRIO / 2 };

struct neuronspi_char_driver neuronspi_cdrv =
{
	.major_number = -1,
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


#define UNIPISPI_PROBE_MESSAGE_LEN	16
static u8 _probe_message_second[UNIPISPI_PROBE_MESSAGE_LEN] = 
{   0x04, 0x00, 0xe8, 0x03, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x12, 0x16
};

static struct neuronspi_op_buffer UNIPISPI_PROBE_MESSAGE = {
    first_message: {0x04, 0x0e, 0xe8, 0x03, 0xa0, 0xdd},
    second_message: _probe_message_second,
};


static struct neuronspi_op_buffer UNIPISPI_IDLE_MESSAGE = {
    first_message: {0xfa, 0x00, 0x55, 0x0e, 0xb6, 0x0a},
    second_message: NULL,
};


/************************************************************

  patches to other modules
		- chip select on spi - monitor elpapsed time between operations
		- cpu frequency setting - try to suppress changing freq during spi op

***********************************************************/


    static unsigned long loop_per_us = 24; 
    u16 unipi_spi_master_flag = 0;
    void (*unipi_spi_master_set_cs)(struct spi_device *spi, bool enable) = NULL;
    //cycles_t unipi_spi_cs_cycles;
#ifdef USE_UNIPI_CPUFREQ_PATCH
	static struct cpufreq_policy * current_policy = NULL;
#endif

static void unipi_spi_set_cs(struct spi_device *spi, bool enable)
{
    unsigned long udelta;
    struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);
    //if ((d_data->neuron_index < NEURONSPI_MAX_DEVS) && (neuronspi_s_dev[d_data->neuron_index] == spi)) {
    //  
    //}
    cycles_t cs_cycles = get_cycles();
    if (!enable && d_data) {
        udelta = ((cs_cycles - d_data->last_cs_cycles) / loop_per_us);
        unipi_spi_trace(KERN_INFO "UNIPISPI: enable=%d csdelta:%ld us\n", !enable, udelta);
        if (udelta < NEURONSPI_LAST_TRANSFER_DELAY) {
            udelay(NEURONSPI_LAST_TRANSFER_DELAY - udelta);
        }
    }
#ifdef USE_UNIPI_CPUFREQ_PATCH
	//current_policy = cpufreq_cpu_get_raw(task_cpu(current));
	current_policy = cpufreq_cpu_get_raw(0);
	if (current_policy && !enable) {
 wait:
        wait_event(current_policy->transition_wait, !current_policy->transition_ongoing);
        spin_lock(&current_policy->transition_lock);
        if (unlikely(current_policy->transition_ongoing)) {
            spin_unlock(&current_policy->transition_lock);
            goto wait;
        }

        current_policy->transition_ongoing = true;
        current_policy->transition_task = current;
        spin_unlock(&current_policy->transition_lock);
	}
#endif
	if (gpio_is_valid(-spi->cs_gpio)) {
		gpio_set_value(-spi->cs_gpio, enable);
        if ((unipi_spi_master_set_cs != NULL)  &&
           (unipi_spi_master_flag & SPI_MASTER_GPIO_SS)) {
            unipi_spi_master_set_cs(spi, enable);
        }
    } else {
        if (unipi_spi_master_set_cs != NULL) 
            unipi_spi_master_set_cs(spi, enable);
    }
    if (d_data) d_data->last_cs_cycles = cs_cycles;
#ifdef USE_UNIPI_CPUFREQ_PATCH             
	if (current_policy && enable) {
        current_policy->transition_ongoing = false;
        current_policy->transition_task = NULL;
        wake_up(&current_policy->transition_wait);     
	}
#endif
}


/************************
 * Non-static Functions *
 ************************/
int neuronspi_spi_send_op(struct spi_device* spi_dev, struct neuronspi_op_buffer* send_buf,
                            struct neuronspi_op_buffer* recv_buf, s32 len, 
                            s32 freq, s32 delay, s32 send_header, u8 lock_val);

int neuronspi_spi_send_const_op(struct spi_device* spi_dev, struct neuronspi_op_buffer* send_buf,
                            struct neuronspi_op_buffer* recv_buf, s32 len, 
                            s32 freq, s32 delay)
{
    return neuronspi_spi_send_op(spi_dev, send_buf, recv_buf, len, freq, delay, UNIPISPI_OP_MODE_SEND_HEADER, 0);
}

void unipi_spi_read_str(struct spi_device* spi, struct neuronspi_port* port);


struct unipi_spi_context {
    struct completion* unipi_spi_completion;
    int len;
    struct neuronspi_op_buffer* recv_buf;
    struct neuronspi_port* string_op_port;
    struct spi_message message;
    //struct spi_transfer _transfer[]   
};

static struct neuronspi_port* unipi_spi_check_message(struct unipi_spi_context* context)
{
    struct neuronspi_op_buffer* recv_buf = context->recv_buf;
    struct neuronspi_driver_data *d_data;// = context->d_data;
    int len = context->len;
    struct neuronspi_port* port = NULL;
    u16 packet_crc, recv_crc;
    u8 opcode;    
    int portindex;
    
    d_data = spi_get_drvdata(context->message.spi);
    
    if (context->message.status != 0) {
        // odeslani se nepovedlo z nejakeho duvodu
        unipi_spi_trace(KERN_INFO "UNIPISPI: Error spi transaction: txopcode:%d\n", recv_buf->first_message[0]);
        goto err;
    }

    unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI Master Read (op1) %8ph\n", recv_buf->first_message);
    if (d_data && d_data->poll_enabled) {
       // reschedule poll timer (pseudo interrupt)
       hrtimer_start_range_ns(&d_data->poll_timer, 2000000, 4000000, HRTIMER_MODE_REL);
    }
    // check crc on header
	recv_crc = neuronspi_spi_crc(recv_buf->first_message, 4, 0);
	packet_crc = *((u16*)(recv_buf->first_message+4));

	if (recv_crc != packet_crc) {
		unipi_spi_trace(KERN_INFO "UNIPISPI: SPI CRC1 Not Correct (Received: %04x Calculated: %04x)\n", packet_crc, recv_crc);
        recv_buf->first_message[0] = 0;
        goto err;
    }
    opcode = recv_buf->first_message[0];
    if ((opcode >= 0x41)&&(opcode <= 0x44)) {
        // Signal the UART to issue character reads
        portindex = (opcode - 0x41);
        if ((d_data != NULL) && d_data->uart_count && (portindex < d_data->uart_count)) {
            unipi_spi_trace_1(KERN_INFO "UNIPISPI: Reading UART data for device %d, opcode=%02x\n", d_data->neuron_index, opcode);
            port = neuronspi_uart_data_global->p + d_data->uart_pindex + portindex;
            // put one incomming character from UART
            neuronspi_uart_handle_rx(port, 1, recv_buf->first_message+3);
            // read queue length
            port->rx_remain = (recv_buf->first_message[2]==0  ? 256 : recv_buf->first_message[2]) - 1;
                        
            unipi_spi_trace(KERN_INFO "UNIPISPI: UART Buffer:%d, ttyNS%d(%d:%d)\n", port->rx_remain, port->port.line, port->dev_index, port->dev_port);
        }
    } else if (opcode != 0xfa) {
        unipi_spi_trace(KERN_INFO "UNIPISPI: Err rx:%d\n",  opcode);
        goto err;
	}

    if (len > 0) {
        // Check second message crc
		recv_crc = neuronspi_spi_crc(recv_buf->second_message, len, recv_crc);
        packet_crc = recv_buf->second_message[len] | (recv_buf->second_message[len+1] << 8);
		unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI Master Read - %d:\n\t%64ph\n\t%64ph\n\t%64ph\n\t%64ph\n", len, 
                                        recv_buf->second_message, recv_buf->second_message + 64,
                                        recv_buf->second_message+128, recv_buf->second_message+192);

		if (recv_crc != packet_crc) {
            unipi_spi_trace(KERN_INFO "UNIPISPI: SPI CRC2 Not Correct: %04x COMPUTED: %04x\n", packet_crc, recv_crc);
            goto err;
        }
#ifdef UNIPISPI_USE_RX_THREAD
        if (recv_buf->second_message[0] == 0x65) {
            // this op can be invoked only from kernel_work rx_proc
            portindex = 0; // second_message[2]; Overit ve firmware
            if (d_data->uart_count && (portindex < d_data->uart_count)) {
                port = neuronspi_uart_data_global->p + d_data->uart_pindex + portindex;
                port->rx_remain = recv_buf->second_message[3];
                neuronspi_rx_queue_swap(port);
            }
        }
#endif
    }

    return(port);

err:
    if (len>0) recv_buf->second_message[0] = 0;
    recv_buf->first_message[0] = 0;

    return(port);
}




#define clear_op_buffer(buffer, len)    { buffer->first_message[0] = 0;\
                                if ((len > 0) && (buffer->second_message)) buffer->second_message[0] = 0; }


struct unipi_spi_context* unipi_spi_setup_context(struct spi_device* spi_dev, struct neuronspi_op_buffer* send_buf,
                            struct neuronspi_op_buffer* recv_buf, int len, 
                            int freq, int delay, s32 send_header)
{
    //struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	int trans_count, remain, i;
	struct unipi_spi_context *context;
    struct spi_transfer * s_trans;
	u16 packet_crc = 0;

    if (len == 0) {
        trans_count = 2;
    } else {
        if (send_header & UNIPISPI_OP_MODE_DO_CRC) len += 2;
        trans_count = ((len-1) / NEURONSPI_MAX_TX) + 3; // number of transmissions
    }

	context = kzalloc(sizeof(struct unipi_spi_context)	+ trans_count * sizeof(struct spi_transfer), GFP_ATOMIC);
	if (! context) {
        return NULL;
	}
    s_trans = (struct spi_transfer *)(context + 1);
    spi_message_init_with_transfers(&context->message, s_trans, trans_count);

	s_trans[0].delay_usecs = NEURONSPI_EDGE_DELAY;
	s_trans[0].bits_per_word = 8;
	s_trans[0].speed_hz = freq;

	s_trans[1].bits_per_word = 8;
	s_trans[1].speed_hz = freq;
    if (send_header  & UNIPISPI_OP_MODE_SEND_HEADER) {
		unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI Master Write(op1) %8ph\n", send_buf->first_message);
        if (send_header & UNIPISPI_OP_MODE_DO_CRC) {
            packet_crc = neuronspi_spi_crc(send_buf->first_message, 4, 0);
            *((u16*)(send_buf->first_message+4)) = packet_crc;
        }
        s_trans[1].delay_usecs = delay;
        s_trans[1].len = NEURONSPI_FIRST_MESSAGE_LENGTH;
        s_trans[1].tx_buf = send_buf->first_message;
        s_trans[1].rx_buf = recv_buf->first_message;
    }
    if (len > 0) {
        if (send_header & UNIPISPI_OP_MODE_DO_CRC) {
            packet_crc = neuronspi_spi_crc(send_buf->second_message, len-2, packet_crc);
            send_buf->second_message[len-2] = packet_crc & 0xff;
            send_buf->second_message[len-1] = packet_crc >> 8;
        }
		unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI Master Write(%3d) %32ph\n", len, send_buf->second_message);
		remain = len;
		for (i = 2; i < trans_count; i++) {
			s_trans[i].delay_usecs = 0;
			s_trans[i].bits_per_word = 8;
			s_trans[i].speed_hz = freq;
			s_trans[i].tx_buf = send_buf->second_message + (NEURONSPI_MAX_TX * (i - 2));
			s_trans[i].rx_buf = recv_buf->second_message + (NEURONSPI_MAX_TX * (i - 2));
			s_trans[i].len = (remain > NEURONSPI_MAX_TX) ? NEURONSPI_MAX_TX : remain;
			remain -= NEURONSPI_MAX_TX;
		}
        // len is size of second message WITHOUT crc
        len -= 2;
	}
    
    context->message.context = context;
    context->message.spi = spi_dev;
    context->len = len;
    context->recv_buf = recv_buf;
    return context;
}

/*
static void unipi_spi_idle_op_complete(void *arg)
{
    struct unipi_spi_context* context = (struct unipi_spi_context*) arg;
    struct neuronspi_port *port;

    port = unipi_spi_check_message(context);
    if (port && (port->rx_remain))  unipi_spi_read_str(context->message.spi, port);    
    kfree(context->recv_buf);
    kfree(context);
}

void unipi_spi_idle_op(struct spi_device* spi)
{
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);
    struct neuronspi_op_buffer* recv_op;// = &d_data->idle_recv_buf;
	struct unipi_spi_context *context;
    unsigned long flags;

    unipi_spi_trace(KERN_INFO "UNIPISPI: Idle op\n");
    recv_op = kzalloc(sizeof(struct neuronspi_op_buffer), GFP_ATOMIC);
    clear_op_buffer(recv_op, 0);
    
    context = unipi_spi_setup_context(spi, &UNIPISPI_IDLE_MESSAGE, recv_op, 0, d_data->ideal_frequency, 0, UNIPISPI_OP_MODE_SEND_HEADER);
    if (context == NULL) goto err; // no mem for kzalloc
    
    context->message.complete = unipi_spi_idle_op_complete;
     
    spin_lock_irqsave(&unipi_spi_master_lock, flags);
    if  ((d_data->reserved_device)       // Running reserved operations
         ||(spi_async(spi, &context->message) != 0)) {
        spin_unlock_irqrestore(&unipi_spi_master_lock, flags);
        kfree(context);
        unipi_spi_trace(KERN_INFO "UNIPISPI: Err=3 Idle op\n");
        goto err;
    }
    spin_unlock_irqrestore(&unipi_spi_master_lock, flags);
    return;
    
err:
    kfree(recv_op);
    if (d_data->poll_enabled) {
        // reschedule poll timer
        hrtimer_start_range_ns(&d_data->poll_timer, 2000000, 4000000, HRTIMER_MODE_REL);
    }
}
*/

/************************************************************************************
 * 
 *    unipi_spi_read_str()
 *      can be called only from spi thread
 *      for particular port can be only one read_str operation in progress
 */
 
static void unipi_spi_read_str_complete(void *arg)
{
    struct unipi_spi_context* context = (struct unipi_spi_context*) arg;
    struct neuronspi_op_buffer* recv_buf = context->recv_buf;
    struct neuronspi_driver_data *d_data = spi_get_drvdata(context->message.spi);
    struct neuronspi_port *port, *port2;
    int portindex2;

    port = unipi_spi_check_message(context);
    context->string_op_port->rx_in_progress = 0;

    if (recv_buf->second_message[0] == 0x65) {
        // this op can be invoked only from kernel_work rx_proc
        portindex2 = 0; // second_message[2]; Overit ve firmware
        if (d_data->uart_count && (portindex2 < d_data->uart_count)) {
            port2 = neuronspi_uart_data_global->p + d_data->uart_pindex + portindex2;
            if (port && (port == port2)) {
                port = NULL;
            }
            neuronspi_uart_handle_rx(port2, recv_buf->second_message[1], recv_buf->second_message+4);
            port2->rx_remain = recv_buf->second_message[3];
            if (port2->rx_remain) unipi_spi_read_str(context->message.spi, port2);
        }
    }

    if (port && (port->rx_remain))  unipi_spi_read_str(context->message.spi, port);

    kfree(context);
}

void unipi_spi_read_str(struct spi_device* spi, struct neuronspi_port* port)
{
	struct neuronspi_op_buffer* send_op = &port->rx_send_buf;
	struct neuronspi_op_buffer* recv_op = &port->rx_recv_buf;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);
	struct unipi_spi_context *context;
	int transmit_len, len, locked;
    unsigned long flags;

    spin_lock_irqsave(&port->rx_in_progress_lock, flags);
    locked = port->rx_in_progress;
    if (!locked) port->rx_in_progress = 1;
    spin_unlock_irqrestore(&port->rx_in_progress_lock, flags);
    if (locked) return;

    len = port->rx_remain;
	if (len < 246) {
		len = (len+4);
	} else {
		len = 250;
	}
	transmit_len = (len & 1 ? len+1 : len) + 4;  // transmit_length must be even

	send_op->first_message[1] = transmit_len;     //
	send_op->first_message[2] = 0;
	send_op->first_message[3] = port->dev_port;
	send_op->first_message[0] = 0x65;
	send_op->second_message[0] = 0x65;

    unipi_spi_trace(KERN_INFO "UNIPISPI: Read string ttyNS%d, len=%d\n", port->port.line, transmit_len-4);

    clear_op_buffer(recv_op, len);
    
    context = unipi_spi_setup_context(spi, send_op, recv_op, transmit_len, d_data->ideal_frequency, 20, UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC);
    if (context == NULL) goto err; // no mem for kzalloc
    
    context->message.complete = unipi_spi_read_str_complete;
    context->string_op_port = port;
     
    spin_lock_irqsave(&unipi_spi_master_lock, flags);
    if  ((d_data->reserved_device) ||        // Running reserved operations
         (spi_async(spi, &context->message) != 0)) {
        spin_unlock_irqrestore(&unipi_spi_master_lock, flags);
        kfree(context);
        unipi_spi_trace(KERN_INFO "UNIPISPI: Err=3 on Read string\n");
        goto err;
    }
    spin_unlock_irqrestore(&unipi_spi_master_lock, flags);
    return;
    
err:
    port->rx_in_progress = 0;
}

/************************************************************************************
 * 
 *    unipi_spi_write_str()
 *      
 *      for particular port can be only one write_str operation in progress
 */
 
static void unipi_spi_write_str_complete(void *arg)
{
    struct unipi_spi_context* context = (struct unipi_spi_context*) arg;
    //struct neuronspi_op_buffer* recv_buf = context->recv_buf;
    //struct neuronspi_driver_data *d_data = spi_get_drvdata(context->message.spi);
    unsigned long flags;
    struct neuronspi_port *port;

    port = unipi_spi_check_message(context);
    if (port && (port->rx_remain))  unipi_spi_read_str(context->message.spi, port);
	context->string_op_port->tx_fifo_len += context->len;

   	spin_lock_irqsave(&context->string_op_port->port.lock, flags);
    unipi_uart_handle_tx(context->string_op_port, CB_WRITESTRING);
   	spin_unlock_irqrestore(&context->string_op_port->port.lock, flags);
    
    kfree(context);
}

int unipi_spi_write_str(struct spi_device* spi, struct neuronspi_port* port, int length)
{
	struct neuronspi_op_buffer* send_op = &port->tx_send_buf;
	struct neuronspi_op_buffer* recv_op = &port->tx_recv_buf;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);
	struct unipi_spi_context *context;
	int transmit_len;
    unsigned long flags;

	unipi_spi_trace(KERN_INFO "UNIPISPI: SPI Write string ttyNS%d, len:%d\n", port->port.line, length);
	if ((length <= 0) || (length > 256)) return 1;

    send_op->first_message[2] = 0;
    send_op->first_message[3] = port->dev_port;
	if (length == 1) {
		send_op->first_message[0] = 0x41;
		send_op->first_message[1] = port->tx_send_msg[0];
        transmit_len = 0;
	} else {
		send_op->first_message[0] = 0x64;                           //NEURONSPI_SPI_UART_LONG_MESSAGE[0];
		send_op->first_message[1] = length == 256 ? 0 : length;     // special case length==256
		transmit_len = length & 1 ? length+1 : length;              // transmit_length must be even
    }
    clear_op_buffer(recv_op, transmit_len);

    context = unipi_spi_setup_context(spi, send_op, recv_op, transmit_len, d_data->ideal_frequency, 20, UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC);
    if (context == NULL) goto err; // no mem for kzalloc
    
    context->message.complete = unipi_spi_write_str_complete;
    context->string_op_port = port;

    spin_lock_irqsave(&unipi_spi_master_lock, flags);
    if  ((d_data->reserved_device) ||        // Running reserved operations
         (spi_async(spi, &context->message) != 0)) {
        spin_unlock_irqrestore(&unipi_spi_master_lock, flags);
        kfree(context);
        unipi_spi_trace(KERN_INFO "UNIPISPI: Err=3 in Write string\n");
        goto err;
    }
    spin_unlock_irqrestore(&unipi_spi_master_lock, flags);
    return 0;
    
err:
    return 1;
}

/************************************************************************************
 * 
 *    unipi_spi_get_tx_fifo()
 *      
 *      for particular port can be only one get_tx_fifo or write_str operation in progress
 */
 
static void unipi_spi_get_tx_fifo_complete(void *arg)
{
    struct unipi_spi_context* context = (struct unipi_spi_context*) arg;
    u8 * recv_msg = context->string_op_port->tx_recv_msg;
    //struct neuronspi_driver_data *d_data = spi_get_drvdata(context->message.spi);
    struct neuronspi_port *port;
    unsigned long flags;

    port = unipi_spi_check_message(context);
    if (port && (port->rx_remain))  unipi_spi_read_str(context->message.spi, port);

    unipi_spi_trace("UNIPISPI: get_tx_fifo_complete recv_msg: %16ph\n", recv_msg);
	if (recv_msg[0] == 0x03) {
		context->string_op_port->tx_fifo_len = recv_msg[4] + ((recv_msg[5]) << 8);
	}
   	spin_lock_irqsave(&context->string_op_port->port.lock, flags);
    unipi_uart_handle_tx(context->string_op_port, CB_GETTXFIFO);
    spin_unlock_irqrestore(&context->string_op_port->port.lock, flags);
    
    kfree(context);
}

int unipi_spi_get_tx_fifo(struct spi_device* spi, struct neuronspi_port* port)
{
	struct neuronspi_op_buffer* send_op = &port->tx_send_buf;
	struct neuronspi_op_buffer* recv_op = &port->tx_recv_buf;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);
	struct unipi_spi_context *context;
    unsigned long flags;

	unipi_spi_trace(KERN_INFO "UNIPISPI: SPI Get TX fifo length ttyNS%d\n", port->port.line);

    send_op->first_message[0] = 0x03;
    send_op->first_message[1] = 4+2;
    *((u16*)(send_op->first_message + 2)) = port->tx_fifo_reg;
	memcpy(send_op->second_message, send_op->first_message, 4);
    clear_op_buffer(recv_op, (4+2));

    context = unipi_spi_setup_context(spi, send_op, recv_op, 4+2, d_data->ideal_frequency, 35, UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC);
    if (context == NULL) goto err; // no mem for kzalloc
    
    context->message.complete = unipi_spi_get_tx_fifo_complete;
    context->string_op_port = port;

    spin_lock_irqsave(&unipi_spi_master_lock, flags);
    if  ((d_data->reserved_device) ||        // Running reserved operations
         (spi_async(spi, &context->message) != 0)) {
        spin_unlock_irqrestore(&unipi_spi_master_lock, flags);
        kfree(context);
        unipi_spi_trace(KERN_INFO "UNIPISPI: Err=3 in Get Tx fifo\n");
        goto err;
    }
    spin_unlock_irqrestore(&unipi_spi_master_lock, flags);
    return 0;
    
err:
    return 1;
}

/* 
 * Callback from spi thread
 * invoked after transaction inserted by async_spi_send() is finished
 */
 
static void unipi_spi_firmware_op_complete(void *arg)
{
    struct unipi_spi_context* context = (struct unipi_spi_context*) arg;
    //struct neuronspi_op_buffer* recv_buf = context->recv_buf;

    // Don't check crc on firmware operation
	unipi_spi_trace_1(KERN_INFO "UNIPISPI: SPI Firmware op len=%d:\n\t%64ph\n", context->len, context->recv_buf->second_message);
    complete(context->unipi_spi_completion);
    kfree(context);
}


static void unipi_spi_send_op_complete(void *arg)
{
    struct unipi_spi_context* context = (struct unipi_spi_context*) arg;
    struct neuronspi_port* port;

    port = unipi_spi_check_message(context);
    if (port && (port->rx_remain)) unipi_spi_read_str(context->message.spi, port);

    if (context->unipi_spi_completion) 
        complete(context->unipi_spi_completion);
    kfree(context);
}

/*
 *  send_header: bits of UNIPISPI_OP_MODE_SEND_HEADER | UNIPISPI_OP_MODE_DO_CRC
 *  len:         length of second message. If DO_CRC is not set, len includes crc
 *  buffers:     must be long enough to calc crc  
 *  returns:     0 if success
 */
int neuronspi_spi_send_op(struct spi_device* spi_dev, struct neuronspi_op_buffer* send_buf,
                            struct neuronspi_op_buffer* recv_buf, s32 len, 
                            s32 freq, s32 delay, s32 send_header, u8 lock_val)
{
	int ret_code = 0;
	struct neuronspi_driver_data *d_data;
	struct unipi_spi_context *context;
    unsigned long flags;
    DECLARE_COMPLETION_ONSTACK(done); //struct completion done;

    unipi_spi_trace(KERN_INFO "UNIPISPI: SPI Send op=%d len=%d\n", send_buf->first_message[0], len);
    clear_op_buffer(recv_buf, len);
    d_data = spi_get_drvdata(spi_dev);

    context = unipi_spi_setup_context(spi_dev, send_buf, recv_buf, len, freq, delay, send_header);
    if (context == NULL) {
        ret_code = -3; // no mem for kzalloc
        goto err;
    }

    init_completion(&done);
    context->unipi_spi_completion = &done;
    if (send_header & UNIPISPI_OP_MODE_SEND_HEADER) {
        context->message.complete = unipi_spi_send_op_complete;
    } else {
        context->message.complete = unipi_spi_firmware_op_complete;
    }

    spin_lock_irqsave(&unipi_spi_master_lock, flags);
    // Check if there are running reserved operations
	if (d_data != NULL && d_data->reserved_device && lock_val != d_data->reserved_device) {
        ret_code = -1; // blocked by reservation
        goto errcontext;
	}

    ret_code = spi_async(spi_dev, &context->message);
    if (ret_code != 0) {
        unipi_spi_trace(KERN_INFO "UNIPISPI: Err=3 txopcode:%d\n", send_buf->first_message[0]);
        goto errcontext;
    }
    spin_unlock_irqrestore(&unipi_spi_master_lock, flags);

    wait_for_completion(&done);
    //cycles_t ct2 = get_cycles(); ktime_t t2 = ktime_get();
    //unipi_spi_trace(KERN_INFO "UNIPISPI: deltatime:%lldus cycles:%ld\n", ((long long) ktime_to_ns(ktime_sub(t2,t1)))/1000, ct2-ct1);
    return ret_code;

errcontext:
    spin_unlock_irqrestore(&unipi_spi_master_lock, flags);
    kfree(context);

err:
    return ret_code;
}

/*****************************************************************
 *  Modbus like Interface via /dev/unipispi 
 */
int neuronspi_open (struct inode *inode_p, struct file *file_p)
{
	struct neuronspi_file_data *f_internal_data;
	if (file_p == NULL || inode_p == NULL) {
		return -1;
	}
	neuronspi_cdrv.open_counter += 1;
	f_internal_data = kzalloc(sizeof(*f_internal_data), GFP_ATOMIC);
	f_internal_data->recv_buf.second_message = kzalloc(NEURONSPI_BUFFER_MAX, GFP_ATOMIC);
	f_internal_data->send_buf.second_message = kzalloc(NEURONSPI_BUFFER_MAX, GFP_ATOMIC);
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
	kfree(f_internal_data->recv_buf.second_message);
	kfree(f_internal_data->send_buf.second_message);
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
    unipi_spi_trace(KERN_INFO "UNIPISPI: CDEV Read %zu, nspi:%d msglen=%d offset=%d\n", len,
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

	unipi_spi_trace(KERN_INFO "UNIPISPI: CDEV Write len:%zu\n", len);
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
                                    UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC, 0);
	if (ret_code == 0) {
        if ((recv_data[0] == 0x03) && (recv_data[1]==1)) {  // check opcode and register count
            *value = *((u16*)(recv_data + 4));
        } else {
            //unipi_spi_trace("Read reg: %d %8ph\n", reg, recv_data);
            ret_code = 2;
        }
    }
    unipi_spi_trace("Read reg: %d ret: %d %8ph\n", reg, ret_code, recv_data);
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
                                    UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC, 0);
	if (ret_code == 0) {
        if ((recv_data[0] == 0x03) && (recv_data[1]==2)) {
            *value = *((u32*)(recv_data + 4));
        } else {
            ret_code = 2;
        }
    }
    unipi_spi_trace("Read reg32: %d ret: %d %10ph\n", reg, ret_code, recv_data);
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
                                    UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC, 0);
	if (ret_code == 0) {
        if ((recv_data[0] != 0x06) || (recv_data[1]!=1)) {
            //unipi_spi_trace("Write reg: %d %8ph\n", reg, recv_data);
            ret_code = 2;
        }
    }
    unipi_spi_trace("Write reg: %d ret: %d %8ph\n", reg, ret_code, recv_data);
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
                                    UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC, 0);
	if (ret_code == 0) {
        if ((recv_data[0] != 0x06) || (recv_data[1]!=2)) {
            ret_code = 2;
        }
    }
    unipi_spi_trace("Write reg32: %d ret: %d %10ph\n", reg, ret_code, recv_data);
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
                                    UNIPISPI_OP_MODE_SEND_HEADER|UNIPISPI_OP_MODE_DO_CRC, 0);
    return ret_code;
}


void neuronspi_spi_set_irqs(struct spi_device* spi_dev, u16 to)
{
    unipispi_modbus_write_register(spi_dev, 1007, to);
	unipi_spi_trace(KERN_INFO "UNIPISPI: SPI IRQ Set, CS%d, to:0x%x\n", spi_dev->chip_select, to);
}

void neuronspi_uart_flush_proc(struct kthread_work *ws)
{
	struct neuronspi_port *n_port = ((container_of((ws), struct neuronspi_port, flush_work)));
    struct neuronspi_driver_data *n_spi = n_port->n_spi;
	struct spi_device *spi = neuronspi_s_dev[n_spi->neuron_index];
    struct neuronspi_op_buffer recv_buf;
	unsigned long flags;

    unipi_spi_read_str(spi,  n_port);
    if (n_port->rx_in_progress) {
		s32 frequency = NEURONSPI_DEFAULT_FREQ;
		if (n_spi) {
			frequency = n_spi->ideal_frequency;
		}
		neuronspi_spi_send_const_op(spi, &UNIPISPI_IDLE_MESSAGE, &recv_buf, 0, frequency, 25);
	}

	spin_lock_irqsave(&n_port->port.lock, flags);
    n_port->accept_rx = 1;
    spin_unlock_irqrestore(&n_port->port.lock, flags);

}

void neuronspi_irq_proc(struct kthread_work *ws)
{
	struct neuronspi_driver_data *n_spi = ((container_of((ws), struct neuronspi_driver_data, irq_work)));
	struct spi_device *spi = neuronspi_s_dev[n_spi->neuron_index];
    struct neuronspi_op_buffer recv_buf;

	s32 frequency = NEURONSPI_DEFAULT_FREQ;
	if (n_spi) {
		frequency = n_spi->ideal_frequency;
	}
	neuronspi_spi_send_const_op(spi, &UNIPISPI_IDLE_MESSAGE, &recv_buf, 0, frequency, 25);
}


// callback of poll_timer. Schedule ->irq_work
static enum hrtimer_restart neuronspi_poll_timer_func(struct hrtimer *timer)
{
    struct neuronspi_driver_data* n_spi = ((container_of((timer), struct neuronspi_driver_data, poll_timer)));
    //struct spi_device *spi = neuronspi_s_dev [n_spi->neuron_index];
    
	unipi_spi_trace_1(KERN_INFO "UNIPISPI: nspi%d POLL IRQ\n", n_spi->neuron_index);
	kthread_queue_work(n_spi->primary_worker, &n_spi->irq_work);
    //unipi_spi_idle_op(spi);
	return HRTIMER_NORESTART;
}


irqreturn_t neuronspi_spi_irq(s32 irq, void *dev_id)
{
	// real irq handler - schedule idle op to read irq status (irq_work = neuronspi_irq_proc)
    struct spi_device *spi = (struct spi_device *)dev_id;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);

	unipi_spi_trace(KERN_INFO "UNIPISPI: nspi%d SPI IRQ\n", n_spi->neuron_index);
	kthread_queue_work(n_spi->primary_worker, &n_spi->irq_work);
    //unipi_spi_idle_op(spi);
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
        kthread_queue_work(n_spi->primary_worker, &n_spi->irq_work);
        //unipi_spi_idle_op(spi);
    }
}

#define REG1000(first_probe,reg)    ((u16)(first_probe[4+2*(reg-1000)] | (first_probe[5+2*(reg-1000)] << 8)))
#define REG1000_lo(first_probe,reg) (first_probe[4+2*(reg-1000)])
#define REG1000_hi(first_probe,reg) (first_probe[5+2*(reg-1000)])
#define lo(x) (x & 0xff)
#define hi(x) (x >> 8)
const char name_unknown[] = "UNKNOWN\0";

s32 neuronspi_spi_probe(struct spi_device *spi)
{
	//const struct neuronspi_devtype *devtype;
	struct neuronspi_driver_data *n_spi;
    struct neuronspi_op_buffer recv_op;
    u8  first_probe[UNIPISPI_PROBE_MESSAGE_LEN];
	s32 ret, i, no_irq = 0;
	u8 uart_count = 0;
    u16 hardware_model, lower_board;
    u8  upper_board = 0;
    const char *board_name = name_unknown;
    u32 probe_always_succeeds = 0;
	u32 always_create_uart = 0;
    struct kthread_worker   *worker;
	struct sched_param rt_param = { .sched_priority = MAX_RT_PRIO - 1 };

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
		//devtype = (struct neuronspi_devtype *)of_id->data;
		unipi_spi_trace(KERN_INFO "UNIPISPI: CS%d, Device Tree Node defined index=%d\n", spi->chip_select, n_spi->neuron_index);

	} else {
		//const struct spi_device_id *id_entry = spi_get_device_id(spi);
		//devtype = (struct neuronspi_devtype *)id_entry->driver_data;
	}


	// We perform an initial probe of registers 1000-1004 to identify the device, using a premade message
	neuronspi_spi_send_const_op(spi, &UNIPISPI_IDLE_MESSAGE, &recv_op, 0, NEURONSPI_DEFAULT_FREQ, 25);
	// Throw away the first message - the associated SPI Master is sometimes not properly initialised at this point

    recv_op.second_message = first_probe;

	for (i=0; i< 5; i++) {
        neuronspi_spi_send_const_op(spi, &UNIPISPI_PROBE_MESSAGE, &recv_op, UNIPISPI_PROBE_MESSAGE_LEN, NEURONSPI_DEFAULT_FREQ, 25);
		if (first_probe[0] != 0) break;
	}

    n_spi->combination_id = 0xff;
	if (first_probe[0] != 0) {
        // Board found, try to find model in table
        n_spi->firmware_version = REG1000   (first_probe,1000);
		uart_count              = REG1000_lo(first_probe,1002) & 0xf;
        hardware_model          = REG1000_hi(first_probe,1003);
        lower_board             = REG1000   (first_probe,1004) & 0xfff0;
		for (i = 0; i < NEURONSPI_BOARDTABLE_LEN; i++) {
			if (hardware_model == NEURONSPI_BOARDTABLE[i].index) {
                //if ((lower_board>>8) != NEURONSPI_BOARDTABLE[i].lower_board_id) { // strange combination  //break;
                n_spi->combination_id = i;
                upper_board = NEURONSPI_BOARDTABLE[i].upper_board_id;
                n_spi->features = &(NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->features);
                board_name = NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->combination_name;
                break;
			}
		}
        //n_spi->ideal_frequency = neuronspi_is_slower_model(lower_board) ? NEURONSPI_SLOWER_FREQ : NEURONSPI_COMMON_FREQ; 
        n_spi->ideal_frequency = neuronspi_frequency_by_model(lower_board);
        no_irq = neuronspi_is_noirq_model(lower_board);

        printk(KERN_INFO "UNIPISPI: UniPi Board %s (L:%x U:%x C:%x) at CS%d (nspi%d %dkHz) detected.\n\t\t\tFw: v%d.%d Uarts:%d, reg1001-4: %04x %04x %04x %04x\n",
				board_name, hi(lower_board), upper_board, hardware_model, spi->chip_select, n_spi->neuron_index, n_spi->ideal_frequency/1000,
                hi(n_spi->firmware_version), lo(n_spi->firmware_version), uart_count, 
                REG1000(first_probe,1001), REG1000(first_probe,1002), REG1000(first_probe,1003), REG1000(first_probe,1004));

	} else if (probe_always_succeeds) {
        // dummy board
        if (always_create_uart) uart_count = 1;
        n_spi->ideal_frequency = NEURONSPI_SLOWER_FREQ;
		no_irq = 1;
		printk(KERN_INFO "UNIPISPI: DUMMY UniPi Board at CS%d (nspi%d) assigned. Uarts:%d, uses freq. %d Hz\n",
				spi->chip_select,  n_spi->neuron_index, uart_count, n_spi->ideal_frequency);
        
    } else {
		ret = -ENODEV;
		kfree(n_spi);
		printk(KERN_INFO "UNIPISPI: Probe did not detect a valid UniPi device at CS%d\n", spi->chip_select);
		return ret;
	}

	// Set rt priority to spi controller
	//dev_info(&ctlr->dev, "will run message pump with realtime priority\n");
	if (spi->controller->kworker_task)
		sched_setscheduler(spi->controller->kworker_task, SCHED_FIFO, &rt_param);

    if (spi->controller->set_cs != unipi_spi_set_cs) {
        unipi_spi_master_set_cs = spi->controller->set_cs;
        unipi_spi_master_flag = spi->controller->flags;
        spi->controller->set_cs = unipi_spi_set_cs;
        spi->controller->flags |= SPI_MASTER_GPIO_SS;
    }
    if (gpio_is_valid(spi->cs_gpio)) {
        spi->cs_gpio = -spi->cs_gpio;
    }
        
    // Prepare worker for interrupt, LEDs, UARTs
    worker = kthread_create_worker(0, "unipispi%d", n_spi->neuron_index);
	if (IS_ERR(worker)) {
        kfree(n_spi);
		return PTR_ERR(worker);
    }
	sched_setscheduler(worker->task, SCHED_FIFO, &neuronspi_sched_param);
    n_spi->primary_worker = worker;
    
    // Prepare Register map
	n_spi->reg_map = regmap_init(&(spi->dev), &neuronspi_regmap_bus, spi, &neuronspi_regmap_config_default);
	spin_lock_init(&n_spi->sysfs_regmap_lock);
	if (n_spi->features) {
		n_spi->regstart_table = kzalloc(sizeof(struct neuronspi_board_regstart_table), 1);
		neuronspi_create_reg_starts(n_spi->regstart_table, NEURONSPI_BOARDTABLE[n_spi->combination_id].definition);
	} else {
		n_spi->regstart_table = NULL;
	}

    // Save spi device into global array
	spin_lock_irqsave(neuronspi_probe_spinlock, flags);
	spi_set_drvdata(spi, n_spi);
	neuronspi_s_dev[n_spi->neuron_index] = spi;

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
    // Add platform iogroup_x and LEDs, GPIOs, IIOs
    n_spi->board_device = neuronspi_board_device_probe(n_spi);

 
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

    kthread_init_work(&(n_spi->irq_work), neuronspi_irq_proc); // prepare work function for interrupt status checking
	if (!no_irq) {
		n_spi->no_irq = 0;
		//ret = devm_request_irq(&(spi->dev), spi->irq, neuronspi_spi_irq, 0x81, dev_name(&(spi->dev)), spi);
		//ret = devm_request_irq(&(spi->dev), spi->irq, neuronspi_spi_irq, IRQF_TRIGGER_HIGH, dev_name(&(spi->dev)), spi);
        // load setting of EDGE/LEVEL from devicetree
		ret = devm_request_irq(&(spi->dev), spi->irq, neuronspi_spi_irq, 0, dev_name(&(spi->dev)), spi);
		unipi_spi_trace(KERN_DEBUG "UNIPISPI: SPI IRQ %d registration: ret=%d\n", spi->irq, ret);
        no_irq = (ret !=0);
        ret = 0;
    }
	if (no_irq) {
        n_spi->no_irq = 1;
        hrtimer_init(&n_spi->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        n_spi->poll_timer.function = neuronspi_poll_timer_func;
        unipi_spi_trace(KERN_DEBUG "UNIPISPI: NO IRQ ON THIS MODEL !!\n");
	}

	return ret;
}

s32 neuronspi_spi_remove(struct spi_device *spi)
{
    int neuron_index;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);

    if (n_spi) {
        if ((spi->cs_gpio < 0) && (spi->cs_gpio != -ENOENT)) { spi->cs_gpio = -spi->cs_gpio; }
        if (spi->controller->set_cs == unipi_spi_set_cs) {
            spi->controller->set_cs = unipi_spi_master_set_cs;
            if (unipi_spi_master_flag & SPI_MASTER_GPIO_SS) {
                spi->controller->flags |= SPI_MASTER_GPIO_SS;
            } else {
                spi->controller->flags &= ~SPI_MASTER_GPIO_SS;
            }
        }

        neuron_index = n_spi->neuron_index; 
		if (n_spi->no_irq) {
            hrtimer_cancel(&n_spi->poll_timer);
        } else {
            devm_free_irq(&(spi->dev), spi->irq, spi);             
        }
        neuronspi_uart_remove(spi);

		if (n_spi->board_device) {
            neuronspi_board_device_remove(n_spi->board_device);
		}
        if (n_spi->reg_map) 
            regmap_exit(n_spi->reg_map); 
        if (n_spi->regstart_table)
            kfree(n_spi->regstart_table);

        kthread_destroy_worker(n_spi->primary_worker);
        // clear global array item
        neuronspi_s_dev[neuron_index] = NULL; 
		kfree(n_spi);
        printk(KERN_INFO "UNIPISPI: UniPi Board nspi%d removed\n", neuron_index);
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


int char_register_driver(void)
{
    int major;
    struct device *parent = NULL;
	if (neuronspi_cdrv.major_number >= 0) return 0;

	// Character device registration
	unipi_spi_trace(KERN_DEBUG "UNIPISPI: CDEV Initialising Character Device\n");

	major = register_chrdev(0, NEURON_DEVICE_NAME, &file_ops);
	if (major < 0){
	   printk(KERN_ALERT "NEURONSPI: CDEV Failed to register chrdev\n");
	   return major;
	}
	unipi_spi_trace_1(KERN_DEBUG "UNIPISPI: CDEV major number %d\n", major);

	// Character class registration
	neuronspi_cdrv.driver_class = class_create(THIS_MODULE, NEURON_DEVICE_CLASS);
	if (IS_ERR(neuronspi_cdrv.driver_class)) {
		unregister_chrdev(major, NEURON_DEVICE_NAME);
		printk(KERN_ALERT "NEURONSPI: CDEV Failed to register device class\n");
		return PTR_ERR(neuronspi_cdrv.driver_class);
	}
	unipi_spi_trace_1(KERN_DEBUG "UNIPISPI: CDEV Device class registered\n");

	// Device driver registration
	/*neuronspi_cdrv.dev = device_create_with_groups(neuronspi_cdrv.driver_class, &(neuron_plc_dev->dev), \
                            MKDEV(major, 0), NULL, neuron_plc_attr_groups, NEURON_DEVICE_NAME);*/

    if (neuron_plc_dev != NULL) parent = &(neuron_plc_dev->dev);
	neuronspi_cdrv.dev = device_create(neuronspi_cdrv.driver_class, parent, MKDEV(major, 0), \
                            neuron_plc_dev, NEURON_DEVICE_NAME);
	if (IS_ERR(neuronspi_cdrv.dev)) {
		class_destroy(neuronspi_cdrv.driver_class);
        unregister_chrdev(major, NEURON_DEVICE_NAME);
        printk(KERN_ALERT "NEURONSPI: CDEV Failed to create the device\n");
        return PTR_ERR(neuronspi_cdrv.dev);
	}
	printk(KERN_DEBUG "UNIPISPI: ModBus/SPI interface /dev/%s (%d:0) created.\n", NEURON_DEVICE_NAME, major);

    neuronspi_cdrv.major_number = major;
	return 0;
}

void char_unregister_driver(void)
{
    if (neuronspi_cdrv.major_number < 0) return;

	device_destroy(neuronspi_cdrv.driver_class, MKDEV(neuronspi_cdrv.major_number, 0));     // Destroy the device
	class_destroy(neuronspi_cdrv.driver_class);                             				// Destroy the class
	unregister_chrdev(neuronspi_cdrv.major_number, NEURON_DEVICE_NAME);             		// Unregister the major number
	unipi_spi_trace(KERN_INFO "UNIPISPI: CDEV unloaded\n");
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
	spin_lock_init(&unipi_spi_master_lock);
	//mutex_init(&neuronspi_master_mutex);
	mutex_init(&unipi_inv_speed_mutex);

    loop_per_us = ((loops_per_jiffy) * HZ )/ 1000000;
    if (loop_per_us == 0) loop_per_us = 1;

    // clear global neuron spi devices list
	memset(&neuronspi_s_dev, 0, sizeof(neuronspi_s_dev));
	ret = spi_register_driver(&neuronspi_spi_driver);

	if (ret < 0) {
		printk(KERN_ERR "UNIPISPI: Failed to init spi driver --> %d\n", ret);
		return ret;
	}
	printk(KERN_INFO "UNIPISPI: SPI Driver Registered, Major Version: %s\n", NEURONSPI_MAJOR_VERSIONSTRING);

    neuronspi_invalidate_thread = kthread_run(neuronspi_regmap_invalidate, NULL, "unipispi_inv");
    if (IS_ERR(neuronspi_invalidate_thread)) {
        neuronspi_invalidate_thread = NULL;
    }

	char_register_driver();

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
        neuronspi_invalidate_thread = NULL;
	}
	char_unregister_driver();
    neuronspi_uart_driver_exit();
    unipi_tty_exit();
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
