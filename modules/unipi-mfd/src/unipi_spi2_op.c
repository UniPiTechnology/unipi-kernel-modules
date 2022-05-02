/*
 * Unipi PLC spi device driver - Copyright (C) 2018 Unipi Technology
 * 
 * Author: Tomas Knot <tomasknot@gmail.com>
 * Author: Miroslav Ondra <ondra@faster.cz>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

/************
 * Includes *
 ************/
#include <linux/completion.h>
#include <linux/cpufreq.h>
#include <linux/version.h>
#include <uapi/linux/sched/types.h>

#include "unipi_common.h"
#include "unipi_modbus.h"
#include "unipi_spi.h"
#include "unipi_spi_crc.h"


/****************************************************************/

#define roundup_block(count, blocksize) (1+((count)-1)/(blocksize))
#define UNIPI_SPI2_MESSAGE_LEN 16
#define UNIPI_SPI2_MESSAGE_ALLOC UNIPI_SPI2_MESSAGE_LEN + 4

struct unipi_spi2_context
{
	struct spi_message message;
	void * operation_callback_data;
	OperationCallback operation_callback;
	int len;
	u8 tx_header[UNIPI_SPI2_MESSAGE_ALLOC];
	u8 rx_header[UNIPI_SPI2_MESSAGE_ALLOC];
	//u8 tx_header2[UNIPI_MODBUS_HEADER_SIZE+2*2+4];
	//u8 rx_header2[UNIPI_MODBUS_HEADER_SIZE+2*2+4];
	int simple_read;
	//int simple_len;
	u8* data;
	u16 packet_crc;
};

static int unipi_spi2_zip_reg(unsigned int reg)
{
	if (reg < 64) return reg;
	if (reg < 500) return -EIO;
	if (reg < 564) return 0x40 |  ((reg-500) & 0x3f);
	if (reg < 1000) return -EIO;
	if (reg < 1064) return  0x80 |  ((reg-1000) & 0x3f);
	return -EIO;
}

/* bottom part of all async op */
int unipi_spi2_exec_context(struct spi_device* spi_dev, struct unipi_spi2_context *context,
                           void* cb_data, OperationCallback cb_function);

/* upper part of all async opertations
 * - create context with space for 2+add_trans transactions
 * - setup data of first part */
struct unipi_spi2_context *unipi_spi2_alloc_context(struct spi_device* spi_dev, enum UNIPI_MODBUS_OP op, 
                                                   unsigned int len, unsigned int reg/*, int add_trans*/)
{
	int trans_count, freq;
	struct unipi_spi2_context *context;
	struct spi_transfer * s_trans;
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);

	trans_count = 1; //2 + add_trans;
	context = kzalloc(sizeof(struct unipi_spi2_context) + trans_count * sizeof(struct spi_transfer), GFP_ATOMIC);
	if (! context) {
		return NULL;
	}
	freq = n_spi->frequency;

	context->tx_header[0] = op;
	context->tx_header[1] = len;
	context->tx_header[2] = lo(reg);
	context->tx_header[3] = hi(reg);

	s_trans = (struct spi_transfer *)(context + 1);
	spi_message_init_with_transfers(&context->message, s_trans, trans_count);

	s_trans[0].bits_per_word = UNIPI_SPI_B_PER_WORD;
	s_trans[0].speed_hz = freq;
	s_trans[0].len = UNIPI_SPI2_MESSAGE_LEN+2;
	s_trans[0].tx_buf = context->tx_header;
	s_trans[0].rx_buf = context->rx_header;
	unipi_spi_trace_1(spi_dev, "Write(op1) %6ph\n", context->tx_header);

	context->message.context = context;
	context->message.spi = spi_dev;
	return context;
}


/*  Async op for WRITEREGS, WRITEBITS using own buffer 
 * 		data buffer can be unstable (on-stack)
 * 		regcount = 1..2   pro WRITEREGS
 *		         = 1..32  pro WRITEBITS
 */ 
int unipi_spi2_write_simple(struct spi_device* spi_dev, enum UNIPI_MODBUS_OP op,
                           unsigned int reg, unsigned int count, u8* data,
                           void* cb_data, OperationCallback cb_function)
{
	//int len;
	struct unipi_spi2_context *context;
	//struct spi_transfer * s_trans;

	//len  = ((op==UNIPI_MODBUS_OP_WRITEREG)? count : roundup_block(count, 16)) * 2;
	//len += UNIPI_MODBUS_HEADER_SIZE;
	if (count>4) { return -EIO; } 

	context = unipi_spi2_alloc_context(spi_dev, op, count, reg);
	if (context == NULL) return -ENOMEM;

	memmove(context->tx_header+UNIPI_MODBUS_HEADER_SIZE, data,\
	        (op==UNIPI_MODBUS_OP_WRITEREG)? count*2 : roundup_block(count, 8)); // !!!!!
	context->packet_crc = unipi_spi_crc_set(context->tx_header, UNIPI_SPI2_MESSAGE_LEN, 0);

	unipi_spi_trace_1(spi_dev, "B %8phC count=%d\n", context->tx_header, count);

	context->len = count;
	return unipi_spi2_exec_context(spi_dev, context, cb_data, cb_function);
}

/*  Async op for READREG, READBITS using own data buffer 
 * 		regcount = 1..2   pro READREG
 *		         = 1..32  pro READBIT
 */ 
int unipi_spi2_read_simple(struct spi_device* spi_dev, enum UNIPI_MODBUS_OP op,
                           unsigned int reg, unsigned int count, u8 *data,
                           void* cb_data, OperationCallback cb_function)
{
	//int len;
	//int reg2;
	struct unipi_spi2_context *context;
	//struct spi_transfer * s_trans;

	if (count>4) { return -EIO; } 

	context = unipi_spi2_alloc_context(spi_dev, op, unipi_spi2_zip_reg(reg), reg);
	if (context == NULL) return -ENOMEM;

	context->packet_crc = unipi_spi_crc_set(context->tx_header, UNIPI_SPI2_MESSAGE_LEN, 0);

	context->simple_read = 1;
	context->len = count;
	context->data = data;
	return unipi_spi2_exec_context(spi_dev, context, cb_data, cb_function);
}


int unipi_spi2_write_str_async(void* spi, unsigned int port, u8* data, unsigned int count, 
                              void* cb_data, OperationCallback cb_function)
{
	return -EIO;

/*
	int len, data_trans, i, remain;
	struct unipi_spi_context *context;
	struct spi_transfer * s_trans;
	struct spi_device* spi_dev = (struct spi_device*) spi;

	if (count == 1) {
		context = unipi_spi_alloc_context(spi_dev, UNIPI_MODBUS_OP_WRITECHAR, data[0], port << 8, 0);
		if (context==NULL)
			return -ENOMEM;
		return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
	}

	len = (count & 1) ? count+1 : count;
	data_trans = roundup_block(len+UNIPI_SPI_CRC_LENGTH, NEURONSPI_MAX_TX);
	context = unipi_spi_alloc_context(spi_dev, UNIPI_MODBUS_OP_WRITESTR, (count & 0xff), port << 8, data_trans);
	if (context == NULL) return -ENOMEM;

	context->packet_crc = unipi_spi_crc_set(data, len, context->packet_crc);

	s_trans = (struct spi_transfer *)(context + 1);
	s_trans[1].delay.value = 25;
	s_trans[1].delay.unit = SPI_DELAY_UNIT_USECS;

	remain = len + UNIPI_SPI_CRC_LENGTH;
	for ( i=0; i < data_trans; i++) {
		s_trans[2+i].bits_per_word = UNIPI_SPI_B_PER_WORD;
		s_trans[2+i].speed_hz = s_trans[1].speed_hz;
		s_trans[2+i].tx_buf = data + (NEURONSPI_MAX_TX * i );
		s_trans[2+i].len = (remain > NEURONSPI_MAX_TX) ? NEURONSPI_MAX_TX : remain;
		remain -= NEURONSPI_MAX_TX;
	}

	context->len = len;
	return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
*/
}

int unipi_spi2_read_str_async(void* spi, unsigned int port, u8* data, unsigned int count, 
                             void* cb_data, OperationCallback cb_function)
{
	return -EIO;
/*
	int len, data_trans, i, remain;
	struct unipi_spi_context *context;
	struct spi_transfer * s_trans;
	struct spi_device* spi_dev = (struct spi_device*) spi;

	len = count;
	if (len < 246) {
		len += 4;
		len = (len & 1 ? len+1 : len);  // transmit_length must be even
	} else {
		len = 250;
	}

	data_trans = roundup_block(len+UNIPI_SPI_CRC_LENGTH, NEURONSPI_MAX_TX);
	context = unipi_spi_alloc_context(spi_dev, UNIPI_MODBUS_OP_READSTR, len, port <<8, 1+data_trans);
	if (context == NULL) return -ENOMEM;

	s_trans[1].delay.value = 25;
	s_trans[1].delay.unit = SPI_DELAY_UNIT_USECS;

	s_trans[2].bits_per_word = UNIPI_SPI_B_PER_WORD;
	s_trans[2].speed_hz = s_trans[1].speed_hz;
	s_trans[2].len = UNIPI_MODBUS_HEADER_SIZE;
	s_trans[2].tx_buf = NULL;
	s_trans[2].rx_buf = context->rx_header2;

	remain = len + UNIPI_SPI_CRC_LENGTH;
	for ( i=0; i < data_trans; i++) {
		s_trans[i+3].bits_per_word = UNIPI_SPI_B_PER_WORD;
		s_trans[i+3].speed_hz = s_trans[1].speed_hz;
		s_trans[i+3].rx_buf = data + (NEURONSPI_MAX_TX * i);
		s_trans[i+3].len = (remain > NEURONSPI_MAX_TX) ? NEURONSPI_MAX_TX : remain;
		remain -= NEURONSPI_MAX_TX;
	}

	context->len = len;
	context->data = data;
	return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
*/
}


/**************************************************************
 * 
 * Asynchronous operations
 * 
 * ***********************************************************/

struct unipi_spi_devstatus {
	enum UNIPI_MODBUS_OP opcode;
	unsigned int interrupt;
	uint8_t port;
	int remain;
	uint8_t ch;
};

struct unipi_spi_reply {
	enum UNIPI_MODBUS_OP opcode;
	int len;
	unsigned int reg;
	uint8_t *data;
};

int unipi_spi2_parse_frame(struct unipi_spi2_context* context, struct unipi_spi_devstatus *devstatus, struct unipi_spi_reply *reply)
{
	u16 recv_crc;
	struct unipi_spi_device *n_spi;
	unsigned int reply_max_count, reg;

	//unipi_spi_trace_1(spi, "Read (op1) %8ph\n", context->rx_header);
	// check crc on header
	recv_crc = unipi_spi_crc(context->rx_header, UNIPI_SPI2_MESSAGE_LEN, 0);
	context->packet_crc = *((u16*)(context->rx_header+UNIPI_SPI2_MESSAGE_LEN));
	if (recv_crc != context->packet_crc) {
		reply->len=0;
		reply->opcode=0;
		unipi_spi_error(context->message.spi, "Incorrect CRC (Received: %04x Calculated: %04x)\n", context->packet_crc, recv_crc);
		unipi_spi_error(context->message.spi, "     %8phC\n", context->rx_header);
		n_spi = spi_get_drvdata(context->message.spi);
		n_spi->stat.errors_crc1++;
		return -1;
	}
	devstatus->opcode = context->rx_header[0];
	devstatus->interrupt = context->rx_header[1];

	reply->opcode = context->tx_header[0];
	reply_max_count = context->rx_header[7];
	reg = context->tx_header[1]&0x3f;
	if (reply_max_count == 0xff)
		reply->len = -EIO;
	else if (reg >= reply_max_count)
		reply->len = 0;
	else if (reg + context->len > reply_max_count)
		reply->len = reply_max_count - reg;
	else
		reply->len = context->len;
	reply->data = context->rx_header+8;
	return 0;
}


static void unipi_spi2_op_complete(void *arg)
{
//	int remain, ret = 0;

	struct unipi_spi2_context* context = (struct unipi_spi2_context*) arg;
	struct spi_device* spi = context->message.spi;
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	struct unipi_spi_devstatus devstatus;
	struct unipi_spi_reply reply;
	int len = context->len;

	// schedule timer to unblock busy flag
	hrtimer_start_range_ns(&n_spi->frame_timer, UNIPI_SPI_INTER_FRAME_NS, 10000, HRTIMER_MODE_REL);

	unipi_spi_trace_1(spi, "Read (op1) st=%d %16ph\n", context->message.status, context->rx_header);
	if ((context->message.status == 0)
		&& (unipi_spi2_parse_frame(context, &devstatus, &reply) == 0)){
		// process valid rx_header with out-of-order data
		n_spi->stat.bytes += len + UNIPI_SPI_MESSAGE_HEADER_LENGTH;
		switch (devstatus.opcode) {
			case UNIPI_MODBUS_OP_WRITECHAR:
				n_spi->stat.messages_prio++;
				// Signal the UART about received char
				unipi_mfd_rx_char(&n_spi->mfd, devstatus.port, devstatus.ch, devstatus.remain);
				fallthrough;
			case UNIPI_MODBUS_OP_IDLE:
				if (devstatus.interrupt != 0)
					unipi_mfd_int_status(&n_spi->mfd, devstatus.interrupt);
				break;
			default:
				n_spi->stat.errors_opcode1++;
				break;
		}
		if (context->operation_callback) {
			if (reply.len > 0) {
				if ((context->simple_read) && (context->data))
					memmove(context->data, reply.data, reply.len*2);
				//else
				//	context->data = reply->data;

//			} else if (context->tx_header[0] == UNIPI_MODBUS_OP_READSTR) {
//				/* combine len + remain */
//				ret = ret | (context->rx_header2[3] << 8);
//			}
//			} else {
//				ret = -EIO;
			}
			context->operation_callback(context->operation_callback_data, reply.len, reply.data);
		}
	} else {
		// sending unsuccessfull - reason unknown
		n_spi->stat.errors_tx++;
		unipi_spi_trace(spi, "Unsuccessful spi transaction: opcode:%d\n", context->tx_header[0]);
		if (context->operation_callback)
			context->operation_callback(context->operation_callback_data, -EIO, context->data);
	}
	kfree(context);
}


/* bottom part of all async op */

int unipi_spi2_exec_context(struct spi_device* spi_dev, struct unipi_spi2_context *context,
                           void* cb_data, OperationCallback cb_function)
{
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);
	unsigned long flags, bflags;
	int ret;

	context->operation_callback_data = cb_data;
	context->operation_callback = cb_function;
	context->message.complete = unipi_spi2_op_complete;

	spin_lock_irqsave(&n_spi->firmware_lock, flags);
	if (n_spi->firmware_in_progress) {
		spin_unlock_irqrestore(&n_spi->firmware_lock, flags);
		kfree(context);
		return -EACCES;
	}
	n_spi->stat.messages++;
	spin_lock_irqsave(&n_spi->busy_lock, bflags);
	if (n_spi->busy) {
		list_add_tail(&context->message.queue, &n_spi->queue);
		spin_unlock_irqrestore(&n_spi->busy_lock, bflags);
		spin_unlock_irqrestore(&n_spi->firmware_lock, flags);
		return 0;
	}
	n_spi->busy = 1;
	spin_unlock_irqrestore(&n_spi->busy_lock, bflags);

	ret = spi_async(spi_dev, &context->message);
	spin_unlock_irqrestore(&n_spi->firmware_lock, flags);
	if (ret != 0) {
		n_spi->stat.errors_tx++;
		unipi_spi_trace(spi_dev, "Err=3 txopcode:%d\n", context->tx_header[0]);
		kfree(context);
	}
	return ret;
}

/*  Async op for WRITEBIT */
int unipi_spi2_write_bits_async(struct spi_device* spi_dev, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function)
{
	struct unipi_spi2_context *context;
	if (count==0 || count > 32) return -EINVAL;

	if (count == 1) {
		context = unipi_spi2_alloc_context(spi_dev, UNIPI_MODBUS_OP_WRITEBIT, data[0], reg);
		if (context==NULL)
			return -ENOMEM;
		return unipi_spi2_exec_context(spi_dev, context, cb_data, cb_function);
	}
	return unipi_spi2_write_simple(spi_dev, UNIPI_MODBUS_OP_WRITEBITS, reg, count, data,
	                              cb_data, cb_function);
}

/*  Async op for READBIT */
int unipi_spi2_read_bits_async(struct spi_device* spi_dev, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function)
{
	if (count==0 || count > 32) return -EINVAL;

	return unipi_spi2_read_simple(spi_dev, UNIPI_MODBUS_OP_READBIT, reg, count, data,
	                             cb_data, cb_function);
}

 
int unipi_spi2_idle_op_async(void* self, void* cb_data, OperationCallback cb_function)
{
	struct spi_device* spi_dev = (struct spi_device*) self;
	struct unipi_spi2_context *context;
	context = unipi_spi2_alloc_context(spi_dev, UNIPI_MODBUS_OP_IDLE, 0, 0x0e55);
	if (context==NULL)
		return -ENOMEM;
	return unipi_spi2_exec_context(spi_dev, context, cb_data, cb_function);
} 

/**************************************************************
 * 
 * Synchronous operations
 * 
 * ***********************************************************/
struct unipi_spi_sync_cb_data
{
	struct completion* done;
	int		result;
};

void unipi_spi2_sync_op_callback(void* cb_data, int result, u8* recv)
{
	struct unipi_spi_sync_cb_data* unipi_cb_data = (struct unipi_spi_sync_cb_data*) cb_data;
	if (cb_data) {
		unipi_cb_data->result = result;
		complete(unipi_cb_data->done);
	}
}

int unipi_spi2_idle_op(struct spi_device* spi_dev)
{
	int ret = 0;
	struct unipi_spi_sync_cb_data cb_data;
	DECLARE_COMPLETION_ONSTACK(done); //struct completion done;

	init_completion(&done);
	cb_data.done = &done;
	ret = unipi_spi2_idle_op_async(spi_dev, &cb_data, unipi_spi2_sync_op_callback);
	if (ret != 0)
		return ret;
	wait_for_completion(&done);
	return cb_data.result;
}
/*
int unipi_spi2_regs_op(struct spi_device* spi_dev, enum UNIPI_MODBUS_OP op, unsigned int reg, unsigned int count, u8* data)
{
	int ret = 0;
	struct unipi_spi_sync_cb_data cb_data;
	DECLARE_COMPLETION_ONSTACK(done); //struct completion done;

	init_completion(&done);
	cb_data.done = &done;
	switch (op) {
		case UNIPI_MODBUS_OP_READREG:
			ret = unipi_spi_read_regs_async(spi_dev, reg, count, data, &cb_data, unipi_spi_sync_op_callback);
			break;
		case UNIPI_MODBUS_OP_WRITEREG:
			ret = unipi_spi_write_regs_async(spi_dev, reg, count, data, &cb_data, unipi_spi_sync_op_callback);
			break;
		case UNIPI_MODBUS_OP_READBIT:
			ret = unipi_spi_read_bits_async(spi_dev, reg, count, data, &cb_data, unipi_spi_sync_op_callback);
			break;
		case UNIPI_MODBUS_OP_WRITEBIT:
		case UNIPI_MODBUS_OP_WRITEBITS:
			ret = unipi_spi_write_bits_async(spi_dev, reg, count, data, &cb_data, unipi_spi_sync_op_callback);
			break;
		default:
			ret = -ENOENT;
	}
	if (ret != 0)
		return ret;
	wait_for_completion(&done);
	return cb_data.result;
}
*/
