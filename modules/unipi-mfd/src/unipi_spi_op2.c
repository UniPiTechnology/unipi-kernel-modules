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

static int unipi_spi2_zip_reg(enum UNIPI_SPI_OP op, unsigned int reg)
{
	int ret = 0x20;
	if ((op == UNIPI_SPI_OP_WRITEREG) || (op == UNIPI_SPI_OP_WRITEBITS)) ret |= 0x10;

	if ((op == UNIPI_SPI_OP_WRITEREG) || (op == UNIPI_SPI_OP_READREG)) {
		if (reg < 64) return ret | (reg << 8);
		if (reg < 500) return -EIO;
		if (reg < 564) return ret | 0x1 | (((reg-500) & 0x3f) << 8);
		if (reg < 1000) return -EIO;
		if (reg < 1064) return  ret | 0x2 |  (((reg-1000) & 0x3f) << 8);
	} else {
		if (reg < 64) return ret | 0x4 | (reg << 8);
		if (reg < 1000) return -EIO;
		if (reg < 1016) return  ret | 0x5 |  (((reg-1000) & 0x3f) << 8);
		if (reg < 1080) return  ret | 0x6 |  (((reg-1016) & 0x3f) << 8);
	}
	return -EIO;
}


int unipi_spi2_parse_frame(struct unipi_spi_context* context, struct unipi_spi_devstatus *devstatus, struct unipi_spi_reply *reply)
{
	u16 recv_crc;
	u8* reply_data;
//	struct unipi_spi_device *n_spi;
	unsigned int reply_max_count, reg;

	unipi_spi_trace_1(context->message.spi, "RESP(rqop=%02x)   %6phC %8phC\n", context->tx_header[0], context->rx_header, context->rx_header+6);
	// check crc on header
	recv_crc = unipi_spi_crc(context->rx_header, UNIPI_SPI2_MESSAGE_LEN, 0);
	context->packet_crc = *((u16*)(context->rx_header+UNIPI_SPI2_MESSAGE_LEN));
	if (recv_crc != context->packet_crc) {
		if (((context->rx_header[0] == UNIPI_SPI_OP_IDLE) && (context->rx_header[2] == 0x55))
		   ||(context->rx_header[0] == UNIPI_SPI_OP_WRITECHAR)){
			// probable reply use protocol version 1
			reply->ret = -ENOEXEC;
		}
		return -EIO;
	}
	devstatus->opcode = context->rx_header[0];
	//if (devstatus->opcode == UNIPI_SPI_OP2_IDLE) devstatus->opcode = UNIPI_SPI_OP_IDLE;
	devstatus->interrupt = context->rx_header[3];

	reg = context->tx_header[1]; //&0x3f;
	reply->opcode = context->tx_header[4];
	//reply->data = context->rx_header + ((reply->opcode==UNIPI_SPI_OP_READBIT)? 12 : 8);

	reply_max_count = context->rx_header[7];
	if (reply_max_count == 0xff)
		reply->ret = -EAGAIN;
	else if (reg >= reply_max_count)
		reply->ret = 0;
	else if (reg + context->len > reply_max_count)
		reply->ret = reply_max_count - reg;
	else
		reply->ret = context->len;

	//if (context->operation_callback) {
	if ((reply->ret > 0) && (context->simple_read) && (context->data)) {
		reply_data = context->rx_header + ((reply->opcode==UNIPI_SPI_OP_READBIT)? 12 : 8);
		memmove(context->data, reply_data, context->simple_len);
	}
	return 0;
}

/* upper part of all async opertations
 * - create context with space for 2+add_trans transactions
 * - setup data of first part */
struct unipi_spi_context *unipi_spi2_alloc_context(struct spi_device* spi_dev, enum UNIPI_SPI_OP op, uint16_t fastop)
                                                   //unsigned int len)
{
	int trans_count, freq;
	struct unipi_spi_context *context;
	struct spi_transfer * s_trans;
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);

	trans_count = 1;
	context = kzalloc(sizeof(struct unipi_spi_context) + trans_count * sizeof(struct spi_transfer), GFP_ATOMIC);
	if (! context) {
		return NULL;
	}
	freq = n_spi->frequency;

	context->parse_frame = unipi_spi2_parse_frame;
	context->tx_header[0] = fastop & 0xff;
	context->tx_header[1] = fastop >> 8;
	context->tx_header[4] = op;

	s_trans = (struct spi_transfer *)(context + 1);
	spi_message_init_with_transfers(&context->message, s_trans, trans_count);

	s_trans[0].bits_per_word = UNIPI_SPI_B_PER_WORD;
	s_trans[0].speed_hz = freq;
	s_trans[0].len = UNIPI_SPI2_MESSAGE_LEN+2;
	s_trans[0].tx_buf = context->tx_header;
	s_trans[0].rx_buf = context->rx_header;

	context->message.context = context;
	context->message.spi = spi_dev;
	return context;
}


/**************************************************************
 * 
 * Asynchronous operations
 * 
 * ***********************************************************/

/*  Async op for WRITEREGS, WRITEBITS using own buffer 
 * 		data buffer can be unstable (on-stack)
 * 		regcount = 1..2   pro WRITEREGS
 *		         = 1..32  pro WRITEBITS
 */ 
int unipi_spi2_write_simple(struct spi_device* spi_dev, enum UNIPI_SPI_OP op,
                           unsigned int reg, unsigned int count, u8* data,
                           void* cb_data, OperationCallback cb_function)
{
	struct unipi_spi_context *context;
	int fastop;

	//if (count>4) { return -EIO; } 
	fastop = unipi_spi2_zip_reg(op, reg);
	if (fastop < 0) return fastop;

	context = unipi_spi2_alloc_context(spi_dev, op, fastop);
	if (context == NULL) return -ENOMEM;

	context->tx_header[5] = count;
	context->tx_header[6] = lo(reg);
	context->tx_header[7] = hi(reg);

	memmove(context->tx_header+8, data,\
	        (op==UNIPI_SPI_OP_WRITEREG)? count*2 : roundup_block(count, 8)); // !!!!!
	unipi_spi_crc_set(context->tx_header, UNIPI_SPI2_MESSAGE_LEN, 0);

	unipi_spi_trace_1(spi_dev, "REQ (wr=%4d/%d) %6phC %8phC\n", reg, count, context->tx_header, context->tx_header+6);

	context->len = count;
	context->interframe_nsec = UNIPI_SPI_INTER_FRAME_NS;
	return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
}

/*  Async op for READREG, READBITS using own data buffer 
 * 		regcount = 1..2   pro READREG
 *		         = 1..32  pro READBIT
 */ 
int unipi_spi2_read_simple(struct spi_device* spi_dev, enum UNIPI_SPI_OP op,
                           unsigned int reg, unsigned int count, u8 *data,
                           void* cb_data, OperationCallback cb_function)
{
	struct unipi_spi_context *context;
	int fastop;
	//if (count>4) { return -EIO; } 

	fastop = unipi_spi2_zip_reg(op, reg);
	if (fastop < 0) return fastop;

	context = unipi_spi2_alloc_context(spi_dev, op, fastop);
	if (context == NULL) return -ENOMEM;

	unipi_spi_crc_set(context->tx_header, UNIPI_SPI2_MESSAGE_LEN, 0);

	context->simple_read = 1;
	context->simple_len = (op==UNIPI_SPI_OP_READBIT)? roundup_block(count, 8) : count*2;
	context->len = count;
	context->data = data;
	unipi_spi_trace_1(spi_dev, "REQ (rr=%4d/%d) %6phC\n", reg, count, context->tx_header);
	return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
}

struct cb_multi {
	struct spi_device* spi_dev;
	OperationCallback cb_function;
	void *cb_data;
	unsigned int reg;
	unsigned int count;
	u8 *data;
	enum UNIPI_SPI_OP op;
	int (*func_simple)(struct spi_device*, enum UNIPI_SPI_OP, unsigned int, unsigned int, u8*,
                           void* cb_data, OperationCallback cb_function);
	int ret;
};

static void multi_callback(void* cb_data, int result)
{
	struct cb_multi *cb_multi = cb_data;
	int ret;
	if (result == -EAGAIN) {
		goto next;
	}
	if (result < 0) {
		cb_multi->ret = result;
		goto finish;;
	} 
	cb_multi->ret += result;
	if (result != 4 || result == cb_multi->count) {
		goto finish;;
	}
	cb_multi->reg += 4;
	cb_multi->data += 4*2;
	cb_multi->count -= 4;
next:
	ret = cb_multi->func_simple(cb_multi->spi_dev, cb_multi->op, cb_multi->reg,
	              min((cb_multi->count), (unsigned int)4), cb_multi->data, cb_multi, multi_callback);
	if (ret != 0) {
		cb_multi->ret = ret;
		goto finish;
	}
	return;
finish:
	if (cb_multi->cb_function)
		cb_multi->cb_function(cb_multi->cb_data, cb_multi->ret);
	kfree(cb_multi);
}

int unipi_spi2_read_regs_async(void* self, unsigned int reg, unsigned int count, u8 *data,
                           void* cb_data, OperationCallback cb_function)
{
	struct spi_device* spi_dev = (struct spi_device*) self;
	struct cb_multi *cb_multi;

//	if (count <= 4) 
//		return unipi_spi2_read_simple(spi_dev, UNIPI_SPI_OP_READREG, reg, count, data, cb_data, cb_function);

	cb_multi = kzalloc(sizeof(struct cb_multi), GFP_ATOMIC);
	if (cb_multi == NULL) return -ENOMEM;
	cb_multi->spi_dev = spi_dev;
	cb_multi->cb_data = cb_data;
	cb_multi->cb_function = cb_function;
	cb_multi->reg = reg;
	cb_multi->count = count;
	cb_multi->data = data;
	cb_multi->ret = 0;
	cb_multi->op = UNIPI_SPI_OP_READREG;
	cb_multi->func_simple = unipi_spi2_read_simple;
	return unipi_spi2_read_simple(spi_dev, UNIPI_SPI_OP_READREG, reg, min((cb_multi->count), (unsigned int)4), data, cb_multi, multi_callback);
}


int unipi_spi2_write_regs_async(void* self, unsigned int reg, unsigned int count, u8 *data,
                           void* cb_data, OperationCallback cb_function)
{
	struct spi_device* spi_dev = (struct spi_device*) self;
	struct cb_multi *cb_multi;

	//if (count <= 4)
	//	return unipi_spi2_write_simple(spi_dev, UNIPI_SPI_OP_WRITEREG, reg, count, data, cb_data, cb_function);

	cb_multi = kzalloc(sizeof(struct cb_multi), GFP_ATOMIC);
	if (cb_multi == NULL) return -ENOMEM;
	cb_multi->spi_dev = spi_dev;
	cb_multi->cb_data = cb_data;
	cb_multi->cb_function = cb_function;
	cb_multi->reg = reg;
	cb_multi->count = count;
	cb_multi->data = data;
	cb_multi->ret = 0;
	cb_multi->op = UNIPI_SPI_OP_WRITEREG;
	cb_multi->func_simple = unipi_spi2_write_simple;
	return unipi_spi2_write_simple(spi_dev, UNIPI_SPI_OP_WRITEREG, reg, min((cb_multi->count), (unsigned int)4), data, cb_multi, multi_callback);
}

int unipi_spi2_write_str_async(void* self, unsigned int port, u8* data, unsigned int count, 
                              void* cb_data, OperationCallback cb_function)
{
	struct spi_device* spi_dev = (struct spi_device*) self;
	struct unipi_spi_context *context;
	int fastop;
	if (count>10) { return -EIO; } 

	fastop = UNIPI_SPI_OP2_WRITESTR;

	context = unipi_spi2_alloc_context(spi_dev, UNIPI_SPI_OP2_WRITESTR, fastop);
	if (context == NULL) return -ENOMEM;

	context->tx_header[5] = count;
	memmove(context->tx_header+6, data,count);
	unipi_spi_crc_set(context->tx_header, UNIPI_SPI2_MESSAGE_LEN, 0);

	unipi_spi_trace_1(spi_dev, "REQ (wrstr %d) %6phC %8phC\n", count, context->tx_header, context->tx_header+6);

	context->len = count;
	context->interframe_nsec = UNIPI_SPI_INTER_FRAME_NS;
	return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
}

int unipi_spi2_read_str_async(void* self, unsigned int port, u8* data, unsigned int count, 
                             void* cb_data, OperationCallback cb_function)
{
	struct spi_device* spi_dev = (struct spi_device*) self;
	struct unipi_spi_context *context;
	int fastop;
//	if (count != 8) { return -EIO; } 

	fastop = UNIPI_SPI_OP2_READSTR;

	context = unipi_spi2_alloc_context(spi_dev,UNIPI_SPI_OP2_READSTR, fastop);
	if (context == NULL) return -ENOMEM;

	unipi_spi_crc_set(context->tx_header, UNIPI_SPI2_MESSAGE_LEN, 0);

	context->simple_read = 1;
	context->simple_len = 8; //count;
	context->len = 8; //count;
	context->data = data;
	unipi_spi_trace_1(spi_dev, "REQ (rdstr %d) %6phC\n", count, context->tx_header);
	return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
}


/*  Async op for WRITEBIT */
int unipi_spi2_write_bits_async(void* self, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function)
{
	struct spi_device* spi_dev = (struct spi_device*) self;
	if (count==0 || count > 32) return -EINVAL;

	return unipi_spi2_write_simple(spi_dev, UNIPI_SPI_OP_WRITEBITS, reg, count, data,
	                              cb_data, cb_function);
}

/*  Async op for READBIT */
int unipi_spi2_read_bits_async(void* self, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function)
{
	struct spi_device* spi_dev = (struct spi_device*) self;
	if (count==0 || count > 32) return -EINVAL;

	return unipi_spi2_read_simple(spi_dev, UNIPI_SPI_OP_READBIT, reg, count, data,
	                             cb_data, cb_function);
}

 
int unipi_spi2_idle_op_async(void* self, void* cb_data, OperationCallback cb_function)
{
	struct spi_device* spi_dev = (struct spi_device*) self;
	struct unipi_spi_context *context;
	context = unipi_spi2_alloc_context(spi_dev, UNIPI_SPI_OP2_IDLE, UNIPI_SPI_OP2_IDLE/*, 0x0e55*/);
	if (context==NULL)
		return -ENOMEM;
	return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
} 


struct unipi_protocol_op spi_op_v2 = {
	.ping_async = unipi_spi2_idle_op_async,
	.read_regs_async = unipi_spi2_read_regs_async,
	.write_regs_async = unipi_spi2_write_regs_async,
	.read_bits_async = unipi_spi2_read_bits_async,
	.write_bits_async = unipi_spi2_write_bits_async,
	.read_str_async = unipi_spi2_read_str_async,
	.write_str_async = unipi_spi2_write_str_async,
	.populated = unipi_spi_populated,
	.firmware_sync = unipi_spi_firmware_op,
	.lock_op = unipi_spi_lock,
	.unlock_op = unipi_spi_unlock,
	.max_write_str_len = 10,
};
