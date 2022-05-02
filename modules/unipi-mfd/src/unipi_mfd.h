/*
 * UniPi Multi Function Device Driver - Copyright (C) 2021 UniPi Technology
 *
 * Author: Miroslav Ondra <ondra@faster.cz>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef MODULES_UNIPI_SPI_SRC_UNIPI_MFD_H_
#define MODULES_UNIPI_SPI_SRC_UNIPI_MFD_H_

#include <linux/version.h>
#include "unipi_common.h"

/*
 * Standard registers and coils supported by any Unipi MFD firmware 
*/
#define UNIPI_MFD_COIL_WAS_WATCHDOG  1000
#define UNIPI_MFD_COIL_OW_POWER_OFF  1001
#define UNIPI_MFD_COIL_REBOOT        1002
#define UNIPI_MFD_COIL_NVRAM_SAVE    1003
#define UNIPI_MFD_COIL_FIRMWARE_MODE 1004

#define UNIPI_MFD_REG_FW_VERSION     1000
#define UNIPI_MFD_REG_FW_VARIANT     1003
#define UNIPI_MFD_REG_BOARD_ID       1004
#define UNIPI_MFD_REG_BOARD_SERIAL   1005
#define UNIPI_MFD_REG_INTERRUPT_MASK 1007
#define UNIPI_MFD_REG_MWD_TIMEOUT    1008
#define UNIPI_MFD_REG_VREFINT        1009

#define UNIPI_MFD_REG_SYSLEDS         509
#define UNIPI_MFD_REG_SYSLED_MODE     518
#define UNIPI_MFD_REG_PCB_VERSION     514

#define UNIPI_MFD_REG_UART0_CFLAGS     500
#define UNIPI_MFD_REG_UART0_IFLAGS    502
#define UNIPI_MFD_REG_UART0_LDISC     503
#define UNIPI_MFD_REG_UART0_TIMEOUT   504
#define UNIPI_MFD_REG_UART0_TXQLEN    505

/* speciality pro Braina a jeho analogy */
#define UNIPI_MFD_REG_BRAIN_AO_VAL     2
#define UNIPI_MFD_REG_BRAIN_AI_VAL     3
#define UNIPI_MFD_REG_BRAIN_VREFINP    5
#define UNIPI_MFD_REG_BRAIN_AO_MODE    1019
#define UNIPI_MFD_REG_BRAIN_AOV_DEV    1020
#define UNIPI_MFD_REG_BRAIN_AOV_OFS    1021
#define UNIPI_MFD_REG_BRAIN_AOA_DEV    1022
#define UNIPI_MFD_REG_BRAIN_AOA_OFS    1023
#define UNIPI_MFD_REG_BRAIN_AI_MODE    1024
#define UNIPI_MFD_REG_BRAIN_AIV_DEV    1025
#define UNIPI_MFD_REG_BRAIN_AIV_OFS    1026
#define UNIPI_MFD_REG_BRAIN_AIA_DEV    1027
#define UNIPI_MFD_REG_BRAIN_AIA_OFS    1028

#define UNIPI_MFD_INT_RX_NOT_EMPTY 			0x1
#define UNIPI_MFD_INT_TX_FINISHED  			0x2
#define UNIPI_MFD_INT_RX_MODBUS    			0x4
#define UNIPI_MFD_INT_DI_CHANGED   			0x8
#define UNIPI_MFD_INT_ID_MASK      			0x0f
#define UNIPI_MFD_INT_NO_INT_BIT   			0x0f


struct unipi_mfd_device;

typedef void (*OperationCallback)(void*, int, u8*);

typedef void (*RxCharCallback)(void*, u8, u8, int);
typedef void (*IntStatusCallback)(struct unipi_mfd_device*, u8);
typedef int  (*AsyncIdle)(void*, void*, OperationCallback);
typedef int  (*RegReadAsync)(void *, unsigned int, void*,  OperationCallback);
typedef int  (*StrAsync)(void *, unsigned int, u8*, unsigned int, void*,  OperationCallback);
typedef void (*Populated)(void *);


struct unipi_mfd_op 
{
	AsyncIdle	 async_idle;
	RegReadAsync reg_read_async;
	StrAsync	 write_str_async;
	StrAsync	 read_str_async;
	Populated	 populated;
};

struct unipi_mfd_device
{
	struct regmap*			registers;
	struct regmap*			coils;
	int 					modbus_address;
	void					*rx_self;
	RxCharCallback			rx_char_callback;
	IntStatusCallback		interrupt_status_callback;
	struct unipi_mfd_op  	*op;
	int						irq;
	struct hrtimer			poll_timer;
	int						poll_enabled;
	void*		 self;
};


int unipi_mfd_init(struct unipi_mfd_device * mfd, struct device* dev);
int unipi_mfd_exit(struct unipi_mfd_device * mfd);
struct unipi_mfd_device* unipi_spi_get_mfd(struct spi_device* spi_dev);

int unipi_mfd_enable_interrupt(struct unipi_mfd_device *mfd, u16 mask);



#define unipi_mfd_rx_char(mfd, port, ch, remain) {if ((mfd)->rx_char_callback) \
                         (mfd)->rx_char_callback((mfd)->rx_self, port,ch,remain);}

#define unipi_mfd_int_status(mfd, int_status) {if ((mfd)->interrupt_status_callback) \
                         (mfd)->interrupt_status_callback(mfd, int_status);}

#define unipi_mfd_read_str_async(mfd, port, data, count, cb_data, cb_function) \
                         (mfd)->op->read_str_async(mfd->self, port, data, count, cb_data, cb_function)

#define unipi_mfd_write_str_async(mfd, port, data, count, cb_data, cb_function)  \
                         (mfd)->op->write_str_async(mfd->self, port, data, count, cb_data, cb_function)

#define unipi_mfd_populated(mfd) (mfd)->op->populated(mfd->self)


#endif /* MODULES_UNIPI_SPI_SRC_UNIPI_MFD_H_ */
