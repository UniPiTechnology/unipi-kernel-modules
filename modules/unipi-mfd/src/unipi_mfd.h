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

#ifndef MODULES_UNIPI_SPI_SRC_UNIPI_MFD_IOGROUP_H_
#define MODULES_UNIPI_SPI_SRC_UNIPI_MFD_IOGROUP_H_

#include <linux/version.h>
#include "unipi_common.h"
#include "unipi_iogroup_bus.h"

/*
 * Standard registers and coils supported by any Unipi MFD firmware 
*/
#define UNIPI_MFD_COIL_WAS_WATCHDOG  1000
#define UNIPI_MFD_COIL_OW_POWER_OFF  1001
#define UNIPI_MFD_COIL_REBOOT        1002
#define UNIPI_MFD_COIL_NVRAM_SAVE    1003
#define UNIPI_MFD_COIL_FIRMWARE_MODE 1004
#define UNIPI_MFD_COIL_PROTOCOL_MODE 1007

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

/*
static char* unipi_mfd_firmware_names[] = {
	"B-1000",		// 0
	"E-8Di8Ro",		// 1
	"E-14Ro",	 	// 2
	"E-16Di",		// 3
	"+P-11DiR485",	// 4
	"+P-11DiR485",  // 5
	"+P-11DiR485",  // 6
	"+U-14Ro",  	// 7
	"+U-14Ro",  	// 8
	"+U-14Di",  	// 9
	"+U-14Di",  	// 10  0x0a
	"E-4Ai44o",		// 11  0x0b
	"+P-6Di5Ro",	// 12  0x0c
	"B-485",		// 13  0x0d
	"E-4Light",		// 14  0x0e
	"+U-6Di5Ro",	// 15  0x0f
	"X_1Ir",		// 16  0x10
	"MM_8OW",		// 17  0x11
	"+P-4Di6Ro",	// 18  0x12
	"+U-4Di6Ro",	// 19  0x13
	"+P-4Di4Ro",	// 20  0x14
};
*/

struct regmap* unipi_mfd_get_regmap(struct device* dev, const char* name);
int unipi_mfd_enable_interrupt(struct unipi_iogroup_device *iogroup, u16 mask);


#endif /* MODULES_UNIPI_SPI_SRC_UNIPI_MFD_IOGROUP_H_ */
