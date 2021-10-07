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

#ifndef MODULES_NEURON_SPI_SRC_UNIPI_PLATFORM_H_
#define MODULES_NEURON_SPI_SRC_UNIPI_PLATFORM_H_

/************
 * Includes *
 ************/

#include "unipi_common.h"
#include "unipi_gpio.h"
#include "unipi_misc.h"
#include "unipi_iio.h"

/*******************
 * Data Structures *
 *******************/

struct neuronspi_board_entry {
	u16		index;
	u16		lower_board_id;
	u16		upper_board_id;
	u16		data_register_count;
	u16		config_register_count;
	struct neuronspi_board_combination *definition;
};

struct neuronspi_register_block
{
	u32 	starting_register;
	u32 	register_count;
	u32		*register_flags;
};

struct neuronspi_board_features
{
	u32		do_count;
	u32		ro_count;
	u32		ds_count;
	u32		di_count;
	u32		led_count;
	u32		stm_ai_count;
	u32		stm_ao_count;
	u32		sec_ai_count;
	u32		sec_ao_count;
	u32		uart_master_count;
	u32		uart_slave_count;
	u32		pwm_channel_count;
	u32		wd_count;
	u32		extension_sys_count;
	u32		light_count;
	u32		owire_count;
	u32		sysled_count;
};

struct neuronspi_board_regstart_table
{
	u32		do_val_reg;
	u32		do_val_coil;
	u32		do_pwm_reg;
	u32		do_pwm_ps_reg;
	u32		do_pwm_c_reg;
	u32		di_val_reg;
	u32		di_counter_reg;
	u32		di_direct_reg;
	u32		di_deboun_reg;
	u32		di_polar_reg;
	u32		di_toggle_reg;
	u32		uart_queue_reg;
	u32		uart_conf_reg;
	u32		uart_address_reg;
	u32		led_val_coil;
	u32		led_val_reg;
	u32		wd_val_reg;
	u32		wd_timeout_reg;
	u32		wd_nv_sav_coil;
	u32		wd_reset_coil;
	u32		reg_start_reg;
	u32		ro_val_reg;
	u32		ro_val_coil;
	u32		vref_inp;
	u32		vref_int;
	u32		stm_ao_val_reg;
	u32		stm_ao_mode_reg;
	u32		stm_ao_vol_err;
	u32		stm_ao_vol_off;
	u32		stm_ao_curr_err;
	u32		stm_ao_curr_off;
	u32		stm_ai_val_reg;
	u32		stm_ai_mode_reg;
	u32		stm_ai_curr_err;
	u32		stm_ai_curr_off;
	u32		stm_ai_vol_err;
	u32		stm_ai_vol_off;
	u32		stm_aio_val_reg;
	u32		stm_aio_vol_err;
	u32		stm_aio_vol_off;
	u32		sec_ao_val_reg;
	u32		sec_ao_mode_reg;
	u32		sec_ai_val_reg;
	u32		sec_ai_mode_reg;
	u32		sys_serial_num;
	u32		sys_hw_ver;
	u32		sys_hw_flash_ver;
	u32		sys_sw_ver;
	u32		sysled_val_coil;
	u32		sysled_val_reg;
};

struct neuronspi_board_combination
{
	u32									combination_board_id;
	u16									lower_board_id;
	u16									upper_board_id;
	u32 								block_count;
	size_t								name_length;
	const char*							combination_name;
	struct neuronspi_board_features		features;
	u32					 			 	*blocks;
};

struct neuronspi_model_definition
{
	size_t								eeprom_length;
	const char* 						eeprom_name;
	size_t								name_length;
	const char*							model_name;
	u32									combination_count;
	struct neuronspi_board_combination 	*combinations;
    u32                                 first_cs;
};

struct neuronspi_board_device_data {
    struct neuronspi_driver_data *n_spi;
    struct neuronspi_led_driver *led_driver;
	struct neuronspi_gpio_driver *di_driver;
	struct neuronspi_gpio_driver *do_driver;
	struct neuronspi_gpio_driver *ro_driver;
	struct iio_dev *stm_ai_driver;
	struct iio_dev *stm_ao_driver;
	struct iio_dev **sec_ai_driver;
	struct iio_dev **sec_ao_driver;
};

/***************
 * Definitions *
 ***************/

// Lower Boards
#define NEURONSPI_BOARD_LOWER_B1000_ID		0
#define NEURONSPI_BOARD_LOWER_E8DI8RO_ID	1
#define NEURONSPI_BOARD_LOWER_E14RO_ID		2
#define NEURONSPI_BOARD_LOWER_E16DI_ID		3
#define NEURONSPI_BOARD_LOWER_E4AI4AO_ID	11
#define NEURONSPI_BOARD_LOWER_B485_ID		13
#define NEURONSPI_BOARD_LOWER_E4LIGHT_ID	14
#define IRIS_BOARD_ICDIS4_ID				25

// Upper Boards
#define NEURONSPI_BOARD_UPPER_NONE_ID		0
#define NEURONSPI_BOARD_UPPER_P11DIR485_ID	1
#define NEURONSPI_BOARD_UPPER_U14RO_ID		2
#define NEURONSPI_BOARD_UPPER_U14DI_ID		3
#define NEURONSPI_BOARD_UPPER_P6DI5RO_ID	4
#define NEURONSPI_BOARD_UPPER_U6DI5RO_ID	5
#define NEURONSPI_BOARD_UPPER_P4DI5RO_ID	6
#define NEURONSPI_BOARD_UPPER_U4DI5RO_ID	7

// Register function codes
// Digital Input Functions
#define NEURONSPI_FUNGROUP_DI			0
#define NEURONSPI_REGFUN_DI_READ			0 | NEURONSPI_FUNGROUP_DI << 8
#define NEURONSPI_REGFUN_DI_COUNTER_LOWER	1 | NEURONSPI_FUNGROUP_DI << 8
#define NEURONSPI_REGFUN_DI_COUNTER_UPPER	2 | NEURONSPI_FUNGROUP_DI << 8
#define NEURONSPI_REGFUN_DI_DEBOUNCE		3 | NEURONSPI_FUNGROUP_DI << 8
#define NEURONSPI_REGFUN_DS_ENABLE			4 | NEURONSPI_FUNGROUP_DI << 8
#define NEURONSPI_REGFUN_DS_POLARITY		5 | NEURONSPI_FUNGROUP_DI << 8
#define NEURONSPI_REGFUN_DS_TOGGLE			6 | NEURONSPI_FUNGROUP_DI << 8

// Digital Output Functions
#define NEURONSPI_FUNGROUP_DO			1
#define NEURONSPI_REGFUN_DO_RW				0 | NEURONSPI_FUNGROUP_DO << 8

// B1000 Analog Output Functions
#define NEURONSPI_FUNGROUP_AO_BRAIN		2
#define NEURONSPI_REGFUN_AO_BRAIN			0 | NEURONSPI_FUNGROUP_AO_BRAIN << 8
#define NEURONSPI_REGFUN_AO_BRAIN_MODE		1 | NEURONSPI_FUNGROUP_AO_BRAIN << 8
#define NEURONSPI_REGFUN_AO_BRAIN_V_ERR		2 | NEURONSPI_FUNGROUP_AO_BRAIN << 8
#define NEURONSPI_REGFUN_AO_BRAIN_V_OFF		3 | NEURONSPI_FUNGROUP_AO_BRAIN << 8
#define NEURONSPI_REGFUN_AO_BRAIN_I_ERR 	4 | NEURONSPI_FUNGROUP_AO_BRAIN << 8
#define NEURONSPI_REGFUN_AO_BRAIN_I_OFF 	5 | NEURONSPI_FUNGROUP_AO_BRAIN << 8

// B1000 Analog Input Functions
#define NEURONSPI_FUNGROUP_AI_BRAIN		3
#define NEURONSPI_REGFUN_AI_BRAIN			0 | NEURONSPI_FUNGROUP_AI_BRAIN << 8
#define NEURONSPI_REGFUN_AIO_BRAIN			1 | NEURONSPI_FUNGROUP_AI_BRAIN << 8
#define NEURONSPI_REGFUN_AI_BRAIN_MODE		2 | NEURONSPI_FUNGROUP_AI_BRAIN << 8
#define NEURONSPI_REGFUN_AI_BRAIN_V_ERR		3 | NEURONSPI_FUNGROUP_AI_BRAIN << 8
#define NEURONSPI_REGFUN_AI_BRAIN_V_OFF		4 | NEURONSPI_FUNGROUP_AI_BRAIN << 8
#define NEURONSPI_REGFUN_AI_BRAIN_I_ERR		5 | NEURONSPI_FUNGROUP_AI_BRAIN << 8
#define NEURONSPI_REGFUN_AI_BRAIN_I_OFF		6 | NEURONSPI_FUNGROUP_AI_BRAIN << 8
#define NEURONSPI_REGFUN_AIO_BRAIN_ERR		7 | NEURONSPI_FUNGROUP_AI_BRAIN << 8
#define NEURONSPI_REGFUN_AIO_BRAIN_OFF		8 | NEURONSPI_FUNGROUP_AI_BRAIN << 8

// System Functions
#define NEURONSPI_FUNGROUP_SYSTEM		4
#define NEURONSPI_REGFUN_V_REF_INT			0 | NEURONSPI_FUNGROUP_SYSTEM << 8
#define NEURONSPI_REGFUN_V_REF_INP			1 | NEURONSPI_FUNGROUP_SYSTEM << 8
#define NEURONSPI_REGFUN_LED_RW				2 | NEURONSPI_FUNGROUP_SYSTEM << 8
#define NEURONSPI_REGFUN_SW_VER 			3 | NEURONSPI_FUNGROUP_SYSTEM << 8
#define NEURONSPI_REGFUN_DIDO_COUNT			4 | NEURONSPI_FUNGROUP_SYSTEM << 8
#define NEURONSPI_REGFUN_UAIO_COUNT			5 | NEURONSPI_FUNGROUP_SYSTEM << 8
#define NEURONSPI_REGFUN_HW_VER				6 | NEURONSPI_FUNGROUP_SYSTEM << 8
#define NEURONSPI_REGFUN_FLASH_HW_VER 		7 | NEURONSPI_FUNGROUP_SYSTEM << 8
#define NEURONSPI_REGFUN_SERIAL_NR_LOWER	8 | NEURONSPI_FUNGROUP_SYSTEM << 8
#define NEURONSPI_REGFUN_SERIAL_NR_UPPER	9 | NEURONSPI_FUNGROUP_SYSTEM << 8
#define NEURONSPI_REGFUN_INTERRUPTS			10 | NEURONSPI_FUNGROUP_SYSTEM << 8
#define NEURONSPI_REGFUN_NONE_TEST			11 | NEURONSPI_FUNGROUP_SYSTEM << 8

// Watchdog Functions
#define NEURONSPI_FUNGROUP_MWD			5
#define NEURONSPI_REGFUN_MWD_TO				0 | NEURONSPI_FUNGROUP_MWD << 8
#define NEURONSPI_REGFUN_MWD_STATUS			1 | NEURONSPI_FUNGROUP_MWD << 8

// PWM Functions
#define NEURONSPI_FUNGROUP_PWM			6
#define NEURONSPI_REGFUN_PWM_DUTY			0 | NEURONSPI_FUNGROUP_PWM << 8
#define NEURONSPI_REGFUN_PWM_PRESCALE		1 | NEURONSPI_FUNGROUP_PWM << 8
#define NEURONSPI_REGFUN_PWM_CYCLE			2 | NEURONSPI_FUNGROUP_PWM << 8

// UART Functions
#define NEURONSPI_FUNGROUP_RS485		7
#define NEURONSPI_REGFUN_TX_QUEUE_LEN		0 | NEURONSPI_FUNGROUP_RS485 << 8
#define NEURONSPI_REGFUN_RS485_CONFIG		1 | NEURONSPI_FUNGROUP_RS485 << 8
#define NEURONSPI_REGFUN_RS485_ADDRESS		2 | NEURONSPI_FUNGROUP_RS485 << 8

// Secondary Analog Output Functions
#define NEURONSPI_FUNGROUP_AO_VER2		8
#define NEURONSPI_REGFUN_AO_VER2_RW			0 | NEURONSPI_FUNGROUP_AO_VER2 << 8

// Secondary Analog Input Functions
#define NEURONSPI_FUNGROUP_AI_VER2		9
#define NEURONSPI_REGFUN_AI_VER2_READ_LOWER	0 | NEURONSPI_FUNGROUP_AI_VER2 << 8
#define NEURONSPI_REGFUN_AI_VER2_READ_UPPER	1 | NEURONSPI_FUNGROUP_AI_VER2 << 8
#define NEURONSPI_REGFUN_AI_VER2_MODE		2 | NEURONSPI_FUNGROUP_AI_VER2 << 8

// Special Functions
#define NEURONSPI_FUNGROUP_SPECIAL		127
#define NEURONSPI_REGFUN_SPECIAL_NULL	0 | NEURONSPI_FUNGROUP_SPECIAL << 8

// Register access flags
#define NEURONSPI_REGFLAG_ACC_NEVER	0
#define NEURONSPI_REGFLAG_ACC_AFAP	0x1 << 16
#define NEURONSPI_REGFLAG_ACC_10HZ 	0x2 << 16
#define NEURONSPI_REGFLAG_ACC_1HZ   0x3 << 16
#define NEURONSPI_REGFLAG_ACC_6SEC  0x4 << 16
#define NEURONSPI_REGFLAG_ACC_1MIN  0x5 << 16
#define NEURONSPI_REGFLAG_ACC_15MIN 0x6 << 16
#define NEURONSPI_REGFLAG_ACC_ONCE 	0x7 << 16

// Register system flags
#define NEURONSPI_REGFLAG_SYS_READ_ONLY	0x10 << 24

// IIO Modes
#define NEURONSPI_IIO_AI_STM_MODE_VOLTAGE 0x0
#define NEURONSPI_IIO_AI_STM_MODE_CURRENT 0x1
#define NEURONSPI_IIO_AI_STM_MODE_RESISTANCE 0x3

/*********************
 * Data Declarations *
 *********************/

extern struct platform_device *neuron_plc_dev;

// Board Definitions
extern struct neuronspi_board_combination NEURONSPI_BOARD_B1000_HW_COMBINATION[]; 				// B_1000 (S103)
extern struct neuronspi_board_combination NEURONSPI_BOARD_E8DI8RO_HW_COMBINATION[]; 			// E-8Di8Ro (M103)
extern struct neuronspi_board_combination NEURONSPI_BOARD_E14RO_HW_COMBINATION[]; 				// E-14Ro
extern struct neuronspi_board_combination NEURONSPI_BOARD_E16DI_HW_COMBINATION[]; 				// E-16Di
extern struct neuronspi_board_combination NEURONSPI_BOARD_E8DI8ROP11DIR485_HW_COMBINATION[]; 	// E-8Di8Ro_P-11DiR485 (xS10)
extern struct neuronspi_board_combination NEURONSPI_BOARD_E14ROP11DIR485_HW_COMBINATION[]; 		// E-14Ro_P-11DiR485 (xS40)
extern struct neuronspi_board_combination NEURONSPI_BOARD_E16DIP11DIR485_HW_COMBINATION[];		// E-16Di_P-11DiR485 (xS30)
extern struct neuronspi_board_combination NEURONSPI_BOARD_E14ROU14RO_HW_COMBINATION[];			// E-14Ro_U-14Ro (M403)
extern struct neuronspi_board_combination NEURONSPI_BOARD_E16DIU14RO_HW_COMBINATION[];			// E-16Di_U-14Ro (M203)
extern struct neuronspi_board_combination NEURONSPI_BOARD_E14ROU14DI_HW_COMBINATION[];			// E-14Ro_U-14Di (L503)
extern struct neuronspi_board_combination NEURONSPI_BOARD_E16DIU14DI_HW_COMBINATION[];			// E-16Di_U-14Di (M303)
extern struct neuronspi_board_combination NEURONSPI_BOARD_E4AI4AO_HW_COMBINATION[];				// E-4Ai4Ao
extern struct neuronspi_board_combination NEURONSPI_BOARD_E4AI4AOP6DI5RO_HW_COMBINATION[];		// E-4Ai4Ao_P-6Di5Ro (xS50)
extern struct neuronspi_board_combination NEURONSPI_BOARD_B485_HW_COMBINATION[];				// B-485
extern struct neuronspi_board_combination NEURONSPI_BOARD_E4LIGHT_HW_COMBINATION[];				// E-4Light (M613)
extern struct neuronspi_board_combination NEURONSPI_BOARD_E4AI4AOU6DI5RO_HW_COMBINATION[];		// E-4Ai4Ao_U-6Di5Ro (L503)
extern struct neuronspi_board_combination NEURONSPI_BOARD_E4AI4AOP4DI5RO_HW_COMBINATION[];		// E-4Ai4Ao_P-4Di5Ro (xS5?)
extern struct neuronspi_board_combination NEURONSPI_BOARD_E4AI4AOU4DI5RO_HW_COMBINATION[];		// E-4Ai4Ao_U-4Di5Ro (L5?3)

// Model Definitions
#define NEURONSPI_MODEL_S103_HW_DEFINITION_BOARD_SIZE 1
extern struct neuronspi_board_combination NEURONSPI_MODEL_S103_HW_DEFINITION_BOARD[NEURONSPI_MODEL_S103_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_S103G_HW_DEFINITION_BOARD_SIZE 1
extern struct neuronspi_board_combination NEURONSPI_MODEL_S103G_HW_DEFINITION_BOARD[NEURONSPI_MODEL_S103G_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_S103IQ_HW_DEFINITION_BOARD_SIZE 1
extern struct neuronspi_board_combination NEURONSPI_MODEL_S103IQ_HW_DEFINITION_BOARD[NEURONSPI_MODEL_S103IQ_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_S103EO_HW_DEFINITION_BOARD_SIZE 1
extern struct neuronspi_board_combination NEURONSPI_MODEL_S103EO_HW_DEFINITION_BOARD[NEURONSPI_MODEL_S103EO_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_S105_HW_DEFINITION_BOARD_SIZE 1
extern struct neuronspi_board_combination NEURONSPI_MODEL_S105_HW_DEFINITION_BOARD[NEURONSPI_MODEL_S105_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_S115_HW_DEFINITION_BOARD_SIZE 1
extern struct neuronspi_board_combination NEURONSPI_MODEL_S115_HW_DEFINITION_BOARD[NEURONSPI_MODEL_S115_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_S155_HW_DEFINITION_BOARD_SIZE 1
extern struct neuronspi_board_combination NEURONSPI_MODEL_S155_HW_DEFINITION_BOARD[NEURONSPI_MODEL_S155_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_S205_HW_DEFINITION_BOARD_SIZE 1
extern struct neuronspi_board_combination NEURONSPI_MODEL_S205_HW_DEFINITION_BOARD[NEURONSPI_MODEL_S205_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_S215_HW_DEFINITION_BOARD_SIZE 1
extern struct neuronspi_board_combination NEURONSPI_MODEL_S215_HW_DEFINITION_BOARD[NEURONSPI_MODEL_S215_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_S505_HW_DEFINITION_BOARD_SIZE 1
extern struct neuronspi_board_combination NEURONSPI_MODEL_S505_HW_DEFINITION_BOARD[NEURONSPI_MODEL_S505_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_S515_HW_DEFINITION_BOARD_SIZE 1
extern struct neuronspi_board_combination NEURONSPI_MODEL_S515_HW_DEFINITION_BOARD[NEURONSPI_MODEL_S515_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_S605_HW_DEFINITION_BOARD_SIZE 1
extern struct neuronspi_board_combination NEURONSPI_MODEL_S605_HW_DEFINITION_BOARD[NEURONSPI_MODEL_S605_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_M103_HW_DEFINITION_BOARD_SIZE 2
extern struct neuronspi_board_combination NEURONSPI_MODEL_M103_HW_DEFINITION_BOARD[NEURONSPI_MODEL_M103_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_M203_HW_DEFINITION_BOARD_SIZE 2
extern struct neuronspi_board_combination NEURONSPI_MODEL_M203_HW_DEFINITION_BOARD[NEURONSPI_MODEL_M203_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_M303_HW_DEFINITION_BOARD_SIZE 2
extern struct neuronspi_board_combination NEURONSPI_MODEL_M303_HW_DEFINITION_BOARD[NEURONSPI_MODEL_M303_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_M403_HW_DEFINITION_BOARD_SIZE 2
extern struct neuronspi_board_combination NEURONSPI_MODEL_M403_HW_DEFINITION_BOARD[NEURONSPI_MODEL_M403_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_M503_HW_DEFINITION_BOARD_SIZE 2
extern struct neuronspi_board_combination NEURONSPI_MODEL_M503_HW_DEFINITION_BOARD[NEURONSPI_MODEL_M503_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_M603_HW_DEFINITION_BOARD_SIZE 2
extern struct neuronspi_board_combination NEURONSPI_MODEL_M603_HW_DEFINITION_BOARD[NEURONSPI_MODEL_M603_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_M205_HW_DEFINITION_BOARD_SIZE 2
extern struct neuronspi_board_combination NEURONSPI_MODEL_M205_HW_DEFINITION_BOARD[NEURONSPI_MODEL_M205_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_M505_HW_DEFINITION_BOARD_SIZE 2
extern struct neuronspi_board_combination NEURONSPI_MODEL_M505_HW_DEFINITION_BOARD[NEURONSPI_MODEL_M505_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_M515_HW_DEFINITION_BOARD_SIZE 2
extern struct neuronspi_board_combination NEURONSPI_MODEL_M515_HW_DEFINITION_BOARD[NEURONSPI_MODEL_M515_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_L203_HW_DEFINITION_BOARD_SIZE 3
extern struct neuronspi_board_combination NEURONSPI_MODEL_L203_HW_DEFINITION_BOARD[NEURONSPI_MODEL_L203_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_L303_HW_DEFINITION_BOARD_SIZE 3
extern struct neuronspi_board_combination NEURONSPI_MODEL_L303_HW_DEFINITION_BOARD[NEURONSPI_MODEL_L303_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_L403_HW_DEFINITION_BOARD_SIZE 3
extern struct neuronspi_board_combination NEURONSPI_MODEL_L403_HW_DEFINITION_BOARD[NEURONSPI_MODEL_L403_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_L503_HW_DEFINITION_BOARD_SIZE 3
extern struct neuronspi_board_combination NEURONSPI_MODEL_L503_HW_DEFINITION_BOARD[NEURONSPI_MODEL_L503_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_L513_HW_DEFINITION_BOARD_SIZE 3
extern struct neuronspi_board_combination NEURONSPI_MODEL_L513_HW_DEFINITION_BOARD[NEURONSPI_MODEL_L513_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_L205_HW_DEFINITION_BOARD_SIZE 3
extern struct neuronspi_board_combination NEURONSPI_MODEL_L205_HW_DEFINITION_BOARD[NEURONSPI_MODEL_L205_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_L505_HW_DEFINITION_BOARD_SIZE 3
extern struct neuronspi_board_combination NEURONSPI_MODEL_L505_HW_DEFINITION_BOARD[NEURONSPI_MODEL_L505_HW_DEFINITION_BOARD_SIZE];
#define NEURONSPI_MODEL_L533_HW_DEFINITION_BOARD_SIZE 3
extern struct neuronspi_board_combination NEURONSPI_MODEL_L533_HW_DEFINITION_BOARD[NEURONSPI_MODEL_L533_HW_DEFINITION_BOARD_SIZE];


// Board table
#define NEURONSPI_BOARDTABLE_LEN		19
extern struct neuronspi_board_entry NEURONSPI_BOARDTABLE[];

// Module table
#define NEURONSPI_MODELTABLE_LEN		29
extern struct neuronspi_model_definition NEURONSPI_MODELTABLE[];

/*************************
 * Function Declarations *
 *************************/

int neuronspi_regmap_hw_gather_write(void *context, const void *reg, size_t reg_size, const void *val, size_t val_size);
int neuronspi_regmap_hw_read(void *context, const void *reg_buf, size_t reg_size, void *val_buf, size_t val_size);
int neuronspi_regmap_hw_reg_read(void *context, unsigned int reg, unsigned int *val);
int neuronspi_regmap_hw_reg_write(void *context, unsigned int reg, unsigned int val);
int neuronspi_regmap_hw_write(void *context, const void *data, size_t count);
void neuronspi_regmap_invalidate_device(struct regmap *reg_map, struct neuronspi_board_combination *device_def, u32 period_counter);
s32 neuronspi_regmap_invalidate(void *data);
int neuronspi_create_reg_starts(struct neuronspi_board_regstart_table *out_table, struct neuronspi_board_combination *board);
s32 neuronspi_find_reg_start(struct neuronspi_board_combination *board, u16 regfun);
s32 neuronspi_find_model_id(u32 probe_count);

struct platform_device * neuronspi_board_device_probe(struct neuronspi_driver_data *n_spi);
void neuronspi_board_device_remove(struct platform_device * board_device);

#endif /* MODULES_NEURON_SPI_SRC_UNIPI_PLATFORM_H_ */
