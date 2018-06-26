/*
 * UniPi Neuron tty serial driver - Copyright (C) 2018 UniPi Technologies
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

#ifndef MODULES_NEURON_SPI_SRC_UNIPI_UART_H_
#define MODULES_NEURON_SPI_SRC_UNIPI_UART_H_

/************
 * Includes *
 ************/

#include "unipi_common.h"
#include "unipi_platform.h"

/***************
 * Definitions *
 ***************/

#define NEURONSPI_UART_IFLAGS_REGISTER 	502
#define NEURONSPI_UART_LDISC_REGISTER 	503
#define NEURONSPI_UART_TIMEOUT_REGISTER 504

#define NEURONSPI_MAX_TX_WORK	4

/*************************
 * Function Declarations *
 *************************/

void neuronspi_uart_start_tx(struct uart_port *port);
void neuronspi_uart_stop_tx(struct uart_port *port);
void neuronspi_uart_stop_rx(struct uart_port *port);
void neuronspi_uart_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old);
u32 neuronspi_uart_tx_empty(struct uart_port *port);
void neuronspi_uart_break_ctl(struct uart_port *port, int break_state);
void neuronspi_uart_shutdown(struct uart_port *port);
s32 neuronspi_uart_startup(struct uart_port *port);
s32 neuronspi_uart_request_port(struct uart_port *port);
s32 neuronspi_uart_alloc_line(void);
void neuronspi_uart_set_mctrl(struct uart_port *port, u32 mctrl);
int	neuronspi_uart_ioctl (struct uart_port *port, unsigned int ioctl_code, unsigned long ioctl_arg);
void neuronspi_uart_set_ldisc(struct uart_port *port, struct ktermios *kterm);
u32 neuronspi_uart_get_mctrl(struct uart_port *port);
const char *neuronspi_uart_type(struct uart_port *port);
void neuronspi_uart_null_void(struct uart_port *port);
void neuronspi_uart_config_port(struct uart_port *port, int flags);
s32 neuronspi_uart_verify_port(struct uart_port *port, struct serial_struct *s);
void neuronspi_uart_pm(struct uart_port *port, u32 state,  u32 oldstate);
s32 neuronspi_uart_poll(void *data);
s32 neuronspi_uart_probe(struct spi_device* dev, u8 device_index);
s32 neuronspi_uart_remove(struct neuronspi_uart_data *u_data);
void neuronspi_uart_power(struct uart_port *port, s32 on);
s32 neuronspi_uart_config_rs485(struct uart_port *port, struct serial_rs485 *rs485);
void neuronspi_spi_uart_set_cflag(struct spi_device* spi_dev, u8 port, u32 to);
u32 neuronspi_spi_uart_get_cflag(struct spi_device* spi_dev, u8 port);
void neuronspi_uart_fifo_write(struct neuronspi_port *port, u8 to_send);
void neuronspi_uart_fifo_read(struct uart_port *port, u32 rxlen);
void neuronspi_uart_rx_proc(struct kthread_work *ws);
void neuronspi_uart_tx_proc(struct kthread_work *ws);
void neuronspi_uart_ist(struct kthread_work *ws);
void neuronspi_uart_handle_tx(struct neuronspi_port *port);
void neuronspi_uart_handle_rx(struct neuronspi_port *port, u32 rxlen, u32 iir);
void neuronspi_uart_handle_irq(struct neuronspi_uart_data *uart_data, u32 portno);

/*********************
 * Data Declarations *
 *********************/

extern struct neuronspi_uart_data* neuronspi_uart_glob_data;
extern unsigned long neuronspi_lines;
extern struct uart_driver* neuronspi_uart;

static const struct uart_ops neuronspi_uart_ops =
{
	.tx_empty			= neuronspi_uart_tx_empty,
	.set_mctrl			= neuronspi_uart_set_mctrl,
	.get_mctrl			= neuronspi_uart_get_mctrl,
	.stop_tx			= neuronspi_uart_stop_tx,
	.start_tx			= neuronspi_uart_start_tx,
	.stop_rx			= neuronspi_uart_stop_rx,
	.break_ctl			= neuronspi_uart_break_ctl,
	.startup			= neuronspi_uart_startup,
	.shutdown			= neuronspi_uart_shutdown,
	.set_termios		= neuronspi_uart_set_termios,
	.set_ldisc			= neuronspi_uart_set_ldisc,
	.type				= neuronspi_uart_type,
	.request_port		= neuronspi_uart_request_port,
	.release_port		= neuronspi_uart_null_void,
	.config_port		= neuronspi_uart_config_port,
	.verify_port		= neuronspi_uart_verify_port,
	.pm					= neuronspi_uart_pm,
	.ioctl				= neuronspi_uart_ioctl,
};

#endif /* MODULES_NEURON_SPI_SRC_UNIPI_UART_H_ */
