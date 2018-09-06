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


#define NEURONSPI_MAX_TX_WORK	4

/*************************
 * Function Declarations *
 *************************/

int neuronspi_rx_queue_add(struct neuronspi_port *port, u8 data);
void neuronspi_rx_queue_swap(struct neuronspi_port *port);

int neuronspi_uart_driver_init(void);
int neuronspi_uart_driver_exit(void);
int neuronspi_uart_probe(struct spi_device* spi, struct neuronspi_driver_data *n_spi);
void neuronspi_uart_remove(struct spi_device* spi);
int neuronspi_uart_probe_all(void);

/*********************
 * Data Declarations *
 *********************/

extern struct neuronspi_uart_data* neuronspi_uart_data_global;
extern struct uart_driver* neuronspi_uart_driver_global;

#endif /* MODULES_NEURON_SPI_SRC_UNIPI_UART_H_ */
