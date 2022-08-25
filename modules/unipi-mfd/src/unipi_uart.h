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

/***************
 * Definitions *
 ***************/
#define PORT_UNIPI			184


/*************************
 * Function Declarations *
 *************************/
/*
int neuronspi_rx_queue_add(struct neuronspi_port *port, u8 data);
void neuronspi_rx_queue_swap(struct neuronspi_port *port);
void neuronspi_uart_handle_rx(struct neuronspi_port *port, int rxlen, u8* pbuf);
void unipi_uart_handle_tx(struct neuronspi_port *port, int calling);
*/

#endif /* MODULES_NEURON_SPI_SRC_UNIPI_UART_H_ */
