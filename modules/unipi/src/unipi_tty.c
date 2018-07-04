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

#include "unipi_tty.h"
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/timer.h>
#include <linux/ctype.h>
#include <linux/mm.h>
#include <linux/string.h>

struct tty_ldisc_ops neuronspi_tty_ldisc;

int neuronspi_tty_init()
{
	int err;
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI TTY Init\n");
#endif
	memset(&neuronspi_tty_ldisc, 0, sizeof(neuronspi_tty_ldisc));
	n_tty_inherit_ops(&neuronspi_tty_ldisc);
	neuronspi_tty_ldisc.magic 			= TTY_LDISC_MAGIC;
	neuronspi_tty_ldisc.name 			= "unipi_tty";
	neuronspi_tty_ldisc.owner 			= THIS_MODULE;
	err = tty_register_ldisc(N_PROFIBUS_FDL, &neuronspi_tty_ldisc);
	if (err) {
		printk(KERN_INFO "UniPi line discipline registration failed. (%d)", err);
		return err;
	}
	return 0;
}
