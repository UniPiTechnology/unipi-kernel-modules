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

/************
 * Includes *
 ************/

#include "unipi_uart.h"
#include "unipi_spi.h"

/********************
 * Data Definitions *
 ********************/

struct neuronspi_uart_data* neuronspi_uart_glob_data;
unsigned long neuronspi_lines;
struct uart_driver* neuronspi_uart;

/************************
 * Non-static Functions *
 ************************/

static void neuronspi_uart_set_iflags(struct uart_port *port, int to)
{
	u8 *inp_buf, *outp_buf;
	int write_length;
	struct neuronspi_port *n_port;
	struct spi_device *spi;
	struct neuronspi_driver_data *n_spi;
	n_port = to_neuronspi_port(port, port);
	spi = neuronspi_s_dev[n_port->dev_index];
	n_spi = spi_get_drvdata(spi);
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: SET PARMRK to %d\n", to);
#endif
	write_length = neuronspi_spi_compose_single_register_write(NEURONSPI_UART_IFLAGS_REGISTER, &inp_buf, &outp_buf, to);
	neuronspi_spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 25, 1, 0);
	kfree(inp_buf);
	kfree(outp_buf);
}

void neuronspi_uart_set_ldisc(struct uart_port *port, struct ktermios *kterm)
{
	u8 *inp_buf, *outp_buf;
	int write_length;
	struct neuronspi_port *n_port;
	struct spi_device *spi;
	struct neuronspi_driver_data *n_spi;
	n_port = to_neuronspi_port(port, port);
	spi = neuronspi_s_dev[n_port->dev_index];
	n_spi = spi_get_drvdata(spi);
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: PROFIBUS discipline set\n");
#endif
	write_length = neuronspi_spi_compose_single_register_write(NEURONSPI_UART_LDISC_REGISTER, &inp_buf, &outp_buf, kterm->c_line);
	neuronspi_spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 25, 1, 0);
	kfree(inp_buf);
	kfree(outp_buf);
}

void neuronspi_uart_tx_proc(struct kthread_work *ws)
{
	struct neuronspi_port *port = to_neuronspi_port(ws, tx_work);

	if ((port->port.rs485.flags & SER_RS485_ENABLED) &&
	    (port->port.rs485.delay_rts_before_send > 0)) {
		msleep(port->port.rs485.delay_rts_before_send);
	}
	neuronspi_uart_handle_tx(port);
}

u32 neuronspi_uart_tx_empty(struct uart_port *port)
{
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: UART TX Empty\n");
#endif
	return TIOCSER_TEMT;
}

u32 neuronspi_uart_get_mctrl(struct uart_port *port)
{
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: UART MCTRL Get\n");
#endif
	return TIOCM_DSR | TIOCM_CAR;
}

int	neuronspi_uart_ioctl (struct uart_port *port, unsigned int ioctl_code, unsigned long ioctl_arg)
{
	u8 *inp_buf, *outp_buf;
	int write_length;
	struct neuronspi_port *n_port;
	struct spi_device *spi;
	struct neuronspi_driver_data *n_spi;
	n_port = to_neuronspi_port(port, port);
	spi = neuronspi_s_dev[n_port->dev_index];
	n_spi = spi_get_drvdata(spi);
	switch (ioctl_code) {
	case TIOCSETD: {
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_INFO "NEURONSPI: IOCTL TIOCSETD (processed via set_termios)\n");
#endif
		return 1;
	}
	case 0x5481: {
//#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_INFO "NEURONSPI: IOCTL 0x5481\n");
//#endif
		write_length = neuronspi_spi_compose_single_register_write(NEURONSPI_UART_TIMEOUT_REGISTER, &inp_buf, &outp_buf, (ioctl_arg * 1000000) / n_port->baud);
		printk(KERN_INFO "NEURONSPI: val_upper: %x, val_lower: %x", inp_buf[10], inp_buf[11]);
		neuronspi_spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 25, 1, 0);
		kfree(inp_buf);
		kfree(outp_buf);
		return 0;
	}
	case 0x5480: {
//#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_INFO "NEURONSPI: IOCTL 0x5480\n");
//#endif
		write_length = neuronspi_spi_compose_single_register_write(NEURONSPI_UART_TIMEOUT_REGISTER, &inp_buf, &outp_buf, ioctl_arg * 10);
		printk(KERN_INFO "NEURONSPI: val_upper: %x, val_lower: %x", inp_buf[10], inp_buf[11]);
		neuronspi_spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 25, 1, 0);
		kfree(inp_buf);
		kfree(outp_buf);
		return 0;
	}
	default: {
		return -ENOIOCTLCMD;
	}
	}
}



void neuronspi_uart_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
	struct neuronspi_port *n_port;
	n_port = to_neuronspi_port(port, port);
	if (old && old->c_iflag && old->c_iflag != termios->c_iflag) {
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_INFO "NEURONSPI: c_iflag termios:%d\n", termios->c_iflag);
#endif
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: TERMIOS Set, p:%d, c_cflag:%x\n", port->line, termios->c_cflag);
#endif
	neuronspi_spi_uart_set_cflag(neuronspi_s_dev[n_port->dev_index], n_port->dev_port, termios->c_cflag);
	if (old && termios && (old->c_iflag & PARMRK) != (termios->c_iflag & PARMRK)) {
		if (termios->c_iflag & PARMRK) {
			neuronspi_uart_set_iflags(port, termios->c_iflag);
		} else {
			neuronspi_uart_set_iflags(port, termios->c_iflag);
		}
	}
	if (old && termios && old->c_line != termios->c_line) {
		printk(KERN_INFO "NEURONSPI: Line Discipline change\n");
		if (termios->c_line == N_PROFIBUS_FDL) {
#if NEURONSPI_DETAILED_DEBUG > 0
			printk(KERN_INFO "NEURONSPI: Line Discipline change\n");
#endif
			neuronspi_uart_set_ldisc(port, termios);
		}
	}
	n_port->baud = uart_get_baud_rate(port, termios, old, 134, 115200);
	uart_update_timeout(port, termios->c_cflag, n_port->baud);
}

s32 neuronspi_uart_config_rs485(struct uart_port *port, struct serial_rs485 *rs485)
{
	port->rs485 = *rs485;
	return 0;
}

const char* neuronspi_uart_type(struct uart_port *port)
{
	return port->type == PORT_NEURONSPI ? "NEURONSPI_NAME" : NULL;
}

s32 neuronspi_uart_request_port(struct uart_port *port)
{
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: UART requested port %d\n", port->line);
#endif
	return 0;
}

void neuronspi_uart_fifo_read(struct uart_port *port, u32 rxlen)
{
	s32 i;
	struct neuronspi_port *s = to_neuronspi_port(port,port);
	struct neuronspi_driver_data *d_data = spi_get_drvdata(neuronspi_s_dev[s->dev_index]);
//#if NEURONSPI_DETAILED_DEBUG > 2
	printk(KERN_INFO "NEURONSPI: FIFO Read len:%d\n", rxlen);
//#endif
    memcpy(s->buf, d_data->uart_buf, rxlen);
	for (i = 0; i < rxlen; i++) {
//#if NEURONSPI_DETAILED_DEBUG > 2
		printk(KERN_INFO "NEURONSPI: UART Char Read: %x\n", d_data->uart_buf[i]);
//#endif
	}
}

int static neuronspi_uart_get_charcount(struct neuronspi_port *port) {
	u8 *inp_buf, *outp_buf;
	int read_length;
	struct spi_device *spi;
	struct neuronspi_driver_data *n_spi;
	int ret = 0;
	spi = neuronspi_s_dev[port->dev_index];
	n_spi = spi_get_drvdata(spi);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map && n_spi->regstart_table->uart_queue_reg) {
		read_length = neuronspi_spi_compose_single_register_read(n_spi->regstart_table->uart_queue_reg, &inp_buf, &outp_buf);
		neuronspi_spi_send_message(spi, inp_buf, outp_buf, read_length, n_spi->ideal_frequency, 35, 1, 0);
		ret = outp_buf[MODBUS_FIRST_DATA_BYTE + 1];
		kfree(inp_buf);
		kfree(outp_buf);
	}
	return ret;
}

void neuronspi_uart_fifo_write(struct neuronspi_port *port, u8 to_send)
{
	s32 i;
#if NEURONSPI_DETAILED_DEBUG > 2
	printk(KERN_INFO "NEURONSPI: FIFO Write to_send:%d\n", to_send);
#endif
	for (i = 0; i < to_send; i++) {
#if NEURONSPI_DETAILED_DEBUG > 2
		printk(KERN_INFO "NEURONSPI: UART Char Send: %x\n", port->buf[i]);
#endif
	}
	while(neuronspi_uart_get_charcount(port) > 50) {
		msleep(1);
	}
    neuronspi_spi_uart_write(neuronspi_s_dev[port->dev_index], port->buf, to_send, port->dev_port);
}

s32 neuronspi_uart_alloc_line(void)
{
	s32 i;
	BUILD_BUG_ON(NEURONSPI_MAX_DEVS > BITS_PER_LONG);

	for (i = 0; i < NEURONSPI_MAX_DEVS; i++)
		if (!test_and_set_bit(i, &neuronspi_lines))
			break;

	return i;
}

void neuronspi_uart_handle_rx(struct neuronspi_port *port, u32 rxlen, u32 iir)
{
	u32 ch, flag, bytes_read, i;
	while (rxlen) {

		neuronspi_uart_fifo_read(&port->port, rxlen);
		bytes_read = rxlen;

		port->port.icount.rx++;
		flag = TTY_NORMAL;

		for (i = 0; i < bytes_read; ++i) {
#if NEURONSPI_DETAILED_DEBUG > 0
			printk(KERN_INFO "NEURONSPI: UART Insert Char:%x\n", port->buf[i]);
#endif
			ch = port->buf[i];
			if (uart_handle_sysrq_char(port, ch))
				continue;

			uart_insert_char(&port->port, 0, 0, ch, flag);
		}
		rxlen -= bytes_read;
	}

	tty_flip_buffer_push(&port->port.state->port);
}

void neuronspi_uart_handle_tx(struct neuronspi_port *port)
{
	u32 max_txlen, to_send, i;
	struct spi_device *spi;
	struct neuronspi_driver_data *d_data;
	struct circ_buf *xmit;

	spi = neuronspi_s_dev[port->dev_index];
	d_data = spi_get_drvdata(spi);
	xmit = &port->port.state->xmit;

	if (unlikely(port->port.x_char)) {
		neuronspi_spi_uart_write(spi, &port->port.x_char, 1, port->dev_port);
		port->port.icount.tx++;
		port->port.x_char = 0;
		spin_lock(&port->tx_lock);
		port->tx_work_count--;
		spin_unlock(&port->tx_lock);
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&port->port)) {
		spin_lock(&port->tx_lock);
		port->tx_work_count--;
		spin_unlock(&port->tx_lock);
		return;
	}

	/* Get length of data pending in circular buffer */
	to_send = uart_circ_chars_pending(xmit);
	printk(KERN_INFO "NEURONSPI UART_HANDLE_TX, to_send:%d, tx_work_count:%d\n", to_send, port->tx_work_count);
	if (likely(to_send)) {
		/* Limit to size of (TX FIFO / 2) */
		max_txlen = NEURONSPI_FIFO_SIZE >> 1;
		while (to_send > max_txlen) {
			to_send = (to_send > max_txlen) ? max_txlen : to_send;

			/* Add data to send */
			port->port.icount.tx += to_send;

			/* Convert to linear buffer */
			for (i = 0; i < to_send; ++i) {
				port->buf[i] = xmit->buf[xmit->tail];
				xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			}
			printk(KERN_INFO "NEURONSPI UART_HANDLE_TX, to_send:%d, tx_work_count:%d\n", to_send, port->tx_work_count);
			neuronspi_uart_fifo_write(port, to_send);
		}
		to_send = (to_send > NEURONSPI_FIFO_SIZE - NEURONSPI_FIFO_MIN_CONTINUOUS) ? NEURONSPI_FIFO_SIZE - NEURONSPI_FIFO_MIN_CONTINUOUS : to_send;

		/* Add data to send */
		port->port.icount.tx += to_send;

		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			port->buf[i] = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		}
		printk(KERN_INFO "NEURONSPI UART_HANDLE_TX, to_send:%d, tx_work_count:%d\n", to_send, port->tx_work_count);
		neuronspi_uart_fifo_write(port, to_send);

	}
	spin_lock(&port->tx_lock);
	port->tx_work_count--;
	spin_unlock(&port->tx_lock);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		spin_lock(&port->port.lock);
		uart_write_wakeup(&port->port);
		spin_unlock(&port->port.lock);
	}
}

void neuronspi_uart_handle_irq(struct neuronspi_uart_data *uart_data, u32 portno)
{
	struct neuronspi_port *n_port = &uart_data->p[portno];
	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];
	u8 *send_buf = kzalloc(NEURONSPI_UART_PROBE_MESSAGE_LEN, GFP_ATOMIC);
	u8 *recv_buf = kzalloc(NEURONSPI_UART_PROBE_MESSAGE_LEN, GFP_ATOMIC);
	memcpy(send_buf, NEURONSPI_UART_PROBE_MESSAGE, NEURONSPI_UART_PROBE_MESSAGE_LEN);
	neuronspi_spi_send_message(spi, send_buf, recv_buf, NEURONSPI_UART_PROBE_MESSAGE_LEN, NEURONSPI_DEFAULT_FREQ, 25, 1, 0);
	kfree(send_buf);
	kfree(recv_buf);
}

void neuronspi_uart_ist(struct kthread_work *ws)
{
	struct neuronspi_port *p = to_neuronspi_port(ws, irq_work);
	neuronspi_uart_handle_irq(p->parent, p->line);
}

void neuronspi_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_NEURONSPI;
	}
}

s32 neuronspi_uart_verify_port(struct uart_port *port,
				 struct serial_struct *s)
{
	if ((s->type != PORT_UNKNOWN) && (s->type != PORT_NEURONSPI))
		return -EINVAL;
	if (s->irq != port->irq)
		return -EINVAL;

	return 0;
}

void neuronspi_uart_pm(struct uart_port *port, u32 state, u32 oldstate)
{
	neuronspi_uart_power(port, (state == UART_PM_STATE_ON) ? 1 : 0);
}

s32 neuronspi_uart_probe(struct spi_device* dev, u8 device_index)
{
	struct neuronspi_driver_data* driver_data = spi_get_drvdata(dev);
	struct sched_param sched_param = { .sched_priority = MAX_RT_PRIO / 2 };
	s32 i, j, ret, new_uart_count;
	struct neuronspi_uart_data *uart_data = driver_data->uart_data;

	if (uart_data->p == NULL) {
		uart_data->p = kzalloc(sizeof(struct neuronspi_port[NEURONSPI_MAX_UART]), GFP_ATOMIC);
		for (i = 0; i < NEURONSPI_MAX_UART; i++) {
			uart_data->p[i].parent = uart_data;
		}
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_DEBUG "NEURONSPI: Allocated port structure for %d potential UART devices\n", NEURONSPI_MAX_UART);
#endif
	}

	new_uart_count = driver_data->uart_count + uart_data->p_count;

	// Initialise port data
	for (i = uart_data->p_count; i < new_uart_count; i++) {
		uart_data->p[i].dev_index = device_index;
		uart_data->p[i].dev_port = i - uart_data->p_count;
		uart_data->p[i].line		= i;
		uart_data->p[i].port.dev	= &(dev->dev);
		uart_data->p[i].port.irq	= dev->irq;
		uart_data->p[i].port.type	= PORT_NEURONSPI;
		uart_data->p[i].port.fifosize	= NEURONSPI_FIFO_SIZE;
		uart_data->p[i].port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
		uart_data->p[i].port.iotype	= UPIO_PORT;
		uart_data->p[i].port.uartclk	= 9600;
		uart_data->p[i].port.rs485_config = neuronspi_uart_config_rs485;
		uart_data->p[i].port.ops	= &neuronspi_uart_ops;
		uart_data->p[i].port.line	= neuronspi_uart_alloc_line();
		spin_lock_init(&uart_data->p[i].tx_lock);
		spin_lock_init(&uart_data->p[i].port.lock);
		if (uart_data->p[i].port.line >= NEURONSPI_MAX_DEVS) {
			ret = -ENOMEM;
		}
		kthread_init_work(&(uart_data->p[i].tx_work), neuronspi_uart_tx_proc);
		kthread_init_work(&(uart_data->p[i].rx_work), neuronspi_uart_rx_proc);
		kthread_init_work(&(uart_data->p[i].irq_work), neuronspi_uart_ist);
		uart_add_one_port(driver_data->serial_driver, &uart_data->p[i].port);
		printk(KERN_INFO "NEURONSPI: Added UART port %d\n", i);
	}

	// For ports on multiple SPI devices renumber the ports to correspond to SPI chip-select numbering
	if (uart_data->p_count) {
		// First remove all existing ports
		for (i = 0; i < new_uart_count; i++) {
			uart_remove_one_port(driver_data->serial_driver, &uart_data->p[i].port);
			clear_bit(uart_data->p[i].port.line, &neuronspi_lines);
			kthread_flush_worker(&uart_data->kworker);
		}
		// Now add the ports in the correct order
		for (i = 0; i < NEURONSPI_MAX_DEVS; i++) {
			if (neuronspi_s_dev[i] != NULL) {
				driver_data = spi_get_drvdata(neuronspi_s_dev[i]);
#if NEURONSPI_DETAILED_DEBUG > 0
				printk(KERN_DEBUG "NEURONSPI: Renumber not NULL %d UC:%d\n", i, driver_data->uart_count);
#endif
				if (driver_data->uart_count) {
					for (j = 0; j < new_uart_count; j++) {
						if (uart_data->p[j].dev_index == i) {
							uart_data->p[j].port.dev	= &(neuronspi_s_dev[i]->dev);
							uart_data->p[j].port.irq	= neuronspi_s_dev[i]->irq;
							uart_data->p[j].port.type	= PORT_NEURONSPI;
							uart_data->p[j].port.fifosize	= NEURONSPI_FIFO_SIZE;
							uart_data->p[j].port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
							uart_data->p[j].port.iotype	= UPIO_PORT;
							uart_data->p[j].port.uartclk	= 9600;
							uart_data->p[j].port.rs485_config = neuronspi_uart_config_rs485;
							uart_data->p[j].port.ops	= &neuronspi_uart_ops;
							uart_data->p[j].port.line	= neuronspi_uart_alloc_line();
							uart_add_one_port(driver_data->serial_driver, &uart_data->p[j].port);
#if NEURONSPI_DETAILED_DEBUG > 0
							printk(KERN_DEBUG "NEURONSPI: Added UART port %d\n", j);
#endif
						}
					}
				}
			}
		}
	}

	uart_data->p_count = new_uart_count;
	if (uart_data->kworker_task == NULL) {
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_DEBUG "NEURONSPI: KWorker Task is NULL\n");
#endif

		kthread_init_worker(&uart_data->kworker);

		uart_data->kworker_task = kthread_run(kthread_worker_fn, &uart_data->kworker,
						  "neuronspi");
		if (IS_ERR(uart_data->kworker_task)) {
			ret = PTR_ERR(uart_data->kworker_task);
		}
		sched_setscheduler(uart_data->kworker_task, SCHED_FIFO, &sched_param);
	}
	return ret;
}

s32 neuronspi_uart_remove(struct neuronspi_uart_data *u_data)
{
	struct neuronspi_driver_data *d_data;
	struct spi_device *spi;
	s32 i;

	for (i = 0; i < NEURONSPI_MAX_DEVS; i++) {
		if (!(neuronspi_s_dev[i] == NULL)) {
			spi = neuronspi_s_dev[i];
			d_data = spi_get_drvdata(spi);
			if (d_data->poll_thread != NULL) {
				kthread_stop(d_data->poll_thread);
			}
		}
	}
	for (i = 0; i < u_data->p_count; i++) {
		uart_remove_one_port(neuronspi_uart, &u_data->p[i].port);
		clear_bit(u_data->p[i].port.line, &neuronspi_lines);
		neuronspi_uart_power(&u_data->p[i].port, 0);
	}

	kthread_flush_worker(&u_data->kworker);
	return 0;
}

void neuronspi_uart_rx_proc(struct kthread_work *ws)
{
	s32 end_flag = 0;
	s32 read_count = 0;
	struct neuronspi_port *n_port = to_neuronspi_port(ws, rx_work);
	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);

	u8 *send_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_ATOMIC);
	u8 *recv_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_ATOMIC);

	mutex_lock(&neuronspi_master_mutex);
	read_count = d_data->uart_read;
	mutex_unlock(&neuronspi_master_mutex);

	while (!end_flag) {
		memset(recv_buf, 0, NEURONSPI_BUFFER_MAX);
		neuronspi_spi_uart_read(spi, send_buf, recv_buf, read_count, n_port->dev_port);
		if (recv_buf[6] == 0x65 && recv_buf[7] > 0) {
			mutex_lock(&neuronspi_master_mutex);
			memcpy(&d_data->uart_buf[0], &recv_buf[10], recv_buf[7]);
			neuronspi_uart_handle_rx(n_port, recv_buf[7], 1);
			read_count = recv_buf[9];
			mutex_unlock(&neuronspi_master_mutex);
		} else if (recv_buf[0] != 0x41) {
			mutex_lock(&neuronspi_master_mutex);
			d_data->uart_read = 0;
			end_flag = 1;
			mutex_unlock(&neuronspi_master_mutex);
		}
	}
	kfree(recv_buf);
	kfree(send_buf);
}

void neuronspi_uart_start_tx(struct uart_port *port)
{
	struct neuronspi_port *n_port = to_neuronspi_port(port,port);
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: Start TX\n");
#endif
	spin_lock(&n_port->tx_lock);
	if (n_port->tx_work_count > NEURONSPI_MAX_TX_WORK) {
		spin_unlock(&n_port->tx_lock);
		printk(KERN_INFO "NEURONSPI: TX WORK OVERFLOW\n");
		return;
	} else {
		n_port->tx_work_count++;
	}
	spin_unlock(&n_port->tx_lock);
	kthread_queue_work(&n_port->parent->kworker, &n_port->tx_work);
}

s32 neuronspi_uart_poll(void *data)
{
	struct neuronspi_driver_data *d_data = (struct neuronspi_driver_data*) data;
	struct neuronspi_uart_data *u_data;
	s32 i;
	while (!kthread_should_stop()) {
		usleep_range(2000,8000);
		if (d_data->uart_count) {
			u_data = d_data->uart_data;
			for (i = 0; i < u_data->p_count; i++) {
				if (u_data->p[i].dev_index == d_data->neuron_index) {
					kthread_queue_work(&u_data->kworker, &u_data->p[i].irq_work);
				}
			}
		}
	}
	return 0;
}

// Initialise the driver
s32 neuronspi_uart_startup(struct uart_port *port)
{
	struct neuronspi_port *n_port = to_neuronspi_port(port, port);
	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);
	neuronspi_spi_set_irqs(spi, 0x5);
	if (d_data->poll_thread != NULL) {
		wake_up_process(d_data->poll_thread);
	} else if (d_data->no_irq) {
		d_data->poll_thread = kthread_create(neuronspi_uart_poll, (void *)d_data, "UART_poll_thread");
	}
	neuronspi_uart_power(port, 1);
	// TODO: /* Reset FIFOs*/
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: UART Startup\n");
#endif
	return 0;
}

void neuronspi_uart_shutdown(struct uart_port *port)
{
    neuronspi_uart_power(port, 0);
}

/*******************
 * Empty functions *
 *******************/

void neuronspi_uart_stop_tx(struct uart_port *port)
{
	/* Do Nothing */
}

void neuronspi_uart_stop_rx(struct uart_port *port)
{
	/* Do Nothing */
}
void neuronspi_uart_set_mctrl(struct uart_port *port, u32 mctrl)
{
	/* Do Nothing */
}
void neuronspi_uart_break_ctl(struct uart_port *port, int break_state)
{
	/* Do Nothing */
}
void neuronspi_uart_power(struct uart_port *port, s32 on)
{
    /* Do nothing */
}
void neuronspi_uart_null_void(struct uart_port *port)
{
	/* Do nothing */
}
