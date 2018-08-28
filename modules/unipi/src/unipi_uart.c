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

/************
 * Includes *
 ************/

#include "unipi_uart.h"
#include "unipi_spi.h"

#if NEURONSPI_DETAILED_DEBUG > 2
# define unipi_uart_trace_2(f, args...)	printk(f, ##args)
#else
# define unipi_uart_trace_2(f, args...)
#endif

#if NEURONSPI_DETAILED_DEBUG > 1
# define unipi_uart_trace_1(f, args...)	printk(f, ##args)
#else
# define unipi_uart_trace_1(f, args...)
#endif

#if NEURONSPI_DETAILED_DEBUG > 0
# define unipi_uart_trace(f, args...)	printk(f, ##args)
#else
# define unipi_uart_trace(f, args...)
#endif

/********************
 * Data Definitions *
 ********************/

struct neuronspi_uart_data* neuronspi_uart_glob_data;
unsigned long neuronspi_lines;
struct uart_driver* neuronspi_uart;

static struct sched_param neuronspi_sched_param = { .sched_priority = MAX_RT_PRIO / 2 };

/********************
 * Static Functions *
 ********************/
void neuronspi_uart_update_timeout(struct neuronspi_port *n_port, unsigned int cflag, unsigned int baud);

void neuronspi_uart_set_cflag(struct spi_device* spi_dev, u8 port, u32 to)
{
    unipispi_modbus_write_u32(spi_dev, 500, to);
	unipi_uart_trace(KERN_INFO "UNIPI_UART: TERMIOS cflag SET, Dev-CS:%d, to:%08x\n", spi_dev->chip_select, to);
}


static void neuronspi_uart_set_iflags(struct uart_port *port, int to)
{
	struct neuronspi_port *n_port;
	struct spi_device *spi;
	n_port = to_neuronspi_port(port, port);
	spi = neuronspi_s_dev[n_port->dev_index];
	//n_spi = spi_get_drvdata(spi);
	unipi_uart_trace(KERN_INFO "UNIPI_UART: TERMIOS iflag SET: %s\n", to ? "PARMRK" : "0");

    unipispi_modbus_write_register(spi, NEURONSPI_UART_IFLAGS_REGISTER, to);
/*
	write_length = neuronspi_spi_compose_single_register_write(NEURONSPI_UART_IFLAGS_REGISTER, &inp_buf, &outp_buf, to);
	neuronspi___spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 25, 1, 0);
	kfree(inp_buf);
	kfree(outp_buf); */
}

/************************
 * Non-static Functions *
 ************************/
 
u32 neuronspi_spi_uart_get_cflag(struct spi_device* spi_dev, u8 port)
{
    u32 value;
    unipispi_modbus_read_u32(spi_dev, 500, &value);
	unipi_uart_trace_1(KERN_INFO "UNIPI_UART: SPI TERMIOS cflag GET, Dev-CS:%d, val:%08x\n", spi_dev->chip_select, value);
    return value;
}


void neuronspi_uart_set_ldisc(struct uart_port *port, struct ktermios *kterm)
{
	struct neuronspi_port *n_port;
	struct spi_device *spi;
	n_port = to_neuronspi_port(port, port);
	spi = neuronspi_s_dev[n_port->dev_index];
	unipi_uart_trace(KERN_INFO "UNIPI_UART: TERMIOS ldisc SET: dsc=%d\n", kterm->c_line);

    unipispi_modbus_write_register(spi, NEURONSPI_UART_LDISC_REGISTER, kterm->c_line);
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
	unipi_uart_trace(KERN_INFO "UNIPISPI: UART TX Empty\n");
	return TIOCSER_TEMT;
}

u32 neuronspi_uart_get_mctrl(struct uart_port *port)
{
	unipi_uart_trace(KERN_DEBUG "UNIPISPI: UART MCTRL Get\n");
	return TIOCM_DSR | TIOCM_CAR;
}

int	neuronspi_uart_ioctl (struct uart_port *port, unsigned int ioctl_code, unsigned long ioctl_arg)
{
    u32 value;
	struct neuronspi_port *n_port;
	struct spi_device *spi;
	struct neuronspi_driver_data *n_spi;
	n_port = to_neuronspi_port(port, port);
	spi = neuronspi_s_dev[n_port->dev_index];
	n_spi = spi_get_drvdata(spi);
	switch (ioctl_code) {
	case TIOCSETD: {
		unipi_uart_trace(KERN_INFO "UNIPISPI: IOCTL TIOCSETD (processed via set_termios)\n");
		return 1;
	}
	case 0x5481: {
        value = ((ioctl_arg * 1000000) / n_port->baud);
        if (value > 0xffff) value = 0xffff;
		unipi_uart_trace(KERN_INFO "UNIPISPI: IOCTL 0x5481 timeout=%d\n", value);
        unipispi_modbus_write_register(spi, NEURONSPI_UART_TIMEOUT_REGISTER, value);
		return 0;
	}
	case 0x5480: {
        value = (ioctl_arg * 10);
        if (value > 0xffff) value = 0xffff;
		unipi_uart_trace(KERN_INFO "UNIPISPI: IOCTL 0x5480 timeout=%d\n", value);
        unipispi_modbus_write_register(spi, NEURONSPI_UART_TIMEOUT_REGISTER, value);
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

    if (old) {
        unipi_uart_trace(KERN_INFO "UNIPI_UART: Termios port:%d old:0x%04x %04x %04x %04x ldisc:%d", port->line,\
         old->c_cflag, old->c_iflag, \
         old->c_oflag, old->c_lflag, old->c_line);
    }
    if (termios) {
        unipi_uart_trace(KERN_INFO "UNIPI_UART: Termios port:%d new:0x%04x %04x %04x %04x ldisc:%d", port->line,\
         termios->c_cflag, termios->c_iflag, \
         termios->c_oflag, termios->c_lflag, termios->c_line);
    }

	neuronspi_uart_set_cflag(neuronspi_s_dev[n_port->dev_index], n_port->dev_port, termios->c_cflag);
	if (termios && (!old || ((old->c_iflag & PARMRK) != (termios->c_iflag & PARMRK)))) {
		neuronspi_uart_set_iflags(port, termios->c_iflag);
	}
	if (termios && !old) {
        // set line discipline only in case of new setting - Mervis behavior
        neuronspi_uart_set_ldisc(port, termios);
	}
	n_port->baud = uart_get_baud_rate(port, termios, old, 134, 115200);
	uart_update_timeout(port, termios->c_cflag, n_port->baud);
    neuronspi_uart_update_timeout(n_port, termios->c_cflag, n_port->baud);
}

s32 neuronspi_uart_config_rs485(struct uart_port *port, struct serial_rs485 *rs485)
{
	port->rs485 = *rs485;
	return 0;
}


void neuronspi_uart_update_timeout(struct neuronspi_port *n_port, unsigned int cflag, unsigned int baud)
{
	unsigned int bits;

	/* byte size and parity */
	switch (cflag & CSIZE) {
	case CS5:
		bits = 7;
		break;
	case CS6:
		bits = 8;
		break;
	case CS7:
		bits = 9;
		break;
	default:
		bits = 10;
		break; /* CS8 */
	}

	if (cflag & CSTOPB)
		bits++;
	if (cflag & PARENB)
		bits++;
	/*
	 * time in microseconds for sending one character
	 */
	n_port->one_char_usec = (1000000 * bits) / baud;
}

const char* neuronspi_uart_type(struct uart_port *port)
{
	return port->type == PORT_NEURONSPI ? "UNIPISPI_NAME" : NULL;
}

s32 neuronspi_uart_request_port(struct uart_port *port)
{
	unipi_uart_trace(KERN_DEBUG "UNIPISPI: UART requested port %d\n", port->line);
	return 0;
}

void neuronspi_uart_fifo_read(struct uart_port *port, u32 rxlen)
{
	s32 i;
	struct neuronspi_port *s = to_neuronspi_port(port,port);
	struct neuronspi_driver_data *d_data = spi_get_drvdata(neuronspi_s_dev[s->dev_index]);
	unipi_uart_trace_1(KERN_INFO "UNIPISPI: FIFO Read len:%d\n", rxlen);

    memcpy(s->buf, d_data->uart_buf, rxlen);
	for (i = 0; i < rxlen; i++) {
		unipi_uart_trace_2(KERN_INFO "UNIPISPI: UART Char Read: %x\n", d_data->uart_buf[i]);
	}
}

int static neuronspi_uart_get_charcount(struct neuronspi_port *port) 
{
    u16 read_length16;
	struct spi_device *spi;
	struct neuronspi_driver_data *n_spi;
	int ret = 0;
    
	spi = neuronspi_s_dev[port->dev_index];
	n_spi = spi_get_drvdata(spi);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map && n_spi->regstart_table->uart_queue_reg) {
        if (unipispi_modbus_read_register(spi, n_spi->regstart_table->uart_queue_reg, &read_length16) == 0) {
            ret = read_length16;
        }
	}
	unipi_uart_trace(KERN_INFO "UNIPI_UART: GET Char count:%d\n", ret);
	return ret;
}

void neuronspi_uart_fifo_write(struct neuronspi_port *n_port, u8 to_send)
{
	int i, in_queue, need;

	unipi_uart_trace_2(KERN_INFO "UNIPISPI: FIFO Write to_send:%d\n", to_send);
	for (i = 0; i < to_send; i++) {
		unipi_uart_trace_2(KERN_INFO "UNIPISPI: UART Char Send: %x\n", n_port->buf[i]);
	}
    do {
        in_queue = neuronspi_uart_get_charcount(n_port);
        need = (int)to_send - (NEURONSPI_FIFO_SIZE - in_queue);
        if (need <= 0)  break;
        usleep_range(need * n_port->one_char_usec, (need + NEURONSPI_FIFO_SIZE/4) * n_port->one_char_usec);
    } while(1);
    neuronspi_spi_uart_write(neuronspi_s_dev[n_port->dev_index], n_port->buf, to_send, n_port->dev_port);
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
	unsigned long flags;
	u32 ch, flag, bytes_read, i;
	while (rxlen) {
		neuronspi_uart_fifo_read(&port->port, rxlen);
		bytes_read = rxlen;
		spin_lock_irqsave(&port->port.lock, flags);
		port->port.icount.rx++;
		flag = TTY_NORMAL;
		for (i = 0; i < bytes_read; ++i) {
			unipi_uart_trace(KERN_INFO "UNIPISPI: UART Insert Char:%x\n", port->buf[i]);

			ch = port->buf[i];
			if (uart_handle_sysrq_char(port, ch))
				continue;

			uart_insert_char(&port->port, 0, 0, ch, flag);
		}
		rxlen -= bytes_read;
		spin_unlock_irqrestore(&port->port.lock, flags);
	}
	tty_flip_buffer_push(&port->port.state->port);
}

void neuronspi_uart_handle_tx(struct neuronspi_port *port)
{
	s32 max_txlen, to_send, to_send_packet;//, i;
	unsigned long flags;
	struct spi_device *spi;
	struct neuronspi_driver_data *d_data;
	struct circ_buf *xmit;
	int new_tail;

	spi = neuronspi_s_dev[port->dev_index];
	d_data = spi_get_drvdata(spi);

	if (unlikely(port->port.x_char)) {
		neuronspi_spi_uart_write(spi, &port->port.x_char, 1, port->dev_port);
		spin_lock_irqsave(&port->port.lock, flags);
		port->port.icount.tx++;
		spin_unlock_irqrestore(&port->port.lock, flags);
		port->port.x_char = 0;
		return;
	}

	//spin_lock_irqsave(&port->port.lock, flags);
	xmit = &port->port.state->xmit;
	//spin_unlock_irqrestore(&port->port.lock, flags);
	spin_lock_irqsave(&port->port.lock, flags);
	if (uart_circ_empty(xmit) || uart_tx_stopped(&port->port)) {
		spin_unlock_irqrestore(&port->port.lock, flags);
		return;
	}
	spin_unlock_irqrestore(&port->port.lock, flags);

	/* Get length of data pending in circular buffer */
	to_send = uart_circ_chars_pending(xmit);
	unipi_uart_trace(KERN_INFO "UNIPISPI: UART_HANDLE_TX A, to_send:%d", to_send);

	if (likely(to_send)) {
		/* Limit to size of (TX FIFO / 2) */
		max_txlen = NEURONSPI_FIFO_SIZE >> 1;
		while (to_send > 0) {
			to_send_packet = (to_send > max_txlen) ? max_txlen : to_send;

			/* Add data to send */
			port->port.icount.tx += to_send_packet;
			new_tail = (xmit->tail + to_send_packet) & (UART_XMIT_SIZE - 1);
			if (new_tail <= xmit->tail) {
				memcpy(port->buf, xmit->buf+xmit->tail, UART_XMIT_SIZE - xmit->tail);
				memcpy(port->buf+UART_XMIT_SIZE - xmit->tail, xmit->buf, new_tail);
			} else {
				memcpy(port->buf, xmit->buf+xmit->tail, to_send_packet);
			}
			xmit->tail = new_tail;

			/* Convert to linear buffer */
			//for (i = 0; i < to_send_packet; ++i) {
			//	port->buf[i] = xmit->buf[xmit->tail];
			//	xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			//}
			unipi_uart_trace(KERN_INFO "UNIPISPI: UART_HANDLE_TX B, to_send:%d", to_send_packet);
			neuronspi_uart_fifo_write(port, to_send_packet);

			spin_lock_irqsave(&port->port.lock, flags);
			to_send = uart_circ_chars_pending(xmit);
			if (to_send < WAKEUP_CHARS) {
				uart_write_wakeup(&port->port);
			}
			spin_unlock_irqrestore(&port->port.lock, flags);
		}
	}
/*
	spin_lock_irqsave(&port->port.lock, flags);
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		uart_write_wakeup(&port->port);
	}
	spin_unlock_irqrestore(&port->port.lock, flags);
*/
}

void neuronspi_uart_handle_irq(struct neuronspi_uart_data *uart_data, u32 portno)
{
	struct neuronspi_port *n_port = &uart_data->p[portno];
	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];
    struct neuronspi_op_buffer recv_buf;
	neuronspi_spi_send_const_op(spi, &UNIPISPI_IDLE_MESSAGE, &recv_buf, 0, NEURONSPI_DEFAULT_FREQ, 25);
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

	s32 i, j, ret, new_uart_count;
	struct neuronspi_uart_data *uart_data = driver_data->uart_data;

	if (uart_data->p == NULL) {
		uart_data->p = kzalloc(sizeof(struct neuronspi_port[NEURONSPI_MAX_UART]), GFP_ATOMIC);
		for (i = 0; i < NEURONSPI_MAX_UART; i++) {
			uart_data->p[i].parent = uart_data;
		}
		unipi_uart_trace(KERN_DEBUG "UNIPISPI: Allocated port structure for %d potential UART devices", NEURONSPI_MAX_UART);
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
		uart_data->p[i].port.fifosize	= NEURONSPI_FIFO_SIZE*8;
		uart_data->p[i].port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
		uart_data->p[i].port.iotype	= UPIO_PORT;
		uart_data->p[i].port.uartclk	= 9600;
		uart_data->p[i].port.rs485_config = neuronspi_uart_config_rs485;
		uart_data->p[i].port.ops	= &neuronspi_uart_ops;
		uart_data->p[i].port.line	= neuronspi_uart_alloc_line();
		spin_lock_init(&uart_data->p[i].port.lock);
		if (uart_data->p[i].port.line >= NEURONSPI_MAX_DEVS) {
			ret = -ENOMEM;
		}
		kthread_init_work(&(uart_data->p[i].tx_work), neuronspi_uart_tx_proc);
		kthread_init_work(&(uart_data->p[i].rx_work), neuronspi_uart_rx_proc);
		kthread_init_work(&(uart_data->p[i].irq_work), neuronspi_uart_ist);
		uart_add_one_port(driver_data->serial_driver, &uart_data->p[i].port);
		unipi_uart_trace(KERN_INFO "UNIPISPI: Added UART port %d\n", i);
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
				unipi_uart_trace(KERN_DEBUG "UNIPISPI: Renumber not NULL %d UC:%d\n", i, driver_data->uart_count);
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
							unipi_uart_trace(KERN_DEBUG "UNIPISPI: Added UART port %d\n", j);
						}
					}
				}
			}
		}
	}

	uart_data->p_count = new_uart_count;
	if (uart_data->kworker_task == NULL) {
		unipi_uart_trace(KERN_DEBUG "UNIPISPI: KWorker Task is NULL\n");

		kthread_init_worker(&uart_data->kworker);

		uart_data->kworker_task = kthread_run(kthread_worker_fn, &uart_data->kworker,
						  "neuronspi");
		if (IS_ERR(uart_data->kworker_task)) {
			ret = PTR_ERR(uart_data->kworker_task);
		}
		sched_setscheduler(uart_data->kworker_task, SCHED_FIFO, &neuronspi_sched_param);
	}
    
    unipi_uart_trace(KERN_DEBUG "UNIPISPI: UART PROBE MCTRL:%d\n", neuronspi_spi_uart_get_cflag(dev, 0));
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

	//u8 *send_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_ATOMIC);
	u8 *recv_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_ATOMIC);

	mutex_lock(&neuronspi_master_mutex);
	read_count = d_data->uart_read;
	mutex_unlock(&neuronspi_master_mutex);

	while (!end_flag) {
		memset(recv_buf, 0, NEURONSPI_BUFFER_MAX);
		//neuronspi_spi_uart_read(spi, send_buf, recv_buf, read_count, n_port->dev_port);
		neuronspi_spi_uart_read(spi, recv_buf, read_count, n_port->dev_port);
		if (((recv_buf[0] == 0x65) || (recv_buf[0] == 0x68))) {
 			if (recv_buf[1] > 0) {
				mutex_lock(&neuronspi_master_mutex);
				memcpy(&d_data->uart_buf[0], &recv_buf[4], recv_buf[1]);
				mutex_unlock(&neuronspi_master_mutex);
				neuronspi_uart_handle_rx(n_port, recv_buf[1], 1);
			}
			//mutex_lock(&neuronspi_master_mutex);
			if ((read_count == 0) && ( recv_buf[3] == 0)) {
				mutex_lock(&neuronspi_master_mutex);
				d_data->uart_read = 0;
				end_flag = 1;
				mutex_unlock(&neuronspi_master_mutex);
			} else {
				read_count = recv_buf[3];
			}
			//mutex_unlock(&neuronspi_master_mutex);
		} else { //if ((recv_buf[0]&0xfd) != 0x41) {
			mutex_lock(&neuronspi_master_mutex);
			d_data->uart_read = 0;
			end_flag = 1;
			mutex_unlock(&neuronspi_master_mutex);
		}
	}
	kfree(recv_buf);
	//kfree(send_buf);
}

void neuronspi_uart_start_tx(struct uart_port *port)
{
	struct neuronspi_port *n_port = to_neuronspi_port(port,port);
	unipi_uart_trace(KERN_INFO "UNIPISPI: Start TX\n");
	if (!kthread_queue_work(&n_port->parent->kworker, &n_port->tx_work)) {
		unipi_uart_trace(KERN_INFO "UNIPISPI: TX WORK OVERFLOW\n");
	}
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
		wake_up_process(d_data->poll_thread);
	}
	neuronspi_uart_power(port, 1);
	// TODO: /* Reset FIFOs*/
	unipi_uart_trace(KERN_DEBUG "UNIPISPI: UART Startup\n");
	return 0;
}

void neuronspi_uart_shutdown(struct uart_port *port)
{
    neuronspi_uart_power(port, 0);
}

/*******************
 * Empty functions *
 *******************/

void neuronspi_uart_power(struct uart_port *port, s32 on)
{
    /* Do nothing */
}
void neuronspi_uart_set_mctrl(struct uart_port *port, u32 mctrl)
{
    /* Do nothing */
}
void neuronspi_uart_break_ctl(struct uart_port *port, int break_state)
{
    /* Do nothing */
}
void neuronspi_uart_null_void(struct uart_port *port)
{
	/* Do nothing */
}
