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
# define unipi_uart_trace_2(f, args...)	printk(KERN_INFO "UNIPIUART: " f, ##args)
#else
# define unipi_uart_trace_2(f, args...)
#endif

#if NEURONSPI_DETAILED_DEBUG > 1
# define unipi_uart_trace_1(f, args...)	printk(KERN_INFO "UNIPIUART: " f, ##args)
#else
# define unipi_uart_trace_1(f, args...)
#endif

#if NEURONSPI_DETAILED_DEBUG > 0
# define unipi_uart_trace(f, args...)	printk(KERN_INFO "UNIPIUART: " f, ##args)
#else
# define unipi_uart_trace(f, args...)
#endif

/********************
 * Data Definitions *
 ********************/

struct neuronspi_uart_data  *neuronspi_uart_data_global = NULL;
struct uart_driver          *neuronspi_uart_driver_global = NULL;


/********************
 * Static Functions *
 ********************/

void neuronspi_uart_update_timeout(struct neuronspi_port *n_port, unsigned int cflag, unsigned int baud);

#define NEURONSPI_UART_CFLAGS_REGISTER 	500
#define NEURONSPI_UART_IFLAGS_REGISTER 	502
#define NEURONSPI_UART_LDISC_REGISTER 	503
#define NEURONSPI_UART_TIMEOUT_REGISTER 504
#define NEURONSPI_UART_FIFO_REGISTER    505

static inline int port_to_uartregs(u8 port, u16 reg)
{
    return reg + ((port==0) ? 0 : (10*(port+1)));
}

static void neuronspi_uart_set_cflag(struct neuronspi_port *n_port, u32 to)
{
	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];
    unipispi_modbus_write_u32(spi, port_to_uartregs(n_port->dev_port, NEURONSPI_UART_CFLAGS_REGISTER), to);
	unipi_uart_trace("ttyNS%d Set cflag: %08x\n", n_port->port.line, to);
}


static void neuronspi_uart_set_iflags(struct neuronspi_port *n_port, int to)
{
	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];
	unipi_uart_trace("ttyNS%d Set iflag: %s\n", n_port->port.line, (to & PARMRK)? "PARMRK" : "0");

    unipispi_modbus_write_register(spi, port_to_uartregs(n_port->dev_port, NEURONSPI_UART_IFLAGS_REGISTER), to);
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

void neuronspi_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_NEURONSPI;
	}
}

s32 neuronspi_uart_verify_port(struct uart_port *port, struct serial_struct *s)
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

/************************
 * Non-static Functions *
 ************************/
 
u32 neuronspi_spi_uart_get_cflag(struct spi_device* spi_dev, u8 port)
{
    u32 value;
    unipispi_modbus_read_u32(spi_dev, port_to_uartregs(port, NEURONSPI_UART_CFLAGS_REGISTER), &value);
	unipi_uart_trace_1("Get cflag val:%08x\n", value);
    return value;
}

void neuronspi_uart_set_ldisc(struct uart_port *port, struct ktermios *kterm)
{
	struct neuronspi_port *n_port = to_neuronspi_port(port, port);
	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];

	unipi_uart_trace("ttyNS%d Set ldisc: dsc=%d\n", port->line, kterm->c_line);
    unipispi_modbus_write_register(spi, port_to_uartregs(n_port->dev_port, NEURONSPI_UART_LDISC_REGISTER), kterm->c_line);
}

void neuronspi_uart_flush_buffer(struct uart_port* port)
{ 
    //port->lock taken, This call must not sleep
    // ToDo :    
	unipi_uart_trace("ttyNS%d Flush buffer\n", port->line);
}

u32 neuronspi_uart_tx_empty(struct uart_port *port)
{
	struct neuronspi_port *n_port = to_neuronspi_port(port, port);
	unipi_uart_trace("ttyNS%d Tx empty? %s\n", port->line, (n_port->tx_fifo_len==0)?"Yes":"No");
	return (n_port->tx_fifo_len==0) ? TIOCSER_TEMT : 0;
}

u32 neuronspi_uart_get_mctrl(struct uart_port *port)
{
	unipi_uart_trace_1("ttyNS%d Get mctrl\n", port->line);
	return TIOCM_DSR | TIOCM_CAR;
}

int	neuronspi_uart_ioctl (struct uart_port *port, unsigned int ioctl_code, unsigned long ioctl_arg)
{
    u32 value;
	struct neuronspi_port *n_port = to_neuronspi_port(port, port);
	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];

	switch (ioctl_code) {
	case TIOCSETD: {
		unipi_uart_trace("ttyNS%d Ioctl TIOCSETD (processed via set_termios)\n", port->line);
		return 1;
	}
	case 0x5481: {
        value = ((ioctl_arg * 1000000) / n_port->baud);
        if (value > 0xffff) value = 0xffff;
		unipi_uart_trace("ttyNS%d Ioctl 0x5481 set timeout=%d\n", port->line, value);
        unipispi_modbus_write_register(spi, port_to_uartregs(n_port->dev_port, NEURONSPI_UART_TIMEOUT_REGISTER), value);
		return 0;
	}
	case 0x5480: {
        value = (ioctl_arg * 10);
        if (value > 0xffff) value = 0xffff;
		unipi_uart_trace("ttyNS%d Ioctl 0x5480 set timeout=%d\n", port->line, value);
        unipispi_modbus_write_register(spi, port_to_uartregs(n_port->dev_port, NEURONSPI_UART_TIMEOUT_REGISTER), value);
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
        unipi_uart_trace("ttyNS%d Termios old:0x%04x %04x %04x %04x ldisc:%d", port->line,\
         old->c_cflag, old->c_iflag, \
         old->c_oflag, old->c_lflag, old->c_line);
    }
    if (termios) {
        unipi_uart_trace("ttyNS%d Termios new:0x%04x %04x %04x %04x ldisc:%d", port->line,\
         termios->c_cflag, termios->c_iflag, \
         termios->c_oflag, termios->c_lflag, termios->c_line);
    }

	neuronspi_uart_set_cflag(n_port, termios->c_cflag);
	if (termios && (!old || ((old->c_iflag & PARMRK) != (termios->c_iflag & PARMRK)))) {
		neuronspi_uart_set_iflags(n_port, termios->c_iflag);
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
	 * time in nanoseconds for sending one character
	 */
	n_port->one_char_nsec = (((long)1000000 * bits) / baud)*1000;
}

const char* neuronspi_uart_type(struct uart_port *port)
{
	return port->type == PORT_NEURONSPI ? "UNIPISPI_NAME" : NULL;
}

s32 neuronspi_uart_request_port(struct uart_port *port)
{
	unipi_uart_trace("ttyNS%d Request port\n", port->line);
	return 0;
}


void neuronspi_uart_start_tx(struct uart_port *port)
{
	struct neuronspi_port *n_port = to_neuronspi_port(port,port);
	unipi_uart_trace("Start TX\n");

	if (!kthread_queue_work(n_port->n_spi->primary_worker, &n_port->tx_work)) {
		//unipi_uart_trace("TX WORK OVERFLOW\n");
	}
}

// set port->tx_fifo_len by reading modbus register
// return 0 if success, 1 if couldnt read register - fifo len has old value
int static neuronspi_uart_read_tx_fifo_len(struct neuronspi_port *port) 
{
    u16 read_length16 = - 1;
	struct spi_device *spi;
	int ret = 1;
    
    if (port->tx_fifo_reg) {
        spi = neuronspi_s_dev[port->dev_index];
        if (unipispi_modbus_read_register(spi, port->tx_fifo_reg, &read_length16) == 0) {
            port->tx_fifo_len = read_length16;
            ret = 0;
        }
	} else {
        // unknown port!
        port->tx_fifo_len = 0;
        ret = 0;
    }
	unipi_uart_trace("ttyNS%d Get tx fifo len:%d\n", port->port.line, read_length16);
	return ret;
}


void neuronspi_rx_queue_clear(struct neuronspi_port *port, u8 data)
{
    port->rx_qlen_secondary = 0;
}

int neuronspi_rx_queue_add(struct neuronspi_port *port, u8 data)
{
	unsigned long flags;
    int ret = 1;
    spin_lock_irqsave(&port->rx_queue_lock, flags);
    if (port->rx_qlen_primary < MAX_RX_QUEUE_LEN) {
        port->rx_queue_primary[port->rx_qlen_primary++] = data;
        ret = 0;
    }
    spin_unlock_irqrestore(&port->rx_queue_lock, flags);
    return ret;
}

void neuronspi_rx_queue_swap(struct neuronspi_port *port)
{
	unsigned long flags;
    u8* x;
    
    spin_lock_irqsave(&port->rx_queue_lock, flags);
    x = port->rx_queue_primary;
    port->rx_queue_primary = port->rx_queue_secondary;
    port->rx_queue_secondary = x;
    port->rx_qlen_secondary = port->rx_qlen_primary;
    port->rx_qlen_primary = 0;
    spin_unlock_irqrestore(&port->rx_queue_lock, flags);
}

static void neuronspi_uart_handle_rx(struct neuronspi_port *port, int rxlen, u8* pbuf)
{
	unsigned long flags;
	u32 ch, flag, bytes_read, i;
    
    unipi_uart_trace("ttyNS%d Insert Chars (%d): %16ph\n", port->port.line, rxlen, pbuf);

	while (rxlen) {
		bytes_read = rxlen;
		spin_lock_irqsave(&port->port.lock, flags);
		port->port.icount.rx++;
		flag = TTY_NORMAL;
		for (i = 0; i < bytes_read; ++i) {

			ch = *pbuf;
            pbuf++;
			if (uart_handle_sysrq_char(port, ch))
				continue;

			uart_insert_char(&port->port, 0, 0, ch, flag);
		}
		rxlen -= bytes_read;
		spin_unlock_irqrestore(&port->port.lock, flags);
	}
	tty_flip_buffer_push(&port->port.state->port);
}


#define start_tx_timer(port, lowlimit, delta) hrtimer_start_range_ns(&port->tx_timer, lowlimit * port->one_char_nsec, delta*port->one_char_nsec, HRTIMER_MODE_REL)

#define MAX_TXLEN	(NEURONSPI_FIFO_SIZE >> 1)

void neuronspi_uart_handle_tx(struct neuronspi_port *port)
{
	int to_send, to_send_packet, ret, need;
	u8	tx_buf[MAX_TXLEN + 16];
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
        kthread_queue_work(port->n_spi->primary_worker, &port->tx_work);
		return;
	}

	xmit = &port->port.state->xmit;
	spin_lock_irqsave(&port->port.lock, flags);
	if (uart_circ_empty(xmit) || uart_tx_stopped(&port->port)) {
		spin_unlock_irqrestore(&port->port.lock, flags);
        // check tx_fifo status
        if (port->tx_fifo_len) {
            ret = neuronspi_uart_read_tx_fifo_len(port);
            if (ret || port->tx_fifo_len) {
                // set timer to check tx_empty
                unipi_uart_trace_1("ttyNS%d Handle TX. Start timer=%llu", port->port.line, port->tx_fifo_len * port->one_char_nsec);
                start_tx_timer(port, port->tx_fifo_len, 2);
            }
        }
		return;
	}
	spin_unlock_irqrestore(&port->port.lock, flags);

	// Get length of data pending in circular buffer
	to_send = uart_circ_chars_pending(xmit);
	unipi_uart_trace("ttyNS%d Handle TX. tty->pending=%d", port->port.line, to_send);

	if (likely(to_send)) {
		
		// Limit to size of (TX FIFO / 2)
		to_send_packet = (to_send > MAX_TXLEN) ? MAX_TXLEN : to_send;
        if (port->tx_fifo_len + to_send_packet > NEURONSPI_FIFO_SIZE) {
                // read current tx_fifo_len
                ret = neuronspi_uart_read_tx_fifo_len(port);
                need = to_send_packet - (NEURONSPI_FIFO_SIZE - port->tx_fifo_len);
                if ((ret != 0) || (need > 0)) {
                    // reschedule work with pause
                    start_tx_timer(port, need, NEURONSPI_FIFO_SIZE/4);
                    return;
                }
        }
            
		/* Read data from tty buffer and send it to spi */
        spin_lock_irqsave(&port->port.lock, flags);
		port->port.icount.tx += to_send_packet;
		new_tail = (xmit->tail + to_send_packet) & (UART_XMIT_SIZE - 1);
		if (new_tail <= xmit->tail) {
			memcpy(tx_buf, xmit->buf+xmit->tail, UART_XMIT_SIZE - xmit->tail);
			memcpy(tx_buf+UART_XMIT_SIZE - xmit->tail, xmit->buf, new_tail);
		} else {
			memcpy(tx_buf, xmit->buf+xmit->tail, to_send_packet);
		}
		xmit->tail = new_tail;

        spin_unlock_irqrestore(&port->port.lock, flags);

        unipi_uart_trace("ttyNS%d Handle TX Send: %d %16ph\n", port->port.line, to_send_packet, tx_buf);
        neuronspi_spi_uart_write(spi, tx_buf, to_send_packet, port->dev_port);
        port->tx_fifo_len += to_send_packet;
           
		spin_lock_irqsave(&port->port.lock, flags);
		to_send = uart_circ_chars_pending(xmit);
		if (to_send < WAKEUP_CHARS) {
			uart_write_wakeup(&port->port);
		}
		spin_unlock_irqrestore(&port->port.lock, flags);
        if (to_send) {
            // reschedule work
			kthread_queue_work(port->n_spi->primary_worker, &port->tx_work);
		} else {
            // set timer to check tx_empty
            unipi_uart_trace_1("ttyNS%d Handle TX. Start timer=%llu", port->port.line, to_send_packet * port->one_char_nsec);
            start_tx_timer(port, to_send_packet, 2);
        }
	}
}

// callback of tx_timer. Schedule port->tx_work
static enum hrtimer_restart neuronspi_uart_timer_func(struct hrtimer *timer)
{
    struct neuronspi_port* n_port = ((container_of((timer), struct neuronspi_port, tx_timer)));

	kthread_queue_work(n_port->n_spi->primary_worker, &n_port->tx_work);
	return HRTIMER_NORESTART;
}


void neuronspi_uart_tx_proc(struct kthread_work *ws)
{
	struct neuronspi_port *port = to_neuronspi_port(ws, tx_work);
/*
	if ((port->port.rs485.flags & SER_RS485_ENABLED) &&
	    (port->port.rs485.delay_rts_before_send > 0)) {
		msleep(port->port.rs485.delay_rts_before_send);
	}
*/
	neuronspi_uart_handle_tx(port);
}

void neuronspi_uart_rx_proc(struct kthread_work *ws)
{
	struct neuronspi_port *n_port = to_neuronspi_port(ws, rx_work);
	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];

	u8 *recv_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_ATOMIC);

    if (n_port->rx_qlen_secondary) {
        //send to tty and clear secondary queue
        neuronspi_uart_handle_rx(n_port, n_port->rx_qlen_secondary, n_port->rx_queue_secondary);
        n_port->rx_qlen_secondary = 0;
    }
    
    #if NEURONSPI_DETAILED_DEBUG > 0
        memset(recv_buf, 0, NEURONSPI_BUFFER_MAX); 
    #endif

    neuronspi_spi_uart_read(spi, recv_buf, n_port->rx_remain, n_port->dev_port);

    if (recv_buf[0] == 0x65) {
        if (n_port->rx_qlen_secondary) {
            neuronspi_uart_handle_rx(n_port, n_port->rx_qlen_secondary, n_port->rx_queue_secondary);
            n_port->rx_qlen_secondary = 0;
        }
 		if (recv_buf[1] > 0) {
            // send string to tty
            neuronspi_uart_handle_rx(n_port,  recv_buf[1], recv_buf+4);
        }
    }
    neuronspi_rx_queue_swap(n_port);
    if (n_port->rx_qlen_secondary) {
        //send to tty and clear secondary queue
        neuronspi_uart_handle_rx(n_port, n_port->rx_qlen_secondary, n_port->rx_queue_secondary);
        n_port->rx_qlen_secondary = 0;
    }

    if (n_port->rx_remain > 0) {
		kthread_queue_work(n_port->n_spi->primary_worker, &n_port->rx_work);
    }
	kfree(recv_buf);
}


// Initialise the driver - called once on open
s32 neuronspi_uart_startup(struct uart_port *port)
{
	struct neuronspi_port *n_port = to_neuronspi_port(port, port);
	neuronspi_enable_uart_interrupt(n_port);
	neuronspi_uart_power(port, 1);
	// TODO: /* Reset FIFOs*/
    unipi_uart_trace("ttyNS%d Startup\n", port->line);
	return 0;
}


void neuronspi_uart_shutdown(struct uart_port *port)
{
    unipi_uart_trace("ttyNS%d Shutdown\n", port->line);
    neuronspi_uart_power(port, 0);
}


void neuronspi_uart_remove(struct spi_device* spi)
{
    struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
    struct neuronspi_port *port; 
	int i, uart_count;
    
    uart_count = n_spi->uart_count;
    n_spi->uart_count = 0;

	for (i = 0; i < uart_count; i++) {
        port = neuronspi_uart_data_global->p + i + n_spi->uart_pindex;
        hrtimer_cancel(&port->tx_timer);
		uart_remove_one_port(neuronspi_uart_driver_global, &port->port);
        kthread_flush_work(&(port->rx_work));
        kthread_flush_work(&(port->tx_work));

		neuronspi_uart_power(&port->port, 0);
        kfree(port->rx_queue_primary);
        kfree(port->rx_queue_secondary);
        printk(KERN_INFO "UNIPIUART: Serial port ttyNS%d removed\n", i + n_spi->uart_pindex);
	}
}


static const struct uart_ops neuronspi_uart_ops =
{
	.tx_empty			= neuronspi_uart_tx_empty,
	.set_mctrl			= neuronspi_uart_set_mctrl,
	.get_mctrl			= neuronspi_uart_get_mctrl,
	.stop_tx			= neuronspi_uart_null_void,
	.start_tx			= neuronspi_uart_start_tx,
	.stop_rx			= neuronspi_uart_null_void,
	.flush_buffer		= neuronspi_uart_flush_buffer,
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


int neuronspi_uart_probe(struct spi_device* spi, struct neuronspi_driver_data *n_spi)
{
    struct neuronspi_port* port;
    int i, ret = 0;
    
    if (n_spi->uart_count_to_probe) {
        n_spi->uart_pindex = neuronspi_uart_data_global->p_count;
        for (i=0; i<n_spi->uart_count_to_probe; i++) {
            // port is pointer to item ->p[x]
            port = neuronspi_uart_data_global->p + neuronspi_uart_data_global->p_count;

            port->port.dev	= &(spi->dev);
            port->dev_index = n_spi->neuron_index;
            port->dev_port  = i; 
            port->n_spi     = n_spi; // shorthand to n_spi

            port->port.line	= neuronspi_uart_data_global->p_count;
            port->port.irq	= spi->irq;
            port->port.type	= PORT_NEURONSPI;
            port->port.fifosize	= NEURONSPI_FIFO_SIZE*8;
            port->port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
            port->port.iotype	= UPIO_PORT;
            port->port.uartclk	= 9600;
            port->port.rs485_config = neuronspi_uart_config_rs485;
            port->port.ops	= &neuronspi_uart_ops;
            spin_lock_init(&port->port.lock);

            spin_lock_init(&port->rx_queue_lock);
            port->rx_queue_primary = kzalloc(MAX_RX_QUEUE_LEN, GFP_ATOMIC);   
            port->rx_queue_secondary = kzalloc(MAX_RX_QUEUE_LEN, GFP_ATOMIC); 

            port->tx_fifo_len = 0x7fff; //set it to big number; invoke reading current value from Neuron
            if (n_spi && (n_spi->firmware_version >= 0x0519) ) {
                port->tx_fifo_reg = port_to_uartregs(i,NEURONSPI_UART_FIFO_REGISTER);        // define modbus register
            } else if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map && n_spi->regstart_table->uart_queue_reg) {
                port->tx_fifo_reg = n_spi->regstart_table->uart_queue_reg;      // define modbus register
            }

            hrtimer_init(&port->tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
            port->tx_timer.function = neuronspi_uart_timer_func;
            
            kthread_init_work(&(port->tx_work), neuronspi_uart_tx_proc);
            kthread_init_work(&(port->rx_work), neuronspi_uart_rx_proc);
            uart_add_one_port(neuronspi_uart_driver_global, &port->port);
            printk(KERN_INFO "UNIPIUART: Serial port ttyNS%d on UniPi Board nspi%d port:%d created\n", neuronspi_uart_data_global->p_count, port->dev_index, port->dev_port);
            unipi_uart_trace("Probe cflag:%08x\n", neuronspi_spi_uart_get_cflag(spi, i));

            n_spi->uart_count++;

            if (neuronspi_uart_data_global->p_count < NEURONSPI_MAX_UART) {
                neuronspi_uart_data_global->p_count++;
            } else {
                printk(KERN_INFO "UNIPIUART: PROBE maximum UART devices reached!\n");
                ret = 1;
                break;
            }
        }
        n_spi->uart_count_to_probe = 0;
    }
    return ret;
}

int neuronspi_uart_probe_all(void)
{
	struct spi_device* spi;
    struct neuronspi_driver_data* n_spi;// = spi_get_drvdata(dev);
	int si, ret=0;
    
    for (si=0; si < NEURONSPI_MAX_DEVS; si++) {
        spi = neuronspi_s_dev[si];
        if (spi == NULL) {
            continue;
        }
        n_spi = spi_get_drvdata(spi);
        if (n_spi->uart_count_to_probe == 0) continue;
        
        if (neuronspi_uart_data_global->p == NULL) {
            
            neuronspi_uart_data_global->p = kzalloc(sizeof(struct neuronspi_port[NEURONSPI_MAX_UART]), GFP_ATOMIC);
            unipi_uart_trace("Allocated port structure for %d ttyNS devices", NEURONSPI_MAX_UART);
        }
        
        ret = neuronspi_uart_probe(spi, n_spi);
        if (ret)  break; // max number of uarts reached
	}
	return ret;
}


int neuronspi_uart_driver_init(void)
{
    int ret;
	if (neuronspi_uart_driver_global != NULL) {
        return 0;
    }
    // Register UART if not registered
	neuronspi_uart_driver_global = kzalloc(sizeof(struct uart_driver), GFP_ATOMIC);
	neuronspi_uart_driver_global->owner		= THIS_MODULE;
	neuronspi_uart_driver_global->dev_name	= "ttyNS";
	neuronspi_uart_driver_global->driver_name = "ttyNS";
	neuronspi_uart_driver_global->nr	= NEURONSPI_MAX_UART;
	ret = uart_register_driver(neuronspi_uart_driver_global);
	if (ret) {
        printk(KERN_ERR "UNIPIUART: Failed to register the neuronspi uart driver, ERR:%d\n", ret);
        kfree(neuronspi_uart_driver_global);
        neuronspi_uart_driver_global = NULL;
        return ret;
	}
	unipi_uart_trace("UART driver registered successfully!\n");
	if (neuronspi_uart_data_global != NULL) {
		printk(KERN_ERR "UNIPIUART: Uart data already allocated!\n");
        return 0;
	}
	neuronspi_uart_data_global = kzalloc(sizeof(struct neuronspi_uart_data), GFP_ATOMIC);
	unipi_uart_trace("UART driver data allocated successfully!\n");
    return 0;
}

int neuronspi_uart_driver_exit(void)
{
    int i;
	if (neuronspi_uart_driver_global) {
        for (i = 0; i < NEURONSPI_MAX_DEVS; i++) {
            if (neuronspi_s_dev[i] != NULL) {
                neuronspi_uart_remove(neuronspi_s_dev[i]);
            }
        }
		uart_unregister_driver(neuronspi_uart_driver_global);
        if (neuronspi_uart_data_global->p) 
            kfree(neuronspi_uart_data_global->p);
		kfree(neuronspi_uart_driver_global);
		kfree(neuronspi_uart_data_global);
	}
    return 0;
}
