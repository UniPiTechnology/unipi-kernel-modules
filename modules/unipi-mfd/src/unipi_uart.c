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
#include "unipi_mfd.h"
#include "unipi_mfd_iogroup.h"

#define UNIPI_UART_DETAILED_DEBUG 0
#if UNIPI_UART_DETAILED_DEBUG > 2
# define unipi_uart_trace_2(f, args...)	printk(KERN_INFO "UNIPIUART: " f, ##args)
#else
# define unipi_uart_trace_2(f, args...)
#endif

#if UNIPI_UART_DETAILED_DEBUG > 1
# define unipi_uart_trace_1(f, args...)	printk(KERN_INFO "UNIPIUART: " f, ##args)
#else
# define unipi_uart_trace_1(f, args...)
#endif

#if UNIPI_UART_DETAILED_DEBUG > 0
# define unipi_uart_trace(f, args...)	printk(KERN_INFO "UNIPIUART: " f, ##args)
#else
# define unipi_uart_trace(f, args...)
#endif

/********************
 * Data Definitions *
 ********************/

#define PORT_UNIPI			184
#define UNIPI_UART_MAX_NR	16
#define UNIPI_UART_FIFO_SIZE	256


struct unipi_uart_port
{
	struct uart_port			port;
	int							dev_port;    // index of port on mfd 0..3
    u8                          rx_remain;
    int                         accept_rx;
	struct kthread_work			flush_work;
    
	struct kthread_work			tx_work;
    u16                         tx_fifo_reg;  // register in mfd modbus map to read internal tx fifo length
                                              // 0 if undefined
    u16                         tx_fifo_len;  // estimates char count in neuron internal tx fifo
	struct hrtimer				tx_timer;

	s32							baud;
    s64                         one_char_nsec;

    spinlock_t                  rx_in_progress_lock;
    int                         rx_in_progress;
    u8                          rx_send_msg[UNIPI_UART_FIFO_SIZE+4+8];
    u8                          rx_recv_msg[UNIPI_UART_FIFO_SIZE+4+8];

    spinlock_t                  txop_lock;
    int                         pending_txop;
    u8                          tx_send_msg[UNIPI_UART_FIFO_SIZE+4+8];
    u8                          tx_recv_msg[UNIPI_UART_FIFO_SIZE+4+8];
};

struct unipi_uart_device
{
	struct regmap		*regmap;
	int					port_count;
	struct unipi_uart_port	p[0];
};


struct uart_driver unipi_uart = 
{
	.owner		= THIS_MODULE,
	.driver_name= "unipi_tty",
	.dev_name	= "ttyNS",
	.nr			= UNIPI_UART_MAX_NR,
};
static DECLARE_BITMAP(unipi_uart_lines, UNIPI_UART_MAX_NR);

/********************
 * Static Functions *
 ********************/
#define to_unipi_uart_port(p,e)	((container_of((p), struct unipi_uart_port, e)))

static void unipi_uart_update_timeout(struct unipi_uart_port *n_port, unsigned int cflag, unsigned int baud);
int unipi_uart_get_tx_fifo(struct unipi_uart_port* n_port);
void unipi_uart_handle_tx(struct unipi_uart_port *port, int calling);


#define NEURONSPI_UART_FIFO_REGISTER    505

static inline int port_to_uartregs(u8 port, u16 reg)
{
    return reg + ((port==0) ? 0 : (10*(port+1)));
}

static void unipi_uart_set_cflag(struct unipi_uart_port *n_port, u32 to)
{
	struct unipi_uart_device *n_uart = dev_get_drvdata(n_port->port.dev);
	regmap_bulk_write(n_uart->regmap, port_to_uartregs(n_port->dev_port, UNIPI_MFD_REG_UART0_CFLAGS), &to, 2);
	unipi_uart_trace("ttyNS%d Set cflag: %08x\n", n_port->port.line, to);
}


static void unipi_uart_set_iflags(struct unipi_uart_port *n_port, int to)
{
	struct unipi_uart_device *n_uart = dev_get_drvdata(n_port->port.dev);
	unipi_uart_trace("ttyNS%d Set iflag: %s\n", n_port->port.line, (to & PARMRK)? "PARMRK" : "0");
	regmap_write(n_uart->regmap, port_to_uartregs(n_port->dev_port, UNIPI_MFD_REG_UART0_IFLAGS), to);
}


/*******************
 * Empty functions *
 *******************/

void unipi_uart_power(struct uart_port *port, s32 on)
{
    /* Do nothing */
}
void unipi_uart_set_mctrl(struct uart_port *port, u32 mctrl)
{
    /* Do nothing */
}
void unipi_uart_break_ctl(struct uart_port *port, int break_state)
{
    /* Do nothing */
}
void unipi_uart_null_void(struct uart_port *port)
{
	/* Do nothing */
}

void unipi_uart_stop_rx(struct uart_port *port)
{
	struct unipi_uart_port *n_port = to_unipi_uart_port(port, port);
    /*unsigned long flags;*/

    /*spin_lock_irqsave(&port->lock, flags);*/
    n_port->accept_rx = 0;
    /*spin_unlock_irqrestore(&port->lock, flags);*/
}

void unipi_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_UNIPI;
	}
}

int unipi_uart_verify_port(struct uart_port *port, struct serial_struct *s)
{
	if ((s->type != PORT_UNKNOWN) && (s->type != PORT_UNIPI))
		return -EINVAL;
	if (s->irq != port->irq)
		return -EINVAL;

	return 0;
}

void unipi_uart_pm(struct uart_port *port, u32 state, u32 oldstate)
{
	unipi_uart_power(port, (state == UART_PM_STATE_ON) ? 1 : 0);
}

/************************
 * Non-static Functions *
 ************************/
 
u32 unipi_spi_uart_get_cflag(struct regmap *map, u8 port)
{
    u32 value;
    regmap_bulk_read(map, port_to_uartregs(port, UNIPI_MFD_REG_UART0_CFLAGS), &value, 2);
	unipi_uart_trace_1("Get cflag val:%08x\n", value);
    return value;
}

void unipi_uart_set_ldisc(struct uart_port *port, struct ktermios *kterm)
{
	struct unipi_uart_port *n_port = to_unipi_uart_port(port, port);
	struct unipi_uart_device *n_uart = dev_get_drvdata(port->dev);
	unipi_uart_trace("ttyNS%d Set ldisc: dsc=%d\n", port->line, kterm->c_line);
	regmap_write(n_uart->regmap, port_to_uartregs(n_port->dev_port, UNIPI_MFD_REG_UART0_LDISC), kterm->c_line);
}


u32 neuronspi_uart_tx_empty(struct uart_port *port)
{
	struct unipi_uart_port *n_port = to_unipi_uart_port(port, port);
    int len = n_port->tx_fifo_len;
    unsigned long flags;

	if (len > 0 && !n_port->pending_txop) {
		spin_lock_irqsave(&n_port->txop_lock, flags);
		if (!n_port->pending_txop) {
			n_port->pending_txop = 1;
			spin_unlock_irqrestore(&n_port->txop_lock, flags);
			if (unipi_uart_get_tx_fifo(n_port) != 0) {
				n_port->pending_txop = 0; // ERROR
			}
		} else {
			spin_unlock_irqrestore(&n_port->txop_lock, flags);
		}
	}
	unipi_uart_trace("ttyNS%d Tx empty? %s\n", port->line, (len==0)?"Yes":"No");
	return (n_port->tx_fifo_len==0) ? TIOCSER_TEMT : 0;
}

u32 unipi_uart_get_mctrl(struct uart_port *port)
{
	unipi_uart_trace_1("ttyNS%d Get mctrl\n", port->line);
	return TIOCM_DSR | TIOCM_CAR;
}

int	unipi_uart_ioctl(struct uart_port *port, unsigned int ioctl_code, unsigned long ioctl_arg)
{
    u32 value;
	struct unipi_uart_port *n_port = to_unipi_uart_port(port, port);
	struct unipi_uart_device *n_uart = dev_get_drvdata(port->dev);

	switch (ioctl_code) {
	case TIOCSETD: {
		unipi_uart_trace("ttyNS%d Ioctl TIOCSETD (processed via set_termios)\n", port->line);
		return 1;
	}
	case 0x5481: {
        value = ((ioctl_arg * 1000000) / n_port->baud);
        if (value > 0xffff) value = 0xffff;
		unipi_uart_trace("ttyNS%d Ioctl 0x5481 set timeout=%d\n", port->line, value);
        regmap_write(n_uart->regmap, port_to_uartregs(n_port->dev_port, UNIPI_MFD_REG_UART0_TIMEOUT), value);
		return 0;
	}
	case 0x5480: {
        value = (ioctl_arg * 10);
        if (value > 0xffff) value = 0xffff;
		unipi_uart_trace("ttyNS%d Ioctl 0x5480 set timeout=%d\n", port->line, value);
        regmap_write(n_uart->regmap, port_to_uartregs(n_port->dev_port, UNIPI_MFD_REG_UART0_TIMEOUT), value);
		return 0;
	}
	default: {
		return -ENOIOCTLCMD;
	}
	}
}


void unipi_uart_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
	struct unipi_uart_port *n_port = to_unipi_uart_port(port, port);

    if (old) {
        unipi_uart_trace("ttyNS%d Termios old:0x%04x %04x %04x %04x ldisc:%d", port->line,\
         old->c_cflag, old->c_iflag, \
         old->c_oflag, old->c_lflag, old->c_line);
    }
    if (termios) {
        unipi_uart_trace("ttyNS%d Termios new:0x%04x %04x %04x %04x ldisc:%d", port->line,\
         termios->c_cflag, termios->c_iflag, \
         termios->c_oflag, termios->c_lflag, termios->c_line);
        spin_lock_irq(&port->lock);
        n_port->accept_rx = 0;
        spin_unlock_irq(&port->lock);
        unipi_uart_set_cflag(n_port, termios->c_cflag);
        spin_lock_irq(&port->lock);
        n_port->accept_rx = 1;
        spin_unlock_irq(&port->lock);
    }

	if (termios && (!old || ((old->c_iflag) != (termios->c_iflag)))) {
		unipi_uart_set_iflags(n_port, termios->c_iflag);
	}
	if (termios && !old) {
        // set line discipline only in case of new setting - Mervis behavior
        unipi_uart_set_ldisc(port, termios);
	}
	n_port->baud = uart_get_baud_rate(port, termios, old, 134, 115200);
	uart_update_timeout(port, termios->c_cflag, n_port->baud);
    unipi_uart_update_timeout(n_port, termios->c_cflag, n_port->baud);
}

int unipi_uart_config_rs485(struct uart_port *port, struct serial_rs485 *rs485)
{
	port->rs485 = *rs485;
	return 0;
}


static void unipi_uart_update_timeout(struct unipi_uart_port *n_port, unsigned int cflag, unsigned int baud)
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

const char* unipi_uart_type(struct uart_port *port)
{
	return port->type == PORT_UNIPI ? "UNIPISPI_NAME" : NULL;
}

int unipi_uart_request_port(struct uart_port *port)
{
	unipi_uart_trace("ttyNS%d Request port\n", port->line);
	return 0;
}

/*
// set port->tx_fifo_len by reading modbus register
// return 0 if success, 1 if couldnt read register - fifo len has old value
int static neuronspi_uart_read_tx_fifo_len(struct unipi_uart_port *port) 
{
    u16 read_length16 = - 1;
	struct spi_device *spi;
	int ret = 1;
    
    if (port->tx_fifo_reg) {
        spi = neuronspi_s_dev[port->dev_index];
        if (unipispi_modbus_read_register(spi, port->tx_fifo_reg, &read_length16) == 0) {
            ret = 0;
            port->tx_fifo_len = read_length16;
        }
	} else {
        // unknown port!
        port->tx_fifo_len = 0;
        ret = 0;
    }
	unipi_uart_trace("ttyNS%d Get tx fifo len:%d err:%d\n", port->port.line, read_length16, ret);
	return ret;
}
*/

#define start_tx_timer(port, lowlimit, delta) hrtimer_start_range_ns(&port->tx_timer, \
                             lowlimit * port->one_char_nsec, delta*port->one_char_nsec, \
                             HRTIMER_MODE_REL)

#define MAX_TXLEN	(UNIPI_UART_FIFO_SIZE >> 1)

static void unipi_uart_tx_callback(void *arg, int status, u8* data)
{
    struct unipi_uart_port *n_port = (struct unipi_uart_port*) arg;
    unsigned long flags;

	if (status > 0) {
		n_port->tx_fifo_len += status;
	}
   	spin_lock_irqsave(&n_port->port.lock, flags);
    unipi_uart_handle_tx(n_port, CB_WRITESTRING);
   	spin_unlock_irqrestore(&n_port->port.lock, flags);
}


void unipi_uart_handle_tx(struct unipi_uart_port *port, int calling) /* new async ver */
{
	struct device* parent = port->port.dev->parent;
	struct unipi_mfd_device *mfd = dev_get_drvdata(parent);
	int to_send, to_send_packet, need;
	//unsigned long flags;
	struct circ_buf *xmit;
	int new_tail, ret;

    //port->port.lock taken, This call must not sleep

	if (unlikely(port->port.x_char)) {
        // zatim nevim, co s tim
		port->port.icount.tx++;
		port->port.x_char = 0;
	}

 	xmit = &port->port.state->xmit;
	//spin_lock_irqsave(&port->port.lock, flags);
	// Get length of data pending in circular buffer
	to_send = uart_circ_chars_pending(xmit);
    unipi_uart_trace("ttyNS%d Handle TX. to_send=%d calling=%d\n", port->port.line, to_send, calling);
	if ((to_send == 0) || uart_tx_stopped(&port->port)) {
		port->pending_txop = 0;
		//spin_unlock_irqrestore(&port->port.lock, flags);
        // check tx_fifo status
        if (port->tx_fifo_len) {
            unipi_uart_trace_1("ttyNS%d Handle TX. Start timer=%llu", port->port.line, port->tx_fifo_len * port->one_char_nsec);
            start_tx_timer(port, port->tx_fifo_len, 2);
        }
		return;
	}
   
    // Limit to size of (TX FIFO / 2)
    to_send_packet = (to_send > MAX_TXLEN) ? MAX_TXLEN : to_send;
    need = to_send_packet - (UNIPI_UART_FIFO_SIZE - port->tx_fifo_len);
    if (need > 0) {
		//spin_unlock_irqrestore(&port->port.lock, flags);
        if (calling!=CB_GETTXFIFO) {
            if (unipi_uart_get_tx_fifo(port) == 0) return;
		}
        // reschedule work with pause
		port->pending_txop = 0;
        start_tx_timer(port, need, UNIPI_UART_FIFO_SIZE/4);
        return;
    }
            
    // Read data from tty buffer and send it to spi
    //spin_lock_irqsave(&port->port.lock, flags);
    port->port.icount.tx += to_send_packet;
    new_tail = (xmit->tail + to_send_packet) & (UART_XMIT_SIZE - 1);
    if (new_tail <= xmit->tail) {
        memcpy(port->tx_send_msg, xmit->buf+xmit->tail, UART_XMIT_SIZE - xmit->tail);
        memcpy(port->tx_send_msg+UART_XMIT_SIZE - xmit->tail, xmit->buf, new_tail);
    } else {
        memcpy(port->tx_send_msg, xmit->buf+xmit->tail, to_send_packet);
    }
    xmit->tail = new_tail;
    //spin_unlock_irqrestore(&port->port.lock, flags);

    unipi_uart_trace("ttyNS%d Handle TX Send: %d %16ph\n", port->port.line, to_send_packet, port->tx_send_msg);
	ret = mfd->op.write_str_async(mfd->op.self, port->dev_port, port->tx_send_msg, to_send_packet, 
	                              port, unipi_uart_tx_callback);
	if (ret != 0) {
		//ERROR, try later
		port->pending_txop = 0;
        start_tx_timer(port, 10, UNIPI_UART_FIFO_SIZE/4);
	} else if ((to_send-to_send_packet) < WAKEUP_CHARS) {
        uart_write_wakeup(&port->port);
    }
}

void unipi_uart_start_tx(struct uart_port *port)
{
	struct unipi_uart_port *n_port = to_unipi_uart_port(port,port);
	unsigned long flags;
	unipi_uart_trace("Start TX. is_pending=%d\n", n_port->pending_txop);

	if (!n_port->pending_txop) {
		spin_lock_irqsave(&n_port->txop_lock, flags);
		if (!n_port->pending_txop) {
			n_port->pending_txop = 1;
			spin_unlock_irqrestore(&n_port->txop_lock, flags);
			unipi_uart_handle_tx(n_port, START_TX);
			return;
		}
		spin_unlock_irqrestore(&n_port->txop_lock, flags);
	}
}


void unipi_uart_get_tx_fifo_callback(void* cb_data, int result, u8* value)
{
    unsigned long flags;
	struct unipi_uart_port *n_port = (struct unipi_uart_port *)cb_data;
	if (result == 1)
		n_port->tx_fifo_len = *(u16*)value;

   	spin_lock_irqsave(&n_port->port.lock, flags);
    unipi_uart_handle_tx(n_port, CB_GETTXFIFO);
    spin_unlock_irqrestore(&n_port->port.lock, flags);
}

int unipi_uart_get_tx_fifo(struct unipi_uart_port* n_port)
{
	struct device* parent = n_port->port.dev->parent;
	struct unipi_mfd_device *mfd = dev_get_drvdata(parent);
	int ret;

	ret = mfd->op.reg_read_async(mfd->op.self, n_port->tx_fifo_reg,
	                             n_port, unipi_uart_get_tx_fifo_callback);
	return !!ret;
}


// callback of tx_timer. Schedule port->tx_work
static enum hrtimer_restart unipi_uart_timer_func(struct hrtimer *timer)
{
    struct unipi_uart_port* n_port = ((container_of((timer), struct unipi_uart_port, tx_timer)));
	unsigned long flags;
	
	if (!n_port->pending_txop) {
		spin_lock_irqsave(&n_port->txop_lock, flags);
		if (!n_port->pending_txop) {
			n_port->pending_txop = 1;
			spin_unlock_irqrestore(&n_port->txop_lock, flags);
			if (unipi_uart_get_tx_fifo(n_port) != 0) {
				n_port->pending_txop = 0; // ERROR
			}
			return HRTIMER_NORESTART;
		}
		spin_unlock_irqrestore(&n_port->txop_lock, flags);
	}
	return HRTIMER_NORESTART;
}

/*********************************************************************
 * 
 *         RX channel management
 */

static void unipi_uart_handle_rx(struct unipi_uart_port *port, int rxlen, u8* pbuf)
{
	unsigned long flags;
	u32 ch, flag, i;
    
    unipi_uart_trace("ttyNS%d Insert Chars (%d): %16ph\n", port->port.line, rxlen, pbuf);

	if (rxlen) {
		spin_lock_irqsave(&port->port.lock, flags);
		if (port->accept_rx) {
			port->port.icount.rx++;
			flag = TTY_NORMAL;
			for (i = 0; i < rxlen; ++i) {
				ch = *pbuf;
            			pbuf++;
				if (uart_handle_sysrq_char(&port->port, ch))
					continue;

				uart_insert_char(&port->port, 0, 0, ch, flag);
			}
		}
		spin_unlock_irqrestore(&port->port.lock, flags);
	}
	tty_flip_buffer_push(&port->port.state->port);
}

/* called from read_str_async operation 
*/
void unipi_uart_rx_callback(void* cb_data, int result, u8* recv)
{
	struct unipi_uart_port *n_port = (struct unipi_uart_port *)cb_data;
	struct device* parent = n_port->port.dev->parent;
	struct unipi_mfd_device *mfd = dev_get_drvdata(parent);
	unsigned long flags;
	int len = result & 0xff;
	int remain = (result >> 8) & 0xff;

	if (result < 0) {
		/* try again read str */
		if (unipi_mfd_read_str_async(mfd, n_port->dev_port, 
		          n_port->rx_send_msg, n_port->rx_remain,
				  n_port, unipi_uart_rx_callback) == 0)
			return;
		goto unlock;
	}

	/* send data to tty */
	unipi_uart_handle_rx(n_port, len, recv);
	n_port->rx_remain = remain;
	if (n_port->rx_remain) {
		/* continue in rx "process" */
		if (unipi_mfd_read_str_async(mfd, n_port->dev_port, 
		          n_port->rx_send_msg, n_port->rx_remain, 
				  n_port, unipi_uart_rx_callback) == 0)
			return;
	}
unlock:
	/* finish receiving data - buffers in mfd are empty */
	spin_lock_irqsave(&n_port->rx_in_progress_lock, flags);
	n_port->rx_in_progress = 0;
	spin_unlock_irqrestore(&n_port->rx_in_progress_lock, flags);
	spin_lock_irqsave(&n_port->port.lock, flags);
	n_port->accept_rx = 1;
	spin_unlock_irqrestore(&n_port->port.lock, flags);
}

/* Start new (or continue to run) rx process to flush mfd buffer
 * If success return 0 and rx_in_progress flag is set.
 * If process cannot be started, return 1 and  flag is  not set
 */ 
static int unipi_uart_start_rx_process(struct unipi_uart_port *n_port)
{
	struct device* parent = n_port->port.dev->parent;
	struct unipi_mfd_device *mfd = dev_get_drvdata(parent);
	unsigned long flags;
	int locked;
	
	spin_lock_irqsave(&n_port->rx_in_progress_lock, flags);
	locked = n_port->rx_in_progress;
	n_port->rx_in_progress = 1;
	spin_unlock_irqrestore(&n_port->rx_in_progress_lock, flags);
	if (locked) return 0;

	/* Start new rx "process" */
	if (unipi_mfd_read_str_async(mfd, n_port->dev_port, 
				n_port->rx_send_msg, n_port->rx_remain, 
				n_port, unipi_uart_rx_callback) != 0) {

		spin_lock_irqsave(&n_port->rx_in_progress_lock, flags);
		n_port->rx_in_progress = 0;
		spin_unlock_irqrestore(&n_port->rx_in_progress_lock, flags);
		return 1;
	}
	return 0;
}

void unipi_uart_flush_buffer(struct uart_port* port)
{ 
	struct unipi_uart_port *n_port = to_unipi_uart_port(port, port);

	unipi_uart_trace("ttyNS%d Flush buffer\n", port->line);

    /* port->lock taken, This call must not sleep
		disable accepting received data
		will be enabled in rx callback when remain=0
	*/
	n_port->accept_rx = 0;
	/* Try to flush everything from mfd uart if not already started reading */
	if (unipi_uart_start_rx_process(n_port) != 0) 
		n_port->accept_rx = 1;
}

/* called from unipi_spi when priority data are received */
void unipi_uart_rx_char_callback(void *self, u8 port, u8 ch, int remain)
{
	struct unipi_uart_device *n_uart = (struct unipi_uart_device *)self;
	struct unipi_uart_port *n_port;
	unsigned long flags;

	if (port >= n_uart->port_count) return;
	n_port = &n_uart->p[port];
	unipi_uart_handle_rx(n_port, 1, &ch);
	if (remain) {
		if (unipi_uart_start_rx_process(n_port) == 0) 
			return;
	}
	/* finish receiving data - buffers in mfd are empty */
	spin_lock_irqsave(&n_port->port.lock, flags);
	n_port->accept_rx = 1;
	spin_unlock_irqrestore(&n_port->port.lock, flags);
}


// Initialise the driver - called once on open
int unipi_uart_startup(struct uart_port *port)
{
	struct unipi_uart_port *n_port = to_unipi_uart_port(port, port);
	struct device* parent = n_port->port.dev->parent;
	struct unipi_mfd_device *mfd = dev_get_drvdata(parent);

	n_port->accept_rx = 0;
	unipi_mfd_enable_interrupt(mfd, UNIPI_MFD_INT_RX_NOT_EMPTY | UNIPI_MFD_INT_RX_MODBUS);
	unipi_uart_power(port, 1);
	// TODO: /* Reset FIFOs*/
	unipi_uart_trace("ttyNS%d Startup\n", port->line);
	return 0;
}


void unipi_uart_shutdown(struct uart_port *port)
{
    unipi_uart_trace("ttyNS%d Shutdown\n", port->line);
    unipi_uart_power(port, 0);
}


static const struct uart_ops unipi_uart_ops =
{
	.tx_empty			= neuronspi_uart_tx_empty,
	.set_mctrl			= unipi_uart_set_mctrl,
	.get_mctrl			= unipi_uart_get_mctrl,
	.stop_tx			= unipi_uart_null_void,
	.start_tx			= unipi_uart_start_tx,
	.stop_rx			= unipi_uart_stop_rx,
	.flush_buffer		= unipi_uart_flush_buffer,
	.break_ctl			= unipi_uart_break_ctl,
	.startup			= unipi_uart_startup,
	.shutdown			= unipi_uart_shutdown,
	.set_termios		= unipi_uart_set_termios,
	.set_ldisc			= unipi_uart_set_ldisc,
	.type				= unipi_uart_type,
	.request_port		= unipi_uart_request_port,
	.release_port		= unipi_uart_null_void,
	.config_port		= unipi_uart_config_port,
	.verify_port		= unipi_uart_verify_port,
	.pm					= unipi_uart_pm,
	.ioctl				= unipi_uart_ioctl,
};


int unipi_uart_port_probe(struct device *dev, struct unipi_uart_device *n_uart, int i, int irq)
{
    
	struct unipi_uart_port* port;
    int line, ret = 0;
	u32 fw_version, fw_variant;
    
	// port is pointer to item ->p[x]
	port = n_uart->p + i;
	port->dev_port = i;
	
	line = find_first_zero_bit(unipi_uart_lines, UNIPI_UART_MAX_NR);
	if (line == UNIPI_UART_MAX_NR) {
		return -ERANGE;
	}
	port->port.line	= line;
         
	port->port.dev	= dev;
	port->port.type	= PORT_UNIPI;
	port->port.fifosize	= UNIPI_UART_FIFO_SIZE*8;
	port->port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
	port->port.iotype	= UPIO_PORT;
	port->port.uartclk	= 9600;
	port->port.rs485_config = unipi_uart_config_rs485;
	port->port.ops	= &unipi_uart_ops;
	port->port.irq	= irq;

	spin_lock_init(&port->port.lock);

	spin_lock_init(&port->rx_in_progress_lock);
	spin_lock_init(&port->txop_lock);
	port->tx_fifo_len = 0x7fff; //set it to big number; invoke reading current value from MFD

	regmap_read(n_uart->regmap, UNIPI_MFD_REG_FW_VERSION, &fw_version);
	regmap_read(n_uart->regmap, UNIPI_MFD_REG_FW_VARIANT, &fw_variant);
	if (fw_version >= 0x0519) {
		port->tx_fifo_reg = port_to_uartregs(i, UNIPI_MFD_REG_UART0_TXQLEN);
	} else if ((fw_variant & 0xff00) == 0) {
		port->tx_fifo_reg = 7; //Brain=7 n_spi->regstart_table->uart_queue_reg;      // define modbus register
	}
	hrtimer_init(&port->tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	port->tx_timer.function = unipi_uart_timer_func;

	ret = uart_add_one_port(&unipi_uart, &port->port);
	if (ret) {
		port->port.dev = NULL;
		return ret;
	}

	unipi_uart_trace("Probe cflag:%08x\n", unipi_spi_uart_get_cflag(n_uart->regmap, i));

	set_bit(line, unipi_uart_lines);
    return ret;
}



int unipi_uart_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *parent = dev->parent;
	struct device_node *np = dev->of_node;
	struct unipi_mfd_device * mfd;
	struct regmap* map;
	struct unipi_uart_device *n_uart;
	int port_count = 1;
	int ret, i;

	//if (!parent) {
	//	dev_err(dev, "No parent for Unipi AIO\n");
	//	return -ENODEV;
	//}
	//map  = dev_get_regmap(parent, "registers");
	map = unipi_mfd_get_regmap(dev->parent, "registers");
	if (IS_ERR(map) || map == NULL) {
		dev_err(dev, "No regmap for Unipi device\n");
		return PTR_ERR(map);
	}
	mfd = dev_get_drvdata(parent);
	
	of_property_read_u32(np, "port-count", &port_count);
	if (!((port_count >= 1)&&(port_count<=4))) {
		dev_err(dev, "Property port-count (%d) must be 1..4\n", port_count);
		return EINVAL;
	}
	/* Alloc port structure - (struct unipi_uart_device) +  (port_count-1)*(struct unipi_uart_port) */
	n_uart = devm_kzalloc(dev, struct_size(n_uart, p, port_count), GFP_KERNEL);
	if (!n_uart) {
		dev_err(dev, "Error allocating port structure\n");
		return -ENOMEM;
	}
	n_uart->regmap = map;
	n_uart->port_count = port_count;
	platform_set_drvdata(pdev, n_uart);
	
	for (i = 0; i < port_count; i++) {
		ret = unipi_uart_port_probe(dev, n_uart, i, mfd->irq);
		if (ret)
			goto out;
		dev_info(dev, "Serial port ttyNS%d on %s port:%d created\n", n_uart->p[i].port.line, dev_name(parent), i);
	}
	mfd->rx_self = n_uart;
	mfd->rx_char_callback = unipi_uart_rx_char_callback;
	return 0;

out:
	for (i = 0; i < port_count; i++) {
		if (n_uart->p[i].port.dev) {
			uart_remove_one_port(&unipi_uart, &n_uart->p[i].port);
			clear_bit(n_uart->p[i].port.line, unipi_uart_lines);
		}
	}
	return ret;
}

int unipi_uart_remove(struct platform_device *pdev)
{
	struct unipi_uart_device *n_uart = platform_get_drvdata(pdev);
	struct device* parent = pdev->dev.parent;
	struct unipi_mfd_device *mfd = dev_get_drvdata(parent);
	struct unipi_uart_port *port; 
	int i;

	mfd->rx_self = NULL;
	mfd->rx_char_callback = NULL;

	for (i = 0; i < n_uart->port_count; i++) {
        port = n_uart->p + i;
        hrtimer_cancel(&port->tx_timer);
		uart_remove_one_port(&unipi_uart, &port->port);
        //kthread_flush_work(&(port->tx_work));
		unipi_uart_power(&port->port, 0);
        //printk(KERN_INFO "UNIPIUART: Serial port ttyNS%d removed\n", i + n_spi->uart_pindex);
	}
	return 0;
}

/*********************
 * Final definitions *
 *********************/

static const struct of_device_id unipi_uart_id_match[] = {
		{.compatible = "unipi,uart"},
		{}
};
MODULE_DEVICE_TABLE(of, unipi_uart_id_match);

struct platform_driver unipi_uart_driver =
{
	.driver = {
		.name			= "unipi-uart",
		.of_match_table	= of_match_ptr(unipi_uart_id_match)
	},
	.probe				= unipi_uart_probe,
	.remove				= unipi_uart_remove,
};

static int __init unipi_uart_init(void)
{
    int ret;

	bitmap_zero(unipi_uart_lines, UNIPI_UART_MAX_NR);
	ret = uart_register_driver(&unipi_uart);
	if (ret) 
        return ret;
		
	ret = platform_driver_register(&unipi_uart_driver);
	if (ret)
		uart_unregister_driver(&unipi_uart);
	return ret;
}

static void __exit unipi_uart_exit(void)
{
	platform_driver_unregister(&unipi_uart_driver);
	uart_unregister_driver(&unipi_uart);
}


module_init(unipi_uart_init);
module_exit(unipi_uart_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomas Knot <knot@faster.cz>");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi Uart Driver");
