/*
 * UniPi PLC device driver - Copyright (C) 2022 UniPi Technology
 * Author: Miroslav Ondra <ondra@faster.cz>
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

#include "unipi_channel.h"
#include "unipi_iogroup_bus.h"
#include "unipi_mfd.h"

#define UNIPI_DALI_DETAILED_DEBUG 0
#if UNIPI_DALI_DETAILED_DEBUG > 2
# define unipi_dali_trace_2(f, args...)	printk(KERN_INFO "UNIPIDALI: " f, ##args)
#else
# define unipi_dali_trace_2(f, args...)
#endif

#if UNIPI_DALI_DETAILED_DEBUG > 1
# define unipi_dali_trace_1(f, args...)	printk(KERN_INFO "UNIPIDALI: " f, ##args)
#else
# define unipi_dali_trace_1(f, args...)
#endif

#if UNIPI_DALI_DETAILED_DEBUG > 0
# define unipi_dali_trace(f, args...)	printk(KERN_INFO "UNIPIDALI: " f, ##args)
#else
# define unipi_dali_trace(f, args...)
#endif

/********************
 * Data Definitions *
 ********************/

#define PORT_DALI			185
#define UNIPI_DALI_MAX_NR	16
#define UNIPI_DALI_FIFO_SIZE	256

#define CB_WRITESTRING 1
#define CB_GETTXFIFO  2
#define START_TX       3

#define DALI_ST_SENDING1 1

struct unipi_dali_port
{
	struct uart_port	port;
	int			dev_port;    // index of port on mfd 0..3
	u16                     status_reg;      // register in mfd modbus map to read status of channel
//	u16                     rx_counter_reg;  // register in mfd modbus map to read rx frame counter
	u16                     tx_reg;          // register in mfd modbus map to send frame
	int                     accept_rx;
//	struct kthread_work	flush_work;

//	struct kthread_work	tx_work;
	int                     st_sending;   // device is sending data
	int			has_tx_data;
	int			is_polling_mode;
	struct hrtimer		timer;

	int                     rx_counter;      // last value of rx frame counter
	u16                     status_buf[4+4];   // status , rx_counter, rxval0 rxval1 + reserved space for v1 op

//	s32			baud;
//	s64                     one_char_nsec;

	spinlock_t              poll_in_progress_lock;
	int                     poll_in_progress;
//	u8                      rx_recv_msg[UNIPI_DALI_FIFO_SIZE+4+8];

	spinlock_t              txop_lock;
	int                     pending_txop;
	u8                      tx_send_msg[UNIPI_DALI_FIFO_SIZE+4+8];
};

struct unipi_dali_device
{
	struct regmap		*regmap;
	int			port_count;
	struct unipi_dali_port	p[0];
};


static struct uart_driver unipi_dali_uart_driver = {
	.owner          = THIS_MODULE,
	.driver_name    = "unipi_dali_tty",
	.dev_name       = "dali",
	.major          = PORT_DALI,
	.minor          = 0,//MINOR_START,
	.nr             = UNIPI_DALI_MAX_NR,
};
static DECLARE_BITMAP(unipi_dali_lines, UNIPI_DALI_MAX_NR);

/********************
 * Static Functions *
 ********************/
#define to_unipi_dali_port(p,e)	((container_of((p), struct unipi_dali_port, e)))

//static void unipi_dali_update_timeout(struct unipi_dali_port *n_port, unsigned int cflag, unsigned int baud);
int unipi_dali_get_tx_fifo(struct unipi_dali_port* n_port);
void unipi_dali_handle_tx(struct unipi_dali_port *port, int calling);
static int unipi_dali_start_poll_process(struct unipi_dali_port *n_port);


#define NEURONSPI_UART_FIFO_REGISTER    505

static inline int port_to_uartregs(u8 port, u16 reg)
{
    return reg + ((port==0) ? 0 : (10*(port+1)));
}


/*******************
 * Empty functions *
 *******************/
void unipi_dali_set_mctrl(struct uart_port *port, u32 mctrl)
{
    /* Do nothing */
}

u32 unipi_dali_get_mctrl(struct uart_port *port)
{
	unipi_dali_trace_1("dali%d Get mctrl\n", port->line);
	return TIOCM_DSR | TIOCM_CAR;
}

void unipi_dali_null_void(struct uart_port *port)
{
	/* Do nothing */
}

void unipi_dali_stop_rx(struct uart_port *port)
{
	struct unipi_dali_port *n_port = to_unipi_dali_port(port, port);
	/*unsigned long flags;*/
	/*spin_lock_irqsave(&port->lock, flags);*/
	n_port->accept_rx = 0;
	/*spin_unlock_irqrestore(&port->lock, flags);*/
}

const char* unipi_dali_type(struct uart_port *port)
{
	return port->type == PORT_DALI ? "UNIPI_DALI" : NULL;
}

void unipi_dali_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_DALI;
	}
}

/************************
 * Non-static Functions *
 ************************/

void unipi_dali_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
//	struct unipi_uart_port *n_port = to_unipi_dali_port(port, port);
/*
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
*/
}


u32 unipi_dali_tx_empty(struct uart_port *port)
{
	struct unipi_dali_port *n_port = to_unipi_dali_port(port, port);
	//int len = n_port->tx_fifo_len;
	//unsigned long flags;

	if (n_port->st_sending && !n_port->pending_txop && !n_port->poll_in_progress) {
		unipi_dali_start_poll_process(n_port);
/*		spin_lock_irqsave(&n_port->txop_lock, flags);
		if (!n_port->pending_txop) {
			n_port->pending_txop = 1;
			spin_unlock_irqrestore(&n_port->txop_lock, flags);
			if (unipi_dali_get_tx_fifo(n_port) != 0) {
				n_port->pending_txop = 0; // ERROR
			}
		} else {
			spin_unlock_irqrestore(&n_port->txop_lock, flags);
		}
*/
	}
	unipi_dali_trace("dali%d Tx empty? %s\n", port->line, (n_port->st_sending==0)?"Yes":"No");
	return (n_port->st_sending==0) ? TIOCSER_TEMT : 0;
}


int unipi_dali_ioctl(struct uart_port *port, unsigned int ioctl_code, unsigned long ioctl_arg)
{
/*	u32 value;
	struct unipi_dali_port *n_port = to_unipi_dali_port(port, port);
	struct unipi_dali_device *n_uart = dev_get_drvdata(port->dev);
*/
	switch (ioctl_code) {
/*	case TIOCSETD: {
		unipi_dali_trace("dali%d Ioctl TIOCSETD (processed via set_termios)\n", port->line);
		return 1;
	}
	case 0x5481: {
		value = ((ioctl_arg * 1000000) / n_port->baud);
		if (value > 0xffff) value = 0xffff;
		unipi_dali_trace("dali%d Ioctl 0x5481 set timeout=%d\n", port->line, value);
		regmap_write(n_uart->regmap, port_to_uartregs(n_port->dev_port, UNIPI_MFD_REG_UART0_TIMEOUT), value);
		return 0;
	}
	case 0x5480: {
		value = (ioctl_arg * 10);
		if (value > 0xffff) value = 0xffff;
		unipi_dali_trace("dali%d Ioctl 0x5480 set timeout=%d\n", port->line, value);
		regmap_write(n_uart->regmap, port_to_uartregs(n_port->dev_port, UNIPI_MFD_REG_UART0_TIMEOUT), value);
		return 0;
	}
*/
	default: {
		return -ENOIOCTLCMD;
	}
	}
}



#define start_tx_timer(port) hrtimer_start_range_ns(&port->tx_timer, \
                             5000000, 1000000, \
                             HRTIMER_MODE_REL)


static void unipi_dali_tx_callback(void *arg, int status)
{
	struct unipi_dali_port *n_port = (struct unipi_dali_port*) arg;
	unsigned long flags;

	if (status > 0) {
		n_port->st_sending = 1;
	}
	spin_lock_irqsave(&n_port->port.lock, flags);
	unipi_dali_handle_tx(n_port, CB_WRITESTRING);
	spin_unlock_irqrestore(&n_port->port.lock, flags);
}


void unipi_dali_handle_tx(struct unipi_dali_port *port, int calling) /* new async ver */
{
	struct device* parent = port->port.dev->parent;
	struct unipi_channel *channel = to_unipi_iogroup_device(parent)->channel;
	//struct unipi_mfd_device *mfd = dev_get_drvdata(parent);
	int to_send, to_send_packet;//, need;
	//unsigned long flags;
	struct circ_buf *xmit;
	int new_tail, ret;

	//port->port.lock taken, This call must not sleep

	port->has_tx_data = 0;
	xmit = &port->port.state->xmit;
	//spin_lock_irqsave(&port->port.lock, flags);
	// Get length of data pending in circular buffer
	to_send = uart_circ_chars_pending(xmit);
	unipi_dali_trace("dali%d Handle TX. to_send=%d calling=%d\n", port->port.line, to_send, calling);
	if ((to_send == 0) || uart_tx_stopped(&port->port)) {
		port->pending_txop = 0;
		//spin_unlock_irqrestore(&port->port.lock, flags);
		// check tx_fifo status
		//if (port->tx_fifo_len) {
		//	unipi_dali_trace_1("dali%d Handle TX. Start timer=%llu", port->port.line, port->tx_fifo_len * port->one_char_nsec);
		//	start_tx_timer(port);
		//}
		return;
	}
	// Limit to size of 4 ) 
	to_send_packet = (to_send >= 4) ? 4 : to_send;
	if (port->st_sending) {
		// reschedule work with pause
		port->pending_txop = 0;
		port->has_tx_data = 1;
		//start_tx_timer(port);
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

	if (to_send_packet == 4) {
		unipi_dali_trace("dali%d Handle TX Send (%d): %4ph\n", port->port.line, to_send_packet, port->tx_send_msg);
		ret = unipi_write_regs_async(channel, port->tx_reg, 2, port->tx_send_msg, port, unipi_dali_tx_callback);
	} else {
		unipi_dali_trace("dali%d TX discard: Not enough data (%d): %4ph\n", port->port.line, to_send_packet, port->tx_send_msg);
		port->pending_txop = 0;
		ret = 0;
	}
	if (ret != 0) {
		//ERROR, try later
		port->pending_txop = 0;
//		start_tx_timer(port);
	} else if ((to_send-to_send_packet) < WAKEUP_CHARS) {
		uart_write_wakeup(&port->port);
	}
}

void unipi_dali_start_tx(struct uart_port *port)
{
	struct unipi_dali_port *n_port = to_unipi_dali_port(port,port);
	unsigned long flags;
	unipi_dali_trace("Start TX. is_pending=%d\n", n_port->pending_txop);

	if (!n_port->pending_txop) {
		spin_lock_irqsave(&n_port->txop_lock, flags);
		if (!n_port->pending_txop) {
			n_port->pending_txop = 1;
			spin_unlock_irqrestore(&n_port->txop_lock, flags);
			unipi_dali_handle_tx(n_port, START_TX);
			return;
		}
		spin_unlock_irqrestore(&n_port->txop_lock, flags);
	}
}

/*
void unipi_dali_get_tx_fifo_callback(void* cb_data, int result)
{
	unsigned long flags;
	struct unipi_dali_port *n_port = (struct unipi_dali_port *)cb_data;
	spin_lock_irqsave(&n_port->port.lock, flags);
	unipi_dali_handle_tx(n_port, CB_GETTXFIFO);
	spin_unlock_irqrestore(&n_port->port.lock, flags);
}

int unipi_dali_get_tx_fifo(struct unipi_dali_port* n_port)
{
	struct device* parent = n_port->port.dev->parent;
	struct unipi_channel *channel = to_unipi_iogroup_device(parent)->channel;
	//struct unipi_mfd_device *mfd = dev_get_drvdata(parent);
	int ret;

	ret = unipi_read_regs_async(channel, n_port->tx_fifo_reg, 1, (uint8_t*)&n_port->tx_fifo_len,
                             n_port, unipi_dali_get_tx_fifo_callback);
	return !!ret;
}
*/

// callback of tx_timer. Schedule port->tx_work
/*
static enum hrtimer_restart unipi_dali_timer_func(struct hrtimer *timer)
{
	struct unipi_dali_port* n_port = ((container_of((timer), struct unipi_dali_port, tx_timer)));
	unsigned long flags;
	
	if (!n_port->pending_txop) {
		spin_lock_irqsave(&n_port->txop_lock, flags);
		if (!n_port->pending_txop) {
			n_port->pending_txop = 1;
			spin_unlock_irqrestore(&n_port->txop_lock, flags);
			if (unipi_dali_get_tx_fifo(n_port) != 0) {
				n_port->pending_txop = 0; // ERROR
			}
			return HRTIMER_NORESTART;
		}
		spin_unlock_irqrestore(&n_port->txop_lock, flags);
	}
	return HRTIMER_NORESTART;
}
*/

/*********************************************************************
 * 
 *         RX channel management
 */

static void unipi_dali_handle_rx(struct unipi_dali_port *port, int rxlen, u8* pbuf)
{
	unsigned long flags;
	int i;

	if (rxlen) {
		spin_lock_irqsave(&port->port.lock, flags);
		if (port->accept_rx) {
			unipi_dali_trace("dali%d Insert Chars (%d): %4ph\n", port->port.line, rxlen, pbuf);
			port->port.icount.rx++;
			for (i = 0; i < rxlen; ++i) {
				uart_insert_char(&port->port, 0, 0, pbuf[i], TTY_NORMAL);
			}
		}
		spin_unlock_irqrestore(&port->port.lock, flags);
	}
	tty_flip_buffer_push(&port->port.state->port);
}

/* called from read_str_async operation 
*/
void unipi_dali_status_callback(void* cb_data, int result)
{
	struct unipi_dali_port *n_port = (struct unipi_dali_port *)cb_data;
	struct device* parent = n_port->port.dev->parent;
	struct unipi_channel *channel = to_unipi_iogroup_device(parent)->channel;
	unsigned long flags;

	//unipi_dali_trace("dali%d status cb result=%x\n", n_port->port.line, result);

	if (result < 0) {
		/* try again read registers */
		if (unipi_read_regs_async(channel, n_port->status_reg, 4, (uint8_t*)&n_port->status_buf,
		                         n_port, unipi_dali_status_callback) == 0)
			return;
		goto unlock;
	}

	if (n_port->status_buf[1] != n_port->rx_counter) {
		/* copy received data to tty */
		unipi_dali_handle_rx(n_port, 4, (char*)&n_port->status_buf[2]);
		n_port->rx_counter = n_port->status_buf[1];
	}
	if (!n_port->pending_txop) {
		n_port->st_sending = n_port->status_buf[0] & DALI_ST_SENDING1;
		spin_lock_irqsave(&n_port->txop_lock, flags);
		if (!n_port->pending_txop && !n_port->st_sending && n_port->has_tx_data) {
			n_port->pending_txop = 1;
			spin_unlock_irqrestore(&n_port->txop_lock, flags);
			spin_lock_irqsave(&n_port->port.lock, flags);
			unipi_dali_handle_tx(n_port, CB_GETTXFIFO);
			spin_unlock_irqrestore(&n_port->port.lock, flags);
			goto unlock;
		}
		spin_unlock_irqrestore(&n_port->txop_lock, flags);
	}

unlock:
	/* finish receiving data - buffers in mfd are empty */
	spin_lock_irqsave(&n_port->poll_in_progress_lock, flags);
	n_port->poll_in_progress = 0;
	spin_unlock_irqrestore(&n_port->poll_in_progress_lock, flags);
	spin_lock_irqsave(&n_port->port.lock, flags);
	n_port->accept_rx = 1;
	spin_unlock_irqrestore(&n_port->port.lock, flags);

	if (n_port->is_polling_mode) // ToDo:
		//hrtimer_start_range_ns(&n_port->timer, 500000000, 1000000, HRTIMER_MODE_REL);
		hrtimer_start_range_ns(&n_port->timer, 5000000, 1000000, HRTIMER_MODE_REL);
}

/* Start new (or continue to run) poll process to flush mfd buffer
 * If success return 0 and poll_in_progress flag is set.
 * If process cannot be started, return 1 and  flag is  not set
 */ 
static int unipi_dali_start_poll_process(struct unipi_dali_port *n_port)
{
	struct device* parent = n_port->port.dev->parent;
	struct unipi_channel *channel = to_unipi_iogroup_device(parent)->channel;
	//struct unipi_mfd_device *mfd = dev_get_drvdata(parent);
	unsigned long flags;
	int locked;

	spin_lock_irqsave(&n_port->poll_in_progress_lock, flags);
	locked = n_port->poll_in_progress;
	n_port->poll_in_progress = 1;
	spin_unlock_irqrestore(&n_port->poll_in_progress_lock, flags);
	if (locked) return 0;

	/* Start new poll "process" */
	//unipi_dali_trace("dali%d Start poll proc\n", n_port->port.line);
	if (unipi_read_regs_async(channel, n_port->status_reg, 4, (uint8_t*)n_port->status_buf,
	                          n_port, unipi_dali_status_callback)) {
		spin_lock_irqsave(&n_port->poll_in_progress_lock, flags);
		n_port->poll_in_progress = 0;
		spin_unlock_irqrestore(&n_port->poll_in_progress_lock, flags);
		return 1;
	}
	return 0;
}

void unipi_dali_flush_buffer(struct uart_port* port)
{ 
	struct unipi_dali_port *n_port = to_unipi_dali_port(port, port);

	unipi_dali_trace("dali%d Flush buffer\n", port->line);

	/* port->lock taken, This call must not sleep
		disable accepting received data
		will be enabled in rx callback when remain=0
	*/
	n_port->accept_rx = 0;
	/* Try to flush everything from mfd uart if not already started reading */
	if (unipi_dali_start_poll_process(n_port) != 0)
		n_port->accept_rx = 1;
}


/* called from unipi_mfd_int_status_callback when interrupt status contains RX flags */
void unipi_dali_rx_not_empty_callback(void *self, int port)
{
	struct unipi_dali_device *n_uart = (struct unipi_dali_device *)self;
	struct unipi_dali_port *n_port;

	if (port >= n_uart->port_count) return;
	n_port = &n_uart->p[port];
	unipi_dali_start_poll_process(n_port);
}


// Initialise the driver - called once on open
int unipi_dali_startup(struct uart_port *port)
{
	struct unipi_dali_port *n_port = to_unipi_dali_port(port, port);
//	struct device* parent = n_port->port.dev->parent;
//	struct unipi_iogroup_device *iogroup = to_unipi_iogroup_device(parent);
	//struct unipi_dali_device *n_uart = dev_get_drvdata(port->dev);
	//struct unipi_mfd_device *mfd = dev_get_drvdata(parent);

	//int ret;
	unipi_dali_trace("dali%d startup\n", port->line);

	n_port->accept_rx = 0;
	n_port->is_polling_mode = 1;
	unipi_dali_start_poll_process(n_port);
	return 0;

//	ret = regmap_bulk_read(n_uart->regmap, n_port->status_reg, (void*)n_port->status_buf, 4);

//	ret = regmap_read(n_uart->regmap, n_port->status_reg, &st);
//	ret = regmap_read(n_uart->regmap, n_port->status_reg+1, &rxcnt);
//	n_port->status_buf[0] = st;
//	if (ret == 0)
//		n_port->rx_counter = n_port->status_buf[1];

//	unipi_mfd_enable_interrupt(iogroup, UNIPI_MFD_INT_DALI_RX);
}


void unipi_dali_shutdown(struct uart_port *port)
{
	struct unipi_dali_port *n_port = to_unipi_dali_port(port, port);
	n_port->is_polling_mode = 0;
	n_port->accept_rx = 0;
	unipi_dali_trace("dali%d shutdown\n", port->line);
}


/* callback of poll_timer - for devices without or disfunctional interrupt */
static enum hrtimer_restart unipi_dali_timer_func(struct hrtimer *timer)
{
	struct unipi_dali_port *n_port = ((container_of((timer), struct unipi_dali_port, timer)));
	int ret;
	ret = unipi_dali_start_poll_process(n_port);
	if (ret != 0) {
	}
	//unipi_dali_trace((channel->dev), "Pseudo IRQ\n");
	return HRTIMER_NORESTART;
}


static const struct uart_ops unipi_dali_ops =
{
	.tx_empty		= unipi_dali_tx_empty, /* ! */
	.set_mctrl		= unipi_dali_set_mctrl, /* ! */
	.get_mctrl		= unipi_dali_get_mctrl, /* ! */
	.stop_tx		= unipi_dali_null_void, /* ! */
	.start_tx		= unipi_dali_start_tx, /* ! */
	.stop_rx		= unipi_dali_stop_rx, /* ! */
	.flush_buffer		= unipi_dali_flush_buffer, /* not required */
//	.break_ctl		= unipi_dali_break_ctl, /* not required */
	.startup		= unipi_dali_startup, /* ! */
	.shutdown		= unipi_dali_shutdown, /* ! */
	.set_termios		= unipi_dali_set_termios, /* ! */
//	.set_ldisc		= unipi_dali_set_ldisc, /* not required */
	.type			= unipi_dali_type, /* not required */
//	.request_port		= unipi_dali_request_port, /* not required */
//	.release_port		= unipi_dali_null_void, /* not required */
	.config_port		= unipi_dali_config_port, /* ! */
//	.verify_port		= unipi_dali_verify_port, /* not required */
//	.pm			= unipi_dali_pm, /* not required */
	.ioctl			= unipi_dali_ioctl, /* not required */
};


int unipi_dali_port_probe(struct device *dev, struct unipi_dali_device *n_uart, int i, int irq)
{
    
	struct unipi_dali_port* port;
	int line, ret = 0;
    
	// port is pointer to item ->p[x]
	port = n_uart->p + i;
	port->dev_port = i;
	port->status_reg = 0; // ToDo - not constant
	port->tx_reg = 4; // ToDo - not constant

	line = find_first_zero_bit(unipi_dali_lines, UNIPI_DALI_MAX_NR);
	if (line == UNIPI_DALI_MAX_NR) {
		return -ERANGE;
	}
	port->port.line	= line;
         
	port->port.dev	= dev;
	port->port.type	= PORT_DALI;
	port->port.fifosize	= 4; //UNIPI_DALI_FIFO_SIZE*8;
	port->port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
	port->port.iotype	= UPIO_PORT;
	port->port.uartclk	= 9600;
//	port->port.rs485_config = unipi_dali_config_rs485;
	port->port.ops	= &unipi_dali_ops;
	port->port.irq	= irq;

	spin_lock_init(&port->port.lock);

	spin_lock_init(&port->poll_in_progress_lock);
	spin_lock_init(&port->txop_lock);
//	port->tx_fifo_len = 0x7fff; //set it to big number; invoke reading current value from MFD

	hrtimer_init(&port->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	port->timer.function = unipi_dali_timer_func;

	ret = uart_add_one_port(&unipi_dali_uart_driver, &port->port);
	if (ret) {
		port->port.dev = NULL;
		return ret;
	}

//	unipi_dali_trace("Probe cflag:%08x\n", unipi_spi_uart_get_cflag(n_uart->regmap, i));

	set_bit(line, unipi_dali_lines);
    return ret;
}



int unipi_dali_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *parent = dev->parent;
	struct device_node *np = dev->of_node;
	struct unipi_iogroup_device *iogroup = to_unipi_iogroup_device(parent);
//	struct unipi_channel *channel = iogroup->channel;
	struct regmap* map;
	struct unipi_dali_device *n_uart;
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

	of_property_read_u32(np, "port-count", &port_count);
	if (!((port_count >= 1)&&(port_count<=4))) {
		dev_err(dev, "Property port-count (%d) must be 1..4\n", port_count);
		return EINVAL;
	}
	/* Alloc port structure - (struct unipi_dali_device) +  (port_count-1)*(struct unipi_dali_port) */
	n_uart = devm_kzalloc(dev, struct_size(n_uart, p, port_count), GFP_KERNEL);
	if (!n_uart) {
		dev_err(dev, "Error allocating port structure\n");
		return -ENOMEM;
	}
	n_uart->regmap = map;
	n_uart->port_count = port_count;
	platform_set_drvdata(pdev, n_uart);
	
	for (i = 0; i < port_count; i++) {
		ret = unipi_dali_port_probe(dev, n_uart, i, iogroup->irq);
		if (ret)
			goto out;
		dev_info(dev, "Serial port dali%d on %s created\n", n_uart->p[i].port.line, dev_name(parent));
	}
	//channel->rx_self = n_uart;
	//channel->rx_char_callback = unipi_dali_rx_char_callback;
	//iogroup->uart_rx_callback = unipi_dali_rx_not_empty_callback;
	//iogroup->uart_rx_self = n_uart;
	return 0;

out:
	for (i = 0; i < port_count; i++) {
		if (n_uart->p[i].port.dev) {
			uart_remove_one_port(&unipi_dali_uart_driver, &n_uart->p[i].port);
			clear_bit(n_uart->p[i].port.line, unipi_dali_lines);
		}
	}
	return ret;
}

int unipi_dali_remove(struct platform_device *pdev)
{
	struct unipi_dali_device *n_uart = platform_get_drvdata(pdev);
	struct device* parent = pdev->dev.parent;
	struct unipi_iogroup_device *iogroup = to_unipi_iogroup_device(parent);
	struct unipi_channel *channel = iogroup->channel;
	//struct unipi_mfd_device *mfd = dev_get_drvdata(parent);
	struct unipi_dali_port *port; 
	int i;

	channel->rx_self = NULL;
	//channel->rx_char_callback = NULL;
	iogroup->uart_rx_callback = NULL;
	iogroup->uart_rx_self = NULL;

	for (i = 0; i < n_uart->port_count; i++) {
		port = n_uart->p + i;
		hrtimer_cancel(&port->timer);
		uart_remove_one_port(&unipi_dali_uart_driver, &port->port);
	}
	return 0;
}

/*********************
 * Final definitions *
 *********************/
static const struct of_device_id unipi_dali_id_match[] = {
		{.compatible = "unipi,dali"},
		{}
};
MODULE_DEVICE_TABLE(of, unipi_dali_id_match);

struct platform_driver unipi_dali_driver =
{
	.driver = {
		.name			= "unipi-dali",
		.of_match_table	= of_match_ptr(unipi_dali_id_match)
	},
	.probe				= unipi_dali_probe,
	.remove				= unipi_dali_remove,
};


static int __init unipi_dali_init(void)
{
	int ret;
	bitmap_zero(unipi_dali_lines, UNIPI_DALI_MAX_NR);
	ret = uart_register_driver(&unipi_dali_uart_driver);
	if (ret)
		return ret;
		
	ret = platform_driver_register(&unipi_dali_driver);
	return ret;
}

static void __exit unipi_dali_exit(void)
{
	platform_driver_unregister(&unipi_dali_driver);
	uart_unregister_driver(&unipi_dali_uart_driver);
}


module_init(unipi_dali_init);
module_exit(unipi_dali_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi Dali Driver");
