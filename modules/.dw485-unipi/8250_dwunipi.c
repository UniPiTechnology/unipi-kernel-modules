/*
 * Synopsys DesignWare 8250 driver for UniPi 485 ports
 *
 * Copyright 2018 Miroslav Ondra, Faster CZ
 * Copyright 2011 Picochip, Jamie Iles.
 * Copyright 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * The Synopsys DesignWare 8250 has an extra feature whereby it detects if the
 * LCR is written whilst busy.  If it is, then a busy detect interrupt is
 * raised, the LCR needs to be rewritten and the uart status register read.
 */
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/pm_runtime.h>

#include <asm/byteorder.h>

#include "8250.h"

/* Offsets for the DesignWare specific registers */
#define DW_UART_USR	0x1f /* UART Status Register */
#define DW_UART_CPR	0xf4 /* Component Parameter Register */
#define DW_UART_UCV	0xf8 /* UART Component Version */

/* Component Parameter Register bits */
#define DW_UART_CPR_ABP_DATA_WIDTH	(3 << 0)
#define DW_UART_CPR_AFCE_MODE		(1 << 4)
#define DW_UART_CPR_THRE_MODE		(1 << 5)
#define DW_UART_CPR_SIR_MODE		(1 << 6)
#define DW_UART_CPR_SIR_LP_MODE		(1 << 7)
#define DW_UART_CPR_ADDITIONAL_FEATURES	(1 << 8)
#define DW_UART_CPR_FIFO_ACCESS		(1 << 9)
#define DW_UART_CPR_FIFO_STAT		(1 << 10)
#define DW_UART_CPR_SHADOW		(1 << 11)
#define DW_UART_CPR_ENCODED_PARMS	(1 << 12)
#define DW_UART_CPR_DMA_EXTRA		(1 << 13)
#define DW_UART_CPR_FIFO_MODE		(0xff << 16)
/* Helper for fifo size calculation */
#define DW_UART_CPR_FIFO_SIZE(a)	(((a >> 16) & 0xff) * 16)

/* DesignWare specific register fields */
#define DW_UART_MCR_SIRE		BIT(6)

struct dwunipi8250_data {
	u8			usr_reg;
	int			line;
	int			msr_mask_on;
	int			msr_mask_off;
	struct clk		*clk;
	struct clk		*pclk;
	struct reset_control	*rst;
	struct uart_8250_dma	dma;

	unsigned int		skip_autocfg:1;
	unsigned int		uart_16550_compatible:1;
	int			rts_gpio;
	int			enable_gpio;
	u64 		chardelay;
    struct hrtimer hrt;
	void        (*serial_out)(struct uart_port *, int, int);
};

static inline int dwunipi8250_modify_msr(struct uart_port *p, int offset, int value)
{
	struct dwunipi8250_data *d = p->private_data;

	/* Override any modem control signals if needed */
	if (offset == UART_MSR) {
		value |= d->msr_mask_on;
		value &= ~d->msr_mask_off;
	}

	return value;
}

static void dwunipi8250_force_idle(struct uart_port *p)
{
	struct uart_8250_port *up = up_to_u8250p(p);

	serial8250_clear_and_reinit_fifos(up);
	(void)p->serial_in(p, UART_RX);
}

static void dwunipi8250_check_lcr(struct uart_port *p, int value)
{
	void __iomem *offset = p->membase + (UART_LCR << p->regshift);
	int tries = 1000;

	/* Make sure LCR write wasn't ignored */
	while (tries--) {
		unsigned int lcr = p->serial_in(p, UART_LCR);

		if ((value & ~UART_LCR_SPAR) == (lcr & ~UART_LCR_SPAR))
			return;

		dwunipi8250_force_idle(p);

#ifdef CONFIG_64BIT
		if (p->type == PORT_OCTEON)
			__raw_writeq(value & 0xff, offset);
		else
#endif
		if (p->iotype == UPIO_MEM32)
			writel(value, offset);
		else if (p->iotype == UPIO_MEM32BE)
			iowrite32be(value, offset);
		else
			writeb(value, offset);
	}
	/*
	 * FIXME: this deadlocks if port->lock is already held
	 * dev_err(p->dev, "Couldn't set LCR to %d\n", value);
	 */
}

static void dwunipi8250_gpio_serial_out(struct uart_port *p, int offset, int value)
{
	struct dwunipi8250_data *d = p->private_data;

	if (offset == UART_MCR) {
		if (d->rts_gpio >= 0)
			gpio_set_value(d->rts_gpio, (value & UART_MCR_RTS)? 0 : 1);
	}
	d->serial_out(p,offset,value);
}

static void dwunipi8250_serial_out(struct uart_port *p, int offset, int value)
{
	struct dwunipi8250_data *d = p->private_data;
	writeb(value, p->membase + (offset << p->regshift));

	if (offset == UART_LCR && !d->uart_16550_compatible)
			dwunipi8250_check_lcr(p, value);
	/*if (offset == UART_LCR ) {
		if (value & UART_LCR_SBC) {
			serial_port_out(p, UART_MCR, xxx);
		}
		if (!d->uart_16550_compatible)
			dwunipi8250_check_lcr(p, value);
	}*/
}

static unsigned int dwunipi8250_serial_in(struct uart_port *p, int offset)
{
	unsigned int value = readb(p->membase + (offset << p->regshift));

	return dwunipi8250_modify_msr(p, offset, value);
}

#ifdef CONFIG_64BIT
static unsigned int dwunipi8250_serial_inq(struct uart_port *p, int offset)
{
	unsigned int value;

	value = (u8)__raw_readq(p->membase + (offset << p->regshift));

	return dwunipi8250_modify_msr(p, offset, value);
}

static void dwunipi8250_serial_outq(struct uart_port *p, int offset, int value)
{
	struct dwunipi8250_data *d = p->private_data;

	value &= 0xff;
	__raw_writeq(value, p->membase + (offset << p->regshift));
	/* Read back to ensure register write ordering. */
	__raw_readq(p->membase + (UART_LCR << p->regshift));

	if (offset == UART_LCR && !d->uart_16550_compatible)
		dwunipi8250_check_lcr(p, value);
}
#endif /* CONFIG_64BIT */

static void dwunipi8250_serial_out32(struct uart_port *p, int offset, int value)
{
	struct dwunipi8250_data *d = p->private_data;

	writel(value, p->membase + (offset << p->regshift));

	if (offset == UART_LCR && !d->uart_16550_compatible)
		dwunipi8250_check_lcr(p, value);
}

static unsigned int dwunipi8250_serial_in32(struct uart_port *p, int offset)
{
	unsigned int value = readl(p->membase + (offset << p->regshift));

	return dwunipi8250_modify_msr(p, offset, value);
}

static void dwunipi8250_serial_out32be(struct uart_port *p, int offset, int value)
{
	struct dwunipi8250_data *d = p->private_data;

	iowrite32be(value, p->membase + (offset << p->regshift));

	if (offset == UART_LCR && !d->uart_16550_compatible)
		dwunipi8250_check_lcr(p, value);
}

static unsigned int dwunipi8250_serial_in32be(struct uart_port *p, int offset)
{
       unsigned int value = ioread32be(p->membase + (offset << p->regshift));

       return dwunipi8250_modify_msr(p, offset, value);
}


static bool handle_rx_dma(struct uart_8250_port *up, unsigned int iir)
{
	switch (iir & 0x3f) {
	case UART_IIR_RX_TIMEOUT:
		serial8250_rx_dma_flush(up);
		/* fall-through */
	case UART_IIR_RLSI:
		return true;
	}
	return up->dma->rx_dma(up);
}

static inline void dwunipi8250_em485_rts_after_send(struct uart_8250_port *p)
{
	unsigned char mcr = serial8250_in_MCR(p);

	if (p->port.rs485.flags & SER_RS485_RTS_AFTER_SEND)
		mcr |= UART_MCR_RTS;
	else
		mcr &= ~UART_MCR_RTS;
	serial8250_out_MCR(p, mcr);
}


int dwunipi8250_handle_irq(struct uart_port *port, unsigned int iir)
{
	unsigned char status;
	unsigned long flags;
	struct uart_8250_port *up = up_to_u8250p(port);
	struct dwunipi8250_data *d = port->private_data;
    struct circ_buf *xmit = &port->state->xmit;
    int fifolen, do_timer = 0;
    u64 interval;

	if (iir & UART_IIR_NO_INT) {
        if ((iir & UART_IIR_BUSY) == UART_IIR_BUSY) {
            /* Clear the USR */
            (void)port->serial_in(port, d->usr_reg);
            return 1;
        }
		return 0;
    }

	spin_lock_irqsave(&port->lock, flags);
	status = serial_port_in(port, UART_LSR);
	if (!up->dma && ((iir & 0x3f) == UART_IIR_RX_TIMEOUT)) {
       /*
        * There are ways to get Designware-based UARTs into a state where
        * they are asserting UART_IIR_RX_TIMEOUT but there is no actual
        * data available.  If we see such a case then we'll do a bogus
        * read.  If we don't do this then the "RX TIMEOUT" interrupt will
        * fire forever.
        *
        * This problem has only been observed so far when not in DMA mode
        * so we limit the workaround only to non-DMA mode.
        */
		if (!(status & (UART_LSR_DR | UART_LSR_BI)))
            (void) port->serial_in(port, UART_RX);
    }
 
    if (status & (UART_LSR_DR | UART_LSR_BI)) { 
        if (!up->em485 || !(up->ier & UART_IER_THRI)) { /* Don't process rx chars during transmitting - in fifo can be garbage */
            if (!up->dma || handle_rx_dma(up, iir)) {
                status = serial8250_rx_chars(up, status);
            }
        }
	}
	serial8250_modem_status(up);
    
    if (up->em485 && (iir & UART_IIR_THRI)) {
        if (!(port->x_char) && (uart_circ_empty(xmit))) {
            do_timer = 1;
        }
        if (uart_tx_stopped(port)) { 
            dwunipi8250_em485_rts_after_send(up); //???
        }
    }

	if ((!up->dma || up->dma->tx_err) && (status & UART_LSR_THRE) /*&& (iir & UART_IIR_THRI)*/)
		serial8250_tx_chars(up);

    if (do_timer) {
	    fifolen = serial_port_in(port, 0x20);
        if (up->ier & UART_IER_THRI) {
            //serial8250_clear_and_reinit_fifos(up); /* clear fifo - garbage*/
            up->ier &= ~UART_IER_THRI;
            serial_out(up, UART_IER, up->ier);
        }
        if (!hrtimer_active(&d->hrt)) {
            interval = (fifolen+1) * d->chardelay + 5000;  /* delay rts = FIFO * tim_for_one_char + 5us*/
            hrtimer_start(&d->hrt, ns_to_ktime(interval), HRTIMER_MODE_REL);
        }

    }
	spin_unlock_irqrestore(&port->lock, flags);

	return 1;
}


static int default_dwunipi8250_handle_irq(struct uart_port *p)
{
	struct uart_8250_port *up = up_to_u8250p(p);
	unsigned int iir;
	int ret;

	serial8250_rpm_get(up);
   	iir = p->serial_in(p, UART_IIR);
	ret = dwunipi8250_handle_irq(p, iir);
	serial8250_rpm_put(up);
	return ret;
}


#define BOTH_EMPTY	(UART_LSR_TEMT | UART_LSR_THRE)

static enum hrtimer_restart	dwunipi8250_hrtimer_callback(struct hrtimer *hrtp)
{
	struct dwunipi8250_data *d;
	struct uart_8250_port *up;
    unsigned char lsr;
    u64 fifolen;
    ktime_t interval;

    //printk(KERN_INFO "UART: hrt fired\n");
	d = container_of(hrtp, struct dwunipi8250_data, hrt);
    up = serial8250_get_port(d->line);

    if ((up->ier & UART_IER_THRI)) {
        /* if THR interrupt is enabled, timer is not valid anymore, port continues transmitting */
        return HRTIMER_NORESTART;
    }

	lsr = serial_in(up, UART_LSR);
	/*
	 * To provide required timeing and allow FIFO transfer,
	 * __stop_tx_rs485() must be called only when both FIFO and
	 * shift register are empty. It is for device driver to enable
	 * interrupt on TEMT.
	 */
	if ((lsr & BOTH_EMPTY) == BOTH_EMPTY) {         /* THR and FIFO are empty */
            dwunipi8250_em485_rts_after_send(up);        /* mnaage rts pin */
            serial8250_clear_and_reinit_fifos(up);  /* clear fifo - garbage*/
            up->ier |= UART_IER_RLSI | UART_IER_RDI;/* set interrupts mask to reading */
            serial_out(up, UART_IER, up->ier);
            return HRTIMER_NORESTART;               /* transmitting is finished */
    }
    
    /* read TX queue, calc delay and restart timer */
    fifolen = serial_in(up, 0x20);
    interval = (fifolen+1) * d->chardelay + 5000;  /* delay rts = FIFO * tim_for_one_char + 5us*/
    hrtimer_forward_now(hrtp, ns_to_ktime(interval));
	return HRTIMER_RESTART;
}

int dwunipi8250_em485_init(struct uart_8250_port *p)
{
    int ret;
	struct dwunipi8250_data *d = p->port.private_data;
	ret = serial8250_em485_init(p);
    /* create high res timer for 485 driving */
    hrtimer_init(&d->hrt, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    d->hrt.function = dwunipi8250_hrtimer_callback;
	return ret;
}
EXPORT_SYMBOL_GPL(dwunipi8250_em485_init);

/**
 *	dwunipi8250_em485_destroy() - put uart_8250_port into normal state
 *	@p:	uart_8250_port port instance
 *
 *	The function is used to stop rs485 software emulating on the
 *	&struct uart_8250_port* @p. The function is idempotent, so it is safe to
 *	call it multiple times.
 *
 *	The function is supposed to be called from .rs485_config callback
 *	or from any other callback protected with p->port.lock spinlock.
 *
 *	See also serial8250_em485_init()
 */
void dwunipi8250_em485_destroy(struct uart_8250_port *p)
{
	struct dwunipi8250_data *d = p->port.private_data;
	serial8250_em485_destroy(p);
    hrtimer_try_to_cancel(&d->hrt);
}
EXPORT_SYMBOL_GPL(dwunipi8250_em485_destroy);


static int dwunipi8250_rs485_config(struct uart_port *port, struct serial_rs485 *rs485)
{
	struct uart_8250_port *up = up_to_u8250p(port);
 
	/* Clamp the delays to [0, 100ms] */
	rs485->delay_rts_before_send = min(rs485->delay_rts_before_send, 100U);
	rs485->delay_rts_after_send  = min(rs485->delay_rts_after_send, 100U);
 
	port->rs485 = *rs485;
 
	/*
	 * Both dwunipi8250_em485_init and dwunipi8250_em485_destroy
	 * are idempotent
	 */
	if (rs485->flags & SER_RS485_ENABLED) {
		int ret = dwunipi8250_em485_init(up);

		if (ret) {
			rs485->flags &= ~SER_RS485_ENABLED;
			port->rs485.flags &= ~SER_RS485_ENABLED;
		}
		return ret;
 	}
 
	dwunipi8250_em485_destroy(up);
 	return 0;
 }

void dwunipi8250_do_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_8250_port *up = up_to_u8250p(port);
    unsigned char mcr = 0;

    /* mask RTS changing in case of rs485 mode */
	if (port->rs485.flags & SER_RS485_ENABLED) {
        mcr = serial8250_in_MCR(up) & UART_MCR_RTS;
    } else {
        if (mctrl & TIOCM_RTS)
            mcr |= UART_MCR_RTS;
    }
	if (mctrl & TIOCM_DTR)
        mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
    if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr = (mcr & up->mcr_mask) | up->mcr_force | up->mcr;

	serial8250_out_MCR(up, mcr);
}


static void
dwunipi8250_do_pm(struct uart_port *port, unsigned int state, unsigned int old)
{
	if (!state)
		pm_runtime_get_sync(port->dev);

	serial8250_do_pm(port, state, old);

	if (state)
		pm_runtime_put_sync_suspend(port->dev);
}

static unsigned int dwunipi8250_get_bits(unsigned int cflag)
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
	return bits;
}

static void dwunipi8250_set_termios(struct uart_port *p, struct ktermios *termios,
			       struct ktermios *old)
{
	unsigned int baud = tty_termios_baud_rate(termios);
	struct dwunipi8250_data *d = p->private_data;
	long rate;
	int ret;

	if (IS_ERR(d->clk) || !old)
		goto out;

	clk_disable_unprepare(d->clk);
	rate = clk_round_rate(d->clk, baud * 16);
	if (rate < 0)
		ret = rate;
	else if (rate == 0)
		ret = -ENOENT;
	else
		ret = clk_set_rate(d->clk, rate);
	clk_prepare_enable(d->clk);

	if (!ret)
		p->uartclk = rate;

out:
	p->status &= ~UPSTAT_AUTOCTS;
	if (termios->c_cflag & CRTSCTS)
		p->status |= UPSTAT_AUTOCTS;

	serial8250_do_set_termios(p, termios, old);
	if (termios->c_ospeed)
		d->chardelay = (dwunipi8250_get_bits(termios->c_cflag) * (u64)1000000000) / termios->c_ospeed ;
	else
		d->chardelay = 0;
}

static void dwunipi8250_set_ldisc(struct uart_port *p, struct ktermios *termios)
{
	struct uart_8250_port *up = up_to_u8250p(p);
	unsigned int mcr = p->serial_in(p, UART_MCR);

	if (up->capabilities & UART_CAP_IRDA) {
		if (termios->c_line == N_IRDA)
			mcr |= DW_UART_MCR_SIRE;
		else
			mcr &= ~DW_UART_MCR_SIRE;

		p->serial_out(p, UART_MCR, mcr);
	}
	serial8250_do_set_ldisc(p, termios);
}

/*
 * dwunipi8250_fallback_dma_filter will prevent the UART from getting just any free
 * channel on platforms that have DMA engines, but don't have any channels
 * assigned to the UART.
 *
 * REVISIT: This is a work around for limitation in the DMA Engine API. Once the
 * core problem is fixed, this function is no longer needed.
 */
static bool dwunipi8250_fallback_dma_filter(struct dma_chan *chan, void *param)
{
	return false;
}

static bool dwunipi8250_idma_filter(struct dma_chan *chan, void *param)
{
	return param == chan->device->dev->parent;
}

static int dwunipi8250_startup(struct uart_port *port)
{
	struct dwunipi8250_data *d = port->private_data;
	if (d->enable_gpio >= 0) {
		gpio_set_value(d->enable_gpio, 1);
	}
	return serial8250_do_startup(port);
}

static void dwunipi8250_gpio_setup(struct uart_port *port, struct device_node *np, struct dwunipi8250_data *data)
{
	enum of_gpio_flags flags;
	int ret, gpio_pin;

    if (!np) return;

	/* check for tx enable gpio */
	gpio_pin = of_get_named_gpio_flags(np, "rts-gpio", 0, &flags);
	if (gpio_is_valid(gpio_pin)) {
		ret = devm_gpio_request(port->dev, gpio_pin, "dw-apb-uart");
		flags = 0; // initial value of gpio output
		if (ret >= 0) {
			if (gpio_direction_output(gpio_pin, flags) >= 0) {
				data->rts_gpio = gpio_pin;
        		data->serial_out = port->serial_out;
        		port->serial_out = dwunipi8250_gpio_serial_out;
			}
		}
	}
	//} else if (d->rts_gpio == -EPROBE_DEFER) {
	//return -EPROBE_DEFER;

	/* check for enable gpio */
	gpio_pin = of_get_named_gpio_flags(np, "enable-gpio", 0, &flags);
	if (gpio_is_valid(gpio_pin)) {
		ret = devm_gpio_request(port->dev, gpio_pin, "dw-apb-uart");
		flags = 0; // initial value of gpio output
		if (ret >= 0) {
			if (gpio_direction_output(gpio_pin, flags) >= 0)
				data->enable_gpio = gpio_pin;
		}
	}
}

static void dwunipi8250_rs485_setup(struct uart_port *port, struct device_node *np, struct dwunipi8250_data *data)
{
	struct serial_rs485 *rs485conf = &port->rs485;

	rs485conf->flags = 0;

	if (of_property_read_bool(np, "rs485-rts-active-high"))
		rs485conf->flags |= SER_RS485_RTS_ON_SEND;
	else
		rs485conf->flags |= SER_RS485_RTS_AFTER_SEND;
    
	/*if (of_property_read_u32_array(np, "rs485-rts-delay",
				    rs485_delay, 2) == 0) {
		rs485conf->delay_rts_before_send = rs485_delay[0];
		rs485conf->delay_rts_after_send = rs485_delay[1];
	}*/

	if (of_property_read_bool(np, "rs485-rx-during-tx"))
		rs485conf->flags |= SER_RS485_RX_DURING_TX;

	if (of_property_read_bool(np, "linux,rs485-enabled-at-boot-time")) {
		rs485conf->flags |= SER_RS485_ENABLED;
    }
}

static void dwunipi8250_quirks(struct uart_port *p, struct dwunipi8250_data *data)
{
	if (p->dev->of_node) {
		struct device_node *np = p->dev->of_node;
		int id;

		/* get index of serial line, if found in DT aliases */
		id = of_alias_get_id(np, "serial");
		if (id >= 0)
			p->line = id;
#ifdef CONFIG_64BIT
		if (of_device_is_compatible(np, "cavium,octeon-3860-uart")) {
			p->serial_in = dwunipi8250_serial_inq;
			p->serial_out = dwunipi8250_serial_outq;
			p->flags = UPF_SKIP_TEST | UPF_SHARE_IRQ | UPF_FIXED_TYPE;
			p->type = PORT_OCTEON;
			data->usr_reg = 0x27;
			data->skip_autocfg = true;
		}
#endif
		if (of_device_is_big_endian(p->dev->of_node)) {
			p->iotype = UPIO_MEM32BE;
			p->serial_in = dwunipi8250_serial_in32be;
			p->serial_out = dwunipi8250_serial_out32be;
		}
		dwunipi8250_gpio_setup(p, np, data);
		dwunipi8250_rs485_setup(p, np, data);
	} else if (has_acpi_companion(p->dev)) {
		const struct acpi_device_id *id;

		id = acpi_match_device(p->dev->driver->acpi_match_table,
				       p->dev);
		if (id && !strcmp(id->id, "APMC0D08")) {
			p->iotype = UPIO_MEM32;
			p->regshift = 2;
			p->serial_in = dwunipi8250_serial_in32;
			data->uart_16550_compatible = true;
		}
	}

	/* Platforms with iDMA */
	if (platform_get_resource_byname(to_platform_device(p->dev),
					 IORESOURCE_MEM, "lpss_priv")) {
		data->dma.rx_param = p->dev->parent;
		data->dma.tx_param = p->dev->parent;
		data->dma.fn = dwunipi8250_idma_filter;
	}
}

static void dwunipi8250_setup_port(struct uart_port *p)
{
	struct uart_8250_port *up = up_to_u8250p(p);
	u32 reg;

	/*
	 * If the Component Version Register returns zero, we know that
	 * ADDITIONAL_FEATURES are not enabled. No need to go any further.
	 */
	if (p->iotype == UPIO_MEM32BE)
		reg = ioread32be(p->membase + DW_UART_UCV);
	else
		reg = readl(p->membase + DW_UART_UCV);
	if (!reg)
		return;

	dev_dbg(p->dev, "UniPi 485 via Designware UART version %c.%c%c\n",
		(reg >> 24) & 0xff, (reg >> 16) & 0xff, (reg >> 8) & 0xff);

	if (p->iotype == UPIO_MEM32BE)
		reg = ioread32be(p->membase + DW_UART_CPR);
	else
		reg = readl(p->membase + DW_UART_CPR);
	if (!reg)
		return;

	/* Select the type based on fifo */
	if (reg & DW_UART_CPR_FIFO_MODE) {
		p->type = PORT_16550A;
		p->flags |= UPF_FIXED_TYPE;
		p->fifosize = DW_UART_CPR_FIFO_SIZE(reg);
		up->capabilities = UART_CAP_FIFO;
	}

	if (reg & DW_UART_CPR_AFCE_MODE)
		up->capabilities |= UART_CAP_AFE;

	if (reg & DW_UART_CPR_SIR_MODE)
		up->capabilities |= UART_CAP_IRDA;
}

static int dwunipi8250_probe(struct platform_device *pdev)
{
	struct uart_8250_port uart = {};
	struct resource *regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	int irq = platform_get_irq(pdev, 0);
	struct uart_port *p = &uart.port;
	struct device *dev = &pdev->dev;
	struct dwunipi8250_data *data;
	int err;
	u32 val;

	if (!regs) {
		dev_err(dev, "no registers defined\n");
		return -EINVAL;
	}

	if (irq < 0) {
		if (irq != -EPROBE_DEFER)
			dev_err(dev, "cannot get irq\n");
		return irq;
	}

	spin_lock_init(&p->lock);
	p->mapbase	= regs->start;
	p->irq		= irq;
	p->handle_irq	= default_dwunipi8250_handle_irq;
	p->pm		= dwunipi8250_do_pm;
	p->type		= PORT_8250;
	p->flags	= UPF_SHARE_IRQ | UPF_FIXED_PORT;
	p->dev		= dev;
	p->iotype	= UPIO_MEM;
	p->serial_in	= dwunipi8250_serial_in;
	p->serial_out	= dwunipi8250_serial_out;
	p->set_ldisc	= dwunipi8250_set_ldisc;
	p->set_termios	= dwunipi8250_set_termios;
    p->set_mctrl    = dwunipi8250_do_set_mctrl;
    p->rs485_config = dwunipi8250_rs485_config;

	p->membase = devm_ioremap(dev, regs->start, resource_size(regs));
	if (!p->membase)
		return -ENOMEM;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dma.fn = dwunipi8250_fallback_dma_filter;
	data->usr_reg = DW_UART_USR;
	data->rts_gpio = -EINVAL;
	data->enable_gpio = -EINVAL;
    data->chardelay = 0;
	p->private_data = data;

	data->uart_16550_compatible = device_property_read_bool(dev,
						"snps,uart-16550-compatible");

	err = device_property_read_u32(dev, "reg-shift", &val);
	if (!err)
		p->regshift = val;

	err = device_property_read_u32(dev, "reg-io-width", &val);
	if (!err && val == 4) {
		p->iotype = UPIO_MEM32;
		p->serial_in = dwunipi8250_serial_in32;
		p->serial_out = dwunipi8250_serial_out32;
	}

	if (device_property_read_bool(dev, "dcd-override")) {
		/* Always report DCD as active */
		data->msr_mask_on |= UART_MSR_DCD;
		data->msr_mask_off |= UART_MSR_DDCD;
	}

	if (device_property_read_bool(dev, "dsr-override")) {
		/* Always report DSR as active */
		data->msr_mask_on |= UART_MSR_DSR;
		data->msr_mask_off |= UART_MSR_DDSR;
	}

	if (device_property_read_bool(dev, "cts-override")) {
		/* Always report CTS as active */
		data->msr_mask_on |= UART_MSR_CTS;
		data->msr_mask_off |= UART_MSR_DCTS;
	}

	if (device_property_read_bool(dev, "ri-override")) {
		/* Always report Ring indicator as inactive */
		data->msr_mask_off |= UART_MSR_RI;
		data->msr_mask_off |= UART_MSR_TERI;
	}

	/* Always ask for fixed clock rate from a property. */
	device_property_read_u32(dev, "clock-frequency", &p->uartclk);

	/* If there is separate baudclk, get the rate from it. */
	data->clk = devm_clk_get(dev, "baudclk");
	if (IS_ERR(data->clk) && PTR_ERR(data->clk) != -EPROBE_DEFER)
		data->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(data->clk) && PTR_ERR(data->clk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (!IS_ERR_OR_NULL(data->clk)) {
		err = clk_prepare_enable(data->clk);
		if (err)
			dev_warn(dev, "could not enable optional baudclk: %d\n",
				 err);
		else
			p->uartclk = clk_get_rate(data->clk);
	}

	/* If no clock rate is defined, fail. */
	if (!p->uartclk) {
		dev_err(dev, "clock rate not defined\n");
		err = -EINVAL;
		goto err_clk;
	}

	data->pclk = devm_clk_get(dev, "apb_pclk");
	if (IS_ERR(data->pclk) && PTR_ERR(data->pclk) == -EPROBE_DEFER) {
		err = -EPROBE_DEFER;
		goto err_clk;
	}
	if (!IS_ERR(data->pclk)) {
		err = clk_prepare_enable(data->pclk);
		if (err) {
			dev_err(dev, "could not enable apb_pclk\n");
			goto err_clk;
		}
	}

	data->rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(data->rst)) {
		err = PTR_ERR(data->rst);
		goto err_pclk;
	}
	reset_control_deassert(data->rst);

	dwunipi8250_quirks(p, data);
	if (data->enable_gpio >= 0) {
		p->startup = dwunipi8250_startup;
	}

	/* If the Busy Functionality is not implemented, don't handle it */
	if (data->uart_16550_compatible)
		p->handle_irq = NULL;

	if (!data->skip_autocfg)
		dwunipi8250_setup_port(p);

	/* If we have a valid fifosize, try hooking up DMA */
	if (p->fifosize) {
		data->dma.rxconf.src_maxburst = p->fifosize / 4;
		data->dma.txconf.dst_maxburst = p->fifosize / 4;
		uart.dma = &data->dma;
	}

	data->line = serial8250_register_8250_port(&uart);
	if (data->line < 0) {
		err = data->line;
		goto err_reset;
	}

	platform_set_drvdata(pdev, data);

    if (p->rs485.flags & SER_RS485_ENABLED) {
        dwunipi8250_em485_init(serial8250_get_port(data->line));
    }

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;

err_reset:
	reset_control_assert(data->rst);

err_pclk:
	if (!IS_ERR(data->pclk))
		clk_disable_unprepare(data->pclk);

err_clk:
	if (!IS_ERR(data->clk))
		clk_disable_unprepare(data->clk);

	return err;
}

static int dwunipi8250_remove(struct platform_device *pdev)
{
	struct dwunipi8250_data *data = platform_get_drvdata(pdev);

	pm_runtime_get_sync(&pdev->dev);

	serial8250_unregister_port(data->line);

	reset_control_assert(data->rst);

	if (!IS_ERR(data->pclk))
		clk_disable_unprepare(data->pclk);

	if (!IS_ERR(data->clk))
		clk_disable_unprepare(data->clk);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dwunipi8250_suspend(struct device *dev)
{
	struct dwunipi8250_data *data = dev_get_drvdata(dev);

	serial8250_suspend_port(data->line);

	return 0;
}

static int dwunipi8250_resume(struct device *dev)
{
	struct dwunipi8250_data *data = dev_get_drvdata(dev);

	serial8250_resume_port(data->line);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM
static int dwunipi8250_runtime_suspend(struct device *dev)
{
	struct dwunipi8250_data *data = dev_get_drvdata(dev);

	if (!IS_ERR(data->clk))
		clk_disable_unprepare(data->clk);

	if (!IS_ERR(data->pclk))
		clk_disable_unprepare(data->pclk);

	return 0;
}

static int dwunipi8250_runtime_resume(struct device *dev)
{
	struct dwunipi8250_data *data = dev_get_drvdata(dev);

	if (!IS_ERR(data->pclk))
		clk_prepare_enable(data->pclk);

	if (!IS_ERR(data->clk))
		clk_prepare_enable(data->clk);

	return 0;
}
#endif

static const struct dev_pm_ops dwunipi8250_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwunipi8250_suspend, dwunipi8250_resume)
	SET_RUNTIME_PM_OPS(dwunipi8250_runtime_suspend, dwunipi8250_runtime_resume, NULL)
};

static const struct of_device_id dwunipi8250_of_match[] = {
	{ .compatible = "unipi,dw-apb-485" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, dwunipi8250_of_match);

static const struct acpi_device_id dwunipi8250_acpi_match[] = {
	{ },
};
MODULE_DEVICE_TABLE(acpi, dwunipi8250_acpi_match);

static struct platform_driver dwunipi8250_platform_driver = {
	.driver = {
		.name		= "dw-apb-485",
		.pm		= &dwunipi8250_pm_ops,
		.of_match_table	= dwunipi8250_of_match,
		.acpi_match_table = ACPI_PTR(dwunipi8250_acpi_match),
	},
	.probe			= dwunipi8250_probe,
	.remove			= dwunipi8250_remove,
};

module_platform_driver(dwunipi8250_platform_driver);

MODULE_AUTHOR("Miroslav Ondra");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("UniPi 485 port via Synopsys DesignWare 8250 serial uart driver");
MODULE_ALIAS("platform:dw-apb-485");
