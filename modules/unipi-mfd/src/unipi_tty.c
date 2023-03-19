/*
 * Implements line discpline for using with Neuron/Axon.
 * 
 * Author: Miroslav Ondra <ondra@faster.cz>
 *
 * Derivated from n_tty.c --- implements the N_PROFIBUS line discipline.
 *
 */

#include <linux/types.h>
#include <linux/major.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/fcntl.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/timer.h>
#include <linux/ctype.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/bitops.h>
#include <linux/audit.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/vmalloc.h>
#include <linux/version.h>

#include "unipi_tty.h"
#include "unipi_uart.h"

#if UNIPI_TTY_DETAILED_DEBUG > 0
# define unipi_tty_trace(f, args...)	printk(f, ##args)
#else
# define unipi_tty_trace(f, args...)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,13,0)
static void (*alias_n_tty_receive_buf)(struct tty_struct *tty, const unsigned char *cp,
			      const char *fp, int count);
static int (*alias_n_tty_receive_buf2)(struct tty_struct *tty, const unsigned char *cp,
			      const char *fp, int count);
#else
static void (*alias_n_tty_receive_buf)(struct tty_struct *tty, const unsigned char *cp,
			      char *fp, int count);
static int (*alias_n_tty_receive_buf2)(struct tty_struct *tty, const unsigned char *cp,
			      char *fp, int count);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,17,0)
static int (*alias_n_tty_ioctl)(struct tty_struct *tty, struct file *file,
               unsigned int cmd, unsigned long arg);
#else
static int (*alias_n_tty_ioctl)(struct tty_struct *tty,
               unsigned int cmd, unsigned long arg);
#endif
static int (*alias_n_tty_open)(struct tty_struct *tty);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,13,0)
static void unipi_tty_receive_buf(struct tty_struct *tty, const unsigned char *cp,
                                  const char *fp, int count)
#else
static void unipi_tty_receive_buf(struct tty_struct *tty, const unsigned char *cp,
			      char *fp, int count)
#endif
{
	int is_parmrk = I_PARMRK(tty);
	if (is_parmrk)
		tty->termios.c_iflag = tty->termios.c_iflag & (~PARMRK);
	unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Buf start.");
	alias_n_tty_receive_buf(tty, cp, fp, count);
	if (is_parmrk)
		tty->termios.c_iflag = tty->termios.c_iflag | (PARMRK);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,13,0)
static int unipi_tty_receive_buf2(struct tty_struct *tty, const unsigned char *cp,
                                  const char *fp, int count)
#else
static int unipi_tty_receive_buf2(struct tty_struct *tty, const unsigned char *cp,
			      char *fp, int count)
#endif
{
	int ret;
	int is_parmrk = I_PARMRK(tty);
	if (is_parmrk)
		tty->termios.c_iflag = tty->termios.c_iflag & (~PARMRK);
	unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Buf2 start.");
	ret = alias_n_tty_receive_buf2(tty, cp, fp, count);
	if (is_parmrk)
		tty->termios.c_iflag = tty->termios.c_iflag | (PARMRK);
	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,17,0)
static int unipi_tty_ioctl(struct tty_struct *tty, struct file *file,
                           unsigned int cmd, unsigned long arg)
#else
static int unipi_tty_ioctl(struct tty_struct *tty,
                           unsigned int cmd, unsigned long arg)
#endif
{
	int retval;

	unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Ioctl start. cmd=%x", cmd);
	switch (cmd) {
		case 0x5481:
		case 0x5480:
			if (tty->ops->ioctl != NULL) {
				retval = tty->ops->ioctl(tty, cmd, arg);
				if (retval != -ENOIOCTLCMD)
					return retval;
			}
	}
	return  alias_n_tty_ioctl(tty, file, cmd, arg);
}

static int unipi_is_port_unipi(struct tty_struct *tty)
{
    struct uart_state *state = tty->driver_data;
    struct uart_port *uport;

	if (! state) return 0;
    uport = state->uart_port;
	if (! uport) return 0;
	//printk("LDISC type=%d\n", uport->type);
    return (uport->type == PORT_UNIPI);
}

/**
 *	unipi_tty_open		-	open an ldisc
 *	@tty: terminal to open
 *
 *	Return error if tty is nor UNIPI port 
 */

static int unipi_tty_open(struct tty_struct *tty)
{
	if (! unipi_is_port_unipi(tty)) {
		return -ENOMEM;
	}
	return alias_n_tty_open(tty);
}


struct tty_ldisc_ops unipi_tty_ldisc;

int unipi_tty_init(void)
{
	int err;
	unipi_tty_trace(KERN_INFO "UNIPISPI: TTY Init\n");

	memset(&unipi_tty_ldisc, 0, sizeof(unipi_tty_ldisc));
	n_tty_inherit_ops(&unipi_tty_ldisc);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,13,0)
	unipi_tty_ldisc.num             = N_PROFIBUS_FDL;
#else
	unipi_tty_ldisc.magic           = TTY_LDISC_MAGIC;
#endif
	unipi_tty_ldisc.name            = "unipi_tty";
	unipi_tty_ldisc.owner           = THIS_MODULE;

	alias_n_tty_receive_buf = unipi_tty_ldisc.receive_buf;
	alias_n_tty_receive_buf2 = unipi_tty_ldisc.receive_buf2;
	alias_n_tty_ioctl = unipi_tty_ldisc.ioctl;
	alias_n_tty_open = unipi_tty_ldisc.open;

	unipi_tty_ldisc.receive_buf     = unipi_tty_receive_buf;
	unipi_tty_ldisc.receive_buf2	= unipi_tty_receive_buf2;
	unipi_tty_ldisc.ioctl	        = unipi_tty_ioctl;
	unipi_tty_ldisc.open	        = unipi_tty_open;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,13,0)
	err = tty_register_ldisc(&unipi_tty_ldisc);
#else
	err = tty_register_ldisc(N_PROFIBUS_FDL, &unipi_tty_ldisc);
#endif
	if (err) {
		printk(KERN_INFO "UniPi line discipline registration failed. (%d)", err);
		return err;
	}
	return 0;
}

void unipi_tty_exit(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,13,0)
	tty_unregister_ldisc(&unipi_tty_ldisc);
#else
	tty_unregister_ldisc(N_PROFIBUS_FDL);
#endif
}
