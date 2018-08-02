/*
 * Implements line discpline for using with Neuron/Axon.
 *
 * Derivated from n_tty.c --- implements the N_TTY line discipline.
 *
 * This code used to be in tty_io.c, but things are getting hairy
 * enough that it made sense to split things off.  (The N_TTY
 * processing has changed so much that it's hardly recognizable,
 * anyway...)
 *
 * Note that the open routine for N_TTY is guaranteed never to return
 * an error.  This is because Linux will fall back to setting a line
 * to N_TTY if it can not switch to any other line discipline.
 *
 * Written by Theodore Ts'o, Copyright 1994.
 *
 * This file also contains code originally written by Linus Torvalds,
 * Copyright 1991, 1992, 1993, and by Julian Cowley, Copyright 1994.
 *
 * This file may be redistributed under the terms of the GNU General Public
 * License.
 *
 * Reduced memory usage for older ARM systems  - Russell King.
 *
 * 2000/01/20   Fixed SMP locking on put_tty_queue using bits of
 *		the patch by Andrew J. Kroll <ag784@freenet.buffalo.edu>
 *		who actually finally proved there really was a race.
 *
 * 2002/03/18   Implemented unipi_tty_wakeup to send SIGIO POLL_OUTs to
 *		waiting writing processes-Sapan Bhatia <sapan@corewars.org>.
 *		Also fixed a bug in BLOCKING mode where unipi_tty_write returns
 *		EAGAIN
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

#include "unipi_tty.h"

/* number of characters left in xmit buffer before select has we have room */
#define WAKEUP_CHARS 256

/*
 * This defines the low- and high-watermarks for throttling and
 * unthrottling the TTY driver.  These watermarks are used for
 * controlling the space in the read buffer.
 */
#define TTY_THRESHOLD_THROTTLE		128 /* now based on remaining room */
#define TTY_THRESHOLD_UNTHROTTLE	128


// #undef UNIPI_TTY_TRACE
#ifdef NEURONSPI_DETAILED_DEBUG
# define unipi_tty_trace(f, args...)	trace_printk(f, ##args)
#else
# define unipi_tty_trace(f, args...)
#endif

struct unipi_tty_data {
	/* producer-published */
	size_t read_head;
	size_t commit_head;
	//DECLARE_BITMAP(char_map, 256);

	/* private to unipi_tty_receive_overrun (single-threaded) */
	/* non-atomic */
	bool no_room;

	/* shared by producer and consumer */
	char read_buf[N_TTY_BUF_SIZE];

	/* consumer-published */
	size_t read_tail;


	struct mutex atomic_read_lock;
	struct mutex output_lock;
};

static inline size_t read_cnt(struct unipi_tty_data *ldata)
{
	return ldata->read_head - ldata->read_tail;
}

static inline unsigned char read_buf(struct unipi_tty_data *ldata, size_t i)
{
	return ldata->read_buf[i & (N_TTY_BUF_SIZE - 1)];
}

static inline unsigned char *read_buf_addr(struct unipi_tty_data *ldata, size_t i)
{
	return &ldata->read_buf[i & (N_TTY_BUF_SIZE - 1)];
}


/**
 *	tty_throttle_safe	-	flow control
 *	@tty: terminal
 *
 *	Similar to tty_throttle() but will only attempt throttle
 *	if tty->flow_change is TTY_THROTTLE_SAFE. Prevents an accidental
 *	throttle due to race conditions when throttling is conditional
 *	on factors evaluated prior to throttling.
 *
 *	Returns 0 if tty is throttled (or was already throttled)
 */

int unipi_tty_throttle_safe(struct tty_struct *tty)
{
	int ret = 0;

	mutex_lock(&tty->throttle_mutex);
	if (!tty_throttled(tty)) {
		if (tty->flow_change != TTY_THROTTLE_SAFE)
			ret = 1;
		else {
			set_bit(TTY_THROTTLED, &tty->flags);
			if (tty->ops->throttle)
				tty->ops->throttle(tty);
		}
	}
	mutex_unlock(&tty->throttle_mutex);

	return ret;
}

/**
 *	tty_unthrottle_safe	-	flow control
 *	@tty: terminal
 *
 *	Similar to tty_unthrottle() but will only attempt unthrottle
 *	if tty->flow_change is TTY_UNTHROTTLE_SAFE. Prevents an accidental
 *	unthrottle due to race conditions when unthrottling is conditional
 *	on factors evaluated prior to unthrottling.
 *
 *	Returns 0 if tty is unthrottled (or was already unthrottled)
 */

int unipi_tty_unthrottle_safe(struct tty_struct *tty)
{
	int ret = 0;

	mutex_lock(&tty->throttle_mutex);
	if (tty_throttled(tty)) {
		if (tty->flow_change != TTY_UNTHROTTLE_SAFE)
			ret = 1;
		else {
			clear_bit(TTY_THROTTLED, &tty->flags);
			if (tty->ops->unthrottle)
				tty->ops->unthrottle(tty);
		}
	}
	mutex_unlock(&tty->throttle_mutex);

	return ret;
}


/**
 *	unipi_tty_kick_worker - start input worker (if required)
 *	@tty: terminal
 *
 *	Re-schedules the flip buffer work if it may have stopped
 *
 *	Caller holds exclusive termios_rwsem
 *	   or
 *	unipi_tty_read()/consumer path:
 *		holds non-exclusive termios_rwsem
 */

static void unipi_tty_kick_worker(struct tty_struct *tty)
{
	struct unipi_tty_data *ldata = tty->disc_data;
    struct tty_port *port = tty->port;

	/* Did the input worker stop? Restart it */
	if (unlikely(ldata->no_room)) {
		ldata->no_room = 0;

		WARN_RATELIMIT(tty->port->itty == NULL,
				"scheduling with invalid itty\n");
		/* see if ldisc has been killed - if so, this means that
		 * even though the ldisc has been halted and ->buf.work
		 * cancelled, ->buf.work is about to be rescheduled
		 */
		WARN_RATELIMIT(test_bit(TTY_LDISC_HALTED, &tty->flags),
			       "scheduling buffer work for halted ldisc\n");
		//tty_buffer_restart_work(tty->port);
        queue_work(system_unbound_wq, &port->buf.work);
	}
}

static ssize_t chars_in_buffer(struct tty_struct *tty)
{
	struct unipi_tty_data *ldata = tty->disc_data;
	ssize_t n = 0;

    n = ldata->commit_head - ldata->read_tail;
	return n;
}

/**
 *	unipi_tty_write_wakeup	-	asynchronous I/O notifier
 *	@tty: tty device
 *
 *	Required for the ptys, serial driver etc. since processes
 *	that attach themselves to the master and rely on ASYNC
 *	IO must be woken up
 */

static void unipi_tty_write_wakeup(struct tty_struct *tty)
{
	unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Wakeup start.");

	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	kill_fasync(&tty->fasync, SIGIO, POLL_OUT);
}

static void unipi_tty_check_throttle(struct tty_struct *tty)
{
	struct unipi_tty_data *ldata = tty->disc_data;

	/*
	 * Check the remaining room for the input canonicalization
	 * mode.  We don't want to throttle the driver if we're in
	 * canonical mode and don't have a newline yet!
	 */

	while (1) {
		int throttled;
		tty_set_flow_change(tty, TTY_THROTTLE_SAFE);
		if (N_TTY_BUF_SIZE - read_cnt(ldata) >= TTY_THRESHOLD_THROTTLE)
			break;
		throttled = unipi_tty_throttle_safe(tty);
		if (!throttled)
			break;
	}
	__tty_set_flow_change(tty, 0);
}

static void unipi_tty_check_unthrottle(struct tty_struct *tty)
{
	if (tty->driver->type == TTY_DRIVER_TYPE_PTY) {
		if (chars_in_buffer(tty) > TTY_THRESHOLD_UNTHROTTLE)
			return;
		unipi_tty_kick_worker(tty);
		tty_wakeup(tty->link);
		return;
	}

	/* If there is enough space in the read buffer now, let the
	 * low-level driver know. We use chars_in_buffer() to
	 * check the buffer, as it now knows about canonical mode.
	 * Otherwise, if the driver is throttled and the line is
	 * longer than TTY_THRESHOLD_UNTHROTTLE in canonical mode,
	 * we won't get any more characters.
	 */

	while (1) {
		int unthrottled;
		tty_set_flow_change(tty, TTY_UNTHROTTLE_SAFE);
		if (chars_in_buffer(tty) > TTY_THRESHOLD_UNTHROTTLE)
			break;
		unipi_tty_kick_worker(tty);
		unthrottled = unipi_tty_unthrottle_safe(tty);
		if (!unthrottled)
			break;
	}
	__tty_set_flow_change(tty, 0);
}

/**
 *	reset_buffer_flags	-	reset buffer state
 *	@tty: terminal to reset
 *
 *	Reset the read buffer counters and clear the flags.
 *	Called from unipi_tty_open() and unipi_tty_flush_buffer().
 *
 *	Locking: caller holds exclusive termios_rwsem
 *		 (or locking is not required)
 */

static void reset_buffer_flags(struct unipi_tty_data *ldata)
{
	ldata->read_head =  ldata->read_tail = 0;
	ldata->commit_head = 0;

}

static void unipi_tty_packet_mode_flush(struct tty_struct *tty)
{
	unsigned long flags;

	if (tty->link->packet) {
		spin_lock_irqsave(&tty->ctrl_lock, flags);
		tty->ctrl_status |= TIOCPKT_FLUSHREAD;
		spin_unlock_irqrestore(&tty->ctrl_lock, flags);
		wake_up_interruptible(&tty->link->read_wait);
	}
}

/**
 *	unipi_tty_flush_buffer	-	clean input queue
 *	@tty:	terminal device
 *
 *	Flush the input buffer. Called when the tty layer wants the
 *	buffer flushed (eg at hangup) or when the N_TTY line discipline
 *	internally has to clean the pending queue (for example some signals).
 *
 *	Holds termios_rwsem to exclude producer/consumer while
 *	buffer indices are reset.
 *
 *	Locking: ctrl_lock, exclusive termios_rwsem
 */

static void unipi_tty_flush_buffer(struct tty_struct *tty)
{
	unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Flush start.");

	down_write(&tty->termios_rwsem);
	reset_buffer_flags(tty->disc_data);
	unipi_tty_kick_worker(tty);

	if (tty->link)
		unipi_tty_packet_mode_flush(tty);
	up_write(&tty->termios_rwsem);
}



static void __receive_buf(struct tty_struct *tty, const unsigned char *cp,
			  char *fp, int count)
{
	struct unipi_tty_data *ldata = tty->disc_data;
	size_t n, head;

	head = ldata->read_head & (N_TTY_BUF_SIZE - 1);
	n = min_t(size_t, count, N_TTY_BUF_SIZE - head);
	memcpy(read_buf_addr(ldata, head), cp, n);
	ldata->read_head += n;
	cp += n;
	count -= n;

	head = ldata->read_head & (N_TTY_BUF_SIZE - 1);
	n = min_t(size_t, count, N_TTY_BUF_SIZE - head);
	memcpy(read_buf_addr(ldata, head), cp, n);
	ldata->read_head += n;

	/* publish read_head to consumer */
	smp_store_release(&ldata->commit_head, ldata->read_head);

	if (read_cnt(ldata)) {
		kill_fasync(&tty->fasync, SIGIO, POLL_IN);
		wake_up_interruptible_poll(&tty->read_wait, POLLIN);
	}
}

/**
 *	unipi_tty_receive_buf_common	-	process input
 *	@tty: device to receive input
 *	@cp: input chars
 *	@fp: flags for each char (if NULL, all chars are TTY_NORMAL)
 *	@count: number of input chars in @cp
 *
 *	Called by the terminal driver when a block of characters has
 *	been received. This function must be called from soft contexts
 *	not from interrupt context. The driver is responsible for making
 *	calls one at a time and in order (or using flush_to_ldisc)
 *
 *	Returns the # of input chars from @cp which were processed.
 *
 *	In canonical mode, the maximum line length is 4096 chars (including
 *	the line termination char); lines longer than 4096 chars are
 *	truncated. After 4095 chars, input data is still processed but
 *	not stored. Overflow processing ensures the tty can always
 *	receive more input until at least one line can be read.
 *
 *	In non-canonical mode, the read buffer will only accept 4095 chars;
 *	this provides the necessary space for a newline char if the input
 *	mode is switched to canonical.
 *
 *	Note it is possible for the read buffer to _contain_ 4096 chars
 *	in non-canonical mode: the read buffer could already contain the
 *	maximum canon line of 4096 chars when the mode is switched to
 *	non-canonical.
 *
 *	unipi_tty_receive_buf()/producer path:
 *		claims non-exclusive termios_rwsem
 *		publishes commit_head or canon_head
 */
static int
unipi_tty_receive_buf_common(struct tty_struct *tty, const unsigned char *cp,
			 char *fp, int count, int flow)
{
	struct unipi_tty_data *ldata = tty->disc_data;
	int room, n, rcvd = 0;

	down_read(&tty->termios_rwsem);

	while (1) {

		size_t tail = smp_load_acquire(&ldata->read_tail);

		room = N_TTY_BUF_SIZE - (ldata->read_head - tail);
		room--;
		if (room <= 0) {
            room = 0;
			ldata->no_room = flow;
		}
		n = min(count, room);
		if (!n)
			break;

		__receive_buf(tty, cp, fp, n);

		cp += n;
		count -= n;
		rcvd += n;
	}

	tty->receive_room = room;

	/* Unthrottle if handling overflow on pty */
	if (tty->driver->type == TTY_DRIVER_TYPE_PTY) {
	} else
		unipi_tty_check_throttle(tty);

	up_read(&tty->termios_rwsem);

	return rcvd;
}

static void unipi_tty_receive_buf(struct tty_struct *tty, const unsigned char *cp,
			      char *fp, int count)
{
	unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Buf start.");
	unipi_tty_receive_buf_common(tty, cp, fp, count, 0);
}

static int unipi_tty_receive_buf2(struct tty_struct *tty, const unsigned char *cp,
			      char *fp, int count)
{
	unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Buf2 start.");
	return unipi_tty_receive_buf_common(tty, cp, fp, count, 1);
}

/**
 *	unipi_tty_set_termios	-	termios data changed
 *	@tty: terminal
 *	@old: previous data
 *
 *	Called by the tty layer when the user changes termios flags so
 *	that the line discipline can plan ahead. This function cannot sleep
 *	and is protected from re-entry by the tty layer. The user is
 *	guaranteed that this function will not be re-entered or in progress
 *	when the ldisc is closed.
 *
 *	Locking: Caller holds tty->termios_rwsem
 */

static void unipi_tty_set_termios(struct tty_struct *tty, struct ktermios *old)
{
	struct unipi_tty_data *ldata = tty->disc_data;
    int lx=0; int ix=0; int ox=0; int cx = 0;
    if (old) {
        lx = old->c_lflag; 
        ix = old->c_iflag; 
        ox = old->c_oflag; 
        cx = old->c_cflag; 
    }
	unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Termios new:0x%x %x %x %x old:0x%x %x %x %x", \
         tty->termios.c_lflag, tty->termios.c_iflag, tty->termios.c_oflag,  tty->termios.c_cflag,\
         lx, ix, ox, cx);
	if (!old || (old->c_lflag ^ tty->termios.c_lflag) & (ICANON | EXTPROC)) {
		ldata->commit_head = ldata->read_head;
	}

	/*
	 * Fix tty hang when I_IXON(tty) is cleared, but the tty
	 * been stopped by STOP_CHAR(tty) before it.
	 */
	if (!I_IXON(tty) && old && (old->c_iflag & IXON) && !tty->flow_stopped) {
		start_tty(tty);
	}

	/* The termios change make the tty ready for I/O */
	wake_up_interruptible(&tty->write_wait);
	wake_up_interruptible(&tty->read_wait);
}

/**
 *	unipi_tty_close		-	close the ldisc for this tty
 *	@tty: device
 *
 *	Called from the terminal layer when this line discipline is
 *	being shut down, either because of a close or becsuse of a
 *	discipline change. The function will not be called while other
 *	ldisc methods are in progress.
 */

static void unipi_tty_close(struct tty_struct *tty)
{
	struct unipi_tty_data *ldata = tty->disc_data;

	if (tty->link)
		unipi_tty_packet_mode_flush(tty);

	vfree(ldata);
	tty->disc_data = NULL;
	unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Close OK.");
}

/**
 *	unipi_tty_open		-	open an ldisc
 *	@tty: terminal to open
 *
 *	Called when this line discipline is being attached to the
 *	terminal device. Can sleep. Called serialized so that no
 *	other events will occur in parallel. No further open will occur
 *	until a close.
 */

static int unipi_tty_open(struct tty_struct *tty)
{
	struct unipi_tty_data *ldata;

	/* Currently a malloc failure here can panic */
	ldata = vmalloc(sizeof(*ldata));
	if (!ldata)
		goto err;

	mutex_init(&ldata->atomic_read_lock);
	mutex_init(&ldata->output_lock);

	tty->disc_data = ldata;
	reset_buffer_flags(tty->disc_data);
	ldata->no_room = 0;
	tty->closing = 0;
	/* indicate buffer work may resume */
	clear_bit(TTY_LDISC_HALTED, &tty->flags);
	unipi_tty_set_termios(tty, NULL);
	tty_unthrottle(tty);

	unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Open OK.");

	return 0;
err:
	return -ENOMEM;
}

static inline int input_available_p(struct tty_struct *tty, int poll)
{
	struct unipi_tty_data *ldata = tty->disc_data;
	int amt = poll && !TIME_CHAR(tty) && MIN_CHAR(tty) ? MIN_CHAR(tty) : 1;

    return ldata->commit_head - ldata->read_tail >= amt;
}

/**
 *	copy_from_read_buf	-	copy read data directly
 *	@tty: terminal device
 *	@b: user data
 *	@nr: size of data
 *
 *	Helper function to speed up unipi_tty_read.  It is only called when
 *	ICANON is off; it copies characters straight from the tty queue to
 *	user space directly.  It can be profitably called twice; once to
 *	drain the space from the tail pointer to the (physical) end of the
 *	buffer, and once to drain the space from the (physical) beginning of
 *	the buffer to head pointer.
 *
 *	Called under the ldata->atomic_read_lock sem
 *
 *	unipi_tty_read()/consumer path:
 *		caller holds non-exclusive termios_rwsem
 *		read_tail published
 */

static int copy_from_read_buf(struct tty_struct *tty,
				      unsigned char __user **b,
				      size_t *nr)

{
	struct unipi_tty_data *ldata = tty->disc_data;
	int retval;
	size_t n;
	size_t head = smp_load_acquire(&ldata->commit_head);
	size_t tail = ldata->read_tail & (N_TTY_BUF_SIZE - 1);

	retval = 0;
	n = min(head - ldata->read_tail, N_TTY_BUF_SIZE - tail);
	n = min(*nr, n);
	if (n) {
		const unsigned char *from = read_buf_addr(ldata, tail);
		retval = copy_to_user(*b, from, n);
		n -= retval;
		//tty_audit_add_data(tty, from, n);
		smp_store_release(&ldata->read_tail, ldata->read_tail + n);
		*b += n;
		*nr -= n;
	}
	return retval;
}



/**
 *	unipi_tty_read		-	read function for tty
 *	@tty: tty device
 *	@file: file object
 *	@buf: userspace buffer pointer
 *	@nr: size of I/O
 *
 *	Perform reads for the line discipline. We are guaranteed that the
 *	line discipline will not be closed under us but we may get multiple
 *	parallel readers and must handle this ourselves. We may also get
 *	a hangup. Always called in user context, may sleep.
 *
 *	This code must be sure never to sleep through a hangup.
 *
 *	unipi_tty_read()/consumer path:
 *		claims non-exclusive termios_rwsem
 *		publishes read_tail
 */

static ssize_t unipi_tty_read(struct tty_struct *tty, struct file *file,
			 unsigned char __user *buf, size_t nr)
{
	struct unipi_tty_data *ldata = tty->disc_data;
    struct tty_port *port = tty->port;
	unsigned char __user *b = buf;
	DEFINE_WAIT_FUNC(wait, woken_wake_function);
	int minimum, time;
	ssize_t retval = 0;
	long timeout;
	int packet;
	size_t tail;

	/*
	 *	Internal serialization of reads.
	 */
	unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Read start.");
	if (file->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&ldata->atomic_read_lock))
			return -EAGAIN;
	} else {
		if (mutex_lock_interruptible(&ldata->atomic_read_lock))
			return -ERESTARTSYS;
	}

	down_read(&tty->termios_rwsem);

	minimum = time = 0;
	timeout = MAX_SCHEDULE_TIMEOUT;

	minimum = MIN_CHAR(tty);
	if (minimum) {
		time = (HZ / 10) * TIME_CHAR(tty);
	} else {
		timeout = (HZ / 10) * TIME_CHAR(tty);
		minimum = 1;
	}

	packet = tty->packet;
	tail = ldata->read_tail;

	add_wait_queue(&tty->read_wait, &wait);
	while (nr) {
		/* First test for status change. */
		if (packet && tty->link->ctrl_status) {		
 			unsigned char cs;
			if (b != buf)
				break;
			spin_lock_irq(&tty->link->ctrl_lock);
			cs = tty->link->ctrl_status;
			tty->link->ctrl_status = 0;
			spin_unlock_irq(&tty->link->ctrl_lock);
			if (put_user(cs, b)) {
				retval = -EFAULT;
				break;
			}
			b++;
			nr--;
			break;
		}

		if (!input_available_p(tty, 0)) {
			up_read(&tty->termios_rwsem);
			//tty_buffer_flush_work(tty->port);
            flush_work(&port->buf.work);
			down_read(&tty->termios_rwsem);
			if (!input_available_p(tty, 0)) {
				if (test_bit(TTY_OTHER_CLOSED, &tty->flags)) {
					retval = -EIO;
					break;
				}
				if (tty_hung_up_p(file))
					break;
				/*
				 * Abort readers for ttys which never actually
				 * get hung up.  See __tty_hangup().
				 */
#ifdef TTY_HUPPING                 
				if (test_bit(TTY_HUPPING, &tty->flags))
					break;
#endif
				if (!timeout)
					break;
				if (file->f_flags & O_NONBLOCK) {
					retval = -EAGAIN;
					break;
				}
				if (signal_pending(current)) {
					retval = -ERESTARTSYS;
					break;
				}
				up_read(&tty->termios_rwsem);

				timeout = wait_woken(&wait, TASK_INTERRUPTIBLE,
						timeout);

				down_read(&tty->termios_rwsem);
				continue;
			}
		}

        {
			int uncopied;

			/* Deal with packet mode. */
			if (packet && b == buf) {
				if (put_user(TIOCPKT_DATA, b)) {
					retval = -EFAULT;
					break;
				}
				b++;
				nr--;
			}

			uncopied = copy_from_read_buf(tty, &b, &nr);
			uncopied += copy_from_read_buf(tty, &b, &nr);
			if (uncopied) {
				retval = -EFAULT;
				break;
			}
		}

		unipi_tty_check_unthrottle(tty);

		if (b - buf >= minimum)
			break;
		if (time)
			timeout = time;
	}
	if (tail != ldata->read_tail)
		unipi_tty_kick_worker(tty);
	up_read(&tty->termios_rwsem);

	remove_wait_queue(&tty->read_wait, &wait);
	mutex_unlock(&ldata->atomic_read_lock);

	if (b - buf)
		retval = b - buf;

	return retval;
}

/**
 *	unipi_tty_write		-	write function for tty
 *	@tty: tty device
 *	@file: file object
 *	@buf: userspace buffer pointer
 *	@nr: size of I/O
 *
 *	Write function of the terminal device.  This is serialized with
 *	respect to other write callers but not to termios changes, reads
 *	and other such events.  Since the receive code will echo characters,
 *	thus calling driver write methods, the output_lock is used in
 *	the output processing functions called here as well as in the
 *	echo processing function to protect the column state and space
 *	left in the buffer.
 *
 *	This code must be sure never to sleep through a hangup.
 *
 *	Locking: output_lock to protect column state and space left
 *		 (note that the process_output*() functions take this
 *		  lock themselves)
 */

static ssize_t unipi_tty_write(struct tty_struct *tty, struct file *file,
			   const unsigned char *buf, size_t nr)
{
	const unsigned char *b = buf;
	DEFINE_WAIT_FUNC(wait, woken_wake_function);
    struct unipi_tty_data *ldata;
	int c;
	ssize_t retval = 0;

	unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Write start.");
	down_read(&tty->termios_rwsem);

	add_wait_queue(&tty->write_wait, &wait);
	while (1) {
		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			break;
		}
		if (tty_hung_up_p(file) || (tty->link && !tty->link->count)) {
			retval = -EIO;
			break;
		}
        
		ldata = tty->disc_data;

		while (nr > 0) {
			mutex_lock(&ldata->output_lock);
			c = tty->ops->write(tty, b, nr);
			mutex_unlock(&ldata->output_lock);
			if (c < 0) {
				retval = c;
				goto break_out;
			}
			if (!c)
				break;
			b += c;
			nr -= c;
		}
		
		if (!nr)
			break;
		if (file->f_flags & O_NONBLOCK) {
			retval = -EAGAIN;
			break;
		}
		up_read(&tty->termios_rwsem);

		wait_woken(&wait, TASK_INTERRUPTIBLE, MAX_SCHEDULE_TIMEOUT);

		down_read(&tty->termios_rwsem);
	}
break_out:
	remove_wait_queue(&tty->write_wait, &wait);
	if (nr && tty->fasync)
		set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	up_read(&tty->termios_rwsem);
	return (b - buf) ? b - buf : retval;
}

/**
 *	unipi_tty_poll		-	poll method for N_TTY
 *	@tty: terminal device
 *	@file: file accessing it
 *	@wait: poll table
 *
 *	Called when the line discipline is asked to poll() for data or
 *	for special events. This code is not serialized with respect to
 *	other events save open/close.
 *
 *	This code must be sure never to sleep through a hangup.
 *	Called without the kernel lock held - fine
 */

static unsigned int unipi_tty_poll(struct tty_struct *tty, struct file *file,
							poll_table *wait)
{
	unsigned int mask = 0;
    struct tty_port *port = tty->port;

	//unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Poll start.");
	poll_wait(file, &tty->read_wait, wait);
	poll_wait(file, &tty->write_wait, wait);
	if (input_available_p(tty, 1))
		mask |= POLLIN | POLLRDNORM;
	else {
		//tty_buffer_flush_work(tty->port);
        flush_work(&port->buf.work);
		if (input_available_p(tty, 1))
			mask |= POLLIN | POLLRDNORM;
	}
	if (tty->packet && tty->link->ctrl_status)
		mask |= POLLPRI | POLLIN | POLLRDNORM;
	if (test_bit(TTY_OTHER_CLOSED, &tty->flags))
		mask |= POLLHUP;
	if (tty_hung_up_p(file))
		mask |= POLLHUP;
	if (tty->ops->write && !tty_is_writelocked(tty) &&
			tty_chars_in_buffer(tty) < WAKEUP_CHARS &&
			tty_write_room(tty) > 0)
		mask |= POLLOUT | POLLWRNORM;
	return mask;
}


static int unipi_tty_ioctl(struct tty_struct *tty, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	struct unipi_tty_data *ldata = tty->disc_data;
	int retval;

	unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Ioctl start. cmd=%x", cmd);
	switch (cmd) {
	case TIOCOUTQ:
		return put_user(tty_chars_in_buffer(tty), (int __user *) arg);
	case TIOCINQ:
		down_write(&tty->termios_rwsem);
        retval = read_cnt(ldata);
		up_write(&tty->termios_rwsem);
		return put_user(retval, (unsigned int __user *) arg);
	default:
		return  n_tty_ioctl_helper(tty, file, cmd, arg);
	}
}

static long unipi_tty_compat_ioctl(struct tty_struct *tty, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	int retval;

	unipi_tty_trace(KERN_INFO "UNIPI_LDISC: Compat_ioctl start. cmd=%x", cmd);
	switch (cmd) {
	case 0x5481:
	case 0x5480:
		if (tty->ops->ioctl) {
			retval = tty->ops->ioctl(tty, cmd,  (unsigned long) compat_ptr(arg));
			if (retval != -ENOIOCTLCMD)
				return retval;
		}
		//return tty_mode_ioctl(tty, file, cmd, (unsigned long) compat_ptr(arg));
	default:
		return n_tty_compat_ioctl_helper(tty, file, cmd, arg);
	}
}


static struct tty_ldisc_ops unipi_tty_ops = {
	.magic           = TTY_LDISC_MAGIC,
  	.owner 			 = THIS_MODULE,
	.name            = "unipi_tty",
	.open            = unipi_tty_open,
	.close           = unipi_tty_close,
	.flush_buffer    = unipi_tty_flush_buffer,
	.read            = unipi_tty_read,
	.write           = unipi_tty_write,
	.ioctl           = unipi_tty_ioctl,
	.compat_ioctl    = unipi_tty_compat_ioctl,
	.set_termios     = unipi_tty_set_termios,
	.poll            = unipi_tty_poll,
	.receive_buf     = unipi_tty_receive_buf,
	.write_wakeup    = unipi_tty_write_wakeup,
	.receive_buf2	 = unipi_tty_receive_buf2,
};



int __init unipi_tty_init(void)
{
	int err;
	unipi_tty_trace(KERN_INFO "UNIPISPI: TTY Init\n");
	err = tty_register_ldisc(N_PROFIBUS_FDL, &unipi_tty_ops);
	if (err) {
		printk(KERN_INFO "UNIPISPI: UniPi line discipline registration failed. (%d)", err);
		return err;
	}
	return 0;
}
