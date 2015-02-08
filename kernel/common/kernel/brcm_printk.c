/*
 *  linux/kernel/printk.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 * Modified to make sys_brcm_syslog() more flexible: added commands to
 * return the last 4k of kernel messages, regardless of whether
 * they've been read or not.  Added option to suppress kernel printk's
 * to the console.  Added hook for sending the console messages
 * elsewhere, in preparation for a serial line console (someday).
 * Ted Ts'o, 2/11/93.
 * Modified for sysctl support, 1/8/97, Chris Horn.
 * Fixed SMP synchronization, 08/08/99, Manfred Spraul
 *     manfred@colorfullife.com
 * Rewrote bits to get rid of console_lock
 *	01Mar01 Andrew Morton <andrewm@uow.edu.au>
 */

 /*******************************************************************************************
Copyright 2010 Broadcom Corporation.  All rights reserved.

Unless you and Broadcom execute a separate written software license agreement governing use
of this software, this software is licensed to you under the terms of the GNU General Public
License version 2, available at http://www.gnu.org/copyleft/gpl.html (the "GPL").

Notwithstanding the above, under no circumstances may you combine this software in any way
with any other Broadcom software provided under a license other than the GPL, without
Broadcom's express prior written consent.
*******************************************************************************************/

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/brcm_console.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/nmi.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>			/* For in_interrupt() */
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/security.h>
#include <linux/bootmem.h>
#include <linux/syscalls.h>

#include <asm/uaccess.h>

/*
 * Architectures can override it:
 */
void asmlinkage __attribute__((weak)) early_brcm_printk(const char *fmt, ...)
{
}

#define __LOG_BUF_LEN	(1 << (CONFIG_LOG_BUF_SHIFT + 2)) /* 64 Kbytes */

#ifdef        CONFIG_DEBUG_LL
extern void printascii(char *);
#endif

/* brcm_printk's without a loglevel use this.. */
#define DEFAULT_BRCM_MESSAGE_LOGLEVEL 4 /* KERN_WARNING */

/* We show everything that is MORE important than this.. */
#define MINIMUM_BRCM_CONSOLE_LOGLEVEL 1 /* Minimum loglevel we let people use */
#define DEFAULT_BRCM_CONSOLE_LOGLEVEL 7 /* anything MORE serious than KERN_DEBUG */

DECLARE_WAIT_QUEUE_HEAD(brcm_log_wait);

int brcm_console_brcm_printk[4] = {
	DEFAULT_BRCM_CONSOLE_LOGLEVEL,	/* brcm_console_loglevel */
	DEFAULT_BRCM_MESSAGE_LOGLEVEL,	/* default_message_loglevel */
	MINIMUM_BRCM_CONSOLE_LOGLEVEL,	/* minimum_brcm_console_loglevel */
	DEFAULT_BRCM_CONSOLE_LOGLEVEL,	/* default_brcm_console_loglevel */
};

/*
 * Low level drivers may need that to know if they can schedule in
 * their unblank() callback or not. So let's export it.
 */
int oops_in_progress_brcm;
EXPORT_SYMBOL(oops_in_progress_brcm);

/*
 * brcm_console_sem protects the brcm_console_drivers list, and also
 * provides serialisation for access to the entire brcm_console
 * driver system.
 */
static DECLARE_MUTEX(brcm_console_sem);
struct brcm_console *brcm_console_drivers;
EXPORT_SYMBOL_GPL(brcm_console_drivers);

/*
 * This is used for debugging the mess that is the VT code by
 * keeping track if we have the brcm_console semaphore held. It's
 * definitely not the perfect debug tool (we don't know if _WE_
 * hold it are racing, but it helps tracking those weird code
 * path in the brcm_console code where we end up in places I want
 * locked without the brcm_console sempahore held
 */
static int brcm_console_locked, brcm_console_suspended;

/*
 * logbuf_lock protects log_buf, log_start, log_end, con_start and logged_chars
 * It is also used in interesting ways to provide interlocking in
 * release_brcm_console_sem().
 */
static DEFINE_SPINLOCK(logbuf_lock);

#define LOG_BUF_MASK (log_buf_len-1)
#define LOG_BUF(idx) (log_buf[(idx) & LOG_BUF_MASK])

/*
 * The indices into log_buf are not constrained to log_buf_len - they
 * must be masked before subscripting
 */
static unsigned log_start;	/* Index into log_buf: next char to be read by brcm_syslog() */
static unsigned con_start;	/* Index into log_buf: next char to be sent to brcm_consoles */
static unsigned log_end;	/* Index into log_buf: most-recently-written-char + 1 */

/*
 *	Array of brcm_consoles built from command line options (brcm_console=)
 */
struct brcm_console_cmdline
{
	char	name[8];			/* Name of the driver	    */
	int	index;				/* Minor dev. to use	    */
	char	*options;			/* Options for the driver   */
#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
	char	*brl_options;			/* Options for braille driver */
#endif
};

#define MAX_CMDLINECONSOLES 8

static struct brcm_console_cmdline brcm_console_cmdline[MAX_CMDLINECONSOLES];
static int selected_brcm_console = -1;
static int preferred_brcm_console = -1;
int brcm_console_set_on_cmdline;
EXPORT_SYMBOL(brcm_console_set_on_cmdline);

/* Flag: brcm_console code may call schedule() */
static int brcm_console_may_schedule;

#ifdef CONFIG_PRINTK

static char __log_buf[__LOG_BUF_LEN];
static char *log_buf = __log_buf;
static int log_buf_len = __LOG_BUF_LEN;
static unsigned logged_chars; /* Number of chars produced since last read+clear operation */

static int __init brcm_log_buf_len_setup(char *str)
{
	unsigned size = memparse(str, &str);
	unsigned long flags;

	if (size)
		size = roundup_pow_of_two(size);
	if (size > log_buf_len) {
		unsigned start, dest_idx, offset;
		char *new_log_buf;

		new_log_buf = alloc_bootmem(size);
		if (!new_log_buf) {
			printk(KERN_WARNING "log_buf_len: allocation failed\n");
			goto out;
		}

		spin_lock_irqsave(&logbuf_lock, flags);
		log_buf_len = size;
		log_buf = new_log_buf;

		offset = start = min(con_start, log_start);
		dest_idx = 0;
		while (start != log_end) {
			log_buf[dest_idx] = __log_buf[start & (__LOG_BUF_LEN - 1)];
			start++;
			dest_idx++;
		}
		log_start -= offset;
		con_start -= offset;
		log_end -= offset;
		spin_unlock_irqrestore(&logbuf_lock, flags);

		printk(KERN_NOTICE "log_buf_len: %d\n", log_buf_len);
	}
out:
	return 1;
}

__setup("log_buf_len=", brcm_log_buf_len_setup);

#ifdef CONFIG_BOOT_PRINTK_DELAY

static unsigned int boot_delay; /* msecs delay after each brcm_printk during bootup */
static unsigned long long brcm_printk_delay_msec; /* per msec, based on boot_delay */

static int __init brcm_boot_delay_setup(char *str)
{
	unsigned long lpj;
	unsigned long long loops_per_msec;

	lpj = preset_lpj ? preset_lpj : 1000000;	/* some guess */
	loops_per_msec = (unsigned long long)lpj / 1000 * HZ;

	get_option(&str, &boot_delay);
	if (boot_delay > 10 * 1000)
		boot_delay = 0;

	brcm_printk_delay_msec = loops_per_msec;
	printk(KERN_DEBUG "boot_delay: %u, preset_lpj: %ld, lpj: %lu, "
		"HZ: %d, printk_delay_msec: %llu\n",
		boot_delay, preset_lpj, lpj, HZ, brcm_printk_delay_msec);
	return 1;
}
__setup("boot_delay=", brcm_boot_delay_setup);

static void boot_delay_msec(void)
{
	unsigned long long k;
	unsigned long timeout;

	if (boot_delay == 0 || system_state != SYSTEM_BOOTING)
		return;

	k = (unsigned long long)brcm_printk_delay_msec * boot_delay;

	timeout = jiffies + msecs_to_jiffies(boot_delay);
	while (k) {
		k--;
		cpu_relax();
		/*
		 * use (volatile) jiffies to prevent
		 * compiler reduction; loop termination via jiffies
		 * is secondary and may or may not happen.
		 */
		if (time_after(jiffies, timeout))
			break;
		touch_nmi_watchdog();
	}
}
#else
static inline void boot_delay_msec(void)
{
}
#endif

/*
 * Return the number of unread characters in the log buffer.
 */
static int log_buf_get_len(void)
{
	return logged_chars;
}

/*
 * Copy a range of characters from the log buffer.
 */
int brcm_log_buf_copy(char *dest, int idx, int len)
{
	int ret, max;
	bool took_lock = false;

	if (!oops_in_progress_brcm) {
		spin_lock_irq(&logbuf_lock);
		took_lock = true;
	}

	max = log_buf_get_len();
	if (idx < 0 || idx >= max) {
		ret = -1;
	} else {
		if (len > max - idx)
			len = max - idx;
		ret = len;
		idx += (log_end - max);
		while (len-- > 0)
			dest[len] = LOG_BUF(idx + len);
	}

	if (took_lock)
		spin_unlock_irq(&logbuf_lock);

	return ret;
}

/*
 * Commands to do_brcm_syslog:
 *
 * 	0 -- Close the log.  Currently a NOP.
 * 	1 -- Open the log. Currently a NOP.
 * 	2 -- Read from the log.
 * 	3 -- Read all messages remaining in the ring buffer.
 * 	4 -- Read and clear all messages remaining in the ring buffer
 * 	5 -- Clear ring buffer.
 * 	6 -- Disable brcm_printk's to brcm_console
 * 	7 -- Enable brcm_printk's to brcm_console
 *	8 -- Set level of messages printed to brcm_console
 *	9 -- Return number of unread characters in the log buffer
 *     10 -- Return size of the log buffer
 */
int do_brcm_syslog(int type, char __user *buf, int len)
{
	unsigned i, j, limit, count;
	int do_clear = 0;
	char c;
	int error = 0;

	error = security_syslog(type);
	if (error)
		return error;

	switch (type) {
	case 0:		/* Close log */
		break;
	case 1:		/* Open log */
		break;
	case 2:		/* Read from log */
		error = -EINVAL;
		if (!buf || len < 0)
			goto out;
		error = 0;
		if (!len)
			goto out;
		if (!access_ok(VERIFY_WRITE, buf, len)) {
			error = -EFAULT;
			goto out;
		}
		error = wait_event_interruptible(brcm_log_wait,
							(log_start - log_end));
		if (error)
			goto out;
		i = 0;
		spin_lock_irq(&logbuf_lock);
		while (!error && (log_start != log_end) && i < len) {
			c = LOG_BUF(log_start);
			log_start++;
			spin_unlock_irq(&logbuf_lock);
			error = __put_user(c,buf);
			buf++;
			i++;
			cond_resched();
			spin_lock_irq(&logbuf_lock);
		}
		spin_unlock_irq(&logbuf_lock);
		if (!error)
			error = i;
		break;
	case 4:		/* Read/clear last kernel messages */
		do_clear = 1;
		/* FALL THRU */
	case 3:		/* Read last kernel messages */
		error = -EINVAL;
		if (!buf || len < 0)
			goto out;
		error = 0;
		if (!len)
			goto out;
		if (!access_ok(VERIFY_WRITE, buf, len)) {
			error = -EFAULT;
			goto out;
		}
		count = len;
		if (count > log_buf_len)
			count = log_buf_len;
		spin_lock_irq(&logbuf_lock);
		if (count > logged_chars)
			count = logged_chars;
		if (do_clear)
			logged_chars = 0;
		limit = log_end;
		/*
		 * __put_user() could sleep, and while we sleep
		 * printk() could overwrite the messages
		 * we try to copy to user space. Therefore
		 * the messages are copied in reverse. <manfreds>
		 */
		for (i = 0; i < count && !error; i++) {
			j = limit-1-i;
			if (j + log_buf_len < log_end)
				break;
			c = LOG_BUF(j);
			spin_unlock_irq(&logbuf_lock);
			error = __put_user(c,&buf[count-1-i]);
			cond_resched();
			spin_lock_irq(&logbuf_lock);
		}
		spin_unlock_irq(&logbuf_lock);
		if (error)
			break;
		error = i;
		if (i != count) {
			int offset = count-error;
			/* buffer overflow during copy, correct user buffer. */
			for (i = 0; i < error; i++) {
				if (__get_user(c,&buf[i+offset]) ||
				    __put_user(c,&buf[i])) {
					error = -EFAULT;
					break;
				}
				cond_resched();
			}
		}
		break;
	case 5:		/* Clear ring buffer */
		logged_chars = 0;
		break;
	case 6:		/* Disable logging to brcm_console */
		brcm_console_loglevel = minimum_brcm_console_loglevel;
		break;
	case 7:		/* Enable logging to brcm_console */
		brcm_console_loglevel = default_brcm_console_loglevel;
		break;
	case 8:		/* Set level of messages printed to brcm_console */
		error = -EINVAL;
		if (len < 1 || len > 8)
			goto out;
		if (len < minimum_brcm_console_loglevel)
			len = minimum_brcm_console_loglevel;
		brcm_console_loglevel = len;
		error = 0;
		break;
	case 9:		/* Number of chars in the log buffer */
		error = log_end - log_start;
		break;
	case 10:	/* Size of the log buffer */
		error = log_buf_len;
		break;
	default:
		error = -EINVAL;
		break;
	}
out:
	return error;
}

asmlinkage long sys_brcm_syslog(int type, char __user *buf, int len)
{
	return do_brcm_syslog(type, buf, len);
}

/*
 * Call the brcm_console drivers on a range of log_buf
 */
static void __call_brcm_console_drivers(unsigned start, unsigned end)
{
	struct brcm_console *con;

	for (con = brcm_console_drivers; con; con = con->next) {
		if ((con->flags & CON_ENABLED) && con->write &&
				(cpu_online(smp_processor_id()) ||
				(con->flags & CON_ANYTIME)))
			con->write(con, &LOG_BUF(start), end - start);
	}
}

static int __read_mostly ignore_loglevel;

static int __init brcm_ignore_loglevel_setup(char *str)
{
	ignore_loglevel = 1;
	printk(KERN_INFO "debug: ignoring loglevel setting.\n");

	return 0;
}

early_param("ignore_loglevel", brcm_ignore_loglevel_setup);

/*
 * Write out chars from start to end - 1 inclusive
 */
static void _call_brcm_console_drivers(unsigned start,
				unsigned end, int msg_log_level)
{
	if ((msg_log_level < brcm_console_loglevel || ignore_loglevel) &&
			brcm_console_drivers && start != end) {
		if ((start & LOG_BUF_MASK) > (end & LOG_BUF_MASK)) {
			/* wrapped write */
			__call_brcm_console_drivers(start & LOG_BUF_MASK,
						log_buf_len);
			__call_brcm_console_drivers(0, end & LOG_BUF_MASK);
		} else {
			__call_brcm_console_drivers(start, end);
		}
	}
}

/*
 * Call the brcm_console drivers, asking them to write out
 * log_buf[start] to log_buf[end - 1].
 * The brcm_console_sem must be held.
 */
static void call_brcm_console_drivers(unsigned start, unsigned end)
{
	unsigned cur_index, start_print;
	static int msg_level = -1;

	BUG_ON(((int)(start - end)) > 0);

	cur_index = start;
	start_print = start;
	while (cur_index != end) {
		if (msg_level < 0 && ((end - cur_index) > 2) &&
				LOG_BUF(cur_index + 0) == '<' &&
				LOG_BUF(cur_index + 1) >= '0' &&
				LOG_BUF(cur_index + 1) <= '7' &&
				LOG_BUF(cur_index + 2) == '>') {
			msg_level = LOG_BUF(cur_index + 1) - '0';
			cur_index += 3;
			start_print = cur_index;
		}
		while (cur_index != end) {
			char c = LOG_BUF(cur_index);

			cur_index++;
			if (c == '\n') {
				if (msg_level < 0) {
					/*
					 * printk() has already given us loglevel tags in
					 * the buffer.  This code is here in case the
					 * log buffer has wrapped right round and scribbled
					 * on those tags
					 */
					msg_level = default_brcm_message_loglevel;
				}
				_call_brcm_console_drivers(start_print, cur_index, msg_level);
				msg_level = -1;
				start_print = cur_index;
				break;
			}
		}
	}
	_call_brcm_console_drivers(start_print, end, msg_level);
}

static void emit_log_char(char c)
{
	LOG_BUF(log_end) = c;
	log_end++;
	if (log_end - log_start > log_buf_len)
		log_start = log_end - log_buf_len;
	if (log_end - con_start > log_buf_len)
		con_start = log_end - log_buf_len;
	if (logged_chars < log_buf_len)
		logged_chars++;
}

/*
 * Zap brcm_console related locks when oopsing. Only zap at most once
 * every 10 seconds, to leave time for slow brcm_consoles to print a
 * full oops.
 */
static void zap_locks(void)
{
	static unsigned long oops_timestamp;

	if (time_after_eq(jiffies, oops_timestamp) &&
			!time_after(jiffies, oops_timestamp + 30 * HZ))
		return;

	oops_timestamp = jiffies;

	/* If a crash is occurring, make sure we can't deadlock */
	spin_lock_init(&logbuf_lock);
	/* And make sure that we print immediately */
	init_MUTEX(&brcm_console_sem);
}

#if defined(CONFIG_PRINTK_TIME)
static int printk_time = 1;
#else
static int printk_time = 0;
#endif
module_param_named(time, printk_time, bool, S_IRUGO | S_IWUSR);

/* Check if we have any brcm_console registered that can be called early in boot. */
static int have_callable_brcm_console(void)
{
	struct brcm_console *con;

	for (con = brcm_console_drivers; con; con = con->next)
		if (con->flags & CON_ANYTIME)
			return 1;

	return 0;
}

/**
 * printk - print a kernel message
 * @fmt: format string
 *
 * This is printk().  It can be called from any context.  We want it to work.
 * Be aware of the fact that if oops_in_progress_brcm is not set, we might try to
 * wake klogd up which could deadlock on runqueue lock if printk() is called
 * from scheduler code.
 *
 * We try to grab the brcm_console_sem.  If we succeed, it's easy - we log the output and
 * call the brcm_console drivers.  If we fail to get the semaphore we place the output
 * into the log buffer and return.  The current holder of the brcm_console_sem will
 * notice the new output in release_brcm_console_sem() and will send it to the
 * brcm_consoles before releasing the semaphore.
 *
 * One effect of this deferred printing is that code which calls printk() and
 * then changes brcm_console_loglevel may break. This is because brcm_console_loglevel
 * is inspected when the actual printing occurs.
 *
 * See also:
 * printf(3)
 */

asmlinkage int brcm_printk(const char *fmt, ...)
{
	va_list args;
	int r;

	va_start(args, fmt);
	r = brcm_vprintk(fmt, args);
	va_end(args);

	return r;
}

/* cpu currently holding logbuf_lock */
static volatile unsigned int printk_cpu = UINT_MAX;

/*
 * Can we actually use the brcm_console at this time on this cpu?
 *
 * Console drivers may assume that per-cpu resources have
 * been allocated. So unless they're explicitly marked as
 * being able to cope (CON_ANYTIME) don't call them until
 * this CPU is officially up.
 */
static inline int can_use_brcm_console(unsigned int cpu)
{
	return cpu_online(cpu) || have_callable_brcm_console();
}

/*
 * Try to get brcm_console ownership to actually show the kernel
 * messages from a 'printk'. Return true (and with the
 * brcm_console_semaphore held, and 'brcm_console_locked' set) if it
 * is successful, false otherwise.
 *
 * This gets called with the 'logbuf_lock' spinlock held and
 * interrupts disabled. It should return with 'lockbuf_lock'
 * released but interrupts still disabled.
 */
static int acquire_brcm_console_semaphore_for_printk(unsigned int cpu)
{
	int retval = 0;

	if (!try_acquire_brcm_console_sem()) {
		retval = 1;

		/*
		 * If we can't use the brcm_console, we need to release
		 * the brcm_console semaphore by hand to avoid flushing
		 * the buffer. We need to hold the brcm_console semaphore
		 * in order to do this test safely.
		 */
		if (!can_use_brcm_console(cpu)) {
			brcm_console_locked = 0;
			up(&brcm_console_sem);
			retval = 0;
		}
	}
	printk_cpu = UINT_MAX;
	spin_unlock(&logbuf_lock);
	return retval;
}
static const char recursion_bug_msg [] =
		KERN_CRIT "BUG: recent printk recursion!\n";
static int recursion_bug;
	static int new_text_line = 1;
static char printk_buf[1024];

asmlinkage int brcm_vprintk(const char *fmt, va_list args)
{
	int printed_len = 0;
	int current_log_level = default_message_loglevel;
	unsigned long flags;
	int this_cpu;
	char *p;

	boot_delay_msec();

	preempt_disable();
	/* This stops the holder of brcm_console_sem just where we want him */
	raw_local_irq_save(flags);
	this_cpu = smp_processor_id();

	/*
	 * Ouch, printk recursed into itself!
	 */
	if (unlikely(printk_cpu == this_cpu)) {
		/*
		 * If a crash is occurring during printk() on this CPU,
		 * then try to get the crash message out but make sure
		 * we can't deadlock. Otherwise just return to avoid the
		 * recursion and return - but flag the recursion so that
		 * it can be printed at the next appropriate moment:
		 */
		if (!oops_in_progress_brcm) {
			recursion_bug = 1;
			goto out_restore_irqs;
		}
		zap_locks();
	}

	lockdep_off();
	spin_lock(&logbuf_lock);
	printk_cpu = this_cpu;

	if (recursion_bug) {
		recursion_bug = 0;
		strcpy(printk_buf, recursion_bug_msg);
		printed_len = sizeof(recursion_bug_msg);
	}
	/* Emit the output into the temporary buffer */
	printed_len += vscnprintf(printk_buf + printed_len,
				  sizeof(printk_buf) - printed_len, fmt, args);


#ifdef	CONFIG_DEBUG_LL
	printascii(printk_buf);
#endif

	/*
	 * Copy the output into log_buf.  If the caller didn't provide
	 * appropriate log level tags, we insert them here
	 */
	for (p = printk_buf; *p; p++) {
		if (new_text_line) {
			/* If a token, set current_log_level and skip over */
			if (p[0] == '<' && p[1] >= '0' && p[1] <= '7' &&
			    p[2] == '>') {
				current_log_level = p[1] - '0';
				p += 3;
				printed_len -= 3;
			}

			/* Always output the token */
			emit_log_char('<');
			emit_log_char(current_log_level + '0');
			emit_log_char('>');
			printed_len += 3;
			new_text_line = 0;

			if (printk_time) {
				/* Follow the token with the time */
				char tbuf[50], *tp;
				unsigned tlen;
				unsigned long long t;
				unsigned long nanosec_rem;

				t = cpu_clock(printk_cpu);
				nanosec_rem = do_div(t, 1000000000);
				tlen = sprintf(tbuf, "[%5lu.%06lu] ",
						(unsigned long) t,
						nanosec_rem / 1000);

				for (tp = tbuf; tp < tbuf + tlen; tp++)
					emit_log_char(*tp);
				printed_len += tlen;
			}

			if (!*p)
				break;
		}

		emit_log_char(*p);
		if (*p == '\n')
			new_text_line = 1;
	}

	/*
	 * Try to acquire and then immediately release the
	 * brcm_console semaphore. The release will do all the
	 * actual magic (print out buffers, wake up klogd,
	 * etc).
	 *
	 * The acquire_brcm_console_semaphore_for_printk() function
	 * will release 'logbuf_lock' regardless of whether it
	 * actually gets the semaphore or not.
	 */
	if (acquire_brcm_console_semaphore_for_printk(this_cpu))
		release_brcm_console_sem();

	lockdep_on();
out_restore_irqs:
	raw_local_irq_restore(flags);

	preempt_enable();
	return printed_len;
}

EXPORT_SYMBOL(brcm_printk);
EXPORT_SYMBOL(brcm_vprintk);
#else

asmlinkage long sys_brcm_syslog(int type, char __user *buf, int len)
{
	return -ENOSYS;
}

static void call_brcm_console_drivers(unsigned start, unsigned end)
{
}

#endif

static int __add_preferred_brcm_console(char *name, int idx, char *options,
				   char *brl_options)
{
	struct brcm_console_cmdline *c;
	int i;

	/*
	 *	See if this tty is not yet registered, and
	 *	if we have a slot free.
	 */
	for (i = 0; i < MAX_CMDLINECONSOLES && brcm_console_cmdline[i].name[0]; i++)
		if (strcmp(brcm_console_cmdline[i].name, name) == 0 &&
			  brcm_console_cmdline[i].index == idx) {
				if (!brl_options)
					selected_brcm_console = i;
				return 0;
		}
	if (i == MAX_CMDLINECONSOLES)
		return -E2BIG;
	if (!brl_options)
		selected_brcm_console = i;
	c = &brcm_console_cmdline[i];
	strlcpy(c->name, name, sizeof(c->name));
	c->options = options;
#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
	c->brl_options = brl_options;
#endif
	c->index = idx;
	return 0;
}
/*
 * Set up a list of brcm_consoles.  Called from init/main.c
 */
static int __init brcm_console_setup(char *str)
{
	char buf[sizeof(brcm_console_cmdline[0].name) + 4]; /* 4 for index */
	char *s, *options, *brl_options = NULL;
	int idx;

#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
	if (!memcmp(str, "brl,", 4)) {
		brl_options = "";
		str += 4;
	} else if (!memcmp(str, "brl=", 4)) {
		brl_options = str + 4;
		str = strchr(brl_options, ',');
		if (!str) {
			printk(KERN_ERR "need port name after brl=\n");
			return 1;
		}
		*(str++) = 0;
	}
#endif

	/*
	 * Decode str into name, index, options.
	 */
	if (str[0] >= '0' && str[0] <= '9') {
		strcpy(buf, "ttyS");
		strncpy(buf + 4, str, sizeof(buf) - 5);
	} else {
		strncpy(buf, str, sizeof(buf) - 1);
	}
	buf[sizeof(buf) - 1] = 0;
	if ((options = strchr(str, ',')) != NULL)
		*(options++) = 0;
#ifdef __sparc__
	if (!strcmp(str, "ttya"))
		strcpy(buf, "ttyS0");
	if (!strcmp(str, "ttyb"))
		strcpy(buf, "ttyS1");
#endif
	for (s = buf; *s; s++)
		if ((*s >= '0' && *s <= '9') || *s == ',')
			break;
	idx = simple_strtoul(s, NULL, 10);
	*s = 0;

	__add_preferred_brcm_console(buf, idx, options, brl_options);
	brcm_console_set_on_cmdline = 1;
	return 1;
}
__setup("brcm_console=", brcm_console_setup);

/**
 * add_preferred_brcm_console - add a device to the list of preferred brcm_consoles.
 * @name: device name
 * @idx: device index
 * @options: options for this brcm_console
 *
 * The last preferred brcm_console added will be used for kernel messages
 * and stdin/out/err for init.  Normally this is used by brcm_console_setup
 * above to handle user-supplied brcm_console arguments; however it can also
 * be used by arch-specific code either to override the user or more
 * commonly to provide a default brcm_console (ie from PROM variables) when
 * the user has not supplied one.
 */
int add_preferred_brcm_console(char *name, int idx, char *options)
{
	return __add_preferred_brcm_console(name, idx, options, NULL);
}

int update_brcm_console_cmdline(char *name, int idx, char *name_new, int idx_new, char *options)
{
	struct brcm_console_cmdline *c;
	int i;

	for (i = 0; i < MAX_CMDLINECONSOLES && brcm_console_cmdline[i].name[0]; i++)
		if (strcmp(brcm_console_cmdline[i].name, name) == 0 &&
			  brcm_console_cmdline[i].index == idx) {
				c = &brcm_console_cmdline[i];
				strlcpy(c->name, name_new, sizeof(c->name));
				c->name[sizeof(c->name) - 1] = 0;
				c->options = options;
				c->index = idx_new;
				return i;
		}
	/* not found */
	return -1;
}

int brcm_console_suspend_enabled = 1;
EXPORT_SYMBOL(brcm_console_suspend_enabled);

static int __init brcm_console_suspend_disable(char *str)
{
	brcm_console_suspend_enabled = 0;
	return 1;
}
__setup("no_brcm_console_suspend", brcm_console_suspend_disable);

/**
 * suspend_brcm_console - suspend the brcm_console subsystem
 *
 * This disables printk() while we go into suspend states
 */
void suspend_brcm_console(void)
{
	if (!brcm_console_suspend_enabled)
		return;
	printk("Suspending brcm_console(s) (use no_brcm_console_suspend to debug)\n");
	acquire_brcm_console_sem();
	brcm_console_suspended = 1;
	up(&brcm_console_sem);
}

void resume_brcm_console(void)
{
	if (!brcm_console_suspend_enabled)
		return;
	down(&brcm_console_sem);
	brcm_console_suspended = 0;
	release_brcm_console_sem();
}

/**
 * acquire_brcm_console_sem - lock the brcm_console system for exclusive use.
 *
 * Acquires a semaphore which guarantees that the caller has
 * exclusive access to the brcm_console system and the brcm_console_drivers list.
 *
 * Can sleep, returns nothing.
 */
void acquire_brcm_console_sem(void)
{
	BUG_ON(in_interrupt());
	down(&brcm_console_sem);
	if (brcm_console_suspended)
		return;
	brcm_console_locked = 1;
	brcm_console_may_schedule = 1;
}
EXPORT_SYMBOL(acquire_brcm_console_sem);

int try_acquire_brcm_console_sem(void)
{
	if (down_trylock(&brcm_console_sem))
		return -1;
	if (brcm_console_suspended) {
		up(&brcm_console_sem);
		return -1;
	}
	brcm_console_locked = 1;
	brcm_console_may_schedule = 0;
	return 0;
}
EXPORT_SYMBOL(try_acquire_brcm_console_sem);

int is_brcm_console_locked(void)
{
	return brcm_console_locked;
}

void wake_up_klogd_brcm(void)
{
	if (!oops_in_progress_brcm && waitqueue_active(&brcm_log_wait))
		wake_up_interruptible(&brcm_log_wait);
}

/**
 * release_brcm_console_sem - unlock the brcm_console system
 *
 * Releases the semaphore which the caller holds on the brcm_console system
 * and the brcm_console driver list.
 *
 * While the semaphore was held, brcm_console output may have been buffered
 * by printk().  If this is the case, release_brcm_console_sem() emits
 * the output prior to releasing the semaphore.
 *
 * If there is output waiting for klogd, we wake it up.
 *
 * release_brcm_console_sem() may be called from any context.
 */
void release_brcm_console_sem(void)
{
	unsigned long flags;
	unsigned _con_start, _log_end;
	unsigned wake_klogd = 0;

	if (brcm_console_suspended) {
		up(&brcm_console_sem);
		return;
	}

	brcm_console_may_schedule = 0;

	for ( ; ; ) {
		spin_lock_irqsave(&logbuf_lock, flags);
		wake_klogd |= log_start - log_end;
		if (con_start == log_end)
			break;			/* Nothing to print */
		_con_start = con_start;
		_log_end = log_end;
		con_start = log_end;		/* Flush */
		spin_unlock(&logbuf_lock);
		stop_critical_timings();	/* don't trace print latency */
		call_brcm_console_drivers(_con_start, _log_end);
		start_critical_timings();
		local_irq_restore(flags);
	}
	brcm_console_locked = 0;
	up(&brcm_console_sem);
	spin_unlock_irqrestore(&logbuf_lock, flags);
	if (wake_klogd)
		wake_up_klogd_brcm();
}
EXPORT_SYMBOL(release_brcm_console_sem);

/**
 * brcm_console_conditional_schedule - yield the CPU if required
 *
 * If the brcm_console code is currently allowed to sleep, and
 * if this CPU should yield the CPU to another task, do
 * so here.
 *
 * Must be called within acquire_brcm_console_sem().
 */
void __sched brcm_console_conditional_schedule(void)
{
	if (brcm_console_may_schedule)
		cond_resched();
}
EXPORT_SYMBOL(brcm_console_conditional_schedule);

void brcm_console_print(const char *s)
{
	printk(KERN_EMERG "%s", s);
}
EXPORT_SYMBOL(brcm_console_print);

void brcm_console_unblank(void)
{
	struct brcm_console *c;

	/*
	 * brcm_console_unblank can no longer be called in interrupt context unless
	 * oops_in_progress_brcm is set to 1..
	 */
	if (oops_in_progress_brcm) {
		if (down_trylock(&brcm_console_sem) != 0)
			return;
	} else
		acquire_brcm_console_sem();

	brcm_console_locked = 1;
	brcm_console_may_schedule = 0;
	for (c = brcm_console_drivers; c != NULL; c = c->next)
		if ((c->flags & CON_ENABLED) && c->unblank)
			c->unblank();
	release_brcm_console_sem();
}

/*
 * Return the brcm_console tty driver structure and its associated index
 */
struct tty_driver *brcm_console_device(int *index)
{
	struct brcm_console *c;
	struct tty_driver *driver = NULL;

	acquire_brcm_console_sem();
	for (c = brcm_console_drivers; c != NULL; c = c->next) {
		if (!c->device)
			continue;
		driver = c->device(c, index);
		if (driver)
			break;
	}
	release_brcm_console_sem();
	return driver;
}

/*
 * Prevent further output on the passed brcm_console device so that (for example)
 * serial drivers can disable brcm_console output before suspending a port, and can
 * re-enable output afterwards.
 */
void brcm_console_stop(struct brcm_console *brcm_console)
{
	acquire_brcm_console_sem();
	brcm_console->flags &= ~CON_ENABLED;
	release_brcm_console_sem();
}
EXPORT_SYMBOL(brcm_console_stop);

void brcm_console_start(struct brcm_console *brcm_console)
{
	acquire_brcm_console_sem();
	brcm_console->flags |= CON_ENABLED;
	release_brcm_console_sem();
}
EXPORT_SYMBOL(brcm_console_start);

/*
 * The brcm_console driver calls this routine during kernel initialization
 * to register the brcm_console printing procedure with printk() and to
 * print any messages that were printed by the kernel before the
 * brcm_console driver was initialized.
 */
void register_brcm_console(struct brcm_console *brcm_console)
{
	int i;
	unsigned long flags;
	struct brcm_console *boot_brcm_console = NULL;

	if (brcm_console_drivers) {
		if (brcm_console->flags & CON_BOOT)
			return;
		if (brcm_console_drivers->flags & CON_BOOT)
			boot_brcm_console = brcm_console_drivers;
	}

	if (preferred_brcm_console < 0 || boot_brcm_console || !brcm_console_drivers)
		preferred_brcm_console = selected_brcm_console;

	if (brcm_console->early_setup)
		brcm_console->early_setup();

	/*
	 *	See if we want to use this brcm_console driver. If we
	 *	didn't select a brcm_console we take the first one
	 *	that registers here.
	 */
	if (preferred_brcm_console < 0) {
		if (brcm_console->index < 0)
			brcm_console->index = 0;
		if (brcm_console->setup == NULL ||
		    brcm_console->setup(brcm_console, NULL) == 0) {
			brcm_console->flags |= CON_ENABLED;
			if (brcm_console->device) {
				brcm_console->flags |= CON_CONSDEV;
				preferred_brcm_console = 0;
			}
		}
	}

	/*
	 *	See if this brcm_console matches one we selected on
	 *	the command line.
	 */
	for (i = 0; i < MAX_CMDLINECONSOLES && brcm_console_cmdline[i].name[0];
			i++) {
		if (strcmp(brcm_console_cmdline[i].name, brcm_console->name) != 0)
			continue;
		if (brcm_console->index >= 0 &&
		    brcm_console->index != brcm_console_cmdline[i].index)
			continue;
		if (brcm_console->index < 0)
			brcm_console->index = brcm_console_cmdline[i].index;
#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
		if (brcm_console_cmdline[i].brl_options) {
			brcm_console->flags |= CON_BRL;
			braille_register_brcm_console(brcm_console,
					brcm_console_cmdline[i].index,
					brcm_console_cmdline[i].options,
					brcm_console_cmdline[i].brl_options);
			return;
		}
#endif
		if (brcm_console->setup &&
		    brcm_console->setup(brcm_console, brcm_console_cmdline[i].options) != 0)
			break;
		brcm_console->flags |= CON_ENABLED;
		brcm_console->index = brcm_console_cmdline[i].index;
		if (i == selected_brcm_console) {
			brcm_console->flags |= CON_CONSDEV;
			preferred_brcm_console = selected_brcm_console;
		}
		break;
	}

	if (!(brcm_console->flags & CON_ENABLED))
		return;

	if (boot_brcm_console && (brcm_console->flags & CON_CONSDEV)) {
		printk(KERN_INFO "brcm_console handover: boot [%s%d] -> real [%s%d]\n",
		       boot_brcm_console->name, boot_brcm_console->index,
		       brcm_console->name, brcm_console->index);
		unregister_brcm_console(boot_brcm_console);
		brcm_console->flags &= ~CON_PRINTBUFFER;
	} else {
		printk(KERN_INFO "brcm_console [%s%d] enabled\n",
		       brcm_console->name, brcm_console->index);
	}

	/*
	 *	Put this brcm_console in the list - keep the
	 *	preferred driver at the head of the list.
	 */
	acquire_brcm_console_sem();
	if ((brcm_console->flags & CON_CONSDEV) || brcm_console_drivers == NULL) {
		brcm_console->next = brcm_console_drivers;
		brcm_console_drivers = brcm_console;
		if (brcm_console->next)
			brcm_console->next->flags &= ~CON_CONSDEV;
	} else {
		brcm_console->next = brcm_console_drivers->next;
		brcm_console_drivers->next = brcm_console;
	}
	if (brcm_console->flags & CON_PRINTBUFFER) {
		/*
		 * release_brcm_console_sem() will print out the buffered messages
		 * for us.
		 */
		spin_lock_irqsave(&logbuf_lock, flags);
		con_start = log_start;
		spin_unlock_irqrestore(&logbuf_lock, flags);
	}
	release_brcm_console_sem();
}
EXPORT_SYMBOL(register_brcm_console);

int unregister_brcm_console(struct brcm_console *brcm_console)
{
        struct brcm_console *a, *b;
	int res = 1;

#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
	if (brcm_console->flags & CON_BRL)
		return braille_unregister_brcm_console(brcm_console);
#endif

	acquire_brcm_console_sem();
	if (brcm_console_drivers == brcm_console) {
		brcm_console_drivers=brcm_console->next;
		res = 0;
	} else if (brcm_console_drivers) {
		for (a=brcm_console_drivers->next, b=brcm_console_drivers ;
		     a; b=a, a=b->next) {
			if (a == brcm_console) {
				b->next = a->next;
				res = 0;
				break;
			}
		}
	}

	/*
	 * If this isn't the last brcm_console and it has CON_CONSDEV set, we
	 * need to set it on the next preferred brcm_console.
	 */
	if (brcm_console_drivers != NULL && brcm_console->flags & CON_CONSDEV)
		brcm_console_drivers->flags |= CON_CONSDEV;

	release_brcm_console_sem();
	return res;
}
EXPORT_SYMBOL(unregister_brcm_console);

static int __init disable_boot_brcm_consoles(void)
{
	if (brcm_console_drivers != NULL) {
		if (brcm_console_drivers->flags & CON_BOOT) {
			printk(KERN_INFO "turn off boot brcm_console %s%d\n",
				brcm_console_drivers->name, brcm_console_drivers->index);
			return unregister_brcm_console(brcm_console_drivers);
		}
	}
	return 0;
}
late_initcall(disable_boot_brcm_consoles);

/**
 * brcm_tty_write_message - write a message to a certain tty, not just the brcm_console.
 * @tty: the destination tty_struct
 * @msg: the message to write
 *
 * This is used for messages that need to be redirected to a specific tty.
 * We don't put it into the syslog queue right now maybe in the future if
 * really needed.
 */
void brcm_tty_write_message(struct tty_struct *tty, char *msg)
{
	if (tty && tty->ops->write)
		tty->ops->write(tty, msg, strlen(msg));
	return;
}

#if defined CONFIG_PRINTK

/*
 * printk rate limiting, lifted from the networking subsystem.
 *
 * This enforces a rate limit: not more than 10 kernel messages
 * every 5s to make a denial-of-service attack impossible.
 */
DEFINE_RATELIMIT_STATE(brcm_printk_ratelimit_state, 5 * HZ, 10);

int brcm_printk_ratelimit(void)
{
	return __ratelimit(&printk_ratelimit_state);
}
EXPORT_SYMBOL(brcm_printk_ratelimit);

/**
 * printk_timed_ratelimit - caller-controlled printk ratelimiting
 * @caller_jiffies: pointer to caller's state
 * @interval_msecs: minimum interval between prints
 *
 * printk_timed_ratelimit() returns true if more than @interval_msecs
 * milliseconds have elapsed since the last time printk_timed_ratelimit()
 * returned true.
 */
bool brcm_printk_timed_ratelimit(unsigned long *caller_jiffies,
			unsigned int interval_msecs)
{
	if (*caller_jiffies == 0 || time_after(jiffies, *caller_jiffies)) {
		*caller_jiffies = jiffies + msecs_to_jiffies(interval_msecs);
		return true;
	}
	return false;
}
EXPORT_SYMBOL(brcm_printk_timed_ratelimit);
#endif
