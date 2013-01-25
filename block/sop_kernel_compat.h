#ifndef SOP_KERNEL_COMPAT_H
#define SOP_KERNEL_COMPAT_H

#include <linux/version.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36) )
static int do_usleep_range(unsigned long min, unsigned long max)
{
	ktime_t kmin;
	unsigned long delta;

	kmin = ktime_set(0, min * NSEC_PER_USEC);
	delta = (max - min) * NSEC_PER_USEC;
	return schedule_hrtimeout_range(&kmin, delta, HRTIMER_MODE_REL);
}

/**
 * usleep_range - Drop in replacement for udelay where wakeup is flexible
 * @min: Minimum time in usecs to sleep
 * @max: Maximum time in usecs to sleep
 */
void usleep_range(unsigned long min, unsigned long max)
{
	__set_current_state(TASK_UNINTERRUPTIBLE);
	do_usleep_range(min, max);
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0) )
#define MRFN_TYPE	int

/*
 * NB: return value of non-zero would mean
 * that we were a stacking driver.
 * make_request must always succeed.
 */
#define MRFN_RET	0
#else
/* Function return is eliminated */
#define MRFN_TYPE	void
#define MRFN_RET
#endif

#endif /* SOP_KERNEL_COMPAT_H */
