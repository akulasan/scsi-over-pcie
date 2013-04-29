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

#ifndef for_each_set_bit
  #define for_each_set_bit for_each_bit
#endif

#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0) )
#define MRFN_TYPE	int

/*
 * NB: return value of non-zero would mean
 * that we were a stacking driver.
 * make_request must always succeed.
 */
#define MRFN_RET	0

bool kthread_freezable_should_stop(bool *was_frozen)
{
	might_sleep();

	if (unlikely(freezing(current)))
		refrigerator();

	return kthread_should_stop();
}

#else
/* Function return is eliminated */
#define MRFN_TYPE	void
#define MRFN_RET
#endif

/* these next three disappeared in 3.8-rc4 */
#ifndef __devinit
#define __devinit
#endif

#ifndef __devexit
#define __devexit
#endif

#ifndef __devexit_p
#define __devexit_p(x) x
#endif

#endif /* SOP_KERNEL_COMPAT_H */
