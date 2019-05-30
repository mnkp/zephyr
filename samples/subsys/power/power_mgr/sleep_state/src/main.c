/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <power.h>
#include <misc/printk.h>
#include <string.h>
#include <device.h>
#include <gpio.h>

#define WAIT_DELAY_S			(3)
#define BUSY_WAIT_DELAY_US		(WAIT_DELAY_S * USEC_PER_SEC)

static char *state_name_table[] = {
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_1
	STRINGIFY(SYS_POWER_STATE_SLEEP_1),
#endif
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_2
	STRINGIFY(SYS_POWER_STATE_SLEEP_2),
#endif
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_3
	STRINGIFY(SYS_POWER_STATE_SLEEP_3),
#endif
};

static s32_t delay_table[] = {
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_1
	CONFIG_SYS_PM_MIN_RESIDENCY_SLEEP_1 + 1,
#endif
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_2
	CONFIG_SYS_PM_MIN_RESIDENCY_SLEEP_2 + 1,
#endif
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_3
	CONFIG_SYS_PM_MIN_RESIDENCY_SLEEP_3 + 1,
#endif
};

void sys_pm_notify_power_state_entry(enum power_states state)
{
	printk("Entering Power State %s\n", state_name_table[state]);
	k_busy_wait(200);
}

void sys_pm_notify_power_state_exit(enum power_states state)
{
	k_busy_wait(200);
	printk("Exiting Power State %s\n", state_name_table[state]);
}

void main(void)
{
	printk("\n*** OS Power Management Demo on %s ***\n", CONFIG_SOC_SERIES);

	for (int i = 0; i < ARRAY_SIZE(delay_table); i++) {
		printk("\nApp doing busy wait for %d s...\n", WAIT_DELAY_S);
		k_busy_wait(BUSY_WAIT_DELAY_US);

		/* Create Idleness to make Idle thread run */
		s32_t delay = delay_table[i];

		printk("Going to sleep for %d ms\n", delay);
		k_sleep(delay);
	}

	printk("OS managed Power Management Test completed\n");
}
