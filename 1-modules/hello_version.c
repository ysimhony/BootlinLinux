// SPDX-License-Identifier: GPL-2.0
/* Linux kernel module example
 * Copyright Bootlin <https://bootlin.com>
 * License: GNU General Public License 2.0 or later
 *
 * Also available on
 * SOLUTION_URL
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/utsname.h>
#include <linux/timekeeping.h>

/* Module parameters */
static char *who = "World";
module_param(who, charp, 0644);
MODULE_PARM_DESC(who, "Person to greet");

/* Module internal data */
static time64_t init_time;

/* Module init and exit functions */
static int __init hello_init(void)
{
	pr_info("Hello %s, you're running Linux %s\n",
		who, init_uts_ns.name.release);

	/* Record module loading time */
	init_time = ktime_get_seconds();

	/* Module loading was successful */
	return 0;
}

static void __exit hello_exit(void)
{
	pr_info("Goodbye, %s. This module lived %llu seconds\n",
		who, ktime_get_seconds() - init_time);
}

/* Module registration */
module_init(hello_init);
module_exit(hello_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bootlin");
