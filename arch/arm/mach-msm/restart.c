/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/pmic8901.h>
#include <linux/console.h> //p14291_111214

#include <mach/msm_iomap.h>
#include <mach/restart.h>
#include <mach/scm-io.h>
#include <asm/mach-types.h>
#include <mach/irqs.h>

#ifdef CONFIG_PANTECH_ERR_CRASH_LOGGING
#include "sky_sys_reset.h"
#endif

#define TCSR_WDT_CFG 0x30

#define WDT0_RST       (MSM_TMR0_BASE + 0x38)
#define WDT0_EN        (MSM_TMR0_BASE + 0x40)
#define WDT0_BARK_TIME (MSM_TMR0_BASE + 0x4C)
#define WDT0_BITE_TIME (MSM_TMR0_BASE + 0x5C)

#define PSHOLD_CTL_SU (MSM_TLMM_BASE + 0x820)

#define IMEM_BASE           0x2A05F000

#ifdef CONFIG_PANTECH_ERR_CRASH_LOGGING
#define RESTART_REASON_ADDR_IN_HERE 0x65C
#else
#define RESTART_REASON_ADDR 0x65C
#endif

#define DLOAD_MODE_ADDR     0x0

static int restart_mode;
void *restart_reason;

int pmic_reset_irq;
#ifdef CONFIG_PANTECH
#define NORMAL_RESET_MAGIC_NUM 0xbaabcddc
#endif

#ifdef CONFIG_PANTECH_ERR_CRASH_LOGGING
extern void pantech_errlog_display_put_log(const char *log, int size);
#endif

#ifdef CONFIG_MSM_DLOAD_MODE
static int in_panic;
static void *dload_mode_addr;

/* Download mode master kill-switch */
static int dload_set(const char *val, struct kernel_param *kp);
static int download_mode = 1;
module_param_call(download_mode, dload_set, param_get_int,
			&download_mode, 0644);

static int panic_prep_restart(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	in_panic = 1;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= panic_prep_restart,
};

static void set_dload_mode(int on)
{
	if (dload_mode_addr) {
		writel(on ? 0xE47B337D : 0, dload_mode_addr);
		writel(on ? 0xCE14091A : 0,
		       dload_mode_addr + sizeof(unsigned int));
		mb();
	}
}

static int dload_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = download_mode;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	/* If download_mode is not zero or one, ignore. */
	if (download_mode >> 1) {
		download_mode = old_val;
		return -EINVAL;
	}

#ifndef CONFIG_PANTECH_ERR_CRASH_LOGGING
	set_dload_mode(download_mode);
#endif
	return 0;
}
#else
#define set_dload_mode(x) do {} while (0)
#endif

void msm_set_restart_mode(int mode)
{
	restart_mode = mode;
}
EXPORT_SYMBOL(msm_set_restart_mode);

static void __msm_power_off(int lower_pshold)
{
#ifdef CONFIG_PANTECH_ERR_CRASH_LOGGING
    void *restart_reason;

    restart_reason = ioremap_nocache(RESTART_REASON_ADDR, SZ_4K);
    writel(0x00, restart_reason);
    writel(0x00, restart_reason+4);
    iounmap(restart_reason);
#endif
	
	printk(KERN_CRIT "Powering off the SoC\n");
#ifdef CONFIG_MSM_DLOAD_MODE
	set_dload_mode(0);
#endif
	pm8058_reset_pwr_off(0);
	pm8901_reset_pwr_off(0);
	if (lower_pshold)
		writel(0, PSHOLD_CTL_SU);

	mdelay(10000);
	printk(KERN_ERR "Powering off has failed\n");
	return;
}

#ifdef CONFIG_PANTECH_PWR_ONOFF_REASON_CNT
int sky_reset_reason=SYS_RESET_REASON_UNKNOWN;
#endif

static void msm_power_off(void)
{
	/* MSM initiated power off, lower ps_hold */
	__msm_power_off(1);
}

static void cpu_power_off(void *data)
{
	pr_err("PMIC Initiated shutdown %s cpu=%d\n", __func__,
						smp_processor_id());
	if (smp_processor_id() == 0)
		/*
		 * PMIC initiated power off, do not lower ps_hold, pmic will
		 * shut msm down
		 */
		__msm_power_off(0);

	preempt_disable();
	while (1)
		;
}

static irqreturn_t resout_irq_handler(int irq, void *dev_id)
{
	pr_warn("%s PMIC Initiated shutdown\n", __func__);
	oops_in_progress = 1;
	smp_call_function_many(cpu_online_mask, cpu_power_off, NULL, 0);
	if (smp_processor_id() == 0)
		cpu_power_off(NULL);
	preempt_disable();
	while (1)
		;
	return IRQ_HANDLED;
}

void arch_reset(char mode, const char *cmd)
{
#ifdef CONFIG_PANTECH_ERR_CRASH_LOGGING
	char dispbuf[512];
#endif /* CONFIG_PANTECH_ERR_CRASH_LOGGING */

#ifdef CONFIG_MSM_DLOAD_MODE

	/* This looks like a normal reboot at this point. */
	set_dload_mode(0);

#ifndef CONFIG_PANTECH_ERR_CRASH_LOGGING
	/* Write download mode flags if we're panic'ing */
        if (in_panic || restart_mode == RESTART_DLOAD)
                set_dload_mode(1);
#endif

#ifndef CONFIG_PANTECH /* NOT Used in PANTECH */
	set_dload_mode(in_panic);

	/* Write download mode flags if restart_mode says so */
	if (restart_mode == RESTART_DLOAD)
		set_dload_mode(1);

	/* Kill download mode if master-kill switch is set */
	if (!download_mode)
		set_dload_mode(0);
#endif /* CONFIG_PANTECH // Not Used in PANTECH */

#ifndef CONFIG_PANTECH /* NOT Used in PANTECH */
    if (!in_panic && restart_mode == RESTART_NORMAL)
#endif /* CONFIG_PANTECH // NOT Used in PANTECH */
        set_dload_mode(0);
#endif /* CONFIG_MSM_DLOAD_MODE */

	printk(KERN_NOTICE "Going down for restart now\n");

#ifdef CONFIG_PANTECH_WDOG_WORKAROUND
       writel(NORMAL_RESET_MAGIC_NUM, restart_reason+4);
#endif /* CONFIG_PANTECH_WDOG_WORKAROUND */

	pm8058_reset_pwr_off(1);

	if (cmd != NULL) {
		if (!strncmp(cmd, "bootloader", 10)) {
			writel(0x77665500, restart_reason);
		} else if (!strncmp(cmd, "recovery", 8)) {
			writel(0x77665502, restart_reason);
#ifdef CONFIG_PANTECH_ERR_CRASH_LOGGING
		} else if (!strncmp(cmd, "androidpanic", 12)) {
			printk(KERN_ERR "allydrop android panic!!!!in_panic:%x\n",in_panic);
#ifdef CONFIG_PANTECH_PWR_ONOFF_REASON_CNT
			sky_reset_reason=SYS_RESET_REASON_ANDROID;
            writel(sky_reset_reason, restart_reason);
            writel(NORMAL_RESET_MAGIC_NUM, restart_reason+4);
#endif /* CONFIG_PANTECH_PWR_ONOFF_REASON_CNT */
			strcpy(dispbuf,"\n\n     [ANDROID FRAMEWORK ERROR]\n\n");
			strcat(dispbuf,"\n\n     Rebooting cause of Crash\n\n");
			strcat(dispbuf,"\n\n     Press Power key for reboot\n\n");
			strcat(dispbuf,"\n\n     Wait a minute for saving logs until rebooting \n\n");  
			pantech_errlog_display_put_log(dispbuf, strlen(dispbuf));
			panic("android framework error\n"); 
#endif
		} else if (!strncmp(cmd, "oem-", 4)) {
			unsigned long code;
			strict_strtoul(cmd + 4, 16, &code);
			code = code & 0xff;
			writel(0x6f656d00 | code, restart_reason);
		} else {
#ifdef CONFIG_PANTECH_PWR_ONOFF_REASON_CNT
 			if(in_panic){
				writel(sky_reset_reason, restart_reason);
			}
			else
#endif /* CONFIG_PANTECH_PWR_ONOFF_REASON_CNT */
			{
				printk(KERN_ERR "Powering off Default\n");
				writel(0x77665501, restart_reason);
			}
		}
	}
#ifdef CONFIG_PANTECH_WDOG_WORKAROUND
	else
	{
#ifdef CONFIG_PANTECH_PWR_ONOFF_REASON_CNT
                if(in_panic){
                        writel(sky_reset_reason, restart_reason);
                }
                else
#endif /* CONFIG_PANTECH_PWR_ONOFF_REASON_CNT */
                {
                        writel(0x77665501, restart_reason);
                }

	}
        writel(NORMAL_RESET_MAGIC_NUM, restart_reason+4);
#endif /* CONFIG_PANTECH_WDOG_WORKAROUND */
	writel(0, WDT0_EN);
	if (!(machine_is_msm8x60_charm_surf() ||
	      machine_is_msm8x60_charm_ffa()
#ifdef CONFIG_MACH_MSM8X60_EF65L
	      || machine_is_msm8x60_ef65l()
#endif
	      )) {
		dsb();
		writel(0, PSHOLD_CTL_SU); /* Actually reset the chip */
		mdelay(5000);
		pr_notice("PS_HOLD didn't work, falling back to watchdog\n");
	}

	__raw_writel(1, WDT0_RST);
	__raw_writel(5*0x31F3, WDT0_BARK_TIME);
	__raw_writel(0x31F3, WDT0_BITE_TIME);
	__raw_writel(3, WDT0_EN);
	secure_writel(3, MSM_TCSR_BASE + TCSR_WDT_CFG);

	mdelay(10000);
	printk(KERN_ERR "Restarting has failed\n");
}

static int __init msm_restart_init(void)
{
	void *imem = ioremap_nocache(IMEM_BASE, SZ_4K);
	void *phy_log_buf;
	int rc;

#ifdef CONFIG_MSM_DLOAD_MODE
	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	dload_mode_addr = imem + DLOAD_MODE_ADDR;

#ifndef CONFIG_PANTECH_ERR_CRASH_LOGGING
	/* Reset detection is switched on below.*/
	set_dload_mode(1);
#endif
#endif

#ifdef CONFIG_PANTECH_ERR_CRASH_LOGGING
	restart_reason = imem + RESTART_REASON_ADDR_IN_HERE;
#else
	restart_reason = imem + RESTART_REASON_ADDR;
#endif
	pm_power_off = msm_power_off;

	//p14291_111214 --{
	phy_log_buf = (void*)virt_to_phys((void*)get_log_buf_addr());
	writel(phy_log_buf, restart_reason+0xc); //0x0,0x4:magic1,2 0x8:using at msm_fb
	// --}

	if (pmic_reset_irq != 0) {
		rc = request_any_context_irq(pmic_reset_irq,
					resout_irq_handler, IRQF_TRIGGER_HIGH,
					"restart_from_pmic", NULL);
		if (rc < 0)
			pr_err("pmic restart irq fail rc = %d\n", rc);
	} else {
		pr_warn("no pmic restart interrupt specified\n");
	}

	return 0;
}

late_initcall(msm_restart_init);
