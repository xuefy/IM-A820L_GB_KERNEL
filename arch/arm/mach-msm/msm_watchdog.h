/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ARCH_ARM_MACH_MSM_MSM_WATCHDOG_H
#define __ARCH_ARM_MACH_MSM_MSM_WATCHDOG_H

//p14291_111226
#ifdef CONFIG_PANTECH_APPSBARK_FIQ
#define TZBSP_CPU_COUNT           2
typedef  unsigned int uint32;

//CPU context for TZBSP
typedef struct tzbsp_dump_cpu_ctx_s
{
  uint32 mon_lr;
  uint32 mon_spsr;
  uint32 usr_r0;
  uint32 usr_r1;
  uint32 usr_r2;
  uint32 usr_r3;
  uint32 usr_r4;
  uint32 usr_r5;
  uint32 usr_r6;
  uint32 usr_r7;
  uint32 usr_r8;
  uint32 usr_r9;
  uint32 usr_r10;
  uint32 usr_r11;
  uint32 usr_r12;
  uint32 usr_r13;
  uint32 usr_r14;
  uint32 irq_spsr; /* Monitor expects a pointer here for the context saving. */
  uint32 irq_r13;
  uint32 irq_r14;
  uint32 svc_spsr;
  uint32 svc_r13;
  uint32 svc_r14;
  uint32 abt_spsr;
  uint32 abt_r13;
  uint32 abt_r14;
  uint32 und_spsr;
  uint32 und_r13;
  uint32 und_r14;
  uint32 fiq_spsr;
  uint32 fiq_r8;
  uint32 fiq_r9;
  uint32 fiq_r10;
  uint32 fiq_r11;
  uint32 fiq_r12;
  uint32 fiq_r13;
  uint32 fiq_r14;
} tzbsp_dump_cpu_ctx_t;

typedef struct tzbsp_dump_buf_s
{
  uint32 sc_status[TZBSP_CPU_COUNT];
  tzbsp_dump_cpu_ctx_t sc_ns[TZBSP_CPU_COUNT];
  tzbsp_dump_cpu_ctx_t sec;
} tzbsp_dump_buf_t;
#endif

#ifdef CONFIG_MSM_WATCHDOG
void pet_watchdog(void);
#else
static inline void pet_watchdog(void) { }
#endif

#endif
