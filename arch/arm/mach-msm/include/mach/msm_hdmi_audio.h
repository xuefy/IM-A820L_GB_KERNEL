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

#ifndef __MSM_HDMI_AUDIO_H
#define __MSM_HDMI_AUDIO_H

int hdmi_audio_enable(bool on , u32 fifo_water_mark);
#ifdef CONFIG_F_SKYDISP_HDMI_AUDIO_BUGFIX
int hdmi_audio_packet_enable(bool on);
#endif
#endif /* __MSM_HDMI_AUDIO_H*/
