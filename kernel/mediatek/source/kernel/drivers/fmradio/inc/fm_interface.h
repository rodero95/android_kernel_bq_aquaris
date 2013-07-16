/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#ifndef __FM_INTERFACE_H__
#define __FM_INTERFACE_H__

#include <linux/cdev.h>
#include <linux/workqueue.h>

#include "fm_typedef.h"
#include "fm_rds.h"
#include "fm_utils.h"

/******************************************************************************
 * STRUCTURE DEFINITIONS
 *****************************************************************************/
enum fm_op_state {
    FM_STA_STOP = 0,
    FM_STA_PLAY = 1,
    FM_STA_TUNE = 2,
    FM_STA_SEEK = 3,
    FM_STA_SCAN = 4,
    FM_STA_RAMPDOWN = 5,
    FM_STA_UNKOWN = 100,
    FM_STA_MAX
};

enum fm_pwr_state {
    FM_PWR_OFF = 0,
    FM_PWR_RX_ON = 1,
    FM_PWR_TX_ON = 2,
    FM_PWR_MAX
};

enum fm_audio_path {
    FM_AUD_ANALOG = 0,  //analog: line in
    FM_AUD_DIGITAL = 1, //digital: I2S
    FM_AUD_MAX
};

enum fm_antenna_type {
    FM_ANA_LONG = 0,    //long antenna
    FM_ANA_SHORT = 1,   //short antenna
    FM_ANA_MAX
};

struct fm_hw_info {
    fm_s32 chip_id; //chip ID, eg. 6620
    fm_s32 eco_ver; //chip ECO version, eg. E3
    fm_s32 rom_ver; //FM DSP rom code version, eg. V2
    fm_s32 patch_ver; //FM DSP patch version, eg. 1.11
    fm_s32 reserve;
};

typedef struct fm_i2s_info {
    fm_s32 status;
    fm_s32 mode;
    fm_s32 rate;
} fm_i2s_info_t;


struct fm_platform {
    struct cdev cdev;
    dev_t dev_t;
    struct class *cls;
    struct device *dev;
};

struct fm {
    //chip info
    fm_u16 chip_id;                 //chip id, such as 6616/6620/6626/6628
    fm_u16 device_id;               //chip version
    //basic run time info
    fm_s32 ref;                     //fm driver can be multi opened
    fm_bool chipon;                 //Chip power state
    enum fm_pwr_state pwr_sta;      //FM module power state
    enum fm_op_state op_sta;        //current operation state: tune, seek, scan ...
    enum fm_audio_path aud_path;    //I2S or Analog
    fm_s32 vol;                     //current audio volume from chip side
    fm_bool mute;                   // true: mute, false: playing
    fm_bool rds_on;                 // true: on, false: off
    enum fm_antenna_type ana_type;  //long/short antenna
    fm_bool via_bt;                 // true: fm over bt controller; false: fm over host
    fm_u16 min_freq;                // for UE, 875KHz
    fm_u16 max_freq;                // for UE, 1080KHz
    fm_u16 cur_freq;                //current frequency
    fm_u8 band;                     // UE/JAPAN/JPANWD
    //RDS data
    struct fm_flag_event *rds_event;//pointer to rds event
    struct rds_t *pstRDSData;       //rds spec data buffer
    //platform data
    struct fm_platform platform;    //platform related members

    struct fm_workthread *eint_wkthd;
    struct fm_workthread *timer_wkthd;
    struct fm_work *eint_wk;
    struct fm_work *rds_wk;
    struct fm_work *rst_wk; //work for subsystem reset
};

struct fm_callback {
    //call backs
    fm_u16(*cur_freq_get)(void);
    fm_s32(*cur_freq_set)(fm_u16 new_freq);
    fm_u16(*chan_para_get)(fm_u16 freq);    //get channel parameter, HL side/ FA / ATJ
};

struct fm_basic_interface {
    //mt66x6 lib interfaces
    fm_s32(*low_pwr_wa)(fm_s32 onoff);
    fm_s32(*pwron)(fm_s32 data);
    fm_s32(*pwroff)(fm_s32 data);
    fm_s32(*msdelay)(fm_u32 val);
    fm_s32(*usdelay)(fm_u32 val);
    fm_s32(*read)(fm_u8 addr, fm_u16 *val);
    fm_s32(*write)(fm_u8 addr, fm_u16 val);
    fm_s32(*setbits)(fm_u8 addr, fm_u16 bits, fm_u16 msk);
    fm_u16(*chipid_get)(void);
    fm_s32(*mute)(fm_bool mute);
    fm_s32(*rampdown)(void);
    fm_s32(*pwrupseq)(fm_u16 *chip_id, fm_u16 *device_id);
    fm_s32(*pwrdownseq)(void);
    fm_bool(*setfreq)(fm_u16 freq);
    fm_bool(*seek)(fm_u16 min_freq, fm_u16 max_freq, fm_u16 *freq, fm_u16 dir, fm_u16 space);
    fm_s32(*seekstop)(void);
    fm_bool(*scan)(fm_u16 min_freq, fm_u16 max_freq, fm_u16 *freq, fm_u16 *tbl, fm_u16 *tblsize, fm_u16 dir, fm_u16 space);
    fm_bool(*jammer_scan)(fm_u16 min_freq, fm_u16 max_freq, fm_u16 *freq, fm_u16 *tbl, fm_u16 *tblsize, fm_u16 dir, fm_u16 space);
    fm_s32(*cqi_get)(fm_s8 *buf, fm_s32 buf_len);
    fm_s32(*scanstop)(void);
    fm_s32(*rssiget)(fm_s32 *rssi);
    fm_s32(*volset)(fm_u8 vol);
    fm_s32(*volget)(fm_u8 *vol);
    fm_s32(*dumpreg)(void);
    fm_bool(*msget)(fm_u16 *ms);  //mono/stereo indicator get
    fm_s32(*msset)(fm_s32 ms);  //mono/stereo force set
    fm_bool(*pamdget)(fm_u16 *pamd);
    fm_bool(*em)(fm_u16 group, fm_u16 item, fm_u32 val);
    fm_s32(*anaswitch)(fm_s32 ana);
    fm_s32(*anaget)(void);
    fm_s32(*caparray_get)(fm_s32 *ca);
    fm_s32(*i2s_set)(fm_s32 onoff, fm_s32 mode, fm_s32 sample);
    fm_s32(*i2s_get)(fm_s32 *ponoff, fm_s32 *pmode, fm_s32 *psample);
    fm_s32(*hwinfo_get)(struct fm_hw_info *req);
};

struct fm_rds_interface {
    //rds lib interfaces
    fm_s32(*rds_blercheck)(rds_t *dst);
    fm_bool(*rds_onoff)(rds_t *dst, fm_bool onoff);
    fm_s32(*rds_parser)(rds_t *rds_dst, struct rds_rx_t *rds_raw, fm_s32 rds_size, fm_u16(*getfreq)(void));
    fm_u16(*rds_gbc_get)(void);  //good block counter
    fm_u16(*rds_bbc_get)(void);  //bad block counter
    fm_u8(*rds_bbr_get)(void);   //bad block ratio
    fm_s32(*rds_bc_reset)(void);  //reset block counter
    fm_u32(*rds_bci_get)(void);  //bler check interval
    fm_s32(*rds_log_get)(struct rds_rx_t *dst, fm_s32 *dst_len);
    fm_s32(*rds_gc_get)(struct rds_group_cnt_t *dst, rds_t *rdsp);
    fm_s32(*rds_gc_reset)(rds_t *rdsp);
};

struct fm_lowlevel_ops {
    struct fm_callback cb;
    struct fm_basic_interface bi;
    struct fm_rds_interface ri;
};

extern fm_s32 fm_low_ops_register(struct fm_lowlevel_ops *ops);

extern fm_s32 fm_low_ops_unregister(struct fm_lowlevel_ops *ops);

extern fm_s32 fm_rds_ops_register(struct fm_lowlevel_ops *ops);

extern fm_s32 fm_rds_ops_unregister(struct fm_lowlevel_ops *ops);


#endif //__FM_INTERFACE_H__

