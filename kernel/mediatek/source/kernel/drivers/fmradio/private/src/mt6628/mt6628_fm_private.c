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

#include <linux/module.h>
#include <linux/init.h>

#include "stp_exp.h"
#include "wmt_exp.h"
#include "fm_private.h"
#include "mt6628_fm_private.h"
#include "fm_priv_log.h"

static struct fm_pub pub;
static struct fm_pub_cb *pub_cb = &pub.pub_tbl;

static const fm_u16 mt6628_mcu_dese_list[] = {
    763, 780, 794, 832, 926, 960, 971, 992, 1040, 1041
};

static const fm_u16 mt6628_gps_dese_list[] = {
    785, 786
};

static const fm_s8 mt6628_chan_para_map[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1,   //760~769
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //770~779
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //780~789
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //790~799
    8, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //800~809
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //810~819
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0,   //820~829
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //830~839
    0, 0, 0, 0, 0, 1, 1, 1, 0, 0,   //840~849
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //850~859
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //860~869
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //870~879
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //880~889
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //890~899
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //900~909
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //910~919
    0, 1, 1, 1, 0, 0, 0, 0, 0, 0,   //920~929
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //930~939
    0, 0, 0, 0, 0, 1, 1, 1, 1, 0,   //940~949
    1, 1, 1, 0, 0, 2, 0, 0, 0, 1,   //950~959
    8, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //960~969
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //970~979
    0, 0, 0, 0, 2, 0, 0, 0, 0, 0,   //980~989
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1,   //990~999
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0,   //1000~1009
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //1010~1019
    0, 0, 0, 0, 0, 0, 2, 0, 0, 0,   //1020~1029
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //1030~1039
    8, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //1040~1049
    1, 1, 1, 0, 0, 0, 0, 0, 0, 0,   //1050~1059
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   //1060~1069
    0, 0, 0, 0, 0, 1, 1, 1, 0, 0,   //1070~1079
    0   //1080
};


static const fm_u16 mt6628_scan_dese_list[] = {
    768, 821, 845, 921, 922, 960, 984, 1040, 1075, 1076
};

static fm_s32 mt6628_is_dese_chan(fm_u16 freq);
static fm_s32 mt6628_hl_dese(fm_u16 freq, void *arg);
static fm_s32 mt6628_fa_dese(fm_u16 freq, void *arg);
static fm_s32 mt6628_mcu_dese(fm_u16 freq, void *arg);
static fm_s32 mt6628_gps_dese(fm_u16 freq, void *arg);
static fm_u16 mt6628_chan_para_get(fm_u16 freq);



// return value: 0, not a de-sense channel; 1, this is a de-sense channel; else error no
static fm_s32 mt6628_is_dese_chan(fm_u16 freq)
{
    fm_s32 size;

    size = sizeof(mt6628_scan_dese_list) / sizeof(mt6628_scan_dese_list[0]);

    while (size) {
        if (mt6628_scan_dese_list[size -1] == freq)
            return 1;

        size--;
    }

    return 0;
}



// return value: 0, low side; 1, high side; else error no
static fm_s32 mt6628_hl_dese(fm_u16 freq, void *arg)
{
    return 0;
}


// return value: 0, fa off; 1, fa on; else error no
static fm_s32 mt6628_fa_dese(fm_u16 freq, void *arg)
{
    return 0;
}


// return value: 0, mcu dese disable; 1, enable; else error no
static fm_s32 mt6628_mcu_dese(fm_u16 freq, void *arg)
{
    fm_mcu_desense_t state = FM_MCU_DESE_DISABLE;
    fm_s32 len = 0;
    fm_s32 indx = 0;

    FM_LOG_DBG(D_MAIN, "%s, [freq=%d]\n", __func__, (int)freq);

    len = sizeof(mt6628_mcu_dese_list) / sizeof(mt6628_mcu_dese_list[0]);
    indx = 0;

    while ((indx < len) && (state != FM_MCU_DESE_ENABLE)) {
        if (mt6628_mcu_dese_list[indx] == freq) {
            state = FM_MCU_DESE_ENABLE;
        }

        indx++;
    }

    // request 6628 MCU change clk
    if (state == FM_MCU_DESE_DISABLE) {
        if (!mtk_wcn_wmt_dsns_ctrl(WMTDSNS_FM_DISABLE)) {
            return -1;
        }
        return 0;
    } else {
        if (!mtk_wcn_wmt_dsns_ctrl(WMTDSNS_FM_ENABLE)) {
            return -1;
        }
        return 1;
    }
}



// return value: 0,mcu dese disable; 1, enable; else error no
static fm_s32 mt6628_gps_dese(fm_u16 freq, void *arg)
{
    fm_gps_desense_t state = FM_GPS_DESE_DISABLE;
    fm_s32 len = 0;
    fm_s32 indx = 0;

    FM_LOG_DBG(D_MAIN, "%s, [freq=%d]\n", __func__, (int)freq);

    len = sizeof(mt6628_gps_dese_list) / sizeof(mt6628_gps_dese_list[0]);
    indx = 0;

    while ((indx < len) && (state != FM_GPS_DESE_ENABLE)) {
        if (mt6628_gps_dese_list[indx] == freq) {
            state = FM_GPS_DESE_ENABLE;
        }

        indx++;
    }

    // request 6628 GPS change clk
    if (state == FM_GPS_DESE_DISABLE) {
        if  (!mtk_wcn_wmt_dsns_ctrl(WMTDSNS_FM_GPS_DISABLE))  {
            return -1;
        }
        return 0;
    } else {
        if (!mtk_wcn_wmt_dsns_ctrl(WMTDSNS_FM_GPS_ENABLE)) {
            return -1;
        }
        return 1;
    }
}


//get channel parameter, HL side/ FA / ATJ
static fm_u16 mt6628_chan_para_get(fm_u16 freq)
{
    fm_s32 pos = freq - 760;
    fm_s32 size = sizeof(mt6628_chan_para_map) / sizeof(mt6628_chan_para_map[0]);

    pos = (pos < 0) ? 0 : pos;
    pos = (pos > (size - 1)) ? (size - 1) : pos;

    return mt6628_chan_para_map[pos];
}


static fm_s32 init(struct fm_pub *ppub)
{
    fm_s32 ret = 0;
    struct fm_priv priv;

    if (!ppub) return -1;

    ppub->pub_tbl.read = NULL;
    ppub->pub_tbl.write = NULL;
    ppub->pub_tbl.setbits = NULL;
    ppub->pub_tbl.rampdown = NULL;
    ppub->pub_tbl.msdelay = NULL;
    ppub->pub_tbl.usdelay = NULL;
    ppub->pub_tbl.log = NULL;
    ppub->state = UNINITED;
    ppub->data = NULL;

    priv.priv_tbl.hl_dese = mt6628_hl_dese;
    priv.priv_tbl.fa_dese = mt6628_fa_dese;
    priv.priv_tbl.mcu_dese = mt6628_mcu_dese;
    priv.priv_tbl.gps_dese = mt6628_gps_dese;
    priv.priv_tbl.chan_para_get = mt6628_chan_para_get;
    priv.priv_tbl.is_dese_chan = mt6628_is_dese_chan;

    ret = fm_priv_register(&priv, ppub);

    if (ret) {
        FM_LOG_ERR(D_INIT, "FM private module init failed\n");
    }

    FM_LOG_NTC(D_INIT, "FM private module init ok\n");

    return ret;
}


static int uninit(struct fm_pub *ppub)
{
    int ret = 0;

    ret = fm_priv_unregister(NULL, ppub);

    if (ret) {
        FM_LOG_ERR(D_INIT, "FM private module deinit failed\n");
    }

    FM_LOG_NTC(D_INIT, "FM private module deinit ok\n");


    return ret;
}

static fm_s32 __init mtk_fm_probe(void)
{
    fm_s32 ret = 0;

    FM_LOG_NTC(D_INIT, "%s, FM probe ...\n", __func__);
    ret = init(&pub);

    if (ret) {
        FM_LOG_ALT(D_INIT, "%s, FM probe failed\n", __func__);
    }

    FM_LOG_NTC(D_INIT, "%s, FM probe ok\n", __func__);

    return ret;
}

static void __exit mtk_fm_remove(void)
{
    fm_s32 ret = 0;
    ret = uninit(&pub);

    if (ret) {
        FM_LOG_ALT(D_INIT, "%s, FM remove failed\n", __func__);
    }

    FM_LOG_NTC(D_INIT, "%s, FM remove ok\n", __func__);

    return;
}

module_init(mtk_fm_probe);
module_exit(mtk_fm_remove);
MODULE_LICENSE("Proprietary. Send bug reports to hongcheng.xia@MediaTek.com");
MODULE_DESCRIPTION("MediaTek FM Driver Private Part, Need be loaded after FM driver");
MODULE_AUTHOR("Hongcheng <hongcheng.xia@MediaTek.com>");

