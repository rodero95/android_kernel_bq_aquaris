/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
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
 */




/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2005
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE. 
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.h
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _OV5647AMIPI_SENSOR_H
#define _OV5647AMIPI_SENSOR_H

#define OV5647AMIPI_DEBUG
#define OV5647AMIPI_DRIVER_TRACE
//#define OV5647AMIPI_TEST_PATTEM
#ifdef OV5647AMIPI_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

#define OV5647AMIPI_FACTORY_START_ADDR 0
#define OV5647AMIPI_ENGINEER_START_ADDR 10

#define MIPI_INTERFACE
 
typedef enum OV5647AMIPI_group_enum
{
  OV5647AMIPI_PRE_GAIN = 0,
  OV5647AMIPI_CMMCLK_CURRENT,
  OV5647AMIPI_FRAME_RATE_LIMITATION,
  OV5647AMIPI_REGISTER_EDITOR,
  OV5647AMIPI_GROUP_TOTAL_NUMS
} OV5647AMIPI_FACTORY_GROUP_ENUM;

typedef enum OV5647AMIPI_register_index
{
  OV5647AMIPI_SENSOR_BASEGAIN = OV5647AMIPI_FACTORY_START_ADDR,
  OV5647AMIPI_PRE_GAIN_R_INDEX,
  OV5647AMIPI_PRE_GAIN_Gr_INDEX,
  OV5647AMIPI_PRE_GAIN_Gb_INDEX,
  OV5647AMIPI_PRE_GAIN_B_INDEX,
  OV5647AMIPI_FACTORY_END_ADDR
} OV5647AMIPI_FACTORY_REGISTER_INDEX;

typedef enum OV5647AMIPI_engineer_index
{
  OV5647AMIPI_CMMCLK_CURRENT_INDEX = OV5647AMIPI_ENGINEER_START_ADDR,
  OV5647AMIPI_ENGINEER_END
} OV5647AMIPI_FACTORY_ENGINEER_INDEX;

typedef struct _sensor_data_struct
{
  SENSOR_REG_STRUCT reg[OV5647AMIPI_ENGINEER_END];
  SENSOR_REG_STRUCT cct[OV5647AMIPI_FACTORY_END_ADDR];
} sensor_data_struct;

/* SENSOR PREVIEW/CAPTURE VT CLOCK */
#define OV5647AMIPI_PREVIEW_CLK                     56000000
#define OV5647AMIPI_CAPTURE_CLK                     80000000

#if 0
#define OV5647AMIPI_COLOR_FORMAT                    SENSOR_OUTPUT_FORMAT_RAW_Gb //SENSOR_OUTPUT_FORMAT_RAW_R
#else
#define OV5647AMIPI_COLOR_FORMAT                    SENSOR_OUTPUT_FORMAT_RAW_Gr
#endif

#define OV5647AMIPI_MIN_ANALOG_GAIN				1	/* 1x */
#define OV5647AMIPI_MAX_ANALOG_GAIN				32	/* 32x */


/* FRAME RATE UNIT */
#define OV5647AMIPI_FPS(x)                          (10 * (x))

/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
//#define OV5647AMIPI_FULL_PERIOD_PIXEL_NUMS          2700 /* 9 fps */
#if 0
#define OV5647AMIPI_FULL_PERIOD_PIXEL_NUMS          3055 /* 8 fps */
#define OV5647AMIPI_FULL_PERIOD_LINE_NUMS           1968
#define OV5647AMIPI_PV_PERIOD_PIXEL_NUMS            1630 /* 30 fps */
#define OV5647AMIPI_PV_PERIOD_LINE_NUMS             984
#else
#define OV5647AMIPI_FULL_PERIOD_PIXEL_NUMS          2700 /* 15 fps */
#define OV5647AMIPI_FULL_PERIOD_LINE_NUMS           1974
#define OV5647AMIPI_PV_PERIOD_PIXEL_NUMS            1896 /* 30 fps */
#define OV5647AMIPI_PV_PERIOD_LINE_NUMS             984
#endif

#if 0
/* SENSOR START/END POSITION */
#define OV5647AMIPI_FULL_X_START                    3
#define OV5647AMIPI_FULL_Y_START                    11
#define OV5647AMIPI_IMAGE_SENSOR_FULL_WIDTH         (2592 - 52) /* 2560 */
#define OV5647AMIPI_IMAGE_SENSOR_FULL_HEIGHT        (1944 - 40) /* 1920 */
#define OV5647AMIPI_PV_X_START                      1
#define OV5647AMIPI_PV_Y_START                      1
#define OV5647AMIPI_IMAGE_SENSOR_PV_WIDTH           (1280 - 16) /* 1264 */
#define OV5647AMIPI_IMAGE_SENSOR_PV_HEIGHT          (960 - 12) /* 948 */
#else


#define OV5647AMIPI_FULL_X_START                    4   //(1+16+6)
#define OV5647AMIPI_FULL_Y_START                    4  //(1+12+4)
#define OV5647AMIPI_IMAGE_SENSOR_FULL_WIDTH         (2560 - 32) //(2592 - 16) /* 2560 */
#define OV5647AMIPI_IMAGE_SENSOR_FULL_HEIGHT        (1920 - 24) //(1944 - 12) /* 1920 */
#define OV5647AMIPI_PV_X_START                      2
#define OV5647AMIPI_PV_Y_START                      2
#define OV5647AMIPI_IMAGE_SENSOR_PV_WIDTH           (1280 - 16) /* 1264 */
#define OV5647AMIPI_IMAGE_SENSOR_PV_HEIGHT          (960 - 12) /* 948 */
#endif



/* SENSOR READ/WRITE ID */
#define OV5647AMIPI_WRITE_ID (0x6c)
#define OV5647AMIPI_READ_ID  (0x6d)

/* SENSOR ID */
//#define OV5647AMIPI_SENSOR_ID						(0x5647A)

/* SENSOR PRIVATE STRUCT */
typedef struct OV5647AMIPI_sensor_STRUCT
{
  MSDK_SENSOR_CONFIG_STRUCT cfg_data;
  sensor_data_struct eng; /* engineer mode */
  MSDK_SENSOR_ENG_INFO_STRUCT eng_info;
  kal_uint8 mirror;
  kal_bool pv_mode;
  kal_bool video_mode;
  kal_bool NightMode;
  kal_uint16 normal_fps; /* video normal mode max fps */
  kal_uint16 night_fps; /* video night mode max fps */
  kal_uint16 FixedFps;
  kal_uint16 shutter;
  kal_uint16 gain;
  kal_uint32 pclk;
  kal_uint16 frame_height;
  kal_uint16 frame_height_BackUp;
  kal_uint16 line_length;  
} OV5647AMIPI_sensor_struct;

//export functions
UINT32 OV5647AMIPIOpen(void);
UINT32 OV5647AMIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 OV5647AMIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 OV5647AMIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 OV5647AMIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 OV5647AMIPIClose(void);

#define Sleep(ms) mdelay(ms)

#endif 
