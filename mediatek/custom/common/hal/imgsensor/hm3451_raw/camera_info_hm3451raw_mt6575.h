/********************************************************************************************
 *     LEGAL DISCLAIMER
 *
 *     (Header of MediaTek Software/Firmware Release or Documentation)
 *
 *     BY OPENING OR USING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *     THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") RECEIVED
 *     FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON AN "AS-IS" BASIS
 *     ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED,
 *     INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
 *     A PARTICULAR PURPOSE OR NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY
 *     WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 *     INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK
 *     ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *     NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S SPECIFICATION
 *     OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *     BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE LIABILITY WITH
 *     RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION,
TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.
 ************************************************************************************************/

#ifndef _CAMERA_INFO_HM3451RAW_MT6575_H
#define _CAMERA_INFO_HM3451RAW_MT6575_H

/*******************************************************************************
*   Configuration
********************************************************************************/
#define SENSOR_ID                           HM3451_SENSOR_ID
#define SENSOR_DRVNAME                      SENSOR_DRVNAME_HM3451_RAW
#define INCLUDE_FILENAME_ISP_REGS_PARAM     "camera_isp_regs_hm3451raw_mt6575.h"
#define INCLUDE_FILENAME_ISP_PCA_PARAM      "camera_isp_pca_hm3451raw_mt6575.h"

/*******************************************************************************
*   
********************************************************************************/

#if defined(ISP_SUPPORT)

#define HM3451RAW_CAMERA_AUTO_DSC CAM_AUTO_DSC
#define HM3451RAW_CAMERA_PORTRAIT CAM_PORTRAIT
#define HM3451RAW_CAMERA_LANDSCAPE CAM_LANDSCAPE
#define HM3451RAW_CAMERA_SPORT CAM_SPORT
#define HM3451RAW_CAMERA_FLOWER CAM_FLOWER
#define HM3451RAW_CAMERA_NIGHTSCENE CAM_NIGHTSCENE
#define HM3451RAW_CAMERA_DOCUMENT CAM_DOCUMENT
#define HM3451RAW_CAMERA_ISO_ANTI_HAND_SHAKE CAM_ISO_ANTI_HAND_SHAKE
#define HM3451RAW_CAMERA_ISO100 CAM_ISO100
#define HM3451RAW_CAMERA_ISO200 CAM_ISO200
#define HM3451RAW_CAMERA_ISO400 CAM_ISO400
#define HM3451RAW_CAMERA_ISO800 CAM_ISO800
#define HM3451RAW_CAMERA_ISO1600 CAM_ISO1600
#define HM3451RAW_CAMERA_VIDEO_AUTO CAM_VIDEO_AUTO
#define HM3451RAW_CAMERA_VIDEO_NIGHT CAM_VIDEO_NIGHT
#define HM3451RAW_CAMERA_NO_OF_SCENE_MODE CAM_NO_OF_SCENE_MODE

#endif
#endif
