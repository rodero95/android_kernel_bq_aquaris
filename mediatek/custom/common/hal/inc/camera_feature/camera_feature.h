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
#ifndef _CAMERA_FEATURE_H_
#define _CAMERA_FEATURE_H_

#include "camera_feature_types.h"
#include "camera_feature_id.h"
#include "camera_feature_utility.h"
#include "camera_feature_enum.h"
#include "camera_feature_info.h"
#ifdef  USE_CAMERA_FEATURE_MACRO
    #include "camera_feature_macro.h"
    #include "camera_feature_debug.h"
#endif


namespace NSFeature
{


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Typedef
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
typedef MUINT32 FID_T;  //  type of feature id.
typedef MUINT32 SID_T;  //  type of scene id.


typedef FInfoIF*(*PF_GETFINFO_SCENE_INDEP_T)(FID_T const);
typedef FInfoIF*(*PF_GETFINFO_SCENE_DEP_T  )(FID_T const, SID_T const);
struct FeatureInfoProvider
{
    PF_GETFINFO_SCENE_INDEP_T   pfGetFInfo_SceneIndep;
    PF_GETFINFO_SCENE_DEP_T     pfGetFInfo_SceneDep;
};


typedef enum
{
    ECamRole_Main   =   0,  //   Main Camera
    ECamRole_Sub,           //   Sub Camera
}   ECamRole_T;

typedef enum
{
    ESensorType_RAW =   0,  //  RAW Sensor
    ESensorType_YUV,        //  YUV Sensor
}   ESensorType;


enum
{
    ENumOfScene =   Fid2Type<FID_SCENE_MODE>::Num
};


};  //  namespace NSFeature


#endif  //  _CAMERA_FEATURE_H_

