/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2011. All rights reserved.
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

#ifndef __FM_UTILS_H__
#define __FM_UTILS_H__

#include "fm_typedef.h"


/**
 * Base structure of fm object
 */
#define FM_NAME_MAX 20
struct fm_object {
    fm_s8      	name[FM_NAME_MAX+1];						// name of fm object
    fm_u8       type;									// type of fm object
    fm_u8       flag;									// flag of fm object
    fm_s32      ref;
    void        *priv;
    //struct fm_list	*list;									// list node of fm object
};


/*
 * FM asynchronous information mechanism
 */
struct fm_flag_event {
    fm_s32 ref;
    fm_s8  name[FM_NAME_MAX+1];
    void *priv;

    volatile fm_u32 flag;

    //flag methods
    fm_u32(*send)(struct fm_flag_event* thiz, fm_u32 mask);
    fm_s32(*wait)(struct fm_flag_event* thiz, fm_u32 mask);
    long(*wait_timeout)(struct fm_flag_event* thiz, fm_u32 mask, long timeout);
    fm_u32(*clr)(struct fm_flag_event* thiz, fm_u32 mask);
    fm_u32(*get)(struct fm_flag_event* thiz);
    fm_u32(*rst)(struct fm_flag_event* thiz);
};

extern struct fm_flag_event* fm_flag_event_create(const fm_s8 *name);

extern fm_s32 fm_flag_event_get(struct fm_flag_event *thiz);

extern fm_s32 fm_flag_event_put(struct fm_flag_event *thiz);

#define FM_EVENT_SEND(eventp, mask)  \
({                                    \
    fm_u32 __ret = (fm_u32)0;              \
    if(eventp && (eventp)->send){          \
        __ret = (eventp)->send(eventp, mask);    \
    }                               \
    __ret;                          \
})

#define FM_EVENT_WAIT(eventp, mask)  \
({                                    \
    fm_s32 __ret = (fm_s32)0;              \
    if(eventp && (eventp)->wait){          \
        __ret = (eventp)->wait(eventp, mask);    \
    }                               \
    __ret;                          \
})

#define FM_EVENT_WAIT_TIMEOUT(eventp, mask, timeout)  \
({                                    \
    long __ret = (long)0;              \
    if(eventp && (eventp)->wait_timeout){          \
        __ret = (eventp)->wait_timeout(eventp, mask, timeout);    \
    }                               \
    __ret;                          \
})

#define FM_EVENT_GET(eventp)  \
({                                    \
    fm_u32 __ret = (fm_u32)0;              \
    if(eventp && (eventp)->get){          \
        __ret = (eventp)->get(eventp);    \
    }                               \
    __ret;                          \
})

#define FM_EVENT_RESET(eventp)  \
({                                    \
    fm_u32 __ret = (fm_u32)0;              \
    if(eventp && (eventp)->rst){          \
        __ret = (eventp)->rst(eventp);    \
    }                               \
    __ret;                          \
})

#define FM_EVENT_CLR(eventp, mask)  \
({                                    \
    fm_u32 __ret = (fm_u32)0;              \
    if(eventp && (eventp)->clr){          \
        __ret = (eventp)->clr(eventp, mask);    \
    }                               \
    __ret;                          \
})

/*
 * FM lock mechanism
 */
struct fm_lock {
    fm_s8   name[FM_NAME_MAX+1];
    fm_s32  ref;
    void    *priv;

    //lock methods
    fm_s32(*lock)(struct fm_lock* thiz);
    fm_s32(*unlock)(struct fm_lock* thiz);
};

extern struct fm_lock* fm_lock_create(const fm_s8 *name);

extern fm_s32 fm_lock_get(struct fm_lock *thiz);

extern fm_s32 fm_lock_put(struct fm_lock *thiz);

#define FM_LOCK(a)         \
{                           \
    if((a)->lock){          \
        (a)->lock(a);    \
    }                       \
}

#define FM_UNLOCK(a)         \
{                             \
    if((a)->unlock){          \
        (a)->unlock(a);    \
    }                       \
}


/*
 * FM timer mechanism
 */
enum fm_timer_ctrl {
    FM_TIMER_CTRL_GET_TIME = 0,
    FM_TIMER_CTRL_SET_TIME = 1,
    FM_TIMER_CTRL_MAX
};

#define FM_TIMER_FLAG_ACTIVATED (1<<0)

struct fm_timer {
    fm_s32 ref;
    fm_s8  name[FM_NAME_MAX+1];
    void *priv;                                         // platform detail impliment

    fm_s32 flag;                                        // timer active/inactive
    void (*timeout_func)(unsigned long data);		    //  timeout function
    unsigned long data;									// timeout function's parameter
    signed long timeout_ms;							    // timeout tick

    //timer methods
    fm_s32(*init)(struct fm_timer *thiz, void (*timeout)(unsigned long data), unsigned long data, signed long time, fm_s32 flag);
    fm_s32(*start)(struct fm_timer *thiz);
    fm_s32(*update)(struct fm_timer *thiz);
    fm_s32(*stop)(struct fm_timer *thiz);
    fm_s32(*control)(struct fm_timer *thiz, enum fm_timer_ctrl cmd, void* arg);
};

extern struct fm_timer* fm_timer_create(const fm_s8 *name);

extern fm_s32 fm_timer_get(struct fm_timer *thiz);

extern fm_s32 fm_timer_put(struct fm_timer *thiz);

/*
 * FM work thread mechanism
 */
struct fm_work {
    fm_s32 ref;
    fm_s8  name[FM_NAME_MAX+1];
    void *priv;

    void (*work_func)(unsigned long data);
    unsigned long data;
    //work methods
    fm_s32(*init)(struct fm_work *thiz, void (*work_func)(unsigned long data), unsigned long data);
};

extern struct fm_work* fm_work_create(const fm_s8 *name);

extern fm_s32 fm_work_get(struct fm_work *thiz);

extern fm_s32 fm_work_put(struct fm_work *thiz);


struct fm_workthread {
    fm_s32 ref;
    fm_s8  name[FM_NAME_MAX+1];
    void *priv;

    //workthread methods
    fm_s32(*add_work)(struct fm_workthread *thiz, struct fm_work *work);
};

extern struct fm_workthread* fm_workthread_create(const fm_s8* name);

extern fm_s32 fm_workthread_get(struct fm_workthread *thiz);

extern fm_s32 fm_workthread_put(struct fm_workthread *thiz);

#endif //__FM_UTILS_H__

