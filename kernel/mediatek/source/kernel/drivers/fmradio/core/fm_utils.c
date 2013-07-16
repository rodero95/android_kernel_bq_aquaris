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


/* fm_event.c
 *
 * (C) Copyright 2011
 * MediaTek <www.MediaTek.com>
 * Hongcheng <hongcheng.xia@MediaTek.com>
 *
 * FM Radio Driver -- a common event
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>
#include <linux/timer.h>

#include "fm_typedef.h"
#include "fm_dbg.h"
#include "fm_err.h"
#include "fm_stdlib.h"
#include "fm_utils.h"

static fm_u32 fm_event_send(struct fm_flag_event* thiz, fm_u32 mask)
{
    thiz->flag |= mask;
    //WCN_DBG(FM_DBG|MAIN, "%s set 0x%08x\n", thiz->name, thiz->flag);
    wake_up((wait_queue_head_t*)(thiz->priv));

    return thiz->flag;
}

static fm_s32 fm_event_wait(struct fm_flag_event* thiz, fm_u32 mask)
{
    return wait_event_interruptible(*(wait_queue_head_t*)(thiz->priv), ((thiz->flag & mask) == mask));
}

/**
 * fm_event_check - sleep until a condition gets true or a timeout elapses
 * @thiz: the pointer of current object
 * @mask: bitmap in fm_u32
 * @timeout: timeout, in jiffies
 *
 * fm_event_set() has to be called after changing any variable that could
 * change the result of the wait condition.
 *
 * The function returns 0 if the @timeout elapsed, and the remaining
 * jiffies if the condition evaluated to true before the timeout elapsed.
 */
long fm_event_wait_timeout(struct fm_flag_event* thiz, fm_u32 mask, long timeout)
{
    return wait_event_timeout(*((wait_queue_head_t*)(thiz->priv)), ((thiz->flag & mask) == mask), timeout*HZ);
}

static fm_u32 fm_event_clr(struct fm_flag_event* thiz, fm_u32 mask)
{
    thiz->flag &= ~mask;
    //WCN_DBG(FM_DBG|MAIN, "%s clr 0x%08x\n", thiz->name, thiz->flag);
    return thiz->flag;
}

static fm_u32 fm_event_get(struct fm_flag_event* thiz)
{
    return thiz->flag;

}

static fm_u32 fm_event_rst(struct fm_flag_event* thiz)
{
    return thiz->flag = 0;
}

struct fm_flag_event* fm_flag_event_create(const fm_s8 *name) 
{
    struct fm_flag_event *tmp;
    wait_queue_head_t *wq;

    if (!(tmp = fm_zalloc(sizeof(struct fm_flag_event)))) {
        WCN_DBG(FM_ALT | MAIN, "fm_zalloc(fm_event) -ENOMEM\n");
        return NULL;
    }

    if (!(wq = fm_zalloc(sizeof(wait_queue_head_t)))) {
        WCN_DBG(FM_ALT | MAIN, "fm_zalloc(wait_queue_head_t) -ENOMEM\n");
        fm_free(tmp);
        return NULL;
    }

    fm_memcpy(tmp->name, name, FM_NAME_MAX);
    tmp->priv = wq;
    init_waitqueue_head(wq);
    tmp->ref = 0;

    tmp->send = fm_event_send;
    tmp->wait = fm_event_wait;
    tmp->wait_timeout = fm_event_wait_timeout;
    tmp->clr = fm_event_clr;
    tmp->get = fm_event_get;
    tmp->rst = fm_event_rst;

    tmp->rst(tmp);  //set flag to 0x00000000

    return tmp;
}

fm_s32 fm_flag_event_get(struct fm_flag_event *thiz)
{
    FMR_ASSERT(thiz);
    thiz->ref++;
    return 0;
}

fm_s32 fm_flag_event_put(struct fm_flag_event *thiz)
{
    FMR_ASSERT(thiz);
    thiz->ref--;

    if (thiz->ref == 0) {
        fm_free(thiz->priv);
        fm_free(thiz);
        return 0;
    } else if (thiz->ref > 0) {
        return -FM_EINUSE;
    } else {
        return -FM_EPARA;
    }
}

//fm lock methods
static fm_s32 fm_lock_lock(struct fm_lock *thiz)
{
    struct semaphore *sem;
    struct task_struct *task = current;
    FMR_ASSERT(thiz);
    FMR_ASSERT(thiz->priv);

    if (down_interruptible((struct semaphore*)thiz->priv)) {
        WCN_DBG(FM_CRT | MAIN, "get mutex failed\n");
        return -FM_ELOCK;
    }

    sem = (struct semaphore*)thiz->priv;
    WCN_DBG(FM_DBG | MAIN, "%s --->lock, cnt=%d, pid=%d\n", thiz->name, (int)sem->count, task->pid);
    return 0;
}

static fm_s32 fm_lock_unlock(struct fm_lock *thiz)
{
    struct semaphore *sem;
    struct task_struct *task = current;
    FMR_ASSERT(thiz);
    FMR_ASSERT(thiz->priv);
    sem = (struct semaphore*)thiz->priv;
    WCN_DBG(FM_DBG | MAIN, "%s <---unlock, cnt=%d, pid=%d\n", thiz->name, (int)sem->count + 1, task->pid);
    up((struct semaphore*)thiz->priv);
    return 0;
}

struct fm_lock* fm_lock_create(const fm_s8 *name) 
{
    struct fm_lock *tmp;
    struct semaphore *mutex;

    if (!(tmp = fm_zalloc(sizeof(struct fm_lock)))) {
        WCN_DBG(FM_ALT | MAIN, "fm_zalloc(fm_lock) -ENOMEM\n");
        return NULL;
    }

    if (!(mutex = fm_zalloc(sizeof(struct semaphore)))) {
        WCN_DBG(FM_ALT | MAIN, "fm_zalloc(struct semaphore) -ENOMEM\n");
        fm_free(tmp);
        return NULL;
    }

    tmp->priv = mutex;
    *mutex = (struct semaphore) __SEMAPHORE_INITIALIZER(*mutex, 1);
    tmp->ref = 0;
    fm_memcpy(tmp->name, name, FM_NAME_MAX);

    tmp->lock = fm_lock_lock;
    tmp->unlock = fm_lock_unlock;

    return tmp;
}

fm_s32 fm_lock_get(struct fm_lock *thiz)
{
    FMR_ASSERT(thiz);
    thiz->ref++;
    return 0;
}

fm_s32 fm_lock_put(struct fm_lock *thiz)
{
    FMR_ASSERT(thiz);
    thiz->ref--;

    if (thiz->ref == 0) {
        fm_free(thiz->priv);
        fm_free(thiz);
        return 0;
    } else if (thiz->ref > 0) {
        return -FM_EINUSE;
    } else {
        return -FM_EPARA;
    }
}


/*
 * fm timer
 *
 */
static fm_s32 fm_timer_init(struct fm_timer *thiz, void (*timeout)(unsigned long data), unsigned long data, signed long time, fm_s32 flag)
{
    struct timer_list *timerlist = (struct timer_list*)thiz->priv;

    thiz->flag = flag;
    thiz->flag &= ~FM_TIMER_FLAG_ACTIVATED;
    thiz->timeout_func = timeout;
    thiz->data = data;
    thiz->timeout_ms = time;

    timerlist->expires  = jiffies + (thiz->timeout_ms) / (1000 / HZ);
    timerlist->function = thiz->timeout_func;
    timerlist->data     = (unsigned long)thiz->data;

    return 0;
}

static fm_s32 fm_timer_start(struct fm_timer *thiz)
{
    struct timer_list *timerlist = (struct timer_list*)thiz->priv;

    thiz->flag |= FM_TIMER_FLAG_ACTIVATED;
    mod_timer(timerlist, jiffies + (thiz->timeout_ms) / (1000 / HZ));

    return 0;
}

static fm_s32 fm_timer_update(struct fm_timer *thiz)
{
    struct timer_list *timerlist = (struct timer_list*)thiz->priv;

    if (thiz->flag & FM_TIMER_FLAG_ACTIVATED) {
        mod_timer(timerlist, jiffies + (thiz->timeout_ms) / (1000 / HZ));
        return 0;
    } else {
        return 1;
    }
}

static fm_s32 fm_timer_stop(struct fm_timer *thiz)
{
    struct timer_list *timerlist = (struct timer_list*)thiz->priv;

    thiz->flag &= ~FM_TIMER_FLAG_ACTIVATED;
    del_timer(timerlist);

    return 0;
}

static fm_s32 fm_timer_control(struct fm_timer *thiz, enum fm_timer_ctrl cmd, void* arg)
{

    return 0;
}

struct fm_timer* fm_timer_create(const fm_s8 *name) 
{
    struct fm_timer *tmp;
    struct timer_list *timerlist;

    if (!(tmp = fm_zalloc(sizeof(struct fm_timer)))) {
        WCN_DBG(FM_ALT | MAIN, "fm_zalloc(fm_timer) -ENOMEM\n");
        return NULL;
    }

    if (!(timerlist = fm_zalloc(sizeof(struct timer_list)))) {
        WCN_DBG(FM_ALT | MAIN, "fm_zalloc(struct timer_list) -ENOMEM\n");
        fm_free(tmp);
        return NULL;
    }

    init_timer(timerlist);

    fm_memcpy(tmp->name, name, FM_NAME_MAX);
    tmp->priv = timerlist;
    tmp->ref = 0;
    tmp->init = fm_timer_init;
    tmp->start = fm_timer_start;
    tmp->stop = fm_timer_stop;
    tmp->update = fm_timer_update;
    tmp->control = fm_timer_control;

    return tmp;
}

fm_s32 fm_timer_get(struct fm_timer *thiz)
{
    FMR_ASSERT(thiz);
    thiz->ref++;
    return 0;
}

fm_s32 fm_timer_put(struct fm_timer *thiz)
{
    FMR_ASSERT(thiz);
    thiz->ref--;

    if (thiz->ref == 0) {
        fm_free(thiz->priv);
        fm_free(thiz);
        return 0;
    } else if (thiz->ref > 0) {
        return -FM_EINUSE;
    } else {
        return -FM_EPARA;
    }
}


/*
 * FM work thread mechanism
 */
static fm_s32 fm_work_init(struct fm_work *thiz, void (*work_func)(unsigned long data), unsigned long data)
{
    struct work_struct *sys_work = (struct work_struct*)thiz->priv;
    work_func_t func;

    thiz->work_func = work_func;
    thiz->data = data;
    func = (work_func_t)thiz->work_func;

    INIT_WORK(sys_work, func);

    return 0;

}

struct fm_work* fm_work_create(const fm_s8 *name) 
{
    struct fm_work *my_work;
    struct work_struct *sys_work;

    if (!(my_work = fm_zalloc(sizeof(struct fm_work)))) {
        WCN_DBG(FM_ALT | MAIN, "fm_zalloc(fm_work) -ENOMEM\n");
        return NULL;
    }

    if (!(sys_work = fm_zalloc(sizeof(struct work_struct)))) {
        WCN_DBG(FM_ALT | MAIN, "fm_zalloc(struct work_struct) -ENOMEM\n");
        fm_free(my_work);
        return NULL;
    }

    fm_memcpy(my_work->name, name, FM_NAME_MAX);
    my_work->priv = sys_work;
    my_work->init = fm_work_init;

    return my_work;
}

fm_s32 fm_work_get(struct fm_work *thiz)
{
    FMR_ASSERT(thiz);
    thiz->ref++;
    return 0;
}

fm_s32 fm_work_put(struct fm_work *thiz)
{
    FMR_ASSERT(thiz);
    thiz->ref--;

    if (thiz->ref == 0) {
        fm_free(thiz->priv);
        fm_free(thiz);
        return 0;
    } else if (thiz->ref > 0) {
        return -FM_EINUSE;
    } else {
        return -FM_EPARA;
    }
}


static fm_s32 fm_workthread_add_work(struct fm_workthread *thiz, struct fm_work *work)
{
    FMR_ASSERT(thiz);
    FMR_ASSERT(work);

    queue_work((struct workqueue_struct*)thiz->priv, (struct work_struct*)work->priv);
    return 0;
}

struct fm_workthread* fm_workthread_create(const fm_s8* name) 
{
    struct fm_workthread *my_thread;
    struct workqueue_struct *sys_thread;

    if (!(my_thread = fm_zalloc(sizeof(struct fm_workthread)))) {
        WCN_DBG(FM_ALT | MAIN, "fm_zalloc(fm_workthread) -ENOMEM\n");
        return NULL;
    }

    sys_thread = create_singlethread_workqueue(name);

    fm_memcpy(my_thread->name, name, FM_NAME_MAX);
    my_thread->priv = sys_thread;
    my_thread->add_work = fm_workthread_add_work;

    return my_thread;
}

fm_s32 fm_workthread_get(struct fm_workthread *thiz)
{
    FMR_ASSERT(thiz);
    thiz->ref++;
    return 0;
}

fm_s32 fm_workthread_put(struct fm_workthread *thiz)
{
    FMR_ASSERT(thiz);
    thiz->ref--;

    if (thiz->ref == 0) {
        destroy_workqueue((struct workqueue_struct*)thiz->priv);
        fm_free(thiz);
        return 0;
    } else if (thiz->ref > 0) {
        return -FM_EINUSE;
    } else {
        return -FM_EPARA;
    }
}

