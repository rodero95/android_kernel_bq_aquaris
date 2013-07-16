/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2012. All rights reserved.
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

#include "tpd.h"
#include "tpd_custom_gt968.h"

#define TPD_PROXIMITY
#ifdef TPD_PROXIMITY
#define TPD_PROXIMITY_DEBUG
#define TPD_PROXIMITY_DEVICE            "gt968-ps"
#define TPD_PROXIMITY_DMESG(a,arg...) printk(TPD_PROXIMITY_DEVICE ": " a,##arg)
#if 1//defined(TPD_PROXIMITY_DEBUG)
#undef TPD_PROXIMITY_DEBUG
#define TPD_PROXIMITY_DEBUG(a,arg...) printk(TPD_PROXIMITY_DEVICE ": " a,##arg)
#else
#define TPD_PROXIMITY_DEBUG(arg...) 
#endif
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif
#include <linux/platform_device.h>

extern struct tpd_device *tpd;

static int tpd_flag = 0;
static int tpd_halt = 0;
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);

#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#if GTP_HAVE_TOUCH_KEY
const u16 touch_key_array[] = { KEY_MENU, KEY_HOMEPAGE, KEY_BACK, KEY_SEARCH };
#define GTP_MAX_KEY_NUM ( sizeof( touch_key_array )/sizeof( touch_key_array[0] ) )
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
//static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

s32 gtp_send_cfg(struct i2c_client *client);
static void tpd_eint_interrupt_handler(void);
static int touch_event_handler(void *unused);
static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_i2c_remove(struct i2c_client *client);
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);

#ifdef TPD_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client *);
extern void uninit_wr_node(void);
#endif

//#define TPD_CONDITION_SWITCH
#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_MAX_RESET_COUNT 3

#ifdef TPD_ESD_PROTECT
#define TPD_ESD_CHECK_CIRCLE        2000
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
#endif

#ifdef TPD_PROXIMITY
#define TPD_PROXIMITY_REG                   0x814e 
#define TPD_POWER_MODE_REG            0x8042

static u8 tpd_proximity_flag = 0;
static u8 tpd_proximity_detect = 1;//0-->close ; 1--> far away
#endif

#define TPD_CONFIG_REG_BASE           0x6A2
#define TPD_FREQ_CAL_RESULT           0x70F
#define TPD_SENSOR_ID_REG             0x710
#define TPD_TOUCH_INFO_REG_BASE       0x712
#define TPD_POINT_INFO_REG_BASE       0x722
#define TPD_VERSION_INFO_REG          0x713
#define TPD_VERSION_BASIC_REG         0x717
#define TPD_KEY_INFO_REG_BASE         0x721

#define TPD_HANDSHAKING_START_REG     0xFFF
#define TPD_HANDSHAKING_END_REG       0x8000
#define TPD_FREQ_REG                  0x1522
#define TPD_SOFT_RESET_MODE           0x01
#define TPD_POINT_INFO_LEN            8
#define TPD_MAX_POINTS                5
#define MAX_TRANSACTION_LENGTH        8
#define I2C_DEVICE_ADDRESS_LEN        2
#define I2C_MASTER_CLOCK              300

#ifdef MT6573
#define CHR_CON0                      (0xF7000000+0x2FA00)
#endif
#ifdef MT6575
extern kal_bool upmu_is_chr_det(void);
#endif
#ifdef MT6577
extern kal_bool upmu_is_chr_det(void);
#endif

#define MAX_I2C_TRANSFER_SIZE (MAX_TRANSACTION_LENGTH - I2C_DEVICE_ADDRESS_LEN)

#define GT91XX_CONFIG_PROC_FILE "gt91xx_config"
struct gt968_ps_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};

struct gt968_ps_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;

    /*i2c address group*/
    struct gt968_ps_i2c_addr  addr;
    
    /*misc*/

    atomic_t    i2c_retry;

    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;

    /*data*/
    u16          ps;
    u8          _align;

    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};

struct tpd_info_t
{
    u8 vendor_id_1;
    u8 vendor_id_2;
    u8 product_id_1;
    u8 product_id_2;
    u8 version_1;
    u8 version_2;
};

struct i2c_client *i2c_client_point = NULL;
static const struct i2c_device_id tpd_i2c_id[] = {{"gt818b", 0}, {}};
static unsigned short force[] = {0, 0xBA, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces,};
static struct i2c_board_info __initdata i2c_tpd = { I2C_BOARD_INFO("gt818b", (0xBA >> 1))};
static struct i2c_driver tpd_i2c_driver =
{
    .probe = tpd_i2c_probe,
    .remove = tpd_i2c_remove,
    .detect = tpd_i2c_detect,
    .driver.name = "gt818b",
    .id_table = tpd_i2c_id,
    .address_list = (const unsigned short *) forces,
};
struct tpd_info_t tpd_info;
static u8 config[GTP_CONFIG_LENGTH + GTP_ADDR_LENGTH]
                = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
u8 *cfg_data_with_charger = NULL;
u8 chip_type = 0;
u8 int_type = 0;
u32 abs_x_max = 0;
u32 abs_y_max = 0;
u8 gtp_rawdiff_mode = 0;

/* proc file system */
static int i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
static int i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len);
static int i2c_write_dummy(struct i2c_client *client, u16 addr);
static struct proc_dir_entry *gt91xx_config_proc = NULL;

#define VELOCITY_CUSTOM_GT818B
#ifdef VELOCITY_CUSTOM_GT818B
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 10
#endif

// for magnify velocity********************************************
#define TOUCH_IOC_MAGIC 'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

int g_v_magnify_x = TPD_VELOCITY_CUSTOM_X;
int g_v_magnify_y = TPD_VELOCITY_CUSTOM_Y;
static int tpd_misc_open(struct inode *inode, struct file *file)
{
    return nonseekable_open(inode, file);
}

static int tpd_misc_release(struct inode *inode, struct file *file)
{
    return 0;
}

static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
                               unsigned long arg)
{
    //char strbuf[256];
    void __user *data;

    long err = 0;

    if (_IOC_DIR(cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }

    if (err)
    {
        printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

    switch (cmd)
    {
        case TPD_GET_VELOCITY_CUSTOM_X:
            data = (void __user *) arg;

            if (data == NULL)
            {
                err = -EINVAL;
                break;
            }

            if (copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
            {
                err = -EFAULT;
                break;
            }

            break;

        case TPD_GET_VELOCITY_CUSTOM_Y:
            data = (void __user *) arg;

            if (data == NULL)
            {
                err = -EINVAL;
                break;
            }

            if (copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
            {
                err = -EFAULT;
                break;
            }

            break;

        default:
            printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
            break;

    }

    return err;
}


static struct file_operations tpd_fops =
{
//	.owner = THIS_MODULE,
    .open = tpd_misc_open,
    .release = tpd_misc_release,
    .unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "touch",
    .fops = &tpd_fops,
};

//**********************************************
#endif


static int gt91xx_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    char *ptr = page;
    char temp_data[GTP_CONFIG_LENGTH+2] = {0};
    int i;

    ptr += sprintf(ptr, "==== GT91XX config init value====\n");

    for (i = 0 ; i < GTP_CONFIG_LENGTH+2 ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", config[i+2]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }

    ptr += sprintf(ptr, "\n");

    ptr += sprintf(ptr, "==== GT91XX config real value====\n");
    i2c_read_bytes(i2c_client_point, GTP_REG_CONFIG_DATA, temp_data, GTP_CONFIG_LENGTH);

    for (i = 0 ; i < GTP_CONFIG_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", temp_data[i]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }

    ptr += sprintf(ptr, "\n");
    *eof = 1;
    return (ptr - page);
}

static int gt91xx_config_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
    s32 ret = 0;

    GTP_DEBUG("write count %ld\n", count);

    if (count != (GTP_CONFIG_LENGTH * 2))
    {
        GTP_DEBUG("size not match [%d:%ld]\n", GTP_CONFIG_LENGTH * 2, count);
        return -EFAULT;
    }

    if (copy_from_user(&config[2], buffer, count / 2))
    {
        GTP_DEBUG("copy from user fail\n");
        return -EFAULT;
    }

    ret = gtp_send_cfg(i2c_client_point);
    if(ret < 0)
    {
        GTP_DEBUG("send config failed.");
    }

    return count;
}

static int i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buffer[I2C_DEVICE_ADDRESS_LEN];
    u8 retry;
    u16 left = len;
    u16 offset = 0;

    struct i2c_msg msg[2] =
    {
        {
            //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
            .addr = ((client->addr &I2C_MASK_FLAG)),
            .flags = 0,
            .buf = buffer,
            .len = I2C_DEVICE_ADDRESS_LEN,
            .timing = I2C_MASTER_CLOCK
        },
        {
            //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
            .addr = ((client->addr &I2C_MASK_FLAG)),
            .flags = I2C_M_RD,
            .timing = I2C_MASTER_CLOCK
        },
    };

    if (rxbuf == NULL)
        return -1;

   // GTP_DEBUG("i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

    while (left > 0)
    {
        buffer[0] = ((addr + offset) >> 8) & 0xFF;
        buffer[1] = (addr + offset) & 0xFF;

        msg[1].buf = &rxbuf[offset];

        if (left > MAX_TRANSACTION_LENGTH)
        {
            msg[1].len = MAX_TRANSACTION_LENGTH;
            left -= MAX_TRANSACTION_LENGTH;
            offset += MAX_TRANSACTION_LENGTH;
        }
        else
        {
            msg[1].len = left;
            left = 0;
        }

        retry = 0;

        while (i2c_transfer(client->adapter, &msg[0], 2) != 2)
        {
            retry++;

            if (retry == 20)
            {
                GTP_DEBUG("I2C read 0x%X length=%d failed\n", addr + offset, len);
                GTP_INFO("I2C read 0x%X length=%d failed\n", addr + offset, len);
                return -1;
            }
        }
    }

    return 0;
}

s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    s32 ret = -1;
    u16 addr = (buf[0]<<8) + buf[1];
    
    ret = i2c_read_bytes(client, addr, &buf[2], len-2);
    if(!ret)
    {
        return 2;
    }
    else
    {
        gtp_reset_guitar(client, 20);
        return ret;
    }
}

static int i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len)
{
    u8 buffer[MAX_TRANSACTION_LENGTH];
    u16 left = len;
    u16 offset = 0;
    u8 retry = 0;

    struct i2c_msg msg =
    {
        //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
        .addr = ((client->addr &I2C_MASK_FLAG)),
        .flags = 0,
        .buf = buffer,
        .timing = I2C_MASTER_CLOCK,
    };


    if (txbuf == NULL)
        return -1;

  //  GTP_DEBUG("i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

    while (left > 0)
    {
        retry = 0;

        buffer[0] = ((addr + offset) >> 8) & 0xFF;
        buffer[1] = (addr + offset) & 0xFF;

        if (left > MAX_I2C_TRANSFER_SIZE)
        {
            memcpy(&buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], MAX_I2C_TRANSFER_SIZE);
            msg.len = MAX_TRANSACTION_LENGTH;
            left -= MAX_I2C_TRANSFER_SIZE;
            offset += MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            memcpy(&buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], left);
            msg.len = left + I2C_DEVICE_ADDRESS_LEN;
            left = 0;
        }

   //     GTP_DEBUG("byte left %d offset %d\n", left, offset);

        while (i2c_transfer(client->adapter, &msg, 1) != 1)
        {
            retry++;

            if (retry == 20)
            {
                GTP_DEBUG("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
                GTP_INFO("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
                return -1;
            }
            else
                GTP_DEBUG("I2C write retry %d addr 0x%X%X\n", retry, buffer[0], buffer[1]);

        }
    }

    return 0;
}

s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
    s32 ret = -1;
    u16 addr = (buf[0]<<8) + buf[1];
    
    ret = i2c_write_bytes(client, addr, &buf[2], len-2);
    if(!ret)
    {
        return 1;
    }
    else
    {
        gtp_reset_guitar(client, 20);
        return ret;
    }
}

static int i2c_write_dummy(struct i2c_client *client, u16 addr)
{
    u8 buffer[MAX_TRANSACTION_LENGTH];
    s8 ret = -1;

    struct i2c_msg msg =
    {
        .addr = client->addr,
        .flags = 0,
        .buf = buffer,
        .timing = I2C_MASTER_CLOCK,
        .len = 2
    };

    GTP_DEBUG("i2c_write_dummy to device %02X address %04X\n", client->addr, addr);

    buffer[0] = (addr >> 8) & 0xFF;
    buffer[1] = (addr) & 0xFF;

    ret = i2c_transfer(client->adapter, &msg, 1);

    return ret;
}

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, "mtk-tpd");
    return 0;
}

#ifdef TPD_PROXIMITY
int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}

int tpd_enable_ps(int enable)
{
	u8 state;
	int ret = -1;

	ret = i2c_read_bytes(i2c_client_point, TPD_POWER_MODE_REG, &state, 1);

	if (enable){
		state =1;
		tpd_proximity_flag = 1;
		TPD_PROXIMITY_DEBUG("tpd-ps function is on\n");
	}else{
		state =0;	
		tpd_proximity_flag = 0;
		TPD_PROXIMITY_DEBUG("tpd-ps function is off\n");
	}

	ret = i2c_write_bytes(i2c_client_point, TPD_POWER_MODE_REG, &state, 1);
	TPD_PROXIMITY_DEBUG("write: 0x8042's value is 0x%02X\n", state);
	return 0;
}

int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;
	TPD_PROXIMITY_DMESG("tpd_ps_operate!\n");
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				TPD_PROXIMITY_DMESG("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				TPD_PROXIMITY_DMESG("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
					if((tpd_enable_ps(1) != 0))
					{
						TPD_PROXIMITY_DMESG("enable ps fail: %d\n", err); 
						return -1;
					}
//					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						TPD_PROXIMITY_DMESG("disable ps fail: %d\n", err); 
						return -1;
					}
//					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				TPD_PROXIMITY_DMESG("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;				
				
				sensor_data->values[0] = tpd_get_ps_value();
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;	
				TPD_PROXIMITY_DMESG("get sensor %d!\n", sensor_data->values[0]);	
			}
			break;
			
		default:
			TPD_PROXIMITY_DMESG("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
	
}
#endif

/*******************************************************
Function:
	Send config Function.

Input:
	client:	i2c client.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
    s32 ret = 1;
#if GTP_DRIVER_SEND_CFG
    s32 retry = 0;

    for (retry = 0; retry < 5; retry++)
    {
        ret = gtp_i2c_write(client, config , GTP_CONFIG_LENGTH + GTP_ADDR_LENGTH);
        if (ret > 0)
        {
            break;
        }
    }
#endif

    return ret;
}


/*******************************************************
Function:
	Read goodix touchscreen version function.

Input:
	client:	i2c client struct.
	version:address to store version info
	
Output:
	Executive outcomes.0---succeed.
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16* version)
{
    s32 ret = -1;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

    GTP_DEBUG_FUNC();

    ret = gtp_i2c_read(client, buf, sizeof(buf) + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        GTP_ERROR("GTP read version failed"); 
        return ret;
    }

    if (version)
    {
        *version = (buf[7] << 8) | buf[6];
    }

    GTP_INFO("IC VERSION:%c%c%c%c_%02x%02x", 
              buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);

    return ret;
}
/*******************************************************
Function:
	GTP initialize function.

Input:
	client:	i2c client private struct.
	
Output:
	Executive outcomes.0---succeed.
*******************************************************/
static s32 gtp_init_panel(struct i2c_client *client)
{
    s32 ret = -1;
  
#if GTP_DRIVER_SEND_CFG
    s32 i;
    u8 check_sum = 0;
    u8 rd_cfg_buf[16];

    u8 cfg_info_group1[] = CTP_CFG_GROUP1;
    u8 cfg_info_group2[] = CTP_CFG_GROUP2;
    u8 cfg_info_group3[] = CTP_CFG_GROUP3;
    u8 *send_cfg_buf[3] = {cfg_info_group1, cfg_info_group2, cfg_info_group3};
    u8 cfg_info_len[3] = {sizeof(cfg_info_group1)/sizeof(cfg_info_group1[0]), 
                          sizeof(cfg_info_group2)/sizeof(cfg_info_group2[0]),
                          sizeof(cfg_info_group3)/sizeof(cfg_info_group3[0])};
    GTP_DEBUG("len1=%d,len2=%d,len3=%d",cfg_info_len[0],cfg_info_len[1],cfg_info_len[2]);
    if ((!cfg_info_len[1]) && (!cfg_info_len[2]))
    {
        rd_cfg_buf[GTP_ADDR_LENGTH] = 0; 
    }
    else
    {
        rd_cfg_buf[0] = GTP_REG_SENSOR_ID >> 8;
        rd_cfg_buf[1] = GTP_REG_SENSOR_ID & 0xff;
        ret = gtp_i2c_read(client, rd_cfg_buf, 3);
        if (ret < 0)
        {
            GTP_ERROR("Read SENSOR ID failed,default use group1 config!");
            rd_cfg_buf[GTP_ADDR_LENGTH] = 0;
        }
        rd_cfg_buf[GTP_ADDR_LENGTH] &= 0x03;
    }
    GTP_DEBUG("SENSOR ID:%d", rd_cfg_buf[GTP_ADDR_LENGTH]);
    memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[rd_cfg_buf[GTP_ADDR_LENGTH]], GTP_CONFIG_LENGTH);

#if GTP_CUSTOM_CFG
    config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
    config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
    config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
    config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);
#endif  //endif GTP_CUSTOM_CFG
    if (GTP_INT_TRIGGER == 0)  //RISING
    {
        config[TRIGGER_LOC] &= 0xfe; 
    }
    else if (GTP_INT_TRIGGER == 1)  //FALLING
    {
        config[TRIGGER_LOC] |= 0x01;
    }
    
    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < GTP_CONFIG_LENGTH; i++)
    {
        check_sum += config[i];
    }
    config[GTP_CONFIG_LENGTH] = (~check_sum) + 1;
    
#else //else DRIVER NEED NOT SEND CONFIG

    ret = gtp_i2c_read(client, config, GTP_CONFIG_LENGTH + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        GTP_ERROR("GTP read resolution & max_touch_num failed, use default value!");
        abs_x_max = GTP_MAX_WIDTH;
        abs_y_max = GTP_MAX_HEIGHT;
        int_type = GTP_INT_TRIGGER;
    }
#endif //endif GTP_DRIVER_SEND_CFG

    abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
    abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
    int_type = (config[TRIGGER_LOC]) & 0x03;
    if ((!abs_x_max)||(!abs_y_max))
    {
        GTP_ERROR("GTP resolution & max_touch_num invalid, use default value!");
        abs_x_max = GTP_MAX_WIDTH;
        abs_y_max = GTP_MAX_HEIGHT;
    }

    ret = gtp_send_cfg(client);
    if (ret < 0)
    {
        GTP_ERROR("Send config error.");
    }

    GTP_DEBUG("X_MAX = %d,Y_MAX = %d,TRIGGER = 0x%02x",abs_x_max,abs_y_max,int_type);

    msleep(10);

    return 0;
}

static s8 gtp_i2c_test(struct i2c_client *client)
{
    u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
    u8 retry = 0;
    s8 ret = -1;
  
    GTP_DEBUG_FUNC();
  
    while(retry++ < 5)
    {
        ret = gtp_i2c_read(client, test, 3);
        if (ret > 0)
        {
            return ret;
        }
        GTP_ERROR("GTP i2c test failed time %d.",retry);
        msleep(10);
    }
    return ret;
}

void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
    msleep(ms);
    GTP_GPIO_OUTPUT(GTP_INT_PORT, client->addr == 0x14);

    msleep(2);
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
    
    msleep(5);                          //must > 3ms
    //GTP_GPIO_AS_INPUT(GTP_RST_PORT);
    GTP_GPIO_AS_INT(GTP_INT_PORT);
    msleep(60);

    return;
}

static int tpd_power_on(struct i2c_client *client)
{
    int ret = 0;
    int reset_count = 0;
    
reset_proc:
    #ifdef MT6573
        // power on CTP
        mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
    #endif
    #ifdef MT6575
        //power on, need confirm with SA
        hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
        hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "TP");
    #endif
    #ifdef MT6577
        //power on, need confirm with SA
        hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
        hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "TP");
    #endif

    gtp_reset_guitar(client, 20);

    ret = gtp_i2c_test(client);
    if (ret < 0)
    {
        GTP_ERROR("I2C communication ERROR!");
        if (reset_count < TPD_MAX_RESET_COUNT)
        {
            reset_count++;
            goto reset_proc;
        }
    }
    return ret;
}
static ssize_t  set_val(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	tpd_enable_ps(1);
 
    return 1;
}

static DEVICE_ATTR(gt968_ps, S_IRUGO | S_IWUSR, NULL, set_val);

static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;
    int idx = 0;
    s32 ret = 0;
    u16 version_info;
#ifdef TPD_PROXIMITY
	struct hwmsen_object obj_ps;
#endif
    i2c_client_point = client;
    ret = tpd_power_on(client);
    if(ret < 0)
    {
        GTP_ERROR("I2C communication ERROR!");
    }
    
#if GTP_AUTO_UPDATE
    ret = gup_init_update_proc(client);
    if (ret < 0)
    {
        GTP_ERROR("Create update thread error.");
    }
#endif

#ifdef VELOCITY_CUSTOM_GT818B

    if ((err = misc_register(&tpd_misc_device)))
    {
        printk("mtk_tpd: tpd_misc_device register failed\n");
    }
#endif

    ret = gtp_init_panel(client);
    if (ret < 0)
    {
        GTP_ERROR("GTP init panel failed.");
    }
    
    ret = gtp_read_version(client, &version_info);
    if (ret < 0)
    {
        GTP_ERROR("Read version failed.");
    }

    // Create proc file system
    gt91xx_config_proc = create_proc_entry(GT91XX_CONFIG_PROC_FILE, 0666, NULL);
    if (gt91xx_config_proc == NULL)
    {
        GTP_DEBUG("create_proc_entry %s failed\n", GT91XX_CONFIG_PROC_FILE);
    }
    else
    {
       gt91xx_config_proc->read_proc = gt91xx_config_read_proc;
       gt91xx_config_proc->write_proc = gt91xx_config_write_proc;
		//GTP_ERROR("huangzs 12. %p %p", gt91xx_config_proc, gt91xx_config_proc->read_proc);

    }

#ifdef TPD_CREATE_WR_NODE
    init_wr_node(client);
#endif

    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);

    if (IS_ERR(thread))
    {
        err = PTR_ERR(thread);
        GTP_INFO(TPD_DEVICE " failed to create kernel thread: %d\n", err);
    }

#if GTP_HAVE_TOUCH_KEY

    for (idx = 0; idx < GTP_MAX_KEY_NUM; idx++)
    {
        input_set_capability(tpd->dev, EV_KEY, touch_key_array[idx]);
    }

#endif


#ifndef TPD_RESET_ISSUE_WORKAROUND
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);
#endif

    // set INT mode
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);

    msleep(50);

    mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);

    //mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1);
    if (!int_type)
    {
        mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_HIGH, tpd_eint_interrupt_handler, 1);
    }
    else
    {
        mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, tpd_eint_interrupt_handler, 1);
    }

    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

#ifndef TPD_RESET_ISSUE_WORKAROUND
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
#if 0//TPD_PROXIMITY
{
struct gt968_ps_priv *obj;

	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		TPD_PROXIMITY_DMESG("kzalloc fail = %d\n", err);
		return 0;
	}
	memset(obj, 0, sizeof(*obj));


	obj_ps.self = obj;
	obj_ps.polling = 0;//interrupt mode
//	obj_ps.polling = 1;//need to confirm what mode is!!!
	obj_ps.sensor_operate = tpd_ps_operate;
	//set_bit(CMC_BIT_PS, &obj_ps->enable);
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		TPD_PROXIMITY_DMESG("attach fail = %d\n", err);
//		goto exit_create_attr_failed;
	}
	TPD_PROXIMITY_DMESG("attach ok\n");		
}
#endif

#ifdef TPD_ESD_PROTECT
    INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
    gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
    queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
#endif

    tpd_load_status = 1;
	//tpd_enable_ps(1);
	//device_create_file(&client->dev, &dev_attr_gt968_ps);

    return 0;
}

static void tpd_eint_interrupt_handler(void)
{
    TPD_DEBUG_PRINT_INT;
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
}
static int tpd_i2c_remove(struct i2c_client *client)
{
#ifdef TPD_CREATE_WR_NODE
    uninit_wr_node();
#endif

#ifdef TPD_ESD_PROTECT
    destroy_workqueue(gtp_esd_check_workqueue);
#endif

    return 0;
}

#ifdef TPD_ESD_PROTECT
static void force_reset_guitar(void)
{
    s32 i;
    s32 ret;

    GTP_DEBUG("force_reset_guitar\n");

    //Power off TP
#if (defined(MT6575) || defined(MT6577))
    hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
    msleep(30);
    //Power on TP
    hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
    msleep(30);
#else
    //Power off TP
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
    msleep(30);
    //Power on TP
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
    msleep(30);
#endif

    for (i = 0; i < 5; i++)
    {
        //Reset Guitar
        mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
        msleep(10);
        mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
        msleep(20);

        //Send config
        ret = gtp_send_cfg(i2c_client_point);

        if (ret < 0)
        {
            continue;
        }

        break;
    }

}

static void gtp_esd_check_func(struct work_struct *work)
{
    int i;
    int ret = -1;
    u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

    if (tpd_halt)
    {
        return;
    }

    for (i = 0; i < 3; i++)
    {
        ret = gtp_i2c_read(i2c_client_point, test, 3);
        if (ret > 0)
        {
            break;
        }
    }

    if (i >= 3)
    {
        force_reset_guitar();
    }

    if (!tpd_halt)
    {
        queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
    }

    return;
}
#endif

static void tpd_down(int x, int y, int size, int id)
{
    if ((!size) && (!id))
    {
       // input_report_abs(tpd->dev, ABS_PRESSURE, 1);

        input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
    }
    else
	{
       // input_report_abs(tpd->dev, ABS_PRESSURE, size);
        input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, size);
        /* track id Start 0 */
        input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
    }
    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_mt_sync(tpd->dev);
    TPD_EM_PRINT(x, y, x, y, id - 1, 1);

#if (defined(MT6575)||defined(MT6577))
    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {
        tpd_button(x, y, 1);
    }
#endif

}

static void tpd_up(int x, int y, int id)
{
   // input_report_abs(tpd->dev, ABS_PRESSURE, 0);
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
    input_mt_sync(tpd->dev);
    TPD_EM_PRINT(x, y, x, y, id, 0);

#if (defined(MT6575)||defined(MT6577))
    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {
        tpd_button(x, y, 0);
    }
#endif
}

static int touch_event_handler(void *unused)
{
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
    u8  touch_num = 0;
    u8  finger = 0;
    static u8 pre_touch = 0;
	static u8 pre_key = 0;
    u8  key_value = 0;
    u8* coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i  = 0;
    s32 ret = -1;
#ifdef TPD_PROXIMITY
	int err;
	hwm_sensor_data sensor_data;
	u8 proximity_status;
#endif
    sched_setscheduler(current, SCHED_RR, &param);

    do
    {
        set_current_state(TASK_INTERRUPTIBLE);

        while (tpd_halt)
        {
            tpd_flag = 0;
            msleep(20);
        }

        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
        TPD_DEBUG_SET_TIME;
        set_current_state(TASK_RUNNING);
		
#ifdef TPD_PROXIMITY
		/*added by bernard*/	
			if (tpd_proximity_flag == 1)
			{
				i2c_read_bytes( i2c_client_point, TPD_PROXIMITY_REG, &proximity_status, 1 );
				TPD_PROXIMITY_DEBUG("0x71d's value is 0x%02X\n", proximity_status);
				
				if (((proximity_status&0x20) == 0x20) || ((proximity_status&0x40) == 0x40))
				{
					tpd_proximity_detect = 0;
					//sensor_data.values[0] = 0;
				}
				else
				{
					tpd_proximity_detect = 1;
					//sensor_data.values[0] = 1;
				}
				//get raw data
				
				//map and store data to hwm_sensor_data
				sensor_data.values[0] = tpd_get_ps_value();
				sensor_data.value_divide = 1;
				sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
				//let up layer to know
				if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
				{
					TPD_PROXIMITY_DMESG("call hwmsen_get_interrupt_data fail = %d\n", err);
				}
				TPD_PROXIMITY_DEBUG(" ps change %d\n", sensor_data.values[0]);

			}
		/*end of added*/   
#endif
        ret = gtp_i2c_read(i2c_client_point, point_data, 12);
        if (ret < 0)
        {
            GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
            goto exit_work_func;
        }

        finger = point_data[GTP_ADDR_LENGTH];    
        if((finger & 0x80) == 0)
        {
            goto exit_work_func;
        }

        touch_num = finger & 0x0f;
        if (touch_num > GTP_MAX_TOUCH)
        {
            goto exit_work_func;
        }

        if (touch_num > 1)
        {
            u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};
    
            ret = gtp_i2c_read(i2c_client_point, buf, 2 + 8 * (touch_num - 1)); 
            memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
        }

    #if GTP_HAVE_TOUCH_KEY
        key_value = point_data[3 + 8 * touch_num];
    	
		if(key_value || pre_key)
		{
	        for (i = 0; i < GTP_MAX_KEY_NUM; i++)
	        {
				GTP_DEBUG("input_report_key:%02x, finger:%02x.  touch_num %d", pre_touch, finger, touch_num);
	            input_report_key(tpd->dev, touch_key_array[i], key_value & (0x01<<i));   
	        }
		}
		pre_key = key_value;
    #endif

        GTP_DEBUG("pre_touch:%02x, finger:%02x.  touch_num %d", pre_touch, finger, touch_num);

        if (touch_num && (!key_value) && (!pre_key))
        {
            for (i = 0; i < touch_num; i++)
            {
                coor_data = &point_data[i * 8 + 3];
    
                id = coor_data[0];
                input_x  = coor_data[1] | coor_data[2] << 8;
                input_y  = coor_data[3] | coor_data[4] << 8;
                input_w  = coor_data[5] | coor_data[6] << 8;
    			GTP_DEBUG("id:%d x:%d y:%d w:%d ", id, input_x,input_y,input_w);
                tpd_down(input_x, input_y, input_w, id);
            }
        }
        else if (pre_touch)
        {
            GTP_DEBUG("Touch Release!");
            tpd_up(0, 0, 0);
        }

        pre_touch = touch_num;
    //    input_report_key(tpd->dev, BTN_TOUCH, (touch_num || key_value));
    
        if (tpd != NULL && tpd->dev != NULL)
        {
            input_sync(tpd->dev);
        }

exit_work_func:
        if(!gtp_rawdiff_mode)
        {
            ret = gtp_i2c_write(i2c_client_point, end_cmd, 3);
            if (ret < 0)
            {
                GTP_INFO("I2C write end_cmd  error!"); 
            }
        }
    }
    while (!kthread_should_stop());

    return 0;
}

static int tpd_local_init(void)
{

    if (i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        GTP_INFO("unable to add i2c driver.\n");
        return -1;
    }

    if (tpd_load_status == 0) //if(tpd_load_status == 0) // disable auto load touch driver for linux3.0 porting
    {
        GTP_INFO("add error touch panel driver.\n");
        i2c_del_driver(&tpd_i2c_driver);
        return -1;
    }

#ifdef TPD_HAVE_BUTTON
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif

    // set vendor string
    tpd->dev->id.vendor = (tpd_info.vendor_id_2 << 8) | tpd_info.vendor_id_1;
    tpd->dev->id.product = (tpd_info.product_id_2 << 8) | tpd_info.product_id_1;
    tpd->dev->id.version = (tpd_info.version_2 << 8) | tpd_info.version_1;

    GTP_INFO("end %s, %d\n", __FUNCTION__, __LINE__);
    tpd_type_cap = 1;

    return 0;
}


/*******************************************************
Function:
	Eter sleep function.

Input:
	client:i2c_client.
	
Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_enter_sleep(struct i2c_client * client)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

    GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
    msleep(5);
    while(retry++ < 5)
    {
        ret = gtp_i2c_write(client, i2c_control_buf, 3);
        if (ret > 0)
        {
            GTP_DEBUG("GTP enter sleep!");
            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send sleep cmd failed.");
    return ret;
}

/*******************************************************
Function:
	Wakeup from sleep mode Function.

Input:
	client:i2c_client.
	
Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_wakeup_sleep(struct i2c_client * client)
{
    u8 retry = 0;
    s8 ret = -1;

    GTP_GPIO_AS_INT(GTP_INT_PORT);
    msleep(10);

#if GTP_POWER_CTRL_SLEEP
    while(retry++ < 5)
    {
        gtp_reset_guitar(client, 20);
        ret = gtp_send_cfg(client);
        if (ret > 0)
        {
            GTP_DEBUG("Wakeup sleep send config success.");
            return ret;
        }
    }
#else
    while(retry++ < 10)
    {
        ret = gtp_i2c_test(client);
        if (ret > 0)
        {
            GTP_DEBUG("GTP wakeup sleep.");
            return ret;
        }
        gtp_reset_guitar(client, 20);
    }
#endif

    GTP_ERROR("GTP wakeup sleep failed.");
    return ret;
}
/* Function to manage low power suspend */
static void tpd_suspend(struct early_suspend *h)
{
    s32 ret = -1;

#ifdef TPD_ESD_PROTECT
    cancel_delayed_work_sync(&gtp_esd_check_work);
#endif
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		return ;
	}
#endif
    ret = gtp_enter_sleep(i2c_client_point);
    if (ret < 0)
    {
        GTP_ERROR("GTP early suspend failed.");
    }
     GTP_ERROR("tpd_suspend.");
    tpd_halt = 1;
    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
}

/* Function to manage power-on resume */
static void tpd_resume(struct early_suspend *h)
{
    s32 ret = -1;
    
#ifdef TPD_PROXIMITY
		if (tpd_proximity_flag == 1)
		{
			return ;
		}
#endif
    ret = gtp_wakeup_sleep(i2c_client_point);
    if (ret < 0)
    {
        GTP_ERROR("GTP later resume failed.");
    }

    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    tpd_halt = 0;
	GTP_ERROR("tpd_resume.");

#ifdef TPD_ESD_PROTECT
    queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
#endif

}

static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = "gt818",
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
    GTP_INFO("MediaTek gt91xx touch panel driver init\n");
    i2c_register_board_info(0, &i2c_tpd, 1);

    if (tpd_driver_add(&tpd_device_driver) < 0)
        GTP_INFO("add generic driver failed\n");

    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    GTP_INFO("MediaTek gt91xx touch panel driver exit\n");
    //input_unregister_device(tpd->dev);
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

