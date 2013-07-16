/* drivers/hwmon/mt6516/amit/gt968_ps.c - gt968_ps ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
//#include <mach/mt_gpio.h>
#ifdef MT6516
#include <mach/mt6516_devs.h>
#include <mach/mt6516_typedefs.h>
#include <mach/mt6516_gpio.h>
#include <mach/mt6516_pll.h>
#endif

#ifdef MT6573
#include <mach/mt6573_devs.h>
#include <mach/mt6573_typedefs.h>
#include <mach/mt6573_gpio.h>
#include <mach/mt6573_pll.h>
#endif

#ifdef MT6575
#include <mach/mt6575_devs.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_gpio.h>
#include <mach/mt6575_pm_ldo.h>
#endif
#ifdef MT6577
#include <mach/mt6577_devs.h>
#include <mach/mt6577_typedefs.h>
#include <mach/mt6577_gpio.h>
#include <mach/mt6577_pm_ldo.h>
#endif

#ifdef MT6516
#define POWER_NONE_MACRO MT6516_POWER_NONE
#endif

#ifdef MT6573
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6575
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6577
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

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
#endif
#ifdef TPD_PROXIMITY
#define TPD_PROXIMITY_REG             0x814e 
#define TPD_POWER_MODE_REG            0x8042

static u8 tpd_proximity_flag = 0;
static u8 tpd_proximity_detect = 1;//0-->close ; 1--> far away
#endif

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "gt968_ps.h"
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define gt968_ps_DEV_NAME     "gt968_ps"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_ERR APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_ERR APS_TAG fmt, ##args)                 
/******************************************************************************
 * extern functions
*******************************************************************************/
/*for interrup work mode support --add by liaoxl.lenovo 12.08.2011*/
#ifdef MT6577
	extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
	extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
	extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
	extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif
#ifdef MT6575
	extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
	extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
	extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
	extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
										 kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
										 kal_bool auto_umask);
	
#endif
#ifdef MT6516
extern void MT6516_EINTIRQUnmask(unsigned int line);
extern void MT6516_EINTIRQMask(unsigned int line);
extern void MT6516_EINT_Set_Polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void MT6516_EINT_Set_HW_Debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 MT6516_EINT_Set_Sensitivity(kal_uint8 eintno, kal_bool sens);
extern void MT6516_EINT_Registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif
/*----------------------------------------------------------------------------*/
//static struct i2c_client *gt968_ps_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
//static const struct i2c_device_id gt968_ps_i2c_id[] = {{gt968_ps_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_gt968_ps={ I2C_BOARD_INFO("gt968_ps", (0X72>>1))};
/*the adapter id & i2c address will be available in customization*/
//static unsigned short gt968_ps_force[] = {0x02, 0X72, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const gt968_ps_forces[] = { gt968_ps_force, NULL };
//static struct i2c_client_address_data gt968_ps_addr_data = { .forces = gt968_ps_forces,};
/*----------------------------------------------------------------------------*/

static struct gt968_ps_priv *g_gt968_ps_ptr = NULL;

 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct gt968_ps_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
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


static struct gt968_ps_priv *gt968_ps_obj = NULL;

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 0,
	.polling_mode_ps =0,
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    .i2c_addr   = {0x72, 0x48, 0x78, 0x00},
    .ps_threshold_high = 800,
    .ps_threshold_low = 714,
    .ps_threshold = 900,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

extern int tpd_enable_ps(int enable);
extern int tpd_get_ps_value(void);
extern int hwmsen_alsps_sensor_add(struct sensor_init_info* obj) ;


/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int gt968_ps_open(struct inode *inode, struct file *file)
{
	//file->private_data = gt968_ps_i2c_client;

	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int gt968_ps_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}



/*----------------------------------------------------------------------------*/
static long gt968_ps_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	//struct PS_CALI_DATA_STRUCT ps_cali_temp;
	struct gt968_ps_priv *obj = gt968_ps_obj;
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = tpd_enable_ps(1)))
				{
					APS_ERR("enable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if((err = tpd_enable_ps(0)))
				{
					APS_ERR("disable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    

			dat = tpd_get_ps_value();
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;
/*
		case ALSPS_GET_PS_RAW_DATA:    
			if((err = gt968_ps_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              
*/
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations gt968_ps_fops = {
	.owner = THIS_MODULE,
	.open = gt968_ps_open,
	.release = gt968_ps_release,
	.unlocked_ioctl = gt968_ps_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice gt968_ps_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &gt968_ps_fops,
};
/*----------------------------------------------------------------------------*/

int gt968_ps_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
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

/*----------------------------------------------------------------------------*/
static int gt968_ps_probe(void) 
{
	struct gt968_ps_priv *obj;
	struct hwmsen_object obj_ps;
	int err = 0;
	APS_FUN();
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	gt968_ps_obj = obj;

	obj->hw = get_cust_alsps_hw();

	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
//	INIT_WORK(&obj->eint_work, gt968_ps_eint_work);
//	obj->client = client;
//	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->ps_debounce, 10);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	obj->enable = 0;
	obj->pending_intr = 0;
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_PS, &obj->enable);


	if((err = misc_register(&gt968_ps_device)))
	{
		APS_ERR("gt968_ps_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	obj_ps.self = gt968_ps_obj;
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(1 == obj->hw->polling_mode_ps)
	//if(1)
	{
		obj_ps.polling = 1;
	}
	else
	{
		obj_ps.polling = 0;
	}

	obj_ps.sensor_operate = gt968_ps_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	

	APS_LOG("%s: OK\n", __func__);
	//tpd_enable_ps(1);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&gt968_ps_device);
	exit_misc_device_register_failed:
	
	//i2c_detach_client(client);
	//exit_kfree:
	kfree(obj);
	exit:
//	gt968_ps_i2c_client = NULL; 		  
//	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);	/*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}

/*----------------------------------------------------------------------------*/
static int gt968_ps_remove(void)
{
	int err;
	APS_FUN();	
/*	
	if(err = gt968_ps_delete_attr(&gt968_ps_i2c_driver.driver))
	{
		APS_ERR("gt968_ps_delete_attr fail: %d\n", err);
	} 
*/
	if((err = misc_deregister(&gt968_ps_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	

	return 0;
}
static int  gt968_ps_local_init(void)
{
	int err = 0;
	err = gt968_ps_probe();

	if(err != 0)		
	{	   		
		
		return -1;		
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static struct sensor_init_info gt968_ps_init_info = {			
	.name = "gt968_ps",			
	.init = gt968_ps_local_init,			
	.uninit =gt968_ps_remove,	
};


static int __init gt968_ps_init(void)
{
	APS_FUN();
	hwmsen_alsps_sensor_add(&gt968_ps_init_info);	

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit gt968_ps_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(gt968_ps_init);
module_exit(gt968_ps_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Huangzs");
MODULE_DESCRIPTION("gt968_ps driver");
MODULE_LICENSE("GPL");
