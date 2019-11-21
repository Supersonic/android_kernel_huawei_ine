/*
 * Copyright (c) 2013 Linaro Ltd.
 * Copyright (c) 2013 Hisilicon Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch"
/*lint --e{750}*/
#include "dw_mmc_hisi.h"
#include <linux/mmc/dw_mmc_mux_sdsim.h>


#include <linux/bootdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/host.h>
#include <linux/mmc/dw_mmc.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>
#include <linux/clk-provider.h>
#include <linux/hisi/util.h>
#include <linux/hwspinlock.h>

#include "dw_mmc.h"
#include "dw_mmc-pltfm.h"

struct semaphore sem_mux_sdsim_detect;

extern int dw_mci_check_himntn(int feature);

/*
sd_sim_group_io

NUMBER            FUNCTION0        FUNCTION1      FUNCTION4
GPIO160              GPIO               SD_CLK           USIM1_CLK
GPIO161              GPIO               SD_CMD          -
GPIO162              GPIO               SD_DATA0       USIM1_RST
GPIO163              GPIO               SD_DATA1       USIM1_DATA
GPIO164              GPIO               SD_DATA2       -
GPIO165              GPIO               SD_DATA3       -
*/

#define GPIO_160 160
#define GPIO_161 161
#define GPIO_162 162
#define GPIO_163 163
#define GPIO_164 164
#define GPIO_165 165
#define GPIO_171 171


int sd_sim_detect_status_current = SD_SIM_DETECT_STATUS_UNDETECTED;



#define IOMG_BASE_GPIO_160 0xff37e000
#define IOCG_BASE_GPIO_160 0xff37e800

#define FUNCTION0 0
#define FUNCTION1 1
#define FUNCTION2 2
#define FUNCTION3 3
#define FUNCTION4 4

#define PULL_TYPE_NP 0x00
#define PULL_TYPE_PU 0x01
#define PULL_TYPE_PD 0x02


int sd_clk_driver_strength = -1;
int sd_cmd_driver_strength = -1;
int sd_data_driver_strength = -1;

int set_sd_sim_group_io_register(int gpionumber,int function_select,int pulltype,int driverstrenth)
{
	int reg = 0;

	if(gpionumber < GPIO_160 || gpionumber > GPIO_165)
	{
		printk("%s %s argument gpionumber=%d is invalid,just return.\n",MUX_SDSIM_LOG_TAG,__func__,gpionumber);
		return 0;
	}

	writel(function_select, ioremap(IOMG_BASE_GPIO_160, 0x1000) + 0x004 *(gpionumber - GPIO_160) );

	reg = readl(ioremap(IOCG_BASE_GPIO_160, 0x1000) + 0x004 *(gpionumber - GPIO_160));
	reg = reg & 0x08;
	reg = reg | driverstrenth;
	reg = reg | pulltype;
	writel(reg, ioremap(IOCG_BASE_GPIO_160, 0x1000) + 0x004 *(gpionumber - GPIO_160));

	return 0;
}




/*
GpioMode/GpioNum                           Func     PullType        Driver           SlewRate           GpioDirection           OutputValue

SDSIM_MODE_GPIO_DETECT
GPIO_160                                    Function0      NP              25mA-1           -                             -                             -
GPIO_161                                    Function0      PU              2mA-0            N                             I                              -          (detect pin)
GPIO_162                                    Function0      PD              2mA-0            N                             I                              -          (detect pin)
GPIO_163                                    Function0      NP              11mA-1           -                             -                             -
GPIO_164                                    Function0      PD              2mA-0            N                             I                              -          (detect pin)
GPIO_165                                    Function0      PU              2mA-0            N                             I                              -          (detect pin)


SDSIM_MODE_SD_NORMAL
GPIO_160                                    Function1      NP              25mA-1           -                             -                             -
GPIO_161                                    Function1      PU              11mA-1           -                             -                             -
GPIO_162                                    Function1      PU              11mA-1           -                             -                             -
GPIO_163                                    Function1      PU              11mA-1           -                             -                             -
GPIO_164                                    Function1      PU              11mA-1           -                             -                             -
GPIO_165                                    Function1      PU              11mA-1           -                             -                             -


SDSIM_MODE_SD_IDLE
GPIO_160                                    Function0      NP              25mA-1           -                             -                             -
GPIO_161                                    Function0      NP              11mA-1           -                             -                             -
GPIO_162                                    Function0      NP              11mA-1           -                             -                             -
GPIO_163                                    Function0      NP              11mA-1           -                             -                             -
GPIO_164                                    Function0      NP              11mA-1           -                             -                             -
GPIO_165                                    Function0      NP              11mA-1           -                             -                             -


SDSIM_MODE_SIM_NORMAL
GPIO_160                                    Function4      NP              4mA-0             N                             -                             -
GPIO_161                                    Function0      NP              11mA-1           -                             -                             -           (sim not use this)
GPIO_162                                    Function4      NP              4mA-0             N                             -                             -
GPIO_163                                    Function4      NP              4mA-0             N                             -                             -
GPIO_164                                    Function0      NP              11mA-1           -                             -                             -           (sim not use this)
GPIO_165                                    Function0      NP              11mA-1           -                             -                             -           (sim not use this)



SDSIM_MODE_SIM_IDLE
GPIO_160                                    Function4      NP              4mA-0             N                             -                             -
GPIO_161                                    Function0      NP              11mA-1           -                             -                              -           (sim not use this)
GPIO_162                                    Function4      NP              4mA-0             N                             -                             -
GPIO_163                                    Function4      NP              4mA-0             N                             -                             -
GPIO_164                                    Function0      NP              11mA-1           -                             -                              -           (sim not use this)
GPIO_165                                    Function0      NP              11mA-1           -                             -                              -           (sim not use this)


*/

int config_sdsim_gpio_mode(enum sdsim_gpio_mode gpio_mode)
{
	if( -1 == sd_clk_driver_strength )
	{
		sd_clk_driver_strength = SD_CLK_DRIVER_DEFAULT;
		printk("%s %s sd_clk_driver_strength set as default = 0x%x.\n",MUX_SDSIM_LOG_TAG,__func__,sd_clk_driver_strength);
	}

	if( -1 == sd_cmd_driver_strength )
	{
		sd_cmd_driver_strength = SD_CMD_DRIVER_DEFAULT;
		printk("%s %s sd_cmd_driver_strength set as default = 0x%x.\n",MUX_SDSIM_LOG_TAG,__func__,sd_cmd_driver_strength);
	}

	if( -1 == sd_data_driver_strength )
	{
		sd_data_driver_strength = SD_DATA_DRIVER_DEFAULT;
		printk("%s %s sd_data_driver_strength set as default = 0x%x.\n",MUX_SDSIM_LOG_TAG,__func__,sd_data_driver_strength);
	}


	if(SDSIM_MODE_GPIO_DETECT == gpio_mode)
	{
		printk("%s %s set SDSIM_MODE_GPIO_DETECT.\n",MUX_SDSIM_LOG_TAG,__func__);
		set_sd_sim_group_io_register(GPIO_160, FUNCTION0, PULL_TYPE_NP, DRIVER_STRENGTH_2MA_0 );
		set_sd_sim_group_io_register(GPIO_161, FUNCTION0, PULL_TYPE_PU, DRIVER_STRENGTH_2MA_0 );         //detect pin
		set_sd_sim_group_io_register(GPIO_162, FUNCTION0, PULL_TYPE_PD, DRIVER_STRENGTH_2MA_0 );         //detect pin
		set_sd_sim_group_io_register(GPIO_163, FUNCTION0, PULL_TYPE_NP, DRIVER_STRENGTH_2MA_0 );
		set_sd_sim_group_io_register(GPIO_164, FUNCTION0, PULL_TYPE_PD, DRIVER_STRENGTH_2MA_0 );         //detect pin
		set_sd_sim_group_io_register(GPIO_165, FUNCTION0, PULL_TYPE_PU, DRIVER_STRENGTH_2MA_0 );         //detect pin

	}
	else if(SDSIM_MODE_SD_NORMAL == gpio_mode)
	{
		printk("%s %s set SDSIM_MODE_SD_NORMAL.\n",MUX_SDSIM_LOG_TAG,__func__);
		set_sd_sim_group_io_register(GPIO_160, FUNCTION1, PULL_TYPE_NP, sd_clk_driver_strength );
		set_sd_sim_group_io_register(GPIO_161, FUNCTION1, PULL_TYPE_PU, sd_cmd_driver_strength );
		set_sd_sim_group_io_register(GPIO_162, FUNCTION1, PULL_TYPE_PU, sd_data_driver_strength );
		set_sd_sim_group_io_register(GPIO_163, FUNCTION1, PULL_TYPE_PU, sd_data_driver_strength );
		set_sd_sim_group_io_register(GPIO_164, FUNCTION1, PULL_TYPE_PU, sd_data_driver_strength );
		set_sd_sim_group_io_register(GPIO_165, FUNCTION1, PULL_TYPE_PU, sd_data_driver_strength );

	}
	else if(SDSIM_MODE_SD_IDLE == gpio_mode)
	{
		printk("%s %s set SDSIM_MODE_SD_IDLE.\n",MUX_SDSIM_LOG_TAG,__func__);
		set_sd_sim_group_io_register(GPIO_160, FUNCTION0, PULL_TYPE_NP, sd_clk_driver_strength );
		set_sd_sim_group_io_register(GPIO_161, FUNCTION0, PULL_TYPE_NP, sd_cmd_driver_strength );
		set_sd_sim_group_io_register(GPIO_162, FUNCTION0, PULL_TYPE_NP, sd_data_driver_strength );
		set_sd_sim_group_io_register(GPIO_163, FUNCTION0, PULL_TYPE_NP, sd_data_driver_strength );
		set_sd_sim_group_io_register(GPIO_164, FUNCTION0, PULL_TYPE_NP, sd_data_driver_strength );
		set_sd_sim_group_io_register(GPIO_165, FUNCTION0, PULL_TYPE_NP, sd_data_driver_strength );

	}
	else if(SDSIM_MODE_SIM_NORMAL == gpio_mode)
	{
		printk("%s %s set SDSIM_MODE_SIM_NORMAL.\n",MUX_SDSIM_LOG_TAG,__func__);
		set_sd_sim_group_io_register(GPIO_160, FUNCTION4, PULL_TYPE_NP, DRIVER_STRENGTH_4MA_0 );
		set_sd_sim_group_io_register(GPIO_161, FUNCTION0, PULL_TYPE_NP, DRIVER_STRENGTH_4MA_0 );
		set_sd_sim_group_io_register(GPIO_162, FUNCTION4, PULL_TYPE_NP, DRIVER_STRENGTH_4MA_0 );
		set_sd_sim_group_io_register(GPIO_163, FUNCTION4, PULL_TYPE_NP, DRIVER_STRENGTH_4MA_0 );
		set_sd_sim_group_io_register(GPIO_164, FUNCTION0, PULL_TYPE_NP, DRIVER_STRENGTH_4MA_0 );
		set_sd_sim_group_io_register(GPIO_165, FUNCTION0, PULL_TYPE_NP, DRIVER_STRENGTH_4MA_0 );

	}
	else if(SDSIM_MODE_SIM_IDLE == gpio_mode)
	{
		printk("%s %s set SDSIM_MODE_SIM_IDLE.\n",MUX_SDSIM_LOG_TAG,__func__);
		set_sd_sim_group_io_register(GPIO_160, FUNCTION4, PULL_TYPE_NP, DRIVER_STRENGTH_4MA_0 );
		set_sd_sim_group_io_register(GPIO_161, FUNCTION0, PULL_TYPE_NP, DRIVER_STRENGTH_4MA_0 );
		set_sd_sim_group_io_register(GPIO_162, FUNCTION4, PULL_TYPE_NP, DRIVER_STRENGTH_4MA_0 );
		set_sd_sim_group_io_register(GPIO_163, FUNCTION4, PULL_TYPE_NP, DRIVER_STRENGTH_4MA_0 );
		set_sd_sim_group_io_register(GPIO_164, FUNCTION0, PULL_TYPE_NP, DRIVER_STRENGTH_4MA_0 );
		set_sd_sim_group_io_register(GPIO_165, FUNCTION0, PULL_TYPE_NP, DRIVER_STRENGTH_4MA_0 );

	}

	return 0;

}




typedef struct detect_gpio_status_array
{
	int sd_data0_and_sim1_rst_value; //vcc based GPIO162
	int sd_data2_value; //vcc based GPIO164

	int sd_cmd_and_sim1_vpp_value; //gnd based GPIO161
	int sd_data3_value;  //gnd based GPIO165

	int actual_card_type; //sim or sd
}detect_gpio_status_array_s;


static detect_gpio_status_array_s g_array[]=
{
	{GPIO_VALUE_LOW,  GPIO_VALUE_HIGH,  GPIO_VALUE_HIGH,   GPIO_VALUE_LOW,    SD_SIM_DETECT_STATUS_SIM },  // status type 1  //NORMAL_SIM
	{GPIO_VALUE_LOW,  GPIO_VALUE_LOW,   GPIO_VALUE_HIGH,   GPIO_VALUE_HIGH,   SD_SIM_DETECT_STATUS_SD  },  // status type 2  //NORMAL_SD
	{GPIO_VALUE_LOW,  GPIO_VALUE_LOW,   GPIO_VALUE_HIGH,   GPIO_VALUE_HIGH,   SD_SIM_DETECT_STATUS_SD  },  // status type 2  //SLOT_EMPTY,same status with NORMAL_SD,set as SD_SIM_DETECT_STATUS_SD
	{GPIO_VALUE_HIGH, GPIO_VALUE_LOW,   GPIO_VALUE_LOW,    GPIO_VALUE_HIGH,   SD_SIM_DETECT_STATUS_SIM },  // status type 3  //ERROR_1_SIM
	{GPIO_VALUE_LOW,  GPIO_VALUE_LOW,   GPIO_VALUE_HIGH,   GPIO_VALUE_HIGH,   SD_SIM_DETECT_STATUS_SD  },  // status type 2  //ERROR_1_SD
	{GPIO_VALUE_HIGH, GPIO_VALUE_HIGH,  GPIO_VALUE_LOW,    GPIO_VALUE_LOW,    SD_SIM_DETECT_STATUS_SIM },  // status type 4  //ERROR_2_1_SIM
	{GPIO_VALUE_LOW,  GPIO_VALUE_LOW,   GPIO_VALUE_HIGH,   GPIO_VALUE_HIGH,   SD_SIM_DETECT_STATUS_SD  },  // status type 2  //ERROR_2_1_SD
	{GPIO_VALUE_LOW,  GPIO_VALUE_LOW,   GPIO_VALUE_HIGH,   GPIO_VALUE_HIGH,   SD_SIM_DETECT_STATUS_SIM },  // status type 2  //ERROR_2_2_SIM
	{GPIO_VALUE_LOW,  GPIO_VALUE_LOW,   GPIO_VALUE_HIGH,   GPIO_VALUE_HIGH,   SD_SIM_DETECT_STATUS_SD  },  // status type 2  //ERROR_2_2_SD
	{GPIO_VALUE_LOW,  GPIO_VALUE_HIGH,  GPIO_VALUE_HIGH,   GPIO_VALUE_HIGH,   SD_SIM_DETECT_STATUS_SIM  }, // status type 5  //NORMAL_SIM for another card slot
};

char* detect_status_to_string(void)
{
    switch (sd_sim_detect_status_current)
    {
    case SD_SIM_DETECT_STATUS_UNDETECTED:
        return "DETECT_STATUS_UNDETECTED";

    case SD_SIM_DETECT_STATUS_SD:
        return "DETECT_STATUS_SD";

    case SD_SIM_DETECT_STATUS_SIM:
        return "DETECT_STATUS_SIM";

    case SD_SIM_DETECT_STATUS_ERROR:
    default:
        return "DETECT_STATUS_ERROR";
    }
}

int sd_sim_detect_run(void *dw_mci_host, int status, int current_module, int need_sleep)
{
	int ret = 0;
	int i = 0;

	int sd_data0_and_sim1_rst_value = 0;
	int sd_data2_value = 0;

	int sd_cmd_and_sim1_vpp_value = 0;
	int sd_data3_value = 0;

	struct dw_mci *host = NULL;

	down(&sem_mux_sdsim_detect);

	printk("%s %s version 2.4.\n",MUX_SDSIM_LOG_TAG,__func__);
	printk("%s %s argument list: status = %d current_module = %d sd_sim_detect_status_current = %d need_sleep = %d\n",
		MUX_SDSIM_LOG_TAG,__func__,status,current_module,sd_sim_detect_status_current,need_sleep);

	if(NULL == dw_mci_host)
	{
		printk("%s %s argument list: dw_mci_host == NULL.\n",MUX_SDSIM_LOG_TAG,__func__);
	}
	else
	{
		printk("%s %s argument list: dw_mci_host != NULL.\n",MUX_SDSIM_LOG_TAG,__func__);
	}

	if(MODULE_SD == current_module)
	{
		if(NULL == dw_mci_host)
		{
			printk("%s %s MODULE_SD call but dw_mci_host = NULL,This is Error,just return STATUS_PLUG_OUT.\n",MUX_SDSIM_LOG_TAG,__func__);
			up(&sem_mux_sdsim_detect);
			return STATUS_PLUG_OUT;
		}
		else
		{
			struct dw_mci *host_temp = NULL;
			struct dw_mci_hs_priv_data *priv_temp = NULL;

			host_temp = dw_mci_host;
			priv_temp = host_temp->priv;

			printk("%s %s host_temp->hw_mmc_id =%d,priv_temp->mux_sdsim=%d\n",MUX_SDSIM_LOG_TAG,__func__,host_temp->hw_mmc_id,priv_temp->mux_sdsim);

			/*for those non DWMMC_SD_ID device or mux_sdsim not enabled,just return former status value*/
			if( host_temp->hw_mmc_id != DWMMC_SD_ID )
			{
				printk("%s %s for those non DWMMC_SD_ID device,just return former status value.\n",MUX_SDSIM_LOG_TAG,__func__);
				up(&sem_mux_sdsim_detect);
				return status;
			}

			if( !priv_temp->mux_sdsim )
			{
				printk("%s %s for device mux_sdsim not enabled,just return former status value.\n",MUX_SDSIM_LOG_TAG,__func__);
				up(&sem_mux_sdsim_detect);
				return status;
			}

			/*SD device that is ok,update host_from_sd_module*/
			host_from_sd_module = host_temp;

		}
	}
	else if(MODULE_SIM == current_module)
	{
		/*to do something*/
	}


	host = host_from_sd_module;


	if(!host)
	{
		printk("%s %s host=NULL,current_module=%d,This is Error,maybe MODULE_SIM,just return STATUS_PLUG_IN and let SIM go on.\n",MUX_SDSIM_LOG_TAG,__func__,current_module);
		/*This is sim*/
		up(&sem_mux_sdsim_detect);
		return STATUS_PLUG_IN;
	}

	if(STATUS_PLUG_OUT == status)
	{
		/*For plug out event,just update sd_sim_detect_status_current here*/
		if(SD_SIM_DETECT_STATUS_UNDETECTED != sd_sim_detect_status_current)
		{
			config_sdsim_gpio_mode(SDSIM_MODE_GPIO_DETECT);
			printk("%s %s For plug out event,sd_sim_detect_status_current=%d,now is not SD_SIM_DETECT_STATUS_UNDETECTED,so config as SDSIM_MODE_GPIO_DETECT mode for lowpower.\n",
				MUX_SDSIM_LOG_TAG,__func__,sd_sim_detect_status_current);
		}

		sd_sim_detect_status_current = SD_SIM_DETECT_STATUS_UNDETECTED;
		printk("%s %s For plug out event,update sd_sim_detect_status_current here.\n",MUX_SDSIM_LOG_TAG,__func__);

		if(dw_mci_check_himntn(HIMNTN_SD2JTAG) || dw_mci_check_himntn(HIMNTN_SD2DJTAG))
		{
			/*SD2JTAG enable, set GPIO171 HIGH*/
			(void)gpio_request(GPIO_171, "sd_cmd_and_sim1_vpp_switch_gpio");
			gpio_direction_output(GPIO_171,1);
			gpio_set_value(GPIO_171, 1);
			gpio_free(GPIO_171);
		}
		else
		{
			/*resume to SIM mode for NFC*/
			(void)gpio_request(GPIO_171, "sd_cmd_and_sim1_vpp_switch_gpio");
			gpio_direction_output(GPIO_171,0);
			gpio_set_value(GPIO_171, 0);
			gpio_free(GPIO_171);
		}

		up(&sem_mux_sdsim_detect);
		return STATUS_PLUG_OUT;
	}
	else if(STATUS_PLUG_IN == status)
	{
		if( SD_SIM_DETECT_STATUS_SIM == sd_sim_detect_status_current && MODULE_SD == current_module )
		{
			printk("%s %s MODULE_SD find SIM card already detected,just return STATUS_PLUG_OUT and do nothing.\n",MUX_SDSIM_LOG_TAG,__func__);
			up(&sem_mux_sdsim_detect);
			return STATUS_PLUG_OUT;
		}
		else if( SD_SIM_DETECT_STATUS_SD == sd_sim_detect_status_current && MODULE_SIM == current_module )
		{
			printk("%s %s MODULE_SIM find SD card already detected,just return STATUS_PLUG_OUT and do nothing.\n",MUX_SDSIM_LOG_TAG,__func__);
			up(&sem_mux_sdsim_detect);
			return STATUS_PLUG_OUT;
		}
		else if( (SD_SIM_DETECT_STATUS_UNDETECTED == sd_sim_detect_status_current) || (SD_SIM_DETECT_STATUS_ERROR == sd_sim_detect_status_current) )
		{
			if(host)
			{
				if (host->vqmmc) {
					ret = regulator_set_voltage(host->vqmmc, 1800000, 1800000);
					printk("%s %s vqmmc set 1.8v ret=%d\n",MUX_SDSIM_LOG_TAG,__func__,ret);
					ret = regulator_enable(host->vqmmc);
					printk("%s %s vqmmc enable ret=%d\n",MUX_SDSIM_LOG_TAG,__func__,ret);
					usleep_range(1000, 1500);
				}

				if (host->vmmc) {
					ret = regulator_set_voltage(host->vmmc, 1800000, 1800000);
					printk("%s %s vmmc set 1.8v ret=%d\n",MUX_SDSIM_LOG_TAG,__func__,ret);
					ret = regulator_enable(host->vmmc);
					printk("%s %s vmmc enable ret=%d\n",MUX_SDSIM_LOG_TAG,__func__,ret);
					usleep_range(1000, 1500);
				}
			}
			else
			{
				/*now sim module just use mmc host LDO config interface,and host_from_sd_module*/
			}

			config_sdsim_gpio_mode(SDSIM_MODE_GPIO_DETECT);

			(void)gpio_request(GPIO_161, "sd_cmd_and_sim1_vpp");
			(void)gpio_request(GPIO_162, "sd_data0_and_sim1_rst");
			(void)gpio_request(GPIO_164, "sd_data2");
			(void)gpio_request(GPIO_165, "sd_data3");
			(void)gpio_request(GPIO_171, "sd_cmd_and_sim1_vpp_switch_gpio");

			gpio_direction_output(GPIO_171,1);
			gpio_set_value(GPIO_171, 1);

			gpio_direction_input(GPIO_161);
			gpio_direction_input(GPIO_162);
			gpio_direction_input(GPIO_164);
			gpio_direction_input(GPIO_165);

			gpio_free(GPIO_161);
			gpio_free(GPIO_162);
			gpio_free(GPIO_164);
			gpio_free(GPIO_165);
			gpio_free(GPIO_171);

			usleep_range(1000, 1500);

			sd_cmd_and_sim1_vpp_value = gpio_get_value(GPIO_161);
			sd_data0_and_sim1_rst_value = gpio_get_value(GPIO_162);
			sd_data2_value = gpio_get_value(GPIO_164);
			sd_data3_value = gpio_get_value(GPIO_165);


			printk("%s %s <GPIO %d> sd_data0_and_sim1_rst_value=%d\n",MUX_SDSIM_LOG_TAG,__func__,GPIO_162,sd_data0_and_sim1_rst_value);
			printk("%s %s <GPIO %d> sd_data2_value=%d\n",MUX_SDSIM_LOG_TAG,__func__,GPIO_164,sd_data2_value);
			printk("%s %s <GPIO %d> sd_cmd_and_sim1_vpp_value=%d\n",MUX_SDSIM_LOG_TAG,__func__,GPIO_161,sd_cmd_and_sim1_vpp_value);
			printk("%s %s <GPIO %d> sd_data3_value=%d\n",MUX_SDSIM_LOG_TAG,__func__,GPIO_165,sd_data3_value);

			if (need_sleep > 0)
			{
				need_sleep = (need_sleep > 1000) ? 1000 : need_sleep; //max to 1s sleep time
				msleep(need_sleep);
			}
			printk("%s %s RE READ GPIO status after sleep %d ms\n",MUX_SDSIM_LOG_TAG,__func__,need_sleep);

			sd_cmd_and_sim1_vpp_value = gpio_get_value(GPIO_161);
			sd_data0_and_sim1_rst_value = gpio_get_value(GPIO_162);
			sd_data2_value = gpio_get_value(GPIO_164);
			sd_data3_value = gpio_get_value(GPIO_165);


			printk("%s %s <GPIO %d> sd_data0_and_sim1_rst_value=%d\n",MUX_SDSIM_LOG_TAG,__func__,GPIO_162,sd_data0_and_sim1_rst_value);
			printk("%s %s <GPIO %d> sd_data2_value=%d\n",MUX_SDSIM_LOG_TAG,__func__,GPIO_164,sd_data2_value);
			printk("%s %s <GPIO %d> sd_cmd_and_sim1_vpp_value=%d\n",MUX_SDSIM_LOG_TAG,__func__,GPIO_161,sd_cmd_and_sim1_vpp_value);
			printk("%s %s <GPIO %d> sd_data3_value=%d\n",MUX_SDSIM_LOG_TAG,__func__,GPIO_165,sd_data3_value);

			ret = regulator_disable(host->vqmmc);
			printk("%s %s vqmmc regulator disable ret=%d\n",MUX_SDSIM_LOG_TAG,__func__,ret);
			ret = regulator_disable(host->vmmc);
			printk("%s %s vmmc regulator disable ret=%d\n",MUX_SDSIM_LOG_TAG,__func__,ret);


			for(i=0; i < (int)(sizeof(g_array)/sizeof(detect_gpio_status_array_s)); i++)
			{
				if( (sd_cmd_and_sim1_vpp_value == g_array[i].sd_cmd_and_sim1_vpp_value)
					&& (sd_data0_and_sim1_rst_value == g_array[i].sd_data0_and_sim1_rst_value)
					&& (sd_data2_value == g_array[i].sd_data2_value)
					&& (sd_data3_value == g_array[i].sd_data3_value)
				)
				{
					printk("%s %s GPIO_STATUS_LIST compare ok when i = %d\n",MUX_SDSIM_LOG_TAG,__func__,i);

					if(SD_SIM_DETECT_STATUS_SIM == g_array[i].actual_card_type)
					{
						(void)gpio_request(GPIO_171, "sd_cmd_and_sim1_vpp_switch_gpio");
						gpio_direction_output(GPIO_171,0);
						gpio_set_value(GPIO_171, 0);
						gpio_free(GPIO_171);


						config_sdsim_gpio_mode(SDSIM_MODE_SIM_NORMAL);

						sd_sim_detect_status_current = SD_SIM_DETECT_STATUS_SIM;

						printk("%s %s SIM is inserted and detected now,go with SIM.\n",MUX_SDSIM_LOG_TAG,__func__);

						if(current_module == MODULE_SD)
						{
							up(&sem_mux_sdsim_detect);
							return STATUS_PLUG_OUT;
						}
						else if(current_module == MODULE_SIM)
						{
							up(&sem_mux_sdsim_detect);
							return STATUS_PLUG_IN;
						}

					}
					else if(SD_SIM_DETECT_STATUS_SD == g_array[i].actual_card_type)
					{

						(void)gpio_request(GPIO_171, "sd_cmd_and_sim1_vpp_switch_gpio");
						gpio_direction_output(GPIO_171,1);
						gpio_set_value(GPIO_171, 1);
						gpio_free(GPIO_171);

						sd_sim_detect_status_current = SD_SIM_DETECT_STATUS_SD;

						printk("%s %s SD is inserted and detected now,go with SD.\n",MUX_SDSIM_LOG_TAG,__func__);

						if(current_module == MODULE_SD)
						{
							up(&sem_mux_sdsim_detect);
							return STATUS_PLUG_IN;
						}
						else if(current_module == MODULE_SIM)
						{
							up(&sem_mux_sdsim_detect);
							return STATUS_PLUG_OUT;
						}
					}
					break;
				}

			}

			if (i >= (int)(sizeof(g_array)/sizeof(detect_gpio_status_array_s)))
			{
				(void)gpio_request(GPIO_171, "sd_cmd_and_sim1_vpp_switch_gpio");
				gpio_direction_output(GPIO_171,0);
				gpio_set_value(GPIO_171, 0);
				gpio_free(GPIO_171);

				printk("%s %s SD_SIM_DETECT_STATUS_ERROR detected,but just go with SIM.\n", MUX_SDSIM_LOG_TAG, __func__);
				config_sdsim_gpio_mode(SDSIM_MODE_SIM_NORMAL);
				sd_sim_detect_status_current = SD_SIM_DETECT_STATUS_SIM;


				if(current_module == MODULE_SD)
				{
					up(&sem_mux_sdsim_detect);
					return STATUS_PLUG_OUT;
				}
				else
				{
					up(&sem_mux_sdsim_detect);
					return STATUS_PLUG_IN;
				}

			}
			else
			{
				up(&sem_mux_sdsim_detect);
				return STATUS_PLUG_IN;
			}

		}
		else
		{
			printk("%s %s current_module=%d now re-entered but same detect status,just return STATUS_PLUG_IN and do nothing.\n",MUX_SDSIM_LOG_TAG,__func__,current_module);
		}

		up(&sem_mux_sdsim_detect);
		return STATUS_PLUG_IN;

	}
	else
	{
		printk("%s %s input argument status=%d is not correct,just return STATUS_PLUG_OUT.\n",MUX_SDSIM_LOG_TAG,__func__,status);
		up(&sem_mux_sdsim_detect);
		return STATUS_PLUG_OUT;
	}

}


#ifdef CONFIG_HISI_BALONG_MODEM
extern void notify_card1_repower(void);
#endif
void notify_sim_while_sd_fail(struct mmc_host *mmc)
{
	struct dw_mci_slot *slot = NULL;
	struct dw_mci *host = NULL;
	struct dw_mci_hs_priv_data *priv_data = NULL;

	down(&sem_mux_sdsim_detect);

	slot = mmc_priv(mmc);
	host = slot->host;
	priv_data = host->priv;

	printk("%s %s enter.\n",MUX_SDSIM_LOG_TAG,__func__);

	if( host->hw_mmc_id != DWMMC_SD_ID )
	{
		printk("%s %s for those non DWMMC_SD_ID device,just return.\n",MUX_SDSIM_LOG_TAG,__func__);
		up(&sem_mux_sdsim_detect);
		return;
	}

	if( !priv_data->mux_sdsim )
	{
		printk("%s %s for device mux_sdsim not enabled,just return.\n",MUX_SDSIM_LOG_TAG,__func__);
		up(&sem_mux_sdsim_detect);
		return;
	}

	if(SD_SIM_DETECT_STATUS_SD == sd_sim_detect_status_current)
	{
		(void)gpio_request(GPIO_171, "sd_cmd_and_sim1_vpp_switch_gpio");
		gpio_direction_output(GPIO_171,0);
		gpio_set_value(GPIO_171, 0);
		gpio_free(GPIO_171);

		printk("%s %s SD card init fail now,retry go with SIM.\n",MUX_SDSIM_LOG_TAG,__func__);

		config_sdsim_gpio_mode(SDSIM_MODE_SIM_NORMAL);

		sd_sim_detect_status_current = SD_SIM_DETECT_STATUS_SIM;


#ifdef CONFIG_HISI_BALONG_MODEM
		//notify sim card to repower now
		notify_card1_repower();
#endif

	}
	else
	{
		printk("%s %s SD card init fail now,sd_sim_detect_status_current = %d but not SD_SIM_DETECT_STATUS_SD,no need retry with SIM,just return.\n",MUX_SDSIM_LOG_TAG,__func__,sd_sim_detect_status_current);
	}

	up(&sem_mux_sdsim_detect);
	return;
}



#pragma GCC diagnostic pop
