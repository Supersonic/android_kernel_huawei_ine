#ifndef __HISI_HISEE_CHIP_TEST_H__
#define __HISI_HISEE_CHIP_TEST_H__

#ifdef CONFIG_HISEE_SUPPORT_MULTI_COS
/*get key from HISEE*/
#define KEY_NOT_READY 0
#define KEY_READY     1
#define KEY_REQ_FAILED 2
#define SINGLE_RPMBKEY_NUM 1

#ifdef CONFIG_HISEE_SUPPORT_8_COS
#define COS_FLASH_IMG_ID     COS_IMG_ID_5
#else
#define COS_FLASH_IMG_ID     COS_IMG_ID_3
#endif

#endif

int hisee_parallel_manufacture_func(void *buf, int para);
#ifdef CONFIG_HISI_HISEE_NVMFORMAT_TEST
int hisee_nvmformat_func(void *buf, int para);
#endif

#ifdef CONFIG_HISI_HISEE_CHIPTEST_SLT
int hisee_parallel_total_slt_func(void *buf, int para);
int hisee_read_slt_func(void *buf, int para);
int hisee_total_slt_func(void *buf, int para);
#endif
#ifdef CONFIG_HISI_HISEE_CHIPTEST_RT
int hisee_chiptest_rt_run_func(void * buf, int para);
int hisee_chiptest_rt_stop_func(void * buf, int para);
#endif


/* flag to indicate running status of flash otp1 */
typedef enum {
	NO_NEED = 0,
	PREPARED,
	RUNING,
	FINISH,
} E_RUN_STATUS;

/* set the otp1 write work status */
void hisee_chiptest_set_otp1_status(E_RUN_STATUS status);

/* check otp1 write work is running */
bool hisee_chiptest_otp1_is_runing(void);

E_RUN_STATUS hisee_chiptest_get_otp1_status(void);

#endif
