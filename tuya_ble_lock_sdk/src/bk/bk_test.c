#include "bk_test.h"
#include "sf_port.h"




/*********************************************************************
 * LOCAL CONSTANTS
 */

/*********************************************************************
 * LOCAL STRUCT
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTION
 */

/*********************************************************************
 * VARIABLES
 */
#define TUYA_TEST_LEN  32
#define TUYA_TEST_ADDR 0x70000

uint8_t tmpBuf1[TUYA_TEST_LEN] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x00};
uint8_t tmpBuf2[TUYA_TEST_LEN] = {0};


void suble_buzzer_test_handler(unsigned char ucChannel)
{
    suble_gpio_reverse(ANTILOCK_BUZZER_DIN);
}


void suble_buzzer_test(void)
{
	PWM_DRV_DESC pwm;
    suble_gpio_init_output(ANTILOCK_BUZZER_DIN);
	pwm.channel = 2;
	pwm.mode = 0x06; //看注释
//	pwm.pre_divid = 1; //没用到
	pwm.end_value = 8; //定时时间
	pwm.duty_cycle = 4;
    pwm.p_Int_Handler = suble_buzzer_test_handler;
	pwm_init(&pwm);
	
	pwm_enable(2);
}


/*********************************************************
FN: 
*/
void bk_test(void)
{
//    suble_buzzer_test();
//    sf_nv_test(SF_AREA_0);
//    sf_nv_test(SF_AREA_1);
//    sf_nv_test(SF_AREA_2);
//    sf_nv_test(SF_AREA_3);
//    sf_nv_test(SF_AREA_4);
}









