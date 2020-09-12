#include "bk_gpio.h"




/*********************************************************************
 * LOCAL CONSTANTS
 */

/*********************************************************************
 * LOCAL STRUCT
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8_t s_bk_gpio_int_array[] = 
{
    BK_INPUT_PIN_0,
    BK_INPUT_PIN_1,
};

/*********************************************************************
 * LOCAL FUNCTION
 */
static void bk_gpio_int_handler(uint32_t pin);
static uint32_t bk_gpio_int_pin_change_format(uint32_t pin);

/*********************************************************************
 * VARIABLES
 */
bk_out_pin_t bk_output_pin_array[] = 
{
    {BK_OUTPUT_PIN_0, LEVEL_LOW},
    {BK_OUTPUT_PIN_1, LEVEL_LOW},
};
uint8_t bk_out_pins_num = sizeof(bk_output_pin_array)/sizeof(bk_output_pin_array[0]);




/*********************************************************
FN: 
*/
void bk_gpio_init_output(uint8_t pin)
{
    gpio_config(pin, OUTPUT, PULL_NONE);
    
    gpio_set(pin, LEVEL_LOW);
    for(uint8_t idx=0; idx<bk_out_pins_num; idx++) {
        if(bk_output_pin_array[idx].pin == pin) {
            bk_output_pin_array[idx].level = LEVEL_LOW;
        }
    }
}

/*********************************************************
FN: 
*/
void bk_gpio_init_input(uint8_t pin, uint8_t pull_type)
{
    gpio_config(pin, INPUT, (Pull_Type)pull_type);
}

/*********************************************************
FN: 
*/
uint8_t bk_gpio_get_input(uint8_t pin)
{
    return gpio_get_input(pin);
}

/*********************************************************
FN: 
*/
uint8_t bk_gpio_get_output(uint8_t pin)
{
    for(uint8_t idx=0; idx<bk_out_pins_num; idx++) {
        if(bk_output_pin_array[idx].pin == pin) {
            return bk_output_pin_array[idx].level;
        }
    }
    return LEVEL_INVALID;
}

/*********************************************************
FN: 输出引脚设置电平
*/
void tuya_gpio_set(uint8_t pin, uint8_t level)
{
    gpio_set(pin, level);
    
    for(uint8_t idx=0; idx<bk_out_pins_num; idx++) {
        if(bk_output_pin_array[idx].pin == pin) {
            bk_output_pin_array[idx].level = level;
        }
    }
}

/*********************************************************
FN: 输出引脚电平翻转
*/
void tuya_gpio_reverse(uint8_t pin)
{   
    uint8_t level = bk_gpio_get_output(pin);
    tuya_gpio_set(pin, !level);
}




/*********************************************************
FN: 外部中断
*/
static void bk_gpio_int_config(uint8_t* gpio_array, uint8_t len)
{
    for(uint8_t i=0; i<len; i++)
    {
        if(gpio_array[i] == BK_INPUT_PIN_0) { //上升沿触发
            bk_gpio_init_input(gpio_array[i], PULL_LOW);
            REG_APB5_GPIO_WUATOD_TYPE |= 0<<(8*(gpio_array[i]>>4)+(gpio_array[i]&0x0f)); //0<<2 = 0
        } else {                              //下降沿触发
            bk_gpio_init_input(gpio_array[i], PULL_HIGH);
            REG_APB5_GPIO_WUATOD_TYPE |= 1<<(8*(gpio_array[i]>>4)+(gpio_array[i]&0x0f)); //1<<2 = 4
        }
        REG_APB5_GPIO_WUATOD_STAT |= 1<<(8*(gpio_array[i]>>4)+(gpio_array[i]&0x0f));
        bk_delay_ms(2);
        REG_APB5_GPIO_WUATOD_ENABLE |= 1<<(8*(gpio_array[i]>>4)+(gpio_array[i]&0x0f));
        REG_AHB0_ICU_DEEP_SLEEP0 |= 1<<(8*(gpio_array[i]>>4)+(gpio_array[i]&0x0f));
    }
    REG_AHB0_ICU_INT_ENABLE |= (0x01 << 9);
}

/*********************************************************
FN: 外部中断初始化
*/
void bk_gpio_int_init(void)
{
    bk_gpio_int_config(s_bk_gpio_int_array, sizeof(s_bk_gpio_int_array));
    gpio_cb_register(bk_gpio_int_handler);
}

/*********************************************************
FN: 外部中断处理
*/
static void bk_gpio_int_handler(uint32_t pin)
{
    //引脚序号是8进制
    pin = bk_gpio_int_pin_change_format(pin);
    
    switch(pin)
    {
        case BK_INPUT_PIN_0: {
//            if(bk_gpio_get_input(pin) == LEVEL_HIGH)
            {
                BK_PRINTF("BK_INPUT_PIN_0");
            }
        } break;
        
        case BK_INPUT_PIN_1: {
//            if(bk_gpio_get_input(pin) == LEVEL_LOW)
            {
                BK_PRINTF("BK_INPUT_PIN_1");
            }
        } break;
        
        default: {
        } break;
    }
    bk_gpio_int_config(s_bk_gpio_int_array, sizeof(s_bk_gpio_int_array));
}

/*********************************************************
FN: 外部中断引脚转换
*/
static uint32_t bk_gpio_int_pin_change_format(uint32_t pin)
{
    uint32_t zero_count = 0;
    for(int idx=0; idx<32; idx++) {
        pin = pin>>1;
        if(pin == 0) {
            break;
        }
        zero_count++;
    }
    return (((zero_count/8)*0x10) + (zero_count%8));
}




/*********************************************************
FN: 初始化引脚
*/
void bk_gpio_init(void)
{
    //output
    for(uint32_t idx=0; idx<bk_out_pins_num; idx++)
    {
        bk_gpio_init_output(bk_output_pin_array[idx].pin);
    }
    
    //input
    bk_gpio_init_input(BK_INPUT_PIN_0, PULL_NONE);
    bk_gpio_init_input(BK_INPUT_PIN_1, PULL_NONE);
    
    //external interrupt
    bk_gpio_int_init();
}







/*********************************************************************
 * LOCAL CONSTANT
 */

/*********************************************************************
 * LOCAL STRUCT
 */

/*********************************************************************
 * LOCAL VARIABLE
 */
static uint8_t s_suble_gpio_irq_array[] = 
{
    ANTILOCK_BUTTON_PIN,
};

/*********************************************************************
 * VARIABLE
 */
suble_out_pin_t suble_out_pin_array[] = 
{
    {ANTILOCK_LED_RED_PIN, SUBLE_LEVEL_HIGH},
    {ANTILOCK_BUZZER_EN1, SUBLE_LEVEL_LOW},
    {ANTILOCK_BUZZER_EN2, SUBLE_LEVEL_HIGH},
    {ANTILOCK_BUZZER_DIN, SUBLE_LEVEL_LOW},
};
static uint8_t suble_out_pin_num = sizeof(suble_out_pin_array)/sizeof(suble_out_pin_array[0]);

/*********************************************************************
 * LOCAL FUNCTION
 */
static void suble_gpio_int_config(uint8_t* gpio_array, uint8_t len);
static void suble_gpio_irq_cb(uint32_t pin);
static uint32_t suble_gpio_irq_pin_change_format(uint32_t pin);




/*********************************************************
FN: 输出引脚初始化
*/
void suble_gpio_init_output(uint8_t pin)
{
    gpio_config(pin, OUTPUT, PULL_NONE);
    
    for(uint8_t idx=0; idx<suble_out_pin_num; idx++) {
        if(suble_out_pin_array[idx].pin == pin) {
            gpio_set(pin, suble_out_pin_array[idx].level);
        }
    }
}

/*********************************************************
FN: 输入引脚初始化
*/
void suble_gpio_init_input(uint8_t pin, uint8_t pull_type)
{
    gpio_config(pin, INPUT, (Pull_Type)pull_type);
}

/*********************************************************
FN: 获取输出引脚电平
*/
uint8_t suble_gpio_get_output(uint8_t pin)
{
    for(uint8_t idx=0; idx<suble_out_pin_num; idx++) {
        if(suble_out_pin_array[idx].pin == pin) {
            return suble_out_pin_array[idx].level;
        }
    }
    return SUBLE_LEVEL_INVALID;
}

/*********************************************************
FN: 获取输入引脚电平
*/
uint8_t suble_gpio_get_input(uint8_t pin)
{
    return gpio_get_input(pin);
}

/*********************************************************
FN: 输出引脚设置电平
*/
void suble_gpio_set(uint8_t pin, uint8_t level)
{
    gpio_set(pin, level);
    
    for(uint8_t idx=0; idx<suble_out_pin_num; idx++) {
        if(suble_out_pin_array[idx].pin == pin) {
            suble_out_pin_array[idx].level = level;
        }
    }
}

/*********************************************************
FN: 输出引脚电平翻转
*/
void suble_gpio_reverse(uint8_t pin)
{   
    uint8_t level = suble_gpio_get_output(pin);
    suble_gpio_set(pin, !level);
}




/*********************************************************
FN: 外部中断
*/
static void suble_gpio_int_config(uint8_t* gpio_array, uint8_t len)
{
    for(uint8_t i=0; i<len; i++)
    {
        { //下降沿触发
            suble_gpio_init_input(gpio_array[i], PULL_HIGH);
            REG_APB5_GPIO_WUATOD_TYPE |= 1<<(8*(gpio_array[i]>>4)+(gpio_array[i]&0x0f)); //1<<2 = 4
        }
        REG_APB5_GPIO_WUATOD_STAT |= 1<<(8*(gpio_array[i]>>4)+(gpio_array[i]&0x0f));
        bk_delay_ms(2);
        REG_APB5_GPIO_WUATOD_ENABLE |= 1<<(8*(gpio_array[i]>>4)+(gpio_array[i]&0x0f));
        REG_AHB0_ICU_DEEP_SLEEP0 |= 1<<(8*(gpio_array[i]>>4)+(gpio_array[i]&0x0f));
    }
    REG_AHB0_ICU_INT_ENABLE |= (0x01 << 9);
}

/*********************************************************
FN: 外部中断初始化
*/
void suble_gpio_irq_init(void)
{
    suble_gpio_int_config(s_suble_gpio_irq_array, sizeof(s_suble_gpio_irq_array));
    gpio_cb_register(suble_gpio_irq_cb);
}

/*********************************************************
FN: 外部中断处理
*/
static void suble_gpio_irq_cb(uint32_t pin)
{
    app_common_evt_send_with_data(APP_EVT_ANTILOST_KEY, &pin, sizeof(uint32_t));
}
void suble_gpio_irq_handler(void* buf, uint32_t size)
{
    if(size == 4)
    {
        uint32_t* pPin = buf;
        uint32_t pin = *pPin;
        //引脚序号是8进制
        pin = suble_gpio_irq_pin_change_format(pin);
        
        switch(pin)
        {
            case ANTILOCK_BUTTON_PIN: {
//                if(bk_gpio_get_input(pin) == LEVEL_LOW)
                {
                    bk_timer_start_0(SUBLE_TIMER104, 10, BK_TIMER_COUNT_ENDLESS);
                }
            } break;
            
            default: {
            } break;
        }
        suble_gpio_int_config(s_suble_gpio_irq_array, sizeof(s_suble_gpio_irq_array));
    }
}

/*********************************************************
FN: 外部中断引脚转换
*/
static uint32_t suble_gpio_irq_pin_change_format(uint32_t pin)
{
    uint32_t zero_count = 0;
    for(int idx=0; idx<32; idx++) {
        pin = pin>>1;
        if(pin == 0) {
            break;
        }
        zero_count++;
    }
    return (((zero_count/8)*0x10) + (zero_count%8));
}

/*********************************************************
FN: 蜂鸣器
*/
void suble_buzzer_start(uint32_t freq)
{
#define SUBLE_BUZZER_CLOCK 16000000
    
    if(freq < SUBLE_BUZZER_CLOCK/65535) {
        BK_PRINTF("Error: freq not support");
        return;
    }
    
	PWM_DRV_DESC pwm;
	pwm.channel = 1;
	pwm.mode = 0x10; //看注释
//	pwm.pre_divid = 1; //没用到
	pwm.end_value = SUBLE_BUZZER_CLOCK/freq; //定时时间
	pwm.duty_cycle = SUBLE_BUZZER_CLOCK/freq/2;
	pwm_init(&pwm);
	
	pwm_enable(1);
}

/*********************************************************
FN: 
*/
void suble_buzzer_stop(void)
{
	pwm_disable(1);
}

void suble_buzzer_set_mode(uint8_t level1, uint8_t level2)
{
    suble_gpio_set(ANTILOCK_BUZZER_EN1, level1);
    suble_gpio_set(ANTILOCK_BUZZER_EN2, level2);
}











/*********************************************************
FN: 初始化引脚
*/
void suble_gpio_init(void)
{
    //output
    for(uint32_t idx=0; idx<suble_out_pin_num; idx++) {
        suble_gpio_init_output(suble_out_pin_array[idx].pin);
    }
    
    //input
    suble_gpio_init_input(ANTILOCK_BUTTON_PIN, PULL_NONE);
    
    //interrupt
    suble_gpio_irq_init();
}




/*********************************************************
FN: 
*/
void suble_gpio_led_on(uint8_t pin)
{
    suble_gpio_set(pin, SUBLE_LEVEL_LOW);
}

/*********************************************************
FN: 
*/
void suble_gpio_led_off(uint8_t pin)
{
    suble_gpio_set(pin, SUBLE_LEVEL_HIGH);
}

/*********************************************************
FN: 
*/
void suble_gpio_led_reverse(uint8_t pin)
{
    suble_gpio_reverse(pin);
}

/*********************************************************
FN: 
*/
void suble_gpio_rled_blink(uint32_t count)
{
    suble_gpio_led_off(ANTILOCK_LED_RED_PIN);
    bk_timer_start_0(SUBLE_TIMER102, 100, count*2);
}

/*********************************************************
FN: 
*/
void suble_gpio_rled_blink_cancel(void)
{
    suble_gpio_led_off(ANTILOCK_LED_RED_PIN);
    bk_timer_stop_0(SUBLE_TIMER102);
}

/*********************************************************
FN: 
*/
void suble_gpio_buzzer_hold(void)
{
    suble_buzzer_start(4000);
    bk_timer_start_0(SUBLE_TIMER103, 1000, 1);
}

/*********************************************************
FN: 
*/
void suble_gpio_buzzer_hold_cancel(void)
{
    suble_buzzer_stop();
    bk_timer_stop_0(SUBLE_TIMER103);
}









/*********************************************************************
 * LOCAL CONSTANT
 */

/*********************************************************************
 * LOCAL STRUCT
 */

/*********************************************************************
 * LOCAL VARIABLE
 */

/*********************************************************************
 * VARIABLE
 */

/*********************************************************************
 * LOCAL FUNCTION
 */




/*********************************************************
FN: 
*/



/*********************************************************************
 * LOCAL CONSTANTS
 */
#define KEY_COUNT_1  (4/1)
#define KEY_COUNT_2  (300/1)
#define KEY_COUNT_3  (1000/1)

enum
{
    KEY_STATE_READY = 0,
    KEY_STATE_PRESSED_1,
    KEY_STATE_PRESSED_2,
    KEY_STATE_RELEASE,
    KEY_STATE_RELEASE_0, //4
    KEY_STATE_RELEASE_1,
    KEY_STATE_RELEASE_2,
};

/*********************************************************************
 * LOCAL STRUCT
 */

/*********************************************************************
 * LOCAL VARIABLES
 */




/*********************************************************
FN: 按键处理函数
*/
static void suble_key_state_handler(uint8_t state)
{
    UART_PRINTF("KEY_STATE: %d\r\n", state);
    switch(state)
    {
        case KEY_STATE_PRESSED_1: {
        } break;
        
        case KEY_STATE_PRESSED_2: {
        } break;
        
        case KEY_STATE_RELEASE_1: {
        } break;
        
        default: {
        } break;
    }
}

/*********************************************************
FN: 按键超时处理
*/
void suble_key_timeout_handler(void)
{
    #define SUBLE_KEY_TIME      5
    #define SUBLE_VALID_LEVEL   SUBLE_LEVEL_LOW
    static uint8_t  key_log[SUBLE_KEY_TIME];
    static uint32_t key_log_idx = 0;
    uint32_t key_log_sum = 0;
    static uint32_t key_count   = 0;
    static bool     key_pressed = true;
    static int      key_state   = KEY_STATE_READY;
    
    //按键计数
    key_count++;
    //中间消抖计数
    key_log_idx++;
    if(key_log_idx == SUBLE_KEY_TIME)
    {
        key_log_idx = 0;
    }
    
    //记录前10次的状态
    if(suble_gpio_get_input(ANTILOCK_BUTTON_PIN) == SUBLE_VALID_LEVEL) {
        key_log[key_log_idx] = 0;
    } else {
        key_log[key_log_idx] = 1;
    }
    //统计前10次的状态
    for(uint8_t idx=0; idx<SUBLE_KEY_TIME; idx++) {
        key_log_sum += key_log[idx];
    }
    //无效次数大于7次
    if(key_log_sum > 4) {
        key_pressed = false;
        memset(key_log, 0, SUBLE_KEY_TIME);
    }
    
//    UART_PRINTF(": %d\r\n", key_pressed);
    if(key_pressed == true)
    {
        if(key_count == KEY_COUNT_1) {
            if(key_state == KEY_STATE_READY) {
                key_state = KEY_STATE_PRESSED_1;
                suble_key_state_handler(key_state);
            }
        }
        else if(key_count == KEY_COUNT_2) {
            if(key_state == KEY_STATE_PRESSED_1) {
                key_state = KEY_STATE_PRESSED_2;
                suble_key_state_handler(key_state);
            }
        }
        else if(key_count == KEY_COUNT_3) {
            if(key_state == KEY_STATE_PRESSED_2) {
                //超时释放
                key_state = KEY_STATE_RELEASE;
            }
        }
    }
    else {
        if(key_count <= KEY_COUNT_1) {
            if(key_state == KEY_STATE_READY) {
                key_state = KEY_STATE_RELEASE_0;
            }
        }
        else if((key_count > KEY_COUNT_1) && (key_count <= KEY_COUNT_2)) {
            if(key_state == KEY_STATE_PRESSED_1) {
                key_state = KEY_STATE_RELEASE_1;
            }
        }
        else if((key_count > KEY_COUNT_2) && (key_count <= KEY_COUNT_3)) {
            if(key_state == KEY_STATE_PRESSED_2) {
                key_state = KEY_STATE_RELEASE_2;
            }
        }
    }
    
    //释放
    if(key_state >= KEY_STATE_RELEASE) {
        suble_key_state_handler(key_state);
        
        key_count = 0;
        key_pressed = true;
        key_state = KEY_STATE_READY;
        bk_timer_stop_0(SUBLE_TIMER104);
    }
}










//中音——C调
#define NTD0 100
#define NTD1 262
#define NTD2 294
#define NTD3 330
#define NTD4 350
#define NTD5 393
#define NTD6 441
#define NTD7 495
//中音——D调
#define NTD0_D 100
#define NTD1_D 294
#define NTD2_D 330
#define NTD3_D 350
#define NTD4_D 393
#define NTD5_D 441
#define NTD6_D 495
#define NTD7_D 556

#pragma pack(1)
//open with bt
typedef struct
{
    const uint16_t* pTune;
    const float* pDurt;
    uint32_t    size;
} music_tune_t;
#pragma pack()

/*********************************************************  提示音1  *********************************************************/
//根据简谱列出各频率
static const uint16_t tune3[]= {
    NTD1,NTD2,NTD1,NTD3,NTD2,NTD1,NTD4,NTD3,NTD5,
};
//根据简谱列出各节拍
float durt3[]= {
    0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5+1,0.5,
};

/*********************************************************  提示音2  *********************************************************/
//根据简谱列出各频率
static const uint16_t tune4[]= {
    NTD1,NTD3,NTD5,NTD1,NTD1,NTD5,NTD3,NTD1
};
//根据简谱列出各节拍
static const float durt4[]= {
    0.5,0.5,0.5,0.5+1,0.5,0.5,0.5,0.5+1
};

/*********************************************************  音乐指针  *********************************************************/
static music_tune_t music[]= {
    { tune3, durt3, sizeof(tune3)/sizeof(tune3[0]) },
    { tune4, durt4, sizeof(tune4)/sizeof(tune4[0]) },
};
static uint32_t s_music_idx = 0;
static uint32_t s_tune_idx = 0;
static uint8_t s_music_mode = MUSIC_MODE_ONCE;



/*********************************************************
FN: 
*/
void suble_buzzer_timeout_handler(void)
{
    s_tune_idx++;
    if(s_tune_idx < music[s_music_idx].size) {
        suble_buzzer_stop();
        bk_delay_ms(10);
        
        uint16_t tune = music[s_music_idx].pTune[s_tune_idx];
        float durt = music[s_music_idx].pDurt[s_tune_idx];
        
        suble_buzzer_start(tune*10);
        if(s_music_mode == MUSIC_MODE_ONCE) {
            bk_timer_start_0(SUBLE_TIMER100, (100*durt), 1);
//            APP_DEBUG_PRINTF("count: %d, delay: %d", s_tune_idx, (100*durt));
        }
        else {
            bk_timer_start_0(SUBLE_TIMER100, (500*durt), 1);
//            APP_DEBUG_PRINTF("count: %d, delay: %d", s_tune_idx, (500*durt));
        }
    }
    else {
        if(s_music_mode == MUSIC_MODE_ONCE) {
            suble_buzzer_stop();
//            g_system_sleep = true;
            APP_DEBUG_PRINTF("suble_buzzer_stop");
        }
        else {
            lock_play_music(MUSIC_MODE_REPEAT, s_music_idx);
        }
    }
}

/*********************************************************
FN: 
*/
void lock_play_music(uint8_t mode, uint32_t music_idx)
{
    APP_DEBUG_PRINTF("music_idx: %d", music_idx);
    
    if((suble_gpio_get_output(ANTILOCK_BUZZER_EN1) == SUBLE_LEVEL_HIGH)
        || (suble_gpio_get_output(ANTILOCK_BUZZER_EN2) == SUBLE_LEVEL_HIGH))
    {
        g_system_sleep = false;
        
        s_music_mode = mode;
        s_tune_idx = 0;
        s_music_idx = music_idx;
        
        uint16_t tune = music[s_music_idx].pTune[s_tune_idx];
        float durt = music[s_music_idx].pDurt[s_tune_idx];
        
        suble_buzzer_start(tune*10);
        if(s_music_mode == MUSIC_MODE_ONCE) {
            bk_timer_start_0(SUBLE_TIMER100, (100*durt), 1);
//            APP_DEBUG_PRINTF("count: %d, delay: %d", s_tune_idx, (100*durt));
        }
        else {
            bk_timer_start_0(SUBLE_TIMER100, (500*durt), 1);
//            APP_DEBUG_PRINTF("count: %d, delay: %d", s_tune_idx, (500*durt));
        }
    }
}

/*********************************************************
FN: 
*/
void lock_play_music_cancel(void)
{
    s_tune_idx = music[s_music_idx].size;
    s_music_mode = MUSIC_MODE_ONCE;
}




















