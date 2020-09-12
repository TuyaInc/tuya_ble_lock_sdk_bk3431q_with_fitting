#include "bk_timer.h"




/*********************************************************************
 * LOCAL CONSTANTS
 */

/*********************************************************************
 * LOCAL STRUCT
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static bk_timer_t s_bk_timer[BK_TIMERX_MAX-BK_TIMER0] = {0};

/*********************************************************************
 * LOCAL FUNCTION
 */
static void bk_timer_continue(ke_msg_id_t const timer_id);
static void bk_timer_app_handler(uint32_t timer_id);

/*********************************************************************
 * VARIABLES
 */




/*********************************************************  bk_timer  *********************************************************/

/*********************************************************
FN: ��ʱ����
*/
void bk_timer_handler(ke_msg_id_t timer_id)
{
    if((timer_id >= BK_TIMER0) && (timer_id < SUBLE_TIMER100)) {
        bk_timer_app_handler(timer_id);
    }
    else {
        switch(timer_id)
        {
            case SUBLE_TIMER100: {
                suble_buzzer_timeout_handler();
            } break;
            
            case SUBLE_TIMER101: {
            } break;
            
            case SUBLE_TIMER102: {
                suble_gpio_led_reverse(ANTILOCK_LED_RED_PIN);
            } break;
            
            case SUBLE_TIMER103: {
                suble_buzzer_stop();
            } break;
            
            case SUBLE_TIMER104: {
                app_common_evt_send_only_evt(APP_EVT_SUBLE_KEY_OUTTIME);
            } break;
            
            case SUBLE_TIMER105: {
            } break;
            
            case SUBLE_TIMER106: {
            } break;
            
            case SUBLE_TIMER107: {
            } break;
            
            case SUBLE_TIMER108: {
            } break;
            
            case SUBLE_TIMER109: {
            } break;
            
            default: {
            } break;
        }
    }
    bk_timer_continue(timer_id);
}

/*********************************************************
FN: ����
*/
void bk_timer_start_0(ke_msg_id_t const timer_id, uint32_t ms, uint32_t count)
{
    if((timer_id < BK_TIMER0) || (timer_id >= BK_TIMERX_MAX))
    {
        return;
    }
    
    uint32_t idx = timer_id-BK_TIMER0;
    
    s_bk_timer[idx].delay = ms/10;
    s_bk_timer[idx].count = count;
    ke_timer_set(timer_id, TASK_APP, s_bk_timer[idx].delay);
}

/*********************************************************
FN: ����
*/
static void bk_timer_continue(ke_msg_id_t const timer_id)
{
    if((timer_id < BK_TIMER0) || (timer_id >= BK_TIMERX_MAX))
    {
        return;
    }
    
    uint32_t idx = timer_id-BK_TIMER0;
    
    //������
    if((s_bk_timer[idx].count != BK_TIMER_COUNT_ENDLESS) && (s_bk_timer[idx].count > 0))
    {
        s_bk_timer[idx].count--;
    }
    
    if(s_bk_timer[idx].count > 0)
    {
        ke_timer_set(timer_id, TASK_APP, s_bk_timer[idx].delay);
    }
}

/*********************************************************
FN: ֹͣ
*/
void bk_timer_stop_0(ke_msg_id_t const timer_id)
{
    if((timer_id < BK_TIMER0) || (timer_id >= BK_TIMERX_MAX))
    {
        return;
    }
    
    uint32_t idx = timer_id-BK_TIMER0;
    
    s_bk_timer[idx].count = 0;
    s_bk_timer[idx].delay = 0;
    ke_timer_clear(timer_id, TASK_APP);
}

/*********************************************************
FN: �Ƿ�����
*/
uint8_t bk_timer_is_running(ke_msg_id_t const timer_id)
{
    if((timer_id < BK_TIMER0) || (timer_id >= BK_TIMERX_MAX))
    {
        return false;
    }
    
    uint32_t idx = timer_id-BK_TIMER0;
    
    return (s_bk_timer[idx].count != 0);
}




/*********************************************************  bk_timer app  *********************************************************/

typedef struct {
    uint8_t is_occupy;
    ke_msg_id_t timer_id;
    uint32_t count;
    uint32_t ms;
    bk_timer_handler_t handler;
} bk_timer_item_t;

static bk_timer_item_t m_timer_pool[] = {
    [0]  = { .timer_id = BK_TIMER0},
    [1]  = { .timer_id = BK_TIMER1},
    [2]  = { .timer_id = BK_TIMER2},
    [3]  = { .timer_id = BK_TIMER3},
    [4]  = { .timer_id = BK_TIMER4},
    [5]  = { .timer_id = BK_TIMER5},
    [6]  = { .timer_id = BK_TIMER6},
    [7]  = { .timer_id = BK_TIMER7},
    [8]  = { .timer_id = BK_TIMER8},
    [9]  = { .timer_id = BK_TIMER9},
    [10] = { .timer_id = BK_TIMER10},
    [11] = { .timer_id = BK_TIMER11},
    [12] = { .timer_id = BK_TIMER12},
    [13] = { .timer_id = BK_TIMER13},
    [14] = { .timer_id = BK_TIMER14},
    [15] = { .timer_id = BK_TIMER15},
    [16] = { .timer_id = BK_TIMER16},
    [17] = { .timer_id = BK_TIMER17},
    [18] = { .timer_id = BK_TIMER18},
    [19] = { .timer_id = BK_TIMER19},
};

/*********************************************************
FN: 
*/
static bk_timer_item_t* acquire_timer(uint32_t ms, uint32_t count, bk_timer_handler_t handler)
{
    for(uint8_t i=0; i<BK_TIMER_MAX_NUM; i++) {
        if (m_timer_pool[i].is_occupy == 0) {
            m_timer_pool[i].is_occupy = 1;
            m_timer_pool[i].count = count;
            m_timer_pool[i].ms = ms;
            m_timer_pool[i].handler = handler;
            return &m_timer_pool[i];
        }
    }
    return NULL;
}

/*********************************************************
FN: 
*/
static int32_t release_timer(void* timer_id)
{
    for(uint8_t i=0; i<BK_TIMER_MAX_NUM; i++) {
        if (timer_id == &m_timer_pool[i]) {
            m_timer_pool[i].is_occupy = 0;
            return i;
        }
    }
    return -1;
}

/*********************************************************
FN: 
*/
static int32_t find_timer_ms(void* timer_id, uint32_t *ms)
{
    for(uint8_t i=0; i<BK_TIMER_MAX_NUM; i++) {
        if (timer_id == &m_timer_pool[i]) {
            *ms = m_timer_pool[i].ms;
            return i;
        }
    }
    return -1;
}

/*********************************************************
FN: 
*/
static void bk_timer_app_handler(uint32_t timer_id)
{
    for(uint8_t i=0; i<BK_TIMER_MAX_NUM; i++) {
        if (timer_id == m_timer_pool[i].timer_id) {
            m_timer_pool[i].handler(&m_timer_pool[i]);
            break;
        }
    }
}

/*********************************************************
FN: 
*/
uint32_t bk_timer_create(void** p_timer_id, uint32_t timeout_value_ms, bk_timer_mode_t mode, bk_timer_handler_t timeout_handler)
{
    bk_timer_handler_t handler = timeout_handler;
    bk_timer_item_t* timer_item = acquire_timer(timeout_value_ms, (mode==BK_TIMER_SINGLE_SHOT ? 1 : BK_TIMER_COUNT_ENDLESS), handler);
    if (timer_item == NULL) {
        return BK_ERROR_COMMON;
    }
    
    *p_timer_id = timer_item;
    return BK_SUCCESS;
}

/*********************************************************
FN: 
*/
uint32_t bk_timer_delete(void* timer_id)
{
    bk_timer_item_t* timer_item = timer_id;
    int id = release_timer(timer_item);
    if (id == -1) {
        return BK_ERROR_COMMON;
    }
    
    bk_timer_stop_0(timer_item->timer_id);
    return BK_SUCCESS;
}

/*********************************************************
FN: 
*/
uint32_t bk_timer_start(void* timer_id)
{
    uint32_t ms;
    bk_timer_item_t* timer_handle = timer_id;
    
    if(find_timer_ms(timer_id, &ms) >= 0)
    {
        bk_timer_start_0(timer_handle->timer_id, timer_handle->ms, timer_handle->count);
    }
    return BK_SUCCESS;
}

/*********************************************************
FN: 
*/
uint32_t bk_timer_stop(void* timer_id)
{
    bk_timer_item_t* timer_handle = timer_id;
    bk_timer_stop_0(timer_handle->timer_id);
    return BK_SUCCESS;
}




/*********************************************************  RTC 2  *********************************************************/

/* Declaring an instance of nrf_drv_rtc for RTC2. */
static uint32_t s_local_timestamp = 0;
static uint32_t s_local_timestamp_when_update = 0;
static uint32_t s_app_timestamp_when_update = 0;


/*********************************************************
FN: ��������ʱ���
*/
void bk_rtc_handler(void)
{
    s_local_timestamp++;
}

/*********************************************************
FN: ��������ʱ���
*/
static void bk_rtc_start(void)
{
    rtc_alarm_init(0x01, NULL, 500, bk_rtc_handler);
    
//	PWM_DRV_DESC pwm;
//    memset(&pwm, 0, sizeof(PWM_DRV_DESC));
//	pwm.channel = 0x00;
//	pwm.mode = 0x06; //��ע��
//	pwm.end_value = (uint16_t)(32.768*1000); //��ʱʱ��
//	pwm.duty_cycle = 0;
//    pwm.p_Int_Handler = bk_rtc_handler;
//	pwm_init(&pwm);
//	
//	pwm_enable(0x00);
}

/*********************************************************
FN: ��������ʱ���
*/
void bk_local_timer_start(void)
{
    bk_rtc_start();
}

/*********************************************************
FN: ����ʱ���
*/
void bk_update_timestamp(uint32_t app_timestamp)
{
    s_local_timestamp_when_update = s_local_timestamp;
    s_app_timestamp_when_update = app_timestamp;
}

/*********************************************************
FN: ��ȡ����ʱ��appʱ���
*/
uint32_t bk_get_app_timestamp_when_update(void)
{
    return s_app_timestamp_when_update;
}

/*********************************************************
FN: ��ȡ����ʱ���
*/
uint32_t bk_get_local_timestamp(void)
{
    return s_local_timestamp;
}

/*********************************************************
FN: ��ȡ��ǰʱ��������û�и��¹�����Ϊ����ʱ�����
*/
uint32_t bk_get_timestamp(void)
{
    return (s_app_timestamp_when_update + (s_local_timestamp - s_local_timestamp_when_update));
}

/*********************************************************
FN: ��ȡ��ȥ��ʱ����������ڸ���ʱ���֮��ʹ�ã����򷵻� old_local_timestamp��
*/
uint32_t bk_get_old_timestamp(uint32_t old_local_timestamp)
{
    return (bk_get_timestamp() - (s_local_timestamp - old_local_timestamp));
}




/*********************************************************  delay  *********************************************************/

/*********************************************************
FN: 
*/
void bk_delay_ms(uint32_t ms)
{
    Delay_ms(ms);
}

/*********************************************************
FN: 
*/
void bk_delay_us(uint32_t us)
{
    Delay_us(us);
}











