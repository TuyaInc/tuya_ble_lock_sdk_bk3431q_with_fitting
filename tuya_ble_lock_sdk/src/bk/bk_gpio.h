/**
****************************************************************************
* @file      bk_gpio.h
* @brief     bk_gpio
* @author    suding
* @version   V1.0.0
* @date      2019-10-12
* @note
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2019 Tuya </center></h2>
*/


#ifndef __BK_GPIO_H__
#define __BK_GPIO_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "bk_common.h"

/*********************************************************************
 * CONSTANTS
 */
#define BK_OUTPUT_PIN_0 0x12
#define BK_OUTPUT_PIN_1 0x13
#define BK_INPUT_PIN_0  0x10
#define BK_INPUT_PIN_1  0x11

#define ANTILOCK_LED_RED_PIN                    0x12
#define ANTILOCK_BUTTON_PIN                     0x10
#define ANTILOCK_BUZZER_EN1                     0x31
#define ANTILOCK_BUZZER_EN2                     0x32
#define ANTILOCK_BUZZER_DIN                     0x11

enum
{
    LEVEL_INVALID = 0xFF,
    LEVEL_LOW  = 0,
    LEVEL_HIGH = 1,
};

enum
{
    SUBLE_LEVEL_INVALID = 0xFF,
    SUBLE_LEVEL_LOW  = 0,
    SUBLE_LEVEL_HIGH = 1,
};

typedef enum
{
    MUSIC_NOTIFY_1 = 0,
    MUSIC_NOTIFY_2,
} music_enum_t;

typedef enum
{
    MUSIC_MODE_ONCE = 0,
    MUSIC_MODE_REPEAT,
} music_mode_t;

/*********************************************************************
 * STRUCT
 */
typedef struct
{
    uint8_t pin;
    uint8_t level;
} bk_out_pin_t;

typedef struct
{
    uint8_t pin;
    uint8_t level;
} suble_out_pin_t;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
void bk_gpio_init_output(uint8_t pin);
void bk_gpio_init_input(uint8_t pin, uint8_t pull_type);
uint8_t bk_gpio_get_input(uint8_t pin);
uint8_t bk_gpio_get_output(uint8_t pin);
void tuya_gpio_set(uint8_t pin, uint8_t level);
void tuya_gpio_reverse(uint8_t pin);
void bk_gpio_int_init(void);

void bk_gpio_init(void);


void suble_gpio_init_output(uint8_t pin);
void suble_gpio_init_input(uint8_t pin, uint8_t pull_type);
uint8_t suble_gpio_get_input(uint8_t pin);
uint8_t suble_gpio_get_output(uint8_t pin);
void suble_gpio_set(uint8_t pin, uint8_t level);
void suble_gpio_reverse(uint8_t pin);
void suble_gpio_irq_init(void);
void suble_gpio_irq_handler(void* buf, uint32_t size);

void suble_buzzer_start(uint32_t freq);
void suble_buzzer_stop(void);
void suble_buzzer_set_sound_volume(uint8_t sound_volume);

void suble_gpio_init(void);

void suble_gpio_led_on(uint8_t pin);
void suble_gpio_led_off(uint8_t pin);
void suble_gpio_led_reverse(uint8_t pin);
void suble_gpio_rled_blink(uint32_t count);
void suble_gpio_rled_blink_cancel(void);
void suble_gpio_buzzer_hold(void);
void suble_gpio_buzzer_hold_cancel(void);


void suble_key_timeout_handler(void);


void suble_buzzer_timeout_handler(void);
void lock_play_music(uint8_t mode, uint32_t music_idx);
void lock_play_music_cancel(void);

#ifdef __cplusplus
}
#endif

#endif //__BK_GPIO_H__
