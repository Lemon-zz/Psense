#include <stdio.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#define LEDC_MAX_BIT_WIDTH      (20)
#define LEDC_CHANNELS           (16)
#define LEDC_TIMER              LEDC_TIMER_0
#define LOW_LEDC_TIMER          LEDC_TIMER_1
#define LEDC_HIGH_MODE          LEDC_HIGH_SPEED_MODE
#define LEDC_LOW_MODE           LEDC_LOW_SPEED_MODE

#define M1_1_PIN                GPIO_NUM_19
#define M1_2_PIN                GPIO_NUM_18
#define M2_1_PIN                GPIO_NUM_5
#define M2_2_PIN                GPIO_NUM_17
#define M3_1_PIN                GPIO_NUM_16
#define M3_2_PIN                GPIO_NUM_4
#define M4_1_PIN                GPIO_NUM_2
#define M4_2_PIN                GPIO_NUM_15
#define M5_1_PIN                GPIO_NUM_22
#define M5_2_PIN                GPIO_NUM_23



#define MOTOR_1_IN_1            LEDC_CHANNEL_1
#define MOTOR_1_IN_2            LEDC_CHANNEL_2
#define MOTOR_2_IN_1            LEDC_CHANNEL_0
#define MOTOR_2_IN_2            LEDC_CHANNEL_3
#define MOTOR_3_IN_1            LEDC_CHANNEL_4
#define MOTOR_3_IN_2            LEDC_CHANNEL_6
#define MOTOR_4_IN_1            LEDC_CHANNEL_5
#define MOTOR_4_IN_2            LEDC_CHANNEL_7
#define MOTOR_5_IN_1            LEDC_CHANNEL_1
#define MOTOR_5_IN_2            LEDC_CHANNEL_2

#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 8 bits
#define LEDC_DUTY_IDLE          (127) // Set duty to 50%. ((2 ** 12) - 1) * 50% = 127
#define LEDC_FREQUENCY          (20000) // Frequency in Hertz. 

#define EN_PIN GPIO_NUM_14
#define LED_PIN GPIO_NUM_13
#define DRIVERS_ON 1
#define DRIVERS_OFF 0

static const char* HW_TAG = "HW_module";



esp_err_t HW_init();
void ledcSetup(uint8_t chan, double freq, uint8_t bit_num, uint8_t pin);
void set_motors_en(uint8_t STATE);
void stop_pwm();
void set_PWM(uint8_t chan, uint32_t pwm);