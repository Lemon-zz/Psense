#include "HW.h"   

esp_err_t HW_init(){
    
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        ledc_timer_config_t low_ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LOW_LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&low_ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    //m1
    ledc_channel_config_t m1_1_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = MOTOR_1_IN_1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M1_1_PIN,
        .duty           = 50, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m1_1_channel));
    
    ledc_channel_config_t m1_2_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = MOTOR_1_IN_2,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M1_2_PIN,
        .duty           = 50, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m1_2_channel));
    
    //m2**************************************************************************
    ledc_channel_config_t m2_1_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = MOTOR_2_IN_1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M2_1_PIN,
        .duty           = 50, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m2_1_channel));
    
    ledc_channel_config_t m2_2_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = MOTOR_2_IN_2,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M2_2_PIN,
        .duty           = 50, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m2_2_channel));
    
    //m3**************************************************************************
    ledc_channel_config_t m3_1_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = MOTOR_3_IN_1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M3_1_PIN,
        .duty           = 50, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m3_1_channel));
    
    ledc_channel_config_t m3_2_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = MOTOR_3_IN_2,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M3_2_PIN,
        .duty           = 50, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m3_2_channel));
    
    //m4**************************************************************************
    ledc_channel_config_t m4_1_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = MOTOR_4_IN_1,
        .timer_sel      = LOW_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M4_1_PIN,
        .duty           = 50, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m4_1_channel));
    
    ledc_channel_config_t m4_2_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = MOTOR_4_IN_2,
        .timer_sel      = LOW_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M4_2_PIN,
        .duty           = 50, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m4_2_channel));
    
    //m5**************************************************************************
    ledc_channel_config_t m5_1_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = MOTOR_5_IN_1,
        .timer_sel      = LOW_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M5_1_PIN,
        .duty           = 50, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m5_1_channel));
    
    ledc_channel_config_t m5_2_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = MOTOR_5_IN_2,
        .timer_sel      = LOW_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M5_2_PIN,
        .duty           = 50, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m5_2_channel));
    



    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 1);
    gpio_pad_select_gpio(EN_PIN);
    gpio_set_direction(EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(EN_PIN, 1);

    uint32_t a =0;
    return a;
}

void set_motors_en(uint8_t STATE){
    if (STATE){
        gpio_pad_select_gpio(EN_PIN);
        gpio_set_level(EN_PIN, 1);
        gpio_pad_select_gpio(LED_PIN);
        gpio_set_level(LED_PIN, 1);
    }
    else{
        gpio_pad_select_gpio(EN_PIN);
        gpio_set_level(EN_PIN, 0);
        gpio_pad_select_gpio(LED_PIN);
        gpio_set_level(LED_PIN, 0);
    }
}

void set_m1_1_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_1_IN_1, pwm));
}

void set_m1_2_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_1_IN_2, pwm));
}

void set_m2_1_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_2_IN_1, pwm));
}

void set_m2_2_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_2_IN_2, pwm));
}

void set_m3_1_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_3_IN_1, pwm));
}

void set_m3_2_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_3_IN_2, pwm));
}

void set_m4_1_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_4_IN_2, pwm));
}

void set_m4_2_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_4_IN_2, pwm));
}

void set_m5_1_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_1, pwm));
}

void set_m5_2_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_2, pwm));
}

void stop_pwm(){
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE,MOTOR_1_IN_1,50));
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE,MOTOR_1_IN_2,50));
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE,MOTOR_2_IN_1,50));
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE,MOTOR_2_IN_2,50));
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE,MOTOR_3_IN_1,50));
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE,MOTOR_3_IN_2,50));
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE,MOTOR_4_IN_1,50));
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE,MOTOR_4_IN_2,50));
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE,MOTOR_5_IN_1,50));
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE,MOTOR_5_IN_1,50));
}