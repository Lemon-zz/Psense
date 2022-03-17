#include "HW.h"   

esp_err_t HW_init(){


    //gpio_pad_select_gpio(LED_PIN);
    //gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    //gpio_set_level(LED_PIN, 1);
    gpio_pad_select_gpio(EN_PIN);
    gpio_set_direction(EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(EN_PIN, 0);
    /*
    gpio_pad_select_gpio(M1_2_PIN);
    gpio_set_direction(M1_2_PIN, GPIO_MODE_OUTPUT);
    gpio_pulldown_en(M1_2_PIN);
    gpio_set_level(M1_2_PIN, 0);
    */

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_timer_config_t ledc_timer1 = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_1,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer1));

    ledc_channel_config_t m1_1_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = MOTOR_1_IN_1,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M1_1_PIN,
        .duty           = 127, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m1_1_channel));
    


    ledc_channel_config_t m1_2_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = MOTOR_1_IN_2,
        .timer_sel      = LEDC_TIMER_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M1_2_PIN,
        .duty           = 127, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m1_2_channel));
    

    ledc_timer_config_t ledc_timer_low0 = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_low0));

    ledc_timer_config_t ledc_timer_low1 = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_1,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_low1));



    ledc_channel_config_t m5_1_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = MOTOR_5_IN_1,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M5_1_PIN,
        .duty           = 127, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m5_1_channel));
    

    ledc_channel_config_t m5_2_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = MOTOR_5_IN_2,
        .timer_sel      = LEDC_TIMER_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M5_2_PIN,
        .duty           = 127, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m5_2_channel));
  
    /*
    set_m1_1_pwm(255);
    set_m1_2_pwm(0);
    vTaskDelay(700 / portTICK_RATE_MS);
    set_m1_1_pwm(0);
    set_m1_2_pwm(255);
    vTaskDelay(500 / portTICK_RATE_MS);
    set_m1_1_pwm(127);
    set_m1_2_pwm(120);
    
    set_m5_1_pwm(255);
    set_m5_2_pwm(0);
    vTaskDelay(700 / portTICK_RATE_MS);
    set_m5_1_pwm(0);
    set_m5_2_pwm(255);
    vTaskDelay(500 / portTICK_RATE_MS);
    set_m5_1_pwm(127);
    set_m5_2_pwm(120);

    for (int  i = 0; i <= 255; i++)
    {
        ESP_LOG_BUFFER_CHAR(HW_TAG,"Duty",5);
        uint32_t duty1 = ledc_get_duty(LEDC_LOW_SPEED_MODE,MOTOR_5_IN_1);
        uint32_t duty2 = ledc_get_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_2);
        esp_log_buffer_hex("", &duty1, sizeof(uint32_t));
        esp_log_buffer_hex("", &duty2, sizeof(uint32_t));

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_1, i));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_1));
        
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_2, 125));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_2));
        
        vTaskDelay(100 / portTICK_RATE_MS);
    }*/
    return 0;
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
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_1_IN_1));
}


void set_m1_2_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_1_IN_2, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_1_IN_2));
}

void set_m2_1_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_2_IN_1, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_2_IN_1));
}

void set_m2_2_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_2_IN_2, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_2_IN_2));
}

void set_m3_1_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_3_IN_1, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_3_IN_1));
}

void set_m3_2_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_3_IN_2, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_3_IN_2));
}

void set_m4_1_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_4_IN_1, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_4_IN_1));
}

void set_m4_2_pwm(uint8_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_4_IN_2, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_4_IN_2));
}

void set_m5_1_pwm(uint32_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_1, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_1));
}

void set_m5_2_pwm(uint32_t pwm){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_2, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_2));
}

void stop_pwm(){
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE,MOTOR_1_IN_1,127));
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE,MOTOR_1_IN_2,127));
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE,MOTOR_2_IN_1,127));
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE,MOTOR_2_IN_2,127));
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE,MOTOR_3_IN_1,127));
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE,MOTOR_3_IN_2,127));
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE,MOTOR_4_IN_1,127));
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE,MOTOR_4_IN_2,127));
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE,MOTOR_5_IN_1,127));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_1));
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE,MOTOR_5_IN_2,127));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_2));

}