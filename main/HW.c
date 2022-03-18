#include "HW.h"

esp_err_t HW_init()
{

    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 1);
    gpio_pad_select_gpio(EN_PIN);
    gpio_set_direction(EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(EN_PIN, 1);





    ledcSetup(MOTOR_1_IN_1, LEDC_FREQUENCY, LEDC_DUTY_RES, M1_1_PIN);
    ledcSetup(MOTOR_1_IN_2, LEDC_FREQUENCY, LEDC_DUTY_RES, M1_2_PIN);

    
    set_PWM(MOTOR_1_IN_1, 255);
    set_PWM(MOTOR_1_IN_2, 0);
    uint32_t duty1 = ledc_get_duty(LEDC_HIGH_SPEED_MODE,MOTOR_5_IN_1);
    uint32_t duty2 = ledc_get_duty(LEDC_HIGH_SPEED_MODE, MOTOR_5_IN_2);

    esp_log_buffer_hex("", &duty1, sizeof(uint32_t));
    esp_log_buffer_hex("", &duty2, sizeof(uint32_t));

    vTaskDelay(700 / portTICK_RATE_MS);

    set_PWM(MOTOR_1_IN_1, 0);
    set_PWM(MOTOR_1_IN_2, 255);

    duty1 = ledc_get_duty(LEDC_HIGH_SPEED_MODE,MOTOR_5_IN_1);
    duty2 = ledc_get_duty(LEDC_HIGH_SPEED_MODE, MOTOR_5_IN_2);
    esp_log_buffer_hex("", &duty1, sizeof(uint32_t));
    esp_log_buffer_hex("", &duty2, sizeof(uint32_t));

    vTaskDelay(500 / portTICK_RATE_MS);
    set_PWM(MOTOR_1_IN_1, 127);
    set_PWM(MOTOR_1_IN_2, 127);
    gpio_set_level(EN_PIN, 0);

    duty1 = ledc_get_duty(LEDC_HIGH_SPEED_MODE,MOTOR_5_IN_1);
    duty2 = ledc_get_duty(LEDC_HIGH_SPEED_MODE, MOTOR_5_IN_2);
    esp_log_buffer_hex("", &duty1, sizeof(uint32_t));
    esp_log_buffer_hex("", &duty2, sizeof(uint32_t));
/*
    set_m5_1_pwm(255);
    set_m5_2_pwm(0);
    vTaskDelay(700 / portTICK_RATE_MS);
    set_m5_1_pwm(0);
    set_m5_2_pwm(255);
    vTaskDelay(500 / portTICK_RATE_MS);
    set_m5_1_pwm(LEDC_DUTY_IDLE  );
    set_m5_2_pwm(120);
*/
/*
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

void set_motors_en(uint8_t STATE)
{
    if (STATE)
    {
        gpio_pad_select_gpio(EN_PIN);
        gpio_set_level(EN_PIN, 1);
        gpio_pad_select_gpio(LED_PIN);
        gpio_set_level(LED_PIN, 1);
    }
    else
    {
        gpio_pad_select_gpio(EN_PIN);
        gpio_set_level(EN_PIN, 0);
        gpio_pad_select_gpio(LED_PIN);
        gpio_set_level(LED_PIN, 0);
    }
}

/*
 * LEDC Chan to Group/Channel/Timer Mapping
 ** ledc: 0  => Group: 0, Channel: 0, Timer: 0
 ** ledc: 1  => Group: 0, Channel: 1, Timer: 0
 ** ledc: 2  => Group: 0, Channel: 2, Timer: 1
 ** ledc: 3  => Group: 0, Channel: 3, Timer: 1
 ** ledc: 4  => Group: 0, Channel: 4, Timer: 2
 ** ledc: 5  => Group: 0, Channel: 5, Timer: 2
 ** ledc: 6  => Group: 0, Channel: 6, Timer: 3
 ** ledc: 7  => Group: 0, Channel: 7, Timer: 3
 ** ledc: 8  => Group: 1, Channel: 0, Timer: 0
 ** ledc: 9  => Group: 1, Channel: 1, Timer: 0
 ** ledc: 10 => Group: 1, Channel: 2, Timer: 1
 ** ledc: 11 => Group: 1, Channel: 3, Timer: 1
 ** ledc: 12 => Group: 1, Channel: 4, Timer: 2
 ** ledc: 13 => Group: 1, Channel: 5, Timer: 2
 ** ledc: 14 => Group: 1, Channel: 6, Timer: 3
 ** ledc: 15 => Group: 1, Channel: 7, Timer: 3
 */

uint8_t channels_resolution[LEDC_CHANNELS] = {0};

void ledcSetup(uint8_t chan, double freq, uint8_t bit_num, uint8_t pin)
{
    if (chan >= LEDC_CHANNELS || bit_num > LEDC_MAX_BIT_WIDTH)
    {
        ESP_LOGE(HW_TAG, "No more LEDC channels available! (maximum %u) or bit width too big (maximum %u)", LEDC_CHANNELS, LEDC_MAX_BIT_WIDTH);
    }

    uint8_t group = (chan / 8), channel = (chan % 8), timer = ((chan / 2) % 4);
    // setup timers

    ledc_timer_config_t ledc_timer = {
        .speed_mode = group,
        .timer_num = timer,
        .duty_resolution = bit_num,
        .freq_hz = freq,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    if (chan >= LEDC_CHANNELS)
    {
        ESP_LOGE(HW_TAG, "Chan error!");
    }

    // setup pins
    ledc_channel_config_t ledc_channel = {
        .speed_mode = group,
        .channel = channel,
        .timer_sel = timer,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = pin,
        .duty = LEDC_DUTY_IDLE,
        .hpoint = 0};

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    channels_resolution[chan] = bit_num;
}

void set_PWM(uint8_t chan, uint32_t pwm)
{
    if(chan >= LEDC_CHANNELS){
        return;
    }

    uint8_t group=(chan/8), channel=(chan%8);

    //Fixing if all bits in resolution is set = LEDC FULL ON
    uint32_t max_duty = (1 << channels_resolution[chan]) - 1;
    if(pwm == max_duty){
        pwm = max_duty + 1;
    }

    ESP_ERROR_CHECK(ledc_set_duty(group, channel, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(group, channel));
}




void stop_pwm()
{
    // m1****************************************************************************
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE, MOTOR_1_IN_1, LEDC_DUTY_IDLE));
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE, MOTOR_1_IN_2, LEDC_DUTY_IDLE));

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_1_IN_1));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_1_IN_2));

    // m2****************************************************************************
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE, MOTOR_2_IN_1, LEDC_DUTY_IDLE));
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE, MOTOR_2_IN_2, LEDC_DUTY_IDLE));

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_2_IN_1));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_2_IN_2));

    // m3****************************************************************************
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE, MOTOR_3_IN_1, LEDC_DUTY_IDLE));
    ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE, MOTOR_3_IN_2, LEDC_DUTY_IDLE));

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_3_IN_1));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_3_IN_2));

    // m4****************************************************************************
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, MOTOR_4_IN_1, LEDC_DUTY_IDLE));
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, MOTOR_4_IN_2, LEDC_DUTY_IDLE));

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_4_IN_1));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_4_IN_2));

    // m5****************************************************************************
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_1, LEDC_DUTY_IDLE));
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_2, LEDC_DUTY_IDLE));

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_1));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_5_IN_2));
}