struct session{
    uint8_t position; //finger
    uint8_t act_time; //sec
    uint8_t idle_time; //sec
    uint8_t PWM; //0-100%
};

struct calibration_data_pwm{
    uint8_t channel_1_pwm_low;
    uint8_t channel_1_pwm_high;

    uint8_t channel_2_pwm_low;
    uint8_t channel_2_pwm_high;

    uint8_t channel_3_pwm_low;
    uint8_t channel_3_pwm_high;

    uint8_t channel_4_pwm_low;
    uint8_t channel_4_pwm_high;

    uint8_t channel_5_pwm_low;
    uint8_t channel_5_pwm_high;
};