

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

struct calibration_data_sensibility{
    uint8_t channel_1_sens_low;
    uint8_t channel_1_sens_high;

    uint8_t channel_1_sens_low;
    uint8_t channel_2_sens_high;

    uint8_t channel_3_sens_low;
    uint8_t channel_3_sens_high;

    uint8_t channel_4_sens_low;
    uint8_t channel_4_sens_high;

    uint8_t channel_5_sens_low;
    uint8_t channel_5_sens_high;
};

void get_session_from_packet();