#include <stdint.h>
#include "esp_log.h"
#include <string.h>
#include "HW.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

//#define CYCLES_PER_BLOCK 5

static const char* S_TAG = "Session_control_module";



struct calibration_data_pwm{
    uint8_t channel_1_pwm_min;
    uint8_t channel_1_pwm_max;

    uint8_t channel_2_pwm_min;
    uint8_t channel_2_pwm_max;

    uint8_t channel_3_pwm_min;
    uint8_t channel_3_pwm_max;

    uint8_t channel_4_pwm_min;
    uint8_t channel_4_pwm_max;

    uint8_t channel_5_pwm_min;
    uint8_t channel_5_pwm_max;
};

struct calibration_data_sensibility{
    uint8_t channel_1_sens_low;
    uint8_t channel_1_sens_high;

    uint8_t channel_2_sens_low;
    uint8_t channel_2_sens_high;

    uint8_t channel_3_sens_low;
    uint8_t channel_3_sens_high;

    uint8_t channel_4_sens_low;
    uint8_t channel_4_sens_high;

    uint8_t channel_5_sens_low;
    uint8_t channel_5_sens_high;
};
/*
struct pwm{
        uint8_t pwm1;
        uint8_t pwm2;
        uint8_t pwm3;
        uint8_t pwm4;
        uint8_t pwm5;
};*/
/*
struct session{
        uint8_t *position;
        uint8_t *act_time;
        uint8_t *idle_time;
        uint8_t *delay_time;
        uint8_t pwm;
};
*/

void stop_session();
void pause_session();
void block_exec(struct session *incoming_session);

