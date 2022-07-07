

#include <stdint.h>
#include "esp_log.h"
#include "crc16.h"
#include <string.h>
#include "esp_spp_api.h"
#include "HW.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#define retract_time_ms 1000


//BT ID data
static const uint8_t reserved           = 0x00; //
static const uint8_t get_state          = 0x01; //online/offline
static const uint8_t get_state_info     = 0x02; //info
static const uint8_t one_step           = 0x03; // next action
static const uint8_t session_info       = 0x04;
static const uint8_t calibration        = 0x05;
static const uint8_t pause              = 0x06;
static const uint8_t stop               = 0x07;
static const uint8_t set_tactor         = 0x08;
static const uint8_t CMD_MOVEMENT_TIME  = 0x09;
static const uint8_t ACK                = 0x10;
static const uint8_t action_ack         = 0x11;
static const uint8_t state_info         = 0x12;


static const char* BT_TAG = "BT_parse_module";
static uint8_t  CYCLES_PER_BLOCK = 5;

struct __attribute__((packed, aligned(1))) bt_packet{
    uint8_t ID;
    uint8_t length;
    uint8_t *payload;
    unsigned short crc16;
};

struct session{
        uint8_t *position;
        uint8_t *act_time;
        uint8_t *idle_time;
        uint8_t *delay_time;
        uint8_t *pwm;
};



void parse_data_from_bt(uint8_t *data, uint16_t *len, uint32_t handle);
void parse_bt_packet(struct bt_packet *rx_packet, uint32_t handle);
void send_to_bt(struct bt_packet *tx_packet, uint32_t handle);
void send_ACK(uint32_t handle);
void num2permutation(uint8_t CYCLES_PER_BLOCK, uint8_t data, uint8_t *return_Arr);
uint8_t fact(uint8_t num);
void block_exec(struct session *incoming_session);
void send_session_ack(uint8_t *position, uint32_t handle);
void send_batt_data(uint32_t handle);
uint8_t get_pwm_from_nvs_by_position(uint8_t position);
void set_data_to_nvs_by_position(uint8_t position, uint8_t pwm);
void set_data_to_nvs_tactors(uint8_t *new_tactors);
void get_data_from_nvs_tactors(uint8_t *tactors_array);
void set_data_to_motor_move(uint8_t *new_retract, uint8_t *new_forward);
void get_data_from_motor_move(uint8_t *saved_retract, uint8_t *saved_forward);