

#include <stdint.h>
#include "esp_log.h"
#include "crc16.h"
#include <string.h>
#include "esp_spp_api.h"

//BT ID data
static const uint8_t reserved       = 0x00; //
static const uint8_t get_state      = 0x01; //online/offline
static const uint8_t get_state_info = 0x02; //info
static const uint8_t next_step      = 0x03; // next action
static const uint8_t session_info   = 0x04;
static const uint8_t calibration    = 0x05;
static const uint8_t pause          = 0x06;
static const uint8_t stop           = 0x07;
static const uint8_t ACK            = 0x10;
static const uint8_t action_ack     = 0x11;
static const uint8_t state_info     = 0x12;


static const char* BT_TAG = "BT_parse_module";

struct __attribute__((packed, aligned(1))) bt_packet{
    uint8_t ID;
    uint8_t length;
    uint8_t *payload;
    unsigned short crc16;
};

void parse_data_from_bt(uint8_t *data, uint16_t len, uint32_t handle);
void parse_bt_packet(struct bt_packet *rx_packet, uint32_t handle);
void send_to_bt(struct bt_packet *tx_packet, uint32_t handle);
void send_ACK(uint32_t handle);