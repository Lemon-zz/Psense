#include "BT_parse.h"






void parse_data_from_bt(uint8_t *data, uint16_t len, uint32_t handle){
    esp_log_buffer_hex("",data, len);
    
    struct bt_packet rx_packet;

    unsigned short calcd_crc = crc16Calc(data, len-2);  
    
    rx_packet.crc16 = data[len-2]  << 8 | (data[len-1]);

    if(calcd_crc != rx_packet.crc16){
        ESP_LOGE(BT_TAG, "CRC ERROR! Expected =%0x, got =%0x", calcd_crc, rx_packet.crc16);
        return;
    }
    
    rx_packet.ID = data[0];
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Packet ID:",12);
    esp_log_buffer_hex("", &rx_packet.ID, 1);

    rx_packet.length = data[1];
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Packet Length:",14);
    esp_log_buffer_hex("", &rx_packet.length, 1);

    if(len > 4){
        uint8_t payload_size = len - 4;
        rx_packet.payload = malloc(sizeof(uint8_t)*payload_size);
        if (!rx_packet.payload){
            ESP_LOGE(BT_TAG, "Payload malloc error!");
            return;
        }
        memset(rx_packet.payload, 0, sizeof(uint8_t)*payload_size);
        int j = 0;
        for(uint8_t i=2; i<len-2; i++){
            rx_packet.payload[j] = data[i];
            j++;
        }
    }

    parse_bt_packet(&rx_packet, handle);
}




void parse_bt_packet(struct bt_packet *rx_packet, uint32_t handle){
    struct bt_packet tx_packet;
     switch(rx_packet->ID){
        case reserved:
            ESP_LOG_BUFFER_CHAR(BT_TAG,"ID: Reserved",14);
            break;


        case get_state:
            ESP_LOG_BUFFER_CHAR(BT_TAG,"ID: Get State",14);
            tx_packet.ID = ACK;
            tx_packet.length = 0x04;
            unsigned char buf = tx_packet.ID  << 8 | tx_packet.length;
            tx_packet.crc16 = crc16Calc(&buf, 2);
            send_to_bt(&tx_packet, handle);
            break;
        

        case get_state_info:
            ESP_LOG_BUFFER_CHAR(BT_TAG,"ID: Get State Info",18);
            tx_packet.ID = 0x12;
            tx_packet.length = 0x04;
            unsigned char buf_GSI = tx_packet.ID  << 8 | tx_packet.length;
            tx_packet.crc16 = crc16Calc(&buf_GSI, 2);
            send_to_bt(&tx_packet, handle);
            break;
        
        case next_step:
            ESP_LOG_BUFFER_CHAR(BT_TAG,"ID: Set next Step",18);

            tx_packet.ID = 0x12;
            tx_packet.length = 0x04;
            unsigned char buf_SnS = tx_packet.ID  << 8 | tx_packet.length;
            tx_packet.crc16 = crc16Calc(&buf_SnS, 2);
            send_to_bt(&tx_packet, handle);
            break;
        
        case session_info:
            break;
        
        case calibration:
            break;
        
        case pause:
            break;
        
        case stop:
            break;
        
        case ACK:
            break;
        
        case action_ack:
            break;
        
        case state_info:
            break;

        default:
            break;
    }
}


void send_to_bt(struct bt_packet *tx_packet, uint32_t handle){
    esp_spp_write(handle, tx_packet->length, (uint8_t*) tx_packet);
}