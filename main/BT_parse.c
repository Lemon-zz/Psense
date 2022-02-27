#include "BT_parse.h"
#include "Session_control.h"





void parse_data_from_bt(uint8_t *data, uint16_t len, uint32_t handle){
    //esp_log_buffer_hex("",data, len);
    
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

            send_ACK(handle);
            break;
        

        case get_state_info:
            ESP_LOG_BUFFER_CHAR(BT_TAG,"ID: Get State Info",18);
            tx_packet.ID = 0x12;
            tx_packet.length = 0x04;

            uint8_t buf_GSI[1];
            buf_GSI[0] = tx_packet.ID;
            buf_GSI[1] = tx_packet.length;
            tx_packet.crc16 = crc16Calc(buf_GSI, 2);
            send_to_bt(&tx_packet, handle);

            break;
        
        case next_step:
            ESP_LOG_BUFFER_CHAR(BT_TAG,"ID: Set next Step",18);

            tx_packet.ID = 0x12;
            tx_packet.length = 0x04;
            uint8_t buf_SnS[1];
            buf_SnS[0] = tx_packet.ID;
            buf_SnS[1] = tx_packet.length;
            tx_packet.crc16 = crc16Calc(buf_SnS, 2);
            send_to_bt(&tx_packet, handle);
            break;
        
        case session_info:
            ESP_LOG_BUFFER_CHAR(BT_TAG,"ID: Session info",18);
            get_session_data(rx_packet->payload, rx_packet->length-4);
            tx_packet.ID = 0x11;
            tx_packet.length = 0x04;
            uint8_t buf_SI[1];
            buf_SI[0] = tx_packet.ID;
            buf_SI[1] = tx_packet.length;
            tx_packet.crc16 = crc16Calc(buf_SI, 2);
            send_to_bt(&tx_packet, handle);
            break;
        
        case calibration:
            break;
        
        case pause:
            break;
        
        case stop:
            break;
        
        case ACK:
            send_ACK(handle);
            break;
        
        case action_ack:
            send_ACK(handle);
            break;
        
        case state_info:
            break;

        default:
            break;
    }
}


void send_ACK(uint32_t handle){
            struct bt_packet tx_packet;
            tx_packet.ID = ACK;
            tx_packet.length = 0x04;
            uint8_t buf[1];
            buf[0] = tx_packet.ID;
            buf[1] = tx_packet.length;
            tx_packet.crc16 = crc16Calc(buf, 2);
            send_to_bt(&tx_packet, handle);
}

void send_to_bt(struct bt_packet *tx_packet, uint32_t handle){
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Send:",6);
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Packet ID:",12);
    esp_log_buffer_hex("", &tx_packet->ID, 1);
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Packet length:",14);
    esp_log_buffer_hex("", &tx_packet->length, 1);
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Packet crc:",12);
    esp_log_buffer_hex("", &tx_packet->crc16, 2);
    esp_log_buffer_hex("", ((uint8_t*) &tx_packet), 4);
    
    if(tx_packet->length == 4){ //to do: 
    uint8_t packet[3];
    packet[0] = tx_packet->ID;
    packet[1] = tx_packet->length;
    packet[2] = tx_packet->crc16;
    packet[3] = tx_packet->crc16>>8;
    
    esp_spp_write(handle, tx_packet->length, packet);
    }
}
