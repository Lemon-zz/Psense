#include "BT_parse.h"



void parse_data_from_bt(uint8_t *data, uint16_t *length, uint32_t handle){
    uint16_t len = length[0];
    esp_log_buffer_hex("",data, len);
    
    struct bt_packet rx_packet;

    unsigned short calcd_crc = crc16Calc(data, len-2);  
    
    rx_packet.crc16 = data[len-2]  << 8 | (data[len-1]);

    if(calcd_crc != rx_packet.crc16){
        ESP_LOGE(BT_TAG, "CRC ERROR! Expected =%0x, got =%0x", calcd_crc, rx_packet.crc16);
        return;
    }
    
    rx_packet.ID = data[0];
    /*
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Packet ID:",12);
    esp_log_buffer_hex("", &rx_packet.ID, 1);
    */

    rx_packet.length = data[1];
    /*
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Packet Length:",14);
    esp_log_buffer_hex("", &rx_packet.length, 1);
    */
    if(len > 4){
        uint8_t payload_size = len - 4;
        rx_packet.payload = calloc(payload_size,sizeof(uint8_t));
        if (!rx_packet.payload){
            ESP_LOGE(BT_TAG, "Payload malloc error!");
            return;
        }
        //memset(rx_packet.payload, 0, sizeof(uint8_t)*payload_size);
        uint8_t j = 0;
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
            ESP_LOG_BUFFER_CHAR(BT_TAG,"ID: Set next Step",20);
            ESP_LOG_BUFFER_CHAR(BT_TAG,"Payload:",10);
            esp_log_buffer_hex("", rx_packet->payload, sizeof(rx_packet->payload));
            //parse_test_session(rx_packet->payload);
            send_ACK(handle);
            break;
        
        case session_info:
            ESP_LOG_BUFFER_CHAR(BT_TAG,"ID: Session info",18);
            struct session incoming_session;
            uint8_t total_block_num         = rx_packet->payload[0];

            ESP_LOG_BUFFER_CHAR(BT_TAG,"total_block_num",18);
            esp_log_buffer_hex("", &total_block_num, sizeof(total_block_num));

            uint8_t unique_block_num        = rx_packet->payload[1];
            ESP_LOG_BUFFER_CHAR(BT_TAG,"unique_block_num",18);
            esp_log_buffer_hex("", &unique_block_num, sizeof(unique_block_num));

            uint8_t total_cycles_num        = total_block_num * CYCLES_PER_BLOCK;
            ESP_LOG_BUFFER_CHAR(BT_TAG,"total_cycles_num",18);
            esp_log_buffer_hex("", &total_cycles_num, sizeof(total_cycles_num));

            incoming_session.position       = calloc(total_cycles_num, sizeof(uint8_t));
            incoming_session.delay_time     = calloc(total_cycles_num, sizeof(uint8_t));
            incoming_session.act_time       = calloc(total_cycles_num, sizeof(uint8_t));
            incoming_session.idle_time      = calloc(total_cycles_num, sizeof(uint8_t));



            for (uint8_t i = 0; i < total_block_num; i++)
            {
                
            
                uint8_t current_block_position = rx_packet->payload[2+i]; 
                uint8_t position[CYCLES_PER_BLOCK*total_block_num];
               
               num2permutation(CYCLES_PER_BLOCK, rx_packet->payload[total_block_num+2+i], position);
               
               for (uint8_t j=0; j < CYCLES_PER_BLOCK; j++)
               {
                   incoming_session.position[i*CYCLES_PER_BLOCK+j]     = position[j];
                   incoming_session.delay_time[i*CYCLES_PER_BLOCK+j]   = rx_packet->payload[2 + (2 * total_block_num) + ((i % unique_block_num)*3)] == 0x03 ? 3:0;
                   incoming_session.act_time[i*CYCLES_PER_BLOCK+j]     = rx_packet->payload[2 + (2 * total_block_num) + ((i % unique_block_num) * 3) + 1 ];
                   incoming_session.idle_time[i*CYCLES_PER_BLOCK+j]    = rx_packet->payload[2 + (2 * total_block_num) + ((i % unique_block_num) * 3) + 2 ];
               }
            }
             



            for(uint8_t i =0; i<= total_block_num; i++){
                struct session next_block;
                next_block.position       = &incoming_session.position[i];
                next_block.delay_time     = &incoming_session.delay_time[i];
                next_block.act_time       = &incoming_session.act_time[i];
                next_block.idle_time      = &incoming_session.idle_time[i];
                uint8_t pwm = 100;
                next_block.pwm            = &pwm;
                block_exec(&next_block);
                ESP_LOG_BUFFER_CHAR(BT_TAG,"Next Session",18);
                //send_ACK(handle);
                tx_packet.ID = 0x11;
                tx_packet.length = 0x04;
                uint8_t buf_SI[1];
                buf_SI[0] = tx_packet.ID;
                buf_SI[1] = tx_packet.length;
                tx_packet.crc16 = crc16Calc(buf_SI, 2);
                send_to_bt(&tx_packet, handle);
            }

            
            break;
        
        case calibration:
            break;
        
        case pause:
            //pause_session();
            send_ACK(handle);
            break;
        
        case stop:
            //stop_session();
            send_ACK(handle);
            break;
        
        case ACK:
            send_ACK(handle);
            break;
        
        case action_ack:
            tx_packet.ID = 0x11;
            tx_packet.length = 0x04;
            uint8_t buf_AA[1];
            buf_AA[0] = tx_packet.ID;
            buf_AA[1] = tx_packet.length;
            tx_packet.crc16 = crc16Calc(buf_AA, 2);
            send_to_bt(&tx_packet, handle);
            break;
        
        case state_info:
            send_ACK(handle);
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
    /*
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Send:",6);
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Packet ID:",12);
    esp_log_buffer_hex("", &tx_packet->ID, 1);
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Packet length:",14);
    esp_log_buffer_hex("", &tx_packet->length, 1);
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Packet crc:",12);
    esp_log_buffer_hex("", &tx_packet->crc16, 2);
    */
    if(tx_packet->length == 4){ //to do: 
    uint8_t packet[3];
    packet[0] = tx_packet->ID;
    packet[1] = tx_packet->length;
    packet[2] = tx_packet->crc16;
    packet[3] = tx_packet->crc16>>8;
    
    ESP_ERROR_CHECK(esp_spp_write(handle, tx_packet->length, packet));
    }
}


void num2permutation(uint8_t CYCLES_PER_BLOCK, uint8_t data, uint8_t *return_Arr){
        
        uint8_t n = CYCLES_PER_BLOCK;
        uint8_t k = data - 1;
        bool *was;
        was = calloc(CYCLES_PER_BLOCK, sizeof(bool));
        

        for (uint8_t i = 1; i <= n; i++) {
            uint8_t alreadyWas = k / fact(n - i);

            k = k % fact(n - i);
            uint8_t curFree = 0;

            for (uint8_t j = 1; j <= n; j++) {
                if (was[j] == false) {
                    curFree++;
                    
                    if (curFree == alreadyWas + 1) {
                        
                        return_Arr[i - 1] = j;
                        
                        was[j] = true;
                    }
                }
            }
        }
    }


uint8_t fact(uint8_t num) {
        if (num <= 1)
            return 1;
        else
            return num * fact(num - 1);
}



void block_exec(struct session *incoming_session){

    ESP_LOG_BUFFER_CHAR(BT_TAG,"Session",8);
    ESP_LOG_BUFFER_CHAR(BT_TAG,"PWM",4);
    esp_log_buffer_hex("", incoming_session->pwm, 1);
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Act time",9);
    esp_log_buffer_hex("", incoming_session->act_time, 1);
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Idle time",10);
    esp_log_buffer_hex("", incoming_session->idle_time, 1);
    ESP_LOG_BUFFER_CHAR(BT_TAG,"Position",9);
    esp_log_buffer_hex("", incoming_session->position, 1);

    switch ((int)incoming_session->position[0])
    {
    case 01:
    ESP_LOG_BUFFER_CHAR(BT_TAG,"case 1",6);
        set_m1_1_pwm(incoming_session->pwm[0]);
        vTaskDelay(((incoming_session->act_time[0]) * 1000) / portTICK_RATE_MS);
        set_m1_1_pwm(0);
        set_m1_2_pwm(100);
        vTaskDelay(100 / portTICK_RATE_MS); //retract
        vTaskDelay(((incoming_session->idle_time[0]) * 1000) / portTICK_RATE_MS);
        break;
    case 02:
    ESP_LOG_BUFFER_CHAR(BT_TAG,"case 2",6);
        set_m2_1_pwm(incoming_session->pwm[0]);
        vTaskDelay(((incoming_session->act_time[0]) * 1000)/ portTICK_RATE_MS);
        set_m2_1_pwm(0);
        set_m2_2_pwm(100);
        vTaskDelay(100 / portTICK_RATE_MS); //retract
        vTaskDelay(((incoming_session->idle_time[0] * 1000) / portTICK_RATE_MS));
    break;
    case 03:
    ESP_LOG_BUFFER_CHAR(BT_TAG,"case 3",6);
        set_m3_1_pwm(incoming_session->pwm[0]);
        vTaskDelay(((incoming_session->act_time[0]) * 1000)/ portTICK_RATE_MS);
        set_m3_1_pwm(0);
        set_m3_2_pwm(100);
        vTaskDelay(100 / portTICK_RATE_MS); //retract
        vTaskDelay(((incoming_session->idle_time[0] * 1000) / portTICK_RATE_MS));
    break;
    case 04:
    ESP_LOG_BUFFER_CHAR(BT_TAG,"case 4",6);
        set_m4_1_pwm(incoming_session->pwm[0]);
        vTaskDelay(((incoming_session->act_time[0]) * 1000)/ portTICK_RATE_MS);
        set_m4_1_pwm(0);
        set_m4_2_pwm(100);
        vTaskDelay(100 / portTICK_RATE_MS); //retract
        vTaskDelay(((incoming_session->idle_time[0] * 1000) / portTICK_RATE_MS));
    break;
    case 05:
    ESP_LOG_BUFFER_CHAR(BT_TAG,"case 5",6);
        set_motors_en(1);
        set_m5_1_pwm(255);
        set_m5_2_pwm(incoming_session->pwm[0]*25);
        vTaskDelay(((incoming_session->act_time[0]) * 1000)/ portTICK_RATE_MS);
        set_m5_1_pwm(10);
        set_m5_2_pwm(100);
        vTaskDelay(100 / portTICK_RATE_MS); //retract
        set_motors_en(0);
        vTaskDelay(((incoming_session->idle_time[0] * 1000) / portTICK_RATE_MS));
    break;
        
    default:
    ESP_LOG_BUFFER_CHAR(BT_TAG,"case d",6);
        //stop_pwm();
        break;
    }
    //stop_pwm();
}
