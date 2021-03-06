#include "BT_to_Session.h"

void parse_data_from_bt(uint8_t *data, uint16_t *length, uint32_t handle)
{
    ESP_LOG_BUFFER_CHAR(BT_TAG, "Incoming packet: ", 17);
    esp_log_buffer_hex("", data, length[0]);

    uint16_t len = length[0];
    struct bt_packet rx_packet;
    unsigned short calcd_crc = crc16Calc(data, len - 2);

    rx_packet.crc16 = data[len - 2] << 8 | (data[len - 1]);

    if (calcd_crc != rx_packet.crc16)
    {
        ESP_LOGE(BT_TAG, "CRC ERROR! Expected =%0x, got =%0x", calcd_crc, rx_packet.crc16);
        return;
    }

    rx_packet.ID = data[0];
    rx_packet.length = data[1];

    if (len > 4)
    {
        uint8_t payload_size = len - 4;
        rx_packet.payload = calloc(payload_size, sizeof(uint8_t));
        if (!rx_packet.payload)
        {
            ESP_LOGE(BT_TAG, "Payload malloc error!");
            return;
        }
        uint8_t j = 0;
        for (uint8_t i = 2; i < len - 2; i++)
        {
            rx_packet.payload[j] = data[i];
            j++;
        }
    }

    parse_bt_packet(&rx_packet, handle);
}

void parse_bt_packet(struct bt_packet *rx_packet, uint32_t handle)
{
    struct bt_packet tx_packet;
    struct session incoming_session;
    switch (rx_packet->ID)
    {

    case reserved:
        break;

    case get_state:

        send_ACK(handle);
        break;

    case get_state_info:

        tx_packet.ID = 0x12;
        tx_packet.length = 0x04;
        uint8_t buf_GSI[1];
        buf_GSI[0] = tx_packet.ID;
        buf_GSI[1] = tx_packet.length;
        tx_packet.crc16 = crc16Calc(buf_GSI, 2);
        send_to_bt(&tx_packet, handle);
        break;

    case one_step:
        incoming_session.position       = calloc(1, sizeof(uint8_t));
        incoming_session.act_time       = calloc(1, sizeof(uint8_t));
        incoming_session.idle_time      = calloc(1, sizeof(uint8_t));
        incoming_session.pwm            = calloc(1, sizeof(uint8_t));
        incoming_session.delay_time     = calloc(1, sizeof(uint8_t));

        incoming_session.position       = &rx_packet->payload[0];
        incoming_session.act_time       = &rx_packet->payload[1];
        incoming_session.idle_time      = &rx_packet->payload[2];
        incoming_session.pwm            = &rx_packet->payload[3];
        
        block_exec(&incoming_session);
        send_session_ack(&incoming_session.position[0], handle);

        break;

    case session_info:
        ;
        uint8_t total_block_num = rx_packet->payload[0];
        uint8_t unique_block_num = rx_packet->payload[1];
        uint8_t total_cycles_num = total_block_num * CYCLES_PER_BLOCK;

        incoming_session.position       = calloc(total_cycles_num, sizeof(uint8_t));
        incoming_session.delay_time     = calloc(total_cycles_num, sizeof(uint8_t));
        incoming_session.act_time       = calloc(total_cycles_num, sizeof(uint8_t));
        incoming_session.idle_time      = calloc(total_cycles_num, sizeof(uint8_t));

        for (uint8_t i = 0; i < total_block_num; i++)
        {
            // get positions (fingers) array
            uint8_t *position;
            position = calloc(CYCLES_PER_BLOCK, sizeof(uint8_t));
            num2permutation(CYCLES_PER_BLOCK, rx_packet->payload[total_block_num + 2 + i], position);

            uint8_t current_block_position = rx_packet->payload[2 + i];

            for (uint8_t j = 0; j < CYCLES_PER_BLOCK; j++)
            {
                incoming_session.position[i * CYCLES_PER_BLOCK + j] = position[j];
                if (incoming_session.position[i * CYCLES_PER_BLOCK + j] == 0)
                {
                    incoming_session.position[i * CYCLES_PER_BLOCK + j] = 5;
                }
                incoming_session.delay_time[i * CYCLES_PER_BLOCK + j]   = rx_packet->payload[2 + (2 * total_block_num) + ((current_block_position - 1) * 3)] == 0x03 ? 3 : 0;
                incoming_session.act_time[i * CYCLES_PER_BLOCK + j]     = rx_packet->payload[2 + (2 * total_block_num) + ((current_block_position - 1) * 3) + 1];
                incoming_session.idle_time[i * CYCLES_PER_BLOCK + j]    = rx_packet->payload[2 + (2 * total_block_num) + ((current_block_position - 1) * 3) + 2];
            }
        }

        struct session next_block;
        for (size_t k = 0; k < total_cycles_num; k++)
        {
            next_block.position     = &incoming_session.position[k];
            next_block.delay_time   = &incoming_session.delay_time[k];
            next_block.act_time     = &incoming_session.act_time[k];
            next_block.idle_time    = &incoming_session.idle_time[k];

            uint8_t pwm = get_pwm_from_nvs_by_position(next_block.position[0]);
            next_block.pwm = &pwm;
            
            send_session_ack(&next_block.position[0], handle);
            block_exec(&next_block);
        }

        break;

    case calibration:
        set_data_to_nvs_by_position(rx_packet->payload[0], rx_packet->payload[1]);
        send_batt_data(handle);
        send_ACK(handle);
        break;

    case pause:
        stop_pwm();
        send_batt_data(handle);
        send_ACK(handle);
        break;

    case stop:
        stop_pwm();
        send_batt_data(handle);
        send_ACK(handle);
        break;

    case set_tactor:
    ;
        uint8_t *new_tactors;
        new_tactors = calloc(5, sizeof(uint8_t));

        for (uint8_t i = 0; i < 5; i++)
        {
           new_tactors[i] = rx_packet->payload[i];
        }

        set_data_to_nvs_tactors(new_tactors);
        send_batt_data(handle);
        send_ACK(handle);
        break;

    case CMD_MOVEMENT_TIME:
        ; 
        uint8_t *new_retract, *new_forward;
        new_retract = calloc(5, sizeof(uint8_t));//for future 
        new_forward = calloc(5, sizeof(uint8_t));

        for(uint8_t i = 0; i<5; i++){
            new_forward[i] = (rx_packet->payload[0]+1);
            new_retract[i] = (rx_packet->payload[1]+1);
        }

        set_data_to_motor_move(new_retract, new_forward);
        send_batt_data(handle);
        send_ACK(handle);
        break;

    case ACK:
        send_batt_data(handle);
        send_ACK(handle);
        break;

    case action_ack:
        break;

    case state_info:
        send_batt_data(handle);
        send_ACK(handle);
        break;

    default:
        break;
    }
}

void send_session_ack(uint8_t *position, uint32_t handle)
{
    struct bt_packet tx_packet;
    tx_packet.ID = 0x11;
    tx_packet.length = 0x05;
    tx_packet.payload = calloc(sizeof(uint8_t), sizeof(uint8_t));
    memcpy(tx_packet.payload, position, sizeof(uint8_t));
    uint8_t buf[4];
    buf[0] = tx_packet.ID;
    buf[1] = tx_packet.length;
    buf[2] = tx_packet.payload[0];
    tx_packet.crc16 = crc16Calc(buf, 2);
    send_to_bt(&tx_packet, handle);
}


void send_batt_data(uint32_t handle)
{
    struct bt_packet tx_packet;
    tx_packet.ID = 0x12;
    tx_packet.length = 0x06;
    tx_packet.payload = calloc(sizeof(uint8_t), sizeof(uint8_t));
    uint32_t batt_voltage = ((get_ADC_data(ADC1, batt_chan)*31) / 11) / 10; //divider 20k/11k so adc = voltage*11000/(20000+11000) so 31*adc=11*voltage. Divide by 10 cause NRF variation
    printf("BUTT: %d\n", batt_voltage);
    memcpy(tx_packet.payload, &batt_voltage, sizeof(uint16_t));
    uint8_t buf[4];
    buf[0] = tx_packet.ID;
    buf[1] = tx_packet.length;
    buf[2] = tx_packet.payload[0];
    buf[3] = tx_packet.payload[1];
    tx_packet.crc16 = crc16Calc(buf, 4);
    send_to_bt(&tx_packet, handle);
}

void send_ACK(uint32_t handle)
{
    struct bt_packet tx_packet;
    tx_packet.ID = ACK;
    tx_packet.length = 0x04;
    uint8_t buf[2];
    buf[0] = tx_packet.ID;
    buf[1] = tx_packet.length;
    tx_packet.crc16 = crc16Calc(buf, 2);
    send_to_bt(&tx_packet, handle);
}

void send_to_bt(struct bt_packet *tx_packet, uint32_t handle)
{

    if (tx_packet->length == 4)
    { // to do:
        uint8_t packet[4];
        packet[0] = tx_packet->ID;
        packet[1] = tx_packet->length;
        packet[2] = tx_packet->crc16;
        packet[3] = tx_packet->crc16 >> 8;
        ESP_ERROR_CHECK(esp_spp_write(handle, tx_packet->length, packet));
    }

    if (tx_packet->length == 5)
    { 
        uint8_t packet[5];
        packet[0] = tx_packet->ID;
        packet[1] = tx_packet->length;
        packet[2] = tx_packet->payload[0];
        packet[3] = tx_packet->crc16;
        packet[4] = tx_packet->crc16 >> 8;
        ESP_ERROR_CHECK(esp_spp_write(handle, tx_packet->length, packet));
    }
    if (tx_packet->length == 6){
        uint8_t packet[6];
        packet[0] = tx_packet->ID;
        packet[1] = tx_packet->length;
        packet[2] = tx_packet->payload[0];
        packet[3] = tx_packet->payload[1];
        packet[4] = tx_packet->crc16;
        packet[5] = tx_packet->crc16 >> 8;
        ESP_ERROR_CHECK(esp_spp_write(handle, tx_packet->length, packet));
    }
}

void num2permutation(uint8_t CYCLES_PER_BLOCK, uint8_t data, uint8_t *return_Arr)
{
    uint8_t n = CYCLES_PER_BLOCK;
    uint8_t k = data - 1;
    bool *was;
    was = calloc(CYCLES_PER_BLOCK, sizeof(bool));

    for (uint8_t i = 1; i <= n; i++)
    {
        uint8_t alreadyWas = k / fact(n - i);

        k = k % fact(n - i);
        uint8_t curFree = 0;

        for (uint8_t j = 1; j <= n; j++)
        {
            if (was[j] == false)
            {
                curFree++;

                if (curFree == alreadyWas + 1)
                {
                    return_Arr[i - 1] = j;
                    was[j] = true;
                }
            }
        }
    }
}

uint8_t fact(uint8_t num)
{
    if (num <= 1)
        return 1;
    else
        return num * fact(num - 1);
}

//map pwm
uint8_t pwm_out_min = 127;
uint8_t pwm_out_max = 0;
uint8_t pwm_in_min = 0;
uint8_t pwm_in_max = 10;
long pwm_map(long x, long in_min, long in_max)
{
  return (x - in_min) * (pwm_out_max - pwm_out_min) / (in_max - in_min) + pwm_out_min;
}



void block_exec(struct session *incoming_session)
{

    ESP_LOG_BUFFER_CHAR(BT_TAG, "Session", 8);
    ESP_LOG_BUFFER_CHAR(BT_TAG, "PWM", 4);
    esp_log_buffer_hex("", incoming_session->pwm, 1);
    ESP_LOG_BUFFER_CHAR(BT_TAG, "Act time", 9);
    esp_log_buffer_hex("", incoming_session->act_time, 1);
    ESP_LOG_BUFFER_CHAR(BT_TAG, "Idle time", 10);
    esp_log_buffer_hex("", incoming_session->idle_time, 1);
    ESP_LOG_BUFFER_CHAR(BT_TAG, "Delay time", 14);
    esp_log_buffer_hex("", incoming_session->delay_time, 1);
    ESP_LOG_BUFFER_CHAR(BT_TAG, "Position", 9);
    esp_log_buffer_hex("", incoming_session->position, 1);
    
    
    uint8_t *tactors_array, *retract_time, *forward_time;
    tactors_array = calloc(5, sizeof(uint8_t));
    retract_time = calloc(5, sizeof(uint8_t));
    forward_time = calloc(5, sizeof(uint8_t));

    get_data_from_nvs_tactors(tactors_array);
    get_data_from_motor_move(retract_time, forward_time);

    ESP_LOG_BUFFER_CHAR(BT_TAG, "Saved Position", sizeof("Saved Position"));
    esp_log_buffer_hex("", &tactors_array[(int)incoming_session->position[0]-1], 1);
    uint8_t pwm1 = pwm_map(incoming_session->pwm[0], pwm_in_min, pwm_in_max);
    printf("PWM 1: %d\n", pwm1);
    switch (tactors_array[(int)incoming_session->position[0]-1])
    {
    case 01:
        vTaskDelay(((incoming_session->delay_time[0]) * 1000) / portTICK_RATE_MS);
        set_motors_en(DRIVERS_ON);
        set_PWM(MOTOR_1_IN_1, pwm1);                   //push   
        set_PWM(MOTOR_1_IN_2, 255-pwm1);                   // drv8601 is differential
        vTaskDelay(forward_time[0]*250/ portTICK_RATE_MS);
        set_motors_en(DRIVERS_OFF);                   
        vTaskDelay(((incoming_session->act_time[0]) * 1000) / portTICK_RATE_MS); // wait

        set_motors_en(DRIVERS_ON);
        set_PWM(MOTOR_1_IN_1, 255-pwm1);                                              // retract
        set_PWM(MOTOR_1_IN_2, pwm1);
        vTaskDelay(retract_time[0]*250 / portTICK_RATE_MS);

        set_PWM(MOTOR_1_IN_1, LEDC_DUTY_IDLE); // set idle
        set_PWM(MOTOR_1_IN_2, LEDC_DUTY_IDLE);
        set_motors_en(DRIVERS_OFF);//cool off
        vTaskDelay(((incoming_session->idle_time[0]) * 1000) / portTICK_RATE_MS);
        break;

    case 02:
        vTaskDelay(((incoming_session->delay_time[0]) * 1000) / portTICK_RATE_MS);
        set_motors_en(DRIVERS_ON);
        set_PWM(MOTOR_2_IN_1, pwm1);                   //push   
        set_PWM(MOTOR_2_IN_2, 255 - pwm1);
        vTaskDelay(forward_time[1]*250/ portTICK_RATE_MS);
        set_motors_en(DRIVERS_OFF);                        
        vTaskDelay(((incoming_session->act_time[0]) * 1000) / portTICK_RATE_MS);
         // wait
        set_motors_en(DRIVERS_ON);
        set_PWM(MOTOR_2_IN_1, 255 - pwm1);                                              // retract
        set_PWM(MOTOR_2_IN_2, pwm1);
        vTaskDelay(retract_time[1]*250 / portTICK_RATE_MS);

        set_PWM(MOTOR_2_IN_1, LEDC_DUTY_IDLE); // set idle
        set_PWM(MOTOR_2_IN_2, LEDC_DUTY_IDLE);
        set_motors_en(DRIVERS_OFF);
        vTaskDelay(((incoming_session->idle_time[0]) * 1000) / portTICK_RATE_MS);
        break;
    case 03:
        vTaskDelay(((incoming_session->delay_time[0]) * 1000) / portTICK_RATE_MS);
        set_motors_en(DRIVERS_ON);
        set_PWM(MOTOR_3_IN_1, pwm1);                   //push   
        set_PWM(MOTOR_3_IN_2, 255 - pwm1);
        vTaskDelay(forward_time[2]*250/ portTICK_RATE_MS);
        set_motors_en(DRIVERS_OFF);                       
        vTaskDelay(((incoming_session->act_time[0]) * 1000) / portTICK_RATE_MS); // wait
        
        set_motors_en(DRIVERS_ON);
        set_PWM(MOTOR_3_IN_1, 255 - pwm1);                                              // retract
        set_PWM(MOTOR_3_IN_2, pwm1);
        vTaskDelay(retract_time[2]*250 / portTICK_RATE_MS);

        set_PWM(MOTOR_3_IN_1, LEDC_DUTY_IDLE); // set idle
        set_PWM(MOTOR_3_IN_2, LEDC_DUTY_IDLE);
        set_motors_en(DRIVERS_OFF);
        vTaskDelay(((incoming_session->idle_time[0]) * 1000) / portTICK_RATE_MS);
        break;
    case 04:
        vTaskDelay(((incoming_session->delay_time[0]) * 1000) / portTICK_RATE_MS);
        set_motors_en(DRIVERS_ON);
        set_PWM(MOTOR_4_IN_1, pwm1);                   //push   
        set_PWM(MOTOR_4_IN_2, 255 - pwm1);
        vTaskDelay(forward_time[3]*250/ portTICK_RATE_MS);
        set_motors_en(DRIVERS_OFF);                        
        vTaskDelay(((incoming_session->act_time[0]) * 1000) / portTICK_RATE_MS); // wait
        
        set_motors_en(DRIVERS_ON);
        set_PWM(MOTOR_4_IN_1, 255-pwm1);                                              // retract
        set_PWM(MOTOR_4_IN_2, pwm1);
        vTaskDelay(retract_time[3]*250 / portTICK_RATE_MS);
        
        set_PWM(MOTOR_4_IN_1, LEDC_DUTY_IDLE); // set idle
        set_PWM(MOTOR_4_IN_2, LEDC_DUTY_IDLE);
        set_motors_en(DRIVERS_OFF);
        vTaskDelay(((incoming_session->idle_time[0]) * 1000) / portTICK_RATE_MS);
        break;
    case 05:
        vTaskDelay(((incoming_session->delay_time[0]) * 1000) / portTICK_RATE_MS);
        set_motors_en(DRIVERS_ON);
        set_PWM(MOTOR_5_IN_1, pwm1);                   //push   
        set_PWM(MOTOR_5_IN_2, 255 - pwm1);
        vTaskDelay(forward_time[4]*250/ portTICK_RATE_MS);
        set_motors_en(DRIVERS_OFF);                       
        
        vTaskDelay(((incoming_session->act_time[0]) * 1000) / portTICK_RATE_MS); // wait
        set_motors_en(DRIVERS_ON);
        set_PWM(MOTOR_5_IN_1, 255 - pwm1);                                              // retract
        set_PWM(MOTOR_5_IN_2, pwm1);
        vTaskDelay(retract_time[4]*250 / portTICK_RATE_MS);
        
        set_PWM(MOTOR_5_IN_1, LEDC_DUTY_IDLE); // set idle
        set_PWM(MOTOR_5_IN_2, LEDC_DUTY_IDLE);
        set_motors_en(DRIVERS_OFF);
        vTaskDelay(((incoming_session->idle_time[0]) * 1000) / portTICK_RATE_MS);
        break;

    default:
        ESP_LOG_BUFFER_CHAR(BT_TAG, "Wrong position!", 16);

        break;
    }
    set_motors_en(DRIVERS_OFF);
}

uint8_t get_pwm_from_nvs_by_position(uint8_t position)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
    uint8_t pwm = 0xFF; // default = max
    switch (position)
    {
    case 1:
        err = nvs_get_u8(nvs_handle, "pwm1", &pwm);
        switch (err)
        {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            nvs_set_u8(nvs_handle, "pwm1", pwm);
            break;
        default:
            ESP_ERROR_CHECK(err);
        }
        break;

    case 2:
        err = nvs_get_u8(nvs_handle, "pwm2", &pwm);
        switch (err)
        {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            nvs_set_u8(nvs_handle, "pwm2", pwm);
            break;
        default:
            ESP_ERROR_CHECK(err);
        }
        break;

    case 3:
        err = nvs_get_u8(nvs_handle, "pwm3", &pwm);
        switch (err)
        {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            nvs_set_u8(nvs_handle, "pwm3", pwm);
            break;
        default:
            ESP_ERROR_CHECK(err);
        }
        break;

    case 4:
        err = nvs_get_u8(nvs_handle, "pwm4", &pwm);
        switch (err)
        {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            nvs_set_u8(nvs_handle, "pwm4", pwm);
            break;
        default:
            ESP_ERROR_CHECK(err);
        }
        break;

    case 5:
        err = nvs_get_u8(nvs_handle, "pwm5", &pwm);
        switch (err)
        {
        case ESP_OK:
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            nvs_set_u8(nvs_handle, "pwm5", pwm);
            break;
        default:
            ESP_ERROR_CHECK(err);
        }
        break;

    default:
        break;
    }
    
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);

    return pwm;
}

void set_data_to_nvs_by_position(uint8_t position, uint8_t pwm)
{
    //incoming from 0 to 10
    nvs_handle_t nvs_handle;
    uint8_t pwm_to_save = pwm;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
    switch (position)
    {
    case 1:
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "pwm1", pwm_to_save));
        break;

    case 2:
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "pwm2", pwm_to_save));
        break;

    case 3:
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "pwm3", pwm_to_save));
        break;

    case 4:
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "pwm4", pwm_to_save));
        break;

    case 5:
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "pwm5", pwm_to_save));
        break;

    default:
        break;
    }
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);
}


void set_data_to_nvs_tactors(uint8_t *new_tactors){
    nvs_handle_t nvs_handle;
    size_t req_size = 5;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
    nvs_set_blob(nvs_handle,"Tactor data", new_tactors, req_size);
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);
}


void get_data_from_nvs_tactors(uint8_t *tactors_array){

    nvs_handle_t nvs_handle;
    size_t *length;
    length = calloc(1, sizeof(size_t));
    *length = 5;
    tactors_array[0] = 1;
    tactors_array[1] = 2;
    tactors_array[2] = 3;
    tactors_array[3] = 4;
    tactors_array[4] = 5;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
    ESP_ERROR_CHECK(nvs_get_blob(nvs_handle,"Tactor data", tactors_array, length));
    nvs_close(nvs_handle);
}

void set_data_to_motor_move(uint8_t *new_retract, uint8_t *new_forward){
    nvs_handle_t nvs_handle;
    size_t req_size = 5;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
    nvs_set_blob(nvs_handle,"Retract data", new_retract, req_size);
    nvs_set_blob(nvs_handle,"Forward data", new_forward, req_size);
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);
}

void get_data_from_motor_move(uint8_t *saved_retract, uint8_t *saved_forward){
    nvs_handle_t nvs_handle;
    esp_err_t err;
    size_t *length;
    length = calloc(1, sizeof(size_t));
    *length = 5;
    saved_retract[0] = 4;
    saved_retract[1] = 4;
    saved_retract[2] = 4;
    saved_retract[3] = 4;
    saved_retract[4] = 4; //default 4*250 ms

    saved_forward[0] = 4;
    saved_forward[1] = 4;
    saved_forward[2] = 4;
    saved_forward[3] = 4;
    saved_forward[4] = 4;

    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
    ESP_ERROR_CHECK(nvs_get_blob(nvs_handle,"Retract data", saved_retract, length));
    ESP_ERROR_CHECK(nvs_get_blob(nvs_handle,"Forward data", saved_forward, length));
    nvs_close(nvs_handle);
}