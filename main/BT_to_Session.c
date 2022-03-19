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
        ESP_LOG_BUFFER_CHAR(BT_TAG, "ID: Reserved", 14);
        break;

    case get_state:
        ESP_LOG_BUFFER_CHAR(BT_TAG, "ID: Get State", 14);
        send_ACK(handle);
        break;

    case get_state_info:
        ESP_LOG_BUFFER_CHAR(BT_TAG, "ID: Get State Info", 18);
        tx_packet.ID = 0x12;
        tx_packet.length = 0x04;
        uint8_t buf_GSI[1];
        buf_GSI[0] = tx_packet.ID;
        buf_GSI[1] = tx_packet.length;
        tx_packet.crc16 = crc16Calc(buf_GSI, 2);
        send_to_bt(&tx_packet, handle);
        break;

    case one_step:
        ESP_LOG_BUFFER_CHAR(BT_TAG, "ID: Set one Step", sizeof("ID: Set one Step"));
        ESP_LOG_BUFFER_CHAR(BT_TAG, "Payload:", 10);
        esp_log_buffer_hex("", rx_packet->payload, sizeof(rx_packet->payload));

        incoming_session.position = &rx_packet->payload[0];
        incoming_session.act_time = &rx_packet->payload[1];
        incoming_session.idle_time = &rx_packet->payload[2];
        incoming_session.pwm = &rx_packet->payload[3];
        *incoming_session.pwm = *incoming_session.pwm * 25;
        block_exec(&incoming_session);
        send_session_ack(&incoming_session.position[0], handle);

        break;

    case session_info:
        ESP_LOG_BUFFER_CHAR(BT_TAG, "ID: Session info", 18);

        uint8_t total_block_num = rx_packet->payload[0];
        uint8_t unique_block_num = rx_packet->payload[1];
        uint8_t total_cycles_num = total_block_num * CYCLES_PER_BLOCK;

        ESP_LOG_BUFFER_CHAR(BT_TAG, "total_block_num", 15);
        esp_log_buffer_hex("", &total_block_num, sizeof(total_block_num));
        ESP_LOG_BUFFER_CHAR(BT_TAG, "unique_block_num", 16);
        esp_log_buffer_hex("", &unique_block_num, sizeof(unique_block_num));
        ESP_LOG_BUFFER_CHAR(BT_TAG, "total_cycles_num", 16);
        esp_log_buffer_hex("", &total_cycles_num, sizeof(total_cycles_num));

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

            ESP_LOG_BUFFER_CHAR(BT_TAG, "Payload", 7);
            esp_log_buffer_hex("", &rx_packet->payload[total_block_num + 2 + i], sizeof(uint8_t));
            ESP_LOG_BUFFER_CHAR(BT_TAG, "Perm", 4);
            esp_log_buffer_hex("", &position[0], 5);

            for (uint8_t j = 0; j < CYCLES_PER_BLOCK; j++)
            {
                incoming_session.position[i * CYCLES_PER_BLOCK + j] = position[j];
                if (incoming_session.position[i * CYCLES_PER_BLOCK + j] == 0)
                {
                    incoming_session.position[i * CYCLES_PER_BLOCK + j] = 5;
                }
                incoming_session.delay_time[i * CYCLES_PER_BLOCK + j]   = rx_packet->payload[2 + (2 * total_block_num) + ((i % unique_block_num) * 3)] == 0x03 ? 3 : 0;
                incoming_session.act_time[i * CYCLES_PER_BLOCK + j]     = rx_packet->payload[2 + (2 * total_block_num) + ((i % unique_block_num) * 3) + 1];
                incoming_session.idle_time[i * CYCLES_PER_BLOCK + j]    = rx_packet->payload[2 + (2 * total_block_num) + ((i % unique_block_num) * 3) + 2];
            }
        }

        struct session next_block;
        for (size_t k = 0; k < total_cycles_num; k++)
        {
            next_block.position     = &incoming_session.position[k];
            next_block.delay_time   = &incoming_session.delay_time[k];
            next_block.act_time     = &incoming_session.act_time[k];
            next_block.idle_time    = &incoming_session.idle_time[k];

            uint8_t pwm = get_data_from_nvs_by_position(next_block.position[0]);
            next_block.pwm = &pwm;

            send_session_ack(&next_block.position[0], handle);
            block_exec(&next_block);
        }

        break;

    case calibration:
        set_data_to_nvs_by_position(rx_packet->payload[0], rx_packet->payload[1]);
        /*
        ESP_LOG_BUFFER_CHAR(BT_TAG, "NVS:", 4);
        esp_log_buffer_hex("", &rx_packet->payload[0], 1);
        esp_log_buffer_hex("", &rx_packet->payload[1], 1);
        */
        send_ACK(handle);
        break;

    case pause:
        stop_pwm();
        send_ACK(handle);
        break;

    case stop:
        stop_pwm();
        send_ACK(handle);
        break;

    case ACK:
        send_ACK(handle);
        break;

    case action_ack:
        break;

    case state_info:
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
    uint8_t buf[2];
    buf[0] = tx_packet.ID;
    buf[1] = tx_packet.length;
    buf[2] = tx_packet.payload[0];
    tx_packet.crc16 = crc16Calc(buf, 2);
    send_to_bt(&tx_packet, handle);
}

void send_ACK(uint32_t handle)
{
    struct bt_packet tx_packet;
    tx_packet.ID = ACK;
    tx_packet.length = 0x04;
    uint8_t buf[1];
    buf[0] = tx_packet.ID;
    buf[1] = tx_packet.length;
    tx_packet.crc16 = crc16Calc(buf, 2);
    send_to_bt(&tx_packet, handle);
}

void send_to_bt(struct bt_packet *tx_packet, uint32_t handle)
{

    if (tx_packet->length == 4)
    { // to do:
        uint8_t packet[3];
        packet[0] = tx_packet->ID;
        packet[1] = tx_packet->length;
        packet[2] = tx_packet->crc16;
        packet[3] = tx_packet->crc16 >> 8;
        ESP_ERROR_CHECK(esp_spp_write(handle, tx_packet->length, packet));
    }

    if (tx_packet->length == 5)
    { // to do:

        // ESP_LOG_BUFFER_CHAR(BT_TAG,"Send:",6);
        // ESP_LOG_BUFFER_CHAR(BT_TAG,"Packet ID:",12);
        // esp_log_buffer_hex("", &tx_packet->ID, 1);
        // ESP_LOG_BUFFER_CHAR(BT_TAG,"Packet length:",14);
        // esp_log_buffer_hex("", &tx_packet->length, 1);
        ESP_LOG_BUFFER_CHAR(BT_TAG, "Packet payload:", 14);
        esp_log_buffer_hex("", tx_packet->payload, 1);
        // ESP_LOG_BUFFER_CHAR(BT_TAG,"Packet crc:",12);
        // esp_log_buffer_hex("", &tx_packet->crc16, 2);

        uint8_t packet[4];
        packet[0] = tx_packet->ID;
        packet[1] = tx_packet->length;
        packet[2] = tx_packet->payload[0];
        packet[3] = tx_packet->crc16;
        packet[4] = tx_packet->crc16 >> 8;
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
    set_motors_en(DRIVERS_ON);
    switch ((int)incoming_session->position[0])
    {
    case 01:
        vTaskDelay(((incoming_session->delay_time[0]) * 1000) / portTICK_RATE_MS);
        set_PWM(MOTOR_1_IN_1, 0);
        set_PWM(MOTOR_1_IN_2, incoming_session->pwm[0]);                         // push
        vTaskDelay(((incoming_session->act_time[0]) * 1000) / portTICK_RATE_MS); // wait
        set_PWM(MOTOR_1_IN_1, 200);                                              // retract
        set_PWM(MOTOR_1_IN_2, 0);
        vTaskDelay(100 / portTICK_RATE_MS);
        set_PWM(MOTOR_1_IN_1, LEDC_DUTY_IDLE); // set idle
        set_PWM(MOTOR_1_IN_2, LEDC_DUTY_IDLE);
        vTaskDelay(((incoming_session->idle_time[0]) * 1000) / portTICK_RATE_MS);
        break;

    case 02:
        vTaskDelay(((incoming_session->delay_time[0]) * 1000) / portTICK_RATE_MS);
        set_PWM(MOTOR_2_IN_1, 0);
        set_PWM(MOTOR_2_IN_2, incoming_session->pwm[0]);                         // push
        vTaskDelay(((incoming_session->act_time[0]) * 1000) / portTICK_RATE_MS); // wait
        set_PWM(MOTOR_2_IN_1, 200);                                              // retract
        set_PWM(MOTOR_2_IN_2, 0);
        vTaskDelay(100 / portTICK_RATE_MS);
        set_PWM(MOTOR_2_IN_1, LEDC_DUTY_IDLE); // set idle
        set_PWM(MOTOR_2_IN_2, LEDC_DUTY_IDLE);
        vTaskDelay(((incoming_session->idle_time[0]) * 1000) / portTICK_RATE_MS);
        break;
    case 03:
        vTaskDelay(((incoming_session->delay_time[0]) * 1000) / portTICK_RATE_MS);
        set_PWM(MOTOR_3_IN_1, 0);
        set_PWM(MOTOR_3_IN_2, incoming_session->pwm[0]);                         // push
        vTaskDelay(((incoming_session->act_time[0]) * 1000) / portTICK_RATE_MS); // wait
        set_PWM(MOTOR_3_IN_1, 200);                                              // retract
        set_PWM(MOTOR_3_IN_2, 0);
        vTaskDelay(100 / portTICK_RATE_MS);
        set_PWM(MOTOR_3_IN_1, LEDC_DUTY_IDLE); // set idle
        set_PWM(MOTOR_3_IN_2, LEDC_DUTY_IDLE);
        vTaskDelay(((incoming_session->idle_time[0]) * 1000) / portTICK_RATE_MS);
        break;
    case 04:
        vTaskDelay(((incoming_session->delay_time[0]) * 1000) / portTICK_RATE_MS);
        set_PWM(MOTOR_4_IN_1, 0);
        set_PWM(MOTOR_4_IN_2, incoming_session->pwm[0]);                         // push
        vTaskDelay(((incoming_session->act_time[0]) * 1000) / portTICK_RATE_MS); // wait
        set_PWM(MOTOR_4_IN_1, 200);                                              // retract
        set_PWM(MOTOR_4_IN_2, 0);
        vTaskDelay(100 / portTICK_RATE_MS);
        set_PWM(MOTOR_4_IN_1, LEDC_DUTY_IDLE); // set idle
        set_PWM(MOTOR_4_IN_2, LEDC_DUTY_IDLE);
        vTaskDelay(((incoming_session->idle_time[0]) * 1000) / portTICK_RATE_MS);
        break;
    case 05:
        vTaskDelay(((incoming_session->delay_time[0]) * 1000) / portTICK_RATE_MS);
        set_PWM(MOTOR_5_IN_1, 0);
        set_PWM(MOTOR_5_IN_2, incoming_session->pwm[0]);                         // push
        vTaskDelay(((incoming_session->act_time[0]) * 1000) / portTICK_RATE_MS); // wait
        set_PWM(MOTOR_5_IN_1, 200);                                              // retract
        set_PWM(MOTOR_5_IN_2, 0);
        vTaskDelay(100 / portTICK_RATE_MS);
        set_PWM(MOTOR_5_IN_1, LEDC_DUTY_IDLE); // set idle
        set_PWM(MOTOR_5_IN_2, LEDC_DUTY_IDLE);
        vTaskDelay(((incoming_session->idle_time[0]) * 1000) / portTICK_RATE_MS);
        break;

    default:
        ESP_LOG_BUFFER_CHAR(BT_TAG, "case d", 6);

        break;
    }
    set_motors_en(DRIVERS_OFF);
}

uint8_t get_data_from_nvs_by_position(uint8_t position)
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

    if(pwm_to_save == 0){
        pwm_to_save = 25;
    }

    else 
    pwm_to_save = pwm_to_save*25;
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
    ESP_LOG_BUFFER_CHAR(BT_TAG, "def:", 4);
        break;
    }
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);
}