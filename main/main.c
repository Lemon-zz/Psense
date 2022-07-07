#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "linenoise/linenoise.h"
#include "time.h"
#include "sys/time.h"

#include "BT_to_Session.h"



#define SPP_TAG "Psense_SPP"
#define SPP_SERVER_NAME "SPP_SERVER"
#define DEVICE_NAME "PSense"

#define s1_max 220
#define s1_min 142
#define s2_max 220
#define s2_min 142
#define s3_max 220
#define s3_min 142
#define s4_max 220
#define s4_min 142
#define s5_max 2700
#define s5_min 142

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

uint8_t  *incoming_data;
uint16_t *data_length;
uint8_t *retract_time, *forward_time;

uint32_t handle;

TaskHandle_t parse_exec_xHandle = NULL;
TaskHandle_t m1_sens1_xHandle = NULL;
TaskHandle_t m2_sens2_xHandle = NULL;
TaskHandle_t m3_sens3_xHandle = NULL;
TaskHandle_t m4_sens4_xHandle = NULL;
TaskHandle_t m5_sens5_xHandle = NULL;
TaskHandle_t ADC_task_xHandle = NULL;

 // Create a queue to hold one uint32_t value.  It is strongly
 // recommended *not* to use xQueueOverwrite() on queues that can
 // contain more than one value, and doing so will trigger an assertion

QueueHandle_t Sens1_xQueue;
QueueHandle_t Sens2_xQueue;
QueueHandle_t Sens3_xQueue;
QueueHandle_t Sens4_xQueue;
QueueHandle_t Sens5_xQueue;
QueueHandle_t is_session_xQueue;


uint32_t ADC_voltage, batt_voltage;
uint8_t out_min = 0;
uint8_t out_max = 127;
uint8_t is_session_active = 1;



long map(long x, long in_min, long in_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ADC_task(void *pvParameter){
    Sens1_xQueue = xQueueCreate( 1, sizeof( uint32_t ) );
    Sens2_xQueue = xQueueCreate( 1, sizeof( uint32_t ) );
    Sens3_xQueue = xQueueCreate( 1, sizeof( uint32_t ) );
    Sens4_xQueue = xQueueCreate( 1, sizeof( uint32_t ) );
    Sens5_xQueue = xQueueCreate( 1, sizeof( uint32_t ) );
    while(1){
    
    ADC_voltage = get_ADC_data(ADC1, CHAN_1);
    xQueueOverwrite(Sens1_xQueue, &ADC_voltage);
    ADC_voltage = get_ADC_data(ADC1, CHAN_2);
    xQueueOverwrite(Sens2_xQueue, &ADC_voltage);
    ADC_voltage = get_ADC_data(ADC2, CHAN_3);
    xQueueOverwrite(Sens3_xQueue, &ADC_voltage);
    ADC_voltage = get_ADC_data(ADC2, CHAN_4);
    xQueueOverwrite(Sens4_xQueue, &ADC_voltage); 
    ADC_voltage = get_ADC_data(ADC2, CHAN_5);
    xQueueOverwrite(Sens5_xQueue, &ADC_voltage);


    if(gpio_get_level(BTN_GPIO) && is_session_active == 0){
        is_session_active = 1;
        xQueueOverwrite(is_session_xQueue, &is_session_active);
        set_motors_en(DRIVERS_OFF);
        vTaskDelay(500 / portTICK_RATE_MS);
    }

    if(gpio_get_level(BTN_GPIO) && is_session_active == 1){
        is_session_active = 0;
        xQueueOverwrite(is_session_xQueue, &is_session_active);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    vTaskDelay(100 / portTICK_RATE_MS);
}
    vTaskDelete(NULL);
}



void m1_sens1(void *pvParameter)
{   
    uint8_t is_m_retracted = 0;
    uint8_t pwm1, saved_pwm = 0;
    uint8_t is_session_going;
    uint32_t sens1, sens2, sens3, sens4, sens5 = 0; 
    
    while(1)
    {
    xQueuePeek(is_session_xQueue, &is_session_going, 1);
    saved_pwm = get_pwm_from_nvs_by_position(1);
    if(is_session_going != 1){
    
    xQueueReceive(Sens1_xQueue, &sens1, 1);
    xQueuePeek(Sens2_xQueue, &sens2, 1);
    xQueuePeek(Sens3_xQueue, &sens3, 1);
    xQueuePeek(Sens4_xQueue, &sens4, 1);
    xQueuePeek(Sens5_xQueue, &sens5, 1);

    printf("Sensor 1: %d\n", sens1);
    printf("Sensor 2: %d\n", sens2);
    printf("Sensor 3: %d\n", sens3);
    printf("Sensor 4: %d\n", sens4);
    printf("Sensor 5: %d\n", sens5);

    set_motors_en(DRIVERS_ON);

    if(sens1 > s1_max){
    sens1 = s1_max;
        }
  if(sens1 < s1_max - 20) {
     is_m_retracted = 0;
     pwm1 = map(sens1, s1_min, s1_max);
     printf("PWM 1: %d\n", pwm1);
     set_PWM(MOTOR_1_IN_1, pwm1);
     set_PWM(MOTOR_1_IN_2, 255 - pwm1);
     vTaskDelay(100 / portTICK_RATE_MS);
  }
    if(sens1  > s1_max - 20 && is_m_retracted == 0){
        set_PWM(MOTOR_1_IN_1, 255-saved_pwm);
        set_PWM(MOTOR_1_IN_2, saved_pwm);
        vTaskDelay(retract_time[0]*250 / portTICK_RATE_MS);
        set_PWM(MOTOR_1_IN_1, 127);
        set_PWM(MOTOR_1_IN_2, 127);
        is_m_retracted = 1;
  }
    }
    vTaskDelay(100 / portTICK_RATE_MS);
    linenoiseClearScreen();
    }
    vTaskDelete(NULL);
}

void m2_sens2(void *pvParameter)
{   
    uint8_t is_m_retracted = 0;
    uint8_t pwm1, saved_pwm = 0;
    uint8_t is_session_going;
    uint32_t sens2 = 0; 
    while(1)
    {
    xQueuePeek(is_session_xQueue, &is_session_going, 1);
    saved_pwm = get_pwm_from_nvs_by_position(2);
    if(is_session_going != 1){
    xQueueReceive(Sens2_xQueue, &sens2, 1);
    if(sens2 > s2_max){
    sens2 = s2_max;
        }

  if(sens2 < s2_max - 20) {
     is_m_retracted = 0;
     pwm1 = map(sens2, s2_min, s2_max);
     set_PWM(MOTOR_2_IN_1, pwm1);
     set_PWM(MOTOR_2_IN_2, 255 - pwm1);
     vTaskDelay(100 / portTICK_RATE_MS);
  }

    if(sens2  > s2_max - 20 && is_m_retracted == 0){
        set_PWM(MOTOR_2_IN_1, 255-saved_pwm);
        set_PWM(MOTOR_2_IN_2, saved_pwm);
        vTaskDelay(retract_time[1]*250 / portTICK_RATE_MS);
        set_PWM(MOTOR_2_IN_1, 127);
        set_PWM(MOTOR_2_IN_2, 127);
        is_m_retracted = 1;
  }
    }
    vTaskDelay(100 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

void m3_sens3(void *pvParameter)
{   
    uint8_t is_m_retracted = 0;
    uint8_t pwm1, saved_pwm = 0;
    uint8_t is_session_going;
    uint32_t sens3 = 0; 
    while(1)
    {
    xQueuePeek(is_session_xQueue, &is_session_going, 1);
    saved_pwm = get_pwm_from_nvs_by_position(3);
    if(is_session_going != 1){

    xQueueReceive(Sens3_xQueue, &sens3, 1);

    
    if(sens3 > s3_max){
    sens3 = s3_max;
        }
  if(sens3 < s3_max - 20) {
     is_m_retracted = 0;
     pwm1 = map(sens3, s3_min, s3_max);
     set_PWM(MOTOR_3_IN_1, pwm1);
     set_PWM(MOTOR_3_IN_2, 255 - pwm1);
     vTaskDelay(100 / portTICK_RATE_MS);
  }
    if(sens3  > s3_max - 20 && is_m_retracted == 0){
        set_PWM(MOTOR_3_IN_1, 255-saved_pwm);
        set_PWM(MOTOR_3_IN_2, saved_pwm);
        vTaskDelay(retract_time[2]*250 / portTICK_RATE_MS);
        set_PWM(MOTOR_3_IN_1, 127);
        set_PWM(MOTOR_3_IN_2, 127);
        is_m_retracted = 1;
  }
    }
    vTaskDelay(100 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

void m4_sens4(void *pvParameter)
{   
    uint8_t is_m_retracted = 0;
    uint8_t pwm1, saved_pwm = 0;
    uint8_t is_session_going;
    uint32_t sens4 = 0; 
    set_motors_en(DRIVERS_ON);
    while(1)
    {
    xQueuePeek(is_session_xQueue, &is_session_going, 1);
    saved_pwm = get_pwm_from_nvs_by_position(4);
    if(is_session_going != 1){

    xQueueReceive(Sens4_xQueue, &sens4, 1);

    if(sens4 > s4_max){
    sens4 = s4_max;
        }
  if(sens4 < s4_max - 20) {
     is_m_retracted = 0;
     pwm1 = map(sens4, s4_min, s4_max);
     printf("PWM 4: %d\n", pwm1);
     set_PWM(MOTOR_4_IN_1, pwm1);
     set_PWM(MOTOR_4_IN_2, 255 - pwm1);
     vTaskDelay(100 / portTICK_RATE_MS);
  }
    if(sens4  > s4_max - 20 && is_m_retracted == 0){
        set_PWM(MOTOR_4_IN_1, 255-saved_pwm);
        set_PWM(MOTOR_4_IN_2, saved_pwm);
        vTaskDelay(retract_time[3]*250 / portTICK_RATE_MS);
        set_PWM(MOTOR_4_IN_1, 127);
        set_PWM(MOTOR_4_IN_2, 127);
        is_m_retracted = 1;
  }
    }
    vTaskDelay(100 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

void m5_sens5(void *pvParameter)
{   
    uint8_t is_m_retracted = 0;
    uint8_t pwm1, saved_pwm = 0;
    uint8_t is_session_going;
    uint32_t sens5 = 0; 
    set_motors_en(DRIVERS_ON);
    while(1)
    {
    xQueuePeek(is_session_xQueue, &is_session_going, 1);
    saved_pwm = get_pwm_from_nvs_by_position(5);
    if(is_session_going != 1){

    xQueueReceive(Sens5_xQueue, &sens5, 1);

    if(sens5 > s5_max){
    sens5 = s5_max;
        }

  if(sens5 > s5_min + 100) {
     is_m_retracted = 0;
     pwm1 = map(sens5, s5_max, s5_min);
     printf("PWM 5: %d\n", pwm1);
     set_PWM(MOTOR_5_IN_1, pwm1);
     set_PWM(MOTOR_5_IN_2, 255 - pwm1);
     vTaskDelay(100 / portTICK_RATE_MS);
  }
    if(sens5  < s5_min + 100 && is_m_retracted == 0){
        set_PWM(MOTOR_5_IN_1, 255-saved_pwm);
        set_PWM(MOTOR_5_IN_2, saved_pwm);
        vTaskDelay(retract_time[4]*250 / portTICK_RATE_MS);
        set_PWM(MOTOR_5_IN_1, 127);
        set_PWM(MOTOR_5_IN_2, 127);
        is_m_retracted = 1;
  }
    }
    vTaskDelay(100 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}





void parse_exec_session(void *pvParameter)
{   
    parse_data_from_bt(incoming_data, data_length, handle);
    vTaskDelete(NULL);
}






static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        set_motors_en(DRIVERS_OFF);
        if(xTaskGetHandle("parse_exec_task") != NULL){
            vTaskDelete(xTaskGetHandle("parse_exec_task"));
        }
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        esp_bt_dev_set_device_name(DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d", param->data_ind.len, param->data_ind.handle);
        //copy data from bt
        incoming_data = calloc(param->data_ind.len, sizeof(uint8_t));
        memcpy(incoming_data, param->data_ind.data, param->data_ind.len);
        data_length = calloc(param->data_ind.len, sizeof(param->data_ind.len));
        memcpy(data_length, &param->data_ind.len, sizeof(param->data_ind.len));
        handle = param->data_ind.handle;

        //create session task
        if(incoming_data[0] == pause || incoming_data[0] == stop){
            if(xTaskGetHandle("parse_exec_task") != NULL){
                vTaskDelete(xTaskGetHandle("parse_exec_task"));
            }
            stop_pwm();
            set_motors_en(DRIVERS_OFF);
            send_ACK(handle);
            break;
        }
        if(xTaskGetHandle("parse_exec_task") != NULL){
            vTaskDelete(xTaskGetHandle("parse_exec_task"));
        }
        is_session_active = 1;
        xQueueOverwrite(is_session_xQueue, &is_session_active);
        xTaskCreate(&parse_exec_session, "parse_exec_task", 3192, NULL, 5, parse_exec_xHandle);
        break;

    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        if (param->write.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG,"ESP_SPP_WRITE_EVT: %u %s", param->write.len, param->write.cong?"CONGESTED":"");
        } else {
            ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT failed!, status:%d", param->write.status);
        }
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        //gettimeofday(&time_old, NULL);
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

    //The functions bt_app_gap_cb handle all the events generated by the Bluetooth stack.
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}



void app_main(void)
{
    //store PHY calibration data and save key-value pairs in flash memory
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK( ret );
    //Release the BLE (Bluetooth Low Energy) memory in the controller via:
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    //The controller memory should be released only before initializing Bluetooth controller or after deinitializing Bluetooth controller.
    //Note that once Bluetooth controller memory is released, the process cannot be reversed.
    //It means you cannot use the Bluetooth mode which you have released by this function.


    /*The main function also initializes the Bluetooth controller by first creating the controller configuration structure named esp_bt_controller_config_t 
    with default settings generated by the BT_CONTROLLER_INIT_CONFIG_DEFAULT() macro. 
    The Bluetooth controller implements the Host Controller Interface (HCI) on the controller side, the Link Layer (LL), 
    and the Physical Layer (PHY). The Bluetooth Controller is invisible to the user applications and
    deals with the lower layers of the Bluetooth stack. 
    The controller configuration includes setting the Bluetooth controller stack size, 
    priority, and HCI baud rate. 
    With the settings created, the Bluetooth controller is initialized and enabled with the esp_bt_controller_init() function:
    */

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    //controller
    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    //bluedroid
    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    //The GAP event handlers are the functions used to catch the events generated by the Bluetooth stack and execute functions 
    //to configure parameters of the application.
    //The functions bt_app_gap_cb handle all the events generated by the Bluetooth stack.
    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    //spp
    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
    HW_init();
    retract_time = calloc(5, sizeof(uint8_t));
    forward_time = calloc(5, sizeof(uint8_t));
    get_data_from_motor_move(retract_time, forward_time);   

    is_session_xQueue = xQueueCreate( 1, sizeof( uint8_t ) );
    xQueueOverwrite(is_session_xQueue, &is_session_active);
    
    if(xTaskGetHandle("ADC_task") == NULL){
        xTaskCreate(&ADC_task, "ADC_Task", 2048, NULL, 3, ADC_task_xHandle);
    }
    if(xTaskGetHandle("m1_sens1") == NULL && (xTaskGetHandle("parse_exec_task") == NULL)){
        xTaskCreate(&m1_sens1, "m1_sens1", 2048, NULL, 5, m1_sens1_xHandle);
    }
    if(xTaskGetHandle("m2_sens2") == NULL && (xTaskGetHandle("parse_exec_task") == NULL)){
        xTaskCreate(&m2_sens2, "m2_sens2", 2048, NULL, 5, m2_sens2_xHandle);
    }
    if(xTaskGetHandle("m3_sens3") == NULL && (xTaskGetHandle("parse_exec_task") == NULL)){
        xTaskCreate(&m3_sens3, "m3_sens3", 2048, NULL, 5, m3_sens3_xHandle);
    }
       if(xTaskGetHandle("m4_sens4") == NULL && (xTaskGetHandle("parse_exec_task") == NULL)){
        xTaskCreate(&m4_sens4, "m4_sens4", 2048, NULL, 5, m4_sens4_xHandle);
    }
       if(xTaskGetHandle("m5_sens5") == NULL && (xTaskGetHandle("parse_exec_task") == NULL)){
        xTaskCreate(&m5_sens5, "m5_sens5", 2048, NULL, 5, m5_sens5_xHandle);
    }
    
} 