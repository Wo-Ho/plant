#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "controller.h"

#include "bt.h"
#include "bt_trace.h"
#include "bt_types.h"
#include "btm_api.h"
#include "bta_api.h"
#include "bta_gatt_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "driver/gpio.h"
#include "esp_system.h"

#include "definitions.h"

#define GATTC_TAG "PLANTCARE"

#define SENSOR_01 "C4:7C:8D:63:3C:36"

static TaskHandle_t sensorTaskHandle = NULL;

///Declare static functions
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

#define BT_BD_ADDR_STR         "%02x:%02x:%02x:%02x:%02x:%02x"
#define BT_BD_ADDR_HEX(addr)   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]

static bool connect = false;
static bool gatt_ready = false;
static const char device_name[] = "Flower care";

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};


#define PROFILE_NUM 1
#define PROFILE_APP_ID 0

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    esp_bd_addr_t remote_bda;
};

static bool checkMAC(esp_bd_addr_t data, const char* client) {

    unsigned char mac[6];
	sscanf(client, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);    
    
    int cmp = memcmp(data, mac, sizeof(esp_bd_addr_t));

    if(cmp == 0)
        return true;
    else
        return false;
}

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        .conn_id = 0,
    },    
};

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    uint16_t conn_id = 0;
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
        case ESP_GATTC_REG_EVT:
            ESP_LOGI(GATTC_TAG, "REG_EVT");
            esp_ble_gap_set_scan_params(&ble_scan_params);
            break;
        case ESP_GATTC_OPEN_EVT:
            conn_id = p_data->open.conn_id;
            ESP_LOGI(GATTC_TAG, "GATT opened id %i", conn_id);		            
            
            memcpy(gl_profile_tab[PROFILE_APP_ID].remote_bda, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
            gl_profile_tab[PROFILE_APP_ID].conn_id = p_data->open.conn_id;
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d, mtu %d", conn_id, gattc_if, p_data->open.status, p_data->open.mtu);

            ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
            esp_log_buffer_hex(GATTC_TAG, gl_profile_tab[PROFILE_APP_ID].remote_bda, sizeof(esp_bd_addr_t));

            // esp_ble_gattc_search_service(gattc_if, conn_id, NULL);
            gatt_ready = true;
            break;
        case ESP_GATTC_CLOSE_EVT:
            ESP_LOGI(GATTC_TAG, "GATT closed");				
            gl_profile_tab[PROFILE_APP_ID].conn_id = 0;
            break;
        case ESP_GATTC_SEARCH_RES_EVT: {    
            break;
        }
        case ESP_GATTC_SEARCH_CMPL_EVT:
            conn_id = p_data->search_cmpl.conn_id;
            ESP_LOGI(GATTC_TAG, "SEARCH_CMPL: conn_id = %x, status %d", conn_id, p_data->search_cmpl.status);
            // esp_ble_gattc_get_characteristic(gattc_if, conn_id, &alert_service_id, NULL);
            break;
        case ESP_GATTC_READ_CHAR_EVT: {
            esp_gatt_id_t *char_id = &p_data->read.char_id;
            
            ESP_LOGI(GATTC_TAG, "READ CHAR: open.conn_id = %x search_res.conn_id = %x  read.conn_id = %x", conn_id,p_data->search_res.conn_id,p_data->read.conn_id);
            ESP_LOGI(GATTC_TAG, "READ CHAR: read.status = %x inst_id = %x", p_data->read.status, char_id->inst_id);
            for (int i = 0; i < p_data->read.value_len; i++) {
                ESP_LOGI(GATTC_TAG, ">> %x:", p_data->read.value[i]);
            }
            break;
        }
        case ESP_GATTC_READ_DESCR_EVT: {
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_READ_DESCR_EVT");
            break;
        }
        case ESP_GATTC_GET_CHAR_EVT:
            ESP_LOGI(GATTC_TAG, "GET CHAR");
            if (p_data->get_char.status != ESP_GATT_OK) {
                break;
            }
            ESP_LOGI(GATTC_TAG, "GET CHAR: conn_id = %x, status %d", p_data->get_char.conn_id, p_data->get_char.status);
            ESP_LOGI(GATTC_TAG, "GET CHAR: srvc_id = %04x, char_id = %04x", p_data->get_char.srvc_id.id.uuid.uuid.uuid16, p_data->get_char.char_id.uuid.uuid.uuid16);           

            //esp_ble_gattc_get_characteristic(gattc_if, conn_id, &alert_service_id, &p_data->get_char.char_id);
            break;
        case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
            ESP_LOGI(GATTC_TAG, "REG FOR NOTIFY: status %d", p_data->reg_for_notify.status);
            ESP_LOGI(GATTC_TAG, "REG FOR_NOTIFY: srvc_id = %04x, char_id = %04x", p_data->reg_for_notify.srvc_id.id.uuid.uuid.uuid16, p_data->reg_for_notify.char_id.uuid.uuid.uuid16);       
            break;
        }
        case ESP_GATTC_NOTIFY_EVT:
            ESP_LOGI(GATTC_TAG, "NOTIFY: len %d, value %08x", p_data->notify.value_len, *(uint32_t *)p_data->notify.value);
            break;

        case ESP_GATTC_WRITE_DESCR_EVT:
            ESP_LOGI(GATTC_TAG, "WRITE: status %d", p_data->write.status);
            break;

        case ESP_GATTC_SRVC_CHG_EVT: {
            esp_bd_addr_t bda;
            memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:%08x%04x",(bda[0] << 24) + (bda[1] << 16) + (bda[2] << 8) + bda[3],
                    (bda[4] << 8) + bda[5]);
            break;
        }
        default:
            break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            //the unit of the duration is second
            uint32_t duration = 30;
            esp_ble_gap_start_scanning(duration);
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            //scan start complete event to indicate scan start successfully or failed
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTC_TAG, "Scan start failed");
            }
            break;
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
            switch (scan_result->scan_rst.search_evt) {            
                case ESP_GAP_SEARCH_INQ_RES_EVT:
                    // ESP_LOGI(GATTC_TAG, "Searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
                    adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);                    
                    if (adv_name != NULL) {
                        if (strlen(device_name) == adv_name_len && strncmp((char *)adv_name, device_name, adv_name_len) == 0) {
                            esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);
                            ESP_LOGI(GATTC_TAG, "\n");                            
                            esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);             

                            if(checkMAC(scan_result->scan_rst.bda, SENSOR_01) == true) {
                                ESP_LOGI(GATTC_TAG, "Address found");    
                                
                                if (connect == false) {
                                    connect = true;
                                    ESP_LOGI(GATTC_TAG, "Connect to the remote device.");
                                    esp_ble_gap_stop_scanning();
                                    esp_ble_gattc_open(gl_profile_tab[PROFILE_APP_ID].gattc_if, scan_result->scan_rst.bda, true);
                                    
                                    // esp_ble_gattc_close(l_profile_tab[PROFILE_APP_ID].gattc_if, 0);                                
                                }
                            }
                            
                        }
                    }
                    break;
                case ESP_GAP_SEARCH_INQ_CMPL_EVT:                                         
                    break;
                default:
                    break;
                }
            break;
        }

        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
                ESP_LOGE(GATTC_TAG, "Scan stop failed");
            }
            else {
                ESP_LOGI(GATTC_TAG, "Stop scan successfully");
            }
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
                ESP_LOGE(GATTC_TAG, "Adv stop failed");
            }
            else {
                ESP_LOGI(GATTC_TAG, "Stop adv successfully");
            }
            break;

        default:
            break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{   
    ESP_LOGI(GATTC_TAG, "EVT %d, gattc if %d", event, gattc_if);

    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id, 
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

void ble_client_appRegister(void)
{
    esp_err_t status;

    ESP_LOGI(GATTC_TAG, "register callback");

    //register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "gap register error, error code = %x", status);
        return;
    }

    //register the callback function to the gattc module
    if ((status = esp_ble_gattc_register_callback(esp_gattc_cb)) != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "gattc register error, error code = %x", status);
        return;
    }

    esp_ble_gattc_app_register(PROFILE_APP_ID);
}

void gattc_client_test(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    ble_client_appRegister();
}

static void led_on()
{
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_4, true);
}

static void led_off()
{
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_4, false);
}

static void sensor_task(void *arg)
{   
    while(1) {
        
        if(connect == true && gatt_ready == true) {
            led_on();

            ESP_LOGI(GATTC_TAG, "Start Sensor");
            
            
            esp_err_t r1 = esp_ble_gattc_read_char(
                gl_profile_tab[PROFILE_APP_ID].gattc_if, 
                gl_profile_tab[PROFILE_APP_ID].conn_id, 
                &sensor_service_name,
                &sensor_characteristic_name,
                ESP_GATT_AUTH_REQ_NONE);
            
            /*
            esp_err_t r2 = esp_ble_gattc_read_char(
                gl_profile_tab[PROFILE_APP_ID].gattc_if, 
                gl_profile_tab[PROFILE_APP_ID].conn_id, 
                &sensor_service_version,
                &sensor_characteristic_version,
                ESP_GATT_AUTH_REQ_NONE);
            */

            ESP_LOGI(GATTC_TAG, "Read result 1: %i", r1);
            // ESP_LOGI(GATTC_TAG, "Read result 2: %i", r2);
            
            // esp_ble_gattc_close(gl_profile_tab[PROFILE_APP_ID].gattc_if, gl_profile_tab[PROFILE_APP_ID].conn_id);            
            // esp_ble_gattc_app_unregister(gl_profile_tab[PROFILE_APP_ID].gattc_if);

            // vTaskDelete(NULL);
            connect = false;
        }

        vTaskDelay(2000 / portTICK_RATE_MS);
    }	
}

void app_main()
{
    led_off();

    ESP_LOGI(GATTC_TAG, "Initialize BLE");

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BTDM);

    gattc_client_test();

    xTaskCreate(&sensor_task, "sensor_task", 2048, NULL, 6, &sensorTaskHandle);
}

// connect wifi
// ESP_LOGI(GATTC_TAG, "Connect Wifi");
// connect_wifi();