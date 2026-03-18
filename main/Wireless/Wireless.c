#include "Wireless.h"

uint16_t BLE_NUM = 0;
uint16_t WIFI_NUM = 0;
bool Scan_finish = 0;

bool WiFi_Scan_Finish = 0;
bool BLE_Scan_Finish = 0;

#define MAX_BLE_DEVICES 10

typedef struct {
    int8_t rssi;
    bool found;
} ble_device_t;

static ble_device_t ble_devices[MAX_BLE_DEVICES];
static int num_ble_devices = 0;

int BLE_Get_Device_RSSI(int index) {
    if (index >= 0 && index < MAX_BLE_DEVICES) {
        return ble_devices[index].found ? ble_devices[index].rssi : -100;
    }
    return -100;
}

int BLE_Get_Num_Devices(void) {
    return num_ble_devices;
}

bool BLE_Is_Device_Found(int index) {
    if (index >= 0 && index < MAX_BLE_DEVICES) {
        return ble_devices[index].found;
    }
    return false;
}

void BLE_Reset_Beacon(void) {
    for (int i = 0; i < MAX_BLE_DEVICES; i++) {
        ble_devices[i].found = false;
        ble_devices[i].rssi = -100;
    }
    num_ble_devices = 0;
}
void Wireless_Init(void)
{
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    // WiFi - disabled to save space
    // xTaskCreatePinnedToCore(
    //     WIFI_Init, 
    //     "WIFI task",
    //     4096, 
    //     NULL, 
    //     2, 
    //     NULL, 
    //     0);
    // BLE
    xTaskCreatePinnedToCore(
        BLE_Init, 
        "BLE task",
        4096, 
        NULL, 
        2, 
        NULL, 
        0);
}

void WIFI_Init(void *arg)
{
    esp_netif_init();                                                     
    esp_event_loop_create_default();                                  
    esp_netif_create_default_wifi_sta();                                 
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();              
    esp_wifi_init(&cfg);                                      
    esp_wifi_set_mode(WIFI_MODE_STA);              
    esp_wifi_start();                         

    WIFI_NUM = WIFI_Scan();
    printf("WIFI:%d\r\n",WIFI_NUM);
    
    vTaskDelete(NULL);
}
uint16_t WIFI_Scan(void)
{
    uint16_t ap_count = 0;
    esp_wifi_scan_start(NULL, true);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    WiFi_Scan_Finish =1;
    if(BLE_Scan_Finish == 1)
        Scan_finish = 1;
    return ap_count;
}


#define GATTC_TAG "GATTC_TAG"
#define SCAN_DURATION 3  

static bool check_beacon_uuid(const uint8_t *adv_data, uint8_t adv_data_len) {
    // For now, detect ANY beacon - simplify to just use RSSI from all devices
    // This is for debugging - shows red dot for any BLE device found
    (void)adv_data;
    (void)adv_data_len;
    return true;
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                if (check_beacon_uuid(param->scan_rst.ble_adv, param->scan_rst.adv_data_len)) {
                    // Store RSSI for each device found (up to MAX_BLE_DEVICES)
                    if (num_ble_devices < MAX_BLE_DEVICES) {
                        ble_devices[num_ble_devices].rssi = param->scan_rst.rssi;
                        ble_devices[num_ble_devices].found = true;
                        num_ble_devices++;
                        // printf("BLE[%d] RSSI: %d\n", num_ble_devices, param->scan_rst.rssi);
                    }
                }
                BLE_NUM++;
            }
            break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            break;
        default:
            break;
    }
}

void BLE_Init(void *arg)
{
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);                                            
    if (ret) {
        printf("%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));        
        return;}
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);                                           
    if (ret) {
        printf("%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));            
        return;}
    ret = esp_bluedroid_init();                                                                 
    if (ret) {
        printf("%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));               
        return;}
    ret = esp_bluedroid_enable();                                                               
    if (ret) {
        printf("%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));             
        return;}

    /* Set BLE device name to "Dragon Radar" */
    ret = esp_ble_gap_set_device_name("Dragon Radar");
    if (ret) {
        printf("%s set device name failed: %s\n", __func__, esp_err_to_name(ret));
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);                                            
    if (ret){
        printf("%s gap register error, error code = %x\n", __func__, ret);                      
        return;
    }

    while (1) {
        BLE_Reset_Beacon();
        BLE_Scan();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

uint16_t BLE_Scan(void)
{
    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_RPA_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,   
        .scan_window = 0x30,       
        .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
    };
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));

    // printf("Starting BLE scan...\n");
    ESP_ERROR_CHECK(esp_ble_gap_start_scanning(SCAN_DURATION));
    
    // Set scanning duration
    vTaskDelay(SCAN_DURATION * 2000 / portTICK_PERIOD_MS);
    
    // printf("Stopping BLE scan...\n");
    ESP_ERROR_CHECK(esp_ble_gap_stop_scanning());
    BLE_Scan_Finish = 1;
    Scan_finish = 1;
    return BLE_NUM;
}

void BLE_Enable(void)
{
    esp_ble_gap_start_scanning(SCAN_DURATION);
}

void BLE_Disable(void)
{
    esp_ble_gap_stop_scanning();
    BLE_Reset_Beacon();
}