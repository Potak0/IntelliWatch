/*****************************************************************************
 *                                                                           *
 *  Copyright 2018 Simon M. Werner                                           *
 *                                                                           *
 *  Licensed under the Apache License, Version 2.0 (the "License");          *
 *  you may not use this file except in compliance with the License.         *
 *  You may obtain a copy of the License at                                  *
 *                                                                           *
 *      http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                           *
 *  Unless required by applicable law or agreed to in writing, software      *
 *  distributed under the License is distributed on an "AS IS" BASIS,        *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 *  See the License for the specific language governing permissions and      *
 *  limitations under the License.                                           *
 *                                                                           *
 *****************************************************************************/
#define LAST_COMMIT_BEFORE_BUILD 0

#define PI 3.1415926535f
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

#include "driver/i2c.h"

#include "ahrs.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"

#include "lvgl.h"
#include "lvgl_helpers.h"

#include "DRV2605.h"

#include "max30102.h"

#include "driver/ledc.h"
#include "driver/timer.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"

#include "esp_sntp.h"
#include <sys/time.h>
#include <time.h>

#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

// ulp
#include "esp_sleep.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32s3/ulp.h"
#include "esp32s3/ulp_riscv.h"
#include "ulp_main.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/temp_sensor.h"

#include "bh1750_i2c.h"

#include "bq25896.h"

#define PRE_PRE_SLEEP_TIME 2 // second
#define PRE_SLEEP_TIME 2
#define SLEEP_TIME 2
/*1:pre_pre_sleep 2:pre_sleep 3:sleep 4:pre_wakeup 5:wakeup
  0:default
  -1:sleeping*/
int sys_state = 4;
int sys_sleep_timer_countstart = 0;
int sys_backlight = 1000;
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

void sys_sleep_timer(void *arg);
void sys_state_machine(void *arg);
void system_load_from_flash();
void system_write_to_flash();
void system_init();
// gpio handler
#define KEY_IRQ GPIO_NUM_10
#define IMU_IRQ GPIO_NUM_9
#define TOUCH_IRQ GPIO_NUM_11
esp_sleep_wakeup_cause_t cause;
static void init_ulp_program(void)
{
  esp_err_t err = ulp_riscv_load_binary(ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start));
  ESP_ERROR_CHECK(err);

  /* The first argument is the period index, which is not used by the ULP-RISC-V timer
   * The second argument is the period in microseconds, which gives a wakeup time period of: 20ms
   */
  ulp_set_wakeup_period(0, 86400000UL);

  /* Start the program */
  err = ulp_riscv_run();
  ESP_ERROR_CHECK(err);
}
static const char *TAG = "main";
void BH1750_Init(void);
void BH1750_Read(uint16_t *light);
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

/*********************
 *      DEFINES
 *********************/
#define LV_TICK_PERIOD_MS 1
#define Filt_SAMPLE_FREQ_Hz 200
#define Fstop 20.0f
/**********************
 *  STATIC PROTOTYPES
 **********************/

#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define ESP_APP_ID 0x55
#define SAMPLE_DEVICE_NAME "IntelliWatch"
#define SVC_INST_ID 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
 *  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
 */
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE 1024
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

/* Attributes State Machine */
enum
{
  IDX_SVC,
  // IDX_CHAR_A,
  // IDX_CHAR_VAL_A,
  // IDX_CHAR_CFG_A,

  // IDX_CHAR_B,
  // IDX_CHAR_VAL_B,

  // IDX_CHAR_C,
  // IDX_CHAR_VAL_C,

  IDX_CHAR_SETTINGS,
  IDX_CHAR_SETTINGS_VAL,
  IDX_CHAR_SETTINGS_CFG,

  IDX_CHAR_DATA,
  IDX_CHAR_DATA_VAL,
  IDX_CHAR_DATA_CFG,
  HRS_IDX_NB,
};
typedef struct
{
  uint8_t *prepare_buf;
  int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static uint8_t adv_config_done = 0;

uint16_t watch_ble_gatt_handle_table[HRS_IDX_NB];

static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xFF,
    0x00,
    0x00,
    0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, // test_manufacturer,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst
{
  esp_gatts_cb_t gatts_cb;
  uint16_t gatts_if;
  uint16_t app_id;
  uint16_t conn_id;
  uint16_t service_handle;
  esp_gatt_srvc_id_t service_id;
  uint16_t char_handle;
  esp_bt_uuid_t char_uuid;
  esp_gatt_perm_t perm;
  esp_gatt_char_prop_t property;
  uint16_t descr_handle;
  esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst watch_ble_gatt_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service */
static const uint16_t GATTS_SERVICE_UUID_TEST = 0x00FF;
// static const uint16_t GATTS_CHAR_UUID_TEST_A = 0xFF01;
// static const uint16_t GATTS_CHAR_UUID_TEST_B = 0xFF02;
// static const uint16_t GATTS_CHAR_UUID_TEST_C = 0xFF03;
static const uint16_t GATTS_CHAR_UUID_TEST_SETTINGS = 0xFF14;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
// static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
// static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t settings_read_reminder[2] = {0x00, 0x00};
// static const uint8_t char_value[4] = {0x11, 0x22, 0x33, 0x44};
uint8_t char_settings_value[20] = {0x01, 0x01, 0x04, 0x05, 0x01, 0x04, 0x01, 0x09, 0x01, 0x09, 0x08, 0x01, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
uint8_t char_data_value[20] = {0};
int global_mode = 0;
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
    {
        // Service Declaration
        [IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

        // /* Characteristic Declaration */
        // [IDX_CHAR_A] =
        //     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

        // /* Characteristic Value */
        // [IDX_CHAR_VAL_A] =
        //     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

        // /* Client Characteristic Configuration Descriptor */
        // [IDX_CHAR_CFG_A] =
        //     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

        // /* Characteristic Declaration */
        // [IDX_CHAR_B] =
        //     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

        // /* Characteristic Value */
        // [IDX_CHAR_VAL_B] =
        //     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

        // /* Characteristic Declaration */
        // [IDX_CHAR_C] =
        //     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

        // /* Characteristic Value */
        // [IDX_CHAR_VAL_C] =
        //     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_C, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

        [IDX_CHAR_SETTINGS] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        [IDX_CHAR_SETTINGS_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_SETTINGS, ESP_GATT_PERM_READ, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_settings_value), (uint8_t *)char_settings_value}},

        [IDX_CHAR_SETTINGS_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(settings_read_reminder), (uint8_t *)settings_read_reminder}},

        [IDX_CHAR_DATA] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write_notify}},

        [IDX_CHAR_DATA_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_SETTINGS, ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_data_value), (uint8_t *)char_data_value}},

        [IDX_CHAR_DATA_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(settings_read_reminder), (uint8_t *)settings_read_reminder}},

};

static xQueueHandle TIM_queue;
static xQueueHandle Health_queue;
static xQueueHandle IMU_queue;
static xQueueHandle sntp_queue;
static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);
static void event_handler(lv_event_t *e);
void create_application(void *arg);

void run_imu(void);
static void imu_task(void *arg);
void get_bpm(void *param);
static void fast_scan(void);
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);
static void initialize_sntp(void);
void sntp(void *arg);
// wifi
#define DEFAULT_SSID "Potako"
#define DEFAULT_PWD "1q2w3e4r"
static const char *WIFI_TAG = "scan";
float sys_temperature = 0;
// lvgl objects
lv_obj_t *scr1;
lv_obj_t *scr2;

lv_obj_t *scr_lock;
lv_obj_t *scr_lock_pic;
lv_obj_t *scr_lock_label_Year_Month_Day;
lv_obj_t *scr_lock_btn_Year_Month_Day;
lv_obj_t *scr_lock_label_Hour_Minute_Second;
lv_obj_t *scr_lock_btn_Hour_Minute_Second;
lv_obj_t *scr_lock_label_Weekday;
lv_obj_t *scr_lock_btn_Weekday;
lv_obj_t *scr_lock_btn_Enter;
lv_obj_t *scr_lock_btn_label;

lv_obj_t *scr_main;
lv_obj_t *main_tabview_1;
lv_obj_t *main_tab_1;
lv_obj_t *main_tab_1_label_Year_Month_Day;
lv_obj_t *main_tab_1_label_Hour_Minute_Second;
lv_obj_t *main_tab_1_btn1_label_sntp;
lv_obj_t *main_tab_1_label_sntp_status;
lv_obj_t *main_tab_1_btn1_sntp;

lv_obj_t *main_tab_2;
lv_obj_t *main_tab_3;
lv_obj_t *main_tab_4;
lv_obj_t *main_tab_5;
lv_obj_t *main_tab_6;
lv_obj_t *scr_remote;

lv_obj_t *scr_lowpower;

lv_obj_t *scr_settings;
lv_obj_t *btn1;
lv_obj_t *btn2;
lv_obj_t *btn3;
lv_obj_t *btn4;
lv_obj_t *btn5;
lv_obj_t *label_health_bpm;
lv_obj_t *label_health_spo2;

typedef struct
{
  unsigned int brightness;
  unsigned int vibration_en;
  unsigned int power_mode;
  unsigned int year;
  unsigned int month;
  unsigned int day;
} settings_t;

#define settings_size 6
nvs_handle_t my_handle;
#define key_brightness "brightness"
#define key_vibration_en "vibration_en"
#define key_power_mode "power_mode"
#define key_year "year"
#define key_month "month"
#define key_day "day"
uint32_t writebuf[settings_size] = {0};
int32_t readbuf[settings_size] = {0};
settings_t settings = {
    .brightness = 114,
    .vibration_en = 1,
    .power_mode = 0,
    .year = 2023,
    .month = 11,
    .day = 19,
};
settings_t test = {
    .brightness = 0,
    .vibration_en = 0,
    .power_mode = 0,
    .year = 0,
    .month = 0,
    .day = 0,
};

/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;

// wifi tasks
int wifi_connect_status = 0;
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    esp_wifi_connect();
    wifi_connect_status = 0;
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    // esp_wifi_connect();
    wifi_connect_status = 0;
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(WIFI_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    wifi_connect_status = 1;
  }
}

static void fast_scan(void)
{
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

  // Initialize default station as network interface instance (esp-netif)
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  // Initialize and start WiFi
  wifi_config_t wifi_config = {
      .sta = {
          .ssid = DEFAULT_SSID,
          .password = DEFAULT_PWD,
          .scan_method = WIFI_FAST_SCAN,
          .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
          .threshold.rssi = -127,
          .threshold.authmode = WIFI_AUTH_OPEN,
      },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  for (int ii = 0; ii < 6; ii++)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS); //!
    sys_sleep_timer_countstart = esp_timer_get_time() / 1000000;
    ESP_LOGI("WIFI", "Waiting for wifi to be connected... (%d/5)", ii);
    if (wifi_connect_status == 0 && ii == 5)
    {
      ESP_LOGW("WIFI", "wifi connect failed!");
      break;
    }
    if (wifi_connect_status == 1)
    {
      sntp(NULL);
      break;
    }
  }
}
void time_sync_notification_cb(struct timeval *tv)
{
  ESP_LOGI("SNTP", "Notification of a time synchronization event");
  int status = 1;
  xQueueSend(sntp_queue, &status, 0);
}
static void initialize_sntp(void)
{
  ESP_LOGI("SNTP", "Initializing SNTP");
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "ntp.ntsc.ac.cn");
  sntp_setservername(1, "pool.ntp.org");
  sntp_setservername(2, "time.windows.com");
  sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
  sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
  sntp_init();
}
void sntp(void *arg)
{
  // initialize_sntp();
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  sys_sleep_timer_countstart = esp_timer_get_time() / 1000000;
  // ESP_ERROR_CHECK( esp_event_loop_create_default() );

  /**
   * NTP server address could be aquired via DHCP,
   * see LWIP_DHCP_GET_NTP_SRV menuconfig option
   */
#ifdef LWIP_DHCP_GET_NTP_SRV
  sntp_servermode_dhcp(1);
#endif

  /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
   * Read "Establishing Wi-Fi or Ethernet Connection" section in
   * examples/protocols/README.md for more information about this function.
   */
  // ESP_ERROR_CHECK(example_connect());

  initialize_sntp();

  // wait for time to be set
  time_t now = 0;
  struct tm timeinfo = {0};
  struct tm systime = {0};
  int retry = 0;
  int status = 0;
  const int retry_count = 15;
  while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
  {
    sys_sleep_timer_countstart = esp_timer_get_time() / 1000000;
    ESP_LOGI("SNTP", "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
  xQueueReceive(sntp_queue, &status, 0);
  if (status == 1)
  {
    printf("sntp success\n");
  }
  else
  {
    printf("sntp Failed\n");
  }
  esp_err_t ret = esp_wifi_disconnect();
  if (ret == ESP_OK)
  {
    printf("WiFi disconnected\n");
  }
  else
  {
    printf("Failed to disconnect WiFi: %s\n", esp_err_to_name(ret));
  }
  ret = esp_wifi_stop();
  if (ret == ESP_OK)
  {
    printf("WiFi stopped\n");
  }
  else
  {
    printf("Failed to stop WiFi: %s\n", esp_err_to_name(ret));
  }
}

// ble tasks

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
  switch (event)
  {
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    adv_config_done &= (~ADV_CONFIG_FLAG);
    if (adv_config_done == 0)
    {
      esp_ble_gap_start_advertising(&adv_params);
    }
    break;
  case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
    adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
    if (adv_config_done == 0)
    {
      esp_ble_gap_start_advertising(&adv_params);
    }
    break;
  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    /* advertising start complete event to indicate advertising start successfully or failed */
    if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
    {
      ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
    }
    else
    {
      ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
    }
    break;
  case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
    if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
    {
      ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
    }
    else
    {
      ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
    }
    break;
  case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
    ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
             param->update_conn_params.status,
             param->update_conn_params.min_int,
             param->update_conn_params.max_int,
             param->update_conn_params.conn_int,
             param->update_conn_params.latency,
             param->update_conn_params.timeout);
    break;
  default:
    break;
  }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
  ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
  esp_gatt_status_t status = ESP_GATT_OK;
  if (prepare_write_env->prepare_buf == NULL)
  {
    prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
    prepare_write_env->prepare_len = 0;
    if (prepare_write_env->prepare_buf == NULL)
    {
      ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
      status = ESP_GATT_NO_RESOURCES;
    }
  }
  else
  {
    if (param->write.offset > PREPARE_BUF_MAX_SIZE)
    {
      status = ESP_GATT_INVALID_OFFSET;
    }
    else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
    {
      status = ESP_GATT_INVALID_ATTR_LEN;
    }
  }
  /*send response when param->write.need_rsp is true */
  if (param->write.need_rsp)
  {
    esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
    if (gatt_rsp != NULL)
    {
      gatt_rsp->attr_value.len = param->write.len;
      gatt_rsp->attr_value.handle = param->write.handle;
      gatt_rsp->attr_value.offset = param->write.offset;
      gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
      memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
      esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
      if (response_err != ESP_OK)
      {
        ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
      }
      free(gatt_rsp);
    }
    else
    {
      ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
    }
  }
  if (status != ESP_GATT_OK)
  {
    return;
  }
  memcpy(prepare_write_env->prepare_buf + param->write.offset,
         param->write.value,
         param->write.len);
  prepare_write_env->prepare_len += param->write.len;
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
  if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf)
  {
    esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
  }
  else
  {
    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
  }
  if (prepare_write_env->prepare_buf)
  {
    free(prepare_write_env->prepare_buf);
    prepare_write_env->prepare_buf = NULL;
  }
  prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
  uint16_t handle = param->write.handle;
  switch (event)
  {
  case ESP_GATTS_REG_EVT:
  {
    esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
    if (set_dev_name_ret)
    {
      ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
    }
    // config adv data
    esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret)
    {
      ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
    }
    adv_config_done |= ADV_CONFIG_FLAG;
    // config scan response data
    ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
    if (ret)
    {
      ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
    }
    adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
    if (create_attr_ret)
    {
      ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
    }
  }
  break;
  case ESP_GATTS_READ_EVT:
    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
    if (handle == watch_ble_gatt_handle_table[IDX_CHAR_SETTINGS_VAL])
    {
      ESP_LOGI(GATTS_TABLE_TAG, "Settings value read");
      global_mode = 1;
    }
    break;
  case ESP_GATTS_WRITE_EVT:
    if (!param->write.is_prep)
    {
      // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
      ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
      esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
      if (watch_ble_gatt_handle_table[IDX_CHAR_SETTINGS_VAL] == param->write.handle)
      {
        ESP_LOGI(GATTS_TABLE_TAG, "Settings value written");
        char_settings_value[param->write.len - 1] = param->write.value[1];
      }
      if (watch_ble_gatt_handle_table[IDX_CHAR_SETTINGS_CFG] == param->write.handle)
      {
        ESP_LOGI(GATTS_TABLE_TAG, "Settings cfg written, length: %d, value: %x", param->write.len, param->write.value[0]);
        uint8_t temp[4];
        temp[0] = rand() % 255;
        temp[1] = rand() % 255;
        temp[2] = rand() % 255;
        temp[3] = rand() % 255;
        char signforsettings[17] = "this is settings";
        esp_ble_gatts_send_indicate(watch_ble_gatt_profile_tab[0].gatts_if, watch_ble_gatt_profile_tab[0].conn_id, watch_ble_gatt_handle_table[IDX_CHAR_SETTINGS_VAL],
                                    sizeof(temp), (uint8_t *)temp, false); // indicate
        esp_ble_gatts_send_indicate(watch_ble_gatt_profile_tab[0].gatts_if, watch_ble_gatt_profile_tab[0].conn_id, watch_ble_gatt_handle_table[IDX_CHAR_SETTINGS_VAL],
                                    sizeof(signforsettings), (uint8_t *)signforsettings, false); // notify
      }

      if (watch_ble_gatt_handle_table[IDX_CHAR_DATA_VAL] == param->write.handle)
      {
        ESP_LOGI(GATTS_TABLE_TAG, "Data value written");
        uint8_t temp[20] = {0};
        printf("char_data_value: %s\n", char_data_value);
        memcpy(temp, char_data_value, sizeof(char_data_value));
        for (int i = 0; i < sizeof(temp); i++)
        {
          printf("%d ", temp[i]);
        }
        printf("\n");
        esp_ble_gatts_set_attr_value(watch_ble_gatt_handle_table[IDX_CHAR_SETTINGS_VAL], sizeof(temp), &temp);
      }
      char signs[13] = "this is data";
      if (watch_ble_gatt_handle_table[IDX_CHAR_DATA_CFG] == param->write.handle)
      {
        ESP_LOGI(GATTS_TABLE_TAG, "Data cfg written, length: %d, value: %x", param->write.len, param->write.value[0]);
        esp_ble_gatts_send_indicate(watch_ble_gatt_profile_tab[0].gatts_if, watch_ble_gatt_profile_tab[0].conn_id, watch_ble_gatt_handle_table[IDX_CHAR_DATA_VAL],
                                    sizeof(signs), (uint8_t *)signs, false); // indicate
      }
      /* send response when param->write.need_rsp is true*/
      if (param->write.need_rsp)
      {
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
      }
    }
    else
    {
      /* handle prepare write */
      example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
    }
    break;
  case ESP_GATTS_EXEC_WRITE_EVT:
    // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
    example_exec_write_event_env(&prepare_write_env, param);
    break;
  case ESP_GATTS_MTU_EVT:
    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
    break;
  case ESP_GATTS_CONF_EVT:
    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
    break;
  case ESP_GATTS_START_EVT:
    ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
    break;
  case ESP_GATTS_CONNECT_EVT:
    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
    esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
    esp_ble_conn_update_params_t conn_params = {0};
    memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
    conn_params.latency = 0;
    conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
    conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
    conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
    // start sent the update connection parameters to the peer device.
    esp_ble_gap_update_conn_params(&conn_params);
    break;
  case ESP_GATTS_DISCONNECT_EVT:
    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
    esp_ble_gap_start_advertising(&adv_params);
    break;
  case ESP_GATTS_CREAT_ATTR_TAB_EVT:
  {
    if (param->add_attr_tab.status != ESP_GATT_OK)
    {
      ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
    }
    else if (param->add_attr_tab.num_handle != HRS_IDX_NB)
    {
      ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)",
               param->add_attr_tab.num_handle, HRS_IDX_NB);
    }
    else
    {
      ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
      memcpy(watch_ble_gatt_handle_table, param->add_attr_tab.handles, sizeof(watch_ble_gatt_handle_table));
      esp_ble_gatts_start_service(watch_ble_gatt_handle_table[IDX_SVC]);
    }
    break;
  }
  case ESP_GATTS_STOP_EVT:
  case ESP_GATTS_OPEN_EVT:
  case ESP_GATTS_CANCEL_OPEN_EVT:
  case ESP_GATTS_CLOSE_EVT:
  case ESP_GATTS_LISTEN_EVT:
  case ESP_GATTS_CONGEST_EVT:
  case ESP_GATTS_UNREG_EVT:
  case ESP_GATTS_DELETE_EVT:
  default:
    break;
  }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

  /* If event is register event, store the gatts_if for each profile */
  if (event == ESP_GATTS_REG_EVT)
  {
    if (param->reg.status == ESP_GATT_OK)
    {
      watch_ble_gatt_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
    }
    else
    {
      ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
               param->reg.app_id,
               param->reg.status);
      return;
    }
  }
  do
  {
    int idx;
    for (idx = 0; idx < PROFILE_NUM; idx++)
    {
      /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
      if (gatts_if == ESP_GATT_IF_NONE || gatts_if == watch_ble_gatt_profile_tab[idx].gatts_if)
      {
        if (watch_ble_gatt_profile_tab[idx].gatts_cb)
        {
          watch_ble_gatt_profile_tab[idx].gatts_cb(event, gatts_if, param);
        }
      }
    }
  } while (0);
}

void ble_init(void)
{
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_err_t ret = esp_bt_controller_init(&bt_cfg);
  if (ret)
  {
    ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret)
  {
    ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_init();
  if (ret)
  {
    ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_enable();
  if (ret)
  {
    ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_ble_gatts_register_callback(gatts_event_handler);
  if (ret)
  {
    ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
    return;
  }

  ret = esp_ble_gap_register_callback(gap_event_handler);
  if (ret)
  {
    ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
    return;
  }

  ret = esp_ble_gatts_app_register(ESP_APP_ID);
  if (ret)
  {
    ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
    return;
  }

  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
  if (local_mtu_ret)
  {
    ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
  }
}

void ble_deinit(void)
{
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
}
// k值的计算方法。
// 符号:Sr-采样率(sampling rate,次/秒),f-截止频率(Hz),Pi-圆周率(3.14...)
// k=(2*Pi*f)/Sr

void Sys_Temp_Task(void *arg)
{
  temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
  temp_sensor_get_config(&temp_sensor);
  ESP_LOGI(TAG, "default dac %d, clk_div %d", temp_sensor.dac_offset, temp_sensor.clk_div);
  temp_sensor.dac_offset = TSENS_DAC_DEFAULT; // DEFAULT: range:-10℃ ~  80℃, error < 1℃.
  temp_sensor_set_config(temp_sensor);
  temp_sensor_start();
  while (1)
  {
    temp_sensor_read_celsius(&sys_temperature);
    ESP_LOGI("SYS_TEMPERATURE", "%.2f\n", sys_temperature);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
void BH1750_Init(void)
{
  i2c_write_byte(I2C_MASTER_NUM, I2C_ADDRESS_BH1750, NULL, BH1750_POWER_ON);
  i2c_write_byte(I2C_MASTER_NUM, I2C_ADDRESS_BH1750, NULL, BH1750_CONT_H_RES_MODE);
}
void BH1750_Read(uint16_t *light)
{
  uint8_t data[2] = {0};
  uint16_t lux = 0;
  i2c_read_bytes(I2C_MASTER_NUM, I2C_ADDRESS_BH1750, NULL, &data, 2);
  lux = ((data[0] << 8) | data[1]) / 1.2;
  *light = lux;
}
typedef struct
{
  float bpm;
  float spo2;
  int statereport; // 0:idle 1:sensing 2:sense over
} health_t;
health_t health;
max30102_config_t max30102 = {};
void get_bpm(void *param)
{
  printf("MAX30102 Test\n");
  int ii = 0;
  int ms_start = (int)(((float)esp_timer_get_time()) / 1000);
  int ms_count = 0;

  max30102_data_t result = {};
  float resultqueue[10] = {0};
  /*ESP_ERROR_CHECK(max30102_print_registers(&max30102));*/
  max30102_en(1);
  while (ms_count < 20000 && ii < 5)
  {
    ms_count = esp_timer_get_time() / 1000 - ms_start;
    ESP_LOGI("health", "ms_count:%d", ms_count);
    // Update sensor, saving to "result"
    ESP_ERROR_CHECK(max30102_update(&max30102, &result));
    if (result.pulse_detected)
    {
      printf("BEAT\n");
      printf("BPM: %f | SpO2: %f%%\n", result.heart_bpm, result.spO2);
      resultqueue[ii] = result.heart_bpm;
      resultqueue[ii + 1] = result.spO2;
      ii++;
    }
    // Update rate: 100Hz

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  max30102_en(0);
  for (int i = 0; i < ii; i++)
  {
    health.bpm += resultqueue[i];
    health.spo2 += resultqueue[i + 1];
  }
  if (ii > 3)
  {
    health.bpm /= (ii);
    health.spo2 /= (ii);
    health.statereport = 2;
    // lv_label_set_text_fmt(label_health_bpm, "BPM:%.1f", health.bpm);
    // lv_label_set_text_fmt(label_health_spo2, "SpO2:%.1f", health.spo2);
    printf("Success.BPM:%.1f\n", health.bpm);
    printf("Success.SpO2:%.1f\n", health.spo2);
  }
  else
  {
    ESP_LOGW("health", "not enough data");
  }
  // delete task
  health.statereport = 0;
  vTaskDelete(NULL);
}

typedef struct
{
  int hour;
  int minute;
  int second;
  struct
  {
    int year;
    int month;
    int day;
    int weekday;
  } date;
} user_time_t;
static const char *weekdayenum[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

user_time_t user_time;
timer_config_t config = {
    .divider = 80,
    .counter_dir = TIMER_COUNT_UP,
    .counter_en = 0,
    .alarm_en = 1,
    .intr_type = TIMER_INTR_LEVEL,
    .auto_reload = 1,
};

void IRAM_ATTR timer_group0_isr(void *arg)
{
  timer_spinlock_take(TIMER_GROUP_0);
  // ESP_LOGW("timer", "triggered");
  static int mode = 1;
  timer_group_clr_intr_status_in_isr(0, 0);
  timer_group_enable_alarm_in_isr(0, 0);
  xQueueSendFromISR(TIM_queue, &mode, NULL);
  timer_spinlock_give(TIMER_GROUP_0);
}
ledc_timer_config_t backlight_timer = {
    .duty_resolution = LEDC_TIMER_12_BIT, // resolution of PWM duty
    .freq_hz = 2000,                      // frequency of PWM signal
    .speed_mode = LEDC_LOW_SPEED_MODE,    // timer mode
    .timer_num = LEDC_TIMER_1,            // timer index
    .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
};
ledc_channel_config_t backlight = {
    .channel = LEDC_CHANNEL_0, // Left_Pos
    .duty = 0,
    .gpio_num = 47,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .hpoint = 0,
    .timer_sel = LEDC_TIMER_1,
};
ledc_channel_config_t LED_RED = {
    .channel = LEDC_CHANNEL_1,
    .duty = 0,
    .gpio_num = 5,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .hpoint = 0,
    .timer_sel = LEDC_TIMER_1,
};
ledc_channel_config_t LED_GREEN = {
    .channel = LEDC_CHANNEL_2,
    .duty = 0,
    .gpio_num = 6,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .hpoint = 0,
    .timer_sel = LEDC_TIMER_1,
};
ledc_channel_config_t LED_BLUE = {
    .channel = LEDC_CHANNEL_3,
    .duty = 0,
    .gpio_num = 7,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .hpoint = 0,
    .timer_sel = LEDC_TIMER_1,
};
typedef struct
{
  struct
  {
    float Yaw;
    float Pitch;
    float Roll;
  } Angle;
  struct
  {
    float X;
    float Y;
    float Z;
    float X_filt;
    float Y_filt;
    float Z_filt;
  } Accel;
  float Temp;
} imu_t;
// init imu
imu_t imu = {
    .Angle.Yaw = 0,
    .Angle.Pitch = 0,
    .Angle.Roll = 0,
    .Accel.X = 0,
    .Accel.Y = 0,
    .Accel.Z = 0,
    .Accel.X_filt = 0,
    .Accel.Y_filt = 0,
    .Accel.Z_filt = 0,
    .Temp = 0};

typedef struct
{
  float k;    // 滤波系数
  float lVal; // 上次计算值
} rcPara_t;
rcPara_t rcParaHp = {.k = (2 * 3.14159f * Fstop / Filt_SAMPLE_FREQ_Hz)};
// 低通滤波：
// rcPara-指向滤波参数
// val-采样值
// 返回值-滤波结果
float rcLfFilter(rcPara_t *rcPara, float val)
{
  rcPara->lVal = ((float)val * rcPara->k + rcPara->lVal * (1 - rcPara->k));
  return rcPara->lVal;
}

// 高通滤波：
float rcHpFilter(rcPara_t *rcPara, float val)
{
  rcPara->lVal = ((float)val * rcPara->k + rcPara->lVal * (rcPara->k));
  return -(val - rcPara->lVal); // 滤波结果
  // return (val-rcPara->lVal);//如果直接返回滤波结果，滤波后图像是倒转的，在心电图等一些场合，需要将图像再镜像过来
}

static void guiTask(void *pvParameter)
{
  (void)pvParameter;
  xGuiSemaphore = xSemaphoreCreateMutex();

  lv_init();

  /* Initialize SPI or I2C bus used by the drivers */
  lvgl_driver_init();

  lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
  assert(buf1 != NULL);
  /* Use double buffered when not working with monochrome displays */
  lv_color_t *buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
  assert(buf2 != NULL);
  static lv_disp_draw_buf_t disp_buf;
  uint32_t size_in_px = DISP_BUF_SIZE;
  /* Initialize the working buffer depending on the selected display. */
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, size_in_px);

  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LV_HOR_RES_MAX;
  disp_drv.ver_res = LV_VER_RES_MAX;
  disp_drv.flush_cb = disp_driver_flush;
  disp_drv.draw_buf = &disp_buf;
  lv_disp_drv_register(&disp_drv);
#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
  ESP_LOGI("LVGL", "TOUCH INIT");
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.read_cb = touch_driver_read;
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  lv_indev_drv_register(&indev_drv);
#endif
  /* Create and start a periodic timer interrupt to call lv_tick_inc */
  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &lv_tick_task,
      .name = "periodic_gui"};
  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

  /* Create the demo application */
  // create_application();
  xTaskCreate(create_application, "create_application", 4096 * 8, NULL, 8, NULL);
  while (1)
  {
    /* Delay 1 tick assumes FreeRTOS tick is 10ms */
    vTaskDelay(pdMS_TO_TICKS(10));
    lv_task_handler();
    // printf("guirunning");
    /* Try to take the semaphore, call lvgl related function on success */
    if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
    {
      lv_task_handler();
      xSemaphoreGive(xGuiSemaphore);
    }
  }
}

void backlight_control_manual(uint16_t light)
{
  if (light > 1000)
  {
    light = 1000;
  }
  light = (int)(4095 - ((float)light / 1000) * 4095);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, light);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void backlight_control_task(void *arg)
{
  int sys_backlight_past = 0;
  while (1)
  {
    if (sys_backlight_past > sys_backlight)
    {
      sys_backlight_past--;
      backlight_control_manual(sys_backlight_past);
    }
    else if (sys_backlight_past < sys_backlight)
    {
      sys_backlight_past++;
      backlight_control_manual(sys_backlight_past);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void touch_scan(void *arg)
{
  while (1)
  {
    if (!rtc_gpio_get_level(TOUCH_IRQ))
    {
      ESP_LOGI("touch", "touched");
      sys_sleep_timer_countstart = esp_timer_get_time() / 1000000;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void imu_scan(void *arg)
{
  while (1)
  {
    if (rtc_gpio_get_level(IMU_IRQ))
      sys_sleep_timer_countstart = esp_timer_get_time() / 1000000;
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void key_scan(void *arg)
{
  int key_status = 0;
  int tick = 0;
  int tick_last = 0;
  while (1)
  {
    tick = esp_timer_get_time() / 1000;
    if (rtc_gpio_get_level(KEY_IRQ) == 0)
    {
      if (tick - tick_last > 20 && key_status == 0)
      {
        key_status = 1;
        sys_sleep_timer_countstart = esp_timer_get_time() / 1000000;
        ESP_LOGI("key", "key pressed");
        if (sys_state == 2)
        {
          sys_sleep_timer_countstart = esp_timer_get_time() / 1000;
        }
        if (sys_state == 0 || sys_state == 1)
        {
          sys_state = 3;
        }
      }
    }
    else if (rtc_gpio_get_level(KEY_IRQ) && key_status == 1)
    {
      key_status = 0;
      tick_last = tick;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

static void event_handler(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_CLICKED)
  {
    LV_LOG_USER("Clicked");
    ESP_LOGI("Main", "Clicked");
    lv_obj_t *btn = lv_event_get_target(e);
    if (btn == scr_lock_btn_Enter)
    {
      lv_scr_load(scr_main);
    }
    // printf("btn:%s", (char *)btn);
    // if (btn == btn1)
    // {
    //   // ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    //   // ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    //   DRV2605_Play(4);
    //   // lv_scr_load(scr2);
    // }
    // else if (btn == btn2)
    // {
    //   // ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 4000);
    //   // ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    //   DRV2605_Play(4); // good
    //   // lv_scr_load(scr1);
    // }
    // else if (btn == btn3)
    // {
    //   DRV2605_Play(1);
    //   if (health.statereport == 0)
    //     xTaskCreate(get_bpm, "Get BPM", 4096 * 2, NULL, 8, NULL);
    //   else
    //   {
    //     ESP_LOGW("health", "already sensing!!!");
    //   }
    // }
  }
  else if (code == LV_EVENT_VALUE_CHANGED)
  {
    LV_LOG_USER("Toggled");
    ESP_LOGI("Main", "Toggled");
  }
  else if (code == LV_EVENT_PRESSED)
  {
    LV_LOG_USER("PRESSED");
    ESP_LOGI("Main", "PRESSED");
  }
}
void create_application(void *arg)
{

  lv_style_t style_1;
  lv_style_init(&style_1);
  vTaskDelay(2 / portTICK_PERIOD_MS);
  lv_style_set_bg_color(&style_1, lv_color_hex(0xFDF5E6));
  lv_style_set_text_font(&style_1, &lv_font_montserrat_14);
  vTaskDelay(2 / portTICK_PERIOD_MS);
  lv_style_t style_2;
  lv_style_init(&style_2);
  vTaskDelay(2 / portTICK_PERIOD_MS);
  lv_style_set_bg_color(&style_2, lv_color_hex(0x48D1CC));
  lv_style_set_text_font(&style_2, &lv_font_montserrat_14);
  vTaskDelay(2 / portTICK_PERIOD_MS);
  lv_style_t style_3;
  lv_style_init(&style_3);
  vTaskDelay(2 / portTICK_PERIOD_MS);
  lv_style_set_text_font(&style_3, &lv_font_montserrat_28);
  vTaskDelay(2 / portTICK_PERIOD_MS);
  // lv_style_t style_4;
  // lv_style_init(&style_4);
  // lv_style_set_text_font(&style_4, &lv_font_montserrat_14);

  scr_lock = lv_obj_create(NULL);
  lv_scr_load(scr_lock);
  LV_IMG_DECLARE(hikari);
  lv_obj_t *img = lv_img_create(lv_scr_act());
  lv_img_set_src(img, &hikari);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  lv_obj_set_pos(img, 0, 0); // 设置图片位置
  lv_img_set_angle(img, 0);  // 设置图片旋转角
  lv_img_set_zoom(img, 300); // 设置图片缩放
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // scr_lock
  scr_lock_btn_Enter = lv_btn_create(scr_lock);
  lv_obj_add_event_cb(scr_lock_btn_Enter, event_handler, LV_EVENT_ALL, NULL);
  lv_obj_align(scr_lock_btn_Enter, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
  lv_obj_set_size(scr_lock_btn_Enter, 70, 70);
  scr_lock_btn_label = lv_label_create(scr_lock_btn_Enter);
  lv_label_set_text_fmt(scr_lock_btn_label, "Unlock");
  lv_obj_align(scr_lock_btn_label, LV_ALIGN_CENTER, 0, 0);
  vTaskDelay(10 / portTICK_PERIOD_MS);

  scr_lock_btn_Year_Month_Day = lv_btn_create(scr_lock);
  scr_lock_label_Year_Month_Day = lv_label_create(scr_lock_btn_Year_Month_Day);
  scr_lock_btn_Hour_Minute_Second = lv_btn_create(scr_lock);
  scr_lock_label_Hour_Minute_Second = lv_label_create(scr_lock_btn_Hour_Minute_Second);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // scr_lock_btn_Weekday = lv_btn_create(scr_lock);
  // scr_lock_label_Weekday = lv_label_create(scr_lock_btn_Weekday);
  lv_obj_align(scr_lock_btn_Year_Month_Day, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_align(scr_lock_btn_Hour_Minute_Second, LV_ALIGN_BOTTOM_LEFT, 0, -60);
  // lv_obj_align(scr_lock_btn_Weekday, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_align(scr_lock_label_Year_Month_Day, LV_ALIGN_CENTER, 0, 0);
  lv_obj_align(scr_lock_label_Hour_Minute_Second, LV_ALIGN_CENTER, 0, 0);
  // lv_obj_align(scr_lock_label_Weekday, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_size(scr_lock_btn_Year_Month_Day, 150, 60);
  lv_obj_set_size(scr_lock_btn_Hour_Minute_Second, 150, 30);
  lv_obj_add_style(scr_lock_btn_Year_Month_Day, &style_2, 0);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  lv_obj_add_style(scr_lock_btn_Hour_Minute_Second, &style_2, 0);
  lv_obj_add_style(scr_lock_label_Year_Month_Day, &style_3, 0);
  lv_obj_add_style(scr_lock_label_Hour_Minute_Second, &style_3, 0);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // scr_main
  scr_main = lv_obj_create(NULL);
  vTaskDelay(10 / portTICK_PERIOD_MS);

  lv_obj_t *main_tabview_1;
  main_tabview_1 = lv_tabview_create(scr_main, LV_DIR_LEFT, 70);
  lv_obj_align(main_tabview_1, NULL, 0, 0);
  lv_obj_add_style(main_tabview_1, &style_1, 0);
  lv_obj_set_size(main_tabview_1, LV_HOR_RES, LV_VER_RES);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  main_tab_1 = lv_tabview_add_tab(main_tabview_1, "Time");
  main_tab_2 = lv_tabview_add_tab(main_tabview_1, "Health\nData");
  main_tab_3 = lv_tabview_add_tab(main_tabview_1, "Settings");
  main_tab_4 = lv_tabview_add_tab(main_tabview_1, "Remote");
  main_tab_5 = lv_tabview_add_tab(main_tabview_1, "Tab 5");
  main_tab_6 = lv_tabview_add_tab(main_tabview_1, "Tab 6");
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // tab1
  main_tab_1_label_Hour_Minute_Second = lv_label_create(main_tab_1);
  main_tab_1_label_Year_Month_Day = lv_label_create(main_tab_1);
  main_tab_1_btn1_sntp = lv_btn_create(main_tab_1);
  main_tab_1_btn1_label_sntp = lv_label_create(main_tab_1_btn1_sntp);
  lv_obj_add_event_cb(main_tab_1_btn1_sntp, event_handler, LV_EVENT_ALL, NULL);
  main_tab_1_label_sntp_status = lv_label_create(main_tab_1);
  lv_obj_add_style(main_tab_1_label_Hour_Minute_Second, &style_3, 0);
  lv_obj_add_style(main_tab_1_label_Year_Month_Day, &style_3, 0);
  vTaskDelay(10 / portTICK_PERIOD_MS);

  lv_obj_align(main_tab_1_label_Hour_Minute_Second, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_align(main_tab_1_label_Year_Month_Day, LV_ALIGN_TOP_LEFT, 0, 40);
  lv_obj_align(main_tab_1_btn1_sntp, LV_ALIGN_TOP_LEFT, 0, 80);
  lv_obj_align(main_tab_1_label_sntp_status, LV_ALIGN_TOP_LEFT, 0, 120);
  vTaskDelay(10 / portTICK_PERIOD_MS);

  lv_obj_set_size(main_tab_1_btn1_sntp, 150, 30);
  lv_label_set_text_fmt(main_tab_1_btn1_label_sntp, "SNTP Cali");
  // scr2 = lv_obj_create(lv_scr_act());

  // /* 设置页面的大小和位置 */
  // lv_obj_set_size(scr1, LV_HOR_RES, LV_VER_RES);
  // lv_obj_set_size(scr2, LV_HOR_RES, LV_VER_RES);
  // lv_obj_add_style(scr2, &style_2, 0);
  // lv_anim_t a;
  // lv_anim_init(&a);
  // lv_anim_set_var(&a, scr1);
  // lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_x);
  // lv_anim_set_values(&a, 0, -LV_HOR_RES);
  // lv_anim_set_time(&a, 500);
  // lv_anim_start(&a);

  while (1)
  {
    lv_label_set_text_fmt(scr_lock_label_Year_Month_Day, "%d-%d-%d\n        %s",
                          user_time.date.year, user_time.date.month, user_time.date.day, weekdayenum[user_time.date.weekday]);
    lv_label_set_text_fmt(scr_lock_label_Hour_Minute_Second, "%02d:%02d:%02d", user_time.hour, user_time.minute, user_time.second);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}
/*
void create_application(void)
{
  //  Get the current screen
  // lv_obj_t * scr = lv_disp_get_scr_act(NULL);
  // Create a Label on the currently active screen
  scr1 = lv_obj_create(NULL);
  scr2 = lv_obj_create(NULL);
  // lv_obj_set_size(scr1, LV_HOR_RES / 2, LV_VER_RES);
  // lv_obj_set_size(scr2, LV_HOR_RES / 2, LV_VER_RES);
  // lv_obj_set_pos(scr1, 0, 0);
  // lv_obj_set_pos(scr2, LV_HOR_RES/2, 0);
  static lv_style_t style_1;
  lv_style_init(&style_1);
  lv_style_set_bg_color(&style_1, lv_color_hex(0x000000));
  lv_style_set_text_font(&style_1, &lv_font_montserrat_28);

  static lv_style_t style_2;
  lv_style_init(&style_2);
  lv_style_set_bg_color(&style_2, lv_color_hex(0xADD8E6));
  lv_style_set_text_font(&style_2, &lv_font_montserrat_14);

  static lv_style_t style_3;
  lv_style_init(&style_3);
  lv_style_set_bg_color(&style_3, lv_color_hex(0xFFB6C1));
  lv_style_set_text_font(&style_3, &lv_font_montserrat_14);
  // lv_obj_set_pos(scr,0,0);
  lv_obj_set_style_bg_color(scr2, lv_color_hex(0xADD8E6), 0);
  lv_scr_load(scr1);
  // lv_scr_load(scr2);
  //  Create a Tabview object
  lv_obj_t *tabview;
  tabview = lv_tabview_create(lv_scr_act(), LV_DIR_LEFT, 50);
  lv_obj_add_style(tabview, &style_2, 0);
  //  Add two tabs (you can add more if you want)
  lv_obj_t *tab1 = lv_tabview_add_tab(tabview, "Tab 1");
  lv_obj_t *tab2 = lv_tabview_add_tab(tabview, "Tab 2");

  // lv_tabview_set_tab_act(tabview, "Tab 1", LV_ANIM_ON);
  // lv_tabview_set_tab_act(tabview, "Tab 2", LV_ANIM_ON);
  // lv_tabview_set_sliding(tabview, true);
  // lv_tabview_set_anim_time(tabview, 300);
  //  Add content to the tabs
  lv_obj_t *label_tab1 = lv_label_create(tab1);
  lv_obj_add_style(label_tab1, &style_1, 0);
  lv_label_set_text_fmt(label_tab1, "Main Time");
  lv_obj_t *label_tab2 = lv_label_create(tab2);
  lv_label_set_text(label_tab2, "This is Tab 2");
  lv_obj_align(tab1, LV_ALIGN_OUT_BOTTOM_MID, 150, 50);

  lv_obj_t *label1 = lv_label_create(tab1);
  // Modify the Label's text
  lv_label_set_recolor(label1, true);
  lv_obj_t *label2 = lv_label_create(tab2);
  lv_label_set_recolor(label2, true);
  lv_obj_add_style(label2, &style_1, 0);
  // Align the Label to the center
  //  NULL means align on parent (which is the screen now)
  //   0, 0 at the end means an x, y offset after alignment
  lv_obj_align(label1, LV_ALIGN_OUT_TOP_LEFT, 30, 20);
  lv_obj_align(label2, LV_ALIGN_OUT_TOP_LEFT, 30, 70);
  lv_obj_t *label_btn1;

  // lv_obj_t *btn1 = lv_btn_create(lv_scr_act());
  btn1 = lv_btn_create(tab1);
  lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
  lv_obj_align(btn1, LV_ALIGN_CENTER, 0, 200);

  label_btn1 = lv_label_create(btn1);
  lv_label_set_text(label_btn1, "Button1");
  lv_obj_center(label_btn1);

  lv_obj_t *label_btn2;
  // lv_obj_t *btn2 = lv_btn_create(lv_scr_act());
  btn2 = lv_btn_create(tab2);
  lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, NULL);
  lv_obj_align(btn2, LV_ALIGN_CENTER, 150, 50);

  label_btn2 = lv_label_create(btn2);
  lv_label_set_text(label_btn2, "Button2");
  lv_obj_center(label_btn2);
  // label_health_bpm = lv_label_create(scr1);
  // label_health_spo2 = lv_label_create(scr1);

  // lv_obj_t *label_btn3;
  // label_btn3 = lv_label_create(btn3);
  // lv_obj_add_style(label_btn3, &style_2, 0);
  // lv_label_set_text(label_btn3, "Health Detect");
  // btn3 = lv_btn_create(scr1);
  // lv_obj_add_event_cb(btn3, event_handler, LV_EVENT_ALL, NULL);
  // lv_obj_add_style(btn3, &style_3, 0);
  // lv_obj_set_size(btn3, 120, 40); // Set the size of btn1
  // lv_obj_align(btn3, LV_ALIGN_CENTER, 0, 200);
  // // lv_obj_align(label_btn3, LV_ALIGN_CENTER, 0, 50);
  // lv_obj_align(label_health_bpm, LV_ALIGN_CENTER, 0, 220);
  // lv_obj_align(label_health_spo2, LV_ALIGN_CENTER, 0, 240);

  // lv_obj_t *sw1 = lv_switch_create(lv_scr_act());
  // lv_obj_add_event_cb(sw1, event_handler, LV_EVENT_ALL, NULL);
  // lv_obj_align(sw1, LV_ALIGN_OUT_TOP_MID, 50, 50);

  // lv_obj_t *label3 = lv_label_create(lv_scr_act());
  // lv_label_set_long_mode(label3, LV_LABEL_LONG_SCROLL_CIRCULAR); Circular scroll
  // lv_obj_set_width(label3, 150);
  // lv_label_set_text(label3, "It is a circularly scrolling text. ");
  // lv_obj_align(label3, LV_ALIGN_CENTER, 0, 40);
  // printf("created");
  int i = 0;
  int mode = 0;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1000);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  while (1) // flush data
  {
    lv_label_set_text_fmt(label1, "#ff0000 position#:%.1f , %.1f , %.1f",
                          imu.Angle.Yaw,
                          imu.Angle.Pitch,
                          imu.Angle.Roll);

    xQueueReceive(TIM_queue, &mode, 0);
    if (mode == 1)
    {
      i++;
      user_time.second++;
      if (user_time.second == 60)
      {
        user_time.second = 0;
        user_time.minute++;
      }
      if (user_time.minute == 60)
      {
        user_time.minute = 0;
        user_time.hour++;
      }
      if (user_time.hour == 24)
      {
        user_time.hour = 0;
      }

      // ESP_LOGW("timer", "triggered %d times", i);
      mode = 0;
    }
    lv_label_set_text_fmt(label2, "#0000ff user_time#:%02d:%02d:%02d", user_time.hour, user_time.minute, user_time.second);
    // lv_label_set_text_fmt(label1, "position:");

    // if (mode == 0)
    // {
    //   i += 10;
    //   if (i >= 4080)
    //   {
    //     mode = 1;
    //     i = 4095;
    //   }
    // }
    // else
    // {
    //   i -= 10;
    //   if (i <= 20)
    //   {
    //     mode = 0;
    //     i = 1;
    //   }
    // }

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}
*/

static void lv_tick_task(void *arg)
{
  (void)arg;

  lv_tick_inc(LV_TICK_PERIOD_MS);
}

calibration_t cal = {
    .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
    .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},
    .accel_offset = {.x = 0.020900, .y = 0.014688, .z = -0.002580},
    .accel_scale_lo = {.x = -0.992052, .y = -0.990010, .z = -1.011147},
    .accel_scale_hi = {.x = 1.013558, .y = 1.011903, .z = 1.019645},

    .gyro_bias_offset = {.x = 0.303956, .y = -1.049768, .z = -0.403782}};

/**
 * Transformation:
 *  - Rotate around Z axis 180 degrees
 *  - Rotate around X axis -90 degrees
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_accel_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -x;
  v->y = -z;
  v->z = -y;
}

/**
 * Transformation: to get magnetometer aligned
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_mag(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -y;
  v->y = z;
  v->z = -x;
}

void run_imu(void)
{

  ahrs_init(SAMPLE_FREQ_Hz, 0.8);
  uint64_t i = 0;
  while (true)
  {
    vector_t va, vg, vm;

    // Get the Accelerometer, Gyroscope and Magnetometer values.
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

    // Transform these values to the orientation of our device.
    // transform_accel_gyro(&va);
    // transform_accel_gyro(&vg);
    // transform_mag(&vm);

    // Apply the AHRS algorithm
    ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                va.x, va.y, va.z,
                vm.x, vm.y, vm.z);
    imu.Accel.X = va.x;
    imu.Accel.Y = va.y;
    imu.Accel.Z = va.z;
    // Print the data out every 10 items

    imu.Accel.X_filt = imu.Accel.Y - sin(imu.Angle.Roll * PI / 180.0) * cos(imu.Angle.Pitch * PI / 180.0) * 0.981;
    imu.Accel.Y_filt = imu.Accel.Z - cos(imu.Angle.Roll * PI / 180.0) * cos(imu.Angle.Pitch * PI / 180.0) * 0.981;
    imu.Accel.Z_filt = imu.Accel.X + sin(imu.Angle.Pitch * PI / 180.0) * 0.981;
    if (i++ % 10 == 0)
    {
      float temp;
      ESP_ERROR_CHECK(get_temperature_celsius(&imu.Temp));
      imu.Temp = imu.Temp - 5;
      // float heading, pitch, roll;
      ahrs_get_euler_in_degrees(&imu.Angle.Yaw, &imu.Angle.Pitch, &imu.Angle.Roll);

      // ESP_LOGI(TAG, "Yaw: %2.3f°, Pitch: %2.3f°, Roll: %2.3f°, Temp %2.3f°C,Accel:X=%2.3f,Y=%2.3f,Z=%2.3f,filt:X=%2.3f,Y=%2.3f,Z=%2.3f",
      //          imu.Angle.Yaw, imu.Angle.Pitch, imu.Angle.Roll, imu.Temp,
      //          imu.Accel.X, imu.Accel.Y, imu.Accel.Z,
      //          imu.Accel.X_filt,
      //          imu.Accel.Y_filt,
      //          imu.Accel.Z_filt);
      // Make the WDT happy
      vTaskDelay(0 / portTICK_PERIOD_MS);
    }

    imu_pause();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

static void imu_task(void *arg)
{

#ifdef CONFIG_CALIBRATION_MODE
  calibrate_gyro();
  calibrate_accel();
  calibrate_mag();
#else
  run_imu();
#endif

  // Exit
  vTaskDelay(100 / portTICK_PERIOD_MS);
  i2c_driver_delete(I2C_MASTER_NUM);

  vTaskDelete(NULL);
}

// main system functions

void sys_sleep_timer(void *arg)
{
  int sys_sleep_timer_count = 0;
  sys_sleep_timer_countstart = esp_timer_get_time() / 1000000;
  while (1)
  {
    sys_sleep_timer_count = esp_timer_get_time() / 1000000 - sys_sleep_timer_countstart;
    // get countstart else where,especially when task in active or touch is captured

    if (sys_sleep_timer_count >= PRE_PRE_SLEEP_TIME && sys_sleep_timer_count <= PRE_PRE_SLEEP_TIME + PRE_SLEEP_TIME)
    {
      sys_state = 1;
    }
    else if (sys_sleep_timer_count >= PRE_PRE_SLEEP_TIME + PRE_SLEEP_TIME && sys_sleep_timer_count <= PRE_PRE_SLEEP_TIME + PRE_SLEEP_TIME + SLEEP_TIME)
    {
      sys_state = 2;
    }
    else if (sys_sleep_timer_count >= PRE_PRE_SLEEP_TIME + PRE_SLEEP_TIME + SLEEP_TIME)
    {
      sys_state = 3;
    }
    else
    {
      sys_state = 0;
    }
    printf("sys_state:%d\n", sys_state);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void sys_state_machine(void *arg)
{
  int last_state = 0;
  int state_change_flag = 0;
  while (1)
  {
    if (last_state != sys_state)
    {
      state_change_flag = 1;
    }
    else
    {
      state_change_flag = 0;
    }
    last_state = sys_state;

    if (sys_state == 1) // pre_pre_sleep
    {
      if (state_change_flag == 1)
      {
        sys_backlight = 100;
      }
    }
    else if (sys_state == 2) // pre_sleep
    {
      if (state_change_flag == 1)
      {
        sys_backlight = 0;
      }
    }
    else if (sys_state == 3) // sleep
    {
      if (state_change_flag == 1)
      {
        esp_sleep_enable_ext0_wakeup(TOUCH_IRQ, 0);
        esp_sleep_enable_ulp_wakeup();
        mpu9250_wake_on_motion_config();
        init_ulp_program();
        ESP_LOGW("sys_state_machine", "going to sleep");
        esp_deep_sleep_start();
      }
    }
    else if (sys_state == 4) // pre_wakeup
    {
      if (state_change_flag == 1)
      {
        switch (cause)
        {
        case ESP_SLEEP_WAKEUP_EXT0:
          printf("Wake up from SCREEN\n");
          sys_state = 5;
          break;
        case ESP_SLEEP_WAKEUP_EXT1:
          // printf("Wake up from KEY\n");
          sys_state = 5;
          break;
        case ESP_SLEEP_WAKEUP_ULP:

          // check current posture
          if (rtc_gpio_get_level(KEY_IRQ) != 0)
          {
            ESP_LOGW("sys_state_machine", "???????????????????IMU pressed");
            i2c_mpu9250_init(&cal);
            xTaskCreate(run_imu, "run_imu", 1024 * 4, NULL, 10, NULL);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (imu.Angle.Pitch < 90 && imu.Angle.Pitch > 0) // pitch or roll
            {
              // wake up
              sys_state = 5;
            }
            else
              sys_state = 3;
          }
          else // key pressed
          {
            ESP_LOGW("sys_state_machine", "!!!!!!!!!!!!!!!!!!!key pressed");
            sys_state = 5;
          }
          break;
        case ESP_SLEEP_WAKEUP_TIMER:
          break;
        default:
          printf("first boot\n");
          sys_state = 5;
          // load from flash
          system_init();
          // sntp sync
          // drv2605
          DRV2605_Init();
          // fast_scan();
          // system init
        }
      }
    }
    else if (sys_state == 5) // wakeup
    {
      if (state_change_flag == 1)
      {
        ESP_LOGW("sys_state_machine", "init");
        system_init();
        sys_state = 0;
      }
    }
    else if (sys_state == -1)
    {
      // sleeping
      if (state_change_flag == 1)
      {
      }
    }
    else if (sys_state == 0)
    {
      // default
      if (state_change_flag == 1)
      {
        sys_backlight = 500;
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void system_load_from_flash()
{

  writebuf[0] = settings.brightness;
  writebuf[1] = settings.vibration_en;
  writebuf[2] = settings.power_mode;
  writebuf[3] = settings.year;
  writebuf[4] = settings.month;
  writebuf[5] = settings.day;
  esp_err_t err;
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  // Open
  printf("\n");
  printf("Opening Non-Volatile Storage (NVS) handle... ");
  nvs_handle_t my_handle;
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    printf("Done\n");

    // Read//do it every time the device boots
    {
      printf("Reading brightness from NVS ... ");
      err = nvs_get_i32(my_handle, key_brightness, &readbuf[0]);
      switch (err)
      {
      case ESP_OK:
        printf("Done\n");
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        printf("The value is not initialized yet!\n");
        break;
      default:
        printf("Error (%s) reading!\n", esp_err_to_name(err));
      }
      test.brightness = readbuf[0];
      // printf("test brightness:%d\n", test.brightness);
      printf("Reading vibration_en from NVS ... ");
      err = nvs_get_i32(my_handle, key_vibration_en, &readbuf[1]);
      switch (err)
      {
      case ESP_OK:
        printf("Done\n");
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        printf("The value is not initialized yet!\n");
        break;
      default:
        printf("Error (%s) reading!\n", esp_err_to_name(err));
      }
      test.vibration_en = readbuf[1];
      // printf("test vibration_en:%d\n", test.vibration_en);
      printf("Reading power_mode from NVS ... ");
      err = nvs_get_i32(my_handle, key_power_mode, &readbuf[2]);
      switch (err)
      {
      case ESP_OK:
        printf("Done\n");
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        printf("The value is not initialized yet!\n");
        break;
      default:
        printf("Error (%s) reading!\n", esp_err_to_name(err));
      }
      test.power_mode = readbuf[2];
      // printf("test power_mode:%d\n", test.power_mode);
      printf("Reading year from NVS ... ");
      err = nvs_get_i32(my_handle, key_year, &readbuf[3]);
      switch (err)
      {
      case ESP_OK:
        printf("Done\n");
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        printf("The value is not initialized yet!\n");
        break;
      default:
        printf("Error (%s) reading!\n", esp_err_to_name(err));
      }
      test.year = readbuf[3];
      // printf("test year:%d\n", test.year);
      printf("Reading month from NVS ... ");
      err = nvs_get_i32(my_handle, key_month, &readbuf[4]);
      switch (err)
      {
      case ESP_OK:
        printf("Done\n");
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        printf("The value is not initialized yet!\n");
        break;
      default:
        printf("Error (%s) reading!\n", esp_err_to_name(err));
      }
      test.month = readbuf[4];
      // printf("test month:%d\n", test.month);
      printf("Reading day from NVS ... ");
      err = nvs_get_i32(my_handle, key_day, &readbuf[5]);
      switch (err)
      {
      case ESP_OK:
        printf("Done\n");
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        printf("The value is not initialized yet!\n");
        break;
      default:
        printf("Error (%s) reading!\n", esp_err_to_name(err));
      }
      test.day = readbuf[5];
      // printf("test day:%d\n", test.day);
    }
    // Close
    nvs_close(my_handle);
  }
  printf("readbuf:%d,%d,%d,%d,%d,%d\n", readbuf[0], readbuf[1], readbuf[2], readbuf[3], readbuf[4], readbuf[5]);
}

void system_write_to_flash()
{
  writebuf[0] = settings.brightness;
  writebuf[1] = settings.vibration_en;
  writebuf[2] = settings.power_mode;
  writebuf[3] = settings.year;
  writebuf[4] = settings.month;
  writebuf[5] = settings.day;
  esp_err_t err;
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  // Open
  printf("\n");
  printf("Opening Non-Volatile Storage (NVS) handle... ");
  nvs_handle_t my_handle;
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    printf("Done\n");
    // Write//Put into individual task(do it after settings update or ntp update or time update)
    {
      printf("Updating brightness in NVS ... ");
      err = nvs_set_i32(my_handle, key_brightness, writebuf[0]);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      // Commit written value.
      // After setting any values, nvs_commit() must be called to ensure changes are written
      // to flash storage. Implementations may write to storage at other times,
      // but this is not guaranteed.
      printf("Committing updates in NVS ... ");
      err = nvs_commit(my_handle);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      // do the same for other keys
      printf("Updating vibration_en in NVS ... ");
      err = nvs_set_i32(my_handle, key_vibration_en, writebuf[1]);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      printf("Committing updates in NVS ... ");
      err = nvs_commit(my_handle);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      printf("Updating power_mode in NVS ... ");
      err = nvs_set_i32(my_handle, key_power_mode, writebuf[2]);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      printf("Committing updates in NVS ... ");
      err = nvs_commit(my_handle);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      printf("Updating year in NVS ... ");
      err = nvs_set_i32(my_handle, key_year, writebuf[3]);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      printf("Committing updates in NVS ... ");
      err = nvs_commit(my_handle);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      printf("Updating month in NVS ... ");
      err = nvs_set_i32(my_handle, key_month, writebuf[4]);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      printf("Committing updates in NVS ... ");
      err = nvs_commit(my_handle);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      printf("Updating day in NVS ... ");
      err = nvs_set_i32(my_handle, key_day, writebuf[5]);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      printf("Committing updates in NVS ... ");
      err = nvs_commit(my_handle);
    }
  }
}
void system_init()
{
  // ui
  xTaskCreate(guiTask, "gui", 4096 * 16, NULL, 8, NULL);
  xTaskCreate(backlight_control_task, "backlight_control_task", 4096, NULL, 8, NULL);
  // imu
  i2c_mpu9250_init(&cal);
  xTaskCreate(imu_task, "imu_task", 4096, NULL, 8, NULL);

  // scan
  xTaskCreate(touch_scan, "touch_scan", 4096, NULL, 8, NULL);
  xTaskCreate(imu_scan, "imu_scan", 4096, NULL, 8, NULL);
  xTaskCreate(key_scan, "key_scan", 4096, NULL, 8, NULL);
  //  health
  ESP_ERROR_CHECK(max30102_init(&max30102, 0,
                                MAX30102_DEFAULT_OPERATING_MODE,
                                MAX30102_DEFAULT_SAMPLING_RATE,
                                MAX30102_DEFAULT_LED_PULSE_WIDTH,
                                MAX30102_DEFAULT_IR_LED_CURRENT,
                                MAX30102_DEFAULT_START_RED_LED_CURRENT,
                                MAX30102_DEFAULT_MEAN_FILTER_SIZE,
                                MAX30102_DEFAULT_PULSE_BPM_SAMPLE_SIZE,
                                MAX30102_DEFAULT_ADC_RANGE,
                                MAX30102_DEFAULT_SAMPLE_AVERAGING,
                                MAX30102_DEFAULT_ROLL_OVER,
                                MAX30102_DEFAULT_ALMOST_FULL,
                                false));
  max30102_en(0);
  //  lux sensor
  BH1750_Init();
  xTaskCreate(sys_sleep_timer, "sys_sleep_timer", 4096, NULL, 8, NULL);
}
void app_main(void)
{
  esp_err_t err;
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  health.bpm = 0;
  health.spo2 = 0;
  health.statereport = 0;

  user_time.hour = 0;
  user_time.minute = 0;
  user_time.second = 0;

  // gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  TIM_queue = xQueueCreate(10, sizeof(int));
  Health_queue = xQueueCreate(10, sizeof(health));
  IMU_queue = xQueueCreate(10, sizeof(imu_t));
  sntp_queue = xQueueCreate(10, sizeof(int));
  ledc_timer_config(&backlight_timer);
  ledc_channel_config(&backlight);
  ledc_channel_config(&LED_RED);
  ledc_channel_config(&LED_GREEN);
  ledc_channel_config(&LED_BLUE);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1000);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 1000);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 1000);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);

  backlight_control_manual(0);
  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 1000000ULL);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr, &config, ESP_INTR_FLAG_IRAM, NULL);
  timer_start(TIMER_GROUP_0, TIMER_0);
  // BLE Server on start
  // ESP_LOGW("BLE", "//////Init//////");
  // ble_init();
  static char Info[512] = {0};

  // ulp config
  rtc_gpio_init(TOUCH_IRQ);
  rtc_gpio_set_direction(TOUCH_IRQ, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_dis(TOUCH_IRQ);
  rtc_gpio_pullup_en(TOUCH_IRQ);
  rtc_gpio_hold_en(TOUCH_IRQ);

  rtc_gpio_init(KEY_IRQ);
  rtc_gpio_set_direction(KEY_IRQ, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_dis(KEY_IRQ);
  rtc_gpio_pullup_dis(KEY_IRQ);
  rtc_gpio_hold_en(KEY_IRQ);

  rtc_gpio_init(IMU_IRQ);
  rtc_gpio_set_direction(IMU_IRQ, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_dis(IMU_IRQ);
  rtc_gpio_pullup_dis(IMU_IRQ);
  rtc_gpio_hold_en(IMU_IRQ);
  esp_sleep_enable_ext0_wakeup(TOUCH_IRQ, 0);

  i2c_master_init(I2C_MASTER_NUM, 40, 39);
  ESP_LOGW("I2C", "I2C init done");

  vTaskDelay(5 / portTICK_PERIOD_MS);
  cause = esp_sleep_get_wakeup_cause();
  xTaskCreate(sys_state_machine, "sys_state_machine", 1024 * 16, NULL, 8, NULL);

  // DRV2605_Init();
  ESP_LOGW("MMMMMMMMMMMMMMMMMMMMMMMMPU", "Read On the Go");

  BH1750_Init();

  vTaskDelay(500 / portTICK_PERIOD_MS);
  BQ25896_Init();
  BQ25896_OTG_en(0);
  xTaskCreate(BQ25896_Get_Status_Task, "BQ25896_Get_Status_Task", 4096 * 2, NULL, 8, NULL);
  xTaskCreate(Sys_Temp_Task, "Sys_Temp_Task", 4096, NULL, 8, NULL);

  while (1)
  {
    // BH1750 Test
    //  int lux = 0;
    //  BH1750_Read(&lux);
    //  printf("lux:%d\n", lux);
    printf("Sleep timer count: %lld\n", esp_timer_get_time() / 1000000 - sys_sleep_timer_countstart);
    struct timeval tv;
    struct tm timeinfo;
    gettimeofday(&tv, NULL);
    time_t now = tv.tv_sec;
    localtime_r(&now, &timeinfo);
    user_time.hour = timeinfo.tm_hour + 8;
    user_time.minute = timeinfo.tm_min;
    user_time.second = timeinfo.tm_sec;
    user_time.date.year = timeinfo.tm_year + 1900;
    user_time.date.month = timeinfo.tm_mon + 1;
    user_time.date.day = timeinfo.tm_mday;
    user_time.date.weekday = timeinfo.tm_wday;
    // printf("Current date/time: %04d-%02d-%02d %02d:%02d:%02d\n",
    //        timeinfo.tm_year + 1900,
    //        timeinfo.tm_mon + 1,
    //        timeinfo.tm_mday,
    //        timeinfo.tm_hour + 8,
    //        timeinfo.tm_min,
    //        timeinfo.tm_sec + 1);
    // printf("temperature:%.1f\n", imu.Temp);

    // vTaskGetRunTimeStats((char *)&Info);
    // printf("\r\n任务名       运行计数         使用率\r\n");
    // printf("\r\n%s\r\n", Info);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}