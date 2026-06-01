

/****************************************************************************
*
* This file is for bt_spp_vfs_acceptor demo. It can create servers, wait for connected and receive data.
* run bt_spp_vfs_acceptor demo, the bt_spp_vfs_initiator demo will automatically connect the bt_spp_vfs_acceptor demo,
* then receive data.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp_mac.h"  

#include "driver/uart.h"
#include "driver/gpio.h"
#include "spp_task.h"


#include "time.h"
#include "sys/time.h"

#include "esp_vfs.h"
#include "sys/unistd.h"
#define IS_LM
#define SPP_TAG "SPP_ANTENNA"

#define BT_CONNECTED_GPIO  12
#define BT_UART_TX_GPIO    19
#define BT_UART_RX_GPIO    22
#define BT_UART_RTS_GPIO   UART_PIN_NO_CHANGE

#define BT_LED_CONNECTED    1
#define BT_LED_DISCONNECTED 0

#define BT_UART_BITRATE     9600

#define BT_UART_RX_BUF_SZ (1024 * 2)
#define BT_UART_TX_BUF_SZ (1024 * 2)

#ifdef IS_LM
  #define SPP_SERVER_NAME "SerialADT44"

  static const char local_device_name[] = "SerialADT44";
  static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE ; //ESP_SPP_SEC_AUTHENTICATE;  ESP_SPP_SEC_NONE; //AUTHENTICATE;
#else
  #define SPP_SERVER_NAME "Anytec"

  static const char local_device_name[] = "Anytec";
  static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE ; //ESP_SPP_SEC_AUTHENTICATE;  ESP_SPP_SEC_NONE; //AUTHENTICATE;
#endif

static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE; //ESP_SPP_ROLE_MASTER ESP_SPP_ROLE_SLAVE

#define SPP_DATA_LEN 100
static uint8_t spp_buff[SPP_DATA_LEN];

#define BT_UART UART_NUM_1

static void configure_led(void)
{
    ESP_LOGI(SPP_TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BT_CONNECTED_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BT_CONNECTED_GPIO, GPIO_MODE_OUTPUT);
}
static int uart_to_bt(int bt_fd, TickType_t ticks_to_wait)
{
    int size = uart_read_bytes(BT_UART, spp_buff, SPP_DATA_LEN, ticks_to_wait);
    if (size <= 0) {
        return 0;
    }
    ESP_LOGD(SPP_TAG, "UART -> %d bytes", size);
    uint8_t* ptr = spp_buff;
    int remain = size;
    while (remain > 0)
    {
        int res = write(bt_fd, ptr, remain);
        if (res < 0) {
            return -1;
        }
        if (res == 0) {
            vTaskDelay(1);
            continue;
        }
        ESP_LOGD(SPP_TAG, "BT <- %d bytes", res);
        remain -= res;
        ptr  += res;
    }
    return size;
}

static void dump(uint8_t * b, size_t size)
{
  for(int i=0; i< size; i++) {
    printf("%02x ",b[i]);
  }
  printf("\n\r");
}

static char *bda2str(uint8_t * bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}



static void spp_read_handle(void * param)
{
    int fd = (int)param;

    ESP_LOGE(SPP_TAG, "BT connected, %u bytes free", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    //gpio_set_level(BT_CONNECTED_GPIO, BT_LED_CONNECTED);
    uart_flush(BT_UART);

    TickType_t ticks_to_wait = 1;

    for (;;)
    {
        // Send available data from UART to BT first
        for (;;) {
            int tx_size = uart_to_bt(fd, ticks_to_wait);
            if (tx_size < 0)
                goto disconnected;
            if (!tx_size)
                break;
            ticks_to_wait = 0;
        }
        // Try receive data from BT
        int const size = read(fd, spp_buff, SPP_DATA_LEN);
        if (size < 0) {
            goto disconnected;
        }
        if (size > 0) {
          dump(spp_buff,size);
            ESP_LOGE(SPP_TAG, "BT -> %.*s %d bytes -> UART", size,(const char *)spp_buff,size);
            uart_write_bytes(BT_UART, (const char *)spp_buff, size);
            ticks_to_wait = 0;
        } else
            ticks_to_wait = 1;
    }

disconnected:
    ESP_LOGI(SPP_TAG, "BT disconnected");
    //gpio_set_level(BT_CONNECTED_GPIO, BT_LED_DISCONNECTED);
    spp_wr_task_shut_down();
}

static void esp_spp_cb(uint16_t e, void *p)
{
    esp_spp_cb_event_t event = e;
    esp_spp_cb_param_t *param = p;
    char bda_str[18] = {0};

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            /* Enable SPP VFS mode */
            esp_spp_vfs_register();
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%"PRIu32" close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%"PRIu32" sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                     param->start.scn);
            esp_bt_gap_set_device_name(local_device_name);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%"PRIu32", rem_bda:[%s]", param->srv_open.status,
                 param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
        if (param->srv_open.status == ESP_SPP_SUCCESS) {
            spp_wr_task_start_up(spp_read_handle, param->srv_open.fd);
        }
        break;
    case ESP_SPP_VFS_REGISTER_EVT:
        if (param->vfs_register.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_VFS_REGISTER_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_VFS_REGISTER_EVT status:%d", param->vfs_register.status);
        }
        break;
    default:
        break;
    }
}

static void esp_spp_stack_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    /* To avoid stucking Bluetooth stack, we dispatch the SPP callback event to the other lower priority task */
    spp_task_work_dispatch(esp_spp_cb, event, param, sizeof(esp_spp_cb_param_t), NULL);
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            ESP_LOG_BUFFER_HEX(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
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
#ifdef IS_LM

            ESP_LOGI(SPP_TAG, "Input pin code: 0000");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '0';
            pin_code[1] = '0';
            pin_code[2] = '0';
            pin_code[3] = '0';
            esp_bt_gap_pin_reply(param->pin_req.bda, false, 0, pin_code);
#else
            ESP_LOGI(SPP_TAG, "Input pin code: 0001");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '0';
            pin_code[1] = '0';
            pin_code[2] = '0';
            pin_code[3] = '1';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
#endif
        }
        break;
    }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%"PRIu32, param->key_notif.passkey);
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
    char bda_str[18] = {0};
#ifdef IS_LM
    uint8_t new_mac[8] = {0x34, 0xc9, 0xf0, 0x8a, 0x5e, 0x50}; // A LM device SLAVE (ANTENA), we are SLAVE BT mac is +2 //34C9-F0-8A5E52

#else
    //uint8_t new_mac[8] = {0x00, 0x0b, 0xce, 0xc4, 0xf1, 0xf1}; // B device MASTER, we are SLAVE (MANDO) BT mac is +2
    uint8_t new_mac[8] = {0x00, 0x0b, 0xce, 0x04, 0xf1, 0xcb}; // B device SLAVE (ANTENA), we are SLAVE BT mac is +2 //00 0B CE 04 F1 CD
#endif
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    //configure_led();
    
    /* Configure UART */
    uart_config_t uart_config = {
        .baud_rate = BT_UART_BITRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE ,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = UART_FIFO_LEN - 4
    };

    ESP_ERROR_CHECK(uart_param_config(BT_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BT_UART, BT_UART_TX_GPIO, BT_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(BT_UART, BT_UART_RX_BUF_SZ, BT_UART_TX_BUF_SZ, 0, NULL, 0));

    esp_base_mac_addr_set(new_mac);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed", __func__);
        return;
    }

    if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed", __func__);
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
#if (CONFIG_EXAMPLE_SSP_ENABLED == false)
    bluedroid_cfg.ssp_en = false;
#endif
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if (esp_bluedroid_enable() != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed", __func__);
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if (esp_spp_register_callback(esp_spp_stack_cb) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed", __func__);
        return;
    }

    spp_task_task_start_up();

    esp_spp_cfg_t bt_spp_cfg = BT_SPP_DEFAULT_CONFIG();
    if (esp_spp_enhanced_init(&bt_spp_cfg) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed", __func__);
        return;
    }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
#if 1//ndef IS_LM

   esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
   esp_bt_pin_code_t pin_code;
   esp_bt_gap_set_pin(pin_type, 0, pin_code);
#else
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;//ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    memset(pin_code, 0,sizeof(pin_code));
    pin_code[0]=0;
    pin_code[1]=0;
    pin_code[2]=0;
    pin_code[3]=0;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
#endif
    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
}
