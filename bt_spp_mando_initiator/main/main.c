/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
 *
 * This file is for bt_spp_vfs_initiator demo. It can discovery servers, connect one device and send data.
 * run bt_spp_vfs_initiator demo, the bt_spp_vfs_initiator demo will automatically connect the bt_spp_vfs_acceptor demo,
 * then send data.
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

#include "spp_task.h"

#include "time.h"
#include "sys/time.h"

#include "esp_vfs.h"
#include "sys/unistd.h"

#define IS_LM

//#define CONNECT_AT_START

#define TAG "SPP_MANDO"
#ifdef IS_LM
#define DEVICE_NAME "SerialADT44"

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE ; //ESP_SPP_SEC_AUTHENTICATE;  ESP_SPP_SEC_NONE; //AUTHENTICATE;
static const esp_spp_role_t role_master = ESP_SPP_ROLE_MASTER;
static const char antenna_baddr[] = "34:c9:f0:8a:5e:52";
static esp_bd_addr_t antenna_uncord_addr={0x34,0xc9,0xf0,0x8a,0x5e,0x52}; //34C9-F0-8A5E52


#else
#define DEVICE_NAME "Anytec"

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE ; //ESP_SPP_SEC_AUTHENTICATE;  ESP_SPP_SEC_NONE; //AUTHENTICATE;
static const esp_spp_role_t role_master = ESP_SPP_ROLE_MASTER;
static const char antenna_baddr[] = "00:0b:ce:04:f1:cd";
static esp_bd_addr_t antenna_uncord_addr={0x00,0x0b,0xce,0x04,0xf1,0xcd};
#endif

#define BT_UART_TX_GPIO 19
#define BT_UART_RX_GPIO 22
#define BT_UART_RTS_GPIO UART_PIN_NO_CHANGE

#define BT_LED_CONNECTED 1
#define BT_LED_DISCONNECTED 0

#define BT_UART_BITRATE 9600

#define BT_UART_RX_BUF_SZ (1024 * 2)
#define BT_UART_TX_BUF_SZ (1024 * 2)

#define SPP_DATA_LEN 100
static uint8_t spp_buff[SPP_DATA_LEN];

#define BT_UART UART_NUM_1

static esp_bd_addr_t peer_bd_addr;
//static uint8_t peer_bdname_len;
//static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
//static const char remote_device_name[] = "Anytec";

static const esp_bt_inq_mode_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY;
static const uint8_t inq_len = 30;
static const uint8_t inq_num_rsps = 0;

static bool found = false;
void set_classic_bt_tx_power() {
    esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9);
}

static int uart_to_bt(int bt_fd, TickType_t ticks_to_wait)
{
  int size = uart_read_bytes(BT_UART, spp_buff, SPP_DATA_LEN, ticks_to_wait);
  if (size <= 0) {
    return 0;
  }
  ESP_LOGE(TAG, "UART -> %d bytes", size);
  uint8_t* ptr = spp_buff;
  int remain = size;
  while (remain > 0) {
    int res = write(bt_fd, ptr, remain);
    if (res < 0) {
      return 0; //-1;
    }
    if (res == 0) {
      vTaskDelay(1);
      continue;
    }
    ESP_LOGD(TAG, "BT <- %d bytes", res);
    remain -= res;
    ptr += res;
  }
  return size;
}

static void dump(uint8_t* b, size_t size)
{
  for (int i = 0; i < size; i++) {
    printf("%02x ", b[i]);
  }
  printf("\n\r");
}

static char* bda2str(uint8_t* bda, char* str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }

  uint8_t* p = bda;
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", p[0], p[1], p[2], p[3], p[4], p[5]);
  return str;
}

static void spp_read_handle(void* param)
{
  int fd = (int)param;

  ESP_LOGE(TAG, "BT connected, %u bytes free", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
  // gpio_set_level(BT_CONNECTED_GPIO, BT_LED_CONNECTED);
  uart_flush(BT_UART);

  TickType_t ticks_to_wait = 1;

  for (;;) {
    // Send available data from UART to BT first
    for (;;) {
      int tx_size = uart_to_bt(fd, ticks_to_wait);
      if (tx_size < 0) {
        ESP_LOGI(TAG, "BT disconnected %d", __LINE__);

        goto disconnected;
      }
      if (!tx_size)
        break;
      ticks_to_wait = 0;
    }
    // Try receive data from BT
    int const size = read(fd, spp_buff, SPP_DATA_LEN);
    if (size < 0) {
      ESP_LOGI(TAG, "BT disconnected %d", __LINE__);

      goto disconnected;
    }
    if (size > 0) {
      dump(spp_buff, size);
      ESP_LOGE(TAG, "BT -> %.*s %d bytes -> UART", size, (const char*)spp_buff, size);
      uart_write_bytes(BT_UART, (const char*)spp_buff, size);
      ticks_to_wait = 0;
    } else
      ticks_to_wait = 1;
  }

disconnected:
  ESP_LOGI(TAG, "BT disconnected");
  // gpio_set_level(BT_CONNECTED_GPIO, BT_LED_DISCONNECTED);
  found = false;
  esp_bt_gap_set_device_name(DEVICE_NAME);
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
  esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
  
  spp_wr_task_shut_down();
}

static void esp_spp_cb(uint16_t e, void* p)
{
  esp_spp_cb_event_t event = e;
  esp_spp_cb_param_t* param = p;
  uint8_t i = 0;
  char bda_str[18] = { 0 };

  switch (event) {
    case ESP_SPP_INIT_EVT:
      if (param->init.status == ESP_SPP_SUCCESS) {
        ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
        /* Enable SPP VFS mode */
        esp_spp_vfs_register();
      } else {
        ESP_LOGE(TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
      }
      break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
      if (param->disc_comp.status == ESP_SPP_SUCCESS) {
        ESP_LOGI(TAG, "ESP_SPP_DISCOVERY_COMP_EVT scn_num:%d", param->disc_comp.scn_num);
        for (i = 0; i < param->disc_comp.scn_num; i++) {
          ESP_LOGI(TAG, "-- [%d] scn:%d service_name:%s", i, param->disc_comp.scn[i], param->disc_comp.service_name[i]);
        }
        /* We only connect to the first found server on the remote SPP acceptor here */
        esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], peer_bd_addr);
      } else {
        ESP_LOGE(TAG, "ESP_SPP_DISCOVERY_COMP_EVT status=%d", param->disc_comp.status);
      }
      break;
    case ESP_SPP_OPEN_EVT:
      if (param->open.status == ESP_SPP_SUCCESS) {
        ESP_LOGI(TAG, "ESP_SPP_OPEN_EVT handle:%" PRIu32 " fd:%d rem_bda:[%s]", param->open.handle, param->open.fd, bda2str(param->open.rem_bda, bda_str, sizeof(bda_str)));
        spp_wr_task_start_up(spp_read_handle, param->open.fd);
        // spp_wr_task_start_up(spp_write_handle, param->open.fd);
      } else {
        ESP_LOGE(TAG, "ESP_SPP_OPEN_EVT status:%d", param->open.status);
      }
      break;
    case ESP_SPP_CLOSE_EVT:
      ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%" PRIu32 " close_by_remote:%d", param->close.status, param->close.handle, param->close.async);
#ifdef CONNECT_AT_START          
#ifdef IS_LM
        if (esp_spp_connect(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER, 1, antenna_uncord_addr) != ESP_OK) {
          ESP_LOGE(TAG, "esp_spp_connect FAILED!!!!");
        }
#else
        if (esp_spp_connect(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_MASTER, 1, antenna_uncord_addr) != ESP_OK) {
          ESP_LOGE(TAG, "esp_spp_connect FAILED!!!!");
        }
#endif      

#endif      
      break;
    case ESP_SPP_START_EVT:
      ESP_LOGI(TAG, "ESP_SPP_START_EVT");
      break;
    case ESP_SPP_CL_INIT_EVT:
      if (param->cl_init.status == ESP_SPP_SUCCESS) {
        ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT handle:%" PRIu32 " sec_id:%d", param->cl_init.handle, param->cl_init.sec_id);
      } else {
        ESP_LOGE(TAG, "ESP_SPP_CL_INIT_EVT status:%d", param->cl_init.status);
      }
      break;
    case ESP_SPP_SRV_OPEN_EVT:
      ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT");
      break;
    case ESP_SPP_VFS_REGISTER_EVT:
      if (param->vfs_register.status == ESP_SPP_SUCCESS) {
        ESP_LOGI(TAG, "ESP_SPP_VFS_REGISTER_EVT");
        esp_bt_gap_set_device_name(DEVICE_NAME);
#ifdef CONNECT_AT_START          
//        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        //esp_spp_start_discovery(antenna_uncord_addr/*peer_bd_addr*/);
#ifdef IS_LM
        if (esp_spp_connect(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER, 1, antenna_uncord_addr) != ESP_OK) {
          ESP_LOGE(TAG, "esp_spp_connect FAILED!!!!");
        }
#else
        if (esp_spp_connect(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_MASTER, 1, antenna_uncord_addr) != ESP_OK) {
          ESP_LOGE(TAG, "esp_spp_connect FAILED!!!!");
        }
#endif      
#else
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
#endif
      } else {
        ESP_LOGE(TAG, "ESP_SPP_VFS_REGISTER_EVT status:%d", param->vfs_register.status);
      }
      break;
    default:
      break;
  }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* param)
{
  char bda_str[18] = { 0 };
  char str[18] = { 0 };
  uint8_t* bda = NULL;

  switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT:
      ESP_LOGI(TAG, "ESP_BT_GAP_DISC_RES_EVT");
      ESP_LOG_BUFFER_HEX(TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
      /* Find the target peer device name in the EIR data */
      for (int i = 0; i < param->disc_res.num_prop; i++) {
        ESP_LOGE(TAG, "remote address:[%s] %s", bda2str((uint8_t*)param->disc_res.bda, bda_str, sizeof(bda_str)), antenna_baddr);
        bda = (uint8_t*)param->disc_res.bda;
        sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        if (strcmp(str, antenna_baddr) == 0 && !found) {
          found = true;
          ESP_LOGE(TAG, "HELLO UNCORD!!!");
          memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
          /* Have found the target peer device, cancel the previous GAP discover procedure. And go on
           * dsicovering the SPP service on the peer device */
          esp_bt_gap_cancel_discovery();
#ifdef CONNECT_AT_START          
          esp_spp_start_discovery(antenna_uncord_addr/*peer_bd_addr*/);
#else
          esp_spp_start_discovery(peer_bd_addr);
#endif          
        }
      }
      break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
      ESP_LOGI(TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
      if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
        ESP_LOGI(TAG, "Device discovery stopped.");

      } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
        ESP_LOGI(TAG, "Discovery started.");
      }
      break;
    case ESP_BT_GAP_RMT_SRVCS_EVT:
      ESP_LOGI(TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
      break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
      ESP_LOGI(TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
      break;
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
      if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
        ESP_LOG_BUFFER_HEX(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
      } else {
        ESP_LOGE(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        found = false;
      }
      break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT: {
      ESP_LOGE(TAG, "**********************ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
      if (param->pin_req.min_16_digit) {
        ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
        esp_bt_pin_code_t pin_code = { 0 };
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
      } else {
#ifdef IS_LM
        ESP_LOGI(TAG, "Input pin code: 0000");
        esp_bt_pin_code_t pin_code;
        pin_code[0] = '0';
        pin_code[1] = '0';
        pin_code[2] = '0';
        pin_code[3] = '0';
        esp_bt_gap_pin_reply(param->pin_req.bda, false, 0, pin_code);
#else
        ESP_LOGI(TAG, "Input pin code: 0001");
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

    case ESP_BT_GAP_MODE_CHG_EVT:
      ESP_LOGI(TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
      break;

    default:
      break;
  }
}

static void esp_spp_stack_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t* param)
{
  /* To avoid stucking Bluetooth stack, we dispatch the SPP callback event to the other lower priority task */
  spp_task_work_dispatch(esp_spp_cb, event, param, sizeof(esp_spp_cb_param_t), NULL);
}

void app_main(void)
{
  esp_err_t ret = ESP_OK;
  char bda_str[18] = { 0 };
  #ifdef IS_LM
    uint8_t new_mac[8] = {0x34, 0xc9, 0xf0, 0x8a, 0x5c, 0xfb}; // A LM device SLAVE (ANTENA), we are MASTER BT mac is +2 //34C9-F0-8A5CFD

#else
  uint8_t new_mac[8] = { 0x00, 0x0b, 0xce, 0x04, 0xf1, 0xf1 }; // B device MASTER, we are SLAVE (MANDO) BT mac is +2
  // uint8_t new_mac[8] = {0x00, 0x0b, 0xce, 0x04, 0xf1, 0xcb}; // B device SLAVE (ANTENA), we are SLAVE BT mac is +2 //00 0B CE 04 F1 CD
#endif

  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  /* Configure UART */
  uart_config_t uart_config = { .baud_rate = BT_UART_BITRATE,
                                .data_bits = UART_DATA_8_BITS,
                                .parity = UART_PARITY_DISABLE,
                                .stop_bits = UART_STOP_BITS_1,
                                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                                .rx_flow_ctrl_thresh = UART_HW_FIFO_LEN(BT_UART) - 4 };

  ESP_ERROR_CHECK(uart_param_config(BT_UART, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(BT_UART, BT_UART_TX_GPIO, BT_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(BT_UART, BT_UART_RX_BUF_SZ, BT_UART_TX_BUF_SZ, 0, NULL, 0));

  esp_base_mac_addr_set(new_mac);

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
    ESP_LOGE(TAG, "%s initialize controller failed", __func__);
    return;
  }

  if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
    ESP_LOGE(TAG, "%s enable controller failed", __func__);
    return;
  }

  esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
  bluedroid_cfg.ssp_en = false;
  if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
    ESP_LOGE(TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
    return;
  }
  set_classic_bt_tx_power();
  if (esp_bluedroid_enable() != ESP_OK) {
    ESP_LOGE(TAG, "%s enable bluedroid failed", __func__);
    return;
  }

  if (esp_bt_gap_register_callback(esp_bt_gap_cb) != ESP_OK) {
    ESP_LOGE(TAG, "%s gap register failed", __func__);
    return;
  }

  if (esp_spp_register_callback(esp_spp_stack_cb) != ESP_OK) {
    ESP_LOGE(TAG, "%s spp register failed", __func__);
    return;
  }

  spp_task_task_start_up();

  esp_spp_cfg_t bt_spp_cfg = BT_SPP_DEFAULT_CONFIG();
  if (esp_spp_enhanced_init(&bt_spp_cfg) != ESP_OK) {
    ESP_LOGE(TAG, "%s spp init failed", __func__);
    return;
  }

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
    pin_code[3]=1;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
#endif
  ESP_LOGI(TAG, "Own address:[%s]", bda2str((uint8_t*)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
}
