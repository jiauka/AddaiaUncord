/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

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
#include "console_uart.h"

#include "time.h"
#include "sys/time.h"

#define PROBE_TAG "Probe-Tec OptoBlue"
#define EXAMPLE_DEVICE_NAME "ESP_SPP_INITIATOR"
#define SPP_SERVER_NAME "SPP_SERVER"

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const bool esp_spp_enable_l2cap_ertm = true;

//static struct timeval time_new, time_old;
//static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;//AUTHENTICATE OR AUTHORIZE OR NONE
//static const esp_spp_role_t role_master = ESP_SPP_ROLE_MASTER;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

esp_bd_addr_t peer_bd_addr = {0};
static uint8_t peer_bdname_len;
static uint8_t peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
//static const char remote_device_name[] = "ESP_SPP_ACCEPTOR";
//static const char remote_device_name[] = "OptoWave 9999";

static const esp_bt_inq_mode_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY; //Can this be limited inguiry???
static const uint8_t inq_len = 10;
static const uint8_t inq_num_rsps = 0;
static const char optowave_baddr[] = "00:07:80:64:ef:51";

//static const char optowave_device_name[] = "94:b9:7e:da:78:32";

int min_tx_power_level, max_tx_power_level;


#define SPP_DATA_LEN 22
static uint8_t spp_data[SPP_DATA_LEN];
static uint8_t *s_p_data = NULL; /* data pointer of spp_data */

typedef enum {
    APP_GAP_STATE_IDLE = 0,
    APP_GAP_STATE_DEVICE_DISCOVERING,
    APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE,
    APP_GAP_STATE_SERVICE_DISCOVERING,
    APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE,
} app_gap_state_t;

typedef struct {
    bool dev_found;
    uint8_t bdname_len;
    uint8_t eir_len;
    uint8_t rssi;
    uint32_t cod;
    uint8_t eir[ESP_BT_GAP_EIR_DATA_LEN];
    uint8_t bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
    esp_bd_addr_t bda;
    app_gap_state_t state;
} app_gap_cb_t;

static app_gap_cb_t m_dev_info;

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
static char *uuid2str(esp_bt_uuid_t *uuid, char *str, size_t size)
{
    if (uuid == NULL || str == NULL) {
        return NULL;
    }

    if (uuid->len == 2 && size >= 5) {
        sprintf(str, "%04x", uuid->uuid.uuid16);
    } else if (uuid->len == 4 && size >= 9) {
        sprintf(str, "%08"PRIx32, uuid->uuid.uuid32);
    } else if (uuid->len == 16 && size >= 37) {
        uint8_t *p = uuid->uuid.uuid128;
        sprintf(str, "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                p[15], p[14], p[13], p[12], p[11], p[10], p[9], p[8],
                p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0]);
    } else {
        return NULL;
    }

    return str;
}

static bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

static void find_probe_tec(esp_bt_gap_cb_param_t *param)
{
    char bda_str[18];
    uint32_t cod = 0;
    int32_t rssi = -129; /* invalid value */
    uint8_t *bdname = NULL;
    uint8_t bdname_len = 0;
    uint8_t *eir = NULL;
    uint8_t eir_len = 0;
    esp_bt_gap_dev_prop_t *p;

    ESP_LOGI(PROBE_TAG, "Device found: %s", bda2str(param->disc_res.bda, bda_str, 18));
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            ESP_LOGI(PROBE_TAG, "--Class of Device: 0x%"PRIx32, cod);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)(p->val);
            ESP_LOGI(PROBE_TAG, "--RSSI: %"PRId32, rssi);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
            bdname_len = (p->len > ESP_BT_GAP_MAX_BDNAME_LEN) ? ESP_BT_GAP_MAX_BDNAME_LEN :
                        (uint8_t)p->len;
            bdname = (uint8_t *)(p->val);
            ESP_LOGI(PROBE_TAG, "Device name: %s", bdname);
            break;
        case ESP_BT_GAP_DEV_PROP_EIR: {
            eir_len = p->len;
            eir = (uint8_t *)(p->val);
            break;
            }
        default:
           break;
        }
    }

    app_gap_cb_t *p_dev = &m_dev_info;
    if (p_dev->dev_found) {
        return;
    }

   // if (!esp_bt_gap_is_valid_cod(cod) ||
   //(!(esp_bt_gap_get_cod_major_dev(cod) == ESP_BT_COD_MAJOR_DEV_PHONE) &&
   //          !(esp_bt_gap_get_cod_major_dev(cod) == ESP_BT_COD_MAJOR_DEV_AV))) {
   //     return;
   // }

   switch (esp_bt_gap_get_cod_major_dev(cod)){

        case ESP_BT_COD_MAJOR_DEV_MISC:
            ESP_LOGI(PROBE_TAG, "Device is MISC ...");
        break;

        case ESP_BT_COD_MAJOR_DEV_COMPUTER:
            ESP_LOGI(PROBE_TAG, "Device is Desktop ...");
        break;

        case ESP_BT_COD_MAJOR_DEV_PHONE:
            ESP_LOGI(PROBE_TAG, "Device is a Phone ...");
        break;

        case ESP_BT_COD_MAJOR_DEV_LAN_NAP:
            ESP_LOGI(PROBE_TAG, "Device is a LAN NAP ...");
        break;

        case ESP_BT_COD_MAJOR_DEV_AV:
            ESP_LOGI(PROBE_TAG, "Device is an A/V ...");
        break;

        case ESP_BT_COD_MAJOR_DEV_PERIPHERAL:
            ESP_LOGI(PROBE_TAG, "Device is a peripheral ...");
        break;

        case ESP_BT_COD_MAJOR_DEV_IMAGING:
            ESP_LOGI(PROBE_TAG, "Device is a Imaging Device ...");
        break;

        case ESP_BT_COD_MAJOR_DEV_WEARABLE:
            ESP_LOGI(PROBE_TAG, "Device is a Wearable ...");
        break;

        case ESP_BT_COD_MAJOR_DEV_TOY:
            ESP_LOGI(PROBE_TAG, "Device is a Toy ...");
        break;

        case ESP_BT_COD_MAJOR_DEV_HEALTH:
            ESP_LOGI(PROBE_TAG, "Device is Health Deivce ...");
        break;

        case ESP_BT_COD_MAJOR_DEV_UNCATEGORIZED:
            ESP_LOGI(PROBE_TAG, "Device is..we dont know...");
        break;

        default:
           break;
   }

    memcpy(p_dev->bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
    p_dev->dev_found = true;

    memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);  //SAVE BDA TO PEER ADDRESS FOR SPP CONNECT CALL

    p_dev->cod = cod;
    p_dev->rssi = rssi;
    if (bdname_len > 0) {
        memcpy(p_dev->bdname, bdname, bdname_len);
        p_dev->bdname[bdname_len] = '\0';
        p_dev->bdname_len = bdname_len;
    }
    if (eir_len > 0) {
        memcpy(p_dev->eir, eir, eir_len);
        p_dev->eir_len = eir_len;
    }

    if (p_dev->bdname_len == 0) {
        get_name_from_eir(p_dev->eir, p_dev->bdname, &p_dev->bdname_len);
    }

    ESP_LOGI(PROBE_TAG, "Found a target device, address %s, name %s", bda_str, p_dev->bdname);
    p_dev->state = APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE;
   
    //******  REMOVE THESE STATEMENTS TO FORCE DISCOVERY UNTIL TIMEOUT  *******
    ESP_LOGI(PROBE_TAG, "Cancel device discovery ...");
    esp_bt_gap_cancel_discovery();
    vTaskDelay(pdMS_TO_TICKS(100)); // SLOW THINGS DOWN FOR NOW
    
}

//SPP discovery will trigger this callback and events....
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    uint8_t i = 0;
    char bda_str[18] = {0};
    esp_err_t ret = ESP_OK;

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(PROBE_TAG, "ESP_SPP_INIT_EVT");
            esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
            //esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
            if (esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME) != ESP_OK) {
                ESP_LOGE(PROBE_TAG, "%s esp spp start server failed: %s", __func__, esp_err_to_name(ret));
            return;
            }
        } else {
            ESP_LOGE(PROBE_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        if (param->disc_comp.status == ESP_SPP_SUCCESS){
            ESP_LOGI(PROBE_TAG, "ESP_SPP_DISCOVERY_COMP_EVT scn_num:%d", param->disc_comp.scn_num);
            for (i = 0; i < param->disc_comp.scn_num; i++) {
                ESP_LOGI(PROBE_TAG, "-- [%d] scn:%d service_name:%s", i, param->disc_comp.scn[i],
                         param->disc_comp.service_name[i]);
            }
            /* We only connect to the first found server on the remote SPP acceptor here */
            esp_spp_connect(sec_mask, role_slave, param->disc_comp.scn[0], peer_bd_addr);
            //ESP_LOGE(PROBE_TAG, "SLOWING DOWN TO CONFIRM BLUE LIGHT CONNECT");
            //vTaskDelay(pdMS_TO_TICKS(2000));
        } else {
            ESP_LOGE(PROBE_TAG, "ESP_SPP_DISCOVERY_COMP_EVT status=%d", param->disc_comp.status);
        }
        break;

    case ESP_SPP_OPEN_EVT:
            if (param->open.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(PROBE_TAG, "ESP_SPP_OPEN_EVT handle:%"PRIu32" rem_bda:[%s]", param->open.handle,
                     bda2str(param->open.rem_bda, bda_str, sizeof(bda_str)));
            /* Start to write the first data packet */
            char * c = "PAIR 00:07:80:64:ef:51";
            uint8_t * spp_data = (uint8_t *)c;
            ESP_LOGI(PROBE_TAG, "ESP_SPP_OPEN_EVT - spp_data :%s", spp_data);
            esp_spp_write(param->open.handle, SPP_DATA_LEN, spp_data);
            s_p_data = spp_data;
            //gettimeofday(&time_old, NULL);
            } else {
                ESP_LOGE(PROBE_TAG, "ESP_SPP_OPEN_EVT status:%d", param->open.status);
            }
        break;

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(PROBE_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%"PRIu32" close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(PROBE_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        if (param->cl_init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(PROBE_TAG, "ESP_SPP_CL_INIT_EVT handle:%"PRIu32" sec_id:%d", param->cl_init.handle, param->cl_init.sec_id);
        } else {
            ESP_LOGE(PROBE_TAG, "ESP_SPP_CL_INIT_EVT status:%d", param->cl_init.status);
        }
        break;

    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(PROBE_TAG, "ESP_SPP_DATA_IND_EVT");
        break;

    case ESP_SPP_WRITE_EVT:
        if (param->write.status == ESP_SPP_SUCCESS) {
            if (s_p_data + param->write.len == spp_data + SPP_DATA_LEN) {
                /* Means the previous data packet be sent completely, send a new data packet */
                ESP_LOGI(PROBE_TAG, "COMPLETE SEND.....");
                s_p_data = spp_data;
            } else {
                /*
                 * Means the previous data packet only be sent partially due to the lower layer congestion, resend the
                 * remainning data.
                 */
                s_p_data += param->write.len;
                ESP_LOGI(PROBE_TAG, "INCOMPLETE SEND.....");
            }
            if (param->write.len < 128) {
                esp_log_buffer_hex(".-.-.-.", spp_data, param->write.len);
                /* Delay a little to avoid the task watch dog */
                vTaskDelay(pdMS_TO_TICKS(1000)); // WAS 10
            }
        }
        break;

    case ESP_SPP_CONG_EVT:
        ESP_LOGI(PROBE_TAG, "ESP_SPP_CONG_EVT cong:%d", param->cong.cong);
        if (param->cong.cong == 0) {
            /* Send the privous (partial) data packet or the next data packet. */
            //esp_spp_write(param->write.handle, spp_data + SPP_DATA_LEN - s_p_data, s_p_data);
        }
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(PROBE_TAG, "ESP_SPP_SRV_OPEN_EVT");
        break;

    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(PROBE_TAG, "ESP_SPP_UNINIT_EVT");
        break;

    default:
        break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    //esp_bt_gap_dev_prop_t *p;
    //uint32_t probe_tec_cod = 0x080500;
    app_gap_cb_t *p_dev = &m_dev_info;
    char bda_str[18];
    char uuid_str[37];
    char str[18] = {0};
    uint8_t *bda = NULL;
    //int number_of_properties;

    switch(event){
    case ESP_BT_GAP_DISC_RES_EVT:
        ESP_LOGI(PROBE_TAG, "ESP_BT_GAP_DISC_RES_EVT");
        esp_log_buffer_hex(PROBE_TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
        /* Find the target peer device name in the EIR data */
        for (int i = 0; i < param->disc_res.num_prop; i++){
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR
                && get_name_from_eir(param->disc_res.prop[i].val, peer_bdname, &peer_bdname_len)){
                esp_log_buffer_char(PROBE_TAG, peer_bdname, peer_bdname_len);
                /*if (strlen(remote_device_name) == peer_bdname_len
                    && strncmp(peer_bdname, remote_device_name, peer_bdname_len) == 0) {*/
                bda = (uint8_t *)param->disc_res.bda;
                sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", bda[0],bda[1],bda[2],bda[3],bda[4],bda[5]);
                if (strcmp(str, optowave_baddr) == 0){
                    memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
                    /*Have found the target peer device, cancel the previous GAP discover procedure. And go on
                     * dsicovering the SPP service on the peer device*/
                    esp_bt_gap_cancel_discovery();
                    esp_spp_start_discovery(peer_bd_addr);
                    bda = (uint8_t *)param->disc_res.bda;
                    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", bda[0],bda[1],bda[2],bda[3],bda[4],bda[5]);
                    if (strcmp(str, optowave_baddr) == 0){
                        ESP_LOGI(PROBE_TAG, "HELLO MOTO!!!");
                    }
                    //number_of_properties = (char *)param->disc_res.num_prop;
                    //ESP_LOGI(PROBE_TAG, "Number of properties: %d", number_of_properties);
                }
            }
        }
        break;

    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            ESP_LOGI(PROBE_TAG, "Device discovery stopped.");
            if ( (p_dev->state == APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE ||
                    p_dev->state == APP_GAP_STATE_DEVICE_DISCOVERING)
                    && p_dev->dev_found) {
                p_dev->state = APP_GAP_STATE_SERVICE_DISCOVERING;
                ESP_LOGI(PROBE_TAG, "Discover services ...");
                esp_bt_gap_get_remote_services(p_dev->bda);
            } else {
                ESP_LOGI(PROBE_TAG, "Probe-Tec not found....Starting UART in 5 seconds");
                vTaskDelay(pdMS_TO_TICKS(5000));
                //TODO:  START UART
            }
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            ESP_LOGI(PROBE_TAG, "Discovery started.");
        }
        break;
    }
    case ESP_BT_GAP_RMT_SRVCS_EVT:{
        ESP_LOGI(PROBE_TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
        if (memcmp(param->rmt_srvcs.bda, p_dev->bda, ESP_BD_ADDR_LEN) == 0 &&
                p_dev->state == APP_GAP_STATE_SERVICE_DISCOVERING) {
            p_dev->state = APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE;
            if (param->rmt_srvcs.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(PROBE_TAG, "Services for device %s found",  bda2str(p_dev->bda, bda_str, 18));
                for (int i = 0; i < param->rmt_srvcs.num_uuids; i++) {
                    esp_bt_uuid_t *u = param->rmt_srvcs.uuid_list + i;
                    ESP_LOGI(PROBE_TAG, "--%s", uuid2str(u, uuid_str, 37));
                }
            } else {
                ESP_LOGI(PROBE_TAG, "Services for device %s not found",  bda2str(p_dev->bda, bda_str, 18));
            }
        esp_spp_start_discovery(p_dev->bda);
        vTaskDelay(pdMS_TO_TICKS(100)); // SLOW THINGS DOWN FOR NOW
        }
        break;
    }
        break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        ESP_LOGI(PROBE_TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        ESP_LOGI(PROBE_TAG, "ESP_BT_GAP_AUTH_CMPL_EVT");
        break;
    case ESP_BT_GAP_PIN_REQ_EVT:
        ESP_LOGI(PROBE_TAG, "ESP_BT_GAP_PIN_REQ_EVT");
        ESP_LOGI(PROBE_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(PROBE_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(PROBE_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        break;
        }
        break;

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(PROBE_TAG, "ESP_BT_GAP_MODE_CHG_EVT");
        break;

    default:
        break;
    }
}

void app_main(void)
{
    esp_err_t ret = ESP_OK;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(PROBE_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(PROBE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(PROBE_TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(PROBE_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(PROBE_TAG, "%s gap register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IN;
    ESP_LOGE(PROBE_TAG, "<<<<<< SSP IS ENABLED >>>>>");
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
    if (iocap == ESP_BT_IO_CAP_IN || iocap == ESP_BT_IO_CAP_IO) {
        //console_uart_init();
        //vTaskDelay(pdMS_TO_TICKS(500));
    }
#endif

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(PROBE_TAG, "%s spp register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    //if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
    //        ESP_LOGE(PROBE_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
    //        return;
    //    }

// REMOVE THE STRUCTURE AND FUNCTION BELOW AND UNCOMMENT THE esp_spp_init FUNCTION ABOVE TO PAIR AS "JUST WORKS"//
// THIS SETS UP SSP TO REQUIRE A PIN
    esp_spp_cfg_t bt_spp_cfg = {                                                                                //
        .mode = esp_spp_mode,                                                                                   //
        .enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,                                                         //
        .tx_buffer_size = 0, //Only used for ESP_SPP_MODE_VFS mode                                              //
    };                                                                                                          //    
    if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK) {                                                 //    
        ESP_LOGE(PROBE_TAG, "%s spp init failed: %s", __func__, esp_err_to_name(ret));                          //
        return;                                                                                                 //    
    }                                                                                                           //    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////      


    min_tx_power_level = ESP_PWR_LVL_N0; //TEMP SETTING UNTIL UART CONTROL IS BUILT
    max_tx_power_level = ESP_PWR_LVL_P3;
    if ((ret = esp_bredr_tx_power_set(max_tx_power_level, max_tx_power_level)) != ESP_OK) {
        ESP_LOGE(PROBE_TAG, "%s set RSSI power level failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE; //ESP_BT_PIN_TYPE_VARIABLE
    esp_bt_pin_code_t pin_code;
    //pin_code[0] = "1";
    //pin_code[1] = "2";
    //pin_code[2] = "3";
    //pin_code[3] = "4";
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    //ESP_LOGI(PROBE_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
}
