/*
 * ESP32 + BME280 (SPI) -> WiFi -> MQTT
 *
 * - Connects to WiFi (configurable via menuconfig)
 * - Reads BME280 over SPI
 * - Publishes T/P/H to MQTT broker on configurable IP:port
 * - Subscribes to the same topics for loopback testing
 * - Publishes 2x per second (500 ms interval)
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "driver/spi_master.h"
#include "mqtt_client.h"

/* -------------------- Kconfig -------------------- */
#define WIFI_SSID           CONFIG_WIFI_SSID
#define WIFI_PASS           CONFIG_WIFI_PASSWORD
#define MQTT_BROKER_IP      CONFIG_MQTT_BROKER_IP
#define MQTT_BROKER_PORT    CONFIG_MQTT_BROKER_PORT
#define TOPIC_PREFIX        CONFIG_MQTT_TOPIC_PREFIX
#define BME280_SCK          CONFIG_BME280_SPI_SCK
#define BME280_MOSI         CONFIG_BME280_SPI_MOSI
#define BME280_MISO         CONFIG_BME280_SPI_MISO
#define BME280_CS           CONFIG_BME280_SPI_CS

/* -------------------- Topics -------------------- */
#define TOPIC_TEMP          TOPIC_PREFIX "/temp"
#define TOPIC_HUM           TOPIC_PREFIX "/hum"
#define TOPIC_PRES          TOPIC_PREFIX "/pres"
#define TOPIC_ENV           TOPIC_PREFIX "/env"

#define TAG                 "bme280_mqtt"

/* -------------------- BME280 -------------------- */
#define BME280_REG_CHIP_ID      0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_DATA         0xF7
#define BME280_REG_CALIB_00     0x88
#define BME280_REG_CALIB_26     0xE1
#define BME280_CHIP_ID          0x60

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_data_t;

static bme280_calib_data_t calib;
static int32_t t_fine;
static spi_device_handle_t bme280_spi = NULL;

/* --- BME280 SPI helpers --- */
static esp_err_t bme280_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {reg & 0x7F, val};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    return spi_device_transmit(bme280_spi, &t);
}

static esp_err_t bme280_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    /* Full-duplex: send command + dummy bytes, receive garbage + data */
    uint8_t tx[32] = {reg | 0x80};
    uint8_t rx[32] = {0};
    if (len + 1 > sizeof(tx)) return ESP_ERR_INVALID_SIZE;

    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_transmit(bme280_spi, &t);
    if (err == ESP_OK) {
        memcpy(buf, &rx[1], len);
    }
    return err;
}

static esp_err_t bme280_read_calib(void)
{
    uint8_t buf[26];
    esp_err_t err = bme280_read_regs(BME280_REG_CALIB_00, buf, 26);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "calib read 0x88 failed");
        return err;
    }

    calib.dig_T1 = (buf[1] << 8) | buf[0];
    calib.dig_T2 = (buf[3] << 8) | buf[2];
    calib.dig_T3 = (buf[5] << 8) | buf[4];
    calib.dig_P1 = (buf[7] << 8) | buf[6];
    calib.dig_P2 = (buf[9] << 8) | buf[8];
    calib.dig_P3 = (buf[11] << 8) | buf[10];
    calib.dig_P4 = (buf[13] << 8) | buf[12];
    calib.dig_P5 = (buf[15] << 8) | buf[14];
    calib.dig_P6 = (buf[17] << 8) | buf[16];
    calib.dig_P7 = (buf[19] << 8) | buf[18];
    calib.dig_P8 = (buf[21] << 8) | buf[20];
    calib.dig_P9 = (buf[23] << 8) | buf[22];
    calib.dig_H1 = buf[25];

    uint8_t buf2[7];
    err = bme280_read_regs(BME280_REG_CALIB_26, buf2, 7);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "calib read 0xE1 failed");
        return err;
    }
    calib.dig_H2 = (buf2[1] << 8) | buf2[0];
    calib.dig_H3 = buf2[2];
    calib.dig_H4 = ((int16_t)buf2[3] << 4) | (buf2[4] & 0x0F);
    calib.dig_H5 = ((int16_t)buf2[5] << 4) | ((buf2[4] >> 4) & 0x0F);
    calib.dig_H6 = (int8_t)buf2[6];

    return ESP_OK;
}

/* Bosch integer compensation formulas */
static int32_t bme280_compensate_temp(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
                    ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

static uint32_t bme280_compensate_press(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = (int64_t)t_fine - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) +
           ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    return (uint32_t)p;
}

static uint32_t bme280_compensate_hum(int32_t adc_H)
{
    int32_t v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) -
                    (((int32_t)calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) *
                     (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)calib.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)calib.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

static esp_err_t bme280_init(void)
{
    spi_bus_config_t bus_cfg = {
        .miso_io_num = BME280_MISO,
        .mosi_io_num = BME280_MOSI,
        .sclk_io_num = BME280_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 500000,
        .mode = 0,  /* CPOL=0, CPHA=0 */
        .spics_io_num = BME280_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &bme280_spi));

    uint8_t chip_id = 0;
    ESP_ERROR_CHECK(bme280_read_regs(BME280_REG_CHIP_ID, &chip_id, 1));
    if (chip_id != BME280_CHIP_ID) {
        ESP_LOGE(TAG, "BME280 not found! Chip ID: 0x%02X (expected 0x%02X)", chip_id, BME280_CHIP_ID);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BME280 found, chip ID: 0x%02X", chip_id);

    ESP_ERROR_CHECK(bme280_read_calib());

    /* Soft reset */
    bme280_write_reg(BME280_REG_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Humidity oversampling x1 */
    bme280_write_reg(BME280_REG_CTRL_HUM, 0x01);
    /* Standby 0.5 ms, no filter */
    bme280_write_reg(BME280_REG_CONFIG, 0x00);
    /* Temp x1, Press x1, Forced mode */
    bme280_write_reg(BME280_REG_CTRL_MEAS, 0x25);

    ESP_LOGI(TAG, "BME280 initialized (SPI)");
    return ESP_OK;
}

static esp_err_t bme280_read(float *temp, float *press, float *hum)
{
    /* Trigger forced measurement */
    esp_err_t err = bme280_write_reg(BME280_REG_CTRL_MEAS, 0x25);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "trigger failed");
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(15));  /* conversion time */

    uint8_t data[8];
    err = bme280_read_regs(BME280_REG_DATA, data, 8);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "data read failed");
        return err;
    }

    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);
    int32_t adc_H = ((int32_t)data[6] << 8) | (int32_t)data[7];

    int32_t temp_raw = bme280_compensate_temp(adc_T);
    *temp = (float)temp_raw / 100.0f;

    uint32_t press_raw = bme280_compensate_press(adc_P);
    *press = (float)press_raw / 25600.0f;  /* Pa -> hPa */

    uint32_t hum_raw = bme280_compensate_hum(adc_H);
    *hum = (float)hum_raw / 1024.0f;

    return ESP_OK;
}

/* -------------------- WiFi -------------------- */
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT         BIT1

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        static int retry = 0;
        if (retry < 5) {
            esp_wifi_connect();
            retry++;
            ESP_LOGI(TAG, "WiFi retry %d/5", retry);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi init finished, waiting for connection...");

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    }
}

/* -------------------- MQTT -------------------- */
static esp_mqtt_client_handle_t mqtt_client = NULL;
static EventGroupHandle_t mqtt_event_group;
#define MQTT_CONNECTED_BIT  BIT0

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        xEventGroupSetBits(mqtt_event_group, MQTT_CONNECTED_BIT);

        /* Subscribe to the same topics for loopback testing */
        esp_mqtt_client_subscribe(mqtt_client, TOPIC_TEMP, 0);
        esp_mqtt_client_subscribe(mqtt_client, TOPIC_HUM, 0);
        esp_mqtt_client_subscribe(mqtt_client, TOPIC_PRES, 0);
        ESP_LOGI(TAG, "Subscribed to %s, %s, %s", TOPIC_TEMP, TOPIC_HUM, TOPIC_PRES);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT disconnected");
        xEventGroupClearBits(mqtt_event_group, MQTT_CONNECTED_BIT);
        break;

    case MQTT_EVENT_DATA: {
        char topic[64] = {0};
        char data[64] = {0};
        int tlen = event->topic_len < sizeof(topic) - 1 ? event->topic_len : sizeof(topic) - 1;
        int dlen = event->data_len < sizeof(data) - 1 ? event->data_len : sizeof(data) - 1;
        memcpy(topic, event->topic, tlen);
        memcpy(data, event->data, dlen);
        ESP_LOGI(TAG, "RX [%s] -> %s", topic, data);
        break;
    }

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;

    default:
        break;
    }
}

static void mqtt_init(void)
{
    mqtt_event_group = xEventGroupCreate();

    char uri[80];
    snprintf(uri, sizeof(uri), "mqtt://%s:%d", MQTT_BROKER_IP, MQTT_BROKER_PORT);
    ESP_LOGI(TAG, "MQTT broker URI: %s", uri);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = uri,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                                    mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));

    xEventGroupWaitBits(mqtt_event_group, MQTT_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

/* -------------------- Main -------------------- */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "=== BME280 MQTT project starting ===");

    wifi_init();
    mqtt_init();

    if (bme280_init() != ESP_OK) {
        ESP_LOGE(TAG, "BME280 init failed, halting");
        vTaskDelay(portMAX_DELAY);
    }

    while (1) {
        float temp, press, hum;
        if (bme280_read(&temp, &press, &hum) == ESP_OK) {
            /* Full format: T21.2  P762  H60% */
            char env_str[64];
            snprintf(env_str, sizeof(env_str), "T%.1f  P%.0f  H%.0f%%", temp, press, hum);
            ESP_LOGI(TAG, "%s", env_str);

            char val[16];

            /* Publish individual values to their topics */
            snprintf(val, sizeof(val), "T%.1f", temp);
            esp_mqtt_client_publish(mqtt_client, TOPIC_TEMP, val, 0, 0, 0);

            snprintf(val, sizeof(val), "H%.0f%%", hum);
            esp_mqtt_client_publish(mqtt_client, TOPIC_HUM, val, 0, 0, 0);

            snprintf(val, sizeof(val), "P%.0f", press);
            esp_mqtt_client_publish(mqtt_client, TOPIC_PRES, val, 0, 0, 0);

            /* Also publish the full formatted snapshot string */
            esp_mqtt_client_publish(mqtt_client, TOPIC_ENV, env_str, 0, 0, 0);
        } else {
            ESP_LOGE(TAG, "BME280 read failed");
        }

        vTaskDelay(pdMS_TO_TICKS(500));  /* 2 times per second */
    }
}
