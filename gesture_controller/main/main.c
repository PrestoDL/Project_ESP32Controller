#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_http_client.h"

static const char *TAG = "GESTURE_CTRL";

#define WIFI_SSID       "null"
#define WIFI_PASSWORD   "null"
#define RPI_IP          "null"
#define RPI_PORT        "null"

#define I2C_MASTER_SCL_IO           GPIO_NUM_22
#define I2C_MASTER_SDA_IO           GPIO_NUM_21
#define I2C_MASTER_FREQ_HZ          400000
#define MPU6050_DEVICE_ADDRESS      0x68
#define MPU6050_INT_PIN             GPIO_NUM_4
#define FLIP_THRESHOLD_G            0.7f

#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_CONFIG              0x1A
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_MOT_THR_REG         0x1F
#define MPU6050_MOT_DUR_REG         0x20
#define MPU6050_INT_PIN_CFG         0x37
#define MPU6050_INT_ENABLE          0x38
#define MPU6050_INT_STATUS          0x3A
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_PWR_MGMT_2          0x6C
#define MPU6050_MOT_DETECT_INT_BIT  (1 << 6)



static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
static SemaphoreHandle_t motion_sem;
static bool is_wifi_connected = false;



void send_light_command(const char* state)
{
    if (!is_wifi_connected) {
        ESP_LOGE(TAG, "Wi-Fi is not Connected.");
        return;
    }

    char path_buffer[32];
    sprintf(path_buffer, "/light/%s", state);

    esp_http_client_config_t config = {
        .host = RPI_IP,
        .port = RPI_PORT,
        .path = path_buffer,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
        .timeout_ms = 10000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP GET Request Success (%s), Code = %d", state, esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "HTTP GET Request Failed : %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    // vTaskDelete(NULL);
}

esp_err_t mpu6050_init(void)
{
    esp_err_t ret;
    uint8_t cmd_buffer[2];

    cmd_buffer[0] = MPU6050_PWR_MGMT_1; cmd_buffer[1] = 0x80;
    ret = i2c_master_transmit(dev_handle, cmd_buffer, 2, -1); vTaskDelay(pdMS_TO_TICKS(100));
    cmd_buffer[0] = MPU6050_PWR_MGMT_1; cmd_buffer[1] = 0x01;
    ret = i2c_master_transmit(dev_handle, cmd_buffer, 2, -1);
    cmd_buffer[0] = MPU6050_PWR_MGMT_2; cmd_buffer[1] = 0x00;
    ret = i2c_master_transmit(dev_handle, cmd_buffer, 2, -1); vTaskDelay(pdMS_TO_TICKS(100));
    cmd_buffer[0] = MPU6050_CONFIG; cmd_buffer[1] = 0x01;
    ret = i2c_master_transmit(dev_handle, cmd_buffer, 2, -1);
    cmd_buffer[0] = MPU6050_ACCEL_CONFIG; cmd_buffer[1] = 0x08;
    ret = i2c_master_transmit(dev_handle, cmd_buffer, 2, -1);
    cmd_buffer[0] = MPU6050_MOT_THR_REG; cmd_buffer[1] = 2;
    ret = i2c_master_transmit(dev_handle, cmd_buffer, 2, -1);
    cmd_buffer[0] = MPU6050_MOT_DUR_REG; cmd_buffer[1] = 1;
    ret = i2c_master_transmit(dev_handle, cmd_buffer, 2, -1);
    cmd_buffer[0] = MPU6050_INT_PIN_CFG; cmd_buffer[1] = 0x20;
    ret = i2c_master_transmit(dev_handle, cmd_buffer, 2, -1);
    cmd_buffer[0] = MPU6050_INT_ENABLE; cmd_buffer[1] = MPU6050_MOT_DETECT_INT_BIT;
    ret = i2c_master_transmit(dev_handle, cmd_buffer, 2, -1);
    cmd_buffer[0] = MPU6050_PWR_MGMT_1; cmd_buffer[1] = 0x21;
    ret = i2c_master_transmit(dev_handle, cmd_buffer, 2, -1);
    cmd_buffer[0] = MPU6050_PWR_MGMT_2; cmd_buffer[1] = 0xC7;
    ret = i2c_master_transmit(dev_handle, cmd_buffer, 2, -1);
    return ret;
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    xSemaphoreGiveFromISR(motion_sem, NULL);
}

static void gesture_detect_task(void *arg) {
    uint8_t int_status;
    uint8_t acce_buffer[6];
    const float acce_sensitivity = 8192.0f;

    while (1) {
        if (xSemaphoreTake(motion_sem, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Motion Interrupt Received by Task!");

            i2c_master_transmit_receive(dev_handle, &((uint8_t){MPU6050_INT_STATUS}), 1, &int_status, 1, -1);
            
            if (int_status & MPU6050_MOT_DETECT_INT_BIT) {
                i2c_master_transmit_receive(dev_handle, &((uint8_t){MPU6050_ACCEL_XOUT_H}), 1, acce_buffer, 6, -1);
                int16_t raw_acce_z = (int16_t)((acce_buffer[4] << 8) | acce_buffer[5]);
                float acce_z = raw_acce_z / acce_sensitivity;

                if (acce_z > FLIP_THRESHOLD_G) {
                    ESP_LOGW(TAG, "== Palm UP ==> LED ON");
                    send_light_command("on");
                } else if (acce_z < -FLIP_THRESHOLD_G) {
                    ESP_LOGW(TAG, "== Palm UP ==> LED OFF");
                    send_light_command("off");
                }
            }
        }
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        is_wifi_connected = false;
        ESP_LOGI(TAG, "Wi-Fi Disconnected. Retry...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Wi-Fi Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        is_wifi_connected = true;
    }
}

void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));
    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD, }, };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi Initiated.");
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    motion_sem = xSemaphoreCreateBinary();

    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_DEVICE_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    ESP_ERROR_CHECK(mpu6050_init());
    
    wifi_init_sta();

    gpio_install_isr_service(0);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MPU6050_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);
    gpio_isr_handler_add(MPU6050_INT_PIN, gpio_isr_handler, NULL);

    xTaskCreate(gesture_detect_task, "gesture_detect_task", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "Initialization complete. Waiting for gestures...");
}
