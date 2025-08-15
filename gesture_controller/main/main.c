#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

static const char *TAG = "GESTURE_CTRL";

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_FREQ_HZ          400000
#define MPU6050_DEVICE_ADDRESS      0x68

#define LED_PIN                     GPIO_NUM_2
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



esp_err_t mpu6050_init_wake_on_motion(void) {
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
    cmd_buffer[0] = MPU6050_MOT_THR_REG; cmd_buffer[1] = 5;
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
    const float acce_sensitivity = 8192.0f; // 4G

    while (1) {
        if (xSemaphoreTake(motion_sem, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Motion Interrupt Received by Task!");

            i2c_master_transmit_receive(dev_handle, &((uint8_t){MPU6050_INT_STATUS}), 1, &int_status, 1, -1);
            
            if (int_status & MPU6050_MOT_DETECT_INT_BIT) {
                i2c_master_transmit_receive(dev_handle, &((uint8_t){MPU6050_ACCEL_XOUT_H}), 1, acce_buffer, 6, -1);
                int16_t raw_acce_z = (int16_t)((acce_buffer[4] << 8) | acce_buffer[5]);
                float acce_z = raw_acce_z / acce_sensitivity;

                ESP_LOGI(TAG, "Z-axis value: %.2f g", acce_z);

                if (acce_z > FLIP_THRESHOLD_G) {
                    ESP_LOGW(TAG, "== Palm UP ==> LED OFF");
                    gpio_set_level(LED_PIN, 0);
                } else if (acce_z < -FLIP_THRESHOLD_G) {
                    ESP_LOGW(TAG, "== Palm DOWN ==> LED ON");
                    gpio_set_level(LED_PIN, 1);
                }
            }
        }
    }
}

void app_main(void) {
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
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

    ESP_ERROR_CHECK(mpu6050_init_wake_on_motion());

    gpio_install_isr_service(0);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MPU6050_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);
    gpio_isr_handler_add(MPU6050_INT_PIN, gpio_isr_handler, NULL);

    xTaskCreate(gesture_detect_task, "gesture_detect_task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Initialization complete. Waiting for gestures...");
}
