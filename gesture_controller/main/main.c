#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "mpu6050.h"

static const char *TAG = "GESTURE_CTRL";

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000

#define LED_PIN                     GPIO_NUM_2
#define MPU6050_INT_PIN             GPIO_NUM_4
#define FLIP_THRESHOLD_G            0.7f

#define MPU6050_MOT_THR_REG         0x1F

static mpu6050_handle_t mpu6050 = NULL;
static SemaphoreHandle_t motion_sem;



static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t set_motion_threshold(uint8_t threshold) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_MOT_THR_REG, true);
    i2c_master_write_byte(cmd, threshold, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    xSemaphoreGiveFromISR(motion_sem, NULL);
}

static void gesture_detect_task(void *arg) {
    uint8_t int_status;
    mpu6050_acce_value_t acce;

    while (1) {
        if (xSemaphoreTake(motion_sem, portMAX_DELAY) == pdTRUE) {
            mpu6050_get_interrupt_status(mpu6050, &int_status);
            if (mpu6050_is_i2c_master_interrupt(int_status)) {
                 if (mpu6050_get_acce(mpu6050, &acce) == ESP_OK) {
                    ESP_LOGI(TAG, "Motion Detected! Z-axis value: %.2f g", acce.acce_z);
                    if (acce.acce_z > FLIP_THRESHOLD_G) {
                        ESP_LOGW(TAG, "== Palm DOWN ==> LED ON");
                        gpio_set_level(LED_PIN, 1);
                    } else if (acce.acce_z < -FLIP_THRESHOLD_G) {
                        ESP_LOGW(TAG, "== Palm UP ==> LED OFF");
                        gpio_set_level(LED_PIN, 0);
                    }
                }
            }
        }
    }
}

void app_main(void) {
    motion_sem = xSemaphoreCreateBinary();

    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    ESP_ERROR_CHECK(i2c_master_init());
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    ESP_ERROR_CHECK(mpu6050_wake_up(mpu6050));
    ESP_ERROR_CHECK(mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS));

    ESP_LOGI(TAG, "Configuring MPU6050 motion detection...");
    
    gpio_install_isr_service(0);

    mpu6050_int_config_t int_config = {
        .interrupt_pin = MPU6050_INT_PIN,
        .active_level = INTERRUPT_PIN_ACTIVE_HIGH,
        .pin_mode = INTERRUPT_PIN_PUSH_PULL,
        .interrupt_latch = INTERRUPT_LATCH_UNTIL_CLEARED,
        .interrupt_clear_behavior = INTERRUPT_CLEAR_ON_ANY_READ,
    };
    ESP_ERROR_CHECK(mpu6050_config_interrupts(mpu6050, &int_config));
    ESP_ERROR_CHECK(set_motion_threshold(20)); 
    ESP_ERROR_CHECK(mpu6050_enable_interrupts(mpu6050, MPU6050_MOT_DETECT_INT_BIT));
    ESP_ERROR_CHECK(mpu6050_register_isr(mpu6050, gpio_isr_handler));

    ESP_LOGI(TAG, "Initialization complete. Waiting for gestures...");

    xTaskCreate(gesture_detect_task, "gesture_detect_task", 4096, NULL, 10, NULL);
}
