#include "led_driver.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "pca9685.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "led-driver";

#define I2C_MASTER_PORT             I2C_NUM_0
#define I2C_MASTER_SCL_IO           CONFIG_I2C_SCL
#define I2C_MASTER_SDA_IO           CONFIG_I2C_SDA
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0                   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                   /*!< I2C master doesn't need buffer */

#define ANIM_TASK_PRIO 5

#define GET_ADDRESS(channel) (CONFIG_I2C_STARTING_ADDRESS + channel / 16)
#define GET_PIN(channel) (channel % 16)

u_int16_t numLeds = 2;
u_int8_t channelMap[CONFIG_I2C_DEVICES * 16] = {0, 1};

static SemaphoreHandle_t sync_pca9685, sync_animation_task;

void led_driver_init(unsigned short freqHz) {
    int i2c_master_port = I2C_MASTER_PORT;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));  
    ESP_LOGI(TAG, "i2c driver initialized. SCL %i. SDA %i.", conf.scl_io_num, conf.sda_io_num);

    sync_pca9685 = xSemaphoreCreateBinary();
    for (uint8_t i=0; i<CONFIG_I2C_DEVICES; ++i) {
        ESP_LOGI(TAG, "Starting PWM driver at address 0x%02x. Freq %i", CONFIG_I2C_STARTING_ADDRESS + i, freqHz);
        pca9685_begin(CONFIG_I2C_STARTING_ADDRESS + i, freqHz);
    }
    xSemaphoreGive(sync_pca9685);

    sync_animation_task = xSemaphoreCreateBinary();
}

void led_driver_cleanup() {
    xSemaphoreTake(sync_pca9685, portMAX_DELAY);

    for (uint8_t i=0; i<CONFIG_I2C_DEVICES; ++i) {
        pca9685_sleep(CONFIG_I2C_STARTING_ADDRESS + i);
    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_PORT));
    ESP_LOGI(TAG, "i2c driver de-initialized.");
}

void led_driver_map(u_int16_t ledIndex, u_int16_t pwmChannel) {
    channelMap[ledIndex] = pwmChannel;
}

void led_driver_setNumLeds(u_int16_t num) {
    if (num > CONFIG_I2C_DEVICES * 16) {
        ESP_LOGE(TAG, "Only %i leds are available. Asked for %i", CONFIG_I2C_DEVICES * 16, num);
        numLeds = CONFIG_I2C_DEVICES * 16;

        return;
    }

    numLeds = num;
}

static void setPin(uint8_t channel, uint16_t pwm) {
    xSemaphoreTake(sync_pca9685, 100 / portTICK_PERIOD_MS);
    // ESP_LOGI(TAG, "Setting channel %i to %i. Address 0x%02x. Pin %i.", channel, pwm, GET_ADDRESS(channel), GET_PIN(channel));
    pca9685_setPin(GET_ADDRESS(channel), GET_PIN(channel), pwm, false);
    xSemaphoreGive(sync_pca9685);
}

static void demoTask() {
    static uint16_t const maxBrightness = 500;
    static uint16_t const maxStep = 800; // 10ms per step
    static uint16_t step = 0;
    const uint16_t stepsPerLed = maxStep / numLeds;
    uint16_t stepInLed, lastPin, currentPin;
    uint16_t pwm;

    lastPin = channelMap[step / stepsPerLed];

    xSemaphoreTake(sync_animation_task, portMAX_DELAY);

    ESP_LOGI(TAG, "Running DEMO animation");

    while (1) {
        step = (step + 1) % maxStep;
        stepInLed = step % stepsPerLed;

        if (stepInLed < stepsPerLed / 2) {
            pwm = maxBrightness * stepInLed * 2 / stepsPerLed;
        } else {
            pwm = maxBrightness * (stepsPerLed - stepInLed) * 2 / stepsPerLed;
        }

        currentPin = channelMap[step / stepsPerLed];
        
        setPin(currentPin, pwm);

        if (currentPin != lastPin) {
            setPin(lastPin, 0);
            lastPin = currentPin;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void led_driver_startInfiniteAnimation() {
    xTaskCreatePinnedToCore(demoTask, "ledAnimation", 4096, NULL, ANIM_TASK_PRIO, NULL, tskNO_AFFINITY);
    xSemaphoreGive(sync_animation_task);
}