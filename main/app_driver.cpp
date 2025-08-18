/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <esp_log.h>
#include <esp_matter.h>
#include <sdkconfig.h>

#include <app/icd/server/ICDNotifier.h>

#include <app_priv.h>
#include <iot_button.h>
#include <button_gpio.h>

#include <sht4x.h>

#define CONFIG_EXAMPLE_I2C_MASTER_SCL       GPIO_NUM_2
#define CONFIG_EXAMPLE_I2C_MASTER_SDA       GPIO_NUM_3
#define CONFIG_I2C_MASTER_NUM               I2C_NUM_0

#ifdef CONFIG_ENABLE_USER_ACTIVE_MODE_TRIGGER_BUTTON
using namespace chip::app::Clusters;
using namespace esp_matter;

static constexpr char *TAG = "app_driver";
static constexpr char *TAG_SENSOR = "sensor";

static sht4x_t dev;

void sensor_init( void )
{
    ESP_ERROR_CHECK(i2cdev_init());
    memset(&dev, 0, sizeof(sht4x_t));

    ESP_ERROR_CHECK(sht4x_init_desc(&dev, CONFIG_I2C_MASTER_NUM, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(sht4x_init(&dev));
}

void sensor_get( float *temperature, float *humidity )
{
    TickType_t last_wakeup = xTaskGetTickCount();

    ESP_ERROR_CHECK(sht4x_measure(&dev, temperature, humidity));
    ESP_LOGI(TAG_SENSOR,"sht4x Sensor: %.2f °C, %.2f %%\n", *temperature, *humidity);

    // wait until 5 seconds are over
    vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(5000));
}

static void app_driver_button_toggle_cb(void *arg, void *data)
{
    // The device will stay active mode for Active Mode Threshold
    chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t) {
        chip::app::ICDNotifier::GetInstance().NotifyNetworkActivityNotification();
    });
}

app_driver_handle_t app_driver_button_init()
{
    /* Initialize button */
    button_handle_t handle = NULL;
    const button_config_t btn_cfg = {0};
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = CONFIG_USER_ACTIVE_MODE_TRIGGER_BUTTON_PIN,
        .active_level = 0,
        .enable_power_save = true,
    };

    if (iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create button device");
        return NULL;
    }

    iot_button_register_cb(handle, BUTTON_PRESS_DOWN, NULL, app_driver_button_toggle_cb, NULL);
    return (app_driver_handle_t)handle;
}

#endif // CONFIG_ENABLE_USER_ACTIVE_MODE_TRIGGER_BUTTON
