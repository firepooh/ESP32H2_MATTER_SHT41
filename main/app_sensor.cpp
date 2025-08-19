#include <esp_log.h>
#include <sdkconfig.h>

//#include <app/server/Server.h>
#include <esp_matter.h>

#include <string.h>

#include <common_macros.h>

#include <sht4x.h>

#define CONFIG_EXAMPLE_I2C_MASTER_SCL       GPIO_NUM_2
#define CONFIG_EXAMPLE_I2C_MASTER_SDA       GPIO_NUM_3
#define CONFIG_I2C_MASTER_NUM               I2C_NUM_0


using sensor_cb_t = void (*)(uint16_t endpoint_id, float value, void *user_data);

typedef struct {
    struct {
        sensor_cb_t cb = NULL;  // This callback functon will be called periodically to report the temperature.
        uint16_t endpoint_id;   // endpoint_id associated with temperature sensor
    } temperature;

    struct {
        sensor_cb_t cb = NULL;  // This callback functon will be called periodically to report the humidity.
        uint16_t endpoint_id;   // endpoint_id associated with humidity sensor
    } humidity;

    void *user_data = NULL;     // user data

    uint32_t interval_ms = 10000;// polling interval in milliseconds, defaults to 5000 ms  
} sensor_config_t;


typedef struct {
    sht4x_t dev;
    sensor_config_t *config;
    esp_timer_handle_t timer;
    bool is_initialized = false;
} sensor_ctx_t;

static constexpr char *TAG_SENSOR = "sensor";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;


static sensor_ctx_t s_ctx;

static void temp_sensor_notification(uint16_t endpoint_id, float temp, void *user_data);
static void humidity_sensor_notification(uint16_t endpoint_id, float humidity, void *user_data);
void sensor_get( float *temperature, float *humidity );

void sensor_init( void )
{
    ESP_ERROR_CHECK(i2cdev_init());
    memset(&s_ctx.dev, 0, sizeof(sht4x_t));

    ESP_ERROR_CHECK(sht4x_init_desc(&s_ctx.dev, CONFIG_I2C_MASTER_NUM, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(sht4x_init(&s_ctx.dev));
}

void sensor_timer_callback(void *arg)
{
  auto *ctx = (sensor_ctx_t *) arg;
  if (!(ctx && ctx->config)) {
    return;
  }

  float temp, humidity;
  sensor_get(&temp, &humidity);

  if (ctx->config->temperature.cb) {
    ctx->config->temperature.cb(ctx->config->temperature.endpoint_id, temp, ctx->config->user_data);
  }
  if (ctx->config->humidity.cb) {
    ctx->config->humidity.cb(ctx->config->humidity.endpoint_id, humidity, ctx->config->user_data);
  }
}

void sensor_start( uint32_t interval_secs )
{
  /* init timer priodic */
  esp_timer_create_args_t timer_args = {
      .callback = &sensor_timer_callback,
      .arg = &s_ctx,
      .name = "sensor_timer",
  };

  s_ctx.config->interval_ms = interval_secs * 1000;

  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_ctx.timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(s_ctx.timer, s_ctx.config->interval_ms * 1000));
}

void sensor_get( float *temperature, float *humidity )
{
    /* Use High Level Driver */
    ESP_ERROR_CHECK(sht4x_measure(&s_ctx.dev, temperature, humidity));
    ESP_LOGI(TAG_SENSOR,"sht4x Sensor: %.2f °C, %.2f %%\n", *temperature, *humidity);
}

// Application cluster specification, 7.18.2.11. Temperature
// represents a temperature on the Celsius scale with a resolution of 0.01°C.
// temp = (temperature in °C) x 100
static void temp_sensor_notification(uint16_t endpoint_id, float temp, void *user_data)
{
    // schedule the attribute update so that we can report it from matter thread
    chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint_id, temp]() {
        attribute_t * attribute = attribute::get(endpoint_id,
                                                 TemperatureMeasurement::Id,
                                                 TemperatureMeasurement::Attributes::MeasuredValue::Id);

        esp_matter_attr_val_t val = esp_matter_invalid(NULL);
        attribute::get_val(attribute, &val);
        val.val.i16 = static_cast<int16_t>(temp * 100);

        attribute::update(endpoint_id, TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id, &val);
    });
}

// Application cluster specification, 2.6.4.1. MeasuredValue Attribute
// represents the humidity in percent.
// humidity = (humidity in %) x 100
static void humidity_sensor_notification(uint16_t endpoint_id, float humidity, void *user_data)
{
    // schedule the attribute update so that we can report it from matter thread
    chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint_id, humidity]() {
        attribute_t * attribute = attribute::get(endpoint_id,
                                                 RelativeHumidityMeasurement::Id,
                                                 RelativeHumidityMeasurement::Attributes::MeasuredValue::Id);

        esp_matter_attr_val_t val = esp_matter_invalid(NULL);
        attribute::get_val(attribute, &val);
        val.val.u16 = static_cast<uint16_t>(humidity * 100);

        attribute::update(endpoint_id, RelativeHumidityMeasurement::Id, RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &val);
    });
}

void sensor_create_endpoints(node_t *node)
{
    // add temperature sensor device
    temperature_sensor::config_t temp_sensor_config;
    endpoint_t * temp_sensor_ep = temperature_sensor::create(node, &temp_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(temp_sensor_ep != nullptr, ESP_LOGE(TAG_SENSOR, "Failed to create temperature_sensor endpoint"));

    s_ctx.config->temperature.cb = temp_sensor_notification;
    s_ctx.config->temperature.endpoint_id = endpoint::get_id(temp_sensor_ep);

    // add the humidity sensor device
    humidity_sensor::config_t humidity_sensor_config;
    endpoint_t * humidity_sensor_ep = humidity_sensor::create(node, &humidity_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(humidity_sensor_ep != nullptr, ESP_LOGE(TAG_SENSOR, "Failed to create humidity_sensor endpoint"));

    s_ctx.config->humidity.cb = humidity_sensor_notification;
    s_ctx.config->humidity.endpoint_id = endpoint::get_id(humidity_sensor_ep);
}