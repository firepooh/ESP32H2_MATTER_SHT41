#include <esp_log.h>
#include <sdkconfig.h>

//#include <app/server/Server.h>
#include <esp_matter.h>

#include <string.h>

#include <common_macros.h>

#include <sht4x.h>

//#define CONFIG_BATT_LEVEL_USED

#if defined(CONFIG_BATT_LEVEL_USED)
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#endif

#define CONFIG_EXAMPLE_I2C_MASTER_SCL       GPIO_NUM_2
#define CONFIG_EXAMPLE_I2C_MASTER_SDA       GPIO_NUM_3
#define CONFIG_I2C_MASTER_NUM               I2C_NUM_0


#if defined(CONFIG_BATT_LEVEL_USED)
#define VBAT_ADC_UNIT         ADC_UNIT_1
#define VBAT_ADC_CHANNEL      ADC_CHANNEL_0   // 예: ESP32-C6 GPIO4. 보드에 맞게 변경
#define VBAT_ADC_ATTEN        ADC_ATTEN_DB_12 //  ~3.3V 대응(분배 후 입력전압 기준)
#define VBAT_DIV_R1           3900            // 분모/분자 편하게 하려고 Ω 대신 0.1kΩ 단위
#define VBAT_DIV_R2           1000            // 예: 3.9MΩ : 1.0MΩ -> R1=3900, R2=1000
#define VBAT_FULL_MV          4200            // Li-ion 1셀 가정. 코인셀 등은 값 조정
#define VBAT_EMPTY_MV         3300            // 방전 하한 가정. 프로젝트에 맞게 조정
#endif


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

    // ADC/Battery
#if defined(CONFIG_BATT_LEVEL_USED)
    adc_oneshot_unit_handle_t adc_unit = nullptr;
    adc_cali_handle_t adc_cali = nullptr;
    uint32_t vBattmV;
    uint16_t ps_endpoint_id = 0; // Power Source endpoint id    
#endif    
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
#if defined(CONFIG_BATT_LEVEL_USED)
static void battery_update_notification(uint16_t endpoint_id, uint32_t mv);
static uint8_t vbat_mv_to_percent_x2(uint32_t mv);
static uint32_t battery_read_vbat_mv( void );
#endif

#if defined(CONFIG_BATT_LEVEL_USED)
static bool battery_adc_init()
{
    // --- ADC unit ---
    adc_oneshot_unit_init_cfg_t init_cfg = {};
    init_cfg.unit_id = VBAT_ADC_UNIT;
    esp_err_t err = adc_oneshot_new_unit(&init_cfg, &s_ctx.adc_unit);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SENSOR, "adc unit: %s", esp_err_to_name(err));
        return false;
    }

    // --- ADC channel cfg ---
    adc_oneshot_chan_cfg_t chan_cfg = {};
    // 멤버 직접 대입(순서 문제 회피)
    chan_cfg.atten    = VBAT_ADC_ATTEN;
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;

    err = adc_oneshot_config_channel(s_ctx.adc_unit, VBAT_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_SENSOR, "adc ch: %s", esp_err_to_name(err));
        return false;
    }

    // --- Calibration (curve fitting) ---
    adc_cali_curve_fitting_config_t cali_cfg = {};
    cali_cfg.unit_id  = VBAT_ADC_UNIT;
    cali_cfg.atten    = VBAT_ADC_ATTEN;
    cali_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;

    err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_ctx.adc_cali);
    if (err != ESP_OK) {
        ESP_LOGW(TAG_SENSOR, "ADC calibration not available; will report raw mV");
        s_ctx.adc_cali = nullptr;
    }
    return true;
}

#endif

void sensor_init( void )
{
    ESP_ERROR_CHECK(i2cdev_init());
    memset(&s_ctx.dev, 0, sizeof(sht4x_t));

    ESP_ERROR_CHECK(sht4x_init_desc(&s_ctx.dev, CONFIG_I2C_MASTER_NUM, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(sht4x_init(&s_ctx.dev));

    #if defined(CONFIG_BATT_LEVEL_USED)
    battery_adc_init();
    #endif
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

#if defined(CONFIG_BATT_LEVEL_USED)
  if (ctx->ps_endpoint_id) {
    #if 0
    uint32_t vbat_mv = battery_read_vbat_mv();
    #else
    ctx->vBattmV += 100;
    if( ctx->vBattmV > 4200 ) ctx->vBattmV = 3000; // Simulate battery voltage
    #endif
    
    battery_update_notification(ctx->ps_endpoint_id, ctx->vBattmV);

    ESP_LOGI(TAG_SENSOR, "Battery: %lu mV, %u/200 (%.1f%%)",
             (unsigned long)(ctx->vBattmV), (unsigned)(vbat_mv_to_percent_x2(ctx->vBattmV)), (double)(vbat_mv_to_percent_x2(ctx->vBattmV) / 2.0f));
  }  
#endif  
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

#if defined(CONFIG_BATT_LEVEL_USED)
static uint8_t vbat_mv_to_percent_x2(uint32_t mv)
{
    if (mv <= VBAT_EMPTY_MV) return 0;
    if (mv >= VBAT_FULL_MV)  return 200;
    return (uint8_t)(((mv - VBAT_EMPTY_MV) * 200) / (VBAT_FULL_MV - VBAT_EMPTY_MV));
}

static uint32_t battery_read_vbat_mv( void )
{
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(s_ctx.adc_unit, VBAT_ADC_CHANNEL, &raw));
    int mv = raw; // fallback
    if (s_ctx.adc_cali) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(s_ctx.adc_cali, raw, &mv)); // 분배 전 입력핀 mV
    }
    // 분배비 보정: Vbat = Vin * (R1 + R2) / R2
    uint64_t vbat = (uint64_t)mv * (VBAT_DIV_R1 + VBAT_DIV_R2) / VBAT_DIV_R2;
    return (uint32_t)vbat; // mV
}

static void battery_update_notification(uint16_t endpoint_id, uint32_t mv)
{
    uint8_t pct_x2 = vbat_mv_to_percent_x2(mv);

    chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint_id, mv, pct_x2]() {
        using namespace esp_matter;
        using namespace esp_matter::attribute;
        using namespace chip::app::Clusters;

        // BatVoltage (mV)
        if (attribute_t *attr_v = get(endpoint_id, PowerSource::Id,
                                      PowerSource::Attributes::BatVoltage::Id)) {
            esp_matter_attr_val_t val = esp_matter_invalid(NULL);
            get_val(attr_v, &val);     // 현재 타입 메타를 따라감(Nullable 포함)
            val.val.u32 = mv;          // mV
            update(endpoint_id, PowerSource::Id,
                   PowerSource::Attributes::BatVoltage::Id, &val);
        }

        // BatPercentRemaining (0~200, 0.5% 단위)
        if (attribute_t *attr_p = get(endpoint_id, PowerSource::Id,
                                      PowerSource::Attributes::BatPercentRemaining::Id)) {
            esp_matter_attr_val_t val = esp_matter_invalid(NULL);
            get_val(attr_p, &val);
            val.val.u8 = pct_x2;       // 0~200
            update(endpoint_id, PowerSource::Id,
                   PowerSource::Attributes::BatPercentRemaining::Id, &val);
        }
    });
}


static void power_source_create_endpoint(node_t *node)
{
    // 1) Power Source 디바이스 엔드포인트 생성
    power_source_device::config_t ps_cfg = {};
    // description은 device-config 내부의 cluster-config에 들어있습니다.
    strncpy(ps_cfg.power_source.description, "Main Battery",sizeof(ps_cfg.power_source.description) - 1);

    // (선택) 교체 가능/충전가능 등 속성 프리셋은 features.battery 쪽에 설정
    ps_cfg.power_source.features.battery.bat_replaceability = (uint8_t)chip::app::Clusters::PowerSource::BatReplaceabilityEnum::kNotReplaceable;
    ps_cfg.power_source.features.battery.bat_charge_level = (uint8_t)chip::app::Clusters::PowerSource::BatChargeLevelEnum::kOk;

    endpoint_t *ps_ep = power_source_device::create(node, &ps_cfg, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(ps_ep != nullptr,ESP_LOGE(TAG_SENSOR, "Failed to create power_source endpoint"));

    s_ctx.ps_endpoint_id = endpoint::get_id(ps_ep);

    // 2) 배터리 기능(feature) 활성화 + 속성 생성
    cluster_t *ps_cluster = esp_matter::cluster::get(ps_ep, chip::app::Clusters::PowerSource::Id);

    // 네임스페이스를 완전수식으로 호출(가장 안전)
    ESP_ERROR_CHECK( esp_matter::cluster::power_source::feature::battery::add(ps_cluster, &ps_cfg.power_source.features.battery));

    // (버전에 따라 자동 생성될 수도 있음)
    esp_matter::cluster::power_source::attribute::create_bat_percent_remaining(ps_cluster, nullable<uint8_t>(50), /*min*/0, /*max*/200);
    esp_matter::cluster::power_source::attribute::create_bat_voltage(ps_cluster, nullable<uint32_t>(4200), /*min*/0, /*max*/6000);
}
#endif

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

    #if defined(CONFIG_BATT_LEVEL_USED)
    // Battery (Power Source)
    power_source_create_endpoint(node);
    #endif
}