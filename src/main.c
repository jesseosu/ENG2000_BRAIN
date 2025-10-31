#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_spiffs.h"
#include "cJSON.h"
#include "esp_timer.h"
#include "driver/ledc.h"

// Pin Configuration Structure
typedef struct {
    // System pins
    gpio_num_t led_pin;
    gpio_num_t ultrasonic_trig_pin;
    gpio_num_t ultrasonic_echo_pin;
    
    // Motor pins
    gpio_num_t motor_enca_pin;
    gpio_num_t motor_encb_pin;
    gpio_num_t motor_in1_pin;
    gpio_num_t motor_in2_pin;
    gpio_num_t motor_ena_pin;
    
    // Bridge pins
    gpio_num_t bridge_boat_trig_pin;
    gpio_num_t bridge_boat_echo_pin;
    gpio_num_t bridge_boat2_trig_pin;
    gpio_num_t bridge_boat2_echo_pin;
    gpio_num_t bridge_sw_lowered_pin;
    gpio_num_t bridge_sw_raised_pin;
    // Traffic LED pins (4 configurable)
    gpio_num_t traffic_led_r_pin;
    gpio_num_t traffic_led_y_pin;
    gpio_num_t traffic_led_g_pin;
    gpio_num_t traffic_led_b_pin;

    // Boat LED pins (4 configurable)
    gpio_num_t boat_led_r_pin;
    gpio_num_t boat_led_y_pin;
    gpio_num_t boat_led_g_pin;
    gpio_num_t boat_led_b_pin;
} pin_config_t;

// Default pin configuration
static pin_config_t default_pins = {
    .led_pin = GPIO_NUM_5,
    .ultrasonic_trig_pin = GPIO_NUM_12,
    .ultrasonic_echo_pin = GPIO_NUM_13,
    .motor_enca_pin = GPIO_NUM_2,
    .motor_encb_pin = GPIO_NUM_4,
    .motor_in1_pin = GPIO_NUM_14,
    .motor_in2_pin = GPIO_NUM_27,
    .motor_ena_pin = GPIO_NUM_26,
    .bridge_boat_trig_pin = GPIO_NUM_32,
    .bridge_boat_echo_pin = GPIO_NUM_33,
    .bridge_boat2_trig_pin = GPIO_NUM_18,
    .bridge_boat2_echo_pin = GPIO_NUM_19,
    .bridge_sw_lowered_pin = GPIO_NUM_21,
    .bridge_sw_raised_pin = GPIO_NUM_22,
    // Default traffic LEDs (reuse original RGB as R/G/B, yellow defaults to unused GPIO0)
    .traffic_led_r_pin = GPIO_NUM_16,
    .traffic_led_y_pin = GPIO_NUM_0,
    .traffic_led_g_pin = GPIO_NUM_17,
    .traffic_led_b_pin = GPIO_NUM_23,
    // Default boat LEDs (select remaining generally available GPIOs; adjust in UI as needed)
    .boat_led_r_pin = GPIO_NUM_25,
    .boat_led_y_pin = GPIO_NUM_15,
    .boat_led_g_pin = GPIO_NUM_34, // input-only; treated as unused by output ops
    .boat_led_b_pin = GPIO_NUM_35  // input-only; treated as unused by output ops
};

// Current active pin configuration
static pin_config_t pins = {
    .led_pin = GPIO_NUM_5,
    .ultrasonic_trig_pin = GPIO_NUM_12,
    .ultrasonic_echo_pin = GPIO_NUM_13,
    .motor_enca_pin = GPIO_NUM_2,
    .motor_encb_pin = GPIO_NUM_4,
    .motor_in1_pin = GPIO_NUM_14,
    .motor_in2_pin = GPIO_NUM_27,
    .motor_ena_pin = GPIO_NUM_26,
    .bridge_boat_trig_pin = GPIO_NUM_32,
    .bridge_boat_echo_pin = GPIO_NUM_33,
    .bridge_boat2_trig_pin = GPIO_NUM_18,
    .bridge_boat2_echo_pin = GPIO_NUM_19,
    .bridge_sw_lowered_pin = GPIO_NUM_21,
    .bridge_sw_raised_pin = GPIO_NUM_22,
    .traffic_led_r_pin = GPIO_NUM_16,
    .traffic_led_y_pin = GPIO_NUM_0,
    .traffic_led_g_pin = GPIO_NUM_17,
    .traffic_led_b_pin = GPIO_NUM_23,
    .boat_led_r_pin = GPIO_NUM_25,
    .boat_led_y_pin = GPIO_NUM_15,
    .boat_led_g_pin = GPIO_NUM_34,
    .boat_led_b_pin = GPIO_NUM_35
};

// NVS storage key
#define NVS_PIN_CONFIG_KEY "pin_config"
#define WIFI_SSID "ESP32-Control"
#define WIFI_PASS "password123"
#define MAX_CONNECTIONS 4
#define SOUND_SPEED 0.034  // cm/us

// PWM Configuration
#define PWM_FREQUENCY 1000  // 1 kHz
#define PWM_RESOLUTION LEDC_TIMER_8_BIT  // 8-bit resolution (0-255)
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_TIMER LEDC_TIMER_0

// Bridge PWM Configuration
#define BRIDGE_PWM_FREQUENCY 50  // 50 Hz
#define BRIDGE_PWM_RESOLUTION LEDC_TIMER_16_BIT
#define BRIDGE_PWM_CHANNEL LEDC_CHANNEL_1
#define BRIDGE_PWM_TIMER LEDC_TIMER_1

// Bridge constants
#define BOAT_PRESENT_DIST_CM 200
#define T_WARN_TRAFFIC 5000
#define T_AFTER_STOP 5000
#define T_LOWER_CAUT_YEL 5000
#define T_AFTER_LOWERED 5000
#define T_MAX_UP_HOLD 45000

// Ultrasonic calibration constants
#define BASELINE_SAMPLES 50           // Reduced from 80 for faster calibration
#define MEDIAN_SAMPLES 5              // Reduced from 7 for faster filtering  
#define REQUIRED_CONSECUTIVE 3        // Reduced from 4 for faster response
#define RESET_CONSECUTIVE 4           // Reduced from 6 for faster reset
#define THRESHOLD_CM 8.0              // cm closer than baseline for detection
#define SAMPLE_INTERVAL_MS 120        // ms between detection samples
#define MAX_SENSOR_READINGS 100       // buffer size for live graph data
#define CALIB_SAMPLE_DELAY_MS 50      // Faster sampling during calibration
// Hampel filter settings (outlier detection)
#define HAMPEL_WINDOW 7               // number of historical points to consider
#define HAMPEL_K 3.0f                 // threshold multiplier for MAD

// Detection meter parameters (0-100)
#define METER_MAX 100
#define METER_INCREMENT 25            // add per valid candidate reading
#define METER_DECAY_PER_SEC 20        // decay per second when no candidate

// Ultrasonic timeout/backoff parameters
#define ULTRASONIC_TIMEOUT_STREAK 5    // consecutive timeouts before cooldown
#define ULTRASONIC_COOLDOWN_MS 10000   // cooldown duration after repeated timeouts (ms)
#define ULTRASONIC_WARN_RATE_MS 2000   // minimum ms between printed WARN messages

// ADC Configuration for Hall Effect Sensors
#define NO_OF_SAMPLES   64          // Multisampling

static const char *TAG = "esp32_webserver";

// Ultrasonic sensor task handle
static TaskHandle_t ultrasonic_task_handle = NULL;

// Motor encoder variables
static TaskHandle_t motor_task_handle = NULL;
volatile int encoder_pulses = 0;
volatile bool encoder_direction = true; // true = forward, false = reverse
volatile uint8_t encoder_pinA_last = 0;
static bool motor_running = false;
static int motor_speed = 60; // 0-100%, default to 60%
static bool motor_direction = true; // true = forward, false = reverse

// Global distance variable
static float current_distance = -1.0;

// Ultrasonic calibration variables
typedef enum {
    CALIB_IDLE,           // Not calibrating
    CALIB_IN_PROGRESS,    // Collecting baseline samples
    CALIB_COMPLETE        // Calibration finished
} calibration_state_t;

// Boat sensor calibration
static calibration_state_t calib_state = CALIB_IDLE;
static float baseline_mean = 0.0;
static float baseline_stddev = 0.0;
static int calib_sample_count = 0;
static float calib_samples[BASELINE_SAMPLES];

// Detection state variables
static int consecutive_detections = 0;
static int consecutive_clear = 0;
static bool boat_detected = false;

// Live data buffer for graphing (boat sensor)
static float sensor_readings[MAX_SENSOR_READINGS];
static uint32_t reading_timestamps[MAX_SENSOR_READINGS];
static int reading_index = 0;
static int readings_count = 0;

static calibration_state_t boat2_calib_state = CALIB_IDLE;
static float boat2_baseline_mean = 0.0;
static float boat2_baseline_stddev = 0.0;
static int boat2_calib_sample_count = 0;
static float boat2_calib_samples[BASELINE_SAMPLES];

// Boat2 detection state variables
static int boat2_consecutive_detections = 0;
static int boat2_consecutive_clear = 0;
static bool boat2_detected = false;

// Live data buffer for graphing (boat2 sensor)
static float boat2_sensor_readings[MAX_SENSOR_READINGS];
static uint32_t boat2_reading_timestamps[MAX_SENSOR_READINGS];
static int boat2_reading_index = 0;
static int boat2_readings_count = 0;

// Boat detection meter (0-100) and timing for decay
static int boat_detection_meter = 0;
static uint32_t last_meter_update_ms = 0;
// Boat2 detection meter and timing
static int boat2_detection_meter = 0;
static uint32_t last_boat2_meter_update_ms = 0;

// Which sensor first saw the boat during an approach: 0=unknown, 1=boat1, 2=boat2
static int boat_initial_sensor = 0;

// Ultrasonic timeout tracking / cooldown state
static int boat_timeout_streak = 0;
static int boat2_timeout_streak = 0;
static uint32_t boat_cooldown_until_ms = 0;
static uint32_t boat2_cooldown_until_ms = 0;
// Rate-limit last warning timestamp (microseconds)
static int64_t last_ultrasonic_warn_time_us = 0;

// System uptime
static uint32_t system_start_time = 0;

// Bridge state management
typedef enum {
    BRIDGE_SELFTEST, BRIDGE_ARMED, BRIDGE_WARN_TRAFFIC, BRIDGE_WAIT_STOP,
    BRIDGE_CHECK_CLEAR, BRIDGE_RAISING, BRIDGE_TOP_REACHED,
    BRIDGE_PRE_LOWER_WARN, BRIDGE_LOWERING, BRIDGE_AFTER_LOWERED, BRIDGE_OPEN_TRAFFIC
} bridge_state_t;

static bridge_state_t bridge_state = BRIDGE_SELFTEST;
static uint32_t bridge_state_start = 0;
static bool bridge_top_saw_gap = false;
static uint32_t bridge_top_enter_time = 0;
static bool bridge_manual_override = false;
static TaskHandle_t bridge_task_handle = NULL;

// LED state variables for status reporting
// LED state variables for status reporting (traffic and boat)
static bool traffic_led_r_state = false;
static bool traffic_led_y_state = false;
static bool traffic_led_g_state = false;
static bool traffic_led_b_state = false;
static bool boat_led_r_state = false;
static bool boat_led_y_state = false;
static bool boat_led_g_state = false;
static bool boat_led_b_state = false;
static bool car_detection_enabled = false;

// Hall effect sensor threshold (in ADC raw value)
static int hall_effect_threshold = 1650;  // Default: triggered when below 1650mV

// ADC handles for hall effect sensors
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;

// WebSocket handles for real-time data streaming
// Forward declarations
void init_spiffs(void);
void wifi_init_softap(void);
httpd_handle_t start_webserver(void);
float measure_distance(gpio_num_t trig_pin, gpio_num_t echo_pin);
void sse_task(void *pvParameters);
// Filtering helpers
float get_filtered_distance_for(gpio_num_t trig_pin, gpio_num_t echo_pin);
float apply_hampel_history(float value, float *history_buf, int history_count, int history_index);

// LED helpers
static inline bool is_output_capable(gpio_num_t pin) {
    // GPIO 34-39 are input-only on classic ESP32; others considered output-capable for this context
    return !(pin >= GPIO_NUM_34 && pin <= GPIO_NUM_39);
}
static void traffic_led(bool r, bool y, bool g, bool b) {
    traffic_led_r_state = r;
    traffic_led_y_state = y;
    traffic_led_g_state = g;
    traffic_led_b_state = b;
    if (is_output_capable(pins.traffic_led_r_pin)) gpio_set_level(pins.traffic_led_r_pin, r);
    if (is_output_capable(pins.traffic_led_y_pin)) gpio_set_level(pins.traffic_led_y_pin, y);
    if (is_output_capable(pins.traffic_led_g_pin)) gpio_set_level(pins.traffic_led_g_pin, g);
    if (is_output_capable(pins.traffic_led_b_pin)) gpio_set_level(pins.traffic_led_b_pin, b);
}
static void boat_led(bool r, bool y, bool g, bool b) {
    boat_led_r_state = r;
    boat_led_y_state = y;
    boat_led_g_state = g;
    boat_led_b_state = b;
    if (is_output_capable(pins.boat_led_r_pin)) gpio_set_level(pins.boat_led_r_pin, r);
    if (is_output_capable(pins.boat_led_y_pin)) gpio_set_level(pins.boat_led_y_pin, y);
    if (is_output_capable(pins.boat_led_g_pin)) gpio_set_level(pins.boat_led_g_pin, g);
    if (is_output_capable(pins.boat_led_b_pin)) gpio_set_level(pins.boat_led_b_pin, b);
}

// Pin management functions
esp_err_t load_pin_config(void);
esp_err_t save_pin_config(void);
bool validate_pin_config(pin_config_t* config, char* error_msg, size_t error_len);
void deinit_all_gpio(void);
esp_err_t reinit_all_systems(void);
bool is_pin_valid(gpio_num_t pin);
bool has_pin_conflicts(pin_config_t* config);

// ADC helper functions
adc_channel_t gpio_to_adc_channel(gpio_num_t gpio);
int read_hall_sensor_mv(gpio_num_t gpio_pin);
void init_adc_for_hall_sensors(void);

// Load pin configuration from NVS
esp_err_t load_pin_config(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
        pins = default_pins;
        return err;
    }

    size_t required_size = sizeof(pin_config_t);
    err = nvs_get_blob(nvs_handle, NVS_PIN_CONFIG_KEY, &pins, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "Pin config not found in NVS, using defaults");
        pins = default_pins;
        err = ESP_OK;
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load pin config: %s", esp_err_to_name(err));
        pins = default_pins;
    } else {
        ESP_LOGI(TAG, "Pin configuration loaded from NVS");
        ESP_LOGI(TAG, "Loaded pins: LED=%d, TRIG=%d, ECHO=%d", pins.led_pin, pins.ultrasonic_trig_pin, pins.ultrasonic_echo_pin);
    }

    nvs_close(nvs_handle);
    return err;
}

// Save pin configuration to NVS
esp_err_t save_pin_config(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(nvs_handle, NVS_PIN_CONFIG_KEY, &pins, sizeof(pin_config_t));
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Pin configuration saved to NVS");
        }
    } else {
        ESP_LOGE(TAG, "Failed to save pin config: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    return err;
}

// Validate GPIO pin number
bool is_pin_valid(gpio_num_t pin) {
    // ESP32 valid GPIO pins: 0, 2, 4, 5, 12-19, 21-23, 25-27, 32-39
    // Avoid: 1 (TX), 3 (RX), 6-11 (flash), 20 (not exposed), 24 (not exposed), 28-31 (not exposed)
    if (pin == GPIO_NUM_1 || pin == GPIO_NUM_3) return false;  // UART
    if (pin >= GPIO_NUM_6 && pin <= GPIO_NUM_11) return false;  // Flash
    if (pin == GPIO_NUM_20) return false;  // Not exposed
    if (pin >= GPIO_NUM_28 && pin <= GPIO_NUM_31) return false;  // Not exposed
    if (pin < GPIO_NUM_0 || pin >= GPIO_NUM_MAX) return false;  // Out of range
    return true;
}

// Check for pin conflicts in configuration
bool has_pin_conflicts(pin_config_t* config) {
    gpio_num_t pins_array[] = {
        config->led_pin,
        config->ultrasonic_trig_pin, config->ultrasonic_echo_pin,
        config->motor_enca_pin, config->motor_encb_pin,
        config->motor_in1_pin, config->motor_in2_pin, config->motor_ena_pin,
        config->bridge_boat_trig_pin, config->bridge_boat_echo_pin,
        config->bridge_boat2_trig_pin, config->bridge_boat2_echo_pin,
        config->bridge_sw_lowered_pin, config->bridge_sw_raised_pin,
        // Exclude LED pins from conflict checks to allow flexible wiring
    };
    
    int num_pins = sizeof(pins_array) / sizeof(pins_array[0]);
    
    for (int i = 0; i < num_pins; i++) {
        for (int j = i + 1; j < num_pins; j++) {
            if (pins_array[i] == pins_array[j]) {
                return true;  // Conflict found
            }
        }
    }
    return false;
}

// Validate complete pin configuration
bool validate_pin_config(pin_config_t* config, char* error_msg, size_t error_len) {
    // Check if all pins are valid
    gpio_num_t pins_to_check[] = {
        config->led_pin,
        config->ultrasonic_trig_pin, config->ultrasonic_echo_pin,
        config->motor_enca_pin, config->motor_encb_pin,
        config->motor_in1_pin, config->motor_in2_pin, config->motor_ena_pin,
        config->bridge_boat_trig_pin, config->bridge_boat_echo_pin,
        config->bridge_boat2_trig_pin, config->bridge_boat2_echo_pin,
        config->bridge_sw_lowered_pin, config->bridge_sw_raised_pin,
        config->traffic_led_r_pin, config->traffic_led_y_pin, config->traffic_led_g_pin, config->traffic_led_b_pin,
        config->boat_led_r_pin, config->boat_led_y_pin, config->boat_led_g_pin, config->boat_led_b_pin
    };
    
    int num_pins = sizeof(pins_to_check) / sizeof(pins_to_check[0]);
    
    for (int i = 0; i < num_pins; i++) {
        if (!is_pin_valid(pins_to_check[i])) {
            snprintf(error_msg, error_len, "Invalid GPIO pin: %d", pins_to_check[i]);
            return false;
        }
    }
    
    // Check for conflicts (LED pins are excluded from conflict checks)
    if (has_pin_conflicts(config)) {
        snprintf(error_msg, error_len, "Pin conflict detected - same pin assigned to multiple functions");
        return false;
    }
    
    return true;
}

// Convert GPIO pin to ADC channel
adc_channel_t gpio_to_adc_channel(gpio_num_t gpio) {
    switch (gpio) {
        case GPIO_NUM_36: return ADC_CHANNEL_0;
        case GPIO_NUM_37: return ADC_CHANNEL_1;
        case GPIO_NUM_38: return ADC_CHANNEL_2;
        case GPIO_NUM_39: return ADC_CHANNEL_3;
        case GPIO_NUM_32: return ADC_CHANNEL_4;
        case GPIO_NUM_33: return ADC_CHANNEL_5;
        case GPIO_NUM_34: return ADC_CHANNEL_6;
        case GPIO_NUM_35: return ADC_CHANNEL_7;
        default: return -1; // Invalid channel
    }
}

// Initialize ADC for hall effect sensors
void init_adc_for_hall_sensors(void) {
    // Initialize ADC1
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return;
    }

    // Configure hall sensor channels if they're valid ADC pins
    adc_channel_t lowered_channel = gpio_to_adc_channel(pins.bridge_sw_lowered_pin);
    adc_channel_t raised_channel = gpio_to_adc_channel(pins.bridge_sw_raised_pin);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,  // Updated from deprecated ADC_ATTEN_DB_11
    };

    if (lowered_channel != -1) {
        ret = adc_oneshot_config_channel(adc1_handle, lowered_channel, &config);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Configured ADC for hall sensor lowered on GPIO%d", pins.bridge_sw_lowered_pin);
        } else {
            ESP_LOGE(TAG, "Failed to configure ADC channel for GPIO%d: %s", pins.bridge_sw_lowered_pin, esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "GPIO%d is not a valid ADC pin for hall sensor lowered", pins.bridge_sw_lowered_pin);
    }

    if (raised_channel != -1) {
        ret = adc_oneshot_config_channel(adc1_handle, raised_channel, &config);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Configured ADC for hall sensor raised on GPIO%d", pins.bridge_sw_raised_pin);
        } else {
            ESP_LOGE(TAG, "Failed to configure ADC channel for GPIO%d: %s", pins.bridge_sw_raised_pin, esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "GPIO%d is not a valid ADC pin for hall sensor raised", pins.bridge_sw_raised_pin);
    }

    // Initialize calibration (simplified - may not be available on all ESP32 variants)
    adc1_cali_handle = NULL;
    ESP_LOGI(TAG, "ADC initialized (calibration not implemented in this version)");
}

// Read hall effect sensor value in millivolts
int read_hall_sensor_mv(gpio_num_t gpio_pin) {
    adc_channel_t channel = gpio_to_adc_channel(gpio_pin);
    if (channel == -1 || adc1_handle == NULL) {
        // If not an ADC pin or ADC not initialized, fall back to digital reading
        int digital_val = gpio_get_level(gpio_pin);
        return digital_val ? 3300 : 0; // Return 3.3V or 0V equivalent
    }

    // Multisampling
    uint32_t adc_reading = 0;
    int successful_reads = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        int raw_value;
        esp_err_t ret = adc_oneshot_read(adc1_handle, channel, &raw_value);
        if (ret == ESP_OK) {
            adc_reading += raw_value;
            successful_reads++;
        } else {
            ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(ret));
        }
    }
    
    if (successful_reads == 0) {
        ESP_LOGE(TAG, "All ADC reads failed for GPIO%d", gpio_pin);
        return 0;
    }
    
    adc_reading /= successful_reads;
    
    // Log raw ADC value for debugging
    // ESP_LOGI(TAG, "GPIO%d: Raw ADC = %lu (from %d samples)", gpio_pin, adc_reading, successful_reads);

    // Convert to voltage if calibration is available
    if (adc1_cali_handle != NULL) {
        int voltage;
        esp_err_t ret = adc_cali_raw_to_voltage(adc1_cali_handle, adc_reading, &voltage);
        if (ret == ESP_OK) {
            return voltage;
        }
    }

    // Fallback: simple linear conversion for ADC_ATTEN_DB_11 (0-3.1V range)
    // int voltage = (adc_reading * 3100) / 4095;  // Changed from 3300 to 3100 for DB_11 attenuation
    int voltage = adc_reading;
    // ESP_LOGI(TAG, "GPIO%d: Converted voltage = %d mV", gpio_pin, voltage);
    return voltage;
}

// Median filter function for noise reduction
float median_filter(float arr[], int n) {
    // Simple insertion sort for small arrays
    float temp[n];
    for (int i = 0; i < n; i++) {
        temp[i] = arr[i];
    }
    
    for (int i = 1; i < n; i++) {
        float key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }
    return temp[n / 2];
}

// Generic filtered distance for any trig/echo pair using median of quick samples
float get_filtered_distance_for(gpio_num_t trig_pin, gpio_num_t echo_pin) {
    float samples[MEDIAN_SAMPLES];
    int valid_count = 0;

    for (int i = 0; i < MEDIAN_SAMPLES; i++) {
        float r = measure_distance(trig_pin, echo_pin);
        if (r >= 0.0f && r < 1000.0f) {
            samples[valid_count++] = r;
        }
        // short delay between sub-samples
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    if (valid_count == 0) {
        return -1.0; // timeout/no valid echoes
    }

    return median_filter(samples, valid_count);
}

// Compute median absolute deviation (MAD) for window
static float compute_mad(float arr[], int n, float med) {
    if (n <= 0) return 0.0f;
    float devs[n];
    for (int i = 0; i < n; i++) devs[i] = fabsf(arr[i] - med);
    return median_filter(devs, n);
}

// Hampel filter applied using history buffer (history contains past readings, newest at history_index-1)
float apply_hampel_history(float value, float *history_buf, int history_count, int history_index) {
    // Build window of up to HAMPEL_WINDOW previous valid values (exclude current 'value')
    float window[HAMPEL_WINDOW];
    int found = 0;
    int idx = history_index - 1;
    if (idx < 0 && history_count > 0) idx += MAX_SENSOR_READINGS;

    for (int i = 0; i < history_count && found < HAMPEL_WINDOW; i++) {
        int pos = (idx - i + MAX_SENSOR_READINGS) % MAX_SENSOR_READINGS;
        float v = history_buf[pos];
        if (v >= 0.0f && v < 1000.0f) {
            window[found++] = v;
        }
    }

    if (found == 0) return value; // no history to compare

    float med = median_filter(window, found);
    float mad = compute_mad(window, found, med);

    // If mad is zero (all values identical), fall back to small epsilon
    if (mad <= 0.0f) mad = 0.0001f;

    float threshold = HAMPEL_K * mad;
    if (fabsf(value - med) > threshold) {
        // Outlier — replace with median (conservative)
        return med;
    }
    return value;
}

// Get filtered distance reading using median filter (for boat sensor)
float get_filtered_distance() {
    float samples[MEDIAN_SAMPLES];
    int valid_count = 0;

    // Collect a few samples, but ignore timeout (-1) readings
    for (int i = 0; i < MEDIAN_SAMPLES; i++) {
        float r = measure_distance(pins.bridge_boat_trig_pin, pins.bridge_boat_echo_pin);
        if (r >= 0.0f && r < 1000.0f) { // accept only reasonable positive readings
            samples[valid_count++] = r;
        }
        vTaskDelay(5 / portTICK_PERIOD_MS); // Reduced delay for faster sampling
    }

    if (valid_count == 0) {
        // No valid echoes received in this sampling window
        return -1.0;
    }

    // If we collected fewer than MEDIAN_SAMPLES, median_filter expects n entries
    // so call it with the actual valid_count
    return median_filter(samples, valid_count);
}

// Fast single reading for calibration (no median filtering, use boat sensor)
float get_calibration_reading() {
    float distance = measure_distance(pins.bridge_boat_trig_pin, pins.bridge_boat_echo_pin);
    if (distance < 0) {
        // Return a more reasonable timeout value for calibration
        // Don't use 500 as it's outside typical indoor range
        return -1.0; // Keep timeout as -1 for proper handling
    }
    return distance;
}

// Validate calibration reading (reject timeouts and outliers)
bool is_valid_calibration_reading(float distance) {
    return (distance > 0 && distance < 400.0); // Reasonable indoor range
}

// Add reading to live data buffer (boat sensor)
void add_to_data_buffer(float distance) {
    sensor_readings[reading_index] = distance;
    reading_timestamps[reading_index] = esp_timer_get_time() / 1000; // milliseconds
    
    reading_index = (reading_index + 1) % MAX_SENSOR_READINGS;
    if (readings_count < MAX_SENSOR_READINGS) {
        readings_count++;
    }
}

// Add reading to live data buffer (boat2 sensor)
void add_to_boat2_data_buffer(float distance) {
    boat2_sensor_readings[boat2_reading_index] = distance;
    boat2_reading_timestamps[boat2_reading_index] = esp_timer_get_time() / 1000; // milliseconds
    
    boat2_reading_index = (boat2_reading_index + 1) % MAX_SENSOR_READINGS;
    if (boat2_readings_count < MAX_SENSOR_READINGS) {
        boat2_readings_count++;
    }
}

// Start ultrasonic calibration
void start_calibration() {
    ESP_LOGI(TAG, "Starting ultrasonic calibration - keep area clear of boats");
    calib_state = CALIB_IN_PROGRESS;
    calib_sample_count = 0;
    baseline_mean = 0.0;
    baseline_stddev = 0.0;
    consecutive_detections = 0;
    consecutive_clear = 0;
    boat_detected = false;
}

// Process calibration sample
void process_calibration_sample(float distance) {
    if (calib_state != CALIB_IN_PROGRESS) return;
    
    // Only accept valid readings for calibration
    if (!is_valid_calibration_reading(distance)) {
        ESP_LOGW(TAG, "Invalid calibration reading: %.2f cm (timeout or out of range)", distance);
        return; // Skip this sample
    }
    
    calib_samples[calib_sample_count] = distance;
    calib_sample_count++;
    
    // Log progress every 10 samples to reduce log spam
    if (calib_sample_count % 10 == 0 || calib_sample_count == BASELINE_SAMPLES) {
        ESP_LOGI(TAG, "Calibration progress: %d/%d samples (%.1f%%) - Latest: %.2f cm", 
                 calib_sample_count, BASELINE_SAMPLES, 
                 (calib_sample_count * 100.0) / BASELINE_SAMPLES, distance);
    }
    
    if (calib_sample_count >= BASELINE_SAMPLES) {
        // Calculate mean
        float sum = 0.0;
        for (int i = 0; i < BASELINE_SAMPLES; i++) {
            sum += calib_samples[i];
        }
        baseline_mean = sum / BASELINE_SAMPLES;
        
        // Calculate standard deviation
        float variance = 0.0;
        for (int i = 0; i < BASELINE_SAMPLES; i++) {
            float diff = calib_samples[i] - baseline_mean;
            variance += diff * diff;
        }
        variance /= BASELINE_SAMPLES;
        baseline_stddev = sqrt(variance);
        
        // Validate baseline is reasonable
        if (baseline_mean < 10.0 || baseline_mean > 300.0) {
            ESP_LOGW(TAG, "⚠️ Suspicious baseline mean: %.2f cm - check sensor setup", baseline_mean);
        }
        
        calib_state = CALIB_COMPLETE;
        
        ESP_LOGI(TAG, "✅ Calibration complete!");
        ESP_LOGI(TAG, "  📊 Baseline mean: %.2f cm", baseline_mean);
        ESP_LOGI(TAG, "  📈 Baseline stddev: %.2f cm", baseline_stddev);
        ESP_LOGI(TAG, "  🎯 Detection threshold: %.2f cm closer than baseline", THRESHOLD_CM);
        ESP_LOGI(TAG, "  ⚡ Requires %d consecutive detections", REQUIRED_CONSECUTIVE);
    }
}

// Enhanced boat detection using baseline calibration
bool detect_boat_calibrated(float distance) {
    if (calib_state != CALIB_COMPLETE) {
        // Fallback to simple threshold if not calibrated
        return distance <= BOAT_PRESENT_DIST_CM;
    }
    
    float delta = baseline_mean - distance; // Positive if something is closer
    bool candidate = (delta >= THRESHOLD_CM);
    
    if (candidate) {
        consecutive_detections++;
        consecutive_clear = 0;
    } else {
        consecutive_detections = 0;
        consecutive_clear++;
    }
    
    // Require consecutive detections to trigger
    if (!boat_detected && consecutive_detections >= REQUIRED_CONSECUTIVE) {
        boat_detected = true;
        ESP_LOGI(TAG, "BOAT DETECTED: distance=%.2f, delta=%.2f, consecutive=%d", 
                 distance, delta, consecutive_detections);
    }
    
    // Require consecutive clear readings to reset
    if (boat_detected && consecutive_clear >= RESET_CONSECUTIVE) {
        boat_detected = false;
        ESP_LOGI(TAG, "BOAT CLEARED: distance=%.2f, consecutive_clear=%d", 
                 distance, consecutive_clear);
    }
    
    return boat_detected;
}

// Helper that determines if a reading is a candidate for 'boat present'
static bool is_boat_candidate(float distance) {
    if (distance < 0) return false; // timeouts aren't candidates
    if (calib_state == CALIB_COMPLETE) {
        float delta = baseline_mean - distance; // Positive if something is closer
        return (delta >= THRESHOLD_CM);
    } else {
        // Fallback unsophisticated threshold
        return distance <= BOAT_PRESENT_DIST_CM;
    }
}

// === BOAT2 SENSOR CALIBRATION FUNCTIONS ===

// Start boat2 sensor calibration
void start_boat2_calibration() {
    ESP_LOGI(TAG, "Starting boat2 sensor calibration - keep area clear of boats");
    boat2_calib_state = CALIB_IN_PROGRESS;
    boat2_calib_sample_count = 0;
    boat2_baseline_mean = 0.0;
    boat2_baseline_stddev = 0.0;
    boat2_consecutive_detections = 0;
    boat2_consecutive_clear = 0;
    boat2_detected = false;
}

// Fast single reading for boat2 calibration
float get_boat2_calibration_reading() {
    float distance = measure_distance(pins.bridge_boat2_trig_pin, pins.bridge_boat2_echo_pin);
    if (distance < 0) {
        return -1.0;
    }
    return distance;
}

// Process boat2 calibration sample
void process_boat2_calibration_sample(float distance) {
    if (boat2_calib_state != CALIB_IN_PROGRESS) return;
    
    // Only accept valid readings for calibration
    if (!is_valid_calibration_reading(distance)) {
        return;
    }
    
    boat2_calib_samples[boat2_calib_sample_count] = distance;
    boat2_calib_sample_count++;
    
    // Log progress every 10 samples to reduce log spam
    if (boat2_calib_sample_count % 10 == 0 || boat2_calib_sample_count == BASELINE_SAMPLES) {
        ESP_LOGI(TAG, "Boat2 calibration progress: %d/%d samples", boat2_calib_sample_count, BASELINE_SAMPLES);
    }
    
    if (boat2_calib_sample_count >= BASELINE_SAMPLES) {
        // Calculate mean
        double sum = 0.0;
        for (int i = 0; i < BASELINE_SAMPLES; i++) {
            sum += boat2_calib_samples[i];
        }
        boat2_baseline_mean = sum / BASELINE_SAMPLES;
        
        // Calculate standard deviation
        double variance_sum = 0.0;
        for (int i = 0; i < BASELINE_SAMPLES; i++) {
            float diff = boat2_calib_samples[i] - boat2_baseline_mean;
            variance_sum += diff * diff;
        }
        boat2_baseline_stddev = sqrt(variance_sum / BASELINE_SAMPLES);
        
        boat2_calib_state = CALIB_COMPLETE;
        ESP_LOGI(TAG, "Boat2 calibration COMPLETE:");
        ESP_LOGI(TAG, "  Baseline Mean: %.2f cm", boat2_baseline_mean);
        ESP_LOGI(TAG, "  Std Deviation: %.2f cm", boat2_baseline_stddev);
        ESP_LOGI(TAG, "  Detection threshold: %.1f cm closer than baseline", THRESHOLD_CM);
    }
}

// Enhanced boat2 detection using baseline calibration
bool detect_boat2_calibrated(float distance) {
    if (boat2_calib_state != CALIB_COMPLETE) {
        // Fallback to simple threshold if not calibrated
        return distance <= BOAT_PRESENT_DIST_CM; // reuse same numeric dist for on-deck checks
    }
    
    float delta = boat2_baseline_mean - distance; // Positive if something is closer
    bool candidate = (delta >= THRESHOLD_CM);
    
    if (candidate) {
        boat2_consecutive_detections++;
        boat2_consecutive_clear = 0;
    } else {
        boat2_consecutive_detections = 0;
        boat2_consecutive_clear++;
    }
    
    // Require consecutive detections to trigger
    if (!boat2_detected && boat2_consecutive_detections >= REQUIRED_CONSECUTIVE) {
        boat2_detected = true;
        ESP_LOGI(TAG, "BOAT2 DETECTED: distance=%.2f, delta=%.2f, consecutive=%d", 
                 distance, delta, boat2_consecutive_detections);
    }
    
    // Require consecutive clear readings to reset
    if (boat2_detected && boat2_consecutive_clear >= RESET_CONSECUTIVE) {
        boat2_detected = false;
        ESP_LOGI(TAG, "BOAT2 CLEARED: distance=%.2f, consecutive_clear=%d", 
                 distance, boat2_consecutive_clear);
    }
    
    return boat2_detected;
}

// Ultrasonic sensor measurement function
float measure_distance(gpio_num_t trig_pin, gpio_num_t echo_pin) {
    // Robust implementation based on busy-wait timing (microsecond accuracy)
    ESP_LOGD(TAG, "Ultrasonic: TRIG=%d, ECHO=%d", trig_pin, echo_pin);

    // If echo line is HIGH before we start, wait briefly for it to go LOW.
    // Some modules or wiring can leave the line high; give it a short chance to clear.
    const int64_t PRE_TRIGGER_WAIT_US = 200000; // 200 ms
    int64_t pre_start = esp_timer_get_time();
    while (gpio_get_level(echo_pin) != 0) {
        if ((esp_timer_get_time() - pre_start) > PRE_TRIGGER_WAIT_US) {
            ESP_LOGW(TAG, "Ultrasonic echo line stuck HIGH before trigger (pin %d)", echo_pin);
            return -1.0;
        }
        // small busy-wait to avoid tight spin
        int64_t _tmp = esp_timer_get_time();
        while (esp_timer_get_time() - _tmp < 50) { }
    }

    // Send trigger pulse (10 us)
    gpio_set_level(trig_pin, 0);
    int64_t ttmp = esp_timer_get_time();
    while (esp_timer_get_time() - ttmp < 2) { }
    gpio_set_level(trig_pin, 1);
    int64_t t0 = esp_timer_get_time();
    while (esp_timer_get_time() - t0 < 10) { }
    gpio_set_level(trig_pin, 0);

    // Wait for echo rising edge
    int64_t start_wait = esp_timer_get_time();
    const int64_t START_TIMEOUT_US = 30000; // 30 ms
    while (gpio_get_level(echo_pin) == 0) {
        if ((esp_timer_get_time() - start_wait) > START_TIMEOUT_US) {
            int64_t now_us = esp_timer_get_time();
            if (now_us - last_ultrasonic_warn_time_us > (int64_t)ULTRASONIC_WARN_RATE_MS * 1000) {
                ESP_LOGW(TAG, "Ultrasonic timeout waiting for echo start (pins %d,%d)", trig_pin, echo_pin);
                last_ultrasonic_warn_time_us = now_us;
            }
            return -1.0;
        }
    }
    int64_t echo_start = esp_timer_get_time();

    // Wait for echo falling edge
    const int64_t END_TIMEOUT_US = 30000; // 30 ms
    while (gpio_get_level(echo_pin) == 1) {
        if ((esp_timer_get_time() - echo_start) > END_TIMEOUT_US) {
            int64_t now_us = esp_timer_get_time();
            if (now_us - last_ultrasonic_warn_time_us > (int64_t)ULTRASONIC_WARN_RATE_MS * 1000) {
                ESP_LOGW(TAG, "Ultrasonic timeout waiting for echo end (pins %d,%d)", trig_pin, echo_pin);
                last_ultrasonic_warn_time_us = now_us;
            }
            return -1.0;
        }
    }
    int64_t echo_end = esp_timer_get_time();

    int64_t pulse_duration = echo_end - echo_start; // microseconds
    float distance = (pulse_duration * SOUND_SPEED) / 2.0f; // cm
    ESP_LOGD(TAG, "Pulse duration: %lld us, Distance: %.2f cm (pins %d,%d)", (long long)pulse_duration, distance, trig_pin, echo_pin);
    return distance;
}

// Ultrasonic measurement task
void ultrasonic_task(void *pvParameters) {
    while (1) {
        float distance;
        float boat2_distance;
        // === BOAT SENSOR ===
        // Use faster sampling during calibration
        uint32_t now_ms = esp_timer_get_time() / 1000;
        if (calib_state == CALIB_IN_PROGRESS) {
            distance = get_calibration_reading();

            // For display purposes, show timeout as "no reading"
            if (distance < 0) {
                current_distance = -1.0; // Indicate timeout
                boat_timeout_streak++;
            } else {
                current_distance = distance;
                // Add to data buffer for live graphing (only valid readings)
                add_to_data_buffer(distance);
                boat_timeout_streak = 0;
            }

            // If repeated timeouts, enter cooldown to avoid spam
            if (boat_timeout_streak >= ULTRASONIC_TIMEOUT_STREAK) {
                boat_cooldown_until_ms = now_ms + ULTRASONIC_COOLDOWN_MS;
                ESP_LOGW(TAG, "Boat sensor entering cooldown for %d ms after %d timeouts", ULTRASONIC_COOLDOWN_MS, boat_timeout_streak);
                boat_timeout_streak = 0; // reset streak after scheduling cooldown
            }

            // Process calibration (only valid readings)
            process_calibration_sample(distance);

            vTaskDelay(CALIB_SAMPLE_DELAY_MS / portTICK_PERIOD_MS);
        } else {
            // If currently in cooldown for boat sensor, skip sampling
            if (now_ms < boat_cooldown_until_ms) {
                current_distance = -1.0;
            } else {
                distance = get_filtered_distance();
                current_distance = distance;

                if (distance >= 0.0f) {
                    add_to_data_buffer(distance);
                    detect_boat_calibrated(distance);
                    boat_timeout_streak = 0;
                    // Update detection meter
                    uint32_t now_meter_ms = esp_timer_get_time() / 1000;
                    if (is_boat_candidate(distance)) {
                        boat_detection_meter += METER_INCREMENT;
                        if (boat_detection_meter > METER_MAX) boat_detection_meter = METER_MAX;
                        last_meter_update_ms = now_meter_ms;
                    } else {
                        // decay based on elapsed time
                        if (last_meter_update_ms == 0) last_meter_update_ms = now_meter_ms;
                        uint32_t elapsed_ms = now_meter_ms - last_meter_update_ms;
                        if (elapsed_ms > 0) {
                            int decay = (METER_DECAY_PER_SEC * (int)elapsed_ms) / 1000;
                            if (decay > 0) {
                                boat_detection_meter -= decay;
                                if (boat_detection_meter < 0) boat_detection_meter = 0;
                                last_meter_update_ms = now_meter_ms;
                            }
                        }
                    }
                } else {
                    current_distance = -1.0;
                    boat_timeout_streak++;
                    if (boat_timeout_streak >= ULTRASONIC_TIMEOUT_STREAK) {
                        boat_cooldown_until_ms = now_ms + ULTRASONIC_COOLDOWN_MS;
                        ESP_LOGW(TAG, "Boat sensor entering cooldown for %d ms after %d timeouts", ULTRASONIC_COOLDOWN_MS, boat_timeout_streak);
                        boat_timeout_streak = 0;
                    }
                }
            }

            vTaskDelay(SAMPLE_INTERVAL_MS / portTICK_PERIOD_MS);
        }
        
        // === BOAT2 SENSOR ===
        // Handle boat2 sensor calibration and detection
        if (boat2_calib_state == CALIB_IN_PROGRESS) {
                boat2_distance = get_boat2_calibration_reading();

            if (boat2_distance > 0) {
                    add_to_boat2_data_buffer(boat2_distance);
                boat2_timeout_streak = 0;
            } else {
                boat2_timeout_streak++;
            }

            // Process calibration (only valid readings)
            process_boat2_calibration_sample(boat2_distance);
        } else {
            // If currently in cooldown for boat2 sensor, skip sampling
            uint32_t now_ms2 = esp_timer_get_time() / 1000;
            if (now_ms2 < boat2_cooldown_until_ms) {
                // Skipped due to cooldown
            } else {
                // Regular boat2 sensor reading - use same filtering pipeline as boat1
                boat2_distance = get_filtered_distance_for(pins.bridge_boat2_trig_pin, pins.bridge_boat2_echo_pin);

                if (boat2_distance >= 0.0f) {
                    add_to_boat2_data_buffer(boat2_distance);
                    // Update detection state (only when not calibrating)
                    detect_boat2_calibrated(boat2_distance);
                    boat2_timeout_streak = 0;

                    // Update boat2 detection meter using the same logic as boat1
                    uint32_t now_meter2_ms = esp_timer_get_time() / 1000;
                    bool candidate2;
                    if (boat2_calib_state == CALIB_COMPLETE) {
                        float delta2 = boat2_baseline_mean - boat2_distance;
                        candidate2 = (delta2 >= THRESHOLD_CM);
                    } else {
                        candidate2 = (boat2_distance <= BOAT_PRESENT_DIST_CM);
                    }

                    if (candidate2) {
                        boat2_detection_meter += METER_INCREMENT;
                        if (boat2_detection_meter > METER_MAX) boat2_detection_meter = METER_MAX;
                        last_boat2_meter_update_ms = now_meter2_ms;
                    } else {
                        if (last_boat2_meter_update_ms == 0) last_boat2_meter_update_ms = now_meter2_ms;
                        uint32_t elapsed_ms2 = now_meter2_ms - last_boat2_meter_update_ms;
                        if (elapsed_ms2 > 0) {
                            int decay2 = (METER_DECAY_PER_SEC * (int)elapsed_ms2) / 1000;
                            if (decay2 > 0) {
                                boat2_detection_meter -= decay2;
                                if (boat2_detection_meter < 0) boat2_detection_meter = 0;
                                last_boat2_meter_update_ms = now_meter2_ms;
                            }
                        }
                    }

                } else {
                    boat2_timeout_streak++;
                    if (boat2_timeout_streak >= ULTRASONIC_TIMEOUT_STREAK) {
                        boat2_cooldown_until_ms = now_ms2 + ULTRASONIC_COOLDOWN_MS;
                        ESP_LOGW(TAG, "Boat2 sensor entering cooldown for %d ms after %d timeouts", ULTRASONIC_COOLDOWN_MS, boat2_timeout_streak);
                        boat2_timeout_streak = 0;
                    }
                }
            }
        }
    }
}

// Forward declarations for motor control functions
void init_motor_pwm(void);
void motor_set_speed(int speed_percent);
void motor_set_direction(bool forward);
void motor_start(void);
void motor_stop(void);
void motor_brake(void);

// PWM initialization for motor speed control
void init_motor_pwm(void) {
    // Configure PWM timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_config);

    // Configure PWM channel
    ledc_channel_config_t channel_config = {
        .gpio_num = pins.motor_ena_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_config);

    ESP_LOGI(TAG, "Motor PWM initialized on GPIO %d", pins.motor_ena_pin);
}

// Motor control functions
void motor_set_speed(int speed_percent) {
    if (speed_percent < 0) speed_percent = 0;
    if (speed_percent > 100) speed_percent = 100;

    // Convert percentage to PWM duty cycle (0-255)
    uint32_t duty = (speed_percent * 255) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);

    // Only log if speed actually changes
    if (motor_speed != speed_percent) {
        ESP_LOGI(TAG, "Motor speed set to %d%% (PWM duty: %lu)", speed_percent, (unsigned long)duty);
    }
    motor_speed = speed_percent;
}

void motor_set_direction(bool forward) {
    motor_direction = forward;

    if (forward) {
        // Forward: IN1 = HIGH, IN2 = LOW
        gpio_set_level(pins.motor_in1_pin, 1);
        gpio_set_level(pins.motor_in2_pin, 0);
        ESP_LOGI(TAG, "Motor direction set to FORWARD");
    } else {
        // Reverse: IN1 = LOW, IN2 = HIGH
        gpio_set_level(pins.motor_in1_pin, 0);
        gpio_set_level(pins.motor_in2_pin, 1);
        ESP_LOGI(TAG, "Motor direction set to REVERSE");
    }
}

void motor_start(void) {
    motor_running = true;
    motor_set_speed(motor_speed);  // Apply current speed setting
    ESP_LOGI(TAG, "Motor started");
}

void motor_stop(void) {
    if (motor_running) {
        motor_running = false;
        motor_set_speed(0);  // Set speed to 0
        // Set both inputs to LOW for coasting stop
        gpio_set_level(pins.motor_in1_pin, 0);
        gpio_set_level(pins.motor_in2_pin, 0);
        ESP_LOGI(TAG, "Motor stopped");
    } else {
        motor_set_speed(0);
        gpio_set_level(pins.motor_in1_pin, 0);
        gpio_set_level(pins.motor_in2_pin, 0);
    }
}

void motor_brake(void) {
    motor_running = false;
    motor_set_speed(0);  // Set speed to 0
    // Set both inputs to HIGH for active braking
    gpio_set_level(pins.motor_in1_pin, 1);
    gpio_set_level(pins.motor_in2_pin, 1);
    ESP_LOGI(TAG, "Motor braked");
}

// Motor encoder interrupt handler
static void IRAM_ATTR motor_encoder_isr(void* arg) {
    uint8_t encoder_pinA_value = gpio_get_level(pins.motor_enca_pin);
    if ((encoder_pinA_last == 0) && encoder_pinA_value == 1) {
        uint8_t b = gpio_get_level(pins.motor_encb_pin);
        encoder_direction = (b == 1);  // true for forward, false for reverse
        if (encoder_direction) {
            encoder_pulses++;
        } else {
            encoder_pulses--;
        }
    }
    encoder_pinA_last = encoder_pinA_value;
}

// Motor control task
void motor_task(void *pvParameters) {
    while (1) {
        // Update motor control based on current state
        if (motor_running && motor_speed > 0) {
            // Motor is running - ensure PWM is set correctly
            motor_set_speed(motor_speed);
        } else if (!motor_running) {
            // Motor is stopped - ensure it's actually stopped
            // Only call motor_stop if not already stopped
            if (motor_speed != 0 || gpio_get_level(pins.motor_in1_pin) != 0 || gpio_get_level(pins.motor_in2_pin) != 0) {
                motor_stop();
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Update every 100ms
    }
}

// Bridge LED helpers (two separate systems)
static void traffic_led_green() { traffic_led(false, false, true, false); }
static void traffic_led_yellow() { traffic_led(false, true, false, false); }
static void traffic_led_red() { traffic_led(true, false, false, false); }
static void traffic_led_off() { traffic_led(false, false, false, false); }

static void boat_led_green() { boat_led(false, false, true, false); }
static void boat_led_yellow() { boat_led(false, true, false, false); }
static void boat_led_red() { boat_led(true, false, false, false); }
static void boat_led_off() { boat_led(false, false, false, false); }

bool bridge_switch_triggered(gpio_num_t pin) {
    // Use adjustable threshold for hall effect sensor
    int voltage_mv = read_hall_sensor_mv(pin);
    return voltage_mv < hall_effect_threshold;
}

float bridge_ping_cm(gpio_num_t trig, gpio_num_t echo) {
    float distance = measure_distance(trig, echo);
    if (distance < 0) {
        return 9999.0; // Return large value for timeout (for backward compatibility)
    }
    return distance;
}

bool bridge_car_on_bridge() {
    // Car ultrasonic has been removed and will be replaced by a thin-film pressure sensor (XC3738) later.
    // This function is a placeholder for the future weight/pressure-based car detection logic.
    // TODO: Integrate XC3738 thin-film pressure sensor here. Typical implementation notes:
    //  - Read sensor via ADC or I2C depending on breakout (XC3738 may require an op-amp / interface)
    //  - Apply a short moving-average and hysteresis to avoid chatter
    //  - Persist calibrated threshold to NVS so it survives reboots
    //  - Expose an API to recalibrate the threshold and to enable/disable car detection
    // For now return false so the bridge logic treats the deck as clear of cars.
    (void)pins; // keep compiler happy about unused symbols
    return false;
}

bool bridge_boat_present() {
    // Use calibrated detection if available, otherwise fallback to simple threshold
    if (calib_state == CALIB_COMPLETE) {
        return boat_detected;
    } else {
        return bridge_ping_cm(pins.bridge_boat_trig_pin, pins.bridge_boat_echo_pin) <= BOAT_PRESENT_DIST_CM;
    }
}

// Presence check for the second boat ultrasonic
bool bridge_boat2_present() {
    if (boat2_calib_state == CALIB_COMPLETE) {
        return boat2_detected;
    } else {
        return bridge_ping_cm(pins.bridge_boat2_trig_pin, pins.bridge_boat2_echo_pin) <= BOAT_PRESENT_DIST_CM;
    }
}

const char* bridge_state_name(bridge_state_t state) {
    switch (state) {
        case BRIDGE_SELFTEST: return "SELFTEST";
        case BRIDGE_ARMED: return "ARMED";
        case BRIDGE_WARN_TRAFFIC: return "WARN_TRAFFIC";
        case BRIDGE_WAIT_STOP: return "WAIT_STOP";
        case BRIDGE_CHECK_CLEAR: return "CHECK_CLEAR";
        case BRIDGE_RAISING: return "RAISING";
        case BRIDGE_TOP_REACHED: return "TOP_REACHED";
        case BRIDGE_PRE_LOWER_WARN: return "PRE_LOWER_WARN";
        case BRIDGE_LOWERING: return "LOWERING";
        case BRIDGE_AFTER_LOWERED: return "AFTER_LOWERED";
        case BRIDGE_OPEN_TRAFFIC: return "OPEN_TRAFFIC";
        default: return "UNKNOWN";
    }
}

void bridge_set_state(bridge_state_t new_state) {
    ESP_LOGI(TAG, "Bridge state: %s -> %s", 
             bridge_state_name(bridge_state), 
             bridge_state_name(new_state));
    bridge_state = new_state;
    bridge_state_start = esp_timer_get_time() / 1000;

    if (new_state == BRIDGE_TOP_REACHED) {
        bridge_top_saw_gap = false;
        bridge_top_enter_time = bridge_state_start;
    }
}

void bridge_task(void *pvParameters) {
    uint32_t boat_blink_last = 0;
    bool boat_blink_on = false;
    while (1) {
        if (bridge_manual_override) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        uint32_t current_time = esp_timer_get_time() / 1000;

        switch (bridge_state) {
            case BRIDGE_SELFTEST: {
                bool boat_ok = (bridge_ping_cm(pins.bridge_boat_trig_pin, pins.bridge_boat_echo_pin) < 9999);
                bool boat2_ok = (bridge_ping_cm(pins.bridge_boat2_trig_pin, pins.bridge_boat2_echo_pin) < 9999);

                // During selftest: show traffic yellow steady; boat red
                traffic_led_yellow();
                boat_led_red();

                if (boat_ok && boat2_ok && (current_time - bridge_state_start) > 1200) {
                    ESP_LOGI(TAG, "Bridge self-test OK. System ARMED.");
                    bridge_set_state(BRIDGE_ARMED);
                }
                break;
            }

            case BRIDGE_ARMED:
                {
                    // Check both boat sensors (primary and boat2). The sensor that first sees the boat
                    // is recorded in boat_initial_sensor to guide the TOP->pre-lower sequencing.
                    if (bridge_boat_present()) {
                        boat_initial_sensor = 1;
                        // Prewarning handled in WARN_TRAFFIC; set initial outputs here
                        // Traffic handled in WARN, boat stays RED until raising
                        boat_led_red();
                        ESP_LOGI(TAG, "Boat detected by sensor1: WARN_TRAFFIC.");
                        bridge_set_state(BRIDGE_WARN_TRAFFIC);
                    } else if (bridge_boat2_present()) {
                        boat_initial_sensor = 2;
                        boat_led_red();
                        ESP_LOGI(TAG, "Boat detected by sensor2: WARN_TRAFFIC.");
                        bridge_set_state(BRIDGE_WARN_TRAFFIC);
                    }
                }
                break;

            case BRIDGE_WARN_TRAFFIC: {
                uint32_t elapsed = current_time - bridge_state_start;
                if (elapsed < 3000) {
                    traffic_led_yellow();
                } else {
                    traffic_led_red();
                }
                boat_led_red();

                if (elapsed >= T_WARN_TRAFFIC) {
                    ESP_LOGI(TAG, "Traffic RED. Preparing to raise.");
                    bridge_set_state(BRIDGE_WAIT_STOP);
                }
                break;
            }

            case BRIDGE_WAIT_STOP:
                traffic_led_red();
                boat_led_red();
                if ((current_time - bridge_state_start) >= T_AFTER_STOP) {
                    ESP_LOGI(TAG, "Checking deck clear (cars/peds).");
                    bridge_set_state(BRIDGE_CHECK_CLEAR);
                }
                break;

            case BRIDGE_CHECK_CLEAR:
                traffic_led_red();
                boat_led_red();
                if (!bridge_car_on_bridge()) {
                    ESP_LOGI(TAG, "Deck clear. Raising bridge.");
                    bridge_set_state(BRIDGE_RAISING);
                }
                break;

            case BRIDGE_RAISING:
                if (bridge_switch_triggered(pins.bridge_sw_raised_pin)) {
                    traffic_led_red();
                    boat_led_green();
                    ESP_LOGI(TAG, "Top limit reached. Boat GREEN.");
                    bridge_set_state(BRIDGE_TOP_REACHED);
                    motor_set_speed(0);
                    motor_stop();
                    break;
                }
                motor_set_direction(false); // Raise
                motor_set_speed(60);
                motor_start();
                // Boat LEDs blink yellow while raising
                if ((current_time - boat_blink_last) >= 500) {
                    boat_blink_last = current_time;
                    boat_blink_on = !boat_blink_on;
                    if (boat_blink_on) boat_led_yellow(); else boat_led_off();
                }
                traffic_led_red();
                break;

            case BRIDGE_TOP_REACHED: {
                bool boat1_present = bridge_boat_present();
                bool boat2_present = bridge_boat2_present();

                if ((current_time - bridge_top_enter_time) >= T_MAX_UP_HOLD) {
                    ESP_LOGI(TAG, "TOP watchdog elapsed — proceeding to pre-lower.");
                    // Enter pre-lower warning; boat will blink yellow there
                    traffic_led_red();
                    bridge_set_state(BRIDGE_PRE_LOWER_WARN);
                    break;
                }

                if (!bridge_top_saw_gap) {
                    // Wait for the initial sensor to lose sight (gap) indicating the boat is passing under
                    if (!boat1_present && !boat2_present) {
                        bridge_top_saw_gap = true;
                        ESP_LOGI(TAG, "Boat sensors gap detected. Waiting for the other sensor to see the boat...");
                    }
                } else {
                    // We already saw a gap. Now require that the other sensor detects the boat to confirm passage.
                    if (boat_initial_sensor == 1) {
                        if (boat2_present) {
                            ESP_LOGI(TAG, "Boat sensed by sensor2 after gap — starting pre-lower delay.");
                            traffic_led_red();
                            bridge_set_state(BRIDGE_PRE_LOWER_WARN);
                        }
                    } else if (boat_initial_sensor == 2) {
                        if (boat1_present) {
                            ESP_LOGI(TAG, "Boat sensed by sensor1 after gap — starting pre-lower delay.");
                            traffic_led_red();
                            bridge_set_state(BRIDGE_PRE_LOWER_WARN);
                        }
                    } else {
                        // Fallback: if any sensor reports presence again, proceed
                        if (boat1_present || boat2_present) {
                            ESP_LOGI(TAG, "Boat sensed again — starting pre-lower delay.");
                            traffic_led_red();
                            bridge_set_state(BRIDGE_PRE_LOWER_WARN);
                        }
                    }
                }
                // While at top and not pre-lowering, boat LED stays green
                boat_led_green();
                traffic_led_red();
                break;
            }

            case BRIDGE_PRE_LOWER_WARN:
                traffic_led_red();
                // Boat LEDs blink yellow in pre-lower warning
                if ((current_time - boat_blink_last) >= 500) {
                    boat_blink_last = current_time;
                    boat_blink_on = !boat_blink_on;
                    if (boat_blink_on) boat_led_yellow(); else boat_led_off();
                }
                if ((current_time - bridge_state_start) >= T_LOWER_CAUT_YEL) {
                    ESP_LOGI(TAG, "Lowering bridge.");
                    bridge_set_state(BRIDGE_LOWERING);
                }
                break;

            case BRIDGE_LOWERING:
                if (bridge_switch_triggered(pins.bridge_sw_lowered_pin)) {
                    ESP_LOGI(TAG, "Bottom limit reached. Settling...");
                    bridge_set_state(BRIDGE_AFTER_LOWERED);
                    motor_set_speed(0);
                    motor_stop();
                    break;
                }
                motor_set_direction(true); // Lower
                motor_set_speed(30);
                motor_start();
                traffic_led_red();
                boat_led_red();
                break;

            case BRIDGE_AFTER_LOWERED:
                traffic_led_red();
                boat_led_red();
                if ((current_time - bridge_state_start) >= T_AFTER_LOWERED) {
                    ESP_LOGI(TAG, "Traffic GREEN. Returning to ARMED.");
                    bridge_set_state(BRIDGE_OPEN_TRAFFIC);
                }
                break;

            case BRIDGE_OPEN_TRAFFIC:
                traffic_led_green();
                boat_led_red();
                if (bridge_boat_present() || bridge_boat2_present()) {
                    ESP_LOGI(TAG, "New boat present — cycling again.");
                    bridge_set_state(BRIDGE_WARN_TRAFFIC);
                } else {
                    bridge_set_state(BRIDGE_ARMED);
                }
                break;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// HTTP handlers
static esp_err_t index_handler(httpd_req_t *req) {
    FILE *file = fopen("/spiffs/index.html", "r");
    if (file == NULL) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    char buffer[1024];
    size_t read_bytes;
    httpd_resp_set_type(req, "text/html");

    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    httpd_resp_send_chunk(req, NULL, 0);
    fclose(file);
    return ESP_OK;
}

static esp_err_t js_handler(httpd_req_t *req) {
    FILE *file = fopen("/spiffs/app.js", "r");
    if (file == NULL) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    char buffer[1024];
    size_t read_bytes;
    httpd_resp_set_type(req, "application/javascript");

    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    httpd_resp_send_chunk(req, NULL, 0);
    fclose(file);
    return ESP_OK;
}

static esp_err_t distance_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddNumberToObject(root, "distance", current_distance);
    cJSON_AddStringToObject(root, "unit", "cm");

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t toggle_handler(httpd_req_t *req) {
    static bool led_state = false;
    led_state = !led_state;
    gpio_set_level(pins.led_pin, led_state);

    const char *response = led_state ? "LED is ON" : "LED is OFF";
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static esp_err_t motor_handler(httpd_req_t *req) {
    FILE *file = fopen("/spiffs/motor.html", "r");
    if (file == NULL) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    char buffer[1024];
    size_t read_bytes;
    httpd_resp_set_type(req, "text/html");

    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    httpd_resp_send_chunk(req, NULL, 0);
    fclose(file);
    return ESP_OK;
}

static esp_err_t motor_js_handler(httpd_req_t *req) {
    FILE *file = fopen("/spiffs/motor.js", "r");
    if (file == NULL) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    char buffer[1024];
    size_t read_bytes;
    httpd_resp_set_type(req, "application/javascript");

    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    httpd_resp_send_chunk(req, NULL, 0);
    fclose(file);
    return ESP_OK;
}

static esp_err_t chart_js_handler(httpd_req_t *req) {
    FILE *file = fopen("/spiffs/chart.min.js", "r");
    if (file == NULL) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Chart.js file not found");
        return ESP_FAIL;
    }

    char buffer[1024];
    size_t read_bytes;
    httpd_resp_set_type(req, "application/javascript");
    
    // Set cache headers for better performance
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=86400");

    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    httpd_resp_send_chunk(req, NULL, 0);
    fclose(file);
    return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req) {
    uint32_t uptime = (esp_timer_get_time() / 1000000) - system_start_time;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddNumberToObject(root, "uptime", uptime);

    // Get actual WiFi client count
    wifi_sta_list_t sta_list;
    esp_err_t wifi_err = esp_wifi_ap_get_sta_list(&sta_list);
    int client_count = 0;
    if (wifi_err == ESP_OK) {
        client_count = sta_list.num;
    }
    cJSON_AddNumberToObject(root, "wifi_clients", client_count);

    cJSON_AddStringToObject(root, "version", "1.0.0");

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t motor_status_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddBoolToObject(root, "running", motor_running);
    cJSON_AddNumberToObject(root, "speed", motor_speed);
    cJSON_AddStringToObject(root, "direction", motor_direction ? "forward" : "reverse");
    cJSON_AddNumberToObject(root, "pulses", encoder_pulses);

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t motor_start_handler(httpd_req_t *req) {
    // Parse query parameters
    char buf[100];
    int speed = 50; // Default speed
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[32];
        if (httpd_query_key_value(buf, "speed", param, sizeof(param)) == ESP_OK) {
            speed = atoi(param);
            if (speed < 0) speed = 0;
            if (speed > 100) speed = 100;
        }
    }

    motor_speed = speed;
    motor_start();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "message", "Motor started");
    cJSON_AddNumberToObject(root, "speed", motor_speed);

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t motor_stop_handler(httpd_req_t *req) {
    motor_stop();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "message", "Motor stopped");

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t motor_direction_handler(httpd_req_t *req) {
    // Parse query parameters
    char buf[100];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[32];
        if (httpd_query_key_value(buf, "dir", param, sizeof(param)) == ESP_OK) {
            if (strcmp(param, "forward") == 0) {
                motor_set_direction(true);
            } else if (strcmp(param, "reverse") == 0) {
                motor_set_direction(false);
            }
        }
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "direction", motor_direction ? "forward" : "reverse");

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t motor_reset_handler(httpd_req_t *req) {
    encoder_pulses = 0;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "message", "Encoder reset");

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t motor_brake_handler(httpd_req_t *req) {
    motor_brake();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "message", "Motor braked");

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t motor_speed_handler(httpd_req_t *req) {
    // Parse query parameters
    char buf[100];
    int speed = motor_speed; // Default to current speed
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[32];
        if (httpd_query_key_value(buf, "speed", param, sizeof(param)) == ESP_OK) {
            speed = atoi(param);
            if (speed < 0) speed = 0;
            if (speed > 100) speed = 100;
        }
    }

    motor_set_speed(speed);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "message", "Speed updated");
    cJSON_AddNumberToObject(root, "speed", motor_speed);

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t test_handler(httpd_req_t *req) {
    const char *response = "Test endpoint working!";
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static esp_err_t config_handler(httpd_req_t *req) {
    FILE *file = fopen("/spiffs/config.html", "r");
    if (file == NULL) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    char buffer[1024];
    size_t read_bytes;
    httpd_resp_set_type(req, "text/html");

    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    httpd_resp_send_chunk(req, NULL, 0);
    fclose(file);
    return ESP_OK;
}

static esp_err_t system_handler(httpd_req_t *req) {
    FILE *file = fopen("/spiffs/system.html", "r");
    if (file == NULL) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    char buffer[1024];
    size_t read_bytes;
    httpd_resp_set_type(req, "text/html");

    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    httpd_resp_send_chunk(req, NULL, 0);
    fclose(file);
    return ESP_OK;
}

static esp_err_t bridge_handler(httpd_req_t *req) {
    FILE *file = fopen("/spiffs/bridge.html", "r");
    if (file == NULL) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    char buffer[1024];
    size_t read_bytes;
    httpd_resp_set_type(req, "text/html");

    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    httpd_resp_send_chunk(req, NULL, 0);
    fclose(file);
    return ESP_OK;
}

static esp_err_t bridge_status_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "state", bridge_state_name(bridge_state));
    cJSON_AddBoolToObject(root, "manual_override", bridge_manual_override);
    cJSON_AddBoolToObject(root, "car_on_bridge", bridge_car_on_bridge());
    cJSON_AddBoolToObject(root, "boat_present", bridge_boat_present());
    cJSON_AddBoolToObject(root, "switch_lowered", bridge_switch_triggered(pins.bridge_sw_lowered_pin));
    cJSON_AddBoolToObject(root, "switch_raised", bridge_switch_triggered(pins.bridge_sw_raised_pin));
    cJSON_AddNumberToObject(root, "boat2_distance", bridge_ping_cm(pins.bridge_boat2_trig_pin, pins.bridge_boat2_echo_pin));
    cJSON_AddNumberToObject(root, "boat_distance", bridge_ping_cm(pins.bridge_boat_trig_pin, pins.bridge_boat_echo_pin));
    
    // Add calibration status
    const char* calib_state_str;
    switch (calib_state) {
        case CALIB_IDLE: calib_state_str = "idle"; break;
        case CALIB_IN_PROGRESS: calib_state_str = "in_progress"; break;
        case CALIB_COMPLETE: calib_state_str = "complete"; break;
        default: calib_state_str = "unknown"; break;
    }
    cJSON_AddStringToObject(root, "calibration_state", calib_state_str);
    cJSON_AddBoolToObject(root, "calibrated", calib_state == CALIB_COMPLETE);
    
    if (calib_state == CALIB_COMPLETE) {
        cJSON_AddNumberToObject(root, "baseline_mean", baseline_mean);
        cJSON_AddNumberToObject(root, "baseline_stddev", baseline_stddev);
        cJSON_AddBoolToObject(root, "boat_detected_calibrated", boat_detected);
        cJSON_AddNumberToObject(root, "consecutive_detections", consecutive_detections);
        cJSON_AddNumberToObject(root, "consecutive_clear", consecutive_clear);
    }
    
    // Add raw hall effect sensor readings in millivolts
    cJSON_AddNumberToObject(root, "hall_lowered_raw", read_hall_sensor_mv(pins.bridge_sw_lowered_pin));
    cJSON_AddNumberToObject(root, "hall_raised_raw", read_hall_sensor_mv(pins.bridge_sw_raised_pin));
    
    // Add raw ADC counts for debugging
    adc_channel_t lowered_channel = gpio_to_adc_channel(pins.bridge_sw_lowered_pin);
    adc_channel_t raised_channel = gpio_to_adc_channel(pins.bridge_sw_raised_pin);
    
    if (lowered_channel != -1 && adc1_handle != NULL) {
        int raw_value;
        esp_err_t ret = adc_oneshot_read(adc1_handle, lowered_channel, &raw_value);
        if (ret == ESP_OK) {
            cJSON_AddNumberToObject(root, "hall_lowered_adc_raw", raw_value);
        }
    }
    
    if (raised_channel != -1 && adc1_handle != NULL) {
        int raw_value;
        esp_err_t ret = adc_oneshot_read(adc1_handle, raised_channel, &raw_value);
        if (ret == ESP_OK) {
            cJSON_AddNumberToObject(root, "hall_raised_adc_raw", raw_value);
        }
    }
    
    // Add hall effect threshold
    cJSON_AddNumberToObject(root, "hall_threshold", hall_effect_threshold);
    
    // Add LED states (traffic and boat)
    cJSON_AddBoolToObject(root, "traffic_led_r", traffic_led_r_state);
    cJSON_AddBoolToObject(root, "traffic_led_y", traffic_led_y_state);
    cJSON_AddBoolToObject(root, "traffic_led_g", traffic_led_g_state);
    cJSON_AddBoolToObject(root, "traffic_led_b", traffic_led_b_state);
    cJSON_AddBoolToObject(root, "boat_led_r", boat_led_r_state);
    cJSON_AddBoolToObject(root, "boat_led_y", boat_led_y_state);
    cJSON_AddBoolToObject(root, "boat_led_g", boat_led_g_state);
    cJSON_AddBoolToObject(root, "boat_led_b", boat_led_b_state);
    // Car detection enabled flag
    cJSON_AddBoolToObject(root, "car_detection_enabled", car_detection_enabled);
    // Detection meter for UI
    cJSON_AddNumberToObject(root, "detection_meter", boat_detection_meter);
    // Detection meter for boat2 (separate)
    cJSON_AddNumberToObject(root, "detection_meter_boat2", boat2_detection_meter);
    // Which sensor first detected the approaching boat (0=none, 1=boat1, 2=boat2)
    cJSON_AddNumberToObject(root, "boat_initial_sensor", boat_initial_sensor);

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t bridge_override_handler(httpd_req_t *req) {
    char buf[100];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[32];
        if (httpd_query_key_value(buf, "enable", param, sizeof(param)) == ESP_OK) {
            bridge_manual_override = (strcmp(param, "true") == 0);
            ESP_LOGI(TAG, "Bridge manual override: %s", bridge_manual_override ? "ENABLED" : "DISABLED");
        }
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddBoolToObject(root, "manual_override", bridge_manual_override);

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t bridge_control_handler(httpd_req_t *req) {
    if (!bridge_manual_override) {
        cJSON *root = cJSON_CreateObject();
        cJSON_AddBoolToObject(root, "success", false);
        cJSON_AddStringToObject(root, "error", "Manual override not enabled");

        const char *json_string = cJSON_Print(root);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json_string, strlen(json_string));

        free((void *)json_string);
        cJSON_Delete(root);
        return ESP_OK;
    }

    char buf[100];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[32];
        if (httpd_query_key_value(buf, "action", param, sizeof(param)) == ESP_OK) {
            if (strcmp(param, "stop") == 0) {
                ESP_LOGI(TAG, "Bridge manual control: STOP");
                motor_stop();
            } else if (strcmp(param, "up") == 0) {
                ESP_LOGI(TAG, "Bridge manual control: UP");
                motor_set_direction(true); // Raise
                motor_set_speed(motor_speed);
                motor_start();
            } else if (strcmp(param, "down") == 0) {
                ESP_LOGI(TAG, "Bridge manual control: DOWN");
                motor_set_direction(false); // Lower
                motor_set_speed(motor_speed);
                motor_start();
            } else if (strcmp(param, "reset") == 0) {
                bridge_manual_override = false;
                bridge_set_state(BRIDGE_ARMED);
                ESP_LOGI(TAG, "Bridge manual control: RESET to auto");
            }
        }
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "message", "Command executed");

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t car_detection_handler(httpd_req_t *req) {
    // GET with query ?enable=true/false to set, otherwise just return current state
    char buf[100];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[32];
        if (httpd_query_key_value(buf, "enable", param, sizeof(param)) == ESP_OK) {
            car_detection_enabled = (strcmp(param, "true") == 0);
            ESP_LOGI(TAG, "Car detection enabled: %s", car_detection_enabled ? "true" : "false");
        }
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddBoolToObject(root, "car_detection_enabled", car_detection_enabled);

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t hall_threshold_get_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddNumberToObject(root, "threshold", hall_effect_threshold);

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t hall_threshold_set_handler(httpd_req_t *req) {
    char buf[100];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[32];
        if (httpd_query_key_value(buf, "threshold", param, sizeof(param)) == ESP_OK) {
            int new_threshold = atoi(param);
            
            // Validate threshold range (0-4095 for 12-bit ADC)
            if (new_threshold < 0 || new_threshold > 4095) {
                cJSON *root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "success", false);
                cJSON_AddStringToObject(root, "error", "Threshold must be between 0 and 4095");

                const char *json_string = cJSON_Print(root);
                httpd_resp_set_type(req, "application/json");
                httpd_resp_send(req, json_string, strlen(json_string));

                free((void *)json_string);
                cJSON_Delete(root);
                return ESP_OK;
            }
            
            hall_effect_threshold = new_threshold;
            ESP_LOGI(TAG, "Hall effect threshold set to: %d", hall_effect_threshold);
        }
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddNumberToObject(root, "threshold", hall_effect_threshold);

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t calibration_start_handler(httpd_req_t *req) {
    start_calibration();
    
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "message", "Calibration started - keep area clear of boats");
    cJSON_AddNumberToObject(root, "total_samples", BASELINE_SAMPLES);

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t calibration_status_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    
    const char* state_str;
    switch (calib_state) {
        case CALIB_IDLE: state_str = "idle"; break;
        case CALIB_IN_PROGRESS: state_str = "in_progress"; break;
        case CALIB_COMPLETE: state_str = "complete"; break;
        default: state_str = "unknown"; break;
    }
    
    cJSON_AddStringToObject(root, "state", state_str);
    cJSON_AddNumberToObject(root, "samples_collected", calib_sample_count);
    cJSON_AddNumberToObject(root, "total_samples", BASELINE_SAMPLES);
    
    if (calib_state == CALIB_COMPLETE) {
        cJSON_AddNumberToObject(root, "baseline_mean", baseline_mean);
        cJSON_AddNumberToObject(root, "baseline_stddev", baseline_stddev);
        cJSON_AddNumberToObject(root, "threshold_cm", THRESHOLD_CM);
        cJSON_AddBoolToObject(root, "boat_detected", boat_detected);
        cJSON_AddNumberToObject(root, "consecutive_detections", consecutive_detections);
        cJSON_AddNumberToObject(root, "consecutive_clear", consecutive_clear);
    }
    
    cJSON_AddNumberToObject(root, "current_distance", current_distance);

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t calibration_reset_handler(httpd_req_t *req) {
    // Reset calibration state
    calib_state = CALIB_IDLE;
    calib_sample_count = 0;
    baseline_mean = 0.0;
    baseline_stddev = 0.0;
    consecutive_detections = 0;
    consecutive_clear = 0;
    boat_detected = false;
    
    ESP_LOGI(TAG, "Calibration reset to idle state");
    
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "message", "Calibration reset");

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t sensor_data_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    
    // Add current reading info
    cJSON_AddNumberToObject(root, "current_distance", current_distance);
    cJSON_AddNumberToObject(root, "detection_meter", boat_detection_meter);
    cJSON_AddNumberToObject(root, "detection_meter_boat2", boat2_detection_meter);
    cJSON_AddBoolToObject(root, "calibrated", calib_state == CALIB_COMPLETE);
    
    if (calib_state == CALIB_COMPLETE) {
        cJSON_AddNumberToObject(root, "baseline_mean", baseline_mean);
        cJSON_AddNumberToObject(root, "threshold_distance", baseline_mean - THRESHOLD_CM);
        cJSON_AddBoolToObject(root, "boat_detected", boat_detected);
        cJSON_AddNumberToObject(root, "delta", baseline_mean - current_distance);
    }
    
    // Add recent sensor readings for graphing
    cJSON *readings_array = cJSON_CreateArray();
    cJSON *timestamps_array = cJSON_CreateArray();
    
    int start_idx = (reading_index - readings_count + MAX_SENSOR_READINGS) % MAX_SENSOR_READINGS;
    for (int i = 0; i < readings_count; i++) {
        int idx = (start_idx + i) % MAX_SENSOR_READINGS;
        cJSON_AddItemToArray(readings_array, cJSON_CreateNumber(sensor_readings[idx]));
        cJSON_AddItemToArray(timestamps_array, cJSON_CreateNumber(reading_timestamps[idx]));
    }
    
    cJSON_AddItemToObject(root, "readings", readings_array);
    cJSON_AddItemToObject(root, "timestamps", timestamps_array);
    cJSON_AddNumberToObject(root, "readings_count", readings_count);

    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

// === BOAT2 SENSOR CALIBRATION API HANDLERS ===

static esp_err_t boat2_calibration_start_handler(httpd_req_t *req) {
    // Start calibration for the second boat ultrasonic
    start_boat2_calibration();
    
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "message", "Boat2 sensor calibration started");
    cJSON_AddStringToObject(root, "state", "in_progress");
    
    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t boat2_calibration_status_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    
    const char *state_str;
    switch (boat2_calib_state) {
        case CALIB_IDLE: state_str = "idle"; break;
        case CALIB_IN_PROGRESS: state_str = "in_progress"; break;
        case CALIB_COMPLETE: state_str = "complete"; break;
        default: state_str = "unknown"; break;
    }
    cJSON_AddStringToObject(root, "state", state_str);
    
    if (boat2_calib_state == CALIB_IN_PROGRESS) {
        cJSON_AddNumberToObject(root, "samples_collected", boat2_calib_sample_count);
        cJSON_AddNumberToObject(root, "total_samples", BASELINE_SAMPLES);
        cJSON_AddNumberToObject(root, "progress_percent", (boat2_calib_sample_count * 100) / BASELINE_SAMPLES);
    } else if (boat2_calib_state == CALIB_COMPLETE) {
        cJSON_AddNumberToObject(root, "baseline_mean", boat2_baseline_mean);
        cJSON_AddNumberToObject(root, "baseline_stddev", boat2_baseline_stddev);
        cJSON_AddNumberToObject(root, "threshold_cm", THRESHOLD_CM);
        cJSON_AddNumberToObject(root, "required_consecutive", REQUIRED_CONSECUTIVE);
        cJSON_AddNumberToObject(root, "reset_consecutive", RESET_CONSECUTIVE);
        cJSON_AddBoolToObject(root, "boat2_detected", boat2_detected);
    }
    
    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t boat2_calibration_reset_handler(httpd_req_t *req) {
    // Reset calibration state
    boat2_calib_state = CALIB_IDLE;
    boat2_baseline_mean = 0.0;
    boat2_baseline_stddev = 0.0;
    boat2_calib_sample_count = 0;
    boat2_consecutive_detections = 0;
    boat2_consecutive_clear = 0;
    boat2_detected = false;
    
    // Clear data buffers
    boat2_reading_index = 0;
    boat2_readings_count = 0;
    
    ESP_LOGI(TAG, "Boat2 sensor calibration reset");
    
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "message", "Boat2 sensor calibration reset");
    cJSON_AddStringToObject(root, "state", "idle");
    
    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t boat2_sensor_data_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    
    // Get current boat2 distance (repurposed from former boat2 ultrasonic)
    float boat2_distance = measure_distance(pins.bridge_boat2_trig_pin, pins.bridge_boat2_echo_pin);

    // Add current reading info (represents boat2 sensor)
    cJSON_AddNumberToObject(root, "current_distance", boat2_distance);
    cJSON_AddBoolToObject(root, "calibrated", boat2_calib_state == CALIB_COMPLETE);
    
    if (boat2_calib_state == CALIB_COMPLETE) {
        cJSON_AddNumberToObject(root, "baseline_mean", boat2_baseline_mean);
        cJSON_AddNumberToObject(root, "threshold_distance", boat2_baseline_mean - THRESHOLD_CM);
        cJSON_AddBoolToObject(root, "boat2_detected", boat2_detected);
    }
    
    // Add buffered readings for charting
    cJSON *readings_array = cJSON_CreateArray();
    cJSON *timestamps_array = cJSON_CreateArray();
    
    for (int i = 0; i < boat2_readings_count; i++) {
        int idx = (boat2_reading_index + i) % MAX_SENSOR_READINGS;
        cJSON_AddItemToArray(readings_array, cJSON_CreateNumber(boat2_sensor_readings[idx]));
        cJSON_AddItemToArray(timestamps_array, cJSON_CreateNumber(boat2_reading_timestamps[idx]));
    }
    
    cJSON_AddItemToObject(root, "readings", readings_array);
    cJSON_AddItemToObject(root, "timestamps", timestamps_array);
    cJSON_AddNumberToObject(root, "count", boat2_readings_count);
    
    const char *json_string = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free((void *)json_string);
    cJSON_Delete(root);
    return ESP_OK;
}

// Server-Sent Events handler for real-time data streaming
esp_err_t sse_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "SSE connection established");
    
    // Set headers for SSE
    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    // Send initial connection message
    const char *initial_msg = "data: {\"type\":\"connected\"}\n\n";
    esp_err_t ret = httpd_resp_send_chunk(req, initial_msg, strlen(initial_msg));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send initial SSE message");
        return ret;
    }
    
    // Send periodic updates for 5 minutes max (300 updates at 1 second intervals)
    for (int i = 0; i < 300; i++) {
        // Create simple status message
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "type", "bridge-status");
        cJSON_AddStringToObject(root, "state", bridge_state_name(bridge_state));
        cJSON_AddBoolToObject(root, "boat_present", bridge_boat_present());
        cJSON_AddNumberToObject(root, "boat_distance", bridge_ping_cm(pins.bridge_boat_trig_pin, pins.bridge_boat_echo_pin));
        // Include repurposed second boat sensor (boat2) data for UI visibility
        cJSON_AddBoolToObject(root, "boat2_present", bridge_boat2_present());
        cJSON_AddNumberToObject(root, "boat2_distance", bridge_ping_cm(pins.bridge_boat2_trig_pin, pins.bridge_boat2_echo_pin));
        // Detection meter
        cJSON_AddNumberToObject(root, "detection_meter", boat_detection_meter);
        cJSON_AddNumberToObject(root, "current_distance", current_distance);
        cJSON_AddBoolToObject(root, "calibrated", calib_state == CALIB_COMPLETE);
        
        if (calib_state == CALIB_IN_PROGRESS) {
            cJSON_AddNumberToObject(root, "calib_progress", calib_sample_count);
            cJSON_AddNumberToObject(root, "calib_total", BASELINE_SAMPLES);
        }
        
        if (calib_state == CALIB_COMPLETE) {
            cJSON_AddNumberToObject(root, "baseline_mean", baseline_mean);
            cJSON_AddBoolToObject(root, "boat_detected", boat_detected);
        }
        
        char *json_string = cJSON_Print(root);
        
        // Format as proper SSE message
        char sse_buffer[1024];
        int msg_len = snprintf(sse_buffer, sizeof(sse_buffer), "data: %s\n\n", json_string);
        
        // Send the message
        ret = httpd_resp_send_chunk(req, sse_buffer, msg_len);
        
        free(json_string);
        cJSON_Delete(root);
        
        if (ret != ESP_OK) {
            ESP_LOGI(TAG, "SSE client disconnected");
            break;
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS); // 1 second interval
    }
    
    // Close the connection
    httpd_resp_send_chunk(req, NULL, 0);
    ESP_LOGI(TAG, "SSE connection ended");
    return ESP_OK;
}

// SPIFFS initialization
void init_spiffs(void) {
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS partition size: total: %d, used: %d", total, used);
    }
}

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station joined, AID=%d", event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station left, AID=%d", event->aid);
    }
}

// Initialize WiFi in AP mode
void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = 1,
            .password = WIFI_PASS,
            .max_connection = MAX_CONNECTIONS,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP initialized. SSID: %s, Password: %s", WIFI_SSID, WIFI_PASS);
}

// Start web server
static esp_err_t pin_config_get_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    
    cJSON_AddNumberToObject(root, "led_pin", pins.led_pin);
    cJSON_AddNumberToObject(root, "ultrasonic_trig_pin", pins.ultrasonic_trig_pin);
    cJSON_AddNumberToObject(root, "ultrasonic_echo_pin", pins.ultrasonic_echo_pin);
    cJSON_AddNumberToObject(root, "motor_enca_pin", pins.motor_enca_pin);
    cJSON_AddNumberToObject(root, "motor_encb_pin", pins.motor_encb_pin);
    cJSON_AddNumberToObject(root, "motor_in1_pin", pins.motor_in1_pin);
    cJSON_AddNumberToObject(root, "motor_in2_pin", pins.motor_in2_pin);
    cJSON_AddNumberToObject(root, "motor_ena_pin", pins.motor_ena_pin);
    cJSON_AddNumberToObject(root, "bridge_boat_trig_pin", pins.bridge_boat_trig_pin);
    cJSON_AddNumberToObject(root, "bridge_boat_echo_pin", pins.bridge_boat_echo_pin);
    cJSON_AddNumberToObject(root, "bridge_boat2_trig_pin", pins.bridge_boat2_trig_pin);
    cJSON_AddNumberToObject(root, "bridge_boat2_echo_pin", pins.bridge_boat2_echo_pin);
    cJSON_AddNumberToObject(root, "bridge_sw_lowered_pin", pins.bridge_sw_lowered_pin);
    cJSON_AddNumberToObject(root, "bridge_sw_raised_pin", pins.bridge_sw_raised_pin);
    // New traffic LED pins
    cJSON_AddNumberToObject(root, "traffic_led_r_pin", pins.traffic_led_r_pin);
    cJSON_AddNumberToObject(root, "traffic_led_y_pin", pins.traffic_led_y_pin);
    cJSON_AddNumberToObject(root, "traffic_led_g_pin", pins.traffic_led_g_pin);
    cJSON_AddNumberToObject(root, "traffic_led_b_pin", pins.traffic_led_b_pin);
    // New boat LED pins
    cJSON_AddNumberToObject(root, "boat_led_r_pin", pins.boat_led_r_pin);
    cJSON_AddNumberToObject(root, "boat_led_y_pin", pins.boat_led_y_pin);
    cJSON_AddNumberToObject(root, "boat_led_g_pin", pins.boat_led_g_pin);
    cJSON_AddNumberToObject(root, "boat_led_b_pin", pins.boat_led_b_pin);
    
    char *json_response = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, strlen(json_response));
    
    free(json_response);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t pin_config_post_handler(httpd_req_t *req) {
    char buf[1024];
    int received = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data received");
        return ESP_FAIL;
    }
    buf[received] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    pin_config_t new_config = pins; // Start with current config
    
    // Parse each pin field
    cJSON *pin_item;
    if ((pin_item = cJSON_GetObjectItem(json, "led_pin"))) {
        new_config.led_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "ultrasonic_trig_pin"))) {
        new_config.ultrasonic_trig_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "ultrasonic_echo_pin"))) {
        new_config.ultrasonic_echo_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "motor_enca_pin"))) {
        new_config.motor_enca_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "motor_encb_pin"))) {
        new_config.motor_encb_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "motor_in1_pin"))) {
        new_config.motor_in1_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "motor_in2_pin"))) {
        new_config.motor_in2_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "motor_ena_pin"))) {
        new_config.motor_ena_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "bridge_boat_trig_pin"))) {
        new_config.bridge_boat_trig_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "bridge_boat_echo_pin"))) {
        new_config.bridge_boat_echo_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "bridge_boat2_trig_pin"))) {
        new_config.bridge_boat2_trig_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "bridge_boat2_echo_pin"))) {
        new_config.bridge_boat2_echo_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "bridge_sw_lowered_pin"))) {
        new_config.bridge_sw_lowered_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "bridge_sw_raised_pin"))) {
        new_config.bridge_sw_raised_pin = pin_item->valueint;
    }
    // Traffic LED pins
    if ((pin_item = cJSON_GetObjectItem(json, "traffic_led_r_pin"))) {
        new_config.traffic_led_r_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "traffic_led_y_pin"))) {
        new_config.traffic_led_y_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "traffic_led_g_pin"))) {
        new_config.traffic_led_g_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "traffic_led_b_pin"))) {
        new_config.traffic_led_b_pin = pin_item->valueint;
    }
    // Boat LED pins
    if ((pin_item = cJSON_GetObjectItem(json, "boat_led_r_pin"))) {
        new_config.boat_led_r_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "boat_led_y_pin"))) {
        new_config.boat_led_y_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "boat_led_g_pin"))) {
        new_config.boat_led_g_pin = pin_item->valueint;
    }
    if ((pin_item = cJSON_GetObjectItem(json, "boat_led_b_pin"))) {
        new_config.boat_led_b_pin = pin_item->valueint;
    }
    
    cJSON_Delete(json);
    
    // Validate configuration
    char error_msg[256];
    if (!validate_pin_config(&new_config, error_msg, sizeof(error_msg))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, error_msg);
        return ESP_FAIL;
    }
    
    // Update current config first
    pins = new_config;
    
    // Save configuration
    if (save_pin_config() != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save configuration");
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"success\"}", -1);
    return ESP_OK;
}

static esp_err_t pin_config_reset_handler(httpd_req_t *req) {
    pins = default_pins;
    
    if (save_pin_config() != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to reset configuration");
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"reset\"}", -1);
    return ESP_OK;
}

static esp_err_t restart_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"restarting\"}", -1);
    
    // Delay to ensure response is sent
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    
    return ESP_OK;
}

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 40;  // Increased for new calibration endpoints
    config.stack_size = 8192;  // Increase from default 4096 to handle larger headers

    httpd_uri_t uri_handlers[] = {
        {"/", HTTP_GET, index_handler, NULL},
        {"/index", HTTP_GET, index_handler, NULL},
        {"/app.js", HTTP_GET, js_handler, NULL},
        {"/api/distance", HTTP_GET, distance_handler, NULL},
        {"/distance", HTTP_GET, distance_handler, NULL},
        {"/api/toggle", HTTP_GET, toggle_handler, NULL},
        {"/motor", HTTP_GET, motor_handler, NULL},
        {"/bridge", HTTP_GET, bridge_handler, NULL},
        {"/motor.js", HTTP_GET, motor_js_handler, NULL},
        {"/chart.min.js", HTTP_GET, chart_js_handler, NULL},
        {"/api/status", HTTP_GET, status_handler, NULL},
        {"/api/motor/status", HTTP_GET, motor_status_handler, NULL},
        {"/api/motor/start", HTTP_GET, motor_start_handler, NULL},
        {"/api/motor/stop", HTTP_GET, motor_stop_handler, NULL},
        {"/api/motor/speed", HTTP_GET, motor_speed_handler, NULL},
        {"/api/motor/direction", HTTP_GET, motor_direction_handler, NULL},
        {"/api/motor/reset", HTTP_GET, motor_reset_handler, NULL},
        {"/api/motor/brake", HTTP_GET, motor_brake_handler, NULL},
        {"/motor/brake", HTTP_GET, motor_brake_handler, NULL},
        {"/api/bridge/status", HTTP_GET, bridge_status_handler, NULL},
        {"/api/bridge/override", HTTP_GET, bridge_override_handler, NULL},
        {"/api/car-detection", HTTP_GET, car_detection_handler, NULL},
        {"/api/bridge/control", HTTP_GET, bridge_control_handler, NULL},
        {"/api/hall/threshold", HTTP_GET, hall_threshold_get_handler, NULL},
        {"/api/hall/threshold/set", HTTP_GET, hall_threshold_set_handler, NULL},
        {"/api/calibration/start", HTTP_POST, calibration_start_handler, NULL},
        {"/api/calibration/status", HTTP_GET, calibration_status_handler, NULL},
        {"/api/calibration/reset", HTTP_POST, calibration_reset_handler, NULL},
        {"/api/sensor/data", HTTP_GET, sensor_data_handler, NULL},
        {"/api/boat2/calibration/start", HTTP_POST, boat2_calibration_start_handler, NULL},
        {"/api/boat2/calibration/status", HTTP_GET, boat2_calibration_status_handler, NULL},
        {"/api/boat2/calibration/reset", HTTP_POST, boat2_calibration_reset_handler, NULL},
        {"/api/boat2/sensor/data", HTTP_GET, boat2_sensor_data_handler, NULL},
        {"/api/pin-config", HTTP_GET, pin_config_get_handler, NULL},
        {"/api/pin-config", HTTP_POST, pin_config_post_handler, NULL},
        {"/api/pin-config/reset", HTTP_POST, pin_config_reset_handler, NULL},
        {"/api/restart", HTTP_POST, restart_handler, NULL},
        {"/config", HTTP_GET, config_handler, NULL},
        {"/system", HTTP_GET, system_handler, NULL},
        {"/test", HTTP_GET, test_handler, NULL}
    };

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        for (int i = 0; i < sizeof(uri_handlers) / sizeof(uri_handlers[0]); i++) {
            httpd_register_uri_handler(server, &uri_handlers[i]);
        }
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void app_main() {
    // Initialize system start time
    system_start_time = esp_timer_get_time() / 1000000;

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Load pin configuration from NVS (or use defaults)
    load_pin_config();

    // Initialize SPIFFS
    init_spiffs();

    // Initialize GPIO for LED
    gpio_set_direction(pins.led_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pins.led_pin, 0); // Start with LED off

    // Initialize GPIO for motor encoder
    gpio_set_direction(pins.motor_enca_pin, GPIO_MODE_INPUT);

    // Initialize GPIO for L298N motor driver
    gpio_set_direction(pins.motor_in1_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(pins.motor_in2_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pins.motor_in1_pin, 0); // Start with motor stopped
    gpio_set_level(pins.motor_in2_pin, 0);

    // Initialize PWM for motor speed control
    init_motor_pwm();

    // Initialize bridge GPIO pins
    gpio_set_direction(pins.bridge_boat_trig_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(pins.bridge_boat_echo_pin, GPIO_MODE_INPUT);
    gpio_set_direction(pins.bridge_boat2_trig_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(pins.bridge_boat2_echo_pin, GPIO_MODE_INPUT);
    
    // Hall effect sensors - only set as GPIO input if not ADC pins
    adc_channel_t lowered_adc = gpio_to_adc_channel(pins.bridge_sw_lowered_pin);
    adc_channel_t raised_adc = gpio_to_adc_channel(pins.bridge_sw_raised_pin);
    
    if (lowered_adc == -1) {
        gpio_set_direction(pins.bridge_sw_lowered_pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(pins.bridge_sw_lowered_pin, GPIO_PULLUP_ONLY);
    }
    if (raised_adc == -1) {
        gpio_set_direction(pins.bridge_sw_raised_pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(pins.bridge_sw_raised_pin, GPIO_PULLUP_ONLY);
    }
    
    // Initialize LED GPIOs (only if output-capable)
    if (is_output_capable(pins.traffic_led_r_pin)) gpio_set_direction(pins.traffic_led_r_pin, GPIO_MODE_OUTPUT);
    if (is_output_capable(pins.traffic_led_y_pin)) gpio_set_direction(pins.traffic_led_y_pin, GPIO_MODE_OUTPUT);
    if (is_output_capable(pins.traffic_led_g_pin)) gpio_set_direction(pins.traffic_led_g_pin, GPIO_MODE_OUTPUT);
    if (is_output_capable(pins.traffic_led_b_pin)) gpio_set_direction(pins.traffic_led_b_pin, GPIO_MODE_OUTPUT);
    if (is_output_capable(pins.boat_led_r_pin)) gpio_set_direction(pins.boat_led_r_pin, GPIO_MODE_OUTPUT);
    if (is_output_capable(pins.boat_led_y_pin)) gpio_set_direction(pins.boat_led_y_pin, GPIO_MODE_OUTPUT);
    if (is_output_capable(pins.boat_led_g_pin)) gpio_set_direction(pins.boat_led_g_pin, GPIO_MODE_OUTPUT);
    if (is_output_capable(pins.boat_led_b_pin)) gpio_set_direction(pins.boat_led_b_pin, GPIO_MODE_OUTPUT);
    
    // Initialize ADC for hall effect sensors
    init_adc_for_hall_sensors();
    
    traffic_led_off();
    boat_led_off();

    // Install GPIO ISR service
    gpio_install_isr_service(0);

    // Attach interrupt to encoder pin A
    gpio_isr_handler_add(pins.motor_enca_pin, motor_encoder_isr, NULL);
    gpio_set_intr_type(pins.motor_enca_pin, GPIO_INTR_ANYEDGE);

    // Initialize WiFi
    wifi_init_softap();

    // Start web server
    httpd_handle_t server = start_webserver();
    if (server) {
        ESP_LOGI(TAG, "Web server started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to start web server");
    }

    // Start boat sensor measurement task (used for calibration and detection)
    // Increased stack size to reduce risk of stack overflow observed in logs
    xTaskCreate(ultrasonic_task, "boat_sensor_task", 4096, NULL, 5, &ultrasonic_task_handle);

    // Start motor control task
    xTaskCreate(motor_task, "motor_task", 2048, NULL, 5, &motor_task_handle);

    // Start bridge control task
    xTaskCreate(bridge_task, "bridge_task", 4096, NULL, 5, &bridge_task_handle);

    // Test motor at startup
    ESP_LOGI(TAG, "Waiting for system stabilization...");
    vTaskDelay(3000 / portTICK_PERIOD_MS); // Wait 3 seconds

    // ESP_LOGI(TAG, "Starting motor test sequence");

    // Forward at 30%
    // motor_set_direction(true);
    // motor_set_speed(30);
    // motor_start();
    // vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Increase to 60%
    // motor_set_speed(60);
    // vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Reverse at 40%
    // motor_set_direction(false);
    // motor_set_speed(40);
    // vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Stop
    // motor_stop();
    // ESP_LOGI(TAG, "Motor test sequence completed");

    // Keep the main task alive
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}