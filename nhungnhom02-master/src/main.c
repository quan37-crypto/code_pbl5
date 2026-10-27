#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "dht11.h"
#include "i2c_lcd.h"

static const char *TAG = "SMART_FARM_FULL";

// --- CẤU HÌNH PIN (Giữ nguyên pin cũ & bổ sung pin cho đủ 6 phân hệ) ---
#define DHT11_PIN           GPIO_NUM_4
#define RELAY_FAN_PIN       GPIO_NUM_25 
#define RELAY_PUMP_PIN      GPIO_NUM_26 
#define RELAY_HEATER_PIN    GPIO_NUM_27  // Đèn sưởi lò ấp (Hysteresis)
#define RELAY_LIGHT_PIN     GPIO_NUM_14  // Đèn chiếu sáng chuồng (LDR)
#define SERVO_FEED_PIN      GPIO_NUM_19  // Động cơ Servo mở máng ăn (PWM)

#define FIRE_GAS_SENSOR_PIN GPIO_NUM_2   // Cảm biến Lửa / Khí Gas MQ-2 (Active Low: 0 = Có cháy)
#define BUZZER_PIN          GPIO_NUM_23    
#define WATER_POWER_PIN     GPIO_NUM_18
#define WATER_SIGNAL_PIN    GPIO_NUM_35
#define LDR_ADC_CHANNEL     ADC_CHANNEL_6 // GPIO_NUM_34 (Đọc ADC quang trở LDR)

// Nút nhấn
#define BTN_MODE_PIN        GPIO_NUM_13 
#define BTN_FAN_PIN         GPIO_NUM_32 
#define BTN_PUMP_PIN        GPIO_NUM_15 

#define I2C_MASTER_NUM      I2C_NUM_0
#define LCD_ADDR            0x27 

// --- CÁC NGƯỠNG ĐIỀU KHIỂN (THRESHOLDS & HYSTERESIS) ---
#define MAX_PUMP_TIME       120   // Bảo vệ bơm tối đa 120s
#define FEED_INTERVAL_SEC   15   // Chu kỳ cho ăn tự động (ví dụ: 300s = 5 phút/lần khi test mô hình)
#define FEED_DURATION_SEC   5     // Thời gian mở máng xả thức ăn (5 giây)
#define TEMP_FAN_THRESHOLD  38.0f // Ngưỡng bật quạt làm mát chuồng
#define TEMP_INCUBATOR_SET  37.5f // Nhiệt độ chuẩn lò ấp
#define TEMP_HYSTERESIS     0.5f  // Dải trễ Hysteresis (+/- 0.5 độ C)
#define LDR_DARK_THRESHOLD  2000  // Ngưỡng ADC nhận biết trời tối (0 - 4095)

typedef struct {
    float temp, hum;
    int ldr_value;
    bool is_water_low;
    bool fire_detected;
    int pump_time_left;
    bool pump_emergency_stop;
    bool is_manual_mode;
    bool fan_state;
    bool pump_state;
    bool heater_state;
    bool light_state;
    bool is_feeding;
} SystemState_t;

QueueHandle_t state_queue;
SystemState_t current_state = {0};
TaskHandle_t control_task_handle = NULL;
adc_oneshot_unit_handle_t adc1_handle;

int pump_timer_sec = 0;
int feed_timer_sec = 0;
volatile bool mode_interrupted = false;
volatile bool timer_tick_1s = false;

// --- ĐIỀU KHIỂN GÓC QUAY SERVO (0 - 180 độ) ---
void servo_set_angle(int angle) {
    // Chu kỳ 20ms (50Hz), độ phân giải 13-bit (8192)
    // 0.5ms (0 độ) ~ 205, 2.5ms (180 độ) ~ 1024
    uint32_t duty = 205 + ((angle * (1024 - 205)) / 180);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// --- 1. HÀM XỬ LÝ NGẮT NGOÀI (ISR) ---
static void IRAM_ATTR btn_mode_isr_handler(void* arg) {
    static uint32_t last_time = 0;
    uint32_t now = xTaskGetTickCountFromISR();
    if (now - last_time > pdMS_TO_TICKS(250)) {
        mode_interrupted = true; 
        last_time = now;
        BaseType_t high_task_awoken = pdFALSE;
        vTaskNotifyGiveFromISR(control_task_handle, &high_task_awoken);
        if (high_task_awoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

// --- 2. HÀM NGẮT TIMER (1 GIÂY) ---
static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    timer_tick_1s = true;
    BaseType_t high_task_awoken = pdFALSE;
    vTaskNotifyGiveFromISR(control_task_handle, &high_task_awoken);
    return high_task_awoken == pdTRUE;
}

// --- 3. KHỞI TẠO PHẦN CỨNG ---
void init_hardware(void) {
    ESP_LOGI(TAG, "Khoi tao phan cung 6 phan he...");

    gpio_set_direction(FIRE_GAS_SENSOR_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(WATER_SIGNAL_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(WATER_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY_FAN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY_PUMP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY_HEATER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAY_LIGHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);

    // Khởi tạo PWM (LEDC) cho Servo cho ăn
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_FEED_PIN,
        .duty           = 205, // Đóng máng ăn (0 độ)
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    // Khởi tạo ADC1 cho cảm biến ánh sáng LDR
    adc_oneshot_unit_init_cfg_t init_config1 = {.unit_id = ADC_UNIT_1};
    adc_oneshot_new_unit(&init_config1, &adc1_handle);
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(adc1_handle, LDR_ADC_CHANNEL, &config);

    // Cấu hình Nút nhấn 32, 15 (Polling)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_FAN_PIN) | (1ULL << BTN_PUMP_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Cấu hình Nút nhấn 13 (Ngắt ngoài)
    gpio_config_t intr_conf = {
        .pin_bit_mask = (1ULL << BTN_MODE_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&intr_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_MODE_PIN, btn_mode_isr_handler, (void*) BTN_MODE_PIN);
}

// --- 4. TASK ĐIỀU KHIỂN ĐA PHÂN HỆ ---
void system_control_task(void *pvParameters) {
    bool last_f_btn = 1, last_p_btn = 1;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Xử lý chuyển chế độ Auto / Manual
        if (mode_interrupted) {
            current_state.is_manual_mode = !current_state.is_manual_mode;
            current_state.fan_state = false;
            current_state.pump_state = false;
            current_state.pump_emergency_stop = false;
            pump_timer_sec = 0;
            mode_interrupted = false; 
        }

        // --- LUỒNG QUÉT CẢM BIẾN ---
        dht11_reading r1 = dht11_read(DHT11_PIN);
        if (r1.temperature != -1) {
            current_state.temp = r1.temperature;
            current_state.hum = r1.humidity;
        }

        gpio_set_level(WATER_POWER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        current_state.is_water_low = (gpio_get_level(WATER_SIGNAL_PIN) == 0);
        gpio_set_level(WATER_POWER_PIN, 0);

        adc_oneshot_read(adc1_handle, LDR_ADC_CHANNEL, &current_state.ldr_value);
        current_state.fire_detected = (gpio_get_level(FIRE_GAS_SENSOR_PIN) == 0);

        // --- LUỒNG GIÁM SÁT AN TOÀN & LIÊN ĐỘNG KHẨN CẤP (Mục 2.3.1.4) ---
        if (current_state.fire_detected) {
            // Cắt toàn bộ phụ tải không cần thiết
            current_state.fan_state = false;
            current_state.heater_state = false;
            current_state.light_state = false;
            servo_set_angle(0);

            // Kích hoạt bơm dập lửa và còi Buzzer
            current_state.pump_state = true;
            gpio_set_level(RELAY_FAN_PIN, 0);
            gpio_set_level(RELAY_HEATER_PIN, 0);
            gpio_set_level(RELAY_LIGHT_PIN, 0);
            gpio_set_level(RELAY_PUMP_PIN, 1);
            gpio_set_level(BUZZER_PIN, 1);
        } else {
            // --- XỬ LÝ 5 PHÂN HỆ KHI BÌNH THƯỜNG ---
            if (current_state.is_manual_mode) {
                // Chế độ Thủ công (Manual)
                bool f_btn = gpio_get_level(BTN_FAN_PIN);
                bool p_btn = gpio_get_level(BTN_PUMP_PIN);
                if (last_f_btn == 1 && f_btn == 0) current_state.fan_state = !current_state.fan_state;
                if (last_p_btn == 1 && p_btn == 0) current_state.pump_state = !current_state.pump_state;
                last_f_btn = f_btn; 
                last_p_btn = p_btn;

                if (current_state.pump_emergency_stop) {
                    current_state.pump_state = false;
                }
            } else {
                // Chế độ Tự động (Auto)
                // 1. Phân hệ quạt thông gió (Threshold Control)
                current_state.fan_state = (current_state.temp >= TEMP_FAN_THRESHOLD);

                // 2. Phân hệ cấp nước (Threshold + Timer Protection)
                if (current_state.is_water_low && !current_state.pump_emergency_stop) {
                    current_state.pump_state = true;
                } else {
                    current_state.pump_state = false;
                }

                // 3. Phân hệ lò ấp trứng (Hysteresis Control: T_set +/- Delta_T)
                if (current_state.temp < (TEMP_INCUBATOR_SET - TEMP_HYSTERESIS)) {
                    current_state.heater_state = true;
                } else if (current_state.temp >= (TEMP_INCUBATOR_SET + TEMP_HYSTERESIS)) {
                    current_state.heater_state = false;
                }

                // 4. Phân hệ chiếu sáng chuồng (LDR ADC)
                current_state.light_state = (current_state.ldr_value > LDR_DARK_THRESHOLD);
            }

            // Xuất tín hiệu ra Relay
            gpio_set_level(RELAY_FAN_PIN, current_state.fan_state);
            gpio_set_level(RELAY_PUMP_PIN, current_state.pump_state);
            gpio_set_level(RELAY_HEATER_PIN, current_state.heater_state);
            gpio_set_level(RELAY_LIGHT_PIN, current_state.light_state);

            // --- ĐẾM ĐỊNH THỜI 1 GIÂY (Cho ăn & Bảo vệ bơm) ---
            if (timer_tick_1s) {
                // 5. Phân hệ cho ăn định lượng (Servo Timer)
                feed_timer_sec++;
                if (feed_timer_sec >= FEED_INTERVAL_SEC && feed_timer_sec < (FEED_INTERVAL_SEC + FEED_DURATION_SEC)) {
                    servo_set_angle(90); // Mở máng ăn
                    current_state.is_feeding = true;
                } else if (feed_timer_sec >= (FEED_INTERVAL_SEC + FEED_DURATION_SEC)) {
                    servo_set_angle(0);  // Đóng máng ăn
                    current_state.is_feeding = false;
                    feed_timer_sec = 0;
                }

                // Đếm thời gian chạy máy bơm
                if (current_state.pump_state) {
                    pump_timer_sec++;
                } else {
                    pump_timer_sec = 0;
                }
                timer_tick_1s = false;
            }

            // Kiểm tra quá thời gian bơm (Overtime >= 120s)
            if (pump_timer_sec >= MAX_PUMP_TIME) {
                current_state.pump_emergency_stop = true;
                current_state.pump_state = false;
                gpio_set_level(RELAY_PUMP_PIN, 0);
            }
            if (!current_state.is_water_low && !current_state.pump_state) {
                current_state.pump_emergency_stop = false;
                pump_timer_sec = 0;
            }

            // Buzzer kêu nếu bơm bị lỗi quá tải thời gian
            gpio_set_level(BUZZER_PIN, current_state.pump_emergency_stop ? 1 : 0);
        }

        current_state.pump_time_left = MAX_PUMP_TIME - pump_timer_sec;
        if (current_state.pump_time_left < 0) current_state.pump_time_left = 0;

        xQueueOverwrite(state_queue, &current_state);
    }
}

// --- 5. TASK HIỂN THỊ LCD ---
void lcd_display_task(void *pvParameters) {
    SystemState_t state;
    char line1[17], line2[17];
    while (1) {
        if (xQueueReceive(state_queue, &state, portMAX_DELAY)) {
            snprintf(line1, sizeof(line1), "T:%.0fC H:%.0f%%  %c", state.temp, state.hum, state.is_manual_mode ? 'M' : 'A');
            if (state.fire_detected) {
                snprintf(line2, sizeof(line2), "ALARM: FIRE/GAS!");
            } else if (state.pump_emergency_stop) {
                snprintf(line2, sizeof(line2), "STOP: OVER TIME!");
            } else {
                snprintf(line2, sizeof(line2), "N:%-3s T:%-3ds F:%d", state.is_water_low ? "Can" : "Day", state.pump_time_left, state.is_feeding);
            }
            lcd_set_cursor(0, 0); lcd_puts(line1);
            lcd_set_cursor(0, 1); lcd_puts(line2);
        }
    }
}

// --- KHỞI TẠO CHUNG ---
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER, 
        .sda_io_num = 21, 
        .scl_io_num = 22, 
        .sda_pullup_en = GPIO_PULLUP_ENABLE, 
        .scl_pullup_en = GPIO_PULLUP_ENABLE, 
        .master.clk_speed = 50000
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

void init_hardware_timer(void) {
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, 
        .direction = GPTIMER_COUNT_UP, 
        .resolution_hz = 1000000
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    gptimer_event_callbacks_t cbs = {.on_alarm = timer_on_alarm_cb};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0, 
        .alarm_count = 1000000, 
        .flags.auto_reload_on_alarm = true
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

void app_main(void) {
    init_hardware();
    ESP_ERROR_CHECK(i2c_master_init());
    lcd_init(LCD_ADDR, I2C_MASTER_NUM);
    lcd_clear();

    state_queue = xQueueCreate(1, sizeof(SystemState_t));
    xTaskCreate(system_control_task, "Control", 4096, NULL, 3, &control_task_handle);
    xTaskCreate(lcd_display_task, "LCD", 4096, NULL, 2, NULL);
    init_hardware_timer();
}