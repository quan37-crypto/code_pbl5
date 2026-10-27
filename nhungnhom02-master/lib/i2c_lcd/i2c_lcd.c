#include "i2c_lcd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

static uint8_t lcd_addr;
static i2c_port_t i2c_num;

// 1. Gửi 1 xung duy nhất (Chỉ dùng lúc đánh thức LCD)
static void lcd_write_nibble(uint8_t nibble) {
    uint8_t data_arr[2];
    data_arr[0] = nibble | 0x0C; // EN=1, Đèn nền=1
    data_arr[1] = nibble | 0x08; // EN=0, Đèn nền=1
    i2c_master_write_to_device(i2c_num, lcd_addr, data_arr, 2, 100 / portTICK_PERIOD_MS);
    ets_delay_us(100); 
}

// 2. Gửi 1 chữ cái hoàn chỉnh (2 nhịp nối liền, chống đứt gãy)
static void lcd_send(uint8_t data, uint8_t rs) {
    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;
    uint8_t data_arr[4];
    
    data_arr[0] = up | rs | 0x0C; // Nhịp 1: High + EN=1
    data_arr[1] = up | rs | 0x08; // Nhịp 1: High + EN=0
    data_arr[2] = lo | rs | 0x0C; // Nhịp 2: Low  + EN=1
    data_arr[3] = lo | rs | 0x08; // Nhịp 2: Low  + EN=0
    
    // Gửi nguyên 4 byte một lúc
    i2c_master_write_to_device(i2c_num, lcd_addr, data_arr, 4, 100 / portTICK_PERIOD_MS);
    ets_delay_us(50);
}

void lcd_clear(void) {
    lcd_send(0x01, 0);
    vTaskDelay(pdMS_TO_TICKS(5)); // Chờ 5ms để xóa xong màn
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_send(0x80 | (col + row_offsets[row]), 0);
}

void lcd_init(uint8_t addr, i2c_port_t port) {
    lcd_addr = addr; 
    i2c_num = port;
    vTaskDelay(pdMS_TO_TICKS(100)); // Đợi điện áp thật ổn định

    // Ép màn hình reset về trạng thái gốc bằng 3 lệnh 0x30
    lcd_write_nibble(0x30);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x30);
    ets_delay_us(150);
    lcd_write_nibble(0x30);
    ets_delay_us(150);

    // Bật chế độ 4-bit
    lcd_write_nibble(0x20); 
    ets_delay_us(150);

    // Cấu hình các thông số
    lcd_send(0x28, 0); // 2 dòng, font 5x8
    lcd_send(0x08, 0); // Tắt màn hình
    lcd_clear();       // Dọn rác
    lcd_send(0x06, 0); // Cập nhật con trỏ
    lcd_send(0x0C, 0); // Bật màn hình, ẩn con trỏ
}

void lcd_puts(const char *s) {
    while (*s) { 
        lcd_send((uint8_t)*s++, 0x01); 
    }
}