#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "driver/i2c.h"

// Khởi tạo LCD với địa chỉ I2C (thường là 0x27 hoặc 0x3F)
void lcd_init(uint8_t addr, i2c_port_t port);
// In 1 chuỗi ra màn hình
void lcd_puts(const char *s);
// Xóa màn hình
void lcd_clear(void);
// Đưa con trỏ đến vị trí Cột (col) và Hàng (row)
void lcd_set_cursor(uint8_t col, uint8_t row);

#endif