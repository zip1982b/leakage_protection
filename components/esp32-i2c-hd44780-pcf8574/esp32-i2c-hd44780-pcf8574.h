#ifndef __ESP32_I2C_LCD1602_
#define __ESP32_I2C_LCD1602_

#include <stdio.h>
#include <inttypes.h>

#include "driver/i2c_master.h"


/* LCD Constants */
/* Constants for command types (page 24 of HD44780 datasheet) */
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYONOFFCONTROL 0x08
#define LCD_CURSORDISPLAYSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

/* Constants for entry mode set command (page 26 of HD44780 datasheet) */
#define LCD_ENTRYINCREMENT 0x02
#define LCD_ENTRYDECREMENT 0x00
#define LCD_ENTRYSHIFT 0x01
#define LCD_ENTRYNOSHIFT 0x00

/* Constants for the display control command (page 26 of HD44780 datasheet) */
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

/* Constants for the cursor shift command (page 27 of HD44780 datasheet) */
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

/* Constants for the function set command (page 27 of HD44780 datasheet) */
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

/* Constants for backlight */
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

/* Constants for the Enable bit, the Read/Write bit, and the Register Select
 * bit */
#define E 0x04
#define Rw 0x02
#define Rs 0x01


struct esp_i2c_hd44780_pcf8574 {
	i2c_master_dev_handle_t *i2c_handle;
	uint8_t address;
	uint8_t columns;
	uint8_t rows;
	uint8_t dotsize;
	uint8_t backlight;
	uint8_t entry_shift;
	uint8_t entry_shift_increment;
};


struct esp_i2c_hd44780_pcf8574 esp_i2c_hd44780_pcf8574_init(uint8_t columns, uint8_t rows, uint8_t dotsize, uint8_t backlight);

void esp_i2c_hd44780_pcf8574_begin(struct esp_i2c_hd44780_pcf8574 *esp_i2c_lcd);

void esp_i2c_hd44780_pcf8574_clear_display(struct esp_i2c_hd44780_pcf8574 *esp_i2c_lcd);

void esp_i2c_hd44780_pcf8574_set_cursor_pos(struct esp_i2c_hd44780_pcf8574 *esp_i2c_lcd, uint8_t ac);

void esp_i2c_hd44780_pcf8574_cursor_home(struct esp_i2c_hd44780_pcf8574 *esp_i2c_lcd);

void esp_i2c_hd44780_pcf8574_entry_mode_set(struct esp_i2c_hd44780_pcf8574 *esp_i2c_lcd, uint8_t
	increment, uint8_t shift);

/** Set the display controls for the i2c LCD */
void esp_i2c_hd44780_pcf8574_display_control(struct esp_i2c_hd44780_pcf8574 *esp_i2c_lcd,
	uint8_t display, uint8_t cursor, uint8_t cursorblinking);

void esp_i2c_hd44780_pcf8574_shift(struct esp_i2c_hd44780_pcf8574 *esp_i2c_lcd, uint8_t screen_cursor,
	uint8_t right_left);

void esp_i2c_hd44780_pcf8574_function_set(struct esp_i2c_hd44780_pcf8574 *esp_i2c_lcd,
	uint8_t data_length, uint8_t display_lines, uint8_t font);

void esp_i2c_hd44780_pcf8574_send_char(struct esp_i2c_hd44780_pcf8574 *esp_i2c_lcd, char c);

void esp_i2c_hd44780_pcf8574_send_str(struct esp_i2c_hd44780_pcf8574 *esp_i2c_lcd, char *str);

void esp_i2c_hd44780_pcf8574_set_backlight(struct esp_i2c_hd44780_pcf8574 *esp_i2c_lcd, uint8_t
	backlight);

void esp_i2c_hd44780_pcf8574_write_to_expander(struct esp_i2c_hd44780_pcf8574 *esp_i2c_lcd,
	uint8_t data_and_mode);

void esp_i2c_hd44780_pcf8574_send_nibble(struct esp_i2c_hd44780_pcf8574 *esp_i2c_lcd,
	uint8_t data_and_mode);

void esp_i2c_hd44780_pcf8574_send_byte_4bitmode(struct esp_i2c_hd44780_pcf8574 *esp_i2c_lcd,
	uint8_t data, uint8_t mode);

uint8_t set_mode(uint8_t rs, uint8_t rw);

#endif
