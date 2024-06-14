/* water leakage protection
 * ESP32 DevkitC v4 - Интерфейсный контроллер
 * Задачи:
 *  - отображение на дисплее меню по управлению устройством.
 *  - связь с беспроводными датчиками по BLE.
 *  - обмен данными с главным контроллером по i2c.
 * 
 * Состав:
 *  дисплей HD44780 подключенный по i2c
 *  3 кнопки по управлению меню (up - down - ok)
 *
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"


#include <driver/i2c.h>
#include <esp_log.h>
#include "sdkconfig.h"
//#include "HD44780.h"
/* Component includes */
#include "esp32-i2c-hd44780-pcf8574.h"

#define I2C_MASTER_SDA_IO	CONFIG_I2C_MASTER_SDA 
#define I2C_MASTER_SCL_IO 	CONFIG_I2C_MASTER_SCL
#define I2C_MASTER_FREQUENCY	CONFIG_I2C_MASTER_FREQUENCY

#define I2C_LCD_DEVICE_ADDRESS 0x3F
#define LCD_COLS 	CONFIG_LCD_COLS
#define LCD_ROWS 	CONFIG_LCD_ROWS





#define GPIO_DOWN		CONFIG_GPIO_DOWN
#define GPIO_UP			CONFIG_GPIO_UP
#define GPIO_OK			CONFIG_GPIO_OK
#define GPIO_ESC		CONFIG_GPIO_ESC	
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_DOWN) | (1ULL<<GPIO_UP) | (1ULL<<GPIO_OK) | (1ULL<<GPIO_ESC))

/*
 * Let's say, GPIO_DOWN=35, GPIO_UP=32, GPIO_OK=33, GPIO_ESC=26
 * In binary representation,
 * 1ULL<<GPIO_UP is equal to	 		0000000100000000000000000000000000000000 and
 * 1ULL<<GPIO_OK is equal to	        0000001000000000000000000000000000110000 and
 * 1ULL<<GPIO_DOWN is equal to		    0000100000000000000000000000000000110000
 * */
#define ESP_INTR_FLAG_DEFAULT 0

/* menu */
	char menuItem1[] = "Sensors";
	char menuItem2[] = "Ball valve";
	char menuItem3[] = "Power";
	char menuItem4[] = "Errors";
	char menuItem11[] = "wired sensors";
	char menuItem12[] = "ble sensors";
	char menuItem13[] = "bs1";
	char menuItem14[] = "bs2";
	char arrow[] = "<-";

static const char* TAG = "main";

void LCD_Display(void* param);


static QueueHandle_t gpio_evt_queue = NULL;
static QueueHandle_t i2c_data_queue = NULL;
static QueueHandle_t buttons_queue = NULL;




static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
	switch(gpio_num){
		case GPIO_DOWN:
			gpio_set_intr_type(GPIO_DOWN, GPIO_INTR_DISABLE);
    		xQueueSendFromISR(buttons_queue, &gpio_num, NULL);
			break;
		case GPIO_UP:
			gpio_set_intr_type(GPIO_UP, GPIO_INTR_DISABLE);
    		xQueueSendFromISR(buttons_queue, &gpio_num, NULL);
			break;
		case GPIO_OK:
			gpio_set_intr_type(GPIO_OK, GPIO_INTR_DISABLE);
    		xQueueSendFromISR(buttons_queue, &gpio_num, NULL);
			break;
		case GPIO_ESC:
			gpio_set_intr_type(GPIO_ESC, GPIO_INTR_DISABLE);
    		xQueueSendFromISR(buttons_queue, &gpio_num, NULL);
			break;
	}
}

	







void LCD_Display(void* param)
{
    uint32_t io_num;
	portBASE_TYPE xStatusReceive;
	uint8_t state = 1;
	uint8_t change = 0;
	uint8_t counter = 0;

/* I2C Device Configuration {{{ */
	/* 1. Configure the i2c master bus */
	i2c_master_bus_config_t i2c_mst_config = {
    		.clk_source = I2C_CLK_SRC_DEFAULT,
    		.i2c_port = -1,
    		.scl_io_num = I2C_MASTER_SCL_IO,
    		.sda_io_num = I2C_MASTER_SDA_IO,
    		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = false,
	};

	i2c_master_bus_handle_t bus_handle;

	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
	
	/* 2. Configure the i2c LCD peripheral */
	i2c_device_config_t lcd_dev_cfg = {
    		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
    		.device_address = I2C_LCD_DEVICE_ADDRESS,
    		.scl_speed_hz = I2C_MASTER_FREQUENCY,
	};

	i2c_master_dev_handle_t lcd_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &lcd_dev_cfg, &lcd_handle));
	

	/* printf("probing for i2c device..."); */
	/* ESP_ERROR_CHECK(i2c_master_probe(bus_handle, 0x27, -1)); */
	/* printf(" SUCCESS!\n"); */

	/* 3a. Perform device specific initialization */
	struct esp_i2c_hd44780_pcf8574 i2c_lcd = esp_i2c_hd44780_pcf8574_init(16, 2, -1, LCD_NOBACKLIGHT);
	i2c_lcd.i2c_handle = &lcd_handle;

	/* 3b. Perform the necessary startup instructions for our LCD. */
	esp_i2c_hd44780_pcf8574_begin(&i2c_lcd);

    	vTaskDelay(100 / portTICK_PERIOD_MS); 

	
	/* state 1 */
	esp_i2c_hd44780_pcf8574_clear_display(&i2c_lcd);
	esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
	esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem1);
	esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0xd);//14 символ в первой строке
	esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
	esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);//начало второй строки
	esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem2);


    while (true) {
		io_num = 0;
		xStatusReceive = xQueueReceive(buttons_queue, &io_num, 50/portTICK_PERIOD_MS); // portMAX_DELAY - пока не получит данные 
		if(xStatusReceive == pdPASS){
    		vTaskDelay(200 / portTICK_PERIOD_MS);
			change = 1;
			switch(io_num){
				case 25: //down
					gpio_set_intr_type(GPIO_DOWN, GPIO_INTR_NEGEDGE);
					ESP_LOGI(TAG, "[LCD Display] DOWN pressed");
					break;
				case 26: //esc
					gpio_set_intr_type(GPIO_ESC, GPIO_INTR_NEGEDGE);
					ESP_LOGI(TAG, "[LCD Display] ESC pressed");
					break;
				case 32: //up
					gpio_set_intr_type(GPIO_UP, GPIO_INTR_NEGEDGE);
					ESP_LOGI(TAG, "[LCD Display] UP pressed");
					break;
				case 33: //OK
					gpio_set_intr_type(GPIO_OK, GPIO_INTR_NEGEDGE);
					ESP_LOGI(TAG, "[LCD Display] OK pressed");	
					break;
			}
		}


		if(change){
			i2c_lcd.backlight = LCD_BACKLIGHT;
			esp_i2c_hd44780_pcf8574_clear_display(&i2c_lcd); 
			switch(state){
				case 1:
					if(io_num == 25) {
						state = 2;
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);		//LCD_setCursor(0, 0);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem1);	//LCD_writeStr(menuItem1)
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);	//LCD_setCursor(0, 1)
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem2);	//LCD_writeStr(menuItem2)
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x4d);	//LCD_setCursor(14, 1)
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);	//LCD_writeStr("<-");
					}
					else if(io_num == 33){
						state = 11;
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem11);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0xd);//14 символ в первой строке
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40); //начало второй строки
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem12);
					}
					else {
						//i2c_lcd.backlight = LCD_BACKLIGHT;
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem1);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0xd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem2);
					}
					break;
				case 2:
					if(io_num == 25){
				   		state = 3;
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem3);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0xd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem4);
					}
					else if (io_num == 32){
				   		state = 1;
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem1);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0xd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem2);
					}
					else {
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem1);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem2);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x4d);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
					}
					break;
				case 3:
					if(io_num == 25){
				   		state = 4;
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem3);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem4);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x4d);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
					}
					else if(io_num == 32) {
						state = 2;
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem1);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem2);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x4d);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
					}
					else {
					
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem3);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0xd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem4);
					}
					break;
				case 4:
					if(io_num == 32){
				   		state = 3;
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem3);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0xd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem4);
					}
					else {
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem3);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem4);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x4d);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
					}
					break;
				case 11:
					if(io_num == 25){
				   		state = 12;
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem11);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem12);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x4d);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
					}
					else if(io_num == 26){
						state = 1;
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem1);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0xd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem2);
					}
					else {
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem11);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0xd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem12);
					}
					break;
				case 12:
					if(io_num == 32){
						state = 11;
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem11);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0xd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem12);
					}
					else if(io_num == 26){
						state = 1;
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem1);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0xd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem2);
					}
					else {
						esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem11);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x40);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, menuItem12);
						esp_i2c_hd44780_pcf8574_set_cursor_pos(&i2c_lcd, 0x4d);
						esp_i2c_hd44780_pcf8574_send_str(&i2c_lcd, arrow);
					}
					break;
			}

			change = 0;
			ESP_LOGI(TAG, "[LCD Display] state = %d", state);	
		}
		else {
			counter = counter + 1;
			if(counter == 200){
				counter = 0;
				i2c_lcd.backlight = LCD_NOBACKLIGHT;
				esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
			}
		}
		
	}
}


void app_main(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    
	//interrupt of falling edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
	//bit mask of the pins use GPIO here
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    //change gpio interrupt type for one pin
    //gpio_set_intr_type(SENSOR1, GPIO_INTR_NEGEDGE);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
	
	gpio_isr_handler_add(GPIO_DOWN, gpio_isr_handler, (void*) GPIO_DOWN);
    //hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_UP, gpio_isr_handler, (void*) GPIO_UP);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_OK, gpio_isr_handler, (void*) GPIO_OK);
	gpio_isr_handler_add(GPIO_ESC, gpio_isr_handler, (void*) GPIO_ESC);
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	buttons_queue = xQueueCreate(10, sizeof(uint32_t));
	i2c_data_queue = xQueueCreate(10, sizeof(uint32_t));



/***** start tasks *************************/
    xTaskCreate(LCD_Display, "LCD Display Task", 3048, NULL, 5, NULL);
    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

}
