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
#include "HD44780.h"



#define LCD_ADDR 0x3F
#define SDA_PIN  18
#define SCL_PIN  19
#define LCD_COLS 16
#define LCD_ROWS 2








#define GPIO_DOWN	     25
#define GPIO_UP			 32
#define GPIO_OK			 33
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_DOWN) | (1ULL<<GPIO_UP) | (1ULL<<GPIO_OK))

/*
 * Let's say, GPIO_DOWN=35, GPIO_UP=32, GPIO_OK=33
 * In binary representation,
 * 1ULL<<GPIO_UP is equal to	 		0000000100000000000000000000000000000000 and
 * 1ULL<<GPIO_OK is equal to	        0000001000000000000000000000000000110000 and
 * 1ULL<<GPIO_DOWN is equal to		    0000100000000000000000000000000000110000
 * */
#define ESP_INTR_FLAG_DEFAULT 0



static const char* TAG = "main";

void LCD_Display(void* param);


static QueueHandle_t gpio_evt_queue = NULL;
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
	}
}












void LCD_Display(void* param)
{
    uint32_t io_num;
	portBASE_TYPE xStatusReceive;
	uint8_t state = 1;
	char menuItem1[] = "Sensors";
	char menuItem2[] = "Ball valve";
	char menuItem3[] = "Power";
	char menuItem4[] = "Errors";
	char menuItem11[] = "ws1";
	char menuItem12[] = "ws2";
	char menuItem13[] = "bs1";
	char menuItem14[] = "bs2";
	char menuItem15[] = "bs3";
	uint8_t change = 1;
	uint8_t cursor = 1;


    LCD_home();
    LCD_clearScreen();
    LCD_writeStr("--- 16x2 LCD ---");
    LCD_setCursor(0, 1);
    LCD_writeStr("leakage protect");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    LCD_home();
    LCD_clearScreen();
    while (true) {
		io_num = 0;
		xStatusReceive = xQueueReceive(buttons_queue, &io_num, 50/portTICK_PERIOD_MS); // portMAX_DELAY - пока не получит данные 
		if(xStatusReceive == pdPASS){
    		vTaskDelay(150 / portTICK_PERIOD_MS);
			change = 1;
			switch(io_num){
				case 25: //down
					gpio_set_intr_type(GPIO_DOWN, GPIO_INTR_NEGEDGE);
					ESP_LOGI(TAG, "[LCD Display] DOWN pressed");	
					cursor++;
					break;
				case 32: //up
					gpio_set_intr_type(GPIO_UP, GPIO_INTR_NEGEDGE);
					ESP_LOGI(TAG, "[LCD Display] UP pressed");	
					cursor--;
					break;
				case 33: //OK
					gpio_set_intr_type(GPIO_OK, GPIO_INTR_NEGEDGE);
					ESP_LOGI(TAG, "[LCD Display] OK pressed");	
					break;
			}
		}
		if(cursor>4) cursor = 4;
		else if (cursor<1) cursor = 1;


		if(change){
			LCD_clearScreen();
			ESP_LOGI(TAG, "[LCD Display] cursor = %d", cursor);	
			ESP_LOGI(TAG, "[LCD Display] state = %d", state);	
			change = 0;
		}	
		

		ESP_LOGI(TAG, "[LCD Display] state = %d", state);	
		switch(state){
			case 1:
				LCD_setCursor(0, 0);
				LCD_writeStr(menuItem1);
				LCD_setCursor(14, 0);
				LCD_writeStr("<-");
				LCD_setCursor(0, 1);
				LCD_writeStr(menuItem2);
				if(io_num == 25) {
					LCD_clearScreen();
					state = 2;
				}
				break;
			case 2:
				LCD_setCursor(0, 0);
				LCD_writeStr(menuItem1);
				LCD_setCursor(0, 1);
				LCD_writeStr(menuItem2);
				LCD_setCursor(14, 1);
				LCD_writeStr("<-");
				if(io_num == 25){
				   	state = 3;
					LCD_clearScreen();
				}
				else if (io_num == 32){
				   	state = 1;
					LCD_clearScreen();
				}
				break;
			case 3:
				LCD_setCursor(0, 0);
				LCD_writeStr(menuItem3);
				LCD_setCursor(14, 0);
				LCD_writeStr("<-");
				LCD_setCursor(0, 1);
				LCD_writeStr(menuItem4);
				if(io_num == 25){
				   	state = 4;
					LCD_clearScreen();
				}
				else if(io_num == 32) {
					state = 2;
					LCD_clearScreen();
				}
				break;
			case 4:
				LCD_setCursor(0, 0);
				LCD_writeStr(menuItem3);
				LCD_setCursor(0, 1);
				LCD_writeStr(menuItem4);
				LCD_setCursor(14, 1);
				LCD_writeStr("<-");
				if(io_num == 32){
				   	state = 3;
					LCD_clearScreen();
				}
				break;
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
    io_conf.pull_up_en = 1;
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
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	buttons_queue = xQueueCreate(10, sizeof(uint32_t));

	//start tasks
    
    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
    xTaskCreate(LCD_Display, "LCD Display Task", 2048, NULL, 5, NULL);

    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());


}
