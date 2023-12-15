/* water leakage protection
 * Защита от протечки воды.
 * Состав:
 *  3 проводных датчика обнаружения протечки (3 gpio настроенные на прерывания по спадающему фронту)
 *  2 реле для подачи питания на электропривод (открытие/закрытие шарового крана на подачу воды в квартиру)
 *  дисплей HD44780 подключенный по i2c
 *  поворотный энкодер
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


/* 	
	cr	- clockwise rotation
	ccr	- counter clockwise rotation
	bp	- button pressed
*/
enum action{cr, ccr, bp}; 




/**
 * Brief:
 *
 * GPIO status:
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 *
 * Note. These are the default GPIO pins to be used in the example. You can
 * change IO pins in menuconfig.
 *
 *
 */

#define SENSOR1     CONFIG_SENSOR_1
//#define SENSOR2     CONFIG_GPIO_INPUT_1
//#define SENSOR3     CONFIG_GPIO_INPUT_2

#define GPIO_ENC_CLK     35
#define GPIO_ENC_DT		 32
#define GPIO_ENC_SW		 33
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_ENC_CLK) | (1ULL<<GPIO_ENC_DT) | (1ULL<<GPIO_ENC_SW) | (1ULL<<SENSOR1))

/*
 * Let's say, SENSOR1=4, (SENSOR2=5), GPIO_ENC_CLK=35, GPIO_ENC_DT=32, GPIO_ENC_SW=33
 * In binary representation,
 * 1ULL<<SENSOR1 is equal to 			0000000000000000000000000000000000010000 and
 * 1ULL<<GPIO_ENC_DT is equal to 		0000000100000000000000000000000000000000 and
 * 1ULL<<GPIO_ENC_SW is equal to        0000001000000000000000000000000000110000 and
 * 1ULL<<GPIO_ENC_CLK is equal to       0000100000000000000000000000000000110000
 * */
#define ESP_INTR_FLAG_DEFAULT 0



static const char* TAG = "main";

void LCD_Display(void* param);


static QueueHandle_t gpio_evt_queue = NULL;
static QueueHandle_t enc_evt_queue = NULL;
static QueueHandle_t ENC_queue = NULL;




static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
	switch(gpio_num){
		case GPIO_ENC_CLK:
			gpio_set_intr_type(GPIO_ENC_CLK, GPIO_INTR_DISABLE);
    		xQueueSendFromISR(enc_evt_queue, &gpio_num, NULL);
			break;
		case GPIO_ENC_DT:
			gpio_set_intr_type(GPIO_ENC_DT, GPIO_INTR_DISABLE);
    		xQueueSendFromISR(enc_evt_queue, &gpio_num, NULL);
			break;
		case GPIO_ENC_SW:
			gpio_set_intr_type(GPIO_ENC_SW, GPIO_INTR_DISABLE);
    		xQueueSendFromISR(enc_evt_queue, &gpio_num, NULL);
			break;
		case SENSOR1:
			gpio_set_intr_type(SENSOR1, GPIO_INTR_DISABLE);
    		xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
			break;
	}
}




static void ENC(void* arg){
	enum action rotate;
	uint32_t io_num;
	for(;;) {
		io_num = 0;
		xQueueReceive(enc_evt_queue, &io_num, portMAX_DELAY); // the task is blocked until the data arrives
		if(io_num == GPIO_ENC_CLK){
			xQueueReceive(enc_evt_queue, &io_num, 50/portTICK_PERIOD_MS);
			gpio_set_intr_type(GPIO_ENC_CLK, GPIO_INTR_ANYEDGE); //enable
			if (io_num == GPIO_ENC_DT){
				rotate = cr;
				ESP_LOGI(TAG, "[ENC] clockwise rotation");	
				xQueueSendToBack(ENC_queue, &rotate, 50/portTICK_PERIOD_MS); //xStatus = 
				gpio_set_intr_type(GPIO_ENC_DT, GPIO_INTR_ANYEDGE);//enable
			}
		}
		else if(io_num == GPIO_ENC_DT){
			xQueueReceive(enc_evt_queue, &io_num, 50/portTICK_PERIOD_MS);
			gpio_set_intr_type(GPIO_ENC_DT, GPIO_INTR_ANYEDGE);
			if (io_num == GPIO_ENC_CLK){
				rotate = ccr;
				ESP_LOGI(TAG, "[ENC]counter clockwise rotation");	
				xQueueSendToBack(ENC_queue, &rotate, 50/portTICK_PERIOD_MS);//xStatus = 
				gpio_set_intr_type(GPIO_ENC_CLK, GPIO_INTR_ANYEDGE);
			}
		}
		else if(io_num == GPIO_ENC_SW){
			rotate = bp;
			ESP_LOGI(TAG, "[ENC] Button is pressed");	
			vTaskDelay(100 / portTICK_PERIOD_MS);
			xQueueSendToBack(ENC_queue, &rotate, 50/portTICK_PERIOD_MS);//xStatus = 
			gpio_set_intr_type(GPIO_ENC_SW, GPIO_INTR_NEGEDGE);//enable
		}

	}
}





static void sensor_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
			ESP_LOGI(TAG, "[sensor task] GPIO[%"PRIu32"] intr, val: %d", io_num, gpio_get_level(io_num));	
        }
    }
}



void LCD_Display(void* param)
{
	enum action rotate;
    uint32_t io_num;
	portBASE_TYPE xStatusReceive;
	char menuItem1[] = "Sensors";
	char menuItem2[] = "Ball valve";
	char menuItem3[] = "Power";
	char menuItem4[] = "Errors";
	uint8_t change = 1;
	uint8_t down_cw;
	uint8_t up_ccw;
	uint8_t press_button;
	uint8_t frame = 1;
    //int row = 0, col = 0;

    LCD_home();
    LCD_clearScreen();
    LCD_writeStr("--- 16x2 LCD ---");
    LCD_setCursor(0, 1);
    LCD_writeStr("Water protection");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    LCD_home();
    LCD_clearScreen();
    while (true) {
		io_num = 0;
		xStatusReceive = xQueueReceive(ENC_queue, &rotate, 300/portTICK_PERIOD_MS); // portMAX_DELAY - (very long time) сколь угодно долго - 100/portTICK_RATE_MS
		if(xStatusReceive == pdPASS){
			change = 1;
			ESP_LOGI(TAG, "[LCD Display] rotate = %d", rotate);	
			switch(rotate){
				case 0: //down_cw = clockwise
					down_cw = 1;
					frame++;
					break;
				case 1: //up_ccw = counter clockwise
					up_ccw = 1;
					frame--;
					break;
				case 2: //press_button = button pressed
					press_button = 1;
					break;
			}
		}
		if(frame>3) frame = 3;
		else if (frame<1) frame = 1;


		if(change){
			LCD_clearScreen();
			ESP_LOGI(TAG, "[LCD Display] frame = %d", frame);	
			change = 0;
		}	
		switch(frame){
			case 1:
				LCD_setCursor(0, 0);
				LCD_writeStr(menuItem1);
				LCD_setCursor(0, 1);
				LCD_writeStr(menuItem2);
				break;
			case 2:
				LCD_setCursor(0, 0);
				LCD_writeStr(menuItem2);
				LCD_setCursor(0, 1);
				LCD_writeStr(menuItem3);
				break;
			case 3:
				LCD_setCursor(0, 0);
				LCD_writeStr(menuItem3);
				LCD_setCursor(0, 1);
				LCD_writeStr(menuItem4);
				break;
		}
	}
}


void app_main(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    //io_conf.intr_type = GPIO_INTR_DISABLE;
    //disable pull-down mode
    //io_conf.pull_down_en = 0;
    //disable pull-up mode
    //io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    //gpio_config(&io_conf);
    
	//interrupt of falling edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;//GPIO_INTR_NEGEDGE
	//bit mask of the pins use GPIO here
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    //change gpio interrupt type for one pin
    gpio_set_intr_type(SENSOR1, GPIO_INTR_NEGEDGE);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(SENSOR1, gpio_isr_handler, (void*) SENSOR1);
	
	gpio_isr_handler_add(GPIO_ENC_CLK, gpio_isr_handler, (void*) GPIO_ENC_CLK);
    //hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_ENC_DT, gpio_isr_handler, (void*) GPIO_ENC_DT);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_ENC_SW, gpio_isr_handler, (void*) GPIO_ENC_SW);
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    enc_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	ENC_queue = xQueueCreate(10, sizeof(uint32_t));

	//start tasks
    xTaskCreate(sensor_task, "water sensor task", 2048, NULL, 10, NULL);
    
    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
    xTaskCreate(LCD_Display, "LCD Display Task", 2048, NULL, 5, NULL);
	xTaskCreate(ENC, "ENC", 2048, NULL, 11, NULL);

    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());


}
