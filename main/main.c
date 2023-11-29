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

#define SENSOR1     CONFIG_GPIO_INPUT_0
//#define SENSOR2     CONFIG_GPIO_INPUT_1
//#define SENSOR3     CONFIG_GPIO_INPUT_2
#define GPIO_INPUT_PIN_SEL  (1ULL<<SENSOR1)
/*
 * Let's say, GPIO_INPUT_IO_0=4, GPIO_INPUT_IO_1=5
 * In binary representation,
 * 1ULL<<GPIO_INPUT_IO_0 is equal to 0000000000000000000000000000000000010000 and
 * 1ULL<<GPIO_INPUT_IO_1 is equal to 0000000000000000000000000000000000100000
 * GPIO_INPUT_PIN_SEL                0000000000000000000000000000000000110000
 * */
#define ESP_INTR_FLAG_DEFAULT 0



static char tag[] = "LCD test";
void LCD_Display(void* param);


static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR sensors_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void sensor_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}





void app_main(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of falling edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    //change gpio interrupt type for one pin
    //gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
    xTaskCreate(LCD_Display, "LCD Display Task", 2048, NULL, 5, NULL);
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start task
    xTaskCreate(sensor_task, "water sensor task", 2048, NULL, 10, NULL);
    



    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(SENSOR1, sensors_isr_handler, (void*) SENSOR1);


    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

    int cnt = 0;
    while(1) {
        printf("cnt: %d\n", cnt++);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}


void LCD_Display(void* param)
{
	char menuItem1[] = "Sensors";
	char menuItem2[] = "Ball valve";
	char menuItem3[] = "Power";
	char menuItem4[] = "Errors";


	uint8_t frame = 1;
    int row = 0, col = 0;
    //char txtBuf[8];
    LCD_home();
    LCD_clearScreen();
    LCD_writeStr("--- 16x2 LCD ---");
    LCD_setCursor(0, 1);
    LCD_writeStr("Water protection");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    LCD_home();
    LCD_clearScreen();
    while (true) {
		for(int i=0; i<3; i++){
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

    	vTaskDelay(5000 / portTICK_PERIOD_MS);
		LCD_clearScreen();
		frame = i+1;
		}

		/*
		switch(frame){
			case 1:
				LCD_setCursor(0, 0);
				LCD_writeStr(&menuItem1);
				LCD_setCursor(0, 1);
				LCD_writeStr(&menuItem2);
				LCD_clearScreen();
				break;
			case 2:
				LCD_setCursor(0, 0);
				LCD_writeStr(&menuItem2);
				LCD_setCursor(0, 1);
				LCD_writeStr(&menuItem3);
				LCD_clearScreen();
				break;
			case 3:
				LCD_setCursor(0, 0);
				LCD_writeStr(&menuItem3);
				LCD_setCursor(0, 1);
				LCD_writeStr(&menuItem4);
				LCD_clearScreen();
				break;
		}*/
    }
}
