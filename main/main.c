/* water leakage protection
 * Display MCU
 * ESP32 DevkitC v4 - Интерфейсный контроллер
 * Задачи:
 *  - отображение на дисплее меню по управлению устройством.
 *  - связь с беспроводными датчиками по BLE.
 *  - обмен данными с mmcu-главным контроллером (stm32f103) по i2c.
 *  - обмен данными с pmcu  (stm32f103) по i2c.
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
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"


#include <driver/i2c_master.h>
#include <esp_log.h>
#include "sdkconfig.h"
/* Component includes */
#include "esp32-i2c-hd44780-pcf8574.h"



/**** I2C abonents ****/
/*** LCD Display ***/
#define I2C_MASTER_SDA_IO	CONFIG_I2C_MASTER_SDA 
#define I2C_MASTER_SCL_IO 	CONFIG_I2C_MASTER_SCL
#define I2C_MASTER_FREQUENCY	CONFIG_I2C_MASTER_FREQUENCY
#define I2C_LCD_DEVICE_ADDRESS	CONFIG_I2C_LCD_DEVICE_ADDRESS 

/*** Main MCU ***/
#define I2C_MASTER_SDA_IO_link		CONFIG_I2C_MASTER_SDA_LINK 
#define I2C_MASTER_SCL_IO_link 		CONFIG_I2C_MASTER_SCL_LINK
#define I2C_MASTER_FREQUENCY_link	CONFIG_I2C_MASTER_FREQUENCY_LINK
#define I2C_mMCU_DEVICE_ADDRESS		CONFIG_I2C_mMCU_DEVICE_ADDRESS 
#define LCD_COLS		 	CONFIG_LCD_COLS
#define LCD_ROWS 			CONFIG_LCD_ROWS




/****** keyboard ******/
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

/* LCD menu */
	char menuItem1[] = "Sensors";
	char menuItem2[] = "Ball valve";
	char menuItem3[] = "Power";
	char menuItem4[] = "I2C Link";
	char menuItem11[] = "wired sensors";
	char menuItem12[] = "ble sensors";
	char menuItem13[] = "bs1";
	char menuItem14[] = "bs2";
	char menuItem15[] = "main MCU";
	char menuItem16[] = "power MCU";
	char arrow[] = "<-";
	char state_ok[] = "OK";
	char state_nok[] = "NOK";

static const char* TAG = "main";
enum state // состояние
{
	OPEN,   // открыт
	CLOSED, // закрыт
	OPENS,	// открывается
	CLOSES	// закрывается
};

enum power{
	BAT,
	LINE
};


struct status_mmcu{
	uint8_t i2c_link_mmcu;//0 - not connected, 1 - connected
	uint8_t sensor1;	//0 - not detected, 1 - detected
	uint8_t sensor2;
	enum state ball_value;//0-open, 1-closed, 2-opens, 3-closes
};

typedef struct{
	uint8_t state_lcd;
	uint8_t backlight;//0-backlight off, 1-backlight on
}lcd_draw;




void Draw_Menu(uint8_t state, struct esp_i2c_hd44780_pcf8574 *i2c_lcd);
void LCD_Display(void *param);


static QueueHandle_t gpio_evt_queue = NULL;
static QueueHandle_t data_for_draw_queue = NULL;
static QueueHandle_t i2c_data_queue_link_status = NULL;
static QueueHandle_t buttons_queue = NULL;
static SemaphoreHandle_t xSemaphore = NULL;


uint8_t timer_state = 0; // 1-active, 0-stoped


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

	

static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    	BaseType_t high_task_awoken = pdFALSE;
	gptimer_stop(timer);
    	timer_state = 0; //stoped
    	xSemaphoreGiveFromISR(xSemaphore, NULL);
	return (high_task_awoken == pdTRUE);
}



void Draw_Menu(uint8_t state, struct esp_i2c_hd44780_pcf8574 *i2c_lcd){


	switch(state){
		case 1:
			esp_i2c_hd44780_pcf8574_clear_display(i2c_lcd);
			esp_i2c_hd44780_pcf8574_cursor_home(i2c_lcd);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, menuItem1);
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0xd);//14 символ в первой строке
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, arrow);
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0x40);//начало второй строки
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, menuItem2);
			break;
		case 2:
			esp_i2c_hd44780_pcf8574_clear_display(i2c_lcd);
			esp_i2c_hd44780_pcf8574_cursor_home(i2c_lcd);		//LCD_setCursor(0, 0);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, menuItem1);	//LCD_writeStr(menuItem1)
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0x40);	//LCD_setCursor(0, 1)
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, menuItem2);	//LCD_writeStr(menuItem2)
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0x4d);	//LCD_setCursor(14, 1)
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, arrow);	//LCD_writeStr("<-");
			break;
		case 3:
			esp_i2c_hd44780_pcf8574_clear_display(i2c_lcd);
			esp_i2c_hd44780_pcf8574_cursor_home(i2c_lcd);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, menuItem3);
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0xd);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, arrow);
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0x40);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, menuItem4);
			break;
		case 4:
			esp_i2c_hd44780_pcf8574_clear_display(i2c_lcd);
			esp_i2c_hd44780_pcf8574_cursor_home(i2c_lcd);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, menuItem3);
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0x40);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, menuItem4);
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0x4d);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, arrow);
			break;
		case 11:
			esp_i2c_hd44780_pcf8574_clear_display(i2c_lcd);
			esp_i2c_hd44780_pcf8574_cursor_home(i2c_lcd);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, menuItem11);
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0xd);//14 символ в первой строке
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, arrow);
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0x40); //начало второй строки
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, menuItem12);
			break;
		case 12:
			esp_i2c_hd44780_pcf8574_clear_display(i2c_lcd);
			esp_i2c_hd44780_pcf8574_cursor_home(i2c_lcd);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, menuItem11);
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0x40);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, menuItem12);
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0x4d);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, arrow);
			break;
		case 41:
			esp_i2c_hd44780_pcf8574_clear_display(i2c_lcd);
			esp_i2c_hd44780_pcf8574_cursor_home(i2c_lcd);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, menuItem15);
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0x0d);
		//	if(i2c_link_mMCU){
				esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, state_ok);
//			}
//			else{
				esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, state_nok);
//			}
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0x40);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, menuItem16);
			esp_i2c_hd44780_pcf8574_set_cursor_pos(i2c_lcd, 0x4d);
			esp_i2c_hd44780_pcf8574_send_str(i2c_lcd, state_nok);
			/*if(i2c_link_mMCU){
				
			}*/
			break;
	}

}




void I2C_Link(void* param){
	struct status_mmcu status_for_lcd = {0, 0, 0, 1};//i2c - not connected, s1-not detected, s2-not detected, bv-closed
	esp_err_t status;
	lcd_draw data_draw; 
	portBASE_TYPE xStatusReceive;
	/***** I2C Device Configuration  *****/
	/* 1. Configure the i2c master bus */
	i2c_master_bus_config_t i2c_mst_config = {
    		.clk_source = I2C_CLK_SRC_DEFAULT,
    		.i2c_port = 1,
    		.scl_io_num = I2C_MASTER_SCL_IO,
    		.sda_io_num = I2C_MASTER_SDA_IO,
    		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = false,
	};

	i2c_master_bus_handle_t bus_handle;

	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
	
	/* 2a. Configure the i2c LCD peripheral */
	i2c_device_config_t lcd_dev_cfg = {
    		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
    		.device_address = I2C_LCD_DEVICE_ADDRESS,
    		.scl_speed_hz = I2C_MASTER_FREQUENCY,
	};

	i2c_master_dev_handle_t lcd_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &lcd_dev_cfg, &lcd_handle));
	

	 printf("probing for lcd i2c device...\n"); 
	 status = i2c_master_probe(bus_handle, 0x3F, 300);
	 if (status == ESP_OK){ 
	 	printf("LCD PCF8574AT device is found!\n");
	 }
	 else {
		 printf("LCD PCF8574AT device is not found!\n");
	 }

	/* 2b. Configure the i2c MainMCU peripheral */
	i2c_device_config_t mainMCU_dev_cfg = {
    		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
    		.device_address = I2C_mMCU_DEVICE_ADDRESS,
    		.scl_speed_hz = I2C_MASTER_FREQUENCY_link,
	};

	i2c_master_dev_handle_t mainMCU_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mainMCU_dev_cfg, &mainMCU_handle));
	
	printf("probing for main mcu i2c device...\n"); 
	status = i2c_master_probe(bus_handle, 0x12, 300);
        if(status==ESP_OK){
		printf("main MCU is found!\n"); 
		status_for_lcd.i2c_link_mmcu = 1;//found
	}
	else{
		status_for_lcd.i2c_link_mmcu = 0;//not found
		printf("main MCU is not found!\n"); 
	}

/* 3a. Perform device specific initialization */
	struct esp_i2c_hd44780_pcf8574 i2c_lcd = esp_i2c_hd44780_pcf8574_init(16, 2, -1, LCD_NOBACKLIGHT);
	i2c_lcd.i2c_handle = &lcd_handle;

	/* 3b. Perform the necessary startup instructions for our LCD. */
	esp_i2c_hd44780_pcf8574_begin(&i2c_lcd);

	data_draw.state_lcd = 1;
	Draw_Menu(data_draw.state_lcd, &i2c_lcd);

	while(true){
		xStatusReceive = xQueueReceive(data_for_draw_queue, &data_draw, 50/portTICK_PERIOD_MS); // portMAX_DELAY - пока не получит данные 
		if(xStatusReceive == pdPASS){
			if(data_draw.backlight){
			/*вкл подсветки экрана*/
				i2c_lcd.backlight = LCD_BACKLIGHT;
				esp_i2c_hd44780_pcf8574_clear_display(&i2c_lcd);
			}
			else{
				i2c_lcd.backlight = LCD_NOBACKLIGHT;
				esp_i2c_hd44780_pcf8574_cursor_home(&i2c_lcd);
			}
			Draw_Menu(data_draw.state_lcd, &i2c_lcd);
		}
	}

}






void LCD_Display(void* param)
{
	struct status_mmcu status_for_lcd = {0, 0, 0, 1, 1};//i2c - not connected, s1-not detected, s2-not detected, bv-closed, pt-line
    	uint32_t io_num;
	portBASE_TYPE xStatusReceive;
	uint8_t state = 1;
	uint8_t state_backlight = 0;
	uint8_t change = 0;
	lcd_draw data_draw; 
	
	
	/***GPTimer settings***/
    	ESP_LOGI(TAG, "Create timer handle");
    	gptimer_handle_t gptimer = NULL;
    	gptimer_config_t timer_config = {
        	.clk_src = GPTIMER_CLK_SRC_DEFAULT,
        	.direction = GPTIMER_COUNT_UP,
        	.resolution_hz = 1000000, // 1MHz, 1 tick=1us
    	};
    	ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    
    	gptimer_event_callbacks_t cbs = {
        	.on_alarm = timer_on_alarm_cb,
    	};
    	ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    	ESP_LOGI(TAG, "Enable timer");
    	ESP_ERROR_CHECK(gptimer_enable(gptimer));

    	ESP_LOGI(TAG, "Start timer, stop it at alarm event");
    	gptimer_alarm_config_t alarm_config = {
        	.reload_count = 0,
		.alarm_count = 5000000, // period = 5 sec
		.flags.auto_reload_on_alarm = true, // enable auto-reload
    	};
    	ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));


    	vTaskDelay(100 / portTICK_PERIOD_MS); 

	
	state = 1;
    	while (true) {
		io_num = 0;
		/*отключение подсветки, после того как таймер отсчитает 5 sec*/
		if(xSemaphoreTake(xSemaphore, 300/portTICK_PERIOD_MS))
     		{
			ESP_LOGI(TAG, "[LCD Display] semathore take");	
			state_backlight = 0;
		        data_draw.backlight = state_backlight;
			data_draw.state_lcd = state;	
			xQueueSendToBack(data_for_draw_queue, &data_draw, 150/portTICK_PERIOD_MS);
     		}



		/* Была ли нажата кнопка на клавиатуре...*/
		xStatusReceive = xQueueReceive(buttons_queue, &io_num, 50/portTICK_PERIOD_MS); // блокирует работу других задач portMAX_DELAY - пока не получит данные 
		if(xStatusReceive == pdPASS){
    			vTaskDelay(500 / portTICK_PERIOD_MS);
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
			/*вкл подсветки экрана*/
			state_backlight = 1;
		        data_draw.backlight = state_backlight;	
			
			if(timer_state){
				gptimer_set_raw_count(gptimer, 1);
			}


			/*отрисовка меню*/
			switch(state){
				case 1:
					/* Sensors	<-
					 * Ball valve
					 * */
					if(io_num == 25) {
						state = 2;
					}
					else if(io_num == 33){
						state = 11;
					}
					break;
				case 2:
					/* Sensors	
					 * Ball valve	<-
					 * */
					if(io_num == 25){
				   		state = 3;
					}
					else if (io_num == 32){
				   		state = 1;
					}
					break;
				case 3:
					/* Power	<-
					 * I2C Link
					 */
					if(io_num == 25){
				   		state = 4;
						//Draw_Menu(state, &i2c_lcd, NULL);
					}
					else if(io_num == 32) {
						state = 2;
					}
					break;
				case 4:
					/* Power	
					 * I2C Link	<-
					 */
					if(io_num == 32){
				   		state = 3;
					}
					if(io_num == 33){
				   		state = 41;
					}
					break;
				case 11:
					/* wired sensors	<-
					 * BLE sensors
					 */
					if(io_num == 25){
				   		state = 12;
					}
					else if(io_num == 26){
						state = 1;
					}
					break;
				case 12:
					/* wired sensors	
					 * BLE sensors	<-
					 */
					if(io_num == 32){
						state = 11;
					}
					else if(io_num == 26){
						state = 1;
					}
					break;
				case 41:
					/* main MCU - ok/nok
					 * power MCU - ok/nok
					 */
					//xQueueReceive(i2c_data_queue_link_status, &state_link, 50/portTICK_PERIOD_MS); // portMAX_DELAY - пока не получит данные 
					if(io_num == 26){
						state = 4;
					}
					break;
			}

			change = 0;
			ESP_LOGI(TAG, "[LCD Display] state = %d", state);
			data_draw.state_lcd = state;	
			xQueueSendToBack(data_for_draw_queue, &data_draw, 150/portTICK_PERIOD_MS);
		}
		else {
			if(state_backlight && (timer_state == 0)){ //если подсветка включена и таймер ещё не запущен, тогда вкл таймер 
    				ESP_ERROR_CHECK(gptimer_start(gptimer));//начать счёт 5 секунд
				ESP_LOGI(TAG, "[LCD Display] timer start");	
				timer_state = 1;
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

    
/***hook isr handler for specific gpio pin***/
	gpio_isr_handler_add(GPIO_DOWN, gpio_isr_handler, (void*) GPIO_DOWN);
	gpio_isr_handler_add(GPIO_UP, gpio_isr_handler, (void*) GPIO_UP);
	gpio_isr_handler_add(GPIO_OK, gpio_isr_handler, (void*) GPIO_OK);
	gpio_isr_handler_add(GPIO_ESC, gpio_isr_handler, (void*) GPIO_ESC);

/***create a queue to handle gpio event from isr***/
    	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	buttons_queue = xQueueCreate(10, sizeof(uint32_t));
	data_for_draw_queue = xQueueCreate(5, sizeof(lcd_draw));
	i2c_data_queue_link_status = xQueueCreate(5, sizeof(uint8_t));


    xSemaphore = xSemaphoreCreateBinary();

/***** start tasks *************************/

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    xTaskCreate(I2C_Link, "i2c link Task", 3048, NULL, 5, NULL);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    xTaskCreate(LCD_Display, "LCD Display Task", 3048, NULL, 5, NULL);
    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

}
