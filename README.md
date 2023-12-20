| Supported Target | ESP32 DevkitC v4 | 

# Project name: Leakage protection                                       BLE
												                         \|/
										--------------------------------  |
|LCD1602|<-PCF8574AT|                   |            ESP32             |--|
           addr:0x3F|    i2c            |                              |
          slave sda |___________________|18 master                    4|------------sensor1
                scl |-------------------|19                           5|------------sensor2
                                        |                              | 
                          |UP|----------|32                          18|------------relay open valve
              keypad      |OK|----------|33                          19|------------relay close valve
                         |DOWN|---------|35                            |
                                        |                              |
                                        |                              |
                                        |______________________________| 


# Future:
# В будущем планируется использовать BLE для связи с датчиками протечки.
# Также будет добавлен батарейный модуль для питания устройства при отключении питания,
# для того чтобы перекрыть подачу воды в квартиру.

## GPIO functions:

| GPIO                         | Direction | Configuration                                          |
| ---------------------------- | --------- | ------------------------------------------------------ |
| CONFIG_OPEN_VALVE            | output    |                                                        |
| CONFIG_CLOSE_VALVE           | output    |                                                        |
| CONFIG_SENSOR1               | input     | pulled up (external), interrupt from falling edge      |
| CONFIG_SENSOR2               | input     | pulled up (external), interrupt from falling edge      |


### Configure the project

### Build and Flash

Build the project and flash it to the board, then run the monitor tool to view the serial output:

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.




