set(srcs "main.c")

idf_component_register(SRCS "${srcs}"
                    REQUIRES driver
                    PRIV_REQUIRES esp32-i2c-hd44780-pcf8574
                    INCLUDE_DIRS ".")
