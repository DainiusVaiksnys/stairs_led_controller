idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES my_wifi my_server led_driver)
