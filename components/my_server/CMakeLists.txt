idf_component_register(SRCS "my_server.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_http_server json)