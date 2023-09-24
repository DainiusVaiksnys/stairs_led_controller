idf_component_register(SRCS "my_server.c"
                    INCLUDE_DIRS "include"
                    EMBED_TXTFILES "fe/index.html"
                    EMBED_FILES "fe/favicon.png"
                    REQUIRES esp_http_server json)
