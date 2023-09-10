idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES "certs/servercert.pem"
                                   "certs/prvtkey.pem"
                    REQUIRES my_wifi my_server)
