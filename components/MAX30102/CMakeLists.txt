idf_component_register(SRCS "max30102.c" 
                            "i2c-easy.c"
                    INCLUDE_DIRS "include"
                    REQUIRES   driver)