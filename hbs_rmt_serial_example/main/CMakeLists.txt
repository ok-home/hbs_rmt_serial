idf_component_register(SRCS "hbs_rmt_serial_example.c"
                    REQUIRES 
                        logic_analyzer 
                        driver 
                        uart_rmt
                    INCLUDE_DIRS ".")
