idf_component_register(SRCS "hbs_rmt_serial_example.c"
                    REQUIRES 
                        logic_analyzer 
                        driver 
                        hbs_rmt_serial
                    INCLUDE_DIRS ".")
