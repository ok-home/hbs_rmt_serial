idf_component_register(SRCS "parttool_main.c"
                    REQUIRES esp_partition
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS ".")
