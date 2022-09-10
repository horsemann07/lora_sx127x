set(component_srcs "sx127x.c" "sx127x_spi.c")

idf_component_register(SRCS "${component_srcs}"
                       PRIV_REQUIRES driver utils
                       INCLUDE_DIRS ".")
