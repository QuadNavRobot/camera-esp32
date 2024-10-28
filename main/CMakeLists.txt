idf_component_register(
                        SRCS "src/main.c"
                        INCLUDE_DIRS "inc"
                        PRIV_INCLUDE_DIRS .
                        PRIV_REQUIRES micro_ros_espidf_component nvs_flash esp32-camera)
