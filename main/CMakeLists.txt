idf_component_register(
                        SRCS 
                          src/main.c
                          src/camera.c
                        INCLUDE_DIRS "inc"
                        PRIV_INCLUDE_DIRS .
                        PRIV_REQUIRES micro_ros_espidf_component nvs_flash esp32-camera)

# set(PARTITION_TABLE_CSV ${CMAKE_SOURCE_DIR}/partitions.csv)
