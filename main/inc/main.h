#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>
#include "esp_heap_caps.h"

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <uros_network_interfaces.h>
#include <sensor_msgs/msg/image.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/compressed_image.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "../inc/camera.h"
// #include "../inc/storage.h"

// Micro-ROS definitions
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

void FreeRTOS_Init();
void init_microROS();
void TaskTakePicture(void *argument);
void TaskSendPicture(void *argument);

rcl_node_t esp32_cam_node;

rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rmw_init_options_t* rmw_options;

rcl_publisher_t image_pub;
