#include "inc/main.h"

void app_main(void)
{
    init_microROS();
    if (ESP_OK != init_camera())
    {
        return;
    }

    FreeRTOS_Init();
}

/**
 * @brief Configure micro-ROS. Create the nodes and publishers.
 */
void init_microROS()
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(
        &support,
        0,
        NULL,
        &init_options,
        &allocator));

    RCCHECK(rclc_node_init_default(
        &esp32_cam_node,
        "esp32_cam_node",
        "",
        &support));

    RCCHECK(rclc_publisher_init_default(
        &image_pub,
        &esp32_cam_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
        "/camera/image/compressed"));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

/**
 * @brief Create the tasks and queue.
 */
void FreeRTOS_Init()
{
    xTaskCreate(TaskTakePicture,
                "Takes picture",
                4000,
                NULL,
                1,
                NULL);
}

void TaskTakePicture(void *argument)
{
    sensor_msgs__msg__CompressedImage img_msg;
    sensor_msgs__msg__CompressedImage__init(&img_msg);
    img_msg.header.frame_id = micro_ros_string_utilities_set(img_msg.header.frame_id, "myframe");

    img_msg.format = micro_ros_string_utilities_set(img_msg.format, "jpeg");

    while (1)
    {
        camera_fb_t *pic = esp_camera_fb_get();
        
        img_msg.data.data = (uint8_t *)malloc(pic->len);
        img_msg.data.size = pic->len;
        memcpy(img_msg.data.data, pic->buf, pic->len);

        RCSOFTCHECK(rcl_publish(&image_pub, &img_msg, NULL));
        
        free(img_msg.data.data);
        esp_camera_fb_return(pic);
    }
}
