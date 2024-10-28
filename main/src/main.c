#include "inc/main.h"


static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    // .pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    // .pixel_format = PIXFORMAT_JPEG,
    .pixel_format = PIXFORMAT_GRAYSCALE,
    // .frame_size = FRAMESIZE_UXGA, // QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.
    .frame_size = FRAMESIZE_SVGA,

    .jpeg_quality = 12, // 0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1,      // When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t init_camera(void)
{
    // initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        printf("Camera Init Failed\n");
        return err;
    }

    return ESP_OK;
}

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

    // Static Agent IP and port can be used instead of autodisvery.
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
// RCCHECK(rmw_uros_discover_agent(rmw_options));
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
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
        "/camera/image_raw"));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

/**
 * @brief Create the tasks and queue.
 */
void FreeRTOS_Init()
{

    pictureQueue = xQueueCreate(10, sizeof(sensor_msgs__msg__Image *));
    if (pictureQueue == NULL)
    {
        printf("Failed to create pictureQueue\n");
        return;
    }

    xTaskCreate(TaskTakePicture,
                "Takes picture",
                2000,
                NULL,
                1,
                NULL);

    // xTaskCreate(TaskSendPicture,
    //             "Sends picture",
    //             2000,
    //             NULL,
    //             1,
    //             NULL);
}

void TaskTakePicture(void *argument)
{
    sensor_msgs__msg__Image img_msg;
    sensor_msgs__msg__Image__init(&img_msg);
    img_msg.header.frame_id = micro_ros_string_utilities_set(img_msg.header.frame_id, "myframe");
    // img_msg.data.capacity = img_msg.height * img_msg.width; //Matrix data for width*height*1 (with another pixel_format maybe this will change)

    img_msg.encoding = micro_ros_string_utilities_set(img_msg.encoding, "mono8");
    img_msg.is_bigendian = false;
    while (1)
    {
        camera_fb_t *pic = esp_camera_fb_get();
        printf("Taking photo... \n");
        img_msg.step = pic->len / pic->height; // 800
        img_msg.width = pic->width; // 800
        img_msg.height = pic->height; // 600
        img_msg.data.data = (uint8_t *)malloc(img_msg.height * img_msg.width);
        img_msg.data.size = img_msg.height * img_msg.width;

        memcpy(img_msg.data.data, pic->buf, pic->len);
        RCSOFTCHECK(rcl_publish(&image_pub, &img_msg, NULL));
        free(img_msg.data.data);
        esp_camera_fb_return(pic);
        vTaskDelay(pdMS_TO_TICKS(100)); // 1 second delay
    }
}

// void TaskSendPicture(void *argument)
// {
//     sensor_msgs__msg__Image *img_msg;
//     img_msg = sensor_msgs__msg__Image__create();
//     // img_msg->header.frame_id = micro_ros_string_utilities_set(img_msg->header.frame_id, "camera");
//     while (1)
//     {
//         if (xQueueReceive(pictureQueue, &img_msg, portMAX_DELAY) == pdTRUE)
//         {
//             // Usar los datos reales del frame buffer
//             // img_msg->height = pic->height;                                                 // Altura obtenida desde el frame buffer
//             // img_msg->width = pic->width;                                                   // Anchura obtenida desde el frame buffer
//             // img_msg->encoding = micro_ros_string_utilities_set(img_msg->encoding, "jpeg"); // Codificación de la imagen
//             // img_msg->is_bigendian = false;

//             // img_msg->data.capacity = pic->height * pic->width * 3;
//             // img_msg->data.data = (uint8_t *)malloc(img_msg->data.capacity);
//             // img_msg->data.size = img_msg->data.capacity;
//             // for (size_t i = 0; i < pic->len; i++)
//             // {
//             //     memset(img_msg->data.data, pic->buf[i], img_msg->data.capacity);
//             //     // img_msg->data.data[i] = pic->buf[i];
//             // }

//             // RCSOFTCHECK(rcl_publish(&image_pub, img_msg, NULL));

//             printf("Picture publish!\n");
//             sensor_msgs__msg__Image__destroy(img_msg);
//             // free(img_msg->data.data);
//             // free(img_msg->data.data); // Liberar la memoria asignada
//         }
//     }
// }
