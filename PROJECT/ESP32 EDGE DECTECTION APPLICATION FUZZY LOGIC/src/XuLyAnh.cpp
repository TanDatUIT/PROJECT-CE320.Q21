/*Mặt nạ 2x2 [ Tối ưu cho esp32]
đầu vào: 4 pixel [ P1, P2, P3, P4]
tập mờ input : 2 tập black and white dùng hàm thuộc hình tam giác
tập mờ output: black, edge và white
luật mờ: 16 luật [ tài liệu 2010]
*/


#include "XuLyAnh.h"

bool XuLyAnh::initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    
    // Sử dụng Grayscale để phù hợp thuật toán Logic mờ 
    config.pixel_format = PIXFORMAT_GRAYSCALE; 
    
    // QVGA (320x240) là mức tối ưu để không bị tràn RAM khi xử lý mờ [cite: 7]
    config.frame_size = FRAMESIZE_QVGA; 
    config.jpeg_quality = 12;
    config.fb_count = 1;

    // Khởi tạo camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return false;
    }

    // Chỉnh thông số ảnh cho nét để dễ bắt cạnh
    sensor_t * s = esp_camera_sensor_get();
    s->set_brightness(s, 1);     // Tăng độ sáng nhẹ (-2 to 2)
    s->set_contrast(s, 1);       // Tăng tương phản để nổi bật biên [cite: 39]
    
    return true;
}

camera_fb_t* XuLyAnh::capture() {
    // Lấy khung hình từ Camera
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return NULL;
    }
    return fb;
}

void XuLyAnh::release(camera_fb_t* fb) {
    if (fb) {
        esp_camera_fb_return(fb); // Trả lại buffer để tránh Memory Leak
    }
}