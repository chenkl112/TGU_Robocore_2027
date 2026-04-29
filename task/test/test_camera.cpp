//
// Created by tgu on 2026/4/16.
//

#include <arv.h>
#include <iostream>
#include <opencv2/opencv.hpp>

static bool print_error_and_clear(GError*& error, const char* where) {
    if (error != nullptr) {
        std::cerr << where << " failed: " << error->message << std::endl;
        g_clear_error(&error);
        return true;
    }
    return false;
}

int main() {
    arv_update_device_list();

    int n_devices = arv_get_n_devices();
    if (n_devices == 0) {
        std::cout << "No camera found\n";
        return -1;
    }

    const char* id = arv_get_device_id(0);
    std::cout << "Using device: " << id << std::endl;

    GError* error = nullptr;

    ArvCamera* camera = arv_camera_new(id, &error);
    if (print_error_and_clear(error, "arv_camera_new") || camera == nullptr) {
        return -1;
    }

    arv_camera_set_exposure_time(camera, 100000.0, &error);
    if (print_error_and_clear(error, "arv_camera_set_exposure_time")) {
        g_object_unref(camera);
        return -1;
    }

    ArvStream* stream = arv_camera_create_stream(camera, nullptr, nullptr, &error);
    if (print_error_and_clear(error, "arv_camera_create_stream") || stream == nullptr) {
        g_object_unref(camera);
        return -1;
    }

    gint payload = arv_camera_get_payload(camera, &error);
    if (print_error_and_clear(error, "arv_camera_get_payload") || payload <= 0) {
        g_object_unref(stream);
        g_object_unref(camera);
        return -1;
    }

    std::cout << "Payload size: " << payload << std::endl;

    for (int i = 0; i < 10; i++) {
        arv_stream_push_buffer(stream, arv_buffer_new(payload, nullptr));
    }

    arv_camera_start_acquisition(camera, &error);
    if (print_error_and_clear(error, "arv_camera_start_acquisition")) {
        g_object_unref(stream);
        g_object_unref(camera);
        return -1;
    }

    ArvBuffer* buffer = arv_stream_timeout_pop_buffer(stream, 2000000); // 2s

    if (buffer && arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS) {
        size_t size = 0;
        const void* data = arv_buffer_get_data(buffer, &size);

        gint width = arv_buffer_get_image_width(buffer);
        gint height = arv_buffer_get_image_height(buffer);

        std::cout << "Got frame, size = " << size << std::endl;
        std::cout << "Resolution: " << width << " x " << height << std::endl;

        cv::Mat raw(height, width, CV_8UC1, const_cast<void*>(data));

        cv::Mat bgr;
        cv::cvtColor(raw, bgr, cv::COLOR_BayerRG2BGR);

        cv::Mat img = bgr.clone();

        cv::imwrite("test_bgr.png", img);
        std::cout << "Saved: test_bgr.png\n";
        // cv::imshow("bgr", img);
        // cv::waitKey(0);

        g_object_unref(buffer);
    } else {
        if (buffer) {
            std::cout << "Failed to get frame, status = "
                      << arv_buffer_get_status(buffer) << std::endl;
            g_object_unref(buffer);
        } else {
            std::cout << "Failed to get frame: timeout\n";
        }
    }

    arv_camera_stop_acquisition(camera, &error);
    print_error_and_clear(error, "arv_camera_stop_acquisition");

    g_object_unref(stream);
    g_object_unref(camera);

    return 0;
}