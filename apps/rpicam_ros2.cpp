/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_hello.cpp - libcamera "hello world" app.
 */

#include <chrono>
#include <atomic>
#include <fcntl.h>
#include "core/rpicam_app.hpp"
#include "core/options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::placeholders;

struct SharedData {
    // It prevents "torn reads" (reading half old, half new data)
    std::atomic<uint32_t> sync_ts;
};

// The main event loop for the application.

static void event_loop(RPiCamApp &app, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& img_pub, struct SharedData *shm_ptr)
{
    static uint32_t prv_sync_ts = 0;
    static long int prv_cam_ts = 0;

	Options const *options = app.GetOptions();

	app.OpenCamera();
	app.ConfigureViewfinder();
	app.StartCamera();

	auto start_time = std::chrono::high_resolution_clock::now();

    sensor_msgs::msg::Image img_msg;
    img_msg.header.frame_id = "body";
    img_msg.encoding = "mono8";
    img_msg.is_bigendian = false;

	for (unsigned int count = 0; ; count++)
	{
		RPiCamApp::Msg msg = app.Wait();
		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type == RPiCamApp::MsgType::Quit)
			return;
		else if (msg.type != RPiCamApp::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		LOG(2, "Viewfinder frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		if (options->Get().timeout && (now - start_time) > options->Get().timeout.value)
			return;

        if (count == 0) {
            libcamera::StreamConfiguration const &cfg = app.ViewfinderStream()->configuration();
   			LOG(1, "stream: " << cfg.size.width << "x" << cfg.size.height << " stride " << cfg.stride << " format " << cfg.pixelFormat.toString());
            img_msg.height = cfg.size.height;
            img_msg.width = cfg.size.width;
            img_msg.step = cfg.stride;
        }

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);

        auto cam_ts = completed_request->metadata.get(controls::SensorTimestamp);
        if (*cam_ts - prv_cam_ts < 40000000) {
            printf("cam ts too close %ld\n", *cam_ts - prv_cam_ts);
        }
        prv_cam_ts = *cam_ts;
        uint32_t sync_ts = atomic_load(&shm_ptr->sync_ts);
        if (sync_ts == prv_sync_ts) {
            printf("sync_ts not updated %u\n", sync_ts);
            continue;
        }
        prv_sync_ts = sync_ts;

		//app.ShowPreview(completed_request, app.ViewfinderStream());
        BufferReadSync w(&app, completed_request->buffers[app.ViewfinderStream()]);
    	libcamera::Span<uint8_t> buffer = w.Get()[0];
    	uint8_t *ptr = (uint8_t *)buffer.data();
        img_msg.data.assign(ptr, ptr+img_msg.height*img_msg.step);
        //auto ns = completed_request->metadata.get(controls::SensorTimestamp);
        //img_msg.header.stamp.sec = static_cast<int32_t>(*ns / 1000000000);
        //img_msg.header.stamp.nanosec = static_cast<uint32_t>(*ns % 1000000000);
        img_msg.header.stamp.sec = static_cast<int32_t>(sync_ts / 1000000);
        img_msg.header.stamp.nanosec = static_cast<uint32_t>((sync_ts % 1000000) * 1000);
        img_pub->publish(img_msg);
	}
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto ros_node = rclcpp::Node::make_shared("rpicam");
    auto img_pub = ros_node->create_publisher<sensor_msgs::msg::Image>("mono_left", rclcpp::QoS(1).best_effort().durability_volatile());

    int shm_fd = shm_open("my_imu_cam_sync", O_RDONLY, 0666);
    if (shm_fd == -1) {
        printf("shm_open (make sure Python writer is running first)\n");
        return 1;
    }
    struct SharedData *shm_ptr = (struct SharedData*)mmap(0, sizeof(struct SharedData), PROT_READ, MAP_SHARED, shm_fd, 0);

	try
	{
		RPiCamApp app;
		Options *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->Get().verbose >= 2)
				options->Get().Print();

			event_loop(app, img_pub, shm_ptr);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}

    munmap(shm_ptr, sizeof(struct SharedData));
    close(shm_fd);
    rclcpp::shutdown();

	return 0;
}
