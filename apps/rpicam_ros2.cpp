/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_hello.cpp - libcamera "hello world" app.
 */

#include <chrono>

#include "core/rpicam_app.hpp"
#include "core/options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::placeholders;

// The main event loop for the application.

static void event_loop(RPiCamApp &app, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& img_pub)
{
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
		//app.ShowPreview(completed_request, app.ViewfinderStream());
        BufferReadSync w(&app, completed_request->buffers[app.ViewfinderStream()]);
    	libcamera::Span<uint8_t> buffer = w.Get()[0];
    	uint8_t *ptr = (uint8_t *)buffer.data();
        img_msg.data.assign(ptr, ptr+img_msg.height*img_msg.step);
        auto ns = completed_request->metadata.get(controls::SensorTimestamp);
        img_msg.header.stamp.sec = static_cast<int32_t>(*ns / 1000000000);
        img_msg.header.stamp.nanosec = static_cast<uint32_t>(*ns % 1000000000);
        img_pub->publish(img_msg);
	}
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto ros_node = rclcpp::Node::make_shared("rpicam");
    auto img_pub = ros_node->create_publisher<sensor_msgs::msg::Image>("mono_left", rclcpp::QoS(1).best_effort().durability_volatile());

	try
	{
		RPiCamApp app;
		Options *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->Get().verbose >= 2)
				options->Get().Print();

			event_loop(app, img_pub);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}

    rclcpp::shutdown();

	return 0;
}
