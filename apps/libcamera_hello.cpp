/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_hello.cpp - libcamera "hello world" app.
 */

#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>

#include "core/libcamera_app.hpp"
#include "core/options.hpp"

using namespace std::placeholders;

// The main event loop for the application.

static void event_loop(LibcameraApp &app)
{
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Publisher pub = it.advertise("camera/image_raw", 1);

	Options const *options = app.GetOptions();

	app.OpenCamera();
	app.ConfigureViewfinder();
	app.StartCamera();

	auto start_time = std::chrono::high_resolution_clock::now();

	sensor_msgs::Image img;
	img.height = 400;
        img.width = 640;
        img.step = 640;
        img.encoding = sensor_msgs::image_encodings::MONO8;
        img.is_bigendian = 0;
        img.data.resize(640*400);

	for (unsigned int count = 0; ; count++)
	{
		LibcameraApp::Msg msg = app.Wait();
		if (msg.type == LibcameraApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type == LibcameraApp::MsgType::Quit)
			return;
		else if (msg.type != LibcameraApp::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		LOG(2, "Viewfinder frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		if (options->timeout && now - start_time > std::chrono::milliseconds(options->timeout))
			return;

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);

		libcamera::Span<uint8_t> buffer = app.Mmap(completed_request->buffers[app.ViewfinderStream()])[0];
                img.header.stamp = ros::Time::now();
		memcpy(&img.data[0], buffer.data(), 640*400);
		pub.publish(img);

		//app.ShowPreview(completed_request, app.ViewfinderStream());
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "libcamera_hello", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
        ros::start();

	try
	{
		LibcameraApp app;
		Options *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose >= 2)
				options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
                ros::shutdown();
		return -1;
	}

	ros::shutdown();

	return 0;
}
