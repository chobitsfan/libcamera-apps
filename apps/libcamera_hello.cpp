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
#include <time.h>

#include "core/libcamera_app.hpp"
#include "core/options.hpp"

using namespace std::placeholders;

// The main event loop for the application.

static void event_loop(LibcameraApp &app)
{
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Publisher pub = it.advertise("camera/image_raw", 1);

	app.OpenCamera();
	app.ConfigureViewfinder();
	app.StartCamera();

    struct timespec boot_ts, epoch_ts;
    clock_gettime(CLOCK_BOOTTIME, &boot_ts);
    clock_gettime(CLOCK_REALTIME, &epoch_ts);
    uint64_t boot_epoch_offset = epoch_ts.tv_sec * 1000000000 + epoch_ts.tv_nsec - (boot_ts.tv_sec * 1000000000 + boot_ts.tv_nsec);

	sensor_msgs::Image img;
	img.height = 400;
    img.width = 640;
    img.step = 640;
    img.encoding = sensor_msgs::image_encodings::MONO8;
    img.is_bigendian = 0;
    img.data.resize(640*400);

	while (1)
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

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);

        auto sensor_ts = completed_request->metadata.get(controls::SensorTimestamp);
        uint64_t sensor_ts_epoch = *sensor_ts + boot_epoch_offset;
        ros::Time sensor_ros_ts(sensor_ts_epoch / 1000000000, sensor_ts_epoch % 1000000000);
        //clock_gettime(CLOCK_BOOTTIME, &boot_ts);
        //printf("%lu %lu\n", (uint64_t)*sensor_ts, boot_ts.tv_sec*1000000000+boot_ts.tv_nsec);

		libcamera::Span<uint8_t> buffer = app.Mmap(completed_request->buffers[app.ViewfinderStream()])[0];
        img.header.stamp = sensor_ros_ts;
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
