/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_hello.cpp - libcamera "hello world" app.
 */

#include <chrono>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "core/rpicam_app.hpp"
#include "core/options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

//#define USE_SHARE_MEM_IPC

using namespace std::placeholders;

// The main event loop for the application.
#ifdef USE_SHARE_MEM_IPC
static void event_loop(RPiCamApp &app, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& img_pub, uint32_t *sync_ts_ptr)
#else
static void event_loop(RPiCamApp &app, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& img_pub, int fifo_fd)
#endif
{
    static long int prv_cam_ts = 0;
    //struct pollfd pfd;
    //pfd.fd = fifo_fd;
    //pfd.events = POLLIN;

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
        if (*cam_ts - prv_cam_ts < 45000000) {
            printf("cam ts too close %ld\n", *cam_ts - prv_cam_ts);
        } else if (prv_cam_ts > 0 && *cam_ts - prv_cam_ts > 55000000) {
            printf("cam ts too far %ld\n", *cam_ts - prv_cam_ts);
        }
        prv_cam_ts = *cam_ts;

#ifdef USE_SHARE_MEM_IPC
        uint32_t sync_ts = *sync_ts_ptr;
#else
        uint32_t sync_ts;
        ssize_t bytes_read = read(fifo_fd, &sync_ts, sizeof(sync_ts));
        if (bytes_read != sizeof(sync_ts)) {
            printf("fifo read failed\n");
            return;
        }
        /*bool poll_again = false;
        do {
            int ret = poll(&pfd, 1, 0);
            if (ret > 0 && (pfd.revents & POLLIN)) {
                bytes_read = read(fifo_fd, &sync_ts, sizeof(sync_ts));
                if (bytes_read != sizeof(sync_ts)) {
                    printf("fifo read failed\n");
                    return;
                }
                poll_again = true;
            } else poll_again = false;
        } while (poll_again);*/
        //printf("%d\n", sync_ts);
#endif

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

#ifdef USE_SHARE_MEM_IPC
    int shm_fd = shm_open("my_imu_cam_sync", O_RDONLY, 0666);
    if (shm_fd == -1) {
        printf("shm_open (make sure writer is running first)\n");
        return -1;
    }
    uint32_t *sync_ts_ptr = (uint32_t*)mmap(0, 4, PROT_READ, MAP_SHARED, shm_fd, 0);
#else
    const char* fifo_path = "/tmp/my_cam_ts_fifo";
    mkfifo(fifo_path, 0666);
    printf("opening fifo\n");
    int fifo_fd = open(fifo_path, O_RDONLY);
    if (fifo_fd < 0) {
        printf("fail to open fifo\n");
        return -1;
    }
#endif

	try
	{
		RPiCamApp app;
		Options *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->Get().verbose >= 2)
				options->Get().Print();
#ifdef USE_SHARE_MEM_IPC
            event_loop(app, img_pub, sync_ts_ptr);
#else
			event_loop(app, img_pub, fifo_fd);
#endif
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}

#ifdef USE_SHARE_MEM_IPC
    munmap(sync_ts_ptr, 4);
    close(shm_fd);
#else
    close(fifo_fd);
#endif
    rclcpp::shutdown();

	return 0;
}
