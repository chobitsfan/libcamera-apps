/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * negate_stage.cpp - image negate effect
 */

#include <libcamera/stream.h>

#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tagStandard41h12.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <iostream>

#include "core/libcamera_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

using Stream = libcamera::Stream;

class AprilTagPlndStage : public PostProcessingStage
{
public:
	AprilTagPlndStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override {}

	void Configure() override;

	void Teardown() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	Stream *stream_;

	apriltag_detector_t *td;
	apriltag_family_t *tf;
	apriltag_detection_info_t det_info;

	int stream_width;
	int stream_height;
	int ipc_fd;
        struct sockaddr_in ipc_server;
};

#define NAME "apriltagplnd"

char const *AprilTagPlndStage::Name() const
{
	return NAME;
}

void AprilTagPlndStage::Configure()
{
	stream_ = app_->GetMainStream();
        StreamInfo info = app_->GetStreamInfo(stream_);
        stream_width = info.width;
        stream_height = info.height;

	det_info.tagsize = 0.113;
	det_info.fx = 496.25399994435088;
	det_info.fy = 496.25399994435088;
	det_info.cx = 320;
	det_info.cy = 240;
	td = apriltag_detector_create();
	td->quad_decimate = 2;
    	td->nthreads = 4;
	tf = tagStandard41h12_create();
	apriltag_detector_add_family(td, tf);

	if ((ipc_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        	return;
	}
	memset(&ipc_server, 0, sizeof(ipc_server));
	ipc_server.sin_family = AF_INET;
	ipc_server.sin_port = htons(17510);
	ipc_server.sin_addr.s_addr = inet_addr("127.0.0.1");
}

void AprilTagPlndStage::Teardown() 
{
	apriltag_detector_destroy(td);
	tagStandard41h12_destroy(tf);
}

bool AprilTagPlndStage::Process(CompletedRequestPtr &completed_request)
{
	libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request->buffers[stream_])[0];
	uint8_t *ptr = buffer.data();

	image_u8_t img_header{stream_width, stream_height, stream_width, ptr};

	zarray_t *detections = apriltag_detector_detect(td, &img_header);
	if (zarray_size(detections) > 0) {
        	apriltag_detection_t *det;
	        zarray_get(detections, 0, &det);

		//std::cout << "apriltag id " << det->id << " found\n";

		*(ptr + (int)(det->c[0]) + stream_width * (int)(det->c[1])) = 76;
		*(ptr + stream_width * stream_height + (int)(det->c[0]/2) + stream_width / 2 * (int)(det->c[1]/2)) = 84;
		*(ptr + stream_width * stream_height + stream_width * stream_height / 4 + (int)(det->c[0]/2) + stream_width / 2 * (int)(det->c[1]/2)) = 255;

		det_info.det = det;
		apriltag_pose_t pose;
		estimate_tag_pose(&det_info, &pose);
                double ipc_data[4] = { (double)det->id, pose.t->data[0],  pose.t->data[1],  pose.t->data[2] };
                sendto(ipc_fd, ipc_data, sizeof(ipc_data), 0, (const struct sockaddr *)&ipc_server, sizeof(ipc_server));
		matd_destroy(pose.t);
		matd_destroy(pose.R);
	}
	apriltag_detections_destroy(detections);

	return false;
}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new AprilTagPlndStage(app);
}

static RegisterStage reg(NAME, &Create);
