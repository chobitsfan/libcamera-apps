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
#include <termios.h> // Contains POSIX terminal control definitions
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
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

	int uart_fd;
};

#define NAME "apriltagplnd"
#define CAM_RES_W 640
#define CAM_RES_H 480

char const *AprilTagPlndStage::Name() const
{
	return NAME;
}

void AprilTagPlndStage::Configure()
{
	stream_ = app_->GetMainStream();
	std::cout << stream_->configuration().size.width << "x" << stream_->configuration().size.height << "\n";

	det_info.tagsize = 0.113;
	det_info.fx = 496.25399994435088;
	det_info.fy = 496.25399994435088;
	det_info.cx = 320;
	det_info.cy = 240;
	td = apriltag_detector_create();
	td->quad_decimate = 1;
    td->nthreads = 4;
	tf = tagStandard41h12_create();
	apriltag_detector_add_family(td, tf);

	uart_fd = open("/dev/ttyAMA0", O_RDWR | O_NONBLOCK);
    if (uart_fd < 0) {
        std::cout << "can not open serial port\n";
        return;
    }
}

void AprilTagPlndStage::Teardown() 
{
	tagStandard41h12_destroy(tf);
    apriltag_detector_destroy(td);
}

bool AprilTagPlndStage::Process(CompletedRequestPtr &completed_request)
{
	libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request->buffers[stream_])[0];
	uint8_t *ptr = (uint8_t *)buffer.data();

	image_u8_t img_header{CAM_RES_W, CAM_RES_H, CAM_RES_W, ptr};
    apriltag_pose_t pose;

	zarray_t *detections = apriltag_detector_detect(td, &img_header);
	if (zarray_size(detections) > 0) {
        apriltag_detection_t *det;
        zarray_get(detections, 0, &det);
		std::cout << "apriltag id " << det->id << " found\n";
		if (det->id == 0) {
			det_info.det = det;
			estimate_tag_pose(&det_info, &pose);
			matd_destroy(pose.t);
            matd_destroy(pose.R);
		}
	}
	apriltag_detections_destroy(detections);

	// Constraints on the stride mean we always have multiple-of-4 bytes.
	//for (unsigned int i = 0; i < buffer.size(); i += 4)
	//	*(ptr++) ^= 0xffffffff;

	return false;
}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new AprilTagPlndStage(app);
}

static RegisterStage reg(NAME, &Create);
