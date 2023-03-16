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

// Modified from ezdib by Robert Umbehant
// https://github.com/wheresjames/ezdib
static const char font_map_medium [] =
{
        // Default glyph
        '.', 2, 10,     0x00, 0x3c, 0x00,

        // Tab width
        '\t', 10, 0,

        // Space
        ' ', 2, 0,

        '!', 1, 10,     0xf6, 0x00,
        '(', 3, 10,     0x2a, 0x48, 0x88, 0x00,
        ')', 3, 10,     0x88, 0x92, 0xa0, 0x00,
        ',', 2, 10,     0x00, 0x16, 0x00,
        '-', 3, 10,     0x00, 0x70, 0x00, 0x00,
        '/', 3, 10,     0x25, 0x25, 0x20, 0x00,
        '@', 6, 10,     0x7a, 0x19, 0x6b, 0x9a, 0x07, 0x80, 0x00, 0x00,
        '$', 5, 10,     0x23, 0xab, 0x47, 0x16, 0xae, 0x20, 0x00,
        '#', 6, 10,     0x49, 0x2f, 0xd2, 0xfd, 0x24, 0x80, 0x00, 0x00,
        '%', 7, 10,     0x43, 0x49, 0x20, 0x82, 0x49, 0x61, 0x00, 0x00, 0x00,
        ':', 2, 10,     0x3c, 0xf0, 0x00,
        '^', 3, 10,     0x54, 0x00, 0x00, 0x00,
        '~', 5, 10,     0x00, 0x11, 0x51, 0x00, 0x00, 0x00, 0x00,

        '0', 5, 10,     0x74, 0x73, 0x59, 0xc5, 0xc0, 0x00, 0x00,
        '1', 3, 10,     0xc9, 0x24, 0xb8, 0x00,
        '2', 5, 10,     0x74, 0x42, 0xe8, 0x43, 0xe0, 0x00, 0x00,
        '3', 5, 10,     0x74, 0x42, 0xe0, 0xc5, 0xc0, 0x00, 0x00,
        '4', 5, 10,     0x11, 0x95, 0x2f, 0x88, 0x40, 0x00, 0x00,
        '5', 5, 10,     0xfc, 0x3c, 0x10, 0xc5, 0xc0, 0x00, 0x00,
        '6', 5, 10,     0x74, 0x61, 0xe8, 0xc5, 0xc0, 0x00, 0x00,
        '7', 5, 10,     0xfc, 0x44, 0x42, 0x10, 0x80, 0x00, 0x00,
        '8', 5, 10,     0x74, 0x62, 0xe8, 0xc5, 0xc0, 0x00, 0x00,
        '9', 5, 10,     0x74, 0x62, 0xf0, 0xc5, 0xc0, 0x00, 0x00,

	0,
};

static const char* ezd_next_glyph( const char* pGlyph )
{
        int sz;

        // Last glyph?
        if ( !pGlyph || !*pGlyph )
                return 0;

        // Glyph size in bits
        sz = pGlyph[ 1 ] * pGlyph[ 2 ];

        // Return a pointer to the next glyph
        return &pGlyph[ 3 + ( ( sz & 0x07 ) ? ( ( sz >> 3 ) + 1 ) : sz >> 3 ) ];
}

static const char* ezd_find_glyph(const char* x_pFt, const char ch )
{
	const char* pGlyph = x_pFt;

        // Find the glyph
        while ( pGlyph && *pGlyph )
                if ( ch == *pGlyph )
                        return pGlyph;
                else
                        pGlyph = ezd_next_glyph( pGlyph );

        // First glyph is the default
        return (const char*)x_pFt;
}

static void ezd_draw_bmp_yuv420( unsigned char *pImg, int x, int y, int sw, int pw,int bw, int bh, const char *pBmp)
{
        int w, h, lx = x;
        unsigned char m = 0x80;

        // Draw the glyph
        for( h = 0; h < bh; h++ )
        {
                // Draw horz line
                for( w = 0; w < bw; w++ )
                {
                        // Next glyph byte?
                        if ( !m ) m = 0x80, pBmp++;
                        // Is this pixel on?
                        if ( *pBmp & m ) {
				pImg[y * sw + lx ] = 0xff;
		        }
                        // Next bmp bit
                        m >>= 1;
                        // Next x pixel
                        lx++;
                }
                // Reset x
                lx = x;
                // Reset y
                y++;
        }
}

__attribute__((unused)) static void ezd_text(unsigned char *img, char *x_pText, int x_nTextLen, int x, int y, int w, int h) {
	int sw = w, pw = 1, i, mh = 0, lx = x;
	const char *pGlyph;

	for ( i = 0; i < x_nTextLen || ( 0 > x_nTextLen && x_pText[ i ] ); i++ ) {
		// Get the specified glyph
		pGlyph = ezd_find_glyph( font_map_medium, x_pText[ i ] );

		// Other characters
		// Draw this glyph if it's completely on the screen
		if ( pGlyph[ 1 ] && pGlyph[ 2 ]
			&& 0 <= lx && ( lx + pGlyph[ 1 ] ) < w
			&& 0 <= y && ( y + pGlyph[ 2 ] ) < h )
		{
		    ezd_draw_bmp_yuv420(img, lx, y, sw, pw, pGlyph[ 1 ], pGlyph[ 2 ], &pGlyph[ 3 ]);
		}

		// Next character position
		lx += 2 + pGlyph[ 1 ];

		// Track max height
		mh = ( pGlyph[ 2 ] > mh ) ? pGlyph[ 2 ] : mh;
	}
}

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

	det_info.tagsize = 0.161;
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

		//*(ptr + (int)(det->c[0]) + stream_width * (int)(det->c[1])) = 76;
		//*(ptr + stream_width * stream_height + (int)(det->c[0]/2) + stream_width / 2 * (int)(det->c[1]/2)) = 84;
		//*(ptr + stream_width * stream_height + stream_width * stream_height / 4 + (int)(det->c[0]/2) + stream_width / 2 * (int)(det->c[1]/2)) = 255;
                char txt_buf[32];
                sprintf(txt_buf, "%d", det->id);
                ezd_text(ptr, txt_buf, -1, det->c[0], det->c[1], stream_width, stream_height);

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
