/*
 * Copyright (C) 2012 Renesas Electronics Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file v4l_camera/camera_preview.cpp
 * @author Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 * @brief C++ camera-preview and frame-manipulation interface
 */

#define LOG_TAG "camera_preview"

#include <sys/types.h>

#include <linux/videodev2.h>

#include <cutils/log.h>
#include <ui/Rect.h>
#include <ui/GraphicBufferMapper.h>

#include "camera_param.h"
#include "camera_preview.h"

#include <Converters.h>
#include <JpegCompressor.h>

namespace android {

uint8_t *camera_preview_lock(buffer_handle_t* buffer, unsigned int width, unsigned int height)
{
	uint8_t* img = NULL;
	/* Rectangle with origin at (0,0) */
	const Rect rect(width, height);
	GraphicBufferMapper& grbuffer_mapper(GraphicBufferMapper::get());

	if (grbuffer_mapper.lock(*buffer, GRALLOC_USAGE_SW_WRITE_OFTEN, rect, (void **)&img) != NO_ERROR)
		return NULL;
	return img;
}

void camera_preview_unlock(buffer_handle_t* buffer)
{
	GraphicBufferMapper& grbuffer_mapper(GraphicBufferMapper::get());
	grbuffer_mapper.unlock(*buffer);
}

/* TODO: add trivial memcpy converters too */
camera_frame_converter camera_frame_converter_select(uint32_t fourcc, int fb_fmt)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_YVU420:
		switch (fb_fmt) {
		case HAL_PIXEL_FORMAT_RGB_565:
			return YV12ToRGB565;
		case HAL_PIXEL_FORMAT_RGBA_8888:
			return YV12ToRGB32;
//		case HAL_PIXEL_FORMAT_RGB_888:
//		case HAL_PIXEL_FORMAT_RGBA_5551:
		}
		break;
	case V4L2_PIX_FMT_YUV420:
		switch (fb_fmt) {
		case HAL_PIXEL_FORMAT_RGBA_8888:
			return YU12ToRGB32;
		}
		break;
	case V4L2_PIX_FMT_NV21:
		switch (fb_fmt) {
		case HAL_PIXEL_FORMAT_RGB_565:
			return NV21ToRGB565;
		case HAL_PIXEL_FORMAT_RGBA_8888:
			return NV21ToRGB32;
		}
		break;
	case V4L2_PIX_FMT_NV12:
		switch (fb_fmt) {
		case HAL_PIXEL_FORMAT_RGB_565:
			return NV12ToRGB565;
		case HAL_PIXEL_FORMAT_RGBA_8888:
			return NV12ToRGB32;
		}
	}
	LOGE("%s(): %x -> %x conversion unsupported", __func__, fourcc, fb_fmt);
	return NULL;
}

int camera_frame_compress(struct camera_callback *cb, uint8_t *in,
			  unsigned int width, unsigned int height)
{
	NV21JpegCompressor compressor;
	status_t ret = compressor.compressRawImage(in, width, height, 90);

	if (ret == NO_ERROR) {
		camera_memory_t* jpeg_buff =
			cb->get_mem(-1, compressor.getCompressedSize(), 1, 0, NULL);
		if (jpeg_buff && jpeg_buff->data) {
			compressor.getCompressedImage(jpeg_buff->data);
			cb->data(CAMERA_MSG_COMPRESSED_IMAGE, jpeg_buff, 0, NULL, cb->arg);
			jpeg_buff->release(jpeg_buff);
		} else {
			LOGE("%s(): memory allocation failure", __func__);
			return -ENOMEM;
		}
	} else {
		LOGE("%s(): compression failure", __func__);
		return -ENODATA;
	}
	return 0;
}

};
