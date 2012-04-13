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
 * @file v4l_camera/camera_param.cpp
 * @author Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 * @brief C++ camera-parameter interface
 */

#include <stdlib.h>
#include <string.h>

#define LOG_TAG "camera_param"

#include <camera/CameraParameters.h>

#include <utils/Log.h>
#include <utils/StrongPointer.h>
#include <utils/RefBase.h>

#include "camera_param.h"

#define CAM_LOG LOGI

namespace android {

void camera_param_init(struct camera_param *cp)
{
	/* catch ENOMEM exception? */
	CameraParameters *p = new CameraParameters();
	CAM_LOG("%s()", __func__);

	p->set(CameraParameters::KEY_SUPPORTED_JPEG_THUMBNAIL_SIZES, "320x240,0x0");
	p->set(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION, "6");
	p->set(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION, "-6");
	p->set(CameraParameters::KEY_EXPOSURE_COMPENSATION_STEP, "0.5");
	p->set(CameraParameters::KEY_EXPOSURE_COMPENSATION, "0");
	p->set(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH, "512");
	p->set(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT, "384");
	p->set(CameraParameters::KEY_JPEG_QUALITY, "90");
	p->set(CameraParameters::KEY_FOCAL_LENGTH, "4.31");
	p->set(CameraParameters::KEY_HORIZONTAL_VIEW_ANGLE, "54.8");
	p->set(CameraParameters::KEY_VERTICAL_VIEW_ANGLE, "42.5");
	p->set(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY, "90");

	p->set(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES, "20,15");
	p->set(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE, "(15,20)");
	p->set(CameraParameters::KEY_PREVIEW_FPS_RANGE, "15,20");
	p->setPreviewFrameRate(20);

	p->set(CameraParameters::KEY_SUPPORTED_FOCUS_MODES, CameraParameters::FOCUS_MODE_FIXED);
	p->set(CameraParameters::KEY_FOCUS_MODE, CameraParameters::FOCUS_MODE_FIXED);

	cp->parm = p;
}

void camera_param_free(struct camera_param *cp)
{
	CAM_LOG("%s()", __func__);
	delete cp->parm;
	cp->parm = NULL;
}

const char *camera_param_fourcc2str(uint32_t fourcc)
{
	CAM_LOG("%s(%x)", __func__, fourcc);
	switch (fourcc) {
	case V4L2_PIX_FMT_NV21:
		return CameraParameters::PIXEL_FORMAT_YUV420SP;
	case V4L2_PIX_FMT_RGB565:
		return CameraParameters::PIXEL_FORMAT_RGB565;
	case V4L2_PIX_FMT_RGB32:
		return CameraParameters::PIXEL_FORMAT_RGBA8888;
	case V4L2_PIX_FMT_NV16:
		return CameraParameters::PIXEL_FORMAT_YUV422SP;
	case V4L2_PIX_FMT_YUYV:
		return CameraParameters::PIXEL_FORMAT_YUV422I;
	case V4L2_PIX_FMT_YUV420M:
		return CameraParameters::PIXEL_FORMAT_YUV420P;
	case V4L2_PIX_FMT_SRGGB8:
		return CameraParameters::PIXEL_FORMAT_BAYER_RGGB;
	case V4L2_PIX_FMT_JPEG:
		return CameraParameters::PIXEL_FORMAT_JPEG;
	default:
		return NULL;
	}
}

void camera_param_set(struct camera_param *cp)
{
	CameraParameters *p = cp->parm;

	p->setPreviewFormat(camera_param_fourcc2str(cp->preview_fmt.fourcc));
	p->setPreviewSize(cp->preview_fmt.width, cp->preview_fmt.height);
	p->set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES, p->get(CameraParameters::KEY_PREVIEW_SIZE));

	p->setPictureFormat(camera_param_fourcc2str(cp->picture_fmt.fourcc));
	p->setPictureSize(cp->picture_fmt.width, cp->picture_fmt.height);
	p->set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES, p->get(CameraParameters::KEY_PICTURE_SIZE));

	p->set(CameraParameters::KEY_VIDEO_FRAME_FORMAT, camera_param_fourcc2str(cp->record_fmt.fourcc));
	p->setVideoSize(cp->record_fmt.width, cp->record_fmt.height);
	/* We don't need this as long as we only support video == preview */
#if 0
	p->set(CameraParameters::KEY_SUPPORTED_VIDEO_SIZES, p->get(CameraParameters::KEY_VIDEO_SIZE));
#endif
}

void camera_param_put(struct camera_param *cp, char *param)
{
	free(param);
}

char *camera_param_get(struct camera_param *cp)
{
	String8 str = cp->parm->flatten();
	LOGV("%s(%s)", __func__, str.string());
	return strdup(str.string());
}

static uint32_t camera_param_str2fourcc(const char *fmt, enum v4l2_colorspace *cspace)
{
	LOGV("%s(%s)", __func__, fmt);
	if (!strcmp(fmt, CameraParameters::PIXEL_FORMAT_YUV420SP)) {
		*cspace = V4L2_COLORSPACE_JPEG;
		return V4L2_PIX_FMT_NV21;
	}
	if (!strcmp(fmt, CameraParameters::PIXEL_FORMAT_RGB565)) {
		*cspace = V4L2_COLORSPACE_SRGB;
		return V4L2_PIX_FMT_RGB565;
	}
	if (!strcmp(fmt, CameraParameters::PIXEL_FORMAT_RGBA8888)) {
		*cspace = V4L2_COLORSPACE_SRGB;
		return V4L2_PIX_FMT_RGB32;
	}
	if (!strcmp(fmt, CameraParameters::PIXEL_FORMAT_YUV422SP)) {
		*cspace = V4L2_COLORSPACE_JPEG;
		return V4L2_PIX_FMT_NV16;
	}
	if (!strcmp(fmt, CameraParameters::PIXEL_FORMAT_YUV422I)) {
		*cspace = V4L2_COLORSPACE_JPEG;
		return V4L2_PIX_FMT_YUYV;
	}
	if (!strcmp(fmt, CameraParameters::PIXEL_FORMAT_YUV420P)) {
		*cspace = V4L2_COLORSPACE_JPEG;
		return V4L2_PIX_FMT_YUV420M;
	}
	if (!strcmp(fmt, CameraParameters::PIXEL_FORMAT_BAYER_RGGB)) {
		*cspace = V4L2_COLORSPACE_SRGB;
		return V4L2_PIX_FMT_SRGGB8;
	}
	if (!strcmp(fmt, CameraParameters::PIXEL_FORMAT_JPEG)) {
		*cspace = V4L2_COLORSPACE_JPEG;
		return V4L2_PIX_FMT_JPEG;
	}

	return 0;
}

void camera_param_add_supported_format(struct camera_param *cp, uint32_t fourcc)
{
	CameraParameters *p = cp->parm;
	/* Add a format to both preview and picture */
	const char *newfmt = camera_param_fourcc2str(fourcc);

	if (!newfmt)
		return;

	const char *formats = p->get(CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS);
	String8 nf(newfmt);

	if (formats)
		nf += String8(",") + formats;

	CAM_LOG("new %s", nf.string());

	p->set(CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS, nf.string());
	p->set(CameraParameters::KEY_SUPPORTED_PICTURE_FORMATS, nf.string());
}

void camera_param_add_picture_format(struct camera_param *cp, uint32_t fourcc)
{
	CameraParameters *p = cp->parm;
	/* Add a format to picture formats */
	const char *newfmt = camera_param_fourcc2str(fourcc);

	if (!newfmt)
		return;

	const char *formats = p->get(CameraParameters::KEY_SUPPORTED_PREVIEW_FORMATS);
	String8 nf(newfmt);

	if (formats)
		nf += String8(",") + formats;

	CAM_LOG("new %s", nf.string());

	p->set(CameraParameters::KEY_SUPPORTED_PICTURE_FORMATS, nf.string());
}

void camera_param_parse_fmts(struct camera_param *cp, const char *parms)
{
	String8 str(parms);
	CameraParameters *p = cp->parm;

	if (!strlen(parms))
		return;

	p->unflatten(str);

	const char *preview_format = p->getPreviewFormat();
	const char *picture_format = p->getPictureFormat();
	const char *record_format = p->get(CameraParameters::KEY_VIDEO_FRAME_FORMAT);
	int width, height;

	p->getPreviewSize(&width, &height);
	cp->preview_fmt.width = width;
	cp->preview_fmt.height = height;
	cp->preview_fmt.fourcc = camera_param_str2fourcc(preview_format,
							 &cp->preview_fmt.cspace);

	CAM_LOG("%s(), preview: %x @ %ux%u", __func__, cp->preview_fmt.fourcc, width, height);

	p->getPictureSize(&width, &height);
	cp->picture_fmt.width = width;
	cp->picture_fmt.height = height;
	cp->picture_fmt.fourcc = camera_param_str2fourcc(picture_format,
							 &cp->picture_fmt.cspace);

	p->getVideoSize(&width, &height);
	cp->record_fmt.width = width;
	cp->record_fmt.height = height;
	cp->record_fmt.fourcc = camera_param_str2fourcc(record_format,
							&cp->record_fmt.cspace);
}

};
