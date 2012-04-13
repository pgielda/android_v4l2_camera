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
 * @file v4l_camera/camera_param.h
 * @author Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 * @brief C++ camera-parameter interface header
 */

#include <linux/videodev2.h>

#ifdef __cplusplus
#include <camera/CameraParameters.h>

namespace android {

extern "C" {
#endif

struct camera_format {
	unsigned int width;
	unsigned int height;
	uint32_t fourcc;
	enum v4l2_colorspace cspace;
};

struct camera_param {
	struct camera_format	preview_fmt;
	struct camera_format	record_fmt;
	struct camera_format	picture_fmt;
#ifdef __cplusplus
	CameraParameters	*parm;
#else
	void			*parm;
#endif
};

void camera_param_init(struct camera_param *cp);
void camera_param_free(struct camera_param *cp);
void camera_param_put(struct camera_param *cp, char *param);
char *camera_param_get(struct camera_param *cp);
void camera_param_set(struct camera_param *cp);
void camera_param_parse_fmts(struct camera_param *cp, const char *parms);
void camera_param_add_supported_format(struct camera_param *cp, uint32_t fourcc);
void camera_param_add_picture_format(struct camera_param *cp, uint32_t fourcc);
const char *camera_param_fourcc2str(uint32_t fourcc);

#ifdef __cplusplus
}

};
#endif
