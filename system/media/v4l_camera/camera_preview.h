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
 * @file v4l_camera/camera_preview.h
 * @author Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 * @brief C++ camera-preview and frame-manipulation interface header
 */

#include <sys/types.h>
#include <system/window.h>

#include <camera.h>

#ifdef __cplusplus
namespace android {

extern "C" {
#endif

struct camera_callback {
	camera_notify_callback		notify;
	camera_data_callback		data;
	camera_data_timestamp_callback	data_ts;
	camera_request_memory		get_mem;
	void				*arg;
};

typedef void (*camera_frame_converter)(const void *in, void *out, int width, int height);

uint8_t *camera_preview_lock(buffer_handle_t* buffer, unsigned int width, unsigned int height);
void camera_preview_unlock(buffer_handle_t* buffer);
camera_frame_converter camera_frame_converter_select(uint32_t fourcc, int fb_fmt);
int camera_frame_compress(struct camera_callback *cb, uint8_t *in,
			  unsigned int width, unsigned int height);

#ifdef __cplusplus
}

};
#endif
