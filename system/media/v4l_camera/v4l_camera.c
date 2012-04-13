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
 * @file v4l_camera/v4l_camera.c
 * @author Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 * @brief generic V4L2 camera HAL
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#define LOG_TAG "V4L2_Camera"

#include <utils/Errors.h>
#include <utils/Log.h>
#include <utils/Timers.h>

#include <camera.h>

#include "camera_param.h"
#include "camera_preview.h"

//#define CAM_LOG LOGE
#define CAM_LOG LOGI

/* These are random numbers so far */
#define NUM_BUF_MAX	4
#define NUM_BUF_PICTURE	3
#define NUM_BUF_PREVIEW	(NUM_BUF_MAX - 1)
#define NUM_BUF_RECORD	NUM_BUF_MAX

#define min(a, b) ({typeof(a) __a = (a); typeof(b) __b = (b); __a > __b ? __b : __a;})

struct buf_desc {
	/* TODO: add a union to support multi-plane */
	struct camera_memory	*cam_mem;
	uint32_t		offset;
};

/**
 * @index		- starting index of buffers in this set
 * @num_buffers		- number of buffers in this set. NOTE: with MMAP this is
 *			  the number of elements in buffers[], with USERPTR we
 *			  use only one camera_memory object with @num_buffers
 *			  elements in it.
 * @buffers		- camera_memory objects (see above)
 */
struct buf_set {
	struct v4l2_format	fmt;
	enum v4l2_memory        memory;
	unsigned int		num_buffers;
	unsigned int		index;
	struct buf_desc		buffers[NUM_BUF_MAX];
};

typedef _Bool bool;

enum {
	false	= 0,
	true	= 1
};

struct v4l_cam {
	struct camera_device	acam;
	struct camera_callback	cb;
	preview_stream_ops_t	*preview_ops;
	unsigned int		preview_width;
	unsigned int		preview_height;
	int			fb_fmt;
	struct camera_param	acp;
	int			id;
	int			fd;
	int32_t			msg;
	int32_t			msg_mask;
	uint32_t		cap;
	bool			exit : 1;
	bool			configured : 1;
	bool			picture : 1;
	bool			preview : 1;
	bool			record : 1;
	bool			record_start : 1;
	bool			record_stop : 1;
	bool			preview_lock : 1;
	struct buf_set		buf_preview;
	struct buf_set		buf_picture;
	struct buf_set		buf_record;
	struct buf_set		*buf_record_active;
	int			preview_pipe[2];
	pthread_t		thread_preview;
	pthread_t		picture_prepare;
	pthread_mutex_t		thread_mutex;
	pthread_cond_t		preview_cond;
	pthread_cond_t		stop_cond;
	uint32_t		picture_compressed_4cc;
	camera_frame_converter	converter;
};

static struct camera_info cam_info_set[] = {
	{
		.facing		= CAMERA_FACING_FRONT,
		.orientation	= 0,
	}, {
		.facing		= CAMERA_FACING_BACK,
		.orientation	= 0,
	},
};

/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 */
#define container_of(ptr, type, member) ({			\
	const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (char *)__mptr - offsetof(type,member) );})

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/*	***			V4L2 API		***	*/

static void v4l_cam_streaming_stop(struct v4l_cam *cam, struct buf_set *set)
{
	enum v4l2_buf_type type = set->fmt.type;
	ioctl(cam->fd, VIDIOC_STREAMOFF, &type);
}

static int v4l_cam_streaming_start(struct v4l_cam *cam, struct buf_set *set)
{
	enum v4l2_buf_type type = set->fmt.type;
	return ioctl(cam->fd, VIDIOC_STREAMON, &type) < 0 ? -errno : 0;
}

static int v4l_cam_format_set(struct v4l_cam *cam, struct v4l2_format *fmt)
{
	/* We also want to support multiplane formats */
	return ioctl(cam->fd, VIDIOC_S_FMT, fmt) < 0 ? -errno : 0;
}

static void v4l_cam_ubuf_free(struct buf_set *set)
{
	struct camera_memory *cam_mem = set->buffers[0].cam_mem;
	cam_mem->release(cam_mem);
}

static void v4l_cam_mbuf_free(struct buf_set *set)
{
	unsigned int i;

	for (i = 0; i < set->num_buffers; i++) {
		struct camera_memory *cam_mem = set->buffers[i].cam_mem;
		if (!cam_mem)
			continue;
		cam_mem->release(cam_mem);
	}
}

static void v4l_cam_free_buffers(struct buf_set *set)
{
	switch (set->memory) {
	case V4L2_MEMORY_USERPTR:
		v4l_cam_ubuf_free(set);
		break;
	case V4L2_MEMORY_MMAP:
		v4l_cam_mbuf_free(set);
		break;
	default:
		break;
	}
}

/*
 * So far buffers of different .memory types cannot be mixed, also, even if
 * mixing different .type works, a REQBUFS(count = 0) would, probably, free all
 * buffers anyway. So, for now a single REQBUFS should actually suffice.
 * Besides, an initialisation failure will lead to close(fd), upon which kernel
 * video buffers will be freed, so, in principle we only have to munmap() MMAP
 * and free() USERPTR buffers. Note, that this function is also called on the
 * error path, therefore it must handle incompletely initialised buffers too.
 */
static void v4l_cam_destroy_buffers(struct v4l_cam *cam)
{
	struct v4l2_requestbuffers req = {
		.count = 0,	/* release all buffers */
		.type = cam->buf_preview.fmt.type,
		.memory = cam->buf_preview.memory,
	};

	v4l_cam_free_buffers(&cam->buf_preview);
	v4l_cam_free_buffers(&cam->buf_record);
	v4l_cam_free_buffers(&cam->buf_picture);

	ioctl(cam->fd, VIDIOC_REQBUFS, &req);
#if 0
	req.type = cam->buf_picture.type;
	req.memory = cam->buf_picture.memory;
	ioctl(cam->fd, VIDIOC_REQBUFS, &req);
	req.type = cam->buf_record.type;
	req.memory = cam->buf_record.memory;
	ioctl(cam->fd, VIDIOC_REQBUFS, &req);
#endif
}

#define v4l_format_pixelformat(f) ((f)->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ? \
				   (f)->fmt.pix.pixelformat : (f)->fmt.pix_mp.pixelformat)
#define v4l_format_field(f) ((f)->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ? \
				   (f)->fmt.pix.field : (f)->fmt.pix_mp.field)
#define v4l_format_width(f) ((f)->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ? \
				   (f)->fmt.pix.width : (f)->fmt.pix_mp.width)
#define v4l_format_height(f) ((f)->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ? \
				   (f)->fmt.pix.height : (f)->fmt.pix_mp.height)

static int v4l_cam_buf_create(struct v4l_cam *cam, struct buf_set *set, int n)
{
	struct v4l2_create_buffers create = {
		.memory = set->memory,
		.count = n,
		.format = set->fmt,
	};
	struct v4l2_buffer buf = {
		.type = set->fmt.type,
		.memory = set->memory,
	};
	/*
	 * If we ever have to support V4L2 devices, not implementing the
	 * VIDIOC_CREATE_BUFS ioctl(), we'll have to use REQBUFS for all our
	 * video buffers
	 */
	unsigned int i;
	int ret = ioctl(cam->fd, VIDIOC_CREATE_BUFS, &create);

	CAM_LOG("CREATE_BUFS for %x @ %ux%u: %d type %d",
		v4l_format_pixelformat(&set->fmt),
		v4l_format_width(&set->fmt), v4l_format_height(&set->fmt),
		ret, set->fmt.type);
	if (ret < 0)
		return -errno;

	buf.field = v4l_format_field(&set->fmt);

	for (i = create.index; i < create.index + create.count; i++) {
		buf.index = i;
		ret = ioctl(cam->fd, VIDIOC_PREPARE_BUF, &buf);
		if (ret < 0)
			break;
	}

	CAM_LOG("PREPARE_BUF successful for %d buffers of %d: %d",
		i - create.index, create.count, errno);

	if (i == create.index)
		/* already the first PREPARE_BUF above failed */
		return -errno;

	set->index = create.index;
	set->num_buffers = create.count;

	return 0;
}

/**
 * v4l_cam_frame_data - return memory address
 * @set:		buffer set
 * @idx:		index within the set
 */
static uint8_t *v4l_cam_frame_data(const struct buf_set *set, int idx)
{
	struct camera_memory *cmem;
	const struct buf_desc *desc;

	switch (set->memory) {
	case V4L2_MEMORY_USERPTR:
		cmem = set->buffers[0].cam_mem;
		return (uint8_t *)cmem->data + cmem->size * idx;
	case V4L2_MEMORY_MMAP:
		desc = set->buffers + idx;
		return (uint8_t *)desc->cam_mem->data;
	default:
		return NULL;
	}
}

static int v4l_cam_mmem_by_addr(const struct buf_set *set, const uint8_t *ptr)
{
	const struct buf_desc *desc;
	unsigned int i;

	for (i = 0, desc = set->buffers; i < set->num_buffers; i++, desc++)
		if (ptr == (uint8_t *)desc->cam_mem->data)
			return i;

	LOGE("MMAP: Buffer %p not found!\n", ptr);

	return -ENOENT;
}

static int v4l_cam_umem_by_addr(const struct buf_set *set, const uint8_t *ptr)
{
	struct camera_memory *cmem = set->buffers[0].cam_mem;
	unsigned int i = (ptr - (uint8_t *)cmem->data) / cmem->size;

	if (i < set->num_buffers && i * cmem->size + (uint8_t *)cmem->data == ptr)
		return i;

	LOGE("USERPTR: Buffer %p not found!\n", ptr);

	return -ENOENT;
}

static int v4l_cam_mem_by_addr(const struct v4l_cam *cam, const uint8_t *ptr)
{
	const struct buf_set *set = cam->buf_record_active;

	switch (set->memory) {
	case V4L2_MEMORY_USERPTR:
		return v4l_cam_umem_by_addr(set, ptr);
	case V4L2_MEMORY_MMAP:
		return v4l_cam_mmem_by_addr(set, ptr);
	default:
		return -EINVAL;
	}
}

static int v4l_cam_ubuf_init(struct v4l_cam *cam, struct buf_set *set)
{
	size_t sizeimage;
	int i;

	switch (set->fmt.type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		sizeimage = set->fmt.fmt.pix.sizeimage;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		for (i = 0, sizeimage = 0;
		     i < min(VIDEO_MAX_PLANES, set->fmt.fmt.pix_mp.num_planes);
		     i++)
			sizeimage += set->fmt.fmt.pix_mp.plane_fmt[i].sizeimage;
		break;
	default:
		return -EINVAL;
	}
	set->buffers[0].cam_mem = cam->cb.get_mem(-1, sizeimage, set->num_buffers,
						  0, cam->cb.arg);
	set->buffers[0].offset = 0;
	return set->buffers[0].cam_mem ? 0 : -ENOMEM;
}

static int v4l_cam_mbuf_init(struct v4l_cam *cam, struct buf_set *set)
{
	struct v4l2_buffer buf = {
		.type	= set->fmt.type,
		.memory	= set->memory,
	};
	unsigned int i;
	int ret;

	for (i = 0; i < set->num_buffers; i++) {
		buf.index = set->index + i;

		ret = ioctl(cam->fd, VIDIOC_QUERYBUF, &buf);
		if (ret < 0)
			return -errno;

		set->buffers[i].cam_mem = cam->cb.get_mem(cam->fd, buf.length,
							  1, buf.m.offset, cam->cb.arg);
		set->buffers[i].offset = buf.m.offset;
	}

	return 0;
}

static int v4l_cam_buf_init(struct v4l_cam *cam, struct buf_set *set)
{
	switch (set->memory) {
	case V4L2_MEMORY_USERPTR:
		return v4l_cam_ubuf_init(cam, set);
	case V4L2_MEMORY_MMAP:
		return v4l_cam_mbuf_init(cam, set);
	default:
		return -EINVAL;
	}
}

static int v4l_cam_configure(struct v4l_cam *cam)
{
	struct v4l2_requestbuffers req = {
		.count = NUM_BUF_PREVIEW,
		.type = cam->buf_preview.fmt.type,
		.memory = cam->buf_preview.memory,
	};
	int ret = v4l_cam_format_set(cam, &cam->buf_preview.fmt);

	CAM_LOG("%s(): S_FMT() = %d", __func__, ret);

	if (ret < 0 || cam->configured)
		return ret;

	ret = ioctl(cam->fd, VIDIOC_REQBUFS, &req);
	if (ret < 0)
		return -errno;

	cam->buf_preview.num_buffers = req.count;
	cam->buf_preview.index = 0;

	ret = v4l_cam_buf_create(cam, &cam->buf_picture, NUM_BUF_PICTURE);
	if (ret < 0)
		goto err;

	ret = v4l_cam_buf_create(cam, &cam->buf_record, NUM_BUF_RECORD);

	if (!ret)
		ret = v4l_cam_buf_init(cam, &cam->buf_preview);
	if (!ret)
		ret = v4l_cam_buf_init(cam, &cam->buf_picture);
	if (!ret)
		ret = v4l_cam_buf_init(cam, &cam->buf_record);

	if (!ret)
		return 0;
err:
	v4l_cam_destroy_buffers(cam);
	return ret;
}

/*
 * Is it enough to support V4L2_MEMORY_MMAP or do we also have to support
 * V4L2_MEMORY_USERPTR? Verify preview_stream_ops_t to see, where preview
 * buffers are allocated.
 */
static int v4l_cam_buf_queue(struct v4l_cam *cam, struct buf_set *set, unsigned int n)
{
	struct v4l2_buffer buf = {
		.type	= set->fmt.type,
		.memory	= set->memory,
		.index	= set->index + n,
	};

	if (n >= set->num_buffers)
		return -ENOMEM;

	/* TODO: support V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE */
	switch (set->memory) {
	case V4L2_MEMORY_USERPTR:
		/* TODO: test USERPTR */
		buf.m.userptr	= (unsigned long)set->buffers[n].cam_mem->data;
		buf.length	= set->buffers[n].cam_mem->size;
		break;
	default:
		return -ENOSYS;
	case V4L2_MEMORY_MMAP:
		break;
	}

	return ioctl(cam->fd, VIDIOC_QBUF, &buf) < 0 ? -errno : 0;
}

static int v4l_cam_buf_dequeue(struct v4l_cam *cam, struct buf_set *set)
{
	struct v4l2_buffer buf = {
		.type	= set->fmt.type,
		.memory	= set->memory,
	};

	return ioctl(cam->fd, VIDIOC_DQBUF, &buf) < 0 ? -errno : (int)buf.index;
}

/*	***		Android camera API		***	*/

/**
 * locking: call only with thread_mutex held!
 */
static int v4l_cam_update_preview_geometry(struct v4l_cam *cam)
{
	struct preview_stream_ops *ops = cam->preview_ops;

	if (ops && (cam->preview_width != cam->acp.preview_fmt.width ||
		    cam->preview_height != cam->acp.preview_fmt.height)) {
		int ret = ops->set_buffers_geometry(ops,
						    cam->acp.preview_fmt.width,
						    cam->acp.preview_fmt.height,
						    cam->fb_fmt);
		if (ret != NO_ERROR)
			return -EREMOTEIO;

		cam->preview_width = cam->acp.preview_fmt.width;
		cam->preview_height = cam->acp.preview_fmt.height;
	}

	return 0;
}

/* Set the preview_stream_ops to which preview frames are sent */
static int v4l_cam_set_preview_window(struct camera_device *dev,
				      struct preview_stream_ops *ops)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);
	int ret;

	pthread_mutex_lock(&cam->thread_mutex);
	cam->preview_width = cam->preview_height = 0;
	cam->preview_ops = ops;
	ret = v4l_cam_update_preview_geometry(cam);
	pthread_mutex_unlock(&cam->thread_mutex);

	CAM_LOG("%s(%p)", __func__, ops);

	return ret;
}

/* Set the notification and data callbacks */
static void v4l_cam_set_callbacks(struct camera_device *dev,
        camera_notify_callback notify_cb,
        camera_data_callback data_cb,
        camera_data_timestamp_callback data_cb_timestamp,
        camera_request_memory get_memory,
        void *arg)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);

	CAM_LOG("%s()", __func__);

	cam->cb.arg	= arg;
	cam->cb.notify	= notify_cb;
	cam->cb.data	= data_cb;
	cam->cb.data_ts	= data_cb_timestamp;
	cam->cb.get_mem	= get_memory;
}

/*
 * The following three functions all take a msg_type, which is a bitmask of
 * the messages defined in include/ui/Camera.h
 */

#define MSG_MASK (CAMERA_MSG_ERROR | CAMERA_MSG_SHUTTER |		\
		  CAMERA_MSG_PREVIEW_FRAME | CAMERA_MSG_VIDEO_FRAME |	\
		  CAMERA_MSG_RAW_IMAGE | CAMERA_MSG_RAW_IMAGE_NOTIFY)

/* Enable a message, or set of messages. */
static void v4l_cam_enable_msg_type(struct camera_device *dev, int32_t msg_type)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);

	CAM_LOG("%s(0x%x)", __func__, msg_type);
	cam->msg |= msg_type & cam->msg_mask;
}

/* Disable a message, or a set of messages. */
static void v4l_cam_disable_msg_type(struct camera_device *dev, int32_t msg_type)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);

	CAM_LOG("%s(0x%x)", __func__, msg_type);
	cam->msg &= ~msg_type;
}

/* Return true, if _all_ requested messages are enabled */
static int v4l_cam_msg_type_enabled(struct camera_device *dev, int32_t msg_type)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);

	CAM_LOG("%s()", __func__);
	return (cam->msg & msg_type) == msg_type;
}

static int v4l_cam_cfmt2v4l2(struct v4l_cam *cam, struct camera_format *cfmt,
			     struct v4l2_format *vfmt, uint32_t cap)
{
	int ret;

	if (cap & V4L2_CAP_VIDEO_CAPTURE) {
		/* Prefer the simple case for now */
		vfmt->type			= V4L2_BUF_TYPE_VIDEO_CAPTURE;
		vfmt->fmt.pix.width		= cfmt->width;
		vfmt->fmt.pix.height		= cfmt->height;
		vfmt->fmt.pix.pixelformat	= cfmt->fourcc;
		vfmt->fmt.pix.colorspace	= cfmt->cspace;
		vfmt->fmt.pix.field		= V4L2_FIELD_ANY;
	} else if (cap & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
		vfmt->type			= V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		vfmt->fmt.pix_mp.width		= cfmt->width;
		vfmt->fmt.pix_mp.height		= cfmt->height;
		vfmt->fmt.pix_mp.pixelformat	= cfmt->fourcc;
		vfmt->fmt.pix_mp.colorspace	= cfmt->cspace;
		vfmt->fmt.pix_mp.field		= V4L2_FIELD_ANY;
	} else {
		/* BUG: impossible, we checked this in .open()! */
		return -EINVAL;
	}

	ret = ioctl(cam->fd, VIDIOC_TRY_FMT, vfmt);
	if (ret < 0 && errno == EINVAL && ((cap & (V4L2_CAP_VIDEO_CAPTURE |
						   V4L2_CAP_VIDEO_CAPTURE_MPLANE)) ==
					   (V4L2_CAP_VIDEO_CAPTURE |
					    V4L2_CAP_VIDEO_CAPTURE_MPLANE)))
		/*
		 * One more chance: if the driver supports both buffer types,
		 * and CAPTURE failed, try CAPTURE_MPLANE
		 */
		return v4l_cam_cfmt2v4l2(cam, cfmt, vfmt,
					 V4L2_CAP_VIDEO_CAPTURE_MPLANE);

	CAM_LOG("%s() type %d", __func__, vfmt->type);

	return ret < 0 ? -errno : 0;
}

#include <linux/fb.h>

#define FBDEV "/dev/graphics/fb0"

#define v4l_cam_try_buftype(cam, cmd, arg)				\
({									\
	int __ret;							\
	/* Try single plane first */					\
	if (cam->cap & V4L2_CAP_VIDEO_CAPTURE) {			\
		(arg)->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;		\
		__ret = ioctl(cam->fd, cmd, (arg));			\
		__ret = __ret < 0 ? -errno : 0;				\
	} else {							\
		__ret = -EINVAL;					\
	}								\
	if (__ret == -EINVAL &&						\
	    (cam->cap & V4L2_CAP_VIDEO_CAPTURE_MPLANE)) {		\
		(arg)->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;	\
		ioctl(cam->fd, cmd, (arg));				\
		__ret = -errno;						\
	}								\
	__ret;								\
})

/* I think the compiler will optimize constant false if's away */
#define v4l_cam_make_fmt(f, p, c, i, w, h, bug) do {			\
	switch ((f)->type) {						\
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:				\
		if (p)							\
			(f)->fmt.pix.pixelformat = p;			\
		if (c)							\
			(f)->fmt.pix.colorspace = c;			\
		if (i > 0)						\
			(f)->fmt.pix.field = i;				\
		if (w)							\
			(f)->fmt.pix.width = w;				\
		if (h)							\
			(f)->fmt.pix.height = h;			\
		break;							\
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:			\
		if (p)							\
			(f)->fmt.pix_mp.pixelformat = p;		\
		if (c)							\
			(f)->fmt.pix_mp.colorspace = c;			\
		if (i > 0)						\
			(f)->fmt.pix_mp.field = i;			\
		if (w)							\
			(f)->fmt.pix_mp.width = w;			\
		if (h)							\
			(f)->fmt.pix_mp.height = h;			\
		break;							\
	default:							\
		bug;							\
	}								\
} while (0)

#define v4l_cam_try_or_bug(fd, fmt) do {				\
	if (ioctl((fd), VIDIOC_TRY_FMT, (fmt)) < 0) {			\
		LOGE("%s(): buggy camera driver! VIDIOC_TRY_FMT should never fail!", \
			__func__);					\
		return -errno;						\
	}								\
} while (0)

/* This is all just guessing, really */
static int v4l_cam_default_fmts(struct v4l_cam *cam)
{
	struct v4l2_format fmt = {.fmt.raw_data = {0}};
	/* Take the first format from enumeration */
	struct v4l2_fmtdesc fmtdesc = {.index = 0,};
	struct camera_format *pvu = &cam->acp.preview_fmt;
	unsigned int width, height, fourcc;
	int fbdev;
	int bpp;
	int ret;

	fbdev = open(FBDEV, O_RDONLY);
	if (fbdev >= 0) {
		struct fb_var_screeninfo vinfo;
		ret = ioctl(fbdev, FBIOGET_VSCREENINFO, &vinfo);
		close(fbdev);
		if (!ret) {
			width = vinfo.xres;
			height = vinfo.yres;
			bpp = vinfo.bits_per_pixel;
			CAM_LOG("%s() fbdev %ux%u:%d", __func__, width, height, bpp);
		} else {
			fbdev = -1;
		}
	}

	if (fbdev < 0) {
		width = 640;
		height = 480;
		bpp = 16;
	}

	/*
	 * It is assumed, that the preview display uses one of the below
	 * formats or the closest to it. For the camera we try to find a
	 * matching format among supported ones. Otherwise we pick up the first
	 * one, supported by Android.
	 */
	switch (bpp) {
	case 15:
		fourcc = V4L2_PIX_FMT_RGB555;
		cam->fb_fmt = HAL_PIXEL_FORMAT_RGBA_5551;
		break;
	case 16:
		fourcc = V4L2_PIX_FMT_RGB565;
		cam->fb_fmt = HAL_PIXEL_FORMAT_RGB_565;
		break;
	case 24:
		fourcc = V4L2_PIX_FMT_BGR24;
		cam->fb_fmt = HAL_PIXEL_FORMAT_RGB_888;
		break;
	case 32:
		fourcc = V4L2_PIX_FMT_BGR32;
		cam->fb_fmt = HAL_PIXEL_FORMAT_RGBA_8888;
		break;
	default:
		return -EINVAL;
	}

	while (!v4l_cam_try_buftype(cam, VIDIOC_ENUM_FMT, &fmtdesc)) {
		if (!fmt.type)
			fmt.type = fmtdesc.type;

		if (fmtdesc.pixelformat == fourcc ||
		    (!pvu->fourcc && camera_param_fourcc2str(fmtdesc.pixelformat)))
			pvu->fourcc = fmtdesc.pixelformat;

		camera_param_add_supported_format(&cam->acp, fmtdesc.pixelformat);

		if (fmtdesc.pixelformat == V4L2_PIX_FMT_JPEG)
			cam->picture_compressed_4cc = V4L2_PIX_FMT_JPEG;
		else if (fmtdesc.pixelformat == V4L2_PIX_FMT_NV21 &&
			 !cam->picture_compressed_4cc)
			cam->picture_compressed_4cc = V4L2_PIX_FMT_NV21;

		fmtdesc.index++;
	}

	if (!fmt.type || !pvu->fourcc)
		return -ENODEV;

	CAM_LOG("%s(): type %x, fourcc %x", __func__, fmt.type, pvu->fourcc);

	/* TODO: determine colorspace, if needed */
	v4l_cam_make_fmt(&fmt, pvu->fourcc, 0, V4L2_FIELD_ANY, width, height, return -EINVAL);
	/*
	 * We assume this TRY_FMT doesn't change the pixel format, since we're
	 * using a definitely supported one
	 */
	v4l_cam_try_or_bug(cam->fd, &fmt);

	cam->converter = camera_frame_converter_select(pvu->fourcc, cam->fb_fmt);
	if (!cam->converter)
		return -EINVAL;

	switch (fmt.type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pvu->width = fmt.fmt.pix.width;
		pvu->height = fmt.fmt.pix.height;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		pvu->width = fmt.fmt.pix_mp.width;
		pvu->height = fmt.fmt.pix_mp.height;
		break;
	default:
		/* BUG */
		return -EINVAL;
	}

	cam->acp.record_fmt = *pvu;
	cam->acp.picture_fmt = *pvu;

	if (cam->picture_compressed_4cc) {
		cam->msg_mask |= CAMERA_MSG_COMPRESSED_IMAGE;

		if (cam->picture_compressed_4cc == V4L2_PIX_FMT_JPEG)
			v4l_cam_make_fmt(&fmt, V4L2_PIX_FMT_JPEG, V4L2_COLORSPACE_JPEG,
					 -1, 0, 0, return -EINVAL);
		else {
			camera_param_add_picture_format(&cam->acp, V4L2_PIX_FMT_JPEG);
			v4l_cam_make_fmt(&fmt, V4L2_PIX_FMT_NV21, V4L2_COLORSPACE_JPEG,
					 -1, 0, 0, return -EINVAL);
		}

		v4l_cam_try_or_bug(cam->fd, &fmt);
		cam->acp.picture_fmt.fourcc = v4l_format_pixelformat(&fmt);
	}

	return 0;
}

/*
 * Set the camera parameters. This returns BAD_VALUE if any parameter is
 * invalid or not supported. Samsung doesn't perform any hardware configuration
 * here, only upon .start_preview(), only verifies and records parameters here.
 */
static int v4l_cam_set_parameters(struct camera_device *dev,
				  const char *parms)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);
	int ret;

	CAM_LOG("%s(%s)", __func__, parms);

	if (parms) {
		camera_param_parse_fmts(&cam->acp, parms);
		/*
		 * TODO: implement independent recording, for now we only
		 * support fmt(record) == fmt(preview)
		 */
		cam->acp.record_fmt = cam->acp.preview_fmt;
	} else {
		ret = v4l_cam_default_fmts(cam);
		if (ret < 0)
			return ret;
	}

	ret = v4l_cam_cfmt2v4l2(cam, &cam->acp.preview_fmt,
				&cam->buf_preview.fmt, cam->cap);
	if (ret < 0)
		return ret;
	ret = v4l_cam_cfmt2v4l2(cam, &cam->acp.record_fmt,
				&cam->buf_record.fmt, cam->cap);
	if (ret < 0)
		return ret;
	ret = v4l_cam_cfmt2v4l2(cam, &cam->acp.picture_fmt,
				&cam->buf_picture.fmt, cam->cap);
	/* Do not reconfigure if preview is running */
	if (ret < 0 || cam->preview)
		return ret;

	/* set preview format and allocate buffer sets */
	ret = v4l_cam_configure(cam);
	CAM_LOG("%s() configure() = %d", __func__, ret);
	if (ret < 0)
		return ret;

	cam->configured = true;

	return 0;
}

static int v4l_cam_buf_queue_set(struct v4l_cam *cam, struct buf_set *set)
{
	unsigned int n;
	int ret = -ENOBUFS;

	for (n = 0; n < set->num_buffers; n++) {
		ret = v4l_cam_buf_queue(cam, set, n);
		if (ret < 0)
			break;
	}

	/* Any buffers queued successfully? */
	return n ? (int)n : ret;
}

/* Start preview mode. */
static int v4l_cam_start_preview(struct camera_device *dev)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);
	int ret;

	CAM_LOG("%s()", __func__);

	if (cam->preview)
		return 0;

	if (!cam->configured) {
		ret = v4l_cam_set_parameters(dev, NULL);
		if (ret < 0)
			return ret;
	}

	pthread_mutex_lock(&cam->thread_mutex);
	ret = v4l_cam_update_preview_geometry(cam);
	pthread_mutex_unlock(&cam->thread_mutex);

	if (ret < 0)
		return ret;

	ret = v4l_cam_buf_queue_set(cam, &cam->buf_preview);
	if (ret < 0)
		return ret;

	pthread_mutex_lock(&cam->thread_mutex);

	cam->preview = true;

	pthread_cond_signal(&cam->preview_cond);
	pthread_mutex_unlock(&cam->thread_mutex);

	return 0;
}

/* Stop a previously started preview. */
static void v4l_cam_stop_preview(struct camera_device *dev)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);
	uint8_t z = 0;

	if (!cam->preview)
		return;

	cam->preview = false;
	/*
	 * Android calls various camera HAL methods asynchronously from
	 * different execution threads. We have to take care to prevent races
	 * and lock-ups ourselves. One of the requirements in this case is, that
	 * we shall not return from our .stop_preview() method until we really
	 * make sure, that the preview thread is safely parked and is ready to
	 * take a .start_preview() call.
	 */
	write(cam->preview_pipe[0], &z, 1);

	pthread_mutex_lock(&cam->thread_mutex);
	pthread_cond_wait(&cam->stop_cond,
			  &cam->thread_mutex);
	pthread_mutex_unlock(&cam->thread_mutex);

	v4l_cam_streaming_stop(cam, &cam->buf_preview);
	CAM_LOG("%s(): streaming stopped", __func__);
}

/* Returns true if preview is enabled. */
static int v4l_cam_preview_enabled(struct camera_device *dev)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);

	CAM_LOG("%s()", __func__);
	return cam->preview;
}

/* See comments in hardware/libhardware/include/hardware/camera.h */
static int v4l_cam_store_meta_data_in_buffers(struct camera_device *dev, int meta)
{
	CAM_LOG("%s(%d)", __func__, meta);
	if (meta)
		return INVALID_OPERATION;
	return OK;
}

static void v4l_cam_picture_deliver(struct v4l_cam *cam, int idx)
{
	struct buf_set *set = &cam->buf_picture;

	if (cam->msg & CAMERA_MSG_SHUTTER)
		cam->cb.notify(CAMERA_MSG_SHUTTER, 0, 0, cam->cb.arg);

	if (cam->msg & CAMERA_MSG_RAW_IMAGE_NOTIFY)
		cam->cb.notify(CAMERA_MSG_RAW_IMAGE_NOTIFY, 0, 0, cam->cb.arg);

	if (cam->msg & CAMERA_MSG_RAW_IMAGE) {
		nsecs_t timestamp = systemTime(SYSTEM_TIME_MONOTONIC);
		switch (set->memory) {
		case V4L2_MEMORY_USERPTR:
			cam->cb.data_ts(timestamp, CAMERA_MSG_RAW_IMAGE,
					set->buffers[0].cam_mem, idx,
					cam->cb.arg);
			break;
		case V4L2_MEMORY_MMAP:
			cam->cb.data_ts(timestamp, CAMERA_MSG_RAW_IMAGE,
					set->buffers[idx].cam_mem, 0,
					cam->cb.arg);
			break;
		default:
			break;
		}
	}

	if (cam->msg & CAMERA_MSG_COMPRESSED_IMAGE) {
		struct camera_format *pic = &cam->acp.picture_fmt;
		camera_frame_compress(&cam->cb, v4l_cam_frame_data(set, idx),
				      pic->width, pic->height);
	}
}

/*
 * We could also prepare other buffers before use: preview while recording or
 * taking a picture and recording buffers while not recording. This would
 * reduce, e.g., recording start latency, but (1) picture frames are typically
 * much larger than preview and video frames, and (2) picture frames are
 * currently only used once per snapshot, whereas video and preview frames are
 * re-used during a running session, so their preparation has to happen "fast
 * enough" anyway.
 */
static void *v4l_cam_picture_prepare(void *arg)
{
	struct v4l_cam *cam = arg;
	struct buf_set *set = &cam->buf_picture;
	struct v4l2_buffer buf = {
		.type = set->fmt.type,
		.memory = set->memory,
	};
	int ret, err = 0;

	for (buf.index = set->index;
	     buf.index < set->index + set->num_buffers;
	     buf.index++) {
		ret = ioctl(cam->fd, VIDIOC_PREPARE_BUF, &buf);
		if (ret < 0)
			err = ret;
	}
	CAM_LOG("%s(): finished preparing %d picture buffers: %d",
		__func__, set->num_buffers, err);
	return (void *)err;
}

/*
 * The case of equal preview and picture formats is rather an exception than a
 * rule, so, we don't optimise for it.
 */
static int v4l_cam_picture(struct v4l_cam *cam)
{
	int ret, n, i, picture_err;
	unsigned int idx;

	/*
	 * Wait for the picture-prepare thread to complete, it shouldn't be
	 * running by now normally
	 */
	pthread_join(cam->picture_prepare, (void **)&ret);
	if (ret < 0)
		CAM_LOG("%s(): Couldn't join the prepare thread: %d",
			__func__, ret);

	v4l_cam_streaming_stop(cam, &cam->buf_preview);

	picture_err = v4l_cam_format_set(cam, &cam->buf_picture.fmt);
	if (picture_err < 0) {
		cam->picture = false;
		goto preview;
	}

	/* From here preview is broken */

	n = v4l_cam_buf_queue_set(cam, &cam->buf_picture);
	if (n < 0) {
		/* No buffers have been queued successfully... */
		picture_err = n;
		cam->picture = false;
		goto preview;
	}

	picture_err = v4l_cam_streaming_start(cam, &cam->buf_picture);
	if (picture_err < 0) {
		cam->picture = false;
		LOGE("%s(): picture capture failed: %d", __func__, picture_err);
		goto preview;
	}

	for (i = 0; i < n; i++) {
		/* Have we been cancelled in the meantime? */
		if (!cam->picture)
			break;
		/* Take .num_picture_buf shots, deliver the last one */
		idx = v4l_cam_buf_dequeue(cam, &cam->buf_picture);
	}

preview:
	v4l_cam_streaming_stop(cam, &cam->buf_picture);

	ret = v4l_cam_format_set(cam, &cam->buf_preview.fmt);
	if (ret < 0)
		return ret;

	pthread_mutex_lock(&cam->thread_mutex);

	if (cam->preview) {
		ret = v4l_cam_buf_queue_set(cam, &cam->buf_preview);
		if (ret > 0)
			ret = v4l_cam_streaming_start(cam, &cam->buf_preview);
	}

	pthread_mutex_unlock(&cam->thread_mutex);

	if (ret < 0)
		return ret;

	/*
	 * Last check: have we been cancelled in the meantime? Note: this can be
	 * preempted by the cancellation method. In that case the user will
	 * first see .cancel_picture() returning, but then a picture delivered,
	 * which is at the very least counterintuitive. To prevent this we would
	 * have to lock the below block, but then a user callback would be
	 * called with our lock held, and if it decides to synchronise with the
	 * user's main thread, we deadlock. Or are user callbacks guaranteed to
	 * run lock-free?
	 */
	if (cam->picture) {
		cam->picture = false;
		v4l_cam_picture_deliver(cam, idx - cam->buf_picture.index);
	}

	ret = pthread_create(&cam->picture_prepare, NULL, v4l_cam_picture_prepare, cam);
	if (ret < 0)
		LOGV("%s(): prepare thread failed: %d, VIDIOC_QBUF will be slower.",
		     __func__, ret);

	return picture_err;
}

/**
 * v4l_cam_frame_done - report completed frame to the user
 * @cam:		pointer to the camera object
 * @set:		buffer set (actually, the preview set)
 * @idx:		index within the set
 */
static int v4l_cam_frame_done(struct v4l_cam *cam, struct buf_set *set, int idx)
{
	struct preview_stream_ops *ops = cam->preview_ops;
	buffer_handle_t* buffer = NULL;
	int ret;
	bool locked = false;

	if (cam->record && (cam->msg & CAMERA_MSG_VIDEO_FRAME)) {
		nsecs_t timestamp = systemTime(SYSTEM_TIME_MONOTONIC);

		LOGV("%s(%d) @ %lld", __func__, idx, timestamp);

		pthread_mutex_lock(&cam->thread_mutex);
		cam->preview_lock = true;
		pthread_mutex_unlock(&cam->thread_mutex);

		/*
		 * TODO: implement independent recording, for now we re-use
		 * preview frames for video
		 */
		switch (set->memory) {
		case V4L2_MEMORY_USERPTR:
			cam->cb.data_ts(timestamp, CAMERA_MSG_VIDEO_FRAME,
					set->buffers[0].cam_mem, idx,
					cam->cb.arg);
			break;
		case V4L2_MEMORY_MMAP:
			cam->cb.data_ts(timestamp, CAMERA_MSG_VIDEO_FRAME,
					set->buffers[idx].cam_mem, 0,
					cam->cb.arg);
			break;
		default:
			break;
		}
	}

	if (cam->msg & CAMERA_MSG_PREVIEW_FRAME) {
		switch (set->memory) {
		case V4L2_MEMORY_USERPTR:
			cam->cb.data(CAMERA_MSG_PREVIEW_FRAME,
				     set->buffers[0].cam_mem, idx, NULL,
				     cam->cb.arg);
			break;
		case V4L2_MEMORY_MMAP:
			cam->cb.data(CAMERA_MSG_PREVIEW_FRAME,
				     set->buffers[idx].cam_mem, 0, NULL,
				     cam->cb.arg);
			break;
		default:
			break;
		}
	}

	/* Are preview notification and preview operations mutually exclusive? */
	if (ops) {
		/* Dequeue preview window buffer for the frame. */
		unsigned int width = cam->acp.preview_fmt.width,
			height = cam->acp.preview_fmt.height;
		void *img;
		int stride = 0;
		ret = ops->dequeue_buffer(ops, &buffer, &stride);
		if (width != 640 || height != 480 || stride != 640)
			CAM_LOG("%s(): Unexpected geometry: %ux%u, stride %u",
				__func__, width, height, stride);
		if (ret != NO_ERROR || !buffer) {
			LOGE("%s(): Unable to dequeue preview window buffer @ %p: %d",
			     __func__, buffer, ret);
			return ret;
		}

		ret = ops->lock_buffer(ops, buffer);
		if (ret != NO_ERROR) {
			LOGE("%s(): Unable to lock preview window buffer: %d",
			     __func__, ret);
			goto fail;
		}

		img = camera_preview_lock(buffer, width, height);
		if (!img) {
			LOGE("%s(): Unable to lock preview window output buffer",
			     __func__);
			ret = -ENOBUFS;
			goto fail;
		}

		/*
		 * FIXME: PreviewWindow::onNextFrameAvailable() in
		 * development/tools/emulator/system/camera/PreviewWindow.cpp
		 * retrieves the output buffer stride, but doesn't seem to use
		 * it. Is it always equal to the width? Can it really be safely
		 * ignored?
		 */
		cam->converter(v4l_cam_frame_data(set, idx), img, width, height);

		pthread_mutex_lock(&cam->thread_mutex);
		if (cam->msg & CAMERA_MSG_VIDEO_FRAME) {
			locked = cam->preview_lock;
			LOGV("%s(%d) %s", __func__, idx, locked ? "wins" : "loses");
			if (locked)
				cam->preview_lock = false;
		}
		pthread_mutex_unlock(&cam->thread_mutex);

		ret = ops->enqueue_buffer(ops, buffer);
		if (cam->record)
			LOGV("%s(): enqueue() = %d", __func__, ret);

		camera_preview_unlock(buffer);
		if (ret != NO_ERROR) {
			LOGE("%s(): Unable to convert preview window buffer: %d",
			     __func__, ret);
			goto fail;
		}
	}

	LOGV("%s(): queue(%d)", __func__, idx);
	/* Always queue next? */
	return cam->msg & CAMERA_MSG_VIDEO_FRAME && locked ? 0 :
		v4l_cam_buf_queue(cam, set, idx);

fail:
	ops->cancel_buffer(ops, buffer);
	return ret;
}

static int v4l_cam_format_switch(struct v4l_cam *cam,
				 struct buf_set *from, struct buf_set *to)
{
	int ret;

	v4l_cam_streaming_stop(cam, from);
	ret = v4l_cam_format_set(cam, &to->fmt);
	if (!ret)
		ret = v4l_cam_buf_queue_set(cam, to);
	if (ret > 0)
		ret = v4l_cam_streaming_start(cam, to);
	return ret;
}

static void *v4l_cam_thread_preview(void *arg)
{
	struct v4l_cam *cam = arg;
	struct buf_set *set = &cam->buf_preview;
	struct pollfd fds[2];
	int ret;
	unsigned int *preview_width = NULL, *preview_height = NULL,
		*record_width = NULL, *record_height = NULL;
	uint32_t *preview_fourcc = NULL, *record_fourcc = NULL;

	close(cam->preview_pipe[1]);
	CAM_LOG("%s(): good morning", __func__);

	pthread_mutex_lock(&cam->thread_mutex);

	while (true) {
		int idx;
		uint8_t z;

		/*
		 * Synchronisation with .cancel_picture(): the main difficulty
		 * is to guarantee, that it doesn't get swapped out between the
		 * check for ->picture and the call to VIDIOC_STREAMOFF, in
		 * which case it would kill the preview. At the same time we
		 * don't want to lock for the whole duration of
		 * v4l_cam_picture() - we want to be able to cancel inside it
		 * too. See comments inside v4l_cam_picture() for more details.
		 */
		if (cam->picture) {
			pthread_mutex_unlock(&cam->thread_mutex);
			ret = v4l_cam_picture(cam);
			pthread_mutex_lock(&cam->thread_mutex);
		}

		if (!cam->preview) {
			bool need_init = !cam->configured;

			/* Preview suspended, .stop_preview() can now return */
			pthread_cond_signal(&cam->stop_cond);

			CAM_LOG("%s(): %sconfigured yawn", __func__, need_init ? "un" : "");

			if (!cam->exit)
				pthread_cond_wait(&cam->preview_cond,
						  &cam->thread_mutex);

			if (cam->exit)
				/* Signal from .release: bye-bye - locked! */
				break;

			pthread_mutex_unlock(&cam->thread_mutex);

			if (need_init) {
				switch (cam->buf_preview.fmt.type) {
				case V4L2_BUF_TYPE_VIDEO_CAPTURE:
					preview_width = &cam->buf_preview.fmt.fmt.pix.width;
					preview_height = &cam->buf_preview.fmt.fmt.pix.height;
					preview_fourcc = &cam->buf_preview.fmt.fmt.pix.pixelformat;
					break;
				case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
					preview_width = &cam->buf_preview.fmt.fmt.pix_mp.width;
					preview_height = &cam->buf_preview.fmt.fmt.pix_mp.height;
					preview_fourcc = &cam->buf_preview.fmt.fmt.pix_mp.pixelformat;
					break;
				default:
					return NULL;
				}

				switch (cam->buf_record.fmt.type) {
				case V4L2_BUF_TYPE_VIDEO_CAPTURE:
					record_width = &cam->buf_record.fmt.fmt.pix.width;
					record_height = &cam->buf_record.fmt.fmt.pix.height;
					record_fourcc = &cam->buf_record.fmt.fmt.pix.pixelformat;
					break;
				case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
					record_width = &cam->buf_record.fmt.fmt.pix_mp.width;
					record_height = &cam->buf_record.fmt.fmt.pix_mp.height;
					record_fourcc = &cam->buf_record.fmt.fmt.pix_mp.pixelformat;
					break;
				default:
					return NULL;
				}
			}

			ret = v4l_cam_streaming_start(cam, &cam->buf_preview);
			pthread_mutex_lock(&cam->thread_mutex);
			if (ret < 0) {
				LOGE("Streaming: STREAMON failed: %d, stopping\n", ret);
				/* RACE: if .start_preview() is called now, you're screwed */
				cam->preview = false;
				continue;
			}
		}
		pthread_mutex_unlock(&cam->thread_mutex);

		fds[0].fd = cam->fd;
		fds[0].events = POLLIN | POLLERR;
		fds[1].fd = cam->preview_pipe[0];
		fds[1].events = POLLIN;
		poll(fds, 2, 0);

		if (fds[1].revents & POLLIN)
			read(cam->preview_pipe[0], &z, 1);

		pthread_mutex_lock(&cam->thread_mutex);

		if (!cam->preview || cam->picture)
			/* Discard the frame: DON'T unlock! */
			continue;

		pthread_mutex_unlock(&cam->thread_mutex);

		LOGV("%s(): hunger!", __func__);

		idx = v4l_cam_buf_dequeue(cam, set);
		if (idx < 0) {
			LOGE("Streaming: DQBUF failed: %d, stopping\n", idx);

			pthread_mutex_lock(&cam->thread_mutex);

			cam->preview = false;
			continue;
		}

		LOGV("%s(%d, ops %p): yumm!", __func__, idx, cam->preview_ops);

		ret = v4l_cam_frame_done(cam, set, idx - set->index);
		if (ret < 0) {
			LOGE("Streaming: QBUF failed: %d, stopping\n", ret);

			pthread_mutex_lock(&cam->thread_mutex);

			cam->preview = false;
			continue;
		}

		if (cam->record_start && !cam->record) {
			cam->record_start = false;
			if (*record_width != *preview_width ||
			    *record_height != *preview_height ||
			    *record_fourcc != *preview_fourcc) {
				/*
				 * The only way to switch to recording buffers
				 * is by stopping streaming, queuing the new
				 * buffers and re-starting streaming.
				 */
				ret = v4l_cam_format_switch(cam, set, &cam->buf_record);
				if (!ret)
					set = &cam->buf_record;
			}
			cam->buf_record_active = set;

			/* If preview and record formats are equal we keep the preview set */
			if (!ret)
				cam->record = true;
			else
				LOGE("%s(): failed to start recording: %d", __func__, ret);
		}

		if (!cam->record) {
			pthread_mutex_lock(&cam->thread_mutex);
			continue;
		}

		if (cam->record_stop) {
			cam->record_stop = false;
			if (set == &cam->buf_record) {
				ret = v4l_cam_format_switch(cam, set, &cam->buf_preview);
				set = &cam->buf_preview;
			}
			cam->record = false;

			if (ret < 0) {
				LOGE("Streaming: preview failed: %d, stopping\n", ret);

				pthread_mutex_lock(&cam->thread_mutex);

				cam->preview = false;
				continue;
			}
		}

		pthread_mutex_lock(&cam->thread_mutex);
	}

	pthread_mutex_unlock(&cam->thread_mutex);

	return NULL;
}

/*
 * Start record mode. When a record image is available, a
 * CAMERA_MSG_VIDEO_FRAME message is sent with the corresponding
 * frame. Take care to handle CAMERA_MSG_VIDEO_FRAME correctly.
 */
static int v4l_cam_start_recording(struct camera_device *dev)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);

	CAM_LOG("%s()", __func__);
	/*
	 * Recording should run concurrently with preview. "Classical" V4L2
	 * doesn't support buffer reuse for multiple consumers: preview and
	 * record in this case. Moreover, preview and record sizes can be
	 * different, so, in the general case we have to configure the video
	 * for the larger of the two formats and downscale frames to produce the
	 * smaller one too. We also assume, that recording is more important,
	 * than preview, so, we want the maximum performance and correct aspect
	 * ratio for it. Therefore we only support 1:1 recording with optionally
	 * scaled preview. Some hardware supports "forking" of data into two
	 * pipes, of which one can even automatically be directed to the preview
	 * device. Some systems will also have hardware video scalers. Support
	 * for such hardware will require a more advanced video recording
	 * implementation.
	 */
	/* The preview thread will switch record on after the next frame */
	if (!cam->record)
		cam->record_start = true;

	return 0;
}

/* Stop a previously started recording. */
static void v4l_cam_stop_recording(struct camera_device *dev)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);

	CAM_LOG("%s()", __func__);

	/* The preview thread will switch record off after the next frame */
	if (cam->record)
		cam->record_stop = true;
}

/* Returns true if recording is enabled. */
static int v4l_cam_recording_enabled(struct camera_device *dev)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);

	CAM_LOG("%s(): %u | %u", __func__, cam->record_start, cam->record);
	return cam->record_start || cam->record;
}

/*
 * Release a record frame previously returned by CAMERA_MSG_VIDEO_FRAME.
 *
 * It is camera HAL client's responsibility to release video recording
 * frames sent out by the camera HAL before the camera HAL receives a call
 * to disableMsgType(CAMERA_MSG_VIDEO_FRAME). After it receives the call to
 * disableMsgType(CAMERA_MSG_VIDEO_FRAME), it is the camera HAL's
 * responsibility to manage the life-cycle of the video recording frames.
 */
static void v4l_cam_release_recording_frame(struct camera_device *dev,
					    const void *opaque)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);
	int n;
	bool locked;

	pthread_mutex_lock(&cam->thread_mutex);
	locked = cam->preview_lock;
	cam->preview_lock = false;
	pthread_mutex_unlock(&cam->thread_mutex);

	if (!cam->record || locked)
		return;

	n = v4l_cam_mem_by_addr(cam, opaque);
	if (n >= 0)
		/* TODO: check error? */
		v4l_cam_buf_queue(cam, cam->buf_record_active, n);

	LOGV("%s(%p): %d", __func__, opaque, n);
}

/*
 * Start auto focus, the notification callback routine is called with
 * CAMERA_MSG_FOCUS once when focusing is complete. autoFocus() will be
 * called again if another auto focus is needed.
 */
static int v4l_cam_auto_focus(struct camera_device *dev)
{
	CAM_LOG("%s()", __func__);
	return 0;
}

/*
 * Cancels auto-focus function. If the auto-focus is still in progress,
 * this function will cancel it. Whether the auto-focus is in progress or
 * not, this function will return the focus position to the default.  If
 * the camera does not support auto-focus, this is a no-op.
 */
static int v4l_cam_cancel_auto_focus(struct camera_device *dev)
{
	CAM_LOG("%s()", __func__);
	return 0;
}

/*
 * Take a picture - can wait for preview stop, but is not blocking, waiting
 * for a still image. A thread is envoked instead, that will call a callback.
 * We assume, it is only called during a running preview.
 */
static int v4l_cam_take_picture(struct camera_device *dev)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);
	uint8_t z = 0;

	CAM_LOG("%s()", __func__);

	if (!(cam->msg_mask & CAMERA_MSG_RAW_IMAGE))
		return -EINVAL;

	if (!cam->preview) {
		LOGE("%s(): preview not running!\n", __func__);
		return -EINVAL;
	}

	cam->picture = true;
	write(cam->preview_pipe[0], &z, 1);

	return 0;
}

/*
 * Cancel a picture that was started with takePicture. Calling this method
 * when no picture is being taken is a no-op.
 */
static int v4l_cam_cancel_picture(struct camera_device *dev)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);

	CAM_LOG("%s()", __func__);

	cam->picture = false;

	return 0;
}

/*
 * Return the (soft) camera parameters. As mentioned above, these have not been
 * sent to the driver yet.
 */
static char *v4l_cam_get_parameters(struct camera_device *dev)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);

	CAM_LOG("%s()", __func__);

	return camera_param_get(&cam->acp);
}

static void v4l_cam_put_parameters(struct camera_device *dev, char *parms)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);
	CAM_LOG("%s()", __func__);
	camera_param_put(&cam->acp, parms);
}

/*
 * Send one of CAMERA_CMD_* commands from system/core/include/system/camera.h
 * to camera driver, e.g., CAMERA_CMD_START_SMOOTH_ZOOM.
 */
static int v4l_cam_send_command(struct camera_device *dev,
                    int32_t cmd, int32_t arg1, int32_t arg2)
{
	CAM_LOG("%s()", __func__);
	return BAD_VALUE;
}

/* Release the hardware resources owned by this object. */
static void v4l_cam_release(struct camera_device *dev)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);
	void *resource;

	CAM_LOG("%s()", __func__);

	cam->record = false;
	cam->picture = false;
	cam->preview = false;
	cam->configured = false;
	cam->exit = true;
	v4l_cam_streaming_stop(cam, &cam->buf_preview);
	pthread_cond_signal(&cam->preview_cond);
	pthread_join(cam->picture_prepare, &resource);
	pthread_join(cam->thread_preview, &resource);
	v4l_cam_destroy_buffers(cam);
	cam->preview_width = cam->preview_height = 0;
	cam->picture_compressed_4cc = 0;
	cam->acp.preview_fmt.fourcc = 0;

	/* FIXME: move to .close() */
	close(cam->fd);
}

/* Dump state of the camera hardware */
static int v4l_cam_dump(struct camera_device *dev, int fd)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam);

	char buf[80] = {};
	CAM_LOG("%s()", __func__);
	snprintf(buf, sizeof(buf), "Configured: %s, preview: %s, record: %s, picture: %s\n",
		 cam->configured ? "yes" : "no", cam->preview ? "on" : "off",
		 cam->record ? "on" : "off", cam->picture ? "on" : "off");
	write(fd, buf, strlen(buf));
	return NO_ERROR;
}

static camera_device_ops_t v4l_camera_device_ops = {
	.set_preview_window		= v4l_cam_set_preview_window,
	.set_callbacks			= v4l_cam_set_callbacks,
	.enable_msg_type		= v4l_cam_enable_msg_type,
	.disable_msg_type		= v4l_cam_disable_msg_type,
	.msg_type_enabled		= v4l_cam_msg_type_enabled,
	.start_preview			= v4l_cam_start_preview,
	.stop_preview			= v4l_cam_stop_preview,
	.preview_enabled		= v4l_cam_preview_enabled,
	.store_meta_data_in_buffers	= v4l_cam_store_meta_data_in_buffers,
	.start_recording		= v4l_cam_start_recording,
	.stop_recording			= v4l_cam_stop_recording,
	.recording_enabled		= v4l_cam_recording_enabled,
	.release_recording_frame	= v4l_cam_release_recording_frame,
	.auto_focus			= v4l_cam_auto_focus,
	.cancel_auto_focus		= v4l_cam_cancel_auto_focus,
	.take_picture			= v4l_cam_take_picture,
	.cancel_picture			= v4l_cam_cancel_picture,
	.set_parameters			= v4l_cam_set_parameters,
	.get_parameters			= v4l_cam_get_parameters,
	.put_parameters			= v4l_cam_put_parameters,
	.send_command			= v4l_cam_send_command,
	.release			= v4l_cam_release,
	.dump				= v4l_cam_dump,
};

static int v4l_cam_close(struct hw_device_t* dev)
{
	struct v4l_cam *cam = container_of(dev, struct v4l_cam, acam.common);

	LOGI("%s(%p)", __func__, dev);
	camera_param_free(&cam->acp);
	if (dev)
		dev->module = NULL;
	return 0;
}

static struct v4l_cam v4l_cam_dev = {
	.acam = {
		.common = {
			.tag		= HARDWARE_DEVICE_TAG,
			.version	= 1,
			.close		= v4l_cam_close,
		},
	},
};

static int v4l_cam_get_number_of_cameras(void)
{
	CAM_LOG("%s()", __func__);
	return ARRAY_SIZE(cam_info_set);
}

static const char *v4l_cam_node[] = {
	"/dev/video0",
	"/dev/video1",
};

static int v4l_cam_open(const struct hw_module_t *module,
			const char *id,
			struct hw_device_t **device)
{
	int cam_id = atoi(id);
	struct v4l2_capability cap;
	int ret;
	uid_t uid = geteuid(), gid = getegid();

	CAM_LOG("%s(): running as %u.%u", __func__, uid, gid);

	if (cam_id < 0 || cam_id >= v4l_cam_get_number_of_cameras()) {
		LOGE("Invalid camera ID %s", id);
		return -EINVAL;
	}

	if (v4l_cam_dev.acam.common.module) {
		if (v4l_cam_dev.id == cam_id) {
			CAM_LOG("returning existing camera ID %s", id);
			goto done;
		} else {
			LOGE("Cannot open camera %d. camera %d is already running!",
			     cam_id, v4l_cam_dev.id);
			return -ENOSYS;
		}
	}

	v4l_cam_dev.fd = open(v4l_cam_node[cam_id], O_RDWR);
	if (v4l_cam_dev.fd < 0) {
		LOGE("open(%s): errno = %d", v4l_cam_node[cam_id], errno);
		return -errno;
	}

	ret = ioctl(v4l_cam_dev.fd, VIDIOC_QUERYCAP, &cap);
	if (ret < 0 || !(cap.capabilities & V4L2_CAP_STREAMING)) {
		LOGE("QUERYCAP(%s): errno = %d, caps=%x", v4l_cam_node[cam_id], errno, cap.capabilities);
		goto ecap;
	}

	v4l_cam_dev.acam.common.module = (struct hw_module_t *)module;
	v4l_cam_dev.acam.ops = &v4l_camera_device_ops;
	v4l_cam_dev.id = cam_id;

	/*
	 * This initialisation shouldn't be needed, .set_parameters will do it,
	 * just have to check, that at least one of CAPTURE or CAPTURE_MPLANE is
	 * supported
	 */
	if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
		v4l_cam_dev.buf_preview.fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	else if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
		v4l_cam_dev.buf_preview.fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	else {
		LOGE("No CAPTURE cap in %x", cap.capabilities);
		goto ecap;
	}

	v4l_cam_dev.cap = cap.capabilities & (V4L2_CAP_VIDEO_CAPTURE |
					      V4L2_CAP_VIDEO_CAPTURE_MPLANE);
	v4l_cam_dev.msg_mask = MSG_MASK;

	/*
	 * The driver might update the buffer type later in response to our
	 * format querying - this shouldn't be needed either, see above
	 */
	v4l_cam_dev.buf_picture.fmt.type = v4l_cam_dev.buf_preview.fmt.type;
	v4l_cam_dev.buf_record.fmt.type = v4l_cam_dev.buf_preview.fmt.type;

	/*
	 * How a decision shall be made whether to use MMAP or USERPTR? Switch
	 * to the latter, when buffers are shared with the framebuffer.
	 */
	v4l_cam_dev.buf_preview.memory = V4L2_MEMORY_MMAP;
	v4l_cam_dev.buf_picture.memory = V4L2_MEMORY_MMAP;
	v4l_cam_dev.buf_record.memory = V4L2_MEMORY_MMAP;

	camera_param_init(&v4l_cam_dev.acp);
	ret = v4l_cam_default_fmts(&v4l_cam_dev);
	if (ret < 0)
		goto efmt;
	camera_param_set(&v4l_cam_dev.acp);

	v4l_cam_dev.exit = false;
	pthread_cond_init(&v4l_cam_dev.preview_cond, NULL);
	pthread_cond_init(&v4l_cam_dev.stop_cond, NULL);
	/* See comment in v4l_cam_stop_preview() */
	ret = pipe(v4l_cam_dev.preview_pipe);
	if (ret < 0)
		goto epipe;
	ret = pthread_create(&v4l_cam_dev.thread_preview, NULL,
			     v4l_cam_thread_preview, &v4l_cam_dev);
	if (ret < 0) {
		LOGE("pthread_create() = %d, errno = %d", ret, errno);
		goto ecrtpvu;
	}
	close(v4l_cam_dev.preview_pipe[0]);

done:
	*device = &v4l_cam_dev.acam.common;

	LOGI("%s: opened camera #%s (%p), type %u", __func__, id, *device, v4l_cam_dev.buf_preview.fmt.type);

	return 0;

ecrtpvu:
	close(v4l_cam_dev.preview_pipe[0]);
	close(v4l_cam_dev.preview_pipe[1]);
epipe:
	camera_param_free(&v4l_cam_dev.acp);
efmt:
ecap:
	close(v4l_cam_dev.fd);

	return -errno;
}

static hw_module_methods_t v4l_cam_ops = {
	.open = v4l_cam_open,
};

static int v4l_cam_get_camera_info(int cam_id, struct camera_info *cam_info)
{
	CAM_LOG("%s()", __func__);
	if (cam_id >= v4l_cam_get_number_of_cameras())
		return -EINVAL;
	memcpy(cam_info, &cam_info_set[cam_id], sizeof(*cam_info));
	return 0;
}

struct camera_module HAL_MODULE_INFO_SYM = {
	.common = {
		.tag		= HARDWARE_MODULE_TAG,
		.version_major	= 1,
		.version_minor	= 0,
		.id		= CAMERA_HARDWARE_MODULE_ID,
		.name		= "Generic V4L2 camera HAL",
		.author		= "Guennadi Liakhovetski",
		.methods	= &v4l_cam_ops,
	},
	.get_number_of_cameras	= v4l_cam_get_number_of_cameras,
	.get_camera_info	= v4l_cam_get_camera_info,
};
