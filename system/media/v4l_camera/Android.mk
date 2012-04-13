LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := camera.v4l
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

LOCAL_SRC_FILES := v4l_camera.c camera_param.cpp camera_preview.cpp \
	../../../development/tools/emulator/system/camera/Converters.cpp \
	../../../development/tools/emulator/system/camera/JpegCompressor.cpp \
	../../../frameworks/base/core/jni/android/graphics/YuvToJpegEncoder.cpp

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/ \
	frameworks/base/include \
	frameworks/base/services/camera/libcameraservice \
	development/tools/emulator/system/camera/ \
	frameworks/base/core/jni/android/graphics \
	external/skia/include/core \
	external/skia/include/images \
	external/jpeg \
	hardware/libhardware/include/hardware

LOCAL_SHARED_LIBRARIES:= \
	libui \
	libbinder \
	libutils \
	libcutils \
	libjpeg \
	libskia \
	libandroid_runtime \
	libcamera_client

LOCAL_CFLAGS := -Wunused-variable

include $(BUILD_SHARED_LIBRARY)
