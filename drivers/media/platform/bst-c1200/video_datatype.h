/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef VIDEO_DATATYPE_H
#define VIDEO_DATATYPE_H

#include <bst/ipc_app_common.h>

#ifdef __cplusplus
extern "C" {
#endif

// user defined types
enum _video_error_e_t {
	VIDEO_NO_ERROR = 0,
	VIDEO_SERVER_FAIL = -1
};

#define video_error_e_t enum _video_error_e_t

// constants

// type serialize / deserialize functions
/**
 * Serialize video_error_e_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_video_error_e(serdes_t *ser,
					      const video_error_e_t *in)
{
	int32_t ret = 0;

	ret = ipc_ser_put_32(ser, (uint32_t *)in);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize video_error_e_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_video_error_e(des_buf_t *buf,
						video_error_e_t *out)
{
	video_error_e_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (video_error_e_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = *ptr;
	return 0;
}

#ifdef __cplusplus
}
#endif

#endif
