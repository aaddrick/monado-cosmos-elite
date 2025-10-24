// Copyright 2025, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Android face tracking related API entrypoint functions.
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @ingroup oxr_main
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "oxr_objects.h"
#include "oxr_logger.h"
#include "oxr_handle.h"

static XrResult
oxr_face_tracker_android_destroy_cb(struct oxr_logger *log, struct oxr_handle_base *hb)
{
	struct oxr_face_tracker_android *face_tracker_android = (struct oxr_face_tracker_android *)hb;
	free(face_tracker_android);
	return XR_SUCCESS;
}

XrResult
oxr_face_tracker_android_create(struct oxr_logger *log,
                                struct oxr_session *sess,
                                const XrFaceTrackerCreateInfoANDROID *createInfo,
                                XrFaceTrackerANDROID *faceTracker)
{
	return oxr_session_success_result(sess);
}

XrResult
oxr_get_face_state_android(struct oxr_logger *log,
                           struct oxr_face_tracker_android *facial_tracker_android,
                           const XrFaceStateGetInfoANDROID *getInfo,
                           XrFaceStateANDROID *faceStateOutput)
{

	return oxr_session_success_result(facial_tracker_android->sess);
}

XrResult
oxr_get_face_calibration_state_android(struct oxr_logger *log,
                                       struct oxr_face_tracker_android *facial_tracker_android,
                                       XrBool32 *faceIsCalibratedOutput)
{
	return oxr_session_success_result(facial_tracker_android->sess);
}
