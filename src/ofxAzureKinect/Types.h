#pragma once

#include <k4a/k4atypes.h>
#include <k4abttypes.h>
#include "ofVectorMath.h"

inline const glm::vec2 & toGlm(const k4a_float2_t & v) 
{
	return *reinterpret_cast<const glm::vec2*>(&v);
}

inline const glm::vec3 & toGlm(const k4a_float3_t & v)
{
	return *reinterpret_cast<const glm::vec3*>(&v);
}

inline const glm::quat toGlm(const k4a_quaternion_t & q)
{
	return glm::quat(q.v[0], q.v[1], q.v[2], q.v[3]);
}

inline const k4a_float2_t& toK4A(const glm::vec2& v)
{
	return *reinterpret_cast<const k4a_float2_t*>(&v);
}

inline const k4a_float3_t& toK4A(const glm::vec3& v)
{
	return *reinterpret_cast<const k4a_float3_t*>(&v);
}

inline const k4a_quaternion_t toK4A(const glm::quat& q)
{
	k4a_quaternion_t quat;
	quat.v[0] = q[3];
	quat.v[1] = q[0];
	quat.v[2] = q[1];
	quat.v[3] = q[2];
	return quat;
}

namespace ofxAzureKinect
{
	typedef k4a_depth_mode_t DepthMode;
	typedef k4a_color_resolution_t ColorResolution;
	typedef k4a_image_format_t ImageFormat;
	typedef k4a_fps_t FramesPerSecond;
	typedef k4a_wired_sync_mode_t WiredSyncMode;

	typedef k4abt_sensor_orientation_t SensorOrientation;
	typedef k4abt_tracker_processing_mode_t ProcessingMode;
	typedef k4abt_joint_confidence_level_t ConfidenceLevel;
}
