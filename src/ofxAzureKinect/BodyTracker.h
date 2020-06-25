#pragma once

#include <k4a/k4a.hpp>
#include <k4abt.h>
#include "ofMain.h"

namespace ofxAzureKinect
{
	class BodyTracker
	{

	public:
		BodyTracker();
		BodyTracker(k4a_calibration_t calibration, k4abt_tracker_configuration_t config);
		~BodyTracker(){};

		void update(k4a_capture_t capture);
		void update_texture();	// why a seg fault when called in update??

		// std::vector<k4abt_skeleton_t> get_skeletons();
		// k4abt_skeleton_t get_skeletons(int id);


		const ofPixels &getBodyIndexPix() const;
		const ofTexture &getBodyIndexTex() const;

		size_t getNumBodies() const;
		const std::vector<k4abt_skeleton_t> &getBodySkeletons() const;
		const std::vector<uint32_t> &getBodyIDs() const;

	private:
		k4abt_tracker_t tracker;

		size_t num_bodies = 0;

		bool active = false;

		std::vector<k4abt_skeleton_t> bodySkeletons;
		std::vector<uint32_t> bodyIDs;

		// Visualization
		ofPixels bodyIndexPix;
		ofTexture bodyIndexTex;
		void draw_skeleton();
		void draw_pointcloud();
		void draw_mask();

		void load_shader();
		ofShader shader;

		ofVbo pointsVbo;
	};
} // namespace ofxAzureKinect