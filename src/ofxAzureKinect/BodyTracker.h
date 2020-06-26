#pragma once

#include <k4a/k4a.hpp>
#include <k4abt.h>

#include "ofMain.h"

#include "Types.h"

namespace ofxAzureKinect
{
	class BodyTracker
	{

	public:
		BodyTracker(){};
		BodyTracker(k4a_calibration_t calibration, k4abt_tracker_configuration_t config);
		~BodyTracker(){};

		void update(k4a_capture_t capture);
		void update_texture(); // why do I get a seg fault when called in update??

		const vector<k4abt_skeleton_t> &get_skeletons() const;
		const k4abt_skeleton_t &get_skeleton(int id) const;
		const vector<uint32_t> &get_body_ids() const;

		size_t get_num_bodies() const;

		const ofPixels &get_body_index_map_pix() const;
		const ofTexture &get_body_index_map_tex() const;

		void set_joint_smoothing(float val);
		float get_joint_smoothing();

		bool is_active() { return active; }

		void draw_body_map(int x = 0, int y = 0, int w = 360, int h = 360);
		void draw_point_clouds(ofTexture depth_texture, ofTexture depth_to_world_texture);
		void draw_skeletons();
		void draw_skeleton(int id);

	private:
		k4abt_tracker_t tracker;

		size_t num_bodies = 0;

		bool active = false;

		std::vector<k4abt_skeleton_t> skeletons;
		std::vector<uint32_t> bodyIDs;

		// Visualization
		ofPixels bodyIndexPix;
		ofTexture bodyIndexTex;

		void load_shaders();
		ofShader shader;
		vector<ofVboMesh *> skeleton_meshes;
		ofVbo pointsVbo;

		ofParameter<float> joint_smoothing{"Joint Smoothing", 0.0f, 0.0f, 1.0f};
	};
} // namespace ofxAzureKinect