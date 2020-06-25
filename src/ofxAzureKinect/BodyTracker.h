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

		vector<k4abt_skeleton_t> get_skeletons();
		k4abt_skeleton_t get_skeleton(int id);

		vector<k4abt_body_t> get_bodies();
		k4abt_body_t get_body(int id);
		
		bool is_active() { return active; }

		void draw_body_map(int x=0, int y=0, int w=360, int h=360);
		void draw_point_clouds(ofTexture depth_texture, ofTexture depth_to_world_texture);
		void draw_skeletons();
		void draw_skeleton(int id);

		

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

		vector<ofVboMesh *> skeletonMeshes;

		// Visualization
		ofPixels bodyIndexPix;
		ofTexture bodyIndexTex;
		void draw_skeleton();
		void draw_pointcloud();
		void draw_mask(); // <-- should have access to segmented body image?

		void load_shaders();
		ofShader shader;

		ofVbo pointsVbo;
	};
} // namespace ofxAzureKinect