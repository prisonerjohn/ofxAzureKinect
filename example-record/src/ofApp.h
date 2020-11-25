#pragma once

#include "ofMain.h"
#include "ofxAzureKinect.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void exit();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		ofxAzureKinect::Device sensor;
		ofFpsCounter fpsCounter;

		void draw_recording_animation();

		float recording_delay=0;
		float recording_start=0;
		float recording_duration=0;

};
