#pragma once

#include "ofMain.h"

#include "ofxAzureKinect.h"

class ofApp 
	: public ofBaseApp 
{
public:
	void setup();
	void exit();

	void update();
	void draw();

	void openDevice();
	void closeDevice();

	void openPlayback();
	void closePlayback();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

private:
	ofxAzureKinect::Device kinectDevice;
	ofFpsCounter fpsDevice;

	ofxAzureKinect::Playback kinectPlayback;
	ofFpsCounter fpsPlayback;

	bool bRecord;
	bool bPlayback;
	std::string filename;
};
