#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	// Load Body Tracking Settings
	auto bodyTrackingSettings = ofxAzureKinect::BodyTrackingSettings();
	bodyTrackingSettings.updateBodies = true;

	// Load a recording with Body Tracking Settings
	if (!streaming)
	{
		filename = ofToDataPath("output_2d_movements.mkv");
		if (!kinectDevice.load(filename))
		{
			exit();
		}
		// Start Playback or Streaming
		kinectDevice.startCameras();
	}
	else
	{
		auto deviceSettings = ofxAzureKinect::DeviceSettings();
		deviceSettings.syncImages = false;
		deviceSettings.depthMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		deviceSettings.updateIr = false;
		deviceSettings.updateColor = false;
		//deviceSettings.colorResolution = K4A_COLOR_RESOLUTION_1080P;
		deviceSettings.updateWorld = true;
		deviceSettings.updateVbo = false;

		if (!kinectDevice.open())
		{
			exit();
		}
		// Start Playback or Streaming
		kinectDevice.startCameras(deviceSettings, bodyTrackingSettings);
	}
}

//--------------------------------------------------------------
void ofApp::exit()
{
	this->kinectDevice.close();
}

//--------------------------------------------------------------
void ofApp::update()
{
}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(0);

	ofxAzureKinect::BodyTracker *tracker = kinectDevice.getBodyTracker();

	tracker->draw_body_map();

	camera.begin();

	ofPushMatrix();
	ofRotateXDeg(180);
	tracker->draw_point_clouds(kinectDevice.getDepthTex(), kinectDevice.getDepthToWorldTex());
	tracker->draw_skeletons();
	ofPopMatrix();

	camera.end();

	stringstream ss;
	ss << ofToString(ofGetFrameRate(), 2) + " FPS" << std::endl;
	ss << "Joint Smoothing: " << tracker->get_joint_smoothing();
	ofDrawBitmapStringHighlight(ss.str(), 10, 20);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
	// handle playback hotkeys
	if (!streaming)
	{
		switch (key)
		{
		case ' ':
		{
			play = !play;
			if (play)
			{
				kinectDevice.play = true;
			}
			else
			{
				kinectDevice.pause = false;
			}
			break;
		}
		case '+':
			play_head = ofClamp(play_head + .01, 0, 1);
			kinectDevice.seek = play_head;
			break;
		case '-':
			play_head = ofClamp(play_head - .01, 0, 1);
			kinectDevice.seek = play_head;
			break;
		}

		switch (key)
		{
		case 'f':
		case 'F':
			ofToggleFullscreen();
			break;
		default:
			break;
		}
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
	if (button == 1)
	{
		this->kinectDevice.getBodyTracker()->set_joint_smoothing(ofMap(x, 0, ofGetWidth(), 0.0f, 1.0f, true));
	}
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg)
{
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)
{
}
