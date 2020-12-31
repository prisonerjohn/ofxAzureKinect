#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	if (kinectDevice.open())
	{
		auto deviceSettings = ofxAzureKinect::DeviceSettings();
		deviceSettings.syncImages = false;
		deviceSettings.depthMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		deviceSettings.updateIr = false;
		deviceSettings.updateColor = true;
		deviceSettings.colorResolution = K4A_COLOR_RESOLUTION_720P;
		deviceSettings.updateWorld = true;
		deviceSettings.updateVbo = false;
		kinectDevice.startCameras(deviceSettings);

		auto bodyTrackerSettings = ofxAzureKinect::BodyTrackerSettings();
		bodyTrackerSettings.sensorOrientation = K4ABT_SENSOR_ORIENTATION_DEFAULT;
		//bodyTrackerSettings.processingMode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
		bodyTrackerSettings.imageType = K4A_CALIBRATION_TYPE_COLOR;
		bodyTrackerSettings.updateBodiesImage = true;
		kinectDevice.startBodyTracker(bodyTrackerSettings);
	}
}

//--------------------------------------------------------------
void ofApp::exit()
{
	kinectDevice.close();
}

//--------------------------------------------------------------
void ofApp::update()
{

}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(0);

	if (kinectDevice.isStreaming())
	{
		// Scale down the 2D drawings.
		static const float kTargetWidth = ofGetWidth();
		const auto texSize = glm::vec2(kinectDevice.getBodyIndexTex().getWidth(), kinectDevice.getBodyIndexTex().getHeight());
		const auto texScale = kTargetWidth / texSize.x;

		// Draw the body index texture. 
		// The pixels are not black, their color equals the body ID which is just a low number.
		kinectDevice.getBodyIndexTex().draw(0, 0, texSize.x * texScale, texSize.y * texScale);

		// Draw the projected joints onto the image.
		const auto skeletons = kinectDevice.getBodySkeletons();
		for (int i = 0; i < skeletons.size(); ++i)
		{
			for (int j = 0; j < K4ABT_JOINT_COUNT; ++j)
			{
				switch (skeletons[i].joints[j].confidenceLevel)
				{
				case K4ABT_JOINT_CONFIDENCE_MEDIUM:
					ofSetColor(ofColor::green);
					break;
				case K4ABT_JOINT_CONFIDENCE_LOW:
					ofSetColor(ofColor::yellow);
					break;
				case K4ABT_JOINT_CONFIDENCE_NONE:
				default:
					ofSetColor(ofColor::red);
					break;
				}
				ofDrawCircle(skeletons[i].joints[j].projPos * texScale, 5.0f);
			}
		}
		ofSetColor(ofColor::white);
	}

	std::ostringstream oss;
	oss << ofToString(ofGetFrameRate(), 2) + " FPS";
	ofDrawBitmapStringHighlight(oss.str(), 10, 20);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
	
}
