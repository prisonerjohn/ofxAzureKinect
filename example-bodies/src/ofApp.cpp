#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	auto kinectSettings = ofxAzureKinect::DeviceSettings();
	kinectSettings.synchronized = false;
	kinectSettings.updateIr = false;
	kinectSettings.updateColor = false;
	//kinectSettings.colorResolution = K4A_COLOR_RESOLUTION_1080P;
	kinectSettings.updateBodies = true;
	kinectSettings.updateWorld = false;
	if (this->kinectDevice.open(kinectSettings))
	{
		this->kinectDevice.startCameras();
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
	ofBackground(128);

	if (this->kinectDevice.isStreaming())
	{
		this->kinectDevice.getBodyIndexTex().draw(0, 0, 360, 360);
	}

	if (this->kinectDevice.getNumBodies() > 0)
	{
		ofNoFill();
		this->camera.begin();
		{
			auto& bodySkeletons = this->kinectDevice.getBodySkeletons();
			for (auto& pair : bodySkeletons)
			{
				auto skeleton = pair.second;
				for (int i = 0; i < K4ABT_JOINT_COUNT; ++i)
				{
					auto joint = skeleton.joints[i];
					ofDrawSphere(joint.position.v[0], joint.position.v[1], joint.position.v[2], 50.0f);
				}
			}
		}
		this->camera.end();
		ofFill();
	}

	ofDrawBitmapString(ofToString(ofGetFrameRate(), 2) + " FPS", 10, 20);
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
