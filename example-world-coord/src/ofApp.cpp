#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	if (kinectDevice.open())
	{
		auto kinectSettings = ofxAzureKinect::DeviceSettings();
		kinectDevice.startCameras(kinectSettings);
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
	if (kinectDevice.isFrameNew())
	{
		kinectFps.newFrame();
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(128);

	if (kinectDevice.isStreaming() && kinectDevice.getDepthTex().isAllocated())
	{
		kinectDevice.getDepthTex().draw(0, 0);
		kinectDevice.getColorInDepthTex().draw(0, 0);

		glm::vec3 worldCoord = getWorldCoordinate(mouseX, mouseY);
		ofDrawBitmapStringHighlight(ofToString(worldCoord), ofGetMouseX() + 16, ofGetMouseY() + 10);
	}

	std::ostringstream oss;
	oss << std::fixed << std::setprecision(2)
		<< "APP: " << ofGetFrameRate() << " FPS" << std::endl
		<< "K4A: " << kinectFps.getFps() << " FPS";
	ofDrawBitmapStringHighlight(oss.str(), 10, 20);
}

//--------------------------------------------------------------
glm::vec3 ofApp::getWorldCoordinate(int x, int y)
{
	const auto& depthPixels = kinectDevice.getDepthPix();
	const auto depthData = depthPixels.getData();
	const auto& depthToWorldPixels = kinectDevice.getDepthToWorldPix();
	const auto depthToWorldData = depthToWorldPixels.getData();

	int sampleX = ofClamp(x, 0, depthPixels.getWidth() - 1);
	int sampleY = ofClamp(y, 0, depthPixels.getHeight() - 1);
	int idx = sampleY * depthPixels.getWidth() + sampleX;

	if (depthData[idx] != 0 &&
		depthToWorldData[idx * 2 + 0] != 0 && depthToWorldData[idx * 2 + 1] != 0)
	{
		float depthVal = static_cast<float>(depthData[idx]);
		return glm::vec3(
			depthToWorldData[idx * 2 + 0] * depthVal,
			depthToWorldData[idx * 2 + 1] * depthVal,
			depthVal
		);
	}
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
