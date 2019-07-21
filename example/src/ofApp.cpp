#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	this->kinectDevice.open();
	this->kinectDevice.startCameras();
}

//--------------------------------------------------------------
void ofApp::exit()
{
	ofLogNotice(__FUNCTION__) << "Found " << this->kinectDevice.getInstalledCount() << " installed devices.";

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
		//this->kinectDevice.getColorTex().draw(0, 0, ofGetWidth(), ofGetHeight());

		this->cam.begin();
		{
			ofDrawAxis(100.0f);

			this->kinectDevice.getColorInDepthTex().bind();
			this->kinectDevice.getPointCloudVbo().draw(
				GL_POINTS, 
				0, this->kinectDevice.getPointCloudVbo().getNumVertices());
			this->kinectDevice.getColorInDepthTex().unbind();
		}
		this->cam.end();
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
