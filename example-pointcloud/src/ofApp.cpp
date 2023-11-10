#include "ofApp.h"

// Uncomment this line to force the VBO to use the depth image size (max num points 512x512).
// By default, it will use the color image size if available (max num points 1920x1080).
//#define FORCE_VBO_DEPTH_SIZE 1

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	if (kinectDevice.open())
	{
		auto kinectSettings = ofxAzureKinect::DeviceSettings();
		kinectSettings.updateIr = false;
		kinectSettings.updateColor = true;
		kinectSettings.colorResolution = K4A_COLOR_RESOLUTION_1080P;
		kinectSettings.updateVbo = true;
#if FORCE_VBO_DEPTH_SIZE
		kinectSettings.forceVboToDepthSize = true;
#endif
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

}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(128);

	if (kinectDevice.isStreaming())
	{
		cam.begin();
		ofEnableDepthTest();
		{
			ofDrawAxis(1.0f);

			ofPushMatrix();
			{ 
				ofRotateXDeg(180);

#if FORCE_VBO_DEPTH_SIZE
				const auto& colorTex = kinectDevice.getColorInDepthTex();
#else
				const auto& colorTex = kinectDevice.getColorTex();
#endif
				if (colorTex.isAllocated())
				{
					colorTex.bind();
				}
				kinectDevice.getPointCloudVbo().draw(
					GL_POINTS,
					0, kinectDevice.getPointCloudVbo().getNumVertices());
				if (colorTex.isAllocated())
				{
					colorTex.unbind();
				}
			}
			ofPopMatrix();
		}
		ofDisableDepthTest();
		cam.end();
	}

	std::ostringstream oss;
	oss << ofToString(ofGetFrameRate(), 2) << " FPS" << std::endl
		<< kinectDevice.getPointCloudVbo().getNumVertices() << " Points";
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
