#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	// Open Kinect.
	if (kinectDevice.open())
	{
		auto kinectSettings = ofxAzureKinect::DeviceSettings();
		kinectSettings.updateIr = false;
		kinectSettings.updateColor = true;
		kinectSettings.colorResolution = K4A_COLOR_RESOLUTION_1080P;
		kinectSettings.updateVbo = false;
		kinectDevice.startCameras(kinectSettings);
	}

	// Load shader.
	auto shaderSettings = ofShaderSettings();
	shaderSettings.shaderFiles[GL_VERTEX_SHADER] = "shaders/render.vert";
	shaderSettings.shaderFiles[GL_GEOMETRY_SHADER] = "shaders/render.geom";
	shaderSettings.shaderFiles[GL_FRAGMENT_SHADER] = "shaders/render.frag";
	shaderSettings.bindDefaults = true;
	if (shader.setup(shaderSettings))
	{
		ofLogNotice(__FUNCTION__) << "Success loading shader!";
	}

	// Setup vbo.
	std::vector<glm::vec3> verts(1);
	vbo.setVertexData(verts.data(), verts.size(), GL_STATIC_DRAW);

	pointSize = 3.0f;
	useColorSpace = false;
}

//--------------------------------------------------------------
void ofApp::exit()
{
	kinectDevice.close();
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(0);

	if (kinectDevice.isStreaming())
	{
		cam.begin();
		{
			ofEnableDepthTest();

			ofDrawAxis(100.0f);

			ofPushMatrix();
			{
				ofRotateXDeg(180);

				shader.begin();
				{
					shader.setUniform1f("uSpriteSize", pointSize);

					int numPoints;
					
					if (useColorSpace)
					{
						shader.setUniformTexture("uDepthTex", kinectDevice.getDepthInColorTex(), 1);
						shader.setUniformTexture("uWorldTex", kinectDevice.getColorToWorldTex(), 2);
						shader.setUniformTexture("uColorTex", kinectDevice.getColorTex(), 3);
						shader.setUniform2i("uFrameSize", kinectDevice.getColorTex().getWidth(), kinectDevice.getColorTex().getHeight());
					
						numPoints = kinectDevice.getColorTex().getWidth() * kinectDevice.getColorTex().getHeight();
					}
					else
					{
						shader.setUniformTexture("uDepthTex", kinectDevice.getDepthTex(), 1);
						shader.setUniformTexture("uWorldTex", kinectDevice.getDepthToWorldTex(), 2);
						shader.setUniformTexture("uColorTex", kinectDevice.getColorInDepthTex(), 3);
						shader.setUniform2i("uFrameSize", kinectDevice.getDepthTex().getWidth(), kinectDevice.getDepthTex().getHeight());
					
						numPoints = kinectDevice.getDepthTex().getWidth() * kinectDevice.getDepthTex().getHeight();
					}

					vbo.drawInstanced(GL_POINTS, 0, 1, numPoints);
				}
				shader.end();
			}
			ofPopMatrix();
		}
		cam.end();
	}

	ofDrawBitmapStringHighlight(ofToString(ofGetFrameRate(), 2) + " FPS", 10, 20);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
	if (key == OF_KEY_UP)
	{
		pointSize *= 2;
	}
	else if (key == OF_KEY_DOWN)
	{
		pointSize /= 2;
	}
	else if (key == ' ')
	{
		useColorSpace ^= 1;
	}
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
