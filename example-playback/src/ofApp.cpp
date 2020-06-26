#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
    // place a recording from k4arecorder or example-record in the data folder
    filename = ofToDataPath("output.mkv");

    device.open(filename);
    device.startCameras();
}

//--------------------------------------------------------------
void ofApp::update()
{
}

//--------------------------------------------------------------
void ofApp::draw()
{
    ofBackground(128);

    // visualize kinect streams
    if (this->device.isStreaming())
    {
        this->device.getColorTex().draw(0, 0, 1280, 720);
        this->device.getDepthTex().draw(1280, 0, 360, 360);
        this->device.getIrTex().draw(1280, 360, 360, 360);
    }

    ofDrawBitmapStringHighlight(ofToString(ofGetFrameRate(), 2) + " FPS", 10, 20);

    stringstream ss;
    ss << "Press SPACEBAR to play / pause the recording." << endl;
    ss << "Press + / - to seek when the recording is paused." << endl;
    ss << "Press 'f' to toggle fullscreen.";
    ofDrawBitmapStringHighlight(ss.str(), 10, ofGetHeight() - 50);
}

//--------------------------------------------------------------
void ofApp::exit()
{
    device.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    switch (key)
    {
    case ' ':
    {
        play = !play;
        if (play)
        {
            device.play = true;
        }
        else
        {
            device.pause = false;
        }
        break;
    }
    case '+':
        play_head = ofClamp(play_head + .01, 0, 1);
        device.seek = play_head;
        break;
    case '-':
        play_head = ofClamp(play_head - .01, 0, 1);
        device.seek = play_head;
        break;
    case 'f':
    case 'F':
        ofToggleFullscreen();
        break;
    default:
        break;
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
