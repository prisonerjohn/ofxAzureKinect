#include "ofApp.h"

int main()
{
	ofGLFWWindowSettings settings;
	settings.setGLVersion(3, 2);
	settings.setSize(1280 + 360, 720);
	ofCreateWindow(settings);

	ofRunApp(new ofApp());
}
