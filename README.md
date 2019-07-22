# ofxAzureKinect

ofxAddon that allows you to use [Azure Kinect](https://azure.microsoft.com/en-us/services/kinect-dk/) in [openFrameworks](https://github.com/openframeworks/openFrameworks).

* Get depth, color, depth to world, and color in depth frames as `ofPixels` or `ofTexture`.
* Get point cloud VBO with texture coordinates in depth space.
* More coming soon... (undistort that crazy fisheye frame, read IMU values, sync between multi-devices, etc.)

## Installation

* Install the [Azure Kinect Sensor SDK](https://docs.microsoft.com/en-us/azure/Kinect-dk/sensor-sdk-download).

* Add an environment variable for `AZUREKINECT_SDK` and set it to the installation path. The default on Windows is `C:\Program Files\Azure Kinect SDK v1.1.0\sdk\`.

    ![Environment Variables](Install-EnvVars.png)

* Add the path to the SDK `bin` folder to the `PATH` variable. The default on Windows is `%AZUREKINECT_SDK%\windows-desktop\amd64\release\bin`.

    ![Path](Install-Path.png)


* Clone this repository in your openFrameworks `addons` folder.

* You can then use the OF Project Generator to generate projects with the appropriate headers and libraries included. ✌️

## Compatibility

* Tested on openFrameworks 0.10.x / Windows 10 / Visual Studio 2017.

## Examples

* `example-streams` demonstrates how to get depth, color, infrared textures from the device.
* `example-pointCloud` demonstrates how to draw the basic point cloud VBO from the device.
* `example-shader` demonstrates how to reconstruct a point cloud using LUTs in a shader.
