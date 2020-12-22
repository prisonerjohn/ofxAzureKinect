#pragma once

#include <k4arecord/record.hpp>

#include "ofParameter.h"

namespace ofxAzureKinect
{
	class Recorder
	{
	public:
		Recorder();
		~Recorder();

		bool open(const k4a::device& device, k4a_device_configuration_t config, std::string filepath);
		bool close();

		bool writeCapture(const k4a::capture& capture);

		bool addTag(const std::string& name, const std::string& value);

		bool isOpen() const;

	private:
		bool bOpen;

		k4a::record record;
	};
}