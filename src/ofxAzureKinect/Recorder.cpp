#include "Recorder.h"

namespace ofxAzureKinect
{
	Recorder::Recorder()
	{

	}

	Recorder::~Recorder()
	{
		this->close();
	}

	bool Recorder::open(const k4a::device& device, k4a_device_configuration_t config, std::string filepath)
	{
		if (this->bOpen) return false;

		if (filepath.empty())
		{
			ofLogError(__FUNCTION__) << "File path cannot be empty!";
		}

		filepath = ofToDataPath(filepath, true);

		try
		{
			this->record = k4a::record::create(filepath.c_str(), device, config);
			
			// TODO: Add IMU and other custom tracks here.

			// Write header after all track metadata is setup.
			this->record.write_header();
		}
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		ofLogNotice(__FUNCTION__) << "Open success, writing to file " << filepath;

		this->bOpen = true;
		return true;
	}

	bool Recorder::close()
	{
		if (!this->bOpen) return false;

		this->record.flush();
		this->record.close();

		ofLogNotice(__FUNCTION__) << "Close success";

		this->bOpen = false;
		return true;
	}

	bool Recorder::writeCapture(const k4a::capture& capture)
	{
		if (!this->isOpen())
		{
			ofLogError(__FUNCTION__) << "Open recorder before writing!";
			return false;
		}

		this->record.write_capture(capture);
		return true;
	}

	bool Recorder::addTag(const std::string& name, const std::string& value)
	{
		if (!this->isOpen())
		{
			ofLogError(__FUNCTION__) << "Open recorder before writing!";
			return false;
		}

		this->record.add_tag(name.c_str(), value.c_str());
		return true;
	}

	bool Recorder::isOpen() const
	{
		return this->bOpen;
	}
}