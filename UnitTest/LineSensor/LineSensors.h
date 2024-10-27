#pragma once

#define ENABLE_PLATFORM_SPECIFIC 0

#if ENABLE_PLATFORM_SPECIFIC == 1
#include <Arduino.h>
#endif // ENABLE_PLATFORM_SPECIFIC == 1



#include <string.h>
#include "geometry2D.h"
#include "cubic.c"
#include "MedianFilter.h"
#include <vector>

#define LINESENSORS_MEDIANFILTER_WINDOW 1

class LineSensor_line {
public:


	LineSensor_line() : _sensorStart(0), width(0) {}
	LineSensor_line(unsigned int sensor_start_index, unsigned int line_width): _sensorStart(sensor_start_index), width(line_width){}

	void SetFirstSensorIndex(unsigned int sensor_start_index) {
		this->_sensorStart = sensor_start_index;
	}
	void SetWidth(unsigned int line_width) {
		this->width = line_width;
	}
	float GetCenter() {
		return (((float)this->_sensorStart) + (((float)this->width) / 2.0f));
	}
	unsigned int GetWidth() {
		return this->width;
	}
	unsigned int GetFirstSensorIndex() {
		return this->_sensorStart;
	}
	unsigned int GetLastSensorIndex() {
		if (this->width == 0) {
			return this->_sensorStart;
		}
		return this->_sensorStart + this->width - 1;
	}

	bool IsValid() {
		return this->width != 0;
	}
	void clear() {
		this->_sensorStart = 0;
		this->width = 0;
	}
private:
	unsigned int _sensorStart;
	unsigned int width;
};

// 0% = white
// 100% = black;
class LineSensors
{
public:
	LineSensors(size_t NumberOfSensors_) {
		this->NumberOfSensors = NumberOfSensors_;
		this->BackgroundColorOnlyCalibrationAvarages = new float[NumberOfSensors_];
		this->LineColorOlyCalibrationAvarages = new float[NumberOfSensors_];
		this->ColorRange = new float[NumberOfSensors_];
		this->BlackLineTreshold = 0.5f;
	}

	void SetBackgroundColorOnlyCalibrationAvarages(float* sensorsReadings) {
		memcpy(this->BackgroundColorOnlyCalibrationAvarages, sensorsReadings, sizeof(float) * NumberOfSensors);
	}
	void SetLineColorOnlyCalibrationAvarages(float* sensorsReadings) {
		memcpy(this->LineColorOlyCalibrationAvarages, sensorsReadings, sizeof(float) * NumberOfSensors);
	}

	void CalculateColorRange(float* backgroundColorOnlyCalibrationAvarages, float* lineColorOnlyCalibrationAvarages) {
		this->SetBackgroundColorOnlyCalibrationAvarages(backgroundColorOnlyCalibrationAvarages);
		this->SetLineColorOnlyCalibrationAvarages(lineColorOnlyCalibrationAvarages);
		for (size_t i = 0; i < this->NumberOfSensors; i++) {
			this->ColorRange[i] = fabs(this->BackgroundColorOnlyCalibrationAvarages[i] - this->LineColorOlyCalibrationAvarages[i]);
		}
	}

	void CalibrateRawValuesToPercentuals(float* sensorsReadings) {
		for (size_t i = 0; i < this->NumberOfSensors; i++) {
			if (this->BackgroundColorOnlyCalibrationAvarages[i] > this->LineColorOlyCalibrationAvarages[i]) {
				sensorsReadings[i] = (sensorsReadings[i] - this->LineColorOlyCalibrationAvarages[i]) / this->ColorRange[i];
			}
			else {
				sensorsReadings[i] = (sensorsReadings[i] - this->BackgroundColorOnlyCalibrationAvarages[i]) / this->ColorRange[i];
			}
		}
	}
	
	std::vector<LineSensor_line> GetLines(float* sensorsReadings, float black_treshold_percentuale) {
		this->CalibrateRawValuesToPercentuals(sensorsReadings);

		std::vector<LineSensor_line> lines;
		LineSensor_line cur_line;

		for (size_t i = 0; i < this->NumberOfSensors; i++)
		{
			if (sensorsReadings[i] >= black_treshold_percentuale)
			{
				if (cur_line.IsValid()) {
					cur_line.SetWidth(cur_line.GetWidth()+1);
				}
				else {
					cur_line.SetWidth(1);
					cur_line.SetFirstSensorIndex(i);
				}
			}
			else if (cur_line.IsValid()) {
				lines.push_back(cur_line);
				cur_line.clear();
			}
		}
		if (lines.size() > 1) {
			LineSensor_lines_bubbleSortDescending(&lines);
		}
		return lines;
	}

	inline std::vector<LineSensor_line> GetLines(float* sensorsReadings) {
		return this->GetLines(sensorsReadings, this->BlackLineTreshold);
	}


	~LineSensors() {
		delete this->BackgroundColorOnlyCalibrationAvarages;
		delete this->LineColorOlyCalibrationAvarages;
		delete this->ColorRange;
	}


private:
	size_t NumberOfSensors;
	float* BackgroundColorOnlyCalibrationAvarages = nullptr;
	float* LineColorOlyCalibrationAvarages = nullptr;
	float* ColorRange = nullptr;
	float BlackLineTreshold;


	static void LineSensor_lines_bubbleSortDescending(std::vector<LineSensor_line>* arr) {
		int n = (int)arr->size();
		LineSensor_line temp;
		for (int i = 0; i < n - 1; i++) {
			for (int j = 0; j < n - i - 1; j++) {
				if (arr->at(j).GetWidth() < arr->at(j + 1).GetWidth()) {
					// Swap arr[j] and arr[j+1]
					temp = arr->at(j);
					arr->at(j) = arr->at(j + 1);
					arr->at(j + 1) = temp;
				}
			}
		}
	}


};

