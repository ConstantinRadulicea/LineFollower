#include "LineSensors.h"
#include "CalculateCarSpeed.h"
#include "CalculateLookAheadDistance.h"

#define DISTANCE_BW_SENSORS_M 0.008f
#define TRACKWIDTH_M 0.17f
#define TOTAL_SENSORS 16

float background_color[]{
	1024,
	1024,
	1024,
	1024,
	1024,
	1024,
	1024,
	1024,
	1024,
	1024,
	1024,
	1024,
	1024,
	1024,
	1024,
	1024,
};


float line_color[]{
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
};

float raw_values[] {
	500,
	0,
	0,
	0,
	700,
	700,
	700,
	0,
	0,
	0,
	0,
	400,
	0,
	0,
	0,
	0,
};

float LineToXAxisPosition(LineSensor_line lines) {
	return lines.GetCenter() - (float)(TOTAL_SENSORS / 2);
}


float g_lookahead_min_distance_m = 0.2f;
float g_lookahead_max_distance_m = 0.5f;
float g_lookahead_distance_m;
Point2D g_car_position = Point2D{0.0f, 0.0f};
LineABC g_middle_line;
PurePursuitInfo g_pure_pursuit_info;


void main() {
	LineSensors lineSensor(16);
	std::vector<LineSensor_line> lines;
	lineSensor.CalculateColorRange(background_color, line_color);
	lines = lineSensor.GetLines(raw_values, 0.3);

	
	if (lines.size() > 0)
	{
		g_middle_line = points2lineABC(g_car_position, Point2D{ LineToXAxisPosition(lines[0]), 100.0f });
	}
	else {
		g_middle_line = yAxisABC();
	}


	g_lookahead_distance_m = CalculateLookAheadDistance(g_lookahead_min_distance_m, g_lookahead_max_distance_m, g_middle_line);
	g_pure_pursuit_info = purePursuitCompute_OneAxle_ABC(g_car_position, g_middle_line, g_lookahead_distance_m);

	//float temp = turnRadius_startEndPoint(g_car_position, Point2D{10.0f, 10.0f});
}




// left_right_turn: negative if turning left, positive if turning right
void SetSpeedRequest(float speed_ms, float turn_radius, int left_right_turn) volatile {
	float left_wheel_turn_radius;
	float right_wheel_turn_radius;
	float left_wheel_turn_circonference;
	float right_wheel_turn_circonference;
	float car_trun_circonference;
	float left_wheel_speed_request_m;
	float right_wheel_speed_request_m;

	turn_radius = fabs(turn_radius);

	if (PowerTrainfloatCmp(turn_radius, 0.0) == 0 || left_right_turn == 0)	// going straight
	{
		left_wheel_speed_request_m = speed_ms;
		right_wheel_speed_request_m = speed_ms;
	}
	else {
		if (left_right_turn < 0)	// left turn
		{
			left_wheel_turn_radius = turn_radius - (this->_distanceBwWheels_m / 2.0);
			right_wheel_turn_radius = turn_radius + (this->_distanceBwWheels_m / 2.0);
		}
		else if (left_right_turn > 0)	// right turn
		{
			left_wheel_turn_radius = turn_radius + (this->_distanceBwWheels_m / 2.0);
			right_wheel_turn_radius = turn_radius - (this->_distanceBwWheels_m / 2.0);
		}

		left_wheel_turn_circonference = (2.0 * left_wheel_turn_radius) * M_PI;
		right_wheel_turn_circonference = (2.0 * right_wheel_turn_radius) * M_PI;
		car_trun_circonference = (2.0 * turn_radius) * M_PI;

		left_wheel_speed_request_m = (left_wheel_turn_circonference / car_trun_circonference) * speed_ms;
		right_wheel_speed_request_m = (right_wheel_turn_circonference / car_trun_circonference) * speed_ms;
	}

	this->SetLeftWheelSpeedRequest(left_wheel_speed_request_m);
	this->SetRightWheelSpeedRequest(right_wheel_speed_request_m);
}