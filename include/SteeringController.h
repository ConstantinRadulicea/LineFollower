#pragma once
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define abs(x)  (((x)<0)?-(x):(x))


#define ENABLE_ARDUINO 1
#if ENABLE_ARDUINO == 1
	#include <Arduino.h>
#endif
#define BaseSteeringController_ENABLE_PURE_VIRTUAL_FUNCTIONS 1


#if ENABLE_ARDUINO == 1
	void moveBackward(int pin1, int pin2, int val) {
		analogWrite(pin1, val);
		analogWrite(pin2, LOW);
	}

	void moveForward(int pin1, int pin2, int val) {
		analogWrite(pin1, LOW);
		analogWrite(pin2, val);
	}
#endif // ENABLE_ARDUINO == 1





class BaseSteeringController
{
public:
	BaseSteeringController(float max_forward_raw_value, float stand_still_speed_raw_value, float max_backward_raw_value){
		this->MaxForwardSpeedRawValue = max_forward_raw_value;
		this->MaxBackwardSpeedRawValue = max_backward_raw_value;
		this->StandStillSpeedRawValue = stand_still_speed_raw_value;
		this->LeftTrackSpeed = 0.0f;
		this->RightTrackSpeed = 0.0f;
		LeftTrackIsGoingForward = false;
		LeftTrackIsGoingBackward = false;
		LeftTrackIsStandingStill = false;
		RightTrackIsGoingForward = false;
		RightTrackIsGoingBackward = false;
		RightTrackIsStandingStill = false;
	}
	~BaseSteeringController(){}

#if BaseSteeringController_ENABLE_PURE_VIRTUAL_FUNCTIONS == 1
	virtual void writeLeftTrackMotor() = 0;
	virtual void writeRightTrackMotor() = 0;
#endif // BaseSteeringController_ENABLE_PURE_VIRTUAL_FUNCTIONS == 1

	void setMaxForwardSpeedRawValue(float speed) {
		this->MaxForwardSpeedRawValue = speed;
	}

	float getMaxForwardSpeedRawValue() {
		return this->MaxForwardSpeedRawValue;
	}

	void setMaxBackwardSpeedRawValue(float speed) {
		this->MaxBackwardSpeedRawValue = speed;
	}

	float getMaxBackwardSpeedRawValue() {
		return this->MaxBackwardSpeedRawValue;
	}

	void setStandStillSpeedRawValue(float speed) {
		this->StandStillSpeedRawValue = speed;
	}

	float getStandStillSpeedRawValue() {
		return this->StandStillSpeedRawValue;
	}

	float getLeftTrackSpeed() {
		return this->LeftTrackSpeed;
	}

	float getRightTrackSpeed() {
		return this->RightTrackSpeed;
	}

	void calculateSteering(float speed_percentage, float left_track_percentage, float right_track_percentage) {
		float temp_LeftTrackSpeed, temp_RightTrackSpeed;
		float forward_speed_span, backward_speed_span;

		LeftTrackIsGoingForward = false;
		LeftTrackIsGoingBackward = false;
		LeftTrackIsStandingStill = false;
		RightTrackIsGoingForward = false;
		RightTrackIsGoingBackward = false;
		RightTrackIsStandingStill = false;


		temp_LeftTrackSpeed = left_track_percentage * speed_percentage;
		temp_RightTrackSpeed = right_track_percentage * speed_percentage;

		forward_speed_span = MaxForwardSpeedRawValue - StandStillSpeedRawValue;
		backward_speed_span = StandStillSpeedRawValue - MaxBackwardSpeedRawValue;


		if (temp_LeftTrackSpeed == 0.0f) {
			temp_LeftTrackSpeed = StandStillSpeedRawValue;
			LeftTrackIsStandingStill = true;
		}
		else if (temp_LeftTrackSpeed > 0.0f) {
			temp_LeftTrackSpeed = StandStillSpeedRawValue + (abs(temp_LeftTrackSpeed) * forward_speed_span);
			LeftTrackIsGoingForward = true;
		}
		else {
			temp_LeftTrackSpeed = StandStillSpeedRawValue - (abs(temp_LeftTrackSpeed) * backward_speed_span);
			LeftTrackIsGoingBackward = true;
		}

		if (temp_RightTrackSpeed == 0.0f) {
			temp_RightTrackSpeed = StandStillSpeedRawValue;
			RightTrackIsStandingStill = true;
		}
		else if (temp_RightTrackSpeed > 0.0f) {
			temp_RightTrackSpeed = StandStillSpeedRawValue + (abs(temp_RightTrackSpeed) * forward_speed_span);
			RightTrackIsGoingForward = true;
		}
		else {
			temp_RightTrackSpeed = StandStillSpeedRawValue - (abs(temp_RightTrackSpeed) * backward_speed_span);
			RightTrackIsGoingBackward = true;
		}

		LeftTrackSpeed = abs(temp_LeftTrackSpeed);
		RightTrackSpeed = abs(temp_RightTrackSpeed);
	}

	void write(float speed_percentage, float left_track_percentage, float right_track_percentage) {
		/*
		speed_percentage = MIN(speed_percentage, 1.0f);
		speed_percentage = MAX(speed_percentage, -1.0f);
		left_track_percentage = MIN(left_track_percentage, 1.0f);
		left_track_percentage = MAX(left_track_percentage, -1.0f);
		right_track_percentage = MIN(right_track_percentage, 1.0f);
		right_track_percentage = MAX(right_track_percentage, -1.0f);
		*/
		this->calculateSteering(speed_percentage, left_track_percentage, right_track_percentage);

		#if BaseSteeringController_ENABLE_PURE_VIRTUAL_FUNCTIONS == 1
			this->writeLeftTrackMotor();
			this->writeRightTrackMotor();
		#endif // BaseSteeringController_ENABLE_PURE_VIRTUAL_FUNCTIONS == 1
	}

protected:
	float MaxForwardSpeedRawValue;
	float StandStillSpeedRawValue;
	float MaxBackwardSpeedRawValue;
	float LeftTrackSpeed;
	float RightTrackSpeed;

	float LeftTrackIsGoingForward;
	float LeftTrackIsGoingBackward;
	float LeftTrackIsStandingStill;

	float RightTrackIsGoingForward;
	float RightTrackIsGoingBackward;
	float RightTrackIsStandingStill;
};



class SteeringController : public BaseSteeringController
{
public:
	SteeringController(float max_forward_raw_value, float stand_still_speed_raw_value, float max_backward_raw_value) : BaseSteeringController(max_forward_raw_value, stand_still_speed_raw_value, max_backward_raw_value){};
	void attach(int left_1, int left_2, int right_3, int right_4){
		this->motors_left_in1 = left_1;
		this->motors_left_in2 = left_2;
		this->motors_right_in3 = right_3;
		this->motors_right_in4 = right_4;
	}
	virtual void writeLeftTrackMotor() {
		if (this->LeftTrackIsStandingStill == true) {
			#if ENABLE_ARDUINO == 1
				moveForward(this->motors_left_in1, this->motors_left_in2, (int)(this->LeftTrackSpeed));
			#endif // ENABLE_ARDUINO == 1
		}
		else if (this->LeftTrackIsGoingForward == true) {
			#if ENABLE_ARDUINO == 1
				moveForward(this->motors_left_in1, this->motors_left_in2, (int)(this->LeftTrackSpeed));
			#endif // ENABLE_ARDUINO == 1
		}
		else if (this->LeftTrackIsGoingBackward == true) {
			#if ENABLE_ARDUINO == 1
				moveBackward(this->motors_left_in1, this->motors_left_in2, (int)(this->LeftTrackSpeed));
			#endif // ENABLE_ARDUINO == 1
		}
	}

	virtual void writeRightTrackMotor() {
		if (this->RightTrackIsStandingStill == true) {
			#if ENABLE_ARDUINO == 1
				moveForward(this->motors_right_in3, this->motors_right_in4, (int)(this->RightTrackSpeed));
			#endif // ENABLE_ARDUINO == 1
		}
		else if (this->RightTrackIsGoingForward == true) {
			#if ENABLE_ARDUINO == 1
				moveForward(this->motors_right_in3, this->motors_right_in4, (int)(this->RightTrackSpeed));
			#endif // ENABLE_ARDUINO == 1
		}
		else if (this->RightTrackIsGoingBackward == true) {
			#if ENABLE_ARDUINO == 1
				moveBackward(this->motors_right_in3, this->motors_right_in4, (int)(this->RightTrackSpeed));
			#endif // ENABLE_ARDUINO == 1
		}
	}

private:
	int motors_left_in1 = -1;
	int motors_left_in2 = -1;
	int motors_right_in3 = -1;
	int motors_right_in4 = -1;
};