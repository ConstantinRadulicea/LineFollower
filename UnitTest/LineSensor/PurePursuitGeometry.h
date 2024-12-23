﻿/*
* Copyright 2023 Constantin Dumitru Petre RĂDULICEA
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*   http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/




#ifndef __PUREPURSUITGEOMETRY_H__
#define __PUREPURSUITGEOMETRY_H__

#include "geometry2D.h"

typedef struct PurePursuitInfo {
	Point2D carPos;
	Point2D nextWayPoint;
	Point2D turnPoint;
	float distanceToWayPoint;
	float lookAheadDistance;
	float TrajectoryToWayPointAngle;
	float steeringAngle;
	float wheelBase;
	float turnRadius;
	float manouvreLength;
}PurePursuitInfo;

static float carTrajectoryAndWayPointAngle(Point2D carPos, Point2D nextWayPoint) {
	float lookAheadDistance, TrajectoryToWayPointAngle;
	Point2D temp;

	lookAheadDistance = euclidianDistance(carPos, nextWayPoint);
	temp = carPos;
	temp.y += lookAheadDistance;
	TrajectoryToWayPointAngle = triangleAngleA(lookAheadDistance, euclidianDistance(nextWayPoint, temp), lookAheadDistance);
	
	if (floatCmp(carPos.x, nextWayPoint.x) < 0) {
		TrajectoryToWayPointAngle = -TrajectoryToWayPointAngle;
	}
	return TrajectoryToWayPointAngle;
}

static float steeringWheelAngle(float TrajectoryToWayPointAngle, float wheelBase, float nextWayPointDistance) {
	float angle;
	float temp_float;
	if (floatCmp(nextWayPointDistance, 0.0f) == 0) {
		return 0.0f;
	}
	
	angle = atanf((2.0f * wheelBase * sinf(TrajectoryToWayPointAngle)) / nextWayPointDistance);
	return angle;
}

static float turnRadiusByWaypoint(float TrajectoryToWayPointAngle, float wheelBase, float nextWayPointDistance) {
	float angle;
	float temp_float;
	
	temp_float = sinf(TrajectoryToWayPointAngle);
	if (floatCmp(temp_float, 0.0f) == 0) {
		return 0.0f;
	}
	angle = (nextWayPointDistance / (2.0f * sinf(TrajectoryToWayPointAngle)));
	return angle;
}

static float turnRadius(float wheelBase, float turnAngle) {
	float angle;
	if (floatCmp(turnAngle, 0.0f) == 0) {
		return 0.0f;
	}
	
	angle = (wheelBase / tanf(turnAngle));
	angle = fabsf(angle);
	return angle;
}

static float purePursuitComputeSteeringWheelAngle(Point2D carPos, LineMQ wayPoints, float wheelBase, float lookAheadDistance) {
	float temp;
	IntersectionPoints2D_2 intersectionPoints;
	Point2D nextWayPoint;

	temp = distance2lineMQ(carPos, wayPoints);
	
	if (floatCmp(temp, lookAheadDistance) >= 0) {
		lookAheadDistance = temp + (temp * 0.1f);
	}

	intersectionPoints = intersectionLineCircleMQ(carPos, lookAheadDistance, wayPoints);
	
	if (floatCmp(intersectionPoints.point1.y, intersectionPoints.point2.y) > 0) {
		nextWayPoint = intersectionPoints.point1;
	}
	else {
		nextWayPoint = intersectionPoints.point2;
	}

	temp = carTrajectoryAndWayPointAngle(carPos, nextWayPoint);
	return steeringWheelAngle(temp, wheelBase, lookAheadDistance);
}

static PurePursuitInfo purePursuitComputeMQ(Point2D carPos, LineMQ wayPoints, float wheelBase, float lookAheadDistance) {
	float temp;
	PurePursuitInfo info;
	IntersectionPoints2D_2 intersectionPoints;
	Point2D nextWayPoint;

	temp = distance2lineMQ(carPos, wayPoints);
	info.distanceToWayPoint = temp;
	if (floatCmp(temp, lookAheadDistance) >= 0) {
		lookAheadDistance = temp + (temp * 0.25f);
	}

	intersectionPoints = intersectionLineCircleMQ(carPos, lookAheadDistance, wayPoints);
	if (floatCmp(intersectionPoints.point1.y, intersectionPoints.point2.y) > 0) {
		nextWayPoint = intersectionPoints.point1;
	}
	else {
		nextWayPoint = intersectionPoints.point2;
	}
	info.TrajectoryToWayPointAngle = carTrajectoryAndWayPointAngle(carPos, nextWayPoint);
	info.steeringAngle = steeringWheelAngle(info.TrajectoryToWayPointAngle, wheelBase, lookAheadDistance);

	info.carPos = carPos;
	info.nextWayPoint = nextWayPoint;
	info.lookAheadDistance = lookAheadDistance;
	info.wheelBase = wheelBase;
	info.turnRadius = turnRadius(wheelBase, info.steeringAngle);
	info.manouvreLength = fabsf(((2.0f * M_PI * info.turnRadius) * info.TrajectoryToWayPointAngle) / (2.0f * M_PI));
	info.turnPoint = carPos;

	

	if (floatCmp(info.TrajectoryToWayPointAngle, 0.0f) < 0) {
		info.turnPoint.x += info.turnRadius;
	}
	else {
		info.turnPoint.x -= info.turnRadius;
	}

	return info;
}

static PurePursuitInfo purePursuitComputeABC(Point2D carPos, LineABC wayPoints, float wheelBase, float lookAheadDistance) {
	float temp;
	PurePursuitInfo info;
	IntersectionPoints2D_2 intersectionPoints;
	Point2D nextWayPoint;

	temp = distance2lineABC(carPos, wayPoints);
	info.distanceToWayPoint = temp;
	if (floatCmp(temp, lookAheadDistance) >= 0) {
		lookAheadDistance = temp + (temp * 0.1f);
	}

	intersectionPoints = intersectionLineCircleABC(carPos, lookAheadDistance, wayPoints);
	if (floatCmp(intersectionPoints.point1.y, intersectionPoints.point2.y) > 0) {
		nextWayPoint = intersectionPoints.point1;
	}
	else {
		nextWayPoint = intersectionPoints.point2;
	}
	info.TrajectoryToWayPointAngle = carTrajectoryAndWayPointAngle(carPos, nextWayPoint);
	info.steeringAngle = steeringWheelAngle(info.TrajectoryToWayPointAngle, wheelBase, lookAheadDistance);

	info.carPos = carPos;
	info.nextWayPoint = nextWayPoint;
	info.lookAheadDistance = lookAheadDistance;
	info.wheelBase = wheelBase;
	info.turnRadius = turnRadius(wheelBase, info.steeringAngle);
	info.manouvreLength = fabsf(((2.0f * M_PI * info.turnRadius) * info.TrajectoryToWayPointAngle) / (2.0f * M_PI));
	info.turnPoint = carPos;

	if (floatCmp(info.TrajectoryToWayPointAngle, 0.0f) < 0) {
		info.turnPoint.x += info.turnRadius;
	}
	else {
		info.turnPoint.x -= info.turnRadius;
	}

	return info;
}


static float carTurnMaxSpeed(float _turn_radius, float _friction_coefficient, float _downward_acceleration) {
	float _max_speed;
	_max_speed = sqrtf(fabs(_friction_coefficient * _turn_radius * _downward_acceleration));
	return _max_speed;
}


float turnRadius_startEndPoint(Point2D start_point, Point2D end_point) {
	LineABC start_end_point_line;
	Point2D turn_point = start_point;
	float angle_bw_x_axis_and_start_end_point_line;
	float angle_aBc, angle_bCa, angle_cAb;
	float side_ab, side_bc, side_ca;
	float temp;

	
	start_end_point_line = points2lineABC(start_point, end_point);
	angle_cAb = angleBetweenLinesABC(start_end_point_line, xAxisABC());
	angle_bCa = angle_cAb;
	angle_aBc = M_PI - angle_bCa - angle_cAb;

	temp = sinf(angle_aBc);
	if (floatCmp(temp, 0.0f) == 0) {
		return 0.0f;
	}

	side_ca = euclidianDistance(start_point, end_point);
	side_bc = (sinf(angle_cAb) * side_ca)/ temp;


	return side_bc;
}

static PurePursuitInfo purePursuitCompute_OneAxle_ABC(Point2D carPos, LineABC wayPoints, float lookAheadDistance) {
	float temp;
	PurePursuitInfo info;
	IntersectionPoints2D_2 intersectionPoints;
	Point2D nextWayPoint;

	temp = distance2lineABC(carPos, wayPoints);
	info.distanceToWayPoint = temp;
	if (floatCmp(temp, lookAheadDistance) >= 0) {
		lookAheadDistance = temp + (temp * 0.1f);
	}

	intersectionPoints = intersectionLineCircleABC(carPos, lookAheadDistance, wayPoints);
	if (floatCmp(intersectionPoints.point1.y, intersectionPoints.point2.y) > 0) {
		nextWayPoint = intersectionPoints.point1;
	}
	else {
		nextWayPoint = intersectionPoints.point2;
	}
	info.TrajectoryToWayPointAngle = carTrajectoryAndWayPointAngle(carPos, nextWayPoint);




	//info.steeringAngle = steeringWheelAngle(info.TrajectoryToWayPointAngle, wheelBase, lookAheadDistance);

	info.carPos = carPos;
	info.nextWayPoint = nextWayPoint;
	info.lookAheadDistance = lookAheadDistance;
	info.wheelBase = 0.0f;
	info.turnRadius = turnRadius_startEndPoint(carPos, nextWayPoint);
	info.manouvreLength = fabsf(((2.0f * M_PI * info.turnRadius) * info.TrajectoryToWayPointAngle) / (2.0f * M_PI));
	info.turnPoint = carPos;

	if (floatCmp(info.TrajectoryToWayPointAngle, 0.0f) < 0) {
		info.turnPoint.x += info.turnRadius;
	}
	else {
		info.turnPoint.x -= info.turnRadius;
	}

	return info;
}

#endif // !__PUREPURSUITGEOMETRY_H__
