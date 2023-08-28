/*
 *  steering_angle_sensor.cpp
 *
 *  Created on: Aug 1, 2023
 *      Author: adakPaleczny
 */

#include <steering_angle_sensor.h>
#include <math.h>


void Steering_angle_sensor::update(){
	float angle_volt = getSensorAngleVolt();

	// Getting direction
	direction_from_sensor = angle_volt > zero_point_angle_volt ? DIR::RIGHT : DIR::LEFT;
	
	// Getting actuall angle
	angle_from_sensor = (zero_point_angle_volt - angle_volt)/(STEERING_ANGLE_SENSOR_MAX_ANGLE_VOLT - STEERING_ANGLE_SENSOR_MIN_ANGLE_VOLT) * STEERING_ANGLE_SENSOR_ANGLE;
}


float Steering_angle_sensor::calculateAngleFromVolt(const float volt_to_angle) const {
	return (zero_point_angle_volt - volt_to_angle)/(STEERING_ANGLE_SENSOR_MAX_ANGLE_VOLT - STEERING_ANGLE_SENSOR_MIN_ANGLE_VOLT) * STEERING_ANGLE_SENSOR_ANGLE;
}
