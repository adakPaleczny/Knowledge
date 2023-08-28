/*
 * steering_angle_sensor.h
 *
 *  Created on: Aug 1, 2023
 *      Author: adakPaleczny
 */

#ifndef INC_SAS_H_
#define INC_SAS_H_

#include <main.h>
#include "adc.h"
#include "support_functions.h"

#define STEERING_ANGLE_SENSOR_ANGLE					360.0
#define STEERING_ANGLE_SENSOR_MAX_VOLT				5
#define STEERING_ANGLE_SENSOR_MAX_ANGLE_VOLT		4.45f // measured physically
#define STEERING_ANGLE_SENSOR_MIN_ANGLE_VOLT		0.55f // measured physically

// VALUES SET WITH SENSOR CALIBRATION
#define STEERING_ANGLE_SENSOR_POSITION_0			2.50 
#define STEERING_ANGLE_SENSOR_MAX_RIGHT				4.21
#define STEERING_ANGLE_SENSOR_MAX_LEFT				1.07

extern ADC_Reader adc;

class Steering_angle_sensor{
private:
	//direction from sensor is important if we are on the left or on the right of the center
	DIR direction_from_sensor = DIR::LEFT;
	//angle is positive when we are on the left and negative if we are on the right
	float angle_from_sensor = 0;

	//voltage for center and max left and max right
	const float zero_point_angle_volt = STEERING_ANGLE_SENSOR_POSITION_0;
	const float max_left_angle_volt = STEERING_ANGLE_SENSOR_MAX_LEFT;
	const float max_right_angle_volt = STEERING_ANGLE_SENSOR_MAX_RIGHT;

	float getSensorAngleVolt()const		{ return adc.getSensorAngleVolt(); }
	float calculateAngleFromVolt(const float volt_to_angle) const;
public:
    // functions getting values
    DIR getDirection() const 			{ return direction_from_sensor; }
    float getSensorAngle() const			{ return angle_from_sensor; }
    float getMaxRight() const			{ return calculateAngleFromVolt(max_right_angle_volt); }
    float getMaxLeft() const				{ return calculateAngleFromVolt(max_left_angle_volt); }

    //functions for checking is we doesn't want to go further than we are able
    bool furtherThanMaxRight(const float angle) const	{ return angle < calculateAngleFromVolt(max_right_angle_volt) && angle < 0.0; }
	bool furtherThanMaxLeft(const float angle)  const	{ return angle > calculateAngleFromVolt(max_left_angle_volt) && angle > 0.0; }

	//function getting update from sensor
	void update();
};

#endif /* INC_SAS_H_ */
