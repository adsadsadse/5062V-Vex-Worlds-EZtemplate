#include "PID.h"
#include "cmath"

void pid::update(float input_sensor){
    sensor = input_sensor;

	//Save the previous error for the derivative
	previous_error = error;

	//Calculate a new error by finding the difference between the current position
	//and the desired position.
	error = target - sensor;

	//Begin summing the errors into the integral term if the error is below a threshold,
	//and reset it if not. This is to prevent the integral from growing too large.
	integral += error;

    if (integral > integral_threshold){
		integral = integral_threshold;
	}
	else if (integral < -integral_threshold){
		integral = -integral_threshold;
	}

	//Calculate the derivative by finding the change between the current error and
	//last update's error
	derivative = error - previous_error;
}

float pid::get_voltage(float input_sensor){
	update(input_sensor);
	return (k_p*error + k_i*integral + k_d*derivative);
}