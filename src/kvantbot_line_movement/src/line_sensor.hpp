#ifndef LINE_SENSOR_WIRING_PI_H_
#define LINE_SENSOR_WIRING_PI_H_

#include <ros/ros.h>
#include <GpioExpanderPi.h>

GpioExpanderPi expander;

class LineSensorWiringPi {
	public:
		LineSensorWiringPi(int8_t in_pin);
        uint16_t getRawValue();
	private:
		int8_t _in_pin;
};

LineSensorWiringPi::LineSensorWiringPi(int8_t in_pin) {
	_in_pin = in_pin;

	if (!expander.begin())
        ROS_ERROR("Failed to init I2C communication.\n");
        throw std::runtime_error("");

	ROS_INFO("LineSensor wiringPi: I2C communication setup");
	expander.pinMode(_in_pin, GPIO_PIN_OUTPUT);
	ROS_INFO("LineSensor wiringPi: Sensor setup");
}

uint16_t LineSensorWiringPi::getRawValue() {
	uint16_t sensor_val = expander.analogRead(_in_pin);
	return sensor_val;
}

#endif // LINE_SENSOR_WIRING_PI_H_