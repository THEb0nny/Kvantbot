#ifndef LINE_SENSOR_WIRING_PI_H_
#define LINE_SENSOR_WIRING_PI_H_

#include <ros/ros.h>
#include <GpioExpanderPi.h>

GpioExpanderPi expander;

class LineSensorGpioExpPi {
	public:
		LineSensorGpioExpPi(int8_t in_pin);
        uint16_t getRawValue();

	private:
		int8_t _in_pin;
};

LineSensorGpioExpPi::LineSensorGpioExpPi(int8_t in_pin) {
	_in_pin = in_pin;

	if (!expander.begin()) {
        ROS_ERROR("Failed to init I2C communication.\n");
        throw std::runtime_error("");
	}

	//ROS_INFO("Line sensor GpioExpanderPi: I2C communication setup");
	expander.pinMode(_in_pin, GPIO_PIN_OUTPUT);
	ROS_INFO("LineSensor GpioExpanderPi: Sensor setup in PIN %i", in_pin);
}

uint16_t LineSensorGpioExpPi::getRawValue() {
	uint16_t sensor_val = expander.analogRead(_in_pin);
	return sensor_val;
}

#endif // LINE_SENSOR_WIRING_PI_H_