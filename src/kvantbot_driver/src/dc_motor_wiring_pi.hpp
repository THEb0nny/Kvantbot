#ifndef DC_MOTOR_WIRING_PI_H_
#define DC_MOTOR_WIRING_PI_H_

#include <ros/ros.h>
#include <wiringPi.h>
//#include <GpioExpanderPi.h>

#define RPI_MAX_PWM_VALUE 1023

//GpioExpanderPi expander;

class DCMotorWiringPi {
	public:
		DCMotorWiringPi(int8_t in1_pin, int8_t in2_pin, int8_t pwm_pin);
		void cw(uint16_t val);
		void ccw(uint16_t val);
		void stop();
	private:
		int8_t _in1_pin;
		int8_t _in2_pin;
		int8_t _pwm_pin;
		uint16_t protectOutput(uint16_t val);
};

DCMotorWiringPi::DCMotorWiringPi(int8_t in1_pin, int8_t in2_pin, int8_t pwm_pin) {
	_in1_pin = in1_pin;
	_in2_pin = in2_pin;
	_pwm_pin = pwm_pin;

	/*
	if (!expander.begin())
		printf("Failed to init I2C communication.\n");
	*/

	if (wiringPiSetupGpio() < 0) {
		ROS_ERROR("DCMotor wiringPi error: GPIO setup error");
        throw std::runtime_error("");
	}
	ROS_INFO("DCMotor wiringPi: GPIO setup");
	pinMode(_in1_pin, OUTPUT);
	pinMode(_in2_pin, OUTPUT);
	pinMode(_pwm_pin, PWM_OUTPUT);
	//expander.pinMode(_pwm_pin, GPIO_PIN_OUTPUT);
	stop();
	ROS_INFO("DCMotor wiringPi: Motor setup");
}

void DCMotorWiringPi::stop() {
	pwmWrite(_pwm_pin, 0);
	//expander.analogWrite(_pwm_pin, 0);
	digitalWrite(_in1_pin, 0);
	digitalWrite(_in2_pin, 0);
}

void DCMotorWiringPi::cw(uint16_t val) {
	//expander.analogWrite(_pwm_pin, val);
	pwmWrite(_pwm_pin, protectOutput(val));
	digitalWrite(_in1_pin, 0);
	digitalWrite(_in2_pin, 1);
}

void DCMotorWiringPi::ccw(uint16_t val) {
	//expander.analogWrite(_pwm_pin, val);
	pwmWrite(_pwm_pin, protectOutput(val));
	digitalWrite(_in1_pin, 1);
	digitalWrite(_in2_pin, 0);
}

uint16_t DCMotorWiringPi::protectOutput(uint16_t val) {
	return val > RPI_MAX_PWM_VALUE ? RPI_MAX_PWM_VALUE : val;
}

#endif // DC_MOTOR_WIRING_PI_H_