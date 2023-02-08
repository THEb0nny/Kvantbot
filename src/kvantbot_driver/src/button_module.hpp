#ifndef BTN_MODULE_WIRING_PI_H_
#define BTN_MODULE_WIRING_PI_H_

#include <ros/ros.h>
#include <GpioExpanderPi.h>

GpioExpanderPi expander;

class BtnModuleGpioExpPi {
	public:
		BtnModuleGpioExpPi(int8_t in_pin);
        bool read();

	private:
		int8_t _in_pin;
};

BtnModuleGpioExpPi::BtnModuleGpioExpPi(int8_t in_pin) {
	_in_pin = in_pin;

	if (!expander.begin()) {
        ROS_ERROR("Failed to init I2C communication.\n");
        throw std::runtime_error("");
	}

	//ROS_INFO("Button module GpioExpanderPi: I2C communication setup");
	expander.pinMode(_in_pin, GPIO_PIN_INPUT);
	ROS_INFO("Btn GpioExpanderPi: Button setup in PIN %i", in_pin);
}

bool BtnModuleGpioExpPi::read() {
	return expander.digitalRead(_in_pin); // Инвертируем значение, т.к. кнопка притянута к земле
}

#endif // BTN_MODULE_WIRING_PI_H_