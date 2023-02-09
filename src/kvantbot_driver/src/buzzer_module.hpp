#ifndef BUZZER_MODULE_WIRING_PI_H_
#define BUZZER_MODULE_WIRING_PI_H_

#include <ros/ros.h>
#include <wiringPi.h>
#include <softTone.h>
#include <GpioExpanderPi.h>
#include <std_msgs/Time.h>
#include <boost/thread.hpp>

#define CM1 262
#define CM2 294
#define CM3 330
#define CM4 350
#define CM5 393
#define CM6 441
#define CM7 495

#define CH1 525
#define CH2 589
#define CH3 661
#define CH4 700
#define CH5 786
#define CH6 882
#define CH7 990

int song_1[] = {CH5, CH2, CM6, CH2, CH3, CH6, 0, CH3, CH5, CH3, CM6, CH2, 0};
int beat_1[] = {1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 3};

GpioExpanderPi expander;

class BuzzerModuleGpioExpPi {
	public:
		BuzzerModuleGpioExpPi(int8_t in_pin);
        void on();
		void off();
		void tone(int frequency);
		void startTone();
		void playTone();

	private:
		int8_t _in_pin;
		bool _threadPlayToneState;
		
};

BuzzerModuleGpioExpPi::BuzzerModuleGpioExpPi(int8_t in_pin) {
	_in_pin = in_pin;
	_threadPlayToneState = false;

	if (wiringPiSetupGpio() < 0) {
		ROS_ERROR("Buzzer wiringPi error: GPIO setup error");
        throw std::runtime_error("");
	}
	ROS_INFO("BuzzerwiringPi: GPIO setup");

	if(softToneCreate(in_pin) == -1) {
        ROS_ERROR("Setup softTone failed!\n");
        throw std::runtime_error("");
    }

	if (!expander.begin()) {
        ROS_ERROR("Failed to init I2C communication.\n");
        throw std::runtime_error("");
	}

	//ROS_INFO("Buzzer module GpioExpanderPi: I2C communication setup");
	expander.pinMode(_in_pin, GPIO_PIN_OUTPUT);
	ROS_INFO("Buzzer GpioExpanderPi: Buzzer setup in PIN %i", in_pin);
}

void BuzzerModuleGpioExpPi::on() {
	softToneWrite(_in_pin, song_1[1]);
}

void BuzzerModuleGpioExpPi::off() {
	_threadPlayToneState = false; // Записать в переменную состояния проигрывания тона false для прерывания
	softToneWrite(_in_pin, 0);
}

void BuzzerModuleGpioExpPi::tone(int frequency) {
	softToneWrite(_in_pin, song_1[1]);
}

void BuzzerModuleGpioExpPi::startTone() {
	if (!_threadPlayToneState) { // Если переменная проигрывания тона была false, тогда можно запустить проигрывание
		_threadPlayToneState = true; // Устанавливаем true, что проигрывание было запущено
		boost::thread threadPlayTone = boost::thread(boost::bind(&BuzzerModuleGpioExpPi::playTone, this)); // Запускаем в параллельной задаче функцию проигрывания тона
	}
}

void BuzzerModuleGpioExpPi::playTone() {
	ROS_INFO("Buzzer play!\n");
	for(int i = 0; i < sizeof(song_1) / 4; i++) {
		if(!_threadPlayToneState) { // Если флажок установили на false, тогда нужно прервать проигрывание немедленно
			break;
		}
		ros::Time start_time = ros::Time::now();
		ros::Duration timeout((beat_1[i] * 250.0) / 1000.0);
		softToneWrite(_in_pin, song_1[i]);
		//delay(beat_1[i] * 250);
		while(ros::Time::now() - start_time < timeout) {
			if(!_threadPlayToneState) { // Если флажок установили на false, тогда нужно прервать проигрывание немедленно
				ROS_INFO("Buzzer play interrupted!\n");
				break;
			}
		}
	}
	_threadPlayToneState = false; // Проигрывание мелодии завершено
}

#endif // BUZZER_MODULE_WIRING_PI_H_