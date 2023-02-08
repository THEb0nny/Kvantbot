#include "buzzer_module.hpp"
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <boost/thread.hpp>
#include <chrono>

// Пинs модуля пльезопищалки через плату расширения Troyka Hat
#define BUZZER_MODULE_EXP_PIN 4
#define BUZZER_MODULE_PIN 19

BuzzerModuleGpioExpPi buzzer_module(BUZZER_MODULE_PIN);

void buzzerModuleCallback(const std_msgs::Bool& msg) {
	if (msg.data) {
		//ROS_INFO("Buzzer ON!\n");
		/*if (threadPlayTone.get_id() == false) {
			//buzzer_module.tonePlays = true;
			using namespace boost;
			thread threadPlayTone = thread(&BuzzerModuleGpioExpPi::playTone, buzzer_module);
		}*/
		//threadPlayTone.join();
		buzzer_module.startTone();
		//buzzer_module.on();
	} else {
		//ROS_INFO("Buzzer OFF!\n");
		buzzer_module.off();
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "buzzer_module");
	ros::NodeHandle node;
	ros::Subscriber buzzer_module_sub = node.subscribe("/kvantbot/buzzer_module/state", 1, &buzzerModuleCallback);
	ros::spin();
	return 0;
}