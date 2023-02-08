#include "button_module.hpp"
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <chrono>

// Пинs датчика линии через плату расширения Troyka Hat
#define BTN_EXP_PIN 5

typedef boost::chrono::steady_clock time_source;

class ButtonModule {
    public:
        ButtonModule(double update_rate);

    private:
        ros::NodeHandle node;

        ros::Publisher button_module_pub;

        ros::Timer btn_module_timer;

		std_msgs::Bool btn_module_value_msg;

        BtnModuleGpioExpPi btn_module;

        bool btn_module_value;

        time_source::time_point last_time;

        void btnModuleCallback(const ros::TimerEvent& event);
};

ButtonModule::ButtonModule(double update_rate): btn_module(BTN_EXP_PIN) {
	button_module_pub = node.advertise<std_msgs::Bool>("/kvantbot/button_module/state", 1);
	btn_module_timer = node.createTimer(ros::Duration(update_rate), &ButtonModule::btnModuleCallback, this);
}

void ButtonModule::btnModuleCallback(const ros::TimerEvent& event) {
	time_source::time_point this_time = time_source::now();
	boost::chrono::duration<double> elapsed_duration = this_time - last_time;
	ros::Duration elapsed(elapsed_duration.count());
	last_time = this_time;

	btn_module_value = !btn_module.read();

	btn_module_value_msg.data = btn_module_value;

	button_module_pub.publish(btn_module_value_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "button_module");
	ButtonModule btn_module(0.01);
	ros::spin();
	return 0;
}