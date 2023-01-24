#include "line_sensor.hpp"
#include <std_msgs/Float64.h>
#include <std_msgs/Time.h>
#include <chrono>

#define LINE_SEN_EXP_PIN 7 // Пин датчика линии через плату расширения Troyka Hat

typedef boost::chrono::steady_clock time_source;

class LineSensorsPair {
    public:
        LineSensorsPair(double update_rate);

    private:
        ros::NodeHandle node;

        ros::Publisher line_sensor_pub;

        ros::Timer line_sensors_timer;

        std_msgs::Float64 line_sensor_value_msg;

        LineSensorWiringPi line_sensor;

        int line_sensor_value;

        time_source::time_point last_time;

        void lineSensorsCallback(const ros::TimerEvent& event);
};

LineSensorsPair::LineSensorsPair(double update_rate) :
	line_sensor(LINE_SEN_EXP_PIN) {
	line_sensor_pub = node.advertise<std_msgs::Float64>("/kvantbot/line_sensor/value", 1);

	line_sensors_timer = node.createTimer(ros::Duration(update_rate), &LineSensorsPair::lineSensorsCallback, this);
}

void LineSensorsPair::lineSensorsCallback(const ros::TimerEvent& event) {
	time_source::time_point this_time = time_source::now();
	boost::chrono::duration<double> elapsed_duration = this_time - last_time;
	ros::Duration elapsed(elapsed_duration.count());
	last_time = this_time;

	line_sensor_value = line_sensor.getRawValue();
	line_sensor_value_msg.data = line_sensor_value;
	line_sensor_pub.publish(line_sensor_value_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "line_sensors");
	LineSensorsPair line_sensors_pair(0.01);
	ros::spin();
	return 0;
}