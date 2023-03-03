#include "line_sensor.hpp"
#include <std_msgs/Int16.h>
#include <std_msgs/Time.h>
#include <chrono>

// Пинs датчика линии через плату расширения Troyka Hat
#define L_LINE_SEN_EXP_PIN 6
#define R_LINE_SEN_EXP_PIN 7

#define L_LINE_SEN_REF_RAW_BLACK 2424
#define L_LINE_SEN_REF_RAW_WHITE 282
#define R_LINE_SEN_REF_RAW_BLACK 2218
#define R_LINE_SEN_REF_RAW_WHITE 226

typedef boost::chrono::steady_clock time_source;

class LineSensorsPair {
    public:
        LineSensorsPair(double update_rate);

    private:
        ros::NodeHandle node;

        ros::Publisher left_line_sensor_ref_raw_pub;
		ros::Publisher right_line_sensor_ref_raw_pub;
		ros::Publisher left_line_sensor_ref_pub;
		ros::Publisher right_line_sensor_ref_pub;

        ros::Timer line_sensors_timer;

		std_msgs::Int16 left_line_sensor_ref_raw_value_msg;
        std_msgs::Int16 right_line_sensor_ref_raw_value_msg;
		std_msgs::Int16 left_line_sensor_ref_value_msg;
        std_msgs::Int16 right_line_sensor_ref_value_msg;

        LineSensorGpioExpPi line_sensor_left;
		LineSensorGpioExpPi line_sensor_right;

        int left_line_sensor_ref_raw_value;
		int right_line_sensor_ref_raw_value;
		int left_line_sensor_ref_value;
		int right_line_sensor_ref_value;

        time_source::time_point last_time;

        void lineSensorsCallback(const ros::TimerEvent& event);
		uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
		uint16_t constrain(uint16_t x, uint16_t a, uint16_t b);
};

LineSensorsPair::LineSensorsPair(double update_rate): line_sensor_left(L_LINE_SEN_EXP_PIN), line_sensor_right(R_LINE_SEN_EXP_PIN) {
	left_line_sensor_ref_raw_pub = node.advertise<std_msgs::Int16>("/kvantbot/left_line_sensor/ref_raw", 1);
	right_line_sensor_ref_raw_pub = node.advertise<std_msgs::Int16>("/kvantbot/right_line_sensor/ref_raw", 1);
	left_line_sensor_ref_pub = node.advertise<std_msgs::Int16>("/kvantbot/left_line_sensor/ref", 1);
	right_line_sensor_ref_pub = node.advertise<std_msgs::Int16>("/kvantbot/right_line_sensor/ref", 1);
	line_sensors_timer = node.createTimer(ros::Duration(update_rate), &LineSensorsPair::lineSensorsCallback, this);
}

void LineSensorsPair::lineSensorsCallback(const ros::TimerEvent& event) {
	time_source::time_point this_time = time_source::now();
	boost::chrono::duration<double> elapsed_duration = this_time - last_time;
	ros::Duration elapsed(elapsed_duration.count());
	last_time = this_time;

	left_line_sensor_ref_raw_value = line_sensor_left.getRawValue();
	right_line_sensor_ref_raw_value = line_sensor_right.getRawValue();

	left_line_sensor_ref_raw_value_msg.data = left_line_sensor_ref_raw_value;
	right_line_sensor_ref_raw_value_msg.data = right_line_sensor_ref_raw_value;

	left_line_sensor_ref_raw_pub.publish(left_line_sensor_ref_raw_value_msg);
	right_line_sensor_ref_raw_pub.publish(right_line_sensor_ref_raw_value_msg);
	
	// Программно калибруем и нормализуем с сырых значений
	left_line_sensor_ref_value = LineSensorsPair::map(left_line_sensor_ref_raw_value, L_LINE_SEN_REF_RAW_BLACK, L_LINE_SEN_REF_RAW_WHITE, 0, 255);
	right_line_sensor_ref_value = LineSensorsPair::map(right_line_sensor_ref_raw_value, R_LINE_SEN_REF_RAW_BLACK, R_LINE_SEN_REF_RAW_WHITE, 0, 255);

	// Ограничеваем значение
	left_line_sensor_ref_value = LineSensorsPair::constrain(left_line_sensor_ref_value, 0, 255);
	right_line_sensor_ref_value = LineSensorsPair::constrain(right_line_sensor_ref_value, 0, 255);

	// Записываем значение в std_msgs
	left_line_sensor_ref_value_msg.data = left_line_sensor_ref_value;
	right_line_sensor_ref_value_msg.data = right_line_sensor_ref_value;

	// Паблишим значение
	left_line_sensor_ref_pub.publish(left_line_sensor_ref_value_msg);
	right_line_sensor_ref_pub.publish(right_line_sensor_ref_value_msg);
	
}

uint16_t LineSensorsPair::map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t LineSensorsPair::constrain(uint16_t x, uint16_t a, uint16_t b) {
	return (x < a)? a : (x > b)? b : x;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "line_sensors");
	LineSensorsPair line_sensors_pair(0.01);
	ros::spin();
	return 0;
}