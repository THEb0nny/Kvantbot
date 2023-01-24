#include "encoder_wiring_pi.hpp"
#include <std_msgs/Float64.h>
#include <std_msgs/Time.h>
#include <chrono>

typedef boost::chrono::steady_clock time_source;

class EncodersPair {
	public:
		EncodersPair(double update_rate);

	private:
		ros::NodeHandle node;

		ros::Publisher left_motor_angle_pub;
		ros::Publisher right_motor_angle_pub;
		ros::Publisher left_motor_velocity_pub;
		ros::Publisher right_motor_velocity_pub;

		ros::Timer encoders_timer;

		std_msgs::Float64 left_motor_angle_msg;
		std_msgs::Float64 right_motor_angle_msg;
		std_msgs::Float64 left_motor_velocity_msg;
		std_msgs::Float64 right_motor_velocity_msg;

		EncoderWiringPi encoder_left;
		EncoderWiringPi encoder_right;

		double left_motor_angle;
		double right_motor_angle;
		double left_motor_velocity;
		double right_motor_velocity;
		double left_motor_position;
		double right_motor_position;

		time_source::time_point last_time;

		void encodersCallback(const ros::TimerEvent& event);
};

EncodersPair::EncodersPair(double update_rate) :
	encoder_left(ENCODER_1_PIN_A, ENCODER_1_PIN_B, &EncoderWiringPiISR::encoderISR1, &EncoderWiringPiISR::encoder_position_1),
	encoder_right(ENCODER_2_PIN_A, ENCODER_2_PIN_B, &EncoderWiringPiISR::encoderISR2, &EncoderWiringPiISR::encoder_position_2) {
	left_motor_angle_pub = node.advertise<std_msgs::Float64>("/kvantbot/left_motor/angle", 1);
	right_motor_angle_pub = node.advertise<std_msgs::Float64>("/kvantbot/right_motor/angle", 1);
	left_motor_velocity_pub = node.advertise<std_msgs::Float64>("/kvantbot/left_motor/current_velocity", 1);
	right_motor_velocity_pub = node.advertise<std_msgs::Float64>("/kvantbot/right_motor/current_velocity", 1);

	encoders_timer = node.createTimer(ros::Duration(update_rate), &EncodersPair::encodersCallback, this);
}

void EncodersPair::encodersCallback(const ros::TimerEvent& event) {
	time_source::time_point this_time = time_source::now();
	boost::chrono::duration<double> elapsed_duration = this_time - last_time;
	ros::Duration elapsed(elapsed_duration.count());
	last_time = this_time;

	left_motor_angle = -1 * encoder_left.getAngle(); // Inverted
	right_motor_angle = 1 * encoder_right.getAngle();

	left_motor_angle_msg.data = left_motor_angle;
	right_motor_angle_msg.data = right_motor_angle;

	left_motor_angle_pub.publish(left_motor_angle_msg);
	right_motor_angle_pub.publish(right_motor_angle_msg);

	double delta_left_motor = left_motor_angle - left_motor_position;
	double delta_right_motor = right_motor_angle - right_motor_position;

	left_motor_position += delta_left_motor;
	left_motor_velocity = delta_left_motor / elapsed.toSec();

	right_motor_position += delta_right_motor;
	right_motor_velocity = delta_right_motor / elapsed.toSec();
 
	left_motor_velocity_msg.data = left_motor_velocity;
	right_motor_velocity_msg.data = right_motor_velocity;

	left_motor_velocity_pub.publish(left_motor_velocity_msg);
	right_motor_velocity_pub.publish(right_motor_velocity_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "encoders");
	//ros::AsyncSpinner spinner(0);
    //spinner.start();
	EncodersPair encoders_pair(0.01);
	//ros::waitForShutdown();
	ros::spin();
	return 0;
}