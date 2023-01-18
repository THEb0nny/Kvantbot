#include "dc_motor_wiring_pi.hpp"
#include <std_msgs/Float64.h>

#define MOTOR_L_PIN_IN1 17 // Wiring pi 2 = BCM 27
#define MOTOR_L_PIN_IN2 27 // Wiring pi 0 = BCM 17
#define MOTOR_L_PIN_PWM 18 // Wiring pi 1 = BCM 18

#define MOTOR_R_PIN_IN1 6 // Wiring pi 22 = BCM 6
#define MOTOR_R_PIN_IN2 12 // Wiring pi 26 = BCM 12
#define MOTOR_R_PIN_PWM 13 // Wiring pi 23 = BCM 13

DCMotorWiringPi left_dc_motor(MOTOR_L_PIN_IN1, MOTOR_L_PIN_IN2, MOTOR_L_PIN_PWM);
DCMotorWiringPi right_dc_motor(MOTOR_R_PIN_IN1, MOTOR_R_PIN_IN2, MOTOR_R_PIN_PWM);

void leftMotorCallback(const std_msgs::Float64& msg) {
	int16_t pwm = msg.data * 100;
	if (pwm > 0) {
		left_dc_motor.ccw(abs(pwm));
	} else if (pwm < 0) {
		left_dc_motor.cw(abs(pwm));
	} else if (pwm == 0) {
		left_dc_motor.stop();
	}
}

void rightMotorCallback(const std_msgs::Float64& msg) {
	int16_t pwm = msg.data * 100;
	if (pwm > 0) {
		right_dc_motor.ccw(abs(pwm));
	} else if (pwm < 0) {
		right_dc_motor.cw(abs(pwm));
	} else if (pwm == 0) {
		right_dc_motor.stop();
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "dc_motors");
	ros::NodeHandle node;
	// https://roboticsbackend.com/ros-asyncspinner-example/
	//ros::AsyncSpinner spinner(0);
    //spinner.start();
	ros::Subscriber left_motor_target_vel_sub = node.subscribe("/kvantbot/left_motor/pwm", 1, &leftMotorCallback);
	ros::Subscriber right_motor_target_vel_sub = node.subscribe("/kvantbot/right_motor/pwm", 1, &rightMotorCallback);
	//ros::waitForShutdown();
	ros::spin();
	return 0;
}
