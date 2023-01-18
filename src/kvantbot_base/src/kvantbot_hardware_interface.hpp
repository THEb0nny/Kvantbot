#ifndef KVANTBOT_HARDWARE_INTERFACE_H_
#define KVANTBOT_HARDWARE_INTERFACE_H_

#include <boost/assign/list_of.hpp>
#include <sstream>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <ros/console.h>

class KvantbotHardwareInterface : public hardware_interface::RobotHW {
	public:
		KvantbotHardwareInterface(ros::NodeHandle node, ros::NodeHandle private_node, double target_max_motor_angular_speed);

		void updateJointsFromHardware(const ros::Duration& period);
		void writeCommandsToHardware();

	private:
		ros::NodeHandle _node;
		ros::NodeHandle _private_node;

		hardware_interface::JointStateInterface _joint_state_interface;
		hardware_interface::VelocityJointInterface _velocity_joint_interface;

		ros::Subscriber _left_motor_angle_sub;
		ros::Subscriber _right_motor_angle_sub;
		ros::Publisher _left_motor_vel_pub;
		ros::Publisher _right_motor_vel_pub;

		struct Joint {
			double position;
			double position_offset;
			double velocity;
			double effort;
			double velocity_command;

			Joint()
				: position(0)
				, velocity(0)
				, effort(0)
				, velocity_command(0) {}
		} _joints[2];

		double _left_motor_angle;
		double _right_motor_angle;
		double _max_motor_angular_speed;

		void registerControlInterfaces();
		void leftMotorAngleCallback(const std_msgs::Float64& msg);
		void rightMotorAngleCallback(const std_msgs::Float64& msg);
		void limitDifferentialSpeed(double& diff_speed_left_side, double& diff_speed_right_side);
};

KvantbotHardwareInterface::KvantbotHardwareInterface(ros::NodeHandle node, ros::NodeHandle private_node, double target_max_motor_angular_speed)
	: _node(node)
	, _private_node(private_node)
	, _max_motor_angular_speed(target_max_motor_angular_speed) {
	registerControlInterfaces();

	_left_motor_vel_pub = _node.advertise<std_msgs::Float64>("/kvantbot/left_motor/target_velocity", 1);
	_right_motor_vel_pub = _node.advertise<std_msgs::Float64>("/kvantbot/right_motor/target_velocity", 1);
	_left_motor_angle_sub = _node.subscribe("kvantbot/left_motor/angle", 1, &KvantbotHardwareInterface::leftMotorAngleCallback, this);
	_right_motor_angle_sub = _node.subscribe("kvantbot/right_motor/angle", 1, &KvantbotHardwareInterface::rightMotorAngleCallback, this);
}

void KvantbotHardwareInterface::writeCommandsToHardware() {
	double diff_angle_speed_left = _joints[0].velocity_command;
	double diff_angle_speed_right = _joints[1].velocity_command;

	limitDifferentialSpeed(diff_angle_speed_left, diff_angle_speed_right);

	std_msgs::Float64 left_motor_vel_msg;
	std_msgs::Float64 right_motor_vel_msg;

	left_motor_vel_msg.data = diff_angle_speed_left;
	right_motor_vel_msg.data = diff_angle_speed_right;

	_left_motor_vel_pub.publish(left_motor_vel_msg);
	_right_motor_vel_pub.publish(right_motor_vel_msg);
}

void KvantbotHardwareInterface::updateJointsFromHardware(const ros::Duration& period) {
	double delta_left_motor = _left_motor_angle - _joints[0].position - _joints[0].position_offset;
	double delta_right_motor = _right_motor_angle - _joints[1].position - _joints[1].position_offset;

	if (std::abs(delta_left_motor) < 1) {
		_joints[0].position += delta_left_motor;
		_joints[0].velocity = delta_left_motor / period.toSec();
	} else {
		_joints[0].position_offset += delta_left_motor;
	}

	if (std::abs(delta_right_motor) < 1) {
		_joints[1].position += delta_right_motor;
		_joints[1].velocity = delta_right_motor / period.toSec();
	} else {
		_joints[1].position_offset += delta_right_motor;
	}
}

void KvantbotHardwareInterface::registerControlInterfaces() {
	ros::V_string joint_names = boost::assign::list_of("left_wheel_to_base")("right_wheel_to_base");

	for (unsigned int i = 0; i < joint_names.size(); i++) {
		hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &_joints[i].position, &_joints[i].velocity, &_joints[i].effort);
		_joint_state_interface.registerHandle(joint_state_handle);

		hardware_interface::JointHandle joint_handle(joint_state_handle, &_joints[i].velocity_command);
		_velocity_joint_interface.registerHandle(joint_handle);
	}
	registerInterface(&_joint_state_interface);
	registerInterface(&_velocity_joint_interface);
}

void KvantbotHardwareInterface::leftMotorAngleCallback(const std_msgs::Float64& msg) {
	_left_motor_angle = msg.data;
}

void KvantbotHardwareInterface::rightMotorAngleCallback(const std_msgs::Float64& msg) {
	_right_motor_angle = msg.data;
}

void KvantbotHardwareInterface::limitDifferentialSpeed(double& diff_speed_left_side, double& diff_speed_right_side) {
	double large_speed = std::max(std::abs(diff_speed_left_side), std::abs(diff_speed_right_side));
	if (large_speed > _max_motor_angular_speed) {
		diff_speed_left_side *=  _max_motor_angular_speed / large_speed;
		diff_speed_right_side *=  _max_motor_angular_speed / large_speed;
	}
}

#endif // KVANTBOT_HARDWARE_INTERFACE_H_
