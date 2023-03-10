#ifndef KVANTBOT_TELEOP_H_
#define KVANTBOT_TELEOP_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define MI_AXIS_STICK_LEFT_LEFTWARDS 0
#define MI_AXIS_STICK_LEFT_UPWARDS 1
#define MI_AXIS_STICK_RIGHT_LEFTWARDS 2
#define MI_AXIS_STICK_RIGHT_UPWARDS 3

class KvantbotTeleop {
	public:
		KvantbotTeleop(ros::NodeHandle private_node);
	private:
		ros::NodeHandle _node;
		ros::NodeHandle _private_node;
		ros::Subscriber _joy_sub;
		ros::Publisher _cmd_vel_pub;

		bool _last_zero_twist = true; 
		double _linear_speed_scale;
		double _angular_speed_scale;
		
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
};

KvantbotTeleop::KvantbotTeleop(ros::NodeHandle private_node) :
	_private_node(private_node) {
	_private_node.param<double>("linear_speed_scale", _linear_speed_scale, 0.0);
	_private_node.param<double>("angular_speed_scale", _angular_speed_scale, 0.0);
	_cmd_vel_pub = _node.advertise<geometry_msgs::Twist>("/mobile_kvantbot/cmd_vel", 1);
	_joy_sub = _node.subscribe<sensor_msgs::Joy>("joy", 10, &KvantbotTeleop::joyCallback, this);
	ROS_INFO("Kvantbot teleop node: Start");
}

void KvantbotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	geometry_msgs::Twist twist;

	double twist_linear_x_vel = _linear_speed_scale * joy->axes[MI_AXIS_STICK_LEFT_UPWARDS];
	double twist_angular_z_vel = _angular_speed_scale * joy->axes[MI_AXIS_STICK_LEFT_LEFTWARDS];

	twist.linear.x = twist_linear_x_vel;
	twist.angular.z = twist_angular_z_vel;

	if (twist_linear_x_vel == 0 && twist_angular_z_vel == 0) {
		if (_last_zero_twist == false) {
			_cmd_vel_pub.publish(twist);
			_last_zero_twist = true;
		}
	} else {
		_last_zero_twist = false;
		_cmd_vel_pub.publish(twist);
	}
}

#endif // KVANTBOT_TELEOP_H_
