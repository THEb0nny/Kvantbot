#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "contest");
	ros::NodeHandle node;
	//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(10);

	std::vector<double> vel(2);

	while (ros::ok) {
		//double error = left_sensor - right_sensor;
		//double U = error * Kp;
		//vel[0] = U;
		//vel[1] = 0;
		ros::spinOnce;
	}
	return 0;
}