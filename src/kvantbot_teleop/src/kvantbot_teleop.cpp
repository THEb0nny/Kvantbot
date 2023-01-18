#include "kvantbot_teleop.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "kvantbot_teleop");
	ros::NodeHandle private_node("~");
	KvantbotTeleop kvantbotTeleop(private_node);
	ros::spin();
}