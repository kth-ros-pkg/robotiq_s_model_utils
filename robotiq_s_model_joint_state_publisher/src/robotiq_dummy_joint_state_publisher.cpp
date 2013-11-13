// publishes 0 degrees to the robotiq hand joint states

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


int main(int argc, char** argv)
{
	/// initialize ROS, specify name of node
	ros::init(argc, argv, "robotiq_dummy_joint_state_publisher");

	ros::NodeHandle n_ = ros::NodeHandle("~");
	ros::Publisher topicPub_JointState_;

	topicPub_JointState_ = n_.advertise<sensor_msgs::JointState> ("/joint_states", 1);

	sensor_msgs::JointState joint_state_msg;

	joint_state_msg.name.resize(11);
	joint_state_msg.name[0] = "robotiq_finger_1_joint_1";
	joint_state_msg.name[1] = "robotiq_finger_1_joint_2";
	joint_state_msg.name[2] = "robotiq_finger_1_joint_3";
	joint_state_msg.name[3] = "robotiq_finger_2_joint_1";
	joint_state_msg.name[4] = "robotiq_finger_2_joint_2";
	joint_state_msg.name[5] = "robotiq_finger_2_joint_3";
	joint_state_msg.name[6] = "robotiq_finger_middle_joint_1";
	joint_state_msg.name[7] = "robotiq_finger_middle_joint_2";
	joint_state_msg.name[8] = "robotiq_finger_middle_joint_3";
	joint_state_msg.name[9] = "robotiq_palm_finger_1_joint";
	joint_state_msg.name[10] = "robotiq_palm_finger_2_joint";

	joint_state_msg.position = std::vector<double>(11, 0.0);
	joint_state_msg.velocity = std::vector<double>(11, 0.0);
	joint_state_msg.effort = std::vector<double>(11, 0.0);

	ros::Rate loop_rate(10.0);

	while(n_.ok())
	{
		joint_state_msg.header.stamp = ros::Time::now();

		topicPub_JointState_.publish(joint_state_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;
}
