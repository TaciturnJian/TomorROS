#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_tutorial_node");
    ros::NodeHandle n;
    ros::Rate rate(1);

    while (n.ok())
    {
        ROS_INFO("spin once");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}