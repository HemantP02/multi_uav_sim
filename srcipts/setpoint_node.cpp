#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "setpoint_node");
    ros::NodeHandle nh("~"); // Private node handle to support namespaces

    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    ros::Rate rate(20.0);

    while (ros::ok()) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;  // Set altitude to 2 meters

        setpoint_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

