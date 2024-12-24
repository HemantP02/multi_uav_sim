// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "setpoint_node");
//     ros::NodeHandle nh("~"); // Private node handle to support namespaces

//     ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav2/mavros/setpoint_position/local", 10);

//     ros::Rate rate(20.0);

//     while (ros::ok()) {
//         geometry_msgs::PoseStamped pose;
//         pose.header.stamp = ros::Time::now();
//         pose.pose.position.x = 0;
//         pose.pose.position.y = 0;
//         pose.pose.position.z = 5;  // Set altitude to 2 meters

//         setpoint_pub.publish(pose);
//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

bool start_movement = false;

void statusCallback(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data) {
        start_movement = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "setpoint_node2");
    ros::NodeHandle nh("~");

    ros::Subscriber status_sub = nh.subscribe("/setpoint_node1/uav1/loop_complete", 10, statusCallback);
    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav2/mavros/setpoint_position/local", 10);
    ros::Rate rate(2.0);

    // Define waypoint 1
    geometry_msgs::PoseStamped waypoint1;
    waypoint1.header.frame_id = "map";
    waypoint1.pose.position.x = -2;
    waypoint1.pose.position.y = 0;
    waypoint1.pose.position.z = 5;

    while (ros::ok()) {
        if (start_movement) {
            waypoint1.header.stamp = ros::Time::now();
            setpoint_pub.publish(waypoint1);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
