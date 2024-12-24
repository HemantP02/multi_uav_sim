#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h> // Status message type
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "setpoint_node1");
    ros::NodeHandle nh("~");

    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav1/mavros/setpoint_position/local", 10);
    ros::Publisher status_pub = nh.advertise<std_msgs::Bool>("uav1/loop_complete", 10);
    ros::Rate rate(20.0);

    // Define waypoints
    std::vector<geometry_msgs::PoseStamped> waypoints(4);
    for (size_t i = 0; i < 4; ++i) {
        waypoints[i].header.frame_id = "map";
        waypoints[i].pose.position.z = 5; // Maintain constant altitude
    }
    waypoints[0].pose.position.x = 0; waypoints[0].pose.position.y = 0;
    waypoints[1].pose.position.x = 5; waypoints[1].pose.position.y = 0;
    waypoints[2].pose.position.x = 5; waypoints[2].pose.position.y = 5;
    waypoints[3].pose.position.x = 0; waypoints[3].pose.position.y = 5;

    size_t current_wp = 0;
    size_t next_wp = 1;

    double travel_time = 5.0; 
    int num_steps = static_cast<int>(travel_time / rate.expectedCycleTime().toSec());
    int step_count = 0;

    geometry_msgs::PoseStamped current_pose = waypoints[current_wp];
    geometry_msgs::PoseStamped target_pose = waypoints[next_wp];

    int loop_count = 0; 
    bool stop_movement = false; 

    while (ros::ok()) {
        geometry_msgs::PoseStamped interpolated_pose;
        interpolated_pose.header.frame_id = "map";
        interpolated_pose.header.stamp = ros::Time::now();

        if (!stop_movement) {
            double alpha = static_cast<double>(step_count) / num_steps;
            interpolated_pose.pose.position.x = (1 - alpha) * current_pose.pose.position.x + alpha * target_pose.pose.position.x;
            interpolated_pose.pose.position.y = (1 - alpha) * current_pose.pose.position.y + alpha * target_pose.pose.position.y;
            interpolated_pose.pose.position.z = current_pose.pose.position.z;

            setpoint_pub.publish(interpolated_pose);

            step_count++;
            if (step_count >= num_steps) {
                current_wp = next_wp;
                next_wp = (next_wp + 1) % waypoints.size();
                step_count = 0;

                current_pose = waypoints[current_wp];
                target_pose = waypoints[next_wp];

                if (current_wp == 0) {
                    loop_count++;
                    if (loop_count >= 2) {
                        stop_movement = true;

                        // Publish completion status
                        std_msgs::Bool loop_complete_msg;
                        loop_complete_msg.data = true;
                        status_pub.publish(loop_complete_msg);
                    }
                }
            }
        } else {
            interpolated_pose.pose = waypoints[0].pose;
            interpolated_pose.header.stamp = ros::Time::now();
            setpoint_pub.publish(interpolated_pose);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

