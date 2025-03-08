/**
 * @file offb_node_lidar.cpp
 * @brief Offboard control example node using LiDAR for height estimation, written with MAVROS.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf/transform_listener.h>

// Global variables
mavros_msgs::State current_state;
double lidar_height = 0.0;

// Callback functions
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void lidar_cb(const sensor_msgs::Range::ConstPtr& msg) {
    lidar_height = msg->range; // Update global height from LiDAR data
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offboard_node_lidar");
    ros::NodeHandle nh;

    // ROS Subscribers and Publishers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/hrlv_ez4_pub", 1, lidar_cb);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // Setpoint publishing rate
    ros::Rate rate(50.0);

    // Wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // Initialize position target
    mavros_msgs::PositionTarget current_goal;
    current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_goal.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                             mavros_msgs::PositionTarget::IGNORE_VY |
                             mavros_msgs::PositionTarget::IGNORE_VZ |
                             mavros_msgs::PositionTarget::IGNORE_AFX |
                             mavros_msgs::PositionTarget::IGNORE_AFY |
                             mavros_msgs::PositionTarget::IGNORE_AFZ |
                             mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    current_goal.position.x = 0.0;
    current_goal.position.y = 0.0;
    current_goal.position.z = lidar_height; // Initial height based on LiDAR
    current_goal.yaw = 0.0;

    // Send a few setpoints before starting OFFBOARD
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(current_goal);
        ros::spinOnce();
        rate.sleep();
    }

    // Service request structures
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        // Enable OFFBOARD mode
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            // Arm the UAV
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // Update height using LiDAR data
        current_goal.position.z = lidar_height;

        // Publish the position target
        local_pos_pub.publish(current_goal);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

