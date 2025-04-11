#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/forwards.h>

// Global variable to store the latest odometry message
nav_msgs::Odometry::ConstPtr latest_msg;

// Callback to update the latest odometry data
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  latest_msg = msg; // Store the latest message
}

// Timer callback to broadcast the transform at a lower rate
void timerCallback(const ros::TimerEvent&) {
  static tf2_ros::TransformBroadcaster br;
  
  if (!latest_msg) {
    // If we haven't received any odometry messages yet, do nothing
    return;
  }

  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";  
  transformStamped.child_frame_id = "base_link";  

  // Set translation based on the latest odometry position
  transformStamped.transform.translation.x = latest_msg->pose.pose.position.x;
  transformStamped.transform.translation.y = latest_msg->pose.pose.position.y;
  transformStamped.transform.translation.z = latest_msg->pose.pose.position.z;

  // Set rotation based on the latest odometry orientation
  transformStamped.transform.rotation = latest_msg->pose.pose.orientation;

  // Broadcast transform
  br.sendTransform(transformStamped);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_to_base_link_broadcaster");
  ros::NodeHandle node;

  // Subscribe to the odometry topic
  ros::Subscriber sub = node.subscribe("/odom", 1, &poseCallback);

  // Set up a timer to broadcast transform at 10 Hz
  ros::Timer timer = node.createTimer(ros::Duration(0.05), timerCallback); // 10 Hz rate (0.1 seconds interval)

  ros::spin();
  return 0;
}
