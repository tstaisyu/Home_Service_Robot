#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <cmath>

double robot_x;
double robot_y;


//Source: https://knowledge.udacity.com/questions/843366
//About callback function

void pose_callback(const nav_msgs::Odometry& odom) {
    robot_x = odom.pose.pose.position.y;
    robot_y = -odom.pose.pose.position.x;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers_custom");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("odom", 10, pose_callback);
  bool picked = false;
  bool finish = false;
  double odom_threshold1 = 0.4;
  double odom_threshold2 = 0.4;

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers_custom";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 6.5;
    marker.pose.position.y = 3.3;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();


    double dist1 = sqrt(pow((robot_x - marker.pose.position.x), 2) + pow((robot_y - marker.pose.position.y), 2));

    if (!picked) {
        marker_pub.publish(marker);
        ROS_INFO("Location : %f, %f / Distance : %f", robot_x, robot_y, dist1);
        sleep(1);
        if (dist1 < odom_threshold1) {
            sleep(2);
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker);
            ROS_INFO("Robot is picking up Marker");
            picked = true;
            sleep(3);
        }
    }

    marker.pose.position.x = 6.7;
    marker.pose.position.y = -7.1;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration();
    double dist2 = sqrt(pow((robot_x - marker.pose.position.x), 2) + pow((robot_y - marker.pose.position.y), 2));
    
    if (picked && !finish) {
        ROS_INFO("Location : %f, %f / Distance : %f", robot_x, robot_y, dist2);
        sleep(1);
        if (dist2 < odom_threshold2) {
            sleep(2);
            marker.action = visualization_msgs::Marker::ADD;
            marker_pub.publish(marker);
            ROS_INFO("Marker was succesfully dropped off");
            finish = true;
            sleep(3);
        }
    }
    ros::spinOnce();
  }
}
