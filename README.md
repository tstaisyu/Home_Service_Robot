# Project: Home Service Robot

I have created two different projects using each of turtlebot and myrobot.

## turtlebot

<img src=Summary_turtlebot.gif width=620 />

## MyRobot

<img src=Summary_My_Robot.gif width=832 />


## Setup

```sh
$ mkdir catkin_ws && cd catkin_ws
$ git clone --recursive https://github.com/tstaisyu/Home_Service_Robot.git src
$ catkin_make
$ source devel/setup.bash
```

### test_slam.sh

<img src=test_slam.gif width=832 />

```sh
$ ./src/scripts/test_slam.sh
```

### test_navigation.sh

<img src=test_navigation.gif width=655 />

```sh
$ ./src/scripts/test_navigation.sh
```

### pick_objects.sh

<img src=pick_objects.gif width=655 />

```sh
$ ./src/scripts/pick_objects.sh
```

### To start Home Service with turtlebot

```sh
$ ./src/scripts/home_service.sh
```

### To start Home Service with MyRobot

```sh
$ ./src/scripts/home_service_custom.sh
```

## Code Explaination

### pick_objects.cpp

```sh
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
```
Include the necessary libraries and define an action client to send goal requests.

```sh
int main(int argc, char** argv){

  ros::init(argc, argv, "pick_objects");

  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
```
Initialize "pick_objects" node.
Tell the action client that we want to spin a thread by default.
Wait for move_base action server to come up.

```sh
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose.position.x = 3.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  
  ac.waitForResult();
  ```
Instantiate "goal" message, define the values, and send them.
Wait for the results.

```sh
  goal.target_pose.pose.position.x = 1.2;
  goal.target_pose.pose.position.y = -1.7;
  goal.target_pose.pose.orientation.w = 1.0;

  sleep(5);

  ROS_INFO("Sending dropoff");
  ac.sendGoal(goal);
  
  ac.waitForResult();
```
Again, define the values.
Wait 5 second for picking up.
Then, send the dropoff point and wait for the results.

```sh
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved to goal and then reached dropoff");
  else
    ROS_INFO("The base failed to reach dropoff for some reason");

  return 0;
}
```
Finally, check if the robot reached its goal.


### add_markers.cpp

```sh
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <cmath>
```
Define the necessary libraries.

```sh
double robot_x;
double robot_y;

void pose_callback(const nav_msgs::Odometry& odom) {
    robot_x = odom.pose.pose.position.x;
    robot_y = odom.pose.pose.position.y;
}
```
Define the robot (x, y) location.
Create the callback function.

```sh
int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("odom", 10, pose_callback);
```
Initialize "add_markers" node.
Create a publisher to publish marker visual information.
Create a subscriber to get odom.


```sh
  bool picked = false;
  bool finish = false;
  double odom_threshold1 = 0.5;
  double odom_threshold2 = 0.7;

  uint32_t shape = visualization_msgs::Marker::CUBE;
```
Create a flag of picking and work completion
Create thresholds for determining distance to objects.
Set the initial shape type.


```sh
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
 
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "add_markers";
    marker.id = 0;

    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 3.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
```
Instantiate a marker and setup the values.

```sh
    double dist1 = sqrt(pow((robot_x - marker.pose.position.x), 2) + pow((robot_y - marker.pose.position.y), 2));
```
Create the variable of distance between a robot and a marker object.

```sh
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
```
Outputs x,y coordinates and distance of the robot every second while moving to the marker.
If the distance falls below the threshold, it is considered an arrival.
Hide the marker after 2 seconds, output information, set the picked flag true, and wait 3 more seconds for the work to be completed.

```sh
    marker.pose.position.x = 1.2;
    marker.pose.position.y = -1.7;
    marker.lifetime = ros::Duration();
    double dist2 = sqrt(pow((robot_x - marker.pose.position.x), 2) + pow((robot_y - marker.pose.position.y), 2));
```
Similarly, define dropoff point and distance.

```sh    
    if (picked && !finish) {
        ROS_INFO("Location : %f, %f / Distance : %f", robot_x, robot_y, dist2);
        sleep(1);
        if (dist2 < odom_threshold2) {
            sleep(5);
            marker.action = visualization_msgs::Marker::ADD;
            marker_pub.publish(marker);
            ROS_INFO("Marker was succesfully dropped off");
            finish = true;
        }
    }
    ros::spinOnce();
  }
}
```
Again, outputs x,y coordinates and distance of the robot every second while moving to the dropoff point.
If the distance falls below the threshold and the robot arrives, wait 5 seconds for the dropoff to be completed and show a marker.



