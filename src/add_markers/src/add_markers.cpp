#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "pick_objects/go_to_xy.h"

ros::Publisher *marker_pub_ptr;
ros::ServiceClient *client_ptr;

struct location{
  float x;
  float y;
};

location pickup = {7, -1};
location dropoff = {-12, -5.5};
enum State { INITIAL, PICKUP, DROPOFF };
State state = INITIAL;

void add_marker(visualization_msgs::Marker &marker, 
  float x, float y) {
  uint32_t shape = visualization_msgs::Marker::CUBE;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.6;
  marker.scale.z = 0.7;
  marker.color.r = 0.5f;
  marker.color.g = 0.0f;
  marker.color.b = 0.7f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
}

void go_to_location(location loc) {
  pick_objects::go_to_xy xy_location;
  xy_location.request.x = loc.x;
  xy_location.request.y = loc.y;
  client_ptr->call(xy_location);
}

void process_odom_callback(const nav_msgs::Odometry odom)
{
  if(state == PICKUP) {
    sleep(5);
    ROS_INFO("picking up");
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "add_markers";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub_ptr->publish(marker);
    ROS_INFO("marker deleted");
    state = DROPOFF;
  
    go_to_location(dropoff);
  }
  if(state == DROPOFF) {
    ROS_INFO("dropped off");
    visualization_msgs::Marker marker;
    add_marker(marker, 0, 0);
    marker_pub_ptr->publish(marker);
    ROS_INFO("marker set again");
    state = INITIAL;
  }
}

int main( int argc, char** argv )
{
  bool isTest = false;
  for(int i = 0; i < argc; i++) {
    ROS_INFO("%s", argv[i]);
    if(strcmp(argv[i], "--test") == 0)
      isTest = true;
  }

  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  marker_pub_ptr = &marker_pub;

  visualization_msgs::Marker marker;
  add_marker(marker, pickup.x, pickup.y);
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  state = PICKUP;
  marker_pub.publish(marker);
  ROS_INFO("marker at pickup zone");

  if(isTest) {
    sleep(5);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "add_markers";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub_ptr->publish(marker);
    ROS_INFO("marker deleted");

    sleep(5);
    add_marker(marker, 3, 8);
    marker_pub_ptr->publish(marker);
    ROS_INFO("marker at dropoff zone");
    ros::spin();
    return 0;
  }
  ros::Subscriber sub1 = n.subscribe("/odom", 10, process_odom_callback);
  ros::ServiceClient client = n.serviceClient<pick_objects::go_to_xy>("go_to_xy");
  client_ptr = &client;

  go_to_location(pickup);

  ros::spin();
}
