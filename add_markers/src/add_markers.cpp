/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <cmath>

// where we start

double pose[2] = {0, 0};  

// where we are on an ongoing basis for the node to subscribe to

void get_pose(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose[0] = msg->pose.pose.position.x;
  pose[1] = msg->pose.pose.position.y;
}

//where we're going

double collectionPoint[2]  = {1.0, 0.0};//... first
double deliverPoint[2] = {-3.0, 2.0}; //... finally

// duo of functions to work out if we've arrived, returning truthily if so

bool arrived_collection_point(double goalPos[2])
{
double dx = goalPos[0] - pose[0];
  double dy = goalPos[1] - pose[1];
    if (sqrt(dx*dx + dy*dy) < 0.7)
return 1;
}


bool arrived_deliver_point(double goalPos[2])
{
double dx = goalPos[0] - pose[0];
  double dy = goalPos[1] - pose[1];
    if (sqrt(dx*dx + dy*dy) < 1.1)
return 1;
}


int main( int argc, char** argv )
{
//set up the node and a publisher of the marker alongside the subscriber to the turtlebot's pose
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber pose_sub = n.subscribe("/odom", 10, get_pose);



  // Set the marker shape as a sphere
  uint32_t shape = visualization_msgs::Marker::SPHERE;

  enum travel_style {

	COLLECT,
	TRANSIT,
	DELIVERED
} travel_style = COLLECT;

  while (ros::ok())
  {
 
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
  


    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  This is a SPHERE
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.77;
    marker.scale.y = 0.77;
    marker.scale.z = 0.77;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();


    ros::spinOnce();

// and action!..

 if (travel_style == COLLECT){
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = collectionPoint[0];
      marker.pose.position.y = collectionPoint[1];
      marker_pub.publish(marker);
      if (arrived_collection_point(collectionPoint)) {
        sleep(5);
        ROS_INFO("Transporting to delivery point ... ");
        travel_style = TRANSIT;
	}
}

else if (travel_style == TRANSIT) {
      marker.action = visualization_msgs::Marker::DELETE;
      marker.pose.position.x = deliverPoint[0];
      marker.pose.position.y = deliverPoint[1];
      marker_pub.publish(marker);
      ROS_INFO("Still carrying");
if(arrived_deliver_point(deliverPoint)){
        ROS_INFO("Arrived at delivery point. ");
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = deliverPoint[0];
      marker.pose.position.y = deliverPoint[1];
      marker_pub.publish(marker);
      ROS_INFO("Package delivered!.. and did you see the size of it? Whoa, that was heavy!!!");
      travel_style = DELIVERED;
    }
  }

else if (travel_style == DELIVERED){
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = deliverPoint[0];
      marker.pose.position.y = deliverPoint[1];
      marker_pub.publish(marker);
}

else {
      ROS_INFO("Something went terribly wrong!");
    }
  }
}