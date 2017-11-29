/**
* @file   ${replay.cpp}
* @author Rainer Koch
* @date   ${2015/03/06}
*
* Listen to laser data and set them to new frame and tf
*/


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <float.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <memory>
#include <stdlib.h>
#include <math.h>

#include "replay.h"

using namespace std;

ros::Publisher _pub_scan;
std::string _pub_frame;


// Ros functions
void publisherFunc(const sensor_msgs::LaserScan& scan)
{
  _pub_scan.publish(scan);

}


void subscriberFunc(const sensor_msgs::LaserScan& msg)
{
  sensor_msgs::LaserScan scan;
  int scanSize = 0;

  // copy msg information

  scan.header.frame_id  = _pub_frame;
  scan.header.stamp     = ros::Time::now();
  scan.header.seq       = msg.header.seq;
  scan.angle_min        = msg.angle_min;
  scan.angle_max        = msg.angle_max;
  scan.angle_increment  = msg.angle_increment;
  scan.range_min        = msg.range_min;
  scan.range_max        = msg.range_max;
  scan.scan_time        = msg.scan_time;

  scanSize              = (unsigned int) (scan.angle_max -  scan.angle_min) / scan.angle_increment;

  //ROS_INFO("Reallocating buffer for scan size: %d", bufferSize);
  scan.ranges.resize(scanSize);
  scan.intensities.resize(scanSize);

  for (unsigned int i = 0; i < scanSize; i++)
  {
    scan.ranges[i]      = msg.ranges[i];
    scan.intensities[i] = msg.intensities[i];
  }

  publisherFunc(scan);

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "ohm_replay");
  ros::NodeHandle nh_sub("~");
  ros::NodeHandle nh_pub("~");


  // Read parameter from launch file
  std::string sub_scan;
  std::string pub_scan;

  nh_sub.param<std::string>("sub_scan",            sub_scan,                "scan");
  nh_pub.param<std::string>("pub_scan",            pub_scan,                "scan_new");
  nh_pub.param<std::string>("pub_frame",           _pub_frame,               "laser_new");

  // initialize subscriber & publisher
  ros::Subscriber sub = nh_sub.subscribe(sub_scan, 1, subscriberFunc);
  _pub_scan               = nh_pub.advertise<sensor_msgs::LaserScan>(pub_scan, 1);

  ROS_INFO("Ohm_replay init");

  ros::Rate r(50);

  while(ros::ok())
  {
    ros::spinOnce();

    r.sleep();
  }

  ROS_INFO("Shutting down");

  return 0;
}

