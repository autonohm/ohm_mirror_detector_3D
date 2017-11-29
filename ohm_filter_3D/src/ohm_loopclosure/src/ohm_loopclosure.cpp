/**
 * @file   ohm_loopclosure.cpp
 * @author Rainer Koch
 * @date   18.06.2015
 *
 *
 */

#include "ohm_loopclosure.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>


//#include <tf/tf.h>
#include <float.h>
#include <cmath>

#include <sstream>


// ROS variables
ros::Publisher _pub_loopclosed;
sensor_msgs::LaserScan _scan;
std::vector<geometry_msgs::PoseStamped*> _pose_history(1000);
nav_msgs::Path _path;

float _thres_x = 0.1;
float _thres_y = 0.1;
float _thres_z = 0.1;
unsigned int _checkAfterPoses = 500;

unsigned int _historysize = 0;
bool _loopClosed = false;
bool _onlyOnce = false;


#define _USE_MATH_DEFINES

using namespace std;

// Ros functions
void subscriberPose(const geometry_msgs::PoseStamped& position)
{
  if(_path.poses.size() == _historysize)
  {
    _path.poses.resize(_historysize + 1000);
  }

  _path.header.seq          = position.header.seq;
  _path.header.stamp        = position.header.stamp;
  _path.header.frame_id     = position.header.frame_id;

  _path.poses[_historysize].pose.position.x     = position.pose.position.x;
  _path.poses[_historysize].pose.position.y     = position.pose.position.y;
  _path.poses[_historysize].pose.position.z     = position.pose.position.z;

  _path.poses[_historysize].pose.orientation.x  = position.pose.orientation.x;
  _path.poses[_historysize].pose.orientation.y  = position.pose.orientation.y;
  _path.poses[_historysize].pose.orientation.z  = position.pose.orientation.z;
  _path.poses[_historysize].pose.orientation.w  = position.pose.orientation.w;

  _historysize++;
 // cout << "Histsize loopclosure " << _historysize << endl;

  if(_historysize > _checkAfterPoses)
  {
//    for(int i=0; i < _historysize; i++)
//    {
//      cout << "hist " << i << " " << _path.poses[i].pose.position.x << endl;
//
//    }
//    cout << "--------------" << endl;
    if(!_onlyOnce)
    {
      ROS_INFO("Starting checking for closed loop");
      _onlyOnce = true;
    }
    _loopClosed = checkLoopClosed(_path, _historysize);
    //TODO: change here later to find multible loops
  }
}

void publisherLoopclosed()
{
  ROS_INFO("Found a closed loop");
  std_msgs::Bool activate;
  activate.data = true;
  _pub_loopclosed.publish(activate);

}

bool checkLoopClosed(nav_msgs::Path history, int size)
{

//  for(int i=0; i < _historysize; i++)
//  {
//    cout << "hist " << i << " " << history.poses[i].pose.position.x << endl;
//
//  }
//  cout << "--------------" << endl;
 // cout << "here: " << historysize << "->" << history[0]->pose.position.x <<  "/"<<  history[5]->pose.position.x << endl;

  float offset_x = _thres_x+100;
  float offset_y = _thres_y+100;
  float offset_z = _thres_z+100;
  offset_x = abs(history.poses[0].pose.position.x - history.poses[size-1].pose.position.x);

  offset_y = abs(history.poses[0].pose.position.y - history.poses[size-1].pose.position.y);
  offset_z = abs(history.poses[0].pose.position.z - history.poses[size-1].pose.position.z);

//  cout << "hist 0 " << history.poses[0].pose.position.x << "/" << history.poses[0].pose.position.y << "/" << history.poses[0].pose.position.z <<endl;
//  cout << "hist " << size << ":" << history.poses[size-1].pose.position.x << "/" << history.poses[size-1].pose.position.y << "/" << history.poses[size-1].pose.position.z << endl;
//  cout << "offset " << offset_x << "/" << offset_y << "/" << offset_z << endl;
//  cout << "thres " << _thres_x << "/" << _thres_y << "/" << _thres_z << endl;

  if((offset_x < _thres_x) && (offset_y < _thres_y) && (offset_z < _thres_z))
    return true;
  else
    return false;
}

/*
 * Init
 */
void init()
{

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ohm_loopclosure");
  ros::NodeHandle nh_sub("~");
  ros::NodeHandle nh_pub("~");

  // Parameters for launch file
  std::string sub_pose;
  std::string pub_loopclosed;
  double dVar = 0;
  int iVar = 0;

  nh_sub.param < std::string > ("sub_pose", sub_pose, "pose");
  nh_sub.param < std::string > ("pub_loopClosed", pub_loopclosed, "loopClosed");
  nh_pub.param<double>("thres_x", dVar, 0.1);                      // allowed offset in x between start and end position of loop
  _thres_x = static_cast<float>(dVar);
  nh_pub.param<double>("thres_y", dVar, 0.1);                      // allowed offset in y between start and end position of loop
  _thres_y = static_cast<float>(dVar);
  nh_pub.param<double>("thres_z", dVar, 0.1);                      // allowed offset in z between start and end position of loop
  _thres_z = static_cast<float>(dVar);
  nh_pub.param<int>("checkAfterPoses", iVar, 500);              // number of poses before loop closure starts checking for a loop
  _checkAfterPoses = static_cast<unsigned int>(iVar);

  // initialize subscriber
  ros::Subscriber sub = nh_sub.subscribe(sub_pose, 1, subscriberPose);

  // initialize publisher
  _pub_loopclosed = nh_pub.advertise <std_msgs::Bool> (pub_loopclosed, 1);

 // init();

  ros::Rate r(50);
  ROS_INFO("Init Loopclosure done");

  while(ros::ok())
  {
    if(_loopClosed)
    {
      publisherLoopclosed();
      _historysize = 0;
      _loopClosed = false;
    }

    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Shutting down");

 // delete _pose_history;

  return 0;
}
