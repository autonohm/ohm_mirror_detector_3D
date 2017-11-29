/*
 * simple_joy.cpp
 *
 *  Created on: Sep 19, 2016
 *      Author: phil
 */


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <string>

/* The axis and buttons are from a PS3 controller. */
enum Axis_ps3_sixad
 {
   A1_Y = 0,             //!< y axis of left analog stick
   A1_X,             //!< x axis of left analog stick
   A2_Y,             //!< y axis of right analog stick
   A2_X,             //!< x axis of right analog stick
   SIXXAXIS_Y,     //Accelerometer
   SIXXAXIS_X,     //Accelerometer
   SIXXAXIS_Z,     //Accelerometer
   nc,
   CROSS_UP,         //!< command cross up
   CROSS_R,          //!< command cross right
   CROSS_D,          //!< command cross down
   CROSS_L,          //!< command cross left
   L2,               //!< L2
   R2,               //!< R2
   L1,               //!< L1
   R1,               //!< R1
   AB_T,             //!< triangle button
   AB_C,             //!< circle button
   AB_X,             //!< cross button
   AB_S,             //!< square botton
 };

 /**
  *  @enum Buttons_ps3
  *  Buttons of the ps3 controller
  */
 enum Buttons_ps3_sixad
 {
   B_SELECT = 0,         //!< SELECT
   B_A1,             //!< A1
   B_A2,             //!< A2
   B_START,          //!< START
   B_UP,             //!< UP
   B_RIGHT,          //!< RIGHT
   B_DOWN,           //!< DOWN
   B_LEFT,           //!< LEFT
   B_L2,             //!< L2
   B_R2,             //!< R2
   B_L1,             //!< L1
   B_R1,             //!< R1
   B_T,              //!< triangle button
   B_C,              //!< circle button
   B_X,              //!< cross button
   B_S,              //!< square button
   B_PS        //!< PS button
 };

void callBackJoy(const sensor_msgs::Joy& joy);

static ros::Publisher _pubTwist;
static double _threshVel;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_joy");
  ros::NodeHandle nh;
  ros::NodeHandle prvNh("~");

  std::string topicJoy;
  std::string topicTwist;

  prvNh.param<std::string>("topic_joy",   topicJoy, "joy");
  prvNh.param<std::string>("topic_twist", topicTwist, "robot0/cmd_vel");
  prvNh.param<double>     ("thresh_vel", _threshVel, 0.5);

  ros::Subscriber subsJoy = nh.subscribe(topicJoy, 1, callBackJoy);
  _pubTwist = nh.advertise<geometry_msgs::Twist>(topicTwist, 1);
  ros::spin();
}

void callBackJoy(const sensor_msgs::Joy& joy)
{
  geometry_msgs::Twist twist;
  double forward = 0.0;
  double backward = 0.0;
  forward = std::abs(joy.axes[R2]);
  backward = std::abs(joy.axes[L2]);

  twist.angular.z = joy.axes[A1_Y];
  twist.linear.x = (forward - backward) * _threshVel;
  twist.angular.z *= _threshVel;
  _pubTwist.publish(twist);
}
