/**
 * @file   ohm_mirror_postfilter_V1.h
 * @author Rainer Koch
 * @date   23.04.2015
 *
 *
 */

//TODO: sync between echo filter and tsd-slam
#include <vector>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ohm_mirror_postfilter/ohm_sensor_msgs.h>

#include <obcore/math/Quaternion.h>
#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"

void subscriberPose(const geometry_msgs::PoseStamped& pose);

void publisherLoopclosed();

/*
 * Main function
 * checks offset of start and actual point.
 * If offset smaller than thresholds, then returns true
 *
 * Input:
 * history  ros path to get first and last point
 * size     number of poses
 *
 * Return:
 * true, if loop is closed
 */
bool checkLoopClosed(nav_msgs::Path history, int size);
