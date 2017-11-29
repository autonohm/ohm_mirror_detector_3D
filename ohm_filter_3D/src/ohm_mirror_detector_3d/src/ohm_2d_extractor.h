/**
* @file   ohm_2d_extractor.h
* @author Rainer Koch
* @date   13.02.2017
*
*/

/*
 * includes
 */
#include <string>

/*
 * ros includes
 */
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ohm_hokuyo3d/CoordDistMulti.h>

/*
 * subscribes messages with 3D multi echo coords from the 3D Scanner
 */
void subscriberFunc(const ohm_hokuyo3d::CoordDistMulti& msg);

