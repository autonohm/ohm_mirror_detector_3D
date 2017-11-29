/**
* @file   ohm_mirror_prefilter3D.h
* @author Rainer Koch
* @date   20.03.2016
*
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
#include <visualization_msgs/Marker.h>

/*
 * mirror detector3D includes
 */
#include "Scan3D.h"
#include "Object3D.h"
#include "planeFunctions.h"
#include "rosfunctions3D.h"
#include "testFunctions.h"

/*
 * initializes the prefilter
 */
void init();

/*
 * subscribes messages with 3D multi echo coords from the 3D Scanner
 */
void subscriberFunc(const ohm_hokuyo3d::CoordDistMulti& msg);

void pubPoints();

void pubLines();

/*
 * publish prefiltered 3D multi echo coords
 */
void publisherFunc(void);

/*
 * function to detect reflective or transparent objects and copy them into an separate object
 */
void separateObjectPoints(mirrordetector::scan3D* scan, std::vector<mirrordetector::object3D*>& object, unsigned int amount_affected, unsigned int& objectCounter, double planeDetection_threshold, double visionCone_threshold, unsigned int planeDetectin_minPoints, double maxDist, double maxRot, double fitnessICP, double threshold_median, unsigned int normIntensWhPaper_rise,  unsigned int normIntensWhPaper_offset);

/*
 * assign objectsType based on corners
 */
void cleanScan(std::vector<mirrordetector::scan3D*> scan, std::vector<mirrordetector::object3D*>& object, unsigned int amount_objects, double thres_dist, double thres_visionCone);

