/**
* @file   ohm_mirror_postfilter3D.h
* @author Rainer Koch
* @date   01.04.2016
*
*
*/

/*
 * includes
 */
#include <string>
#include <cstring>

/*
 * ros includes
 */
#include <ros/ros.h>
#include <ohm_hokuyo3d/CoordDistMulti.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

/*
 * obvious includes
 */
#include <obcore/math/Quaternion.h>

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
 * subscribes messages with 3D multi echo coords from the pre filter
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
//void separateAffectedPoints(mirrordetector::scan3D* scan, mirrordetector::object3D* object, double thres_dist);
void separateAffectedPoints(std::vector<mirrordetector::scan3D*> scan, unsigned int size_scanHistory, unsigned int size_corrupted, std::vector<unsigned int> scan_nr_corrupted, mirrordetector::scan3D*& corrupted_scans);

/*
 * assign objectsType based on corners
 */
//void cleanScan(mirrordetector::scan3D* scan, <mirrordetector::object3D*> object, double thres_dist);
void cleanScan(std::vector<mirrordetector::scan3D*> scan, std::vector<mirrordetector::object3D*> object, unsigned int amount_objects, double thres_dist, double thres_visionCone);

/*
 * uses a new scan which has an transparent or reflective object and adds it to the object list.
 */
void buildObjectHistory(mirrordetector::scan3D scan, mirrordetector::object3D* object);
