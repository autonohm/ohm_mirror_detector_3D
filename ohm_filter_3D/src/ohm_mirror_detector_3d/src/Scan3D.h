/**
* @file   SCAN3D.h
* @author Rainer Koch
* @date   21.03.2016
*
* all distance vales are in [m]
*/

#ifndef SCAN3D_H
#define SCAN3D_H

/*
 * includes
 */
#include <string>
#include <vector>

/*
 * ros includes
 */
#include <ros/ros.h>

/*
 * PCL includes
 */
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/filters/conditional_removal.h>

// for object clustering
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter.h>

// for object clustering region growing
//#include <vector>
//#include <pcl/search/search.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/segmentation/region_growing.h>

#include <obcore/math/linalg/eigen/Matrix.h>



//for pca analysis
#include <pcl/features/moment_of_inertia_estimation.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
//#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>

/*
 * obvious includes
 */
//#include "obcore/math/linalg/gsl/Matrix.h"

/*
 * own includes
 */
#include "Object3D.h"
#include "testFunctions.h"


using namespace std;
//using namespace obvious;

namespace mirrordetector
{
/**
 * @class SCAN3D
 * @brief class to describe objects with he mirror detector
 * @author Rainer Koch
 */
class scan3D
{
public:

  /**
   * Standard constructor
   * @param[in] seq: sequenz number of scan
   * @param[in] timestamp: time stamp in nsec
   * @param[in] frame_id: frame name
   * @param[in] fragment_id: id of fragment of scan
   * @param[in] fragment_size: size of the fragment
   * @param[in] size: size of scan
   * @param[in] coords_1: coordinates (x,y,z) of echo 1
   * @param[in] coords_2: coordinates (x,y,z) of echo 2
   *
   */
  scan3D(unsigned int seq, unsigned long timestamp, std::string frame_id, unsigned int fragmentid, unsigned int fragement_size, unsigned int size, unsigned int stepsPhi, double* coords_1, unsigned int* intens_1, double* coords_2, unsigned int* intens_2);

  /**
   * Destructor
   */
  virtual ~scan3D();

  /*
   * setCoords1
   * @param[in] coords: write coordinates (x,y,z) of echo 1
   */
  void setCoords1(double* coords);
  /*
   * getCoords1
   * @param[out] coords: read coordinates (x,y,z) of echo 1
   */
  double* getCoords1();
  /*
   * setNormals1
   * @param[in] normals: write normals (x,y,z) of coordinates of echo 1
   */
  void setNormals1(double* normals);
  /*
   * getNormals1
   * @param[out] normals: read normals (x,y,z) of coordinates of echo 1
   */
  double* getNormals1();
  /*
   * setMask1
   * @param[out] mask: write mask of coordinates of echo 1
   */
  void setMask1(bool* mask);
  /*
   * getMask1
   * @param[in] mask: read mask of coordinates of echo 1
   */
  bool* getMask1();
  /*
   * setIntensity1
   * @param[out] intens: write intensities of echo 1
   */
  void setIntensity1(unsigned int* intens);
  /*
   * getIntensity1
   * @param[out] intens: read intensities of echo 1
   */
  unsigned int* getIntensity1();

  /*
   * setDist1
   * @param[in] distance: write distances of scan points echo 1
   */
  void setDist1(double* distance);
  /*
   * getDist1
   * @param[out] distance: read distances of scan points of echo 1
   */
  double* getDist1();
  /*
   * setPhi1
   * @param[in] phi: write angle of scan points echo 1
   */
  void setPhi1(double* phi);
  /*
   * getPhi1
   * @param[out] phi: read angle of scan points echo 1
   */
  double* getPhi1();

  /*
   * setObjectType1
   * @param[in] objectType: write type of object of echo 1
   *            0: not defined/checked yet
   *            1: regular scan point
   *            2: on error causing object
   *            3: error
   *            4: point on reflective object surface
   *            5: reflected point
   *            6: point on transparent object surface
   *            7: behind transparent object surface
   */
  void setObjectType1(object3D::objType* objectType);
  /*
   * getObjectType1
   * @param[out] objectType: read type of object of echo 1
   *              0: not defined/checked yet
   *              1: regular scan point
   *              2: error
   *              3: behind error causing object
   *              4: point on reflective object surface
   *              5: reflected point
   *              6: point on transparent object surface
   *              7: behind transparent object surface
   */
  object3D::objType* getObjectType1();

  /*
   * setCoords2
   * @param[in] coords: write coordinates (x,y,z) of echo 2
   */
  void setCoords2(double* coords);
  /*
   * getCoords2
   * @param[out] coords: read coordinates (x,y,z) of echo 2
   */
  double* getCoords2();
  /*
   * setNormals2
   * @param[in] normals: write normals (x,y,z) of coordinates of echo 2
   */
  void setNormals2(double* normals);
  /*
   * getNormals2
   * @param[out] normals: read normals (x,y,z) of coordinates of echo 2
   */
  double* getNormals2();
  /*
   * setMask2
   * @param[out] mask: write mask of coordinates of echo 2
   */
  void setMask2(bool* mask);
  /*
   * getMask2
   * @param[in] mask: read mask of coordinates of echo 2
   */
  bool* getMask2();
  /*
   * setIntensity2
   * @param[out] intens: write intensities of echo 2
   */
  void setIntensity2(unsigned int* intens);
  /*
   * getIntensity2
   * @param[out] intens: read intensities of echo 2
   */
  unsigned int* getIntensity2();

  /*
   * setDist2
   * @param[in] distance: write distances of scan points echo 2
   */
  void setDist2(double* distance);
  /*
   * getDist2
   * @param[out] distance: read distances of scan points of echo 2
   */
  double* getDist2();
  /*
   * setPhi2
   * @param[in] phi: write angle of scan points echo 2
   */
  void setPhi2(double* phi);
  /*
   * getPhi2
   * @param[out] phi: read angle of scan points echo 2
   */
  double* getPhi2();

  /*
   * setObjectType2
   * @param[in] objectType: write type of object of echo 2
   *            0: not defined/checked yet
   *            1: regular scan point
   *            2: on error causing object
   *            3: error
   *            4: point on reflective object surface
   *            5: reflected point
   *            6: point on transparent object surface
   *            7: behind transparent object surface
   */
  void setObjectType2(object3D::objType* objectType);
  /*
   * getObjectType2
   * @param[out] objectType: read type of object of echo 2
   *              0: not defined/checked yet
   *              1: regular scan point
   *              2: error
   *              3: behind error causing object
   *              4: point on reflective object surface
   *              5: reflected point
   *              6: point on transparent object surface
   *              7: behind transparent object surface
   */
  object3D::objType* getObjectType2();

  /*
   * setSequenz
   * @param[in] seq: sequenz nr
   *
   */
  void setSeqenz(unsigned int seq);
  /*
   * getSequenz
   * @param[out] number: read number of sequenz
   */
  unsigned int getSeqenz();

  /*
   * setTimeStamp
   * @param[in] time: ros time stamp
   */
  void setTimeStamp(unsigned long time);
  /*
   * getTimeStamp
   * @param[out] time: read timestamp in nsec
   */
  unsigned long getTimeStamp();

  /*
   * setFrame_id
   * @param[in] frame_id: name of frame
   */
  void setFrame_id(std::string frame_id);
  /*
   * getFrame_id
   * @param[out] name: read id name of frame
   */
  std::string getFrame_id();

  /*
   * setFragment_id
   * @param[in] fragment_id: id of fragment
   */
  void setFragment_id(unsigned int fragment_id);
  /*
   * getFragment_id
   * @param[out] number: read id number of fragment
   */
  unsigned int getFragment_id();

  /*
   * setFragment_size
   * @param[in] fragment_size: size of fragment
   */
  void setFragment_size(unsigned int fragment_size);
  /*
   * getFragment_size
   * @param[out] number: read fragment size
   */
  unsigned int getFragment_size();

  /*
   * setSize
   * @param[in] size: amount of points
   */
  void setSize(unsigned int size);
  /*
   * getSize
   * @param[out] number: read size of scan
   */
  unsigned int getSize();

  /*
   * setStepsPhi
   * @param[in] steps: amount of steps for phi
   */
  void setStepsPhi(unsigned int steps);
  /*
   * getStepsPhi
   * @param[out] steps: read amount of steps of phi
   */
  unsigned int getStepsPhi();

  /*
   * setPubResults
   * @param[in] pub_Results: publishes the Results of object classification
   */
  void setPubResults(bool pubResults);
  /*
   * getPubResults
   * @param[out] pub_Results: publishes the Results of object classification
   */
  bool getPubResults();

  /*
   * setTransformation
   * @param[in] T: Transformation matrix (4x4)
   */
  void setTransformation(obvious::Matrix T);
  /*
   * getTransformation
   * @param[out] T: Transformation matrix (4x4)
   */
  obvious::Matrix getTransformation();

  /*
   * removePoint
   * @param[in] index: index of point to remove
   */
  void removePoint(int idx);

  /*
   * distanceThresFilter
   * filters data with a min. and max. distance
   * @param[in] min_dist: minimal measurement distance of scanner
   * @param[in] max_dist: maximal measurement distance of scanner
   */
  void distanceThresFilter(double min_dist, double max_dist);

  /*
   * boxFilter
   * filters out data located in the box
   * @param[in] box_min_x: minimal x value of box
   * @param[in] box_min_y: minimal y value of box
   * @param[in] box_min_z: minimal z value of box
   * @param[in] box_max_x: maximal x value of box
   * @param[in] box_max_y: maximal y value of box
   * @param[in] box_max_z: maximal z value of box
   */
  void boxFilter(double box_min_x, double box_max_x, double box_min_y, double box_max_y, double box_min_z, double box_max_z);

  /*
   * validPointFilter
   * filters points which are not valid and marks
   */
  //void validPointFilter();

  /*
   * outlierFilterkMean (not used)
   * http://pointclouds.org/documentation/tutorials/remove_outliers.php
   * @param[in] kMean: number of neighbors to analyze for each point
   * @param[in] stdDeviation: standard deviation multiplier. What this means is that all points who have a distance larger than kMean standard deviation of the mean distance to the query point will be marked as outliers and removed
   */
  void outlierFilterKMean(int kMean, double stdDeviation, object3D::objType objectType);

  /*
   * outlierFilterRadius
   * http://pointclouds.org/documentation/tutorials/remove_outliers.php
   * @param[in] neightbors: min number of neighbors in the search radius
   * @param[in] radius: search radius for neightbors
   */
  void outlierFilterRadius(int neightbors, double radius, object3D::objType objectType);

  /*
   * identifyReflections
   * identifies reflections by checking the distances of echo1 vs. echo2
   * @param[in] threshold: if < threshold, points have same distance => no reflection
   * @param[out]: number: amount of points affected by reflections, 0 = no reflections
   */
  unsigned int identifyReflections(double thres_substract_distance);

  /*
   * indentifyObjects
   * extract object planes out of coords.
   * additional cluster points before search
   * @param[in] planeDetection_threshold: max. distance of object plane
   * @param[in] planeDetection_minPoints: minimal amount of point to identify plane as an object
   * @param[out] object: vector of found objects
   */
  void indentifyObjects(std::vector<mirrordetector::object3D*>& object, double planeDetection_threshold, unsigned int planeDetectin_minPoints);

  /*
   * indentifyObjects_old
   * extract object planes out of coords.
   * @param[in] planeDetection_threshold: max. distance of object plane
   * @param[in] planeDetection_minPoints: minimal amount of point to identify plane as an object
   * @param[out] object: vector of found objects
   */
  void indentifyObjects_old(std::vector<mirrordetector::object3D*>& object, double planeDetection_threshold, unsigned int planeDetectin_minPoints);


  //TODO: Change std::vector<double*>& centroid to double* centroid, similar as double* dimensions
  /*
   * indentifyCorners
   * takes an object plane and returns the corners of this plane
   * @param[in] coefficients: coefficients of the plane ax+by+cy+d = 0;
   * @param[in] coords: coords of object plane
   * @param[in] coords: amount of coords belong to the object plane
   * @param[out]: dimensions: dimensions of object (width and length)
   * @param[out]: centroid: center point of object plane
   * @param[out]: corners: four corner points of object plane
   *
   */
  double* indentifyCorners(double* coefficients, std::vector<double*>& centroid, double* coords, int amount_points, double* dimensions);

  /*
   * prints all scan points of a scan into designated files
   * filename_valid, filename_error.....
   */
  void printScanCoords2File(const char* filename);



protected:
  unsigned int _seq;
  unsigned long _time_stamp;
  unsigned int _fragmentid;
  unsigned int _fragmet_size;
  unsigned int  _stepsPhi;
  std::string _frame_id;


  unsigned int _size;
  bool _pub_Results;


  /*
   * objectType:
   *  0: not defined/checked yet
   *  1: regular scan point
   *  2: error causing object point
   *  3: point behind error causing object
   *  4: point on reflective object surface
   *  5: reflected point
   *  6: point on transparent object surface
   *  7: behind transparent object surface
   */
  object3D::objType* _objectType_1;
  object3D::objType* _objectType_2;

  /*
   * echo 1
   */
  double* _coords_1;
  double* _normals_1;

  bool*   _mask_1;
  double* _dist_1;
  double* _phi_1;

  unsigned int* _intensity_1;

  /*
   * echo 2
   */
  double* _coords_2;
  double* _normals_2;

  bool*   _mask_2;
  double* _dist_2;
  double* _phi_2;

  unsigned int* _intensity_2;

  obvious::Matrix* _T;

};
}


#endif
