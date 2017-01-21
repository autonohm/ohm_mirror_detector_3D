/**
* @file   Object3D.h
* @author Rainer Koch
* @date   21.03.2016
*
*
*/

#ifndef OBJECT3D_H
#define OBJECT3D_H

/*
 * includes
 */
#include <vector>

/*
 * ros includes
 */
#include <ros/ros.h>

/*
 * pcl includes
 */
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

/*
 * own includes
 */
#include "planeFunctions.h"

using namespace std;

namespace mirrordetector
{
/**
 * @class OBJECT3D
 * @brief class to describe objects witht he mirror detector
 * @author Rainer Koch
 */
class object3D
{
public:
//  /*
//  *            0: not defined/checked yet
//  *            1: regular scan point
//  *            2: error Surface
//  *            3: behind error
//  *            4: point on reflective object surface
//  *            5: reflected point
//  *            6: point on transparent object surface
//  *            7: behind transparent object surface
//  */
  typedef enum { unchecked,
                 validPoint,
                 errorSurface,
                 behindErrorS,
                 reflectiveSurface,
                 behindReflective,
                 transpSurface,
                 behindTransparent,
                 nanPoint} objType;

  /**
   * Standard constructor
   * @param[in] fragment_id: id of fragment of scan
   * @param[in] fragment_size: size of the fragment
   * @param[in] size: size of scan
   * @param[in] coords_1: coordinates (x,y,z) of echo 1
   * @param[in] coords_2: coordinates (x,y,z) of echo 2
   * @param[in] intens_1: intensities of echo 1
   * @param[in] intens_2: intensities of echo 2
   */
  object3D();
  object3D(unsigned int seq, unsigned long time, double* corners, double* centroid);

  /**
   * Destructor
   */
  virtual ~object3D();

  /*
   * setCoords
   * @param[in] coords: write coordinates (x,y,z) of echo 1
   */
  void setCoords(double* coords);
  /*
   * getCoords
   * @param[out] coords: read coordinates (x,y,z) of echo 1
   */
  double* getCoords();

  /*
   * setNormals
   * @param[in] normals: write normals (x,y,z) of coordinates of echo 1
   */
  void setNormals(double* normals);
  /*
   * getNormals
   * @param[out] normals: read normals (x,y,z) of coordinates of echo 1
   */
  double* getNormals();

  /*
   * setIntensity
   * @param[out] intens: write intensities of echo 1
   */
  void setIntensity(unsigned int* intens);
  /*
   * getIntensity
   * @param[out] intens: read intensities of echo 1
   */
  unsigned int* getIntensity();

  /*
   * setDist
   * @param[in] distance: write distances of scan points echo 1
   */
  void setDist(double* distance);
  /*
   * getDist
   * @param[out] distance: read distances of scan points of echo 1
   */
  double* getDist();

  /*
   * setPhi
   * @param[in] phi: write angle of scan points echo 1
   */
  void setPhi(double* phi);
  /*
   * getPhi
   * @param[out] phi: read angle of scan points echo 1
   */
  double* getPhi();

  /*
   * setObjectType
   * @param[in] objectType: write type of object
   */
  void setObjectType(object3D::objType objectType);
  /*
   * getObjectType
   * @param[out] objectType: read type of object
   */
  object3D::objType getObjectType();

  /*
   * setTransformation
   * @param[in] Trans: Transformation matrix (location from where the object was seen -> to calculate object into world coordinate system
   */
  void setTransformation(Eigen::Matrix4f Trans);
  /*
   * getTransformation
   * @param[out] : Transformation matrix (location from where the object was seen -> to calculate object into world coordinate system
   */
  Eigen::Matrix4f getTransformation();

  /*
   * setCoefficients
   * @param[in] coefficients: a,b,c,d of object
   * @param[in] objectNr: number of object the coefficients belong to.
   */
  void setCoefficents(double* coefficients);
  /*
   * getCoefficients
   * @param[out] coefficients: a,b,c,d of an object
   */
  double* getCoefficents();

  /*
   * setCentroid
   * @param[in] centroid: coordinates of center point
   */
  void setCentroid(double* centroid);
  /*
   * getCentroid
   * @param[out] centroid: coordinates of center point
   */
  double* getCentroid();

  /*
   * getObjectCorners
   * @param[out] corners: 3 corners (x,y,z) of square object
   *                      first point: first corner = center point of object (shifting of center)
   *                      second point, third point: spanning up a vector from center point to corners
   */
  double* getObjectCorners();

  /*
   * setMajorDim
   * @param[in] dim: maximal value of dimension
   */
  void setMajorDim(double dim);
  /*
   * getMajorDim
   * @param[out] dim: maximal value of dimension
   */
  double getMajorDim();

  /*
   * setMinorDim
   * @param[in] dim: minimal value of dimension
   */
  void setMinorDim(double dim);
  /*
   * getMinorDim
   * @param[out] dim: minimal value of dimension
   */
  double getMinorDim();



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
   * getSize
   * @param[out] number: read size of scan
   */
  unsigned int getSize();
  /*
   * setSize
   * @param[in] number: size of scan
   */
  void setSize(unsigned int size);

  /*
   * compareObject
   * compare this object with the existingObject
   * @param[in] existingObject: already existing object
   * @param[in] thres: distance threshold [m]
   * @param[return]: if object is equal return false
   */
  bool compareObject(object3D* existingObject, double thres);

  /*
   * identifyObjectType
   * identify, if object is specular reflective or transparent
   */
  bool analyzeObjectType(double* coords1, double* coords2, unsigned int* intens1, unsigned int* intens2, int scanSize, double thres_dist, double thres_visionCone, double maxDist, double fitnessICP);

protected:
  unsigned int _seq;
  unsigned long _time_stamp;

  /*
   * dimensions of object
   */
  double _major_dim;
  double _minor_dim;

  objType _objectType;                                // reflective, transparent
  double* _objectCorners;                             // four corners
  double* _objectCentroid;                            // Centroid of four corners, Center of plane

  double* _objectCoefficients;                        // a,b,c,d -> ax+by+cz+d = 0

  unsigned int _size;                                 // amount of points of the object, >0 if they are stored
  unsigned int _amount_points;
  double* _coords;
  double* _normals;
  double* _dist;
  double* _phi;
  unsigned int* _intensity;
  Eigen::Matrix4f _transformation;                     // transformation to back project points behind the reflective plane
};
}


#endif
