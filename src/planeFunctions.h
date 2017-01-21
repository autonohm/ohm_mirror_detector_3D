/**
* @file   planeFunctions.h
* @author Rainer Koch
* @date   30.03.2016
*
*
*/

#ifndef PLANEFUNCTIONS_H_
#define PLANEFUNCTIONS_H_

/*
 * includes
 */
#include <cmath>
#include <stdlib.h>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include "obcore/math/linalg/gsl/Matrix.h"

/*
 * own includes
 */
#include "Object3D.h"

using namespace std;

/*
 * pointFrustumFilter
 * checks, if point is between the four planes
 * @param[in] point: point to check (x,y,z)
 * @param[in] dist_E1_E2: distance between plane 1 and plane 2 [m]
 * @param[in] dist_E3_E4: distance between plane 3 and plane 4 [m]
 * @param[in] plane1: left boarder plane (n_x,n_y,n_z,d)
 * @param[in] plane2: right boarder plane (n_x,n_y,n_z,d)
 * @param[in] plane3: bottom boarder plane (n_x,n_y,n_z,d)
 * @param[in] plane4: top boarder plane (n_x,n_y,n_z,d)

 * @param[out] bool: true = point in frustum
 *
 */
bool pointFrustumFilter(double* point, double* top_E, double* bottom_E, double* right_E, double* left_E, double thres_dist);

/*
 * maskPointsInCone
 * vision frustum, checks if the point is
 * @param[in] point: scan point (x,y,z)
 * @param[in] objectCorners: four corners of plane (x,y,z)
 * @param[in] thres_angle: angle threshold from plane (enlarges plane)
 * @param[out] mask: mask point in the vision cone (1 = in vision cone, 2 = out of vision cone)
 */
void pointsInCone(double* points, int size, double* objectCorners, double thres_dist, double thres_visionCone, bool* mask);

/*
 * pointLocation
 * vision frustum
 * @param[in] point: scan point (x,y,z)
 * @param[in] size: amount of scan points
 * @param[in] objectCorners: four corners of plane (x,y,z)
 * @param[in] thres_dist: threshold distance in front and after the plane [m]
 * @param[in] thres_visionCone: threshold around the object (extends the plane size) [m]
 * @param[out] pointLocMask: mask point (0 = in front the object or out of the vision cone, 1 = on the object, 2 =behind the object)
 */
void pointLocation(double* points, int size, double* objectCorners, double* planeValues, double thres_dist, double thres_visionCone, int* pointLocMask);

/*
 * behindPlane
 * checks is a point is behind a plane
 * @param[in] point_x: x value [m]
 * @param[in] point_y: y value [m]
 * @param[in] point_z: z value [m]
 * @param[in] planeValues:  coefficients of plane a,b,c,d (E: a*x + b*y + c*z + d = 0
 * @param[out] bool: true = behind plane, false = not behind plane
 */
bool behindPlane(double point_x, double point_y, double point_z, double* planeValues);

/*
 * cornersToPlaneEqu
 * calculate plane equation values in cartesian equation
 * @param[in] objectCorners: three corner points (x,y,z)
 * @param[out] cartesianEquationValues: (x_n, y_n, z_n, a) E: x_n*X + y_n*Y + z_n*Z + a = 0
 * @param[out] bool: calculation successful
 *
 */
void cornersToPlaneEqu(double* objectCorners, double* cartesianEquationValues);

/*
 * calcNormal
 *
 */
double* calcNormal(double* v1, double* v2);

/*
 * distanceToPlane
 * @param[in] planeEqu: plane values in cartesian equation form (x_n, y_n, z_n, a) E: x_n*X + y_n*Y + z_n*Z + a = 0
 * @param[in] point: point to check (x,y,z)
 * @param[out] double: distance of point to plane
 *
 */
double distanceToPlane(double* planeEqu, double* point);

/*
 * calcVectorLength
 * @param[in] vector: vector to calculate length (x,y,z)
 * @param[out] double: length of vector = sqrt(x*x + y*y + z*z)
 *
 */
double calcVectorLength(double* vector);

/*
 * calcVectorLength
 * @param[in] point1: beginning point(x,y,z)
 * @param[in] point2: end point(x,y,z)
 * @param[out] double: length of vector = sqrt(x*x + y*y + z*z)
 *
 */
double calcVectorLength(double* v1, double* v2);

/*
 * calcVectorLength
 * @param[in] x,y,z: vector values to calculate length
 * @param[out] double: length of vector = sqrt(x*x + y*y + z*z)
 *
 */
double calcVectorLength(double x, double y, double z);

/*
 * calcAngleVectors
 * @param[in] c1, c2: vectors
 * @param[out] double: angle in °
 *
 */
double calcAngleVectors(double* v1, double* v2);

/*
 * calcAngleVectorAxes
 * @param[in] c1: vector
 * @param[out] vector: angle to x, angle to y, angle to z in °
 *
 */
double* calcAngleVectorAxes(double* v1);

/*
 * basepoint2Plane
 * calculates the base point of a point next to a plane
 * @param[in] plane: coefficients of plane a,b,c,d (E: a*x + b*y + c*z + d = 0
 * @param[in] point: point next to the plane (x,y,z)
 * @param[out] double: point on plane (x,y,z)
 */
double* basepoint2Plane(double* plane, double* point);

/*
 * intersectionWithPlane
 * calculates the intersection point of a point with a plane
 * @param[in] plane: coefficients of plane a,b,c,d (E: a*x + b*y + c*z + d = 0
 * @param[in] point: point next to the plane (x,y,z)
 * @param[out] double: point on plane (x,y,z)
 */
double* intersectionWithPlane(double* plane, double* point);

/*
 * sortCorners
 * sorts the corners of the plane
 * @param[in] c1,c2,c3,c4: unsorted corners
 * @param[out] c_sorted: sorted corners
 */
void sortCorners(double* c1, double* c2, double* c3, double* c4, double* c_sorted);

/*
 * mirrorOnPlane
 * mirrors the coords on the plane
 * @param[in] coords: coords to mirror
 * @param[in] plane_Coefficients: coefficients of plane a,b,c,d (ax+by+cz+d = 0)
 * @param[in] corners_in: corners of plane
 * @param[in] size_coords: amount of coords to mirror
 * @param[out] coords: coords which are mirrored
 */
void mirrorOnPlane(double* coords, double* mirrored_coords, double* plane_Coefficients, unsigned int size_coords);

#endif /* PLANEFUNCTIONS_H_ */
