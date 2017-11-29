/**
* @file   reflectionIdentification.h
* @author Rainer Koch
* @date   10.01.2017
*
*
*/

#ifndef REFLECTIONIDENTIFICATION_H_
#define REFLECTIONIDENTIFICATION_H_

/*
 * includes
 */
#include <cmath>
//#include <math.h>
#include <stdlib.h>
#include <vector>
#include <limits>

#include <pcl/point_types.h>

/*
 * own includes
 */
#include "Object3D.h"

using namespace std;

/*
 * meanIntensFactorCheck
 * calculates the arithmetic mean intensity factor by dividing the sum of the intensity values of points on the plane with the sum of intensity values of points behind the plane
 *
 * @param[in] intens_plane: intensities of points on the plane
 * @param[in] intens_plane: intensities of points behind the plane
 * @param[in] size_plane: amount of points on the plane
 * @param[in] size_error: amount of points behind the plane
 * @param[in] threshold_median: threshold to determine between transparent and reflective properties
 * @param[return] int: result of object (factor < threshold_median*size_error -> transparent object properties, factor >= threshold_median*size_error -> reflective object properties)
 *
 */
int meanIntensFactorCheck(unsigned int* intens_plane, unsigned int* intens_error, int size_plane, int size_error, double thres_mean);

/*
 * transformationCheck
 * checks if the coords_error can be fit by ICP with the coords, after they are back-projected at the plane
 * @param[in] coords: scan points (x,y,z) of cleaned scan
 * @param[in] coords_error: erroneous scan points (x,y,z) behind plane -> they get back-projected and fitted to coords
 * @param[in] scanSize: amount of scan points of coords
 * @param[in] errorSize: amount of scan points of coords_error
 * @param[in] plane_coefficients: plane coefficients (a,b,c,d)
 * @param[in] maxDist: maximal Distance to fit ICP
 * @param[in] fitnessICP: maximal allowed fitness for ICP score
 * @param[out] Trans: Transformation matrix of back-projected points to coords (Identity Matrix, if no Transformation found)
 * @param[return] int: result of object (0 = no transformation found -> transparent object properties, 1 = Transformation found -> reflective object properties)
 *
 */
int transformationCheck(double* coords, double* coords_error, int scanSize, int errorSize, double* plane_coefficients, double maxDist, double maxRot, double fitnessICP, Eigen::Matrix4f* Trans);

/*
 * intensVariation
 * fits a line into the intensity values of echo 2.
 */
int intensVariation(unsigned int* intens_error, int size_error, double amVarFactor, double thres_VariationInt);

/*
 * phongCurveCheck
 * fits the phong curve into the intensity values of points on the plane.
 */
int phongCurveCheck(double* coords, double* coords2, unsigned int* intens_plane, unsigned int* intens_error, int size_plane, int size_error, double* centroid);

/*
 * intensity2PaperCheck
 * normed intensity to 1m vs the intensity white paper at 1m
 *
 */
int intensity2PaperCheck(double* coords, unsigned int* intens, int size, double* centroid, unsigned int normIntensWhPaper_rise, unsigned int normIntensWhPaper_offset);


int evaluateResults(int meanIntensResult, int transformationResult, int intensVariationResult);



unsigned int medianIntens(unsigned int* intens, int size);

double meanIntens(unsigned int* intens, int size);

//unsigned int modusIntens(unsigned int* intens, int size);
//
//unsigned int arethmMeanIntens(unsigned int* intens, int size);
//
//double varianzIntens(unsigned int* intens, int size);

double amVarIntens(unsigned int* intens, int size, unsigned int median, double amVar_factor);

#endif /* REFLECTIONIDENTIFICATION_H_ */
