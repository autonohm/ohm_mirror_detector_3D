/**
* @file   testFunctions.h
* @author Rainer Koch
* @date   07.09.2016
*
*
*/

#ifndef TESTFUNCTIONS_H_
#define TESTFUNCTIONS_H_

/*
 * includes
 */
#include <cmath>
#include <stdlib.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>


/*
 * own includes
 */
#include "Object3D.h"
#include "Scan3D.h"


using namespace std;
using namespace mirrordetector;

/*
 * prints amount of each object Type in the array
 * @paramin: infotext : text which is printed as explanation/description
 */
void printObjectTypeAmounts(object3D::objType* objectTypes, int size, string infotext);

/*
 * prints corners
 * @paramin: infotext : text which is printed as explanation/description
 * @paramin: nr : number of corner
 *
 */
void printObjectCorners(double* corners, string infotext);
void printObjectCorners(double* corners, int nr);

/*
 * prints coordinates to file
 * @paramin: filename : name of file e.g. "/home/rainer/workspace/test.txt"
 */
void printCoords2File(double* coords, int size, const char* filename);
void printCoords2File(double* coords, bool* mask, int size, const char* filename);
void printCoords2File(double* coords, object3D::objType* mask, object3D::objType filter, int size, const char* filename);


/*
 * prints intensities to file
 * @paramin: filename : name of file e.g. "/home/rainer/workspace/test.txt"
 */
void printIntens2File(unsigned int* intens, int size, const char* filename);

/*
 * prints intensities on a 3D-surface
 * uses the coords to generate the xy-coordinates and the intensities as z-coordinate
 */
void printIntens2PlaneFile(double* coords, unsigned int* intens, int size, double* centroid, const char* filename);

#endif /* TESTFUNCTIONS_H_ */
