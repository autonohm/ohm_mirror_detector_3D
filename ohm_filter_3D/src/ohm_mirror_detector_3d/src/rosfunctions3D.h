/**
* @file   rosfunctions3D.cpp
* @author Rainer Koch
* @date   21.03.2016
*
*
*/
#ifndef ROSFKT3D_H
#define ROSFKT3D_H

/*
 * ros includes
 */
#include <ohm_hokuyo3d/CoordDist.h>
#include <ohm_hokuyo3d/CoordDistMulti.h>

#include "ros/time.h"

/*
 * mirror detector3D includes
 */
#include "Scan3D.h"
#include "Object3D.h"

using namespace std;
using namespace mirrordetector;

/**
 * copyHeaderScan
 * @param[in] ros msg header
 * @param[out] ros msg header
 */
void copyHeaderScan2Scan(ohm_hokuyo3d::CoordDistMulti scan_in, ohm_hokuyo3d::CoordDistMulti& scan_out);

/**
 * copyScanClass3D_MultiScan
 * @param[in] scan_in     scan3D class object input header
 * @param[out] scan_out   ros 3d msg with multiecho
 */
void copyScanClass3D_MultiScan(mirrordetector::scan3D& scan_in, ohm_hokuyo3d::CoordDistMulti& scan_out);

/*
 * copyScanClass3D_ScanSelected
 * @param[in] scan_in     scan3D class object input header
 * @param[in] objectType  of which points should be copied into the msg
 *
 * @param[out] scan_out   ros 3d msg without echoes
 */
void copyScanClass3D_ScanSelected(mirrordetector::scan3D& scan_in, ohm_hokuyo3d::CoordDist& scan_out, object3D::objType type);

//TODO: Not checked yet
/*
 * copyReprojectedScan
 * @param[in] scan_in     scan3D class object input header
 * @param[in] object      array of objects
 *
 * @param[out] scan_out   back projected ros 3d msg without echoes
 */
void copyReprojectedScan(mirrordetector::scan3D& scan_in, ohm_hokuyo3d::CoordDist& scan_out, mirrordetector::object3D& object);


#endif
