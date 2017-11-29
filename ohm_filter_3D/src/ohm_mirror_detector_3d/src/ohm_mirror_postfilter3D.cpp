/**
* @file   ohm_mirror_postfilter3D
* @author Rainer Koch
* @date   01.04.2016
*
*
*/

#include "ohm_mirror_postfilter3D.h"


using namespace std;
using namespace mirrordetector;

bool _new_dataset   = false;
bool _start_filter  = false;
bool _new_object    = false;
bool _activateFilters = false;

std::vector<mirrordetector::scan3D*> _scan_in(100);
std::vector<mirrordetector::object3D*> _object(1);


std::vector<unsigned int> _scan_nr_corrupted(100);

unsigned int _scan_counter = 0;                          // counts the amount of scans in _scan_in
unsigned int _scan_nr_corrupted_counter = 0;             // counts the amount of scans with corrupted points
unsigned int _object_size = 0;                           // counts the amount of identified objects
unsigned int _newContainerElements = 100;
unsigned int _lastScan = 0;                              // last scan number, when the publish message was received

bool _pub_Results          = false;

// distance filter
double _min_dist                    = 0.023;
double _max_dist                    = 60.0;
// box filter
double _box_min_x                   = -0.01;  // [m]
double _box_min_y                   = -0.01;  // [m]
double _box_min_z                   = -0.01;  // [m]
double _box_max_x                   = 0.01;   // [m]
double _box_max_y                   = 0.01;   // [m]
double _box_max_z                   = 0.01;   // [m]
//
////int _outlierFilter_kMean            = 100;
////double _outlierFilter_stdDeviation  = 0.1;
//int _outlierFilter_minNeighbors     = 10;
//double _outlierFilter_searchRadius  = 0.1;    // [m]
//double _substract_thres_dist        = 0.01;   // [m]
double _planeDetection_thres_dist   = 0.005;  // [m]
int _planeDetection_minPoints       = 200;
double _visionCone_thres            = 0.01;   // [m]
double _maxDistICP                  = 0.01;   // [m]
double _maxRotICP                   = 0.175;   // [rad]
double _fitnessICP                  = 0.005;
double _thres_mean                  = 0.55;
double _sizeVarFactor               = 0.2;
double _thres_variationInt          = 0.3;

unsigned int _normIntensWhPaper_rise    = 5260;
unsigned int _normIntensWhPaper_offset  = 2020;

//mirror variables
//vector<Point2f> _mirror_line_points;
//
//marker variables
visualization_msgs::Marker _marker_points;
visualization_msgs::Marker _marker_line_strip;
visualization_msgs::Marker _marker_line_list;

//
//
//// ROS variables
ros::Publisher _pub_scan;
ros::Publisher _pub_object;
ros::Publisher _pub_obj_Transp;
ros::Publisher _pub_obj_Mirror;
ros::Publisher _pub_affected;
ros::Publisher _pub_behTransp;
ros::Publisher _pub_behMirror;
ros::Publisher _pub_backprojMirror;
ros::Publisher _pub_marker;
ros::Publisher _pub_cleaned;
tf::TransformListener* _transformListener ;

//Totest
int testcounter = 0;
std::stringstream s;

/*
 * Subscriber & Publisher
 */
void subscriberFunc(const ohm_hokuyo3d::CoordDistMulti& msg)
{
  /*
   * get position of the scan from tf
   */
  tf::StampedTransform transform;
  ros::Time laser_timestamp = msg.coorddist[0].coord.header.stamp;
  unsigned long stamp       = msg.coorddist[0].coord.header.stamp.toNSec();

  bool gotTransformation = true;

  try
  {
    //wait till transform is avaiable
    if(_pub_Results)
    cout << "Wait for Transformation" << endl;
    _transformListener->waitForTransform("/map", "/3dscan", ros::Time::now(), ros::Duration(1.0));
    (*_transformListener).lookupTransform("/map", "/3dscan", laser_timestamp, transform);
    if(_pub_Results)
    cout << "Received transformation" << endl;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    gotTransformation = false;
  }

  if(gotTransformation) //(_input_scan.header.seq < msg.header.seq) &&
  {
    if(_pub_Results)
    {
      ROS_INFO("***********************************************************************");
      ROS_INFO("Mirror Detector 3D-Postfilter: Receive scan");
    }
    //_new_dataset = true;
    unsigned int fragmentsize = msg.coorddist[0].coord.fragmentsize;

    unsigned int sizeCoord    = 3*fragmentsize;
    unsigned int sizeOthers   = fragmentsize;
    unsigned int stepsPhi     = msg.coorddist[1].dist.phi.size();

    double* coord1            = new double[sizeCoord];
    double* coord2            = new double[sizeCoord];

//      //TOTEST:
//    double* coord1a            = new double[sizeCoord];

    double* normals1          = new double[sizeCoord];
    double* normals2          = new double[sizeCoord];

    unsigned int* intens1     = new unsigned int[sizeOthers];
    unsigned int* intens2     = new unsigned int[sizeOthers];

    bool* mask1               = new bool[sizeOthers];
    bool* mask2               = new bool[sizeOthers];

    double* dist1             = new double[sizeOthers];
    double* dist2             = new double[sizeOthers];

    double* phi1              = new double[sizeOthers];
    double* phi2              = new double[sizeOthers];

    mirrordetector::scan3D* scan_in_tmp = new mirrordetector::scan3D(
        msg.coorddist[0].coord.header.seq,
        msg.coorddist[0].coord.header.stamp.toNSec(),
        msg.coorddist[0].coord.header.frame_id,
        msg.coorddist[0].coord.fragmentid,
        msg.coorddist[0].coord.fragmentsize,
        msg.coorddist[0].coord.fragmentsize,
        msg.coorddist[0].dist.phi.size(),
        coord1,
        intens1,
        coord2,
        intens2);

    /*
     * calculate the tranformation matrix
     */

    obvious::Quaternion q(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
    obvious::Matrix R_tmp = q.Quaternion::convertToMatrix();
    obvious::Matrix Tr(4,4);
    Tr.setIdentity();
    obvious::Matrix R(4,4);
    R.setIdentity();
    obvious::Matrix T = scan_in_tmp->getTransformation();

    Tr(0,3) = transform.getOrigin().getX();
    Tr(1,3) = transform.getOrigin().getY();
    Tr(2,3) = transform.getOrigin().getY();

    R(0,0) = R_tmp(0,0);
    R(0,1) = R_tmp(0,1);
    R(0,2) = R_tmp(0,2);

    R(1,0) = R_tmp(1,0);
    R(1,1) = R_tmp(1,1);
    R(1,2) = R_tmp(1,2);

    R(2,0) = R_tmp(2,0);
    R(2,1) = R_tmp(2,1);
    R(2,2) = R_tmp(2,2);

    T = Tr*R;

    obvious::Matrix* T_tmp = new obvious::Matrix (4,4);
    T_tmp = &T;

    if(_pub_Results)
    {
      cout << "T: " << endl;
      T.print();
    }

//    cout << "Amount of points: " << sizeCoord << "/ " << sizeOthers << endl;
//    unsigned int seq          = msg.coorddist[0].coord.header.seq;
//    unsigned long stamp       = msg.coorddist[0].coord.header.stamp.toNSec();
//    string frame_id           = msg.coorddist[0].coord.header.frame_id;
//    unsigned int fragmentid   = msg.coorddist[0].coord.fragmentid;


//    cout << "fragmentsize " << fragmentsize << endl;
//    cout << "size coords1 " << msg.coorddist[0].coord.coords.size() << endl;
//    cout << "size coords2 " << msg.coorddist[1].coord.coords.size() << endl;
//    cout << "size normals1 " << msg.coorddist[0].coord.normals.size() << endl;
//    cout << "size normals2 " << msg.coorddist[1].coord.normals.size() << endl;
//    cout << "size intens 1 " << msg.coorddist[0].coord.intensity.size() << endl;
//    cout << "size intens 2 " << msg.coorddist[1].coord.intensity.size() << endl;
//    cout << "size obj 1 " << msg.coorddist[0].coord.objectType.size() << endl;
//    cout << "size obj 2 " << msg.coorddist[1].coord.objectType.size() << endl;
//    cout << "size dist 1 " << msg.coorddist[0].dist.dists.size() << endl;
//    cout << "size dist 2 " << msg.coorddist[1].dist.dists.size() << endl;
//    cout << "size phi 1 " << msg.coorddist[0].dist.phi.size() << endl;
//    cout << "size phi 2 " << msg.coorddist[1].dist.phi.size() << endl;

    object3D::objType* objectType1 = new object3D::objType[sizeOthers];
    object3D::objType* objectType2 = new object3D::objType[sizeOthers];

    obvious::Matrix* tmp_coord1 = new obvious::Matrix(4,1);
    (*tmp_coord1)(3,0)      = 1;
    obvious::Matrix* tmp_coord2 = new obvious::Matrix(4,1);
    (*tmp_coord2)(3,0)      = 1;
    obvious::Matrix* tmp_normals1 = new obvious::Matrix(4,1);
    (*tmp_normals1)(3,0)      = 1;
    obvious::Matrix* tmp_normals2 = new obvious::Matrix(4,1);
    (*tmp_normals2)(3,0)      = 1;

    int count_phi = 0;
    int m = 0;
    for(int i = 0; i < sizeOthers; i++)
    {
      mask1[i]        = msg.coorddist[0].coord.mask[i];
      mask2[i]        = msg.coorddist[1].coord.mask[i];

//      if(!isnan(msg.coorddist[0].coord.coords[3*i]) && !isnan(msg.coorddist[0].coord.coords[3*i+1]) && !isnan(msg.coorddist[0].coord.coords[3*i+2])
//          && !isnan(msg.coorddist[1].coord.coords[3*i]) && !isnan(msg.coorddist[1].coord.coords[3*i+1]) && !isnan(msg.coorddist[1].coord.coords[3*i+2])) // (mask1[i] == 1) && (mask2[i] == 1) &&
//      {
//      //TOTEST:
//        coord1a[3*i]       = msg.coorddist[0].coord.coords[3*i];
//        coord1a[3*i+1]     = msg.coorddist[0].coord.coords[3*i+1];
//        coord1a[3*i+2]     = msg.coorddist[0].coord.coords[3*i+2];

//        coord1[3*i]       = msg.coorddist[0].coord.coords[3*i];
//        coord1[3*i+1]     = msg.coorddist[0].coord.coords[3*i+1];
//        coord1[3*i+2]     = msg.coorddist[0].coord.coords[3*i+2];

      //Transform coords 1

        (*tmp_coord1)(0,0)      = msg.coorddist[0].coord.coords[3*i];
        (*tmp_coord1)(1,0)      = msg.coorddist[0].coord.coords[3*i+1];
        (*tmp_coord1)(2,0)      = msg.coorddist[0].coord.coords[3*i+2];

        *tmp_coord1 = T * (*tmp_coord1);

        coord1[3*i]       = (*tmp_coord1)(0,0);
        coord1[3*i+1]     = (*tmp_coord1)(1,0);
        coord1[3*i+2]     = (*tmp_coord1)(2,0);

//        coord1[3*i]       = msg.coorddist[1].coord.coords[3*i];
//        coord1[3*i+1]     = msg.coorddist[1].coord.coords[3*i+1];
//        coord1[3*i+2]     = msg.coorddist[1].coord.coords[3*i+2];

        //Transform coords 2

        (*tmp_coord2)(0,0)      = msg.coorddist[1].coord.coords[3*i];
        (*tmp_coord2)(1,0)      = msg.coorddist[1].coord.coords[3*i+1];
        (*tmp_coord2)(2,0)      = msg.coorddist[1].coord.coords[3*i+2];

        *tmp_coord2 = T * (*tmp_coord2);

        coord2[3*i]       = (*tmp_coord2)(0,0);
        coord2[3*i+1]     = (*tmp_coord2)(1,0);
        coord2[3*i+2]     = (*tmp_coord2)(2,0);

        intens1[i]        = (unsigned int)msg.coorddist[0].coord.intensity[i];
        intens2[i]        = (unsigned int)msg.coorddist[1].coord.intensity[i];

//        normals1[3*i]     = msg.coorddist[0].coord.normals[3*i];
//        normals1[3*i+1]   = msg.coorddist[0].coord.normals[3*i+1];
//        normals1[3*i+2]   = msg.coorddist[0].coord.normals[3*i+2];

        //Transform normals 1

        (*tmp_normals1)(0,0)      = msg.coorddist[0].coord.normals[3*i];
        (*tmp_normals1)(1,0)      = msg.coorddist[0].coord.normals[3*i+1];
        (*tmp_normals1)(2,0)      = msg.coorddist[0].coord.normals[3*i+2];

        *tmp_normals1 = T * (*tmp_normals1);

        normals1[3*i]       = (*tmp_normals1)(0,0);
        normals1[3*i+1]     = (*tmp_normals1)(1,0);
        normals1[3*i+2]     = (*tmp_normals1)(2,0);

//        normals2[3*i]     = msg.coorddist[1].coord.normals[3*i];
//        normals2[3*i+1]   = msg.coorddist[1].coord.normals[3*i+1];
//        normals2[3*i+2]   = msg.coorddist[1].coord.normals[3*i+2];

        //Transform normals 2

        (*tmp_normals2)(0,0)      = msg.coorddist[1].coord.normals[3*i];
        (*tmp_normals2)(1,0)      = msg.coorddist[1].coord.normals[3*i+1];
        (*tmp_normals2)(2,0)      = msg.coorddist[1].coord.normals[3*i+2];

        *tmp_normals2 = T * (*tmp_normals2);

        normals2[3*i]       = (*tmp_normals2)(0,0);
        normals2[3*i+1]     = (*tmp_normals2)(1,0);
        normals2[3*i+2]     = (*tmp_normals2)(2,0);

        //copy object types
        switch(msg.coorddist[0].coord.objectType[i])
        {
        case 0:
          objectType1[i] = object3D::unchecked;
          break;
        case 1:
          objectType1[i] = object3D::validPoint;
          break;
        case 2:
          objectType1[i] = object3D::errorSurface;
          break;
        case 3:
          objectType1[i] = object3D::behindErrorS;
          break;
        case 4:
          objectType1[i] = object3D::reflectiveSurface;
          break;
        case 5:
          objectType1[i] = object3D::behindReflective;
          break;
        case 6:
          objectType1[i] = object3D::transpSurface;
          break;
        case 7:
          objectType1[i] = object3D::behindTransparent;
          break;
        case 8:
          objectType1[i] = object3D::nanPoint;
          break;
        }
        switch(msg.coorddist[1].coord.objectType[i])
        {
        case 0:
          objectType2[i] = object3D::unchecked;
          break;
        case 1:
          objectType2[i] = object3D::validPoint;
          break;
        case 2:
          objectType2[i] = object3D::errorSurface;
          break;
        case 3:
          objectType2[i] = object3D::behindErrorS;
          break;
        case 4:
          objectType2[i] = object3D::reflectiveSurface;
          break;
        case 5:
          objectType2[i] = object3D::behindReflective;
          break;
        case 6:
          objectType2[i] = object3D::transpSurface;
          break;
        case 7:
          objectType2[i] = object3D::behindTransparent;
          break;
        case 8:
          objectType2[i] = object3D::nanPoint;
          break;
        }

        if((objectType2[i] != object3D::validPoint) and (objectType1[i] != object3D::nanPoint) and (objectType1[i] != object3D::unchecked))
        {
          _new_object = true;
//          if(_pub_Results)
//            ROS_INFO("Find mirror");
        }

        dist1[i]          = msg.coorddist[0].dist.dists[i];
        dist2[i]          = msg.coorddist[1].dist.dists[i];

        phi1[i]           = msg.coorddist[0].dist.phi[count_phi];
        phi2[i]           = msg.coorddist[1].dist.phi[count_phi];
//      }
//      else
//      {
//        coord1[3*i]     = NAN;
//        coord1[3*i+1]   = NAN;
//        coord1[3*i+2]   = NAN;
//
////      //TOTEST:
////        coord1a[3*i]       = NAN;
////        coord1a[3*i+1]     = NAN;
////        coord1a[3*i+2]     = NAN;
//
//        coord2[3*i]     = NAN;
//        coord2[3*i+1]   = NAN;
//        coord2[3*i+2]   = NAN;
//
//        intens1[i]      = 0;
//        intens2[i]      = 0;
//
//        normals1[i]     = NAN;
//        normals2[i]     = NAN;
//
//        objectType1[i]  = object3D::nanPoint;
//        objectType2[i]  = object3D::nanPoint;
//
//        dist1[i]        = NAN;
//        dist2[i]        = NAN;
//
//        phi1[i]         = NAN;
//        phi2[i]         = NAN;
//      }
      /*
       * only count up count_phi, after all scan points of one servo step are copied scanSize/stepsPhi = sizeHookuyo
       */
     m++;
      if (m == stepsPhi)
      {
        m = 0;
        count_phi++;
      }

    }
    //cout << endl;

    scan_in_tmp->setCoords1(coord1);
    scan_in_tmp->setCoords2(coord2);
    scan_in_tmp->setIntensity1(intens1);
    scan_in_tmp->setIntensity2(intens2);

    scan_in_tmp->setNormals1(normals1);
    scan_in_tmp->setMask1(mask1);
    scan_in_tmp->setDist1(dist1);
    scan_in_tmp->setPhi1(phi1);
    scan_in_tmp->setObjectType1(objectType1);

    scan_in_tmp->setNormals2(normals2);
    scan_in_tmp->setMask2(mask2);
    scan_in_tmp->setDist2(dist2);
    scan_in_tmp->setPhi2(phi2);
    scan_in_tmp->setObjectType2(objectType2);
    scan_in_tmp->setTransformation(T);


//    //TOTEST:
//    std::stringstream s;
//    s.str("");
//    s << "/home/rainer/workspace/post_in_sur_e1_" << _scan_counter << ".txt";
//    printCoords2File(scan_in_tmp->getCoords1(),scan_in_tmp->getObjectType1(), object3D::reflectiveSurface, scan_in_tmp->getSize(), s.str().c_str());
//    s.str("");
//    s << "/home/rainer/workspace/post_in_beh_e1_" << _scan_counter << ".txt";
//    printCoords2File(scan_in_tmp->getCoords1(), scan_in_tmp->getObjectType1(), object3D::behindReflective, scan_in_tmp->getSize(), s.str().c_str());
//    s.str("");
//    s << "/home/rainer/workspace/post_in_val_e1_" << _scan_counter << ".txt";
//    printCoords2File(scan_in_tmp->getCoords1(), scan_in_tmp->getObjectType1(), object3D::validPoint, scan_in_tmp->getSize(), s.str().c_str());
//    s.str("");
//    s << "/home/rainer/workspace/post_in_sur_e2_" << _scan_counter << ".txt";
//    printCoords2File(scan_in_tmp->getCoords2(),scan_in_tmp->getObjectType2(), object3D::reflectiveSurface, scan_in_tmp->getSize(), s.str().c_str());
//    s.str("");
//    s << "/home/rainer/workspace/post_in_beh_e2_" << _scan_counter << ".txt";
//    printCoords2File(scan_in_tmp->getCoords2(), scan_in_tmp->getObjectType2(), object3D::behindReflective, scan_in_tmp->getSize(), s.str().c_str());
//    s.str("");
//    s << "/home/rainer/workspace/post_in_val_e2_" << _scan_counter << ".txt";
//    printCoords2File(scan_in_tmp->getCoords2(), scan_in_tmp->getObjectType2(), object3D::validPoint, scan_in_tmp->getSize(), s.str().c_str());
//    cout << "print post in final files" << endl;
//    //ENDTOTEST:
//
//
//    //TOTEST:
//    std::stringstream s2;
//    s2.str("");
//    s2 << "/home/rainer/workspace/post_before_filter_e1_" << _scan_counter << ".txt";
//    printCoords2File(scan_in_tmp->getCoords1(), scan_in_tmp->getSize(), s2.str().c_str());
//    s2.str("");
//    s2 << "/home/rainer/workspace/post_before_filter_e2_" << _scan_counter << ".txt";
//    printCoords2File(scan_in_tmp->getCoords2(), scan_in_tmp->getSize(), s2.str().c_str());
//    //ENDTOTEST:


    if(_activateFilters)
    {
      if(_pub_Results)
        ROS_INFO("Postfilter - Filters activated");
      scan_in_tmp->distanceThresFilter(_min_dist, _max_dist);
      scan_in_tmp->boxFilter(_box_min_x, _box_max_x, _box_min_y, _box_max_y, _box_min_z, _box_max_z);
    }

//    //TOTEST:
//    s2.str("");
//    s2 << "/home/rainer/workspace/post_after_filter_e1_" << _scan_counter << ".txt";
//    printCoords2File(scan_in_tmp->getCoords1(), scan_in_tmp->getSize(), s2.str().c_str());
//    s2.str("");
//    s2 << "/home/rainer/workspace/post_after_filter_e2_" << _scan_counter << ".txt";
//    printCoords2File(scan_in_tmp->getCoords2(), scan_in_tmp->getSize(), s2.str().c_str());
//    //ENDTOTEST:


    /*
     * check if history containers need to be resized
     */
    if(_scan_in.size() == _scan_counter)
      _scan_in.resize(_scan_counter + _newContainerElements);
    if(_scan_nr_corrupted.size() == _scan_nr_corrupted_counter)
      _scan_nr_corrupted.resize(_scan_nr_corrupted_counter + _newContainerElements);

    /*
     * copy scan into history containers
     */
    _scan_in[_scan_counter] = scan_in_tmp;

    if(_new_object)
    {
      _scan_nr_corrupted[_scan_nr_corrupted_counter] = _scan_counter;
      _scan_nr_corrupted_counter++;
    }
    _scan_counter++;

//    //TEST
//    cout << "scan_in_tmp: " <<  scan_in_tmp->getSize() << endl;
//    double* coortest            = new double[scan_in_tmp->getSize()];
//    coortest = scan_in_tmp->getDist1();
//    for (int i=0; i < 10; i++)
//    {
//      cout << coortest[i] << " ";
//    }
//    cout << endl;
//    cout << "scan_in: " << _scan_in[0]->getSize() << endl;
//    double* coord1test            = new double[_scan_in[0]->getSize()];
//    coord1test = _scan_in[0]->getDist1();
//    for (int i=0; i < 10; i++)
//    {
//      cout << coord1test[i] << " ";
//    }
//    cout << endl;

      if(_pub_Results)
      {
        cout << "Scan counter / Scan corrupted counter: " << _scan_counter << " / " << _scan_nr_corrupted_counter << endl;
      }
  }
}

void subscriberActivatePub(std_msgs::Bool activate)
{
  if(_pub_Results)
  ROS_INFO("************************* Post-Filter activated *************************");
  _start_filter = true;
  _lastScan = _scan_counter;
}

void pubPoints()
{
  _marker_points.header.frame_id = "map";
  _marker_points.header.stamp = ros::Time();
  _marker_points.ns = "points";
  _marker_points.action = visualization_msgs::Marker::MODIFY;
  _marker_points.pose.orientation.w = 1.0;
  _marker_points.type = visualization_msgs::Marker::POINTS;
  _marker_points.id = 0;
  _marker_points.lifetime = ros::Duration(0, 1);

  _pub_marker.publish(_marker_points);

}

void pubLines()
{
  _marker_line_strip.header.frame_id = "map";
  _marker_line_strip.header.stamp = ros::Time();
  _marker_line_strip.ns = "lines";
  _marker_line_strip.action = visualization_msgs::Marker::MODIFY;
  _marker_line_strip.pose.orientation.w = 1.0;
  _marker_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  _marker_line_strip.id = 1;

  _pub_marker.publish(_marker_line_strip);
}

void publisherFunc(void)
{

  double* object_corners;

  ohm_hokuyo3d::CoordDistMulti msg_scan;
  ohm_hokuyo3d::CoordDist      msg_object;
  ohm_hokuyo3d::CoordDist      msg_obj_Transp;
  ohm_hokuyo3d::CoordDist      msg_obj_Mirror;
  ohm_hokuyo3d::CoordDist      msg_affected;
  ohm_hokuyo3d::CoordDist      msg_behTransp;
  ohm_hokuyo3d::CoordDist      msg_behMirror;
  ohm_hokuyo3d::CoordDist      msg_backprojMirror;
  ohm_hokuyo3d::CoordDist      msg_cleaned;


  for(unsigned int l=0; l < _lastScan; l++)
  {
    double* scan_points1       = _scan_in[l]->getCoords1();
//    for(int i = 0; i < _scan_in[l]->getSize(); i++)
//    {
//     cout << " " << scan_points1[3*i];
//    }
//    cout << endl;

    copyScanClass3D_MultiScan(*_scan_in[l], msg_scan);

    copyScanClass3D_ScanSelected(*_scan_in[l], msg_cleaned, object3D::validPoint);   // points with are free of influences

//    double* scan_points2      = _scan_in[l]->getCoords1();
//    for(int i = 0; i < _scan_in[l]->getSize(); i++)
//    {
//     cout << " " << msg_scan.coorddist[0].coord.coords[3*i];
//    }
//    cout << endl;

    /*
     * if object was found, copy data of object, affected data and cleand scan data
     */
    if(_object_size > 0)
    {
      //cout << "publish surface: " << endl;
      copyScanClass3D_ScanSelected(*_scan_in[l], msg_object, object3D::errorSurface);   // points which are located on the object
      copyScanClass3D_ScanSelected(*_scan_in[l], msg_obj_Mirror, object3D::reflectiveSurface);   // points which are located on the object
      copyScanClass3D_ScanSelected(*_scan_in[l], msg_obj_Transp, object3D::transpSurface);   // points which are located on the object

      //cout << "publish error: " << endl;
      copyScanClass3D_ScanSelected(*_scan_in[l], msg_affected, object3D::behindErrorS);             // points which are influenced by the object
      copyScanClass3D_ScanSelected(*_scan_in[l], msg_behMirror, object3D::behindReflective);        // points which are influenced by the object
      copyScanClass3D_ScanSelected(*_scan_in[l], msg_behTransp, object3D::behindTransparent);       // points which are influenced by the object

      /*
       * write corner data of object
       */
      for(int i=0; i < _object_size; i++)
      {
        object_corners = _object[i]->getObjectCorners();

        if(_pub_Results)
        {
        double* tmp1 = new double[3];
        double* tmp2 = new double[3];
        tmp1[0] = object_corners[0];
        tmp1[1] = object_corners[1];
        tmp1[2] = object_corners[2];

        tmp2[0] = object_corners[3];
        tmp2[1] = object_corners[4];
        tmp2[2] = object_corners[5];
        double a = calcVectorLength(tmp1, tmp2);
        tmp1[0] = object_corners[3];
        tmp1[1] = object_corners[4];
        tmp1[2] = object_corners[5];

        tmp2[0] = object_corners[6];
        tmp2[1] = object_corners[7];
        tmp2[2] = object_corners[8];
        double b = calcVectorLength(tmp1, tmp2);


        cout << "object:" << i << " size: " << a << "/" << b << endl;
        cout << object_corners[0] << " " << object_corners[1] << " " <<object_corners[2] << endl;
        cout << object_corners[3] << " " << object_corners[4] << " " <<object_corners[5] << endl;
        cout << object_corners[6] << " " << object_corners[7] << " " <<object_corners[8] << endl;
        cout << object_corners[9] << " " << object_corners[10] << " " <<object_corners[11] << endl;
        cout << endl;
        }

        geometry_msgs::Point p;
        p.x = object_corners[0];
        p.y = object_corners[1];
        p.z = object_corners[2];
        _marker_points.points.push_back(p);
        _marker_line_strip.points.push_back(p);
        p.x = object_corners[3];
        p.y = object_corners[4];
        p.z = object_corners[5];
        _marker_points.points.push_back(p);
        _marker_line_strip.points.push_back(p);
        p.x = object_corners[6];
        p.y = object_corners[7];
        p.z = object_corners[8];
        _marker_points.points.push_back(p);
        _marker_line_strip.points.push_back(p);
        p.x = object_corners[9];
        p.y = object_corners[10];
        p.z = object_corners[11];
        _marker_points.points.push_back(p);
        _marker_line_strip.points.push_back(p);
        pubPoints();
        pubLines();
      }
    }

    /*
     * publish scans
     */
    _pub_scan.publish(msg_scan);
   _pub_cleaned.publish(msg_cleaned);

    if(_object_size > 0)
    {
      _pub_object.publish(msg_object);
      _pub_obj_Transp.publish(msg_obj_Transp);
      _pub_obj_Mirror.publish(msg_obj_Mirror);

      _pub_affected.publish(msg_affected);
      _pub_behTransp.publish(msg_behTransp);
      _pub_behMirror.publish(msg_behMirror);
      _pub_backprojMirror.publish(msg_backprojMirror);

    }
  }
}

/*
 * other functions
 */
//
void separateAffectedPoints(std::vector<mirrordetector::scan3D*> scan, unsigned int size_scanHistory, unsigned int size_corrupted, std::vector<unsigned int> scan_nr_corrupted, mirrordetector::scan3D*& corrupted_scans)
{
  unsigned int size_points_corrupted  = 0;        // amount of points in corrupted scans
  /*
   * determine size of tmp_scan
   */
  for(unsigned int i = 0; i < size_corrupted; i++)
  {
    size_points_corrupted += scan[scan_nr_corrupted[i]]->getSize();
  }

  //TOSHOW:
  if(_pub_Results)
  {
    cout << "Total size of points: " << size_points_corrupted << endl;
    cout << "Size scans/corrupted: " << size_scanHistory << " / " << size_corrupted << endl;
  }

  /*
   * copy history in scan to work with
   */
 // std::vector<mirrordetector::scan3D*> scan_work(1);

  unsigned int seq          = scan[0]->getSeqenz();
  unsigned long stamp       = scan[0]->getTimeStamp();
  string frame_id           = scan[0]->getFrame_id();
  unsigned int fragmentid   = scan[0]->getFragment_id();
  unsigned int fragmentsize = scan[0]->getFragment_size();

  unsigned int sizeCoord    = 3*size_points_corrupted;
  unsigned int stepsPhi     = size_points_corrupted;

  // to store all points
  double* tmp_coord1            = new double[3*size_points_corrupted];
  double* tmp_coord2            = new double[3*size_points_corrupted];
  double* tmp_normals1          = new double[3*size_points_corrupted];
  double* tmp_normals2          = new double[3*size_points_corrupted];
  unsigned int* tmp_intens1     = new unsigned int[size_points_corrupted];
  unsigned int* tmp_intens2     = new unsigned int[size_points_corrupted];
  bool* tmp_mask1               = new bool[size_points_corrupted];
  bool* tmp_mask2               = new bool[size_points_corrupted];
  double* tmp_dist1             = new double[size_points_corrupted];
  double* tmp_dist2             = new double[size_points_corrupted];
  double* tmp_phi1              = new double[size_points_corrupted];
  double* tmp_phi2              = new double[size_points_corrupted];

  object3D::objType* tmp_objectType1 = new object3D::objType[size_points_corrupted];
  object3D::objType* tmp_objectType2 = new object3D::objType[size_points_corrupted];

  // to store points of one scan
  double* coord1;
  double* coord2;
  double* normals1;
  double* normals2;
  unsigned int* intens1;
  unsigned int* intens2;
  bool* mask1;
  bool* mask2;
  double* dist1;
  double* dist2;
  double* phi1;
  double* phi2;
  object3D::objType* objectType1;
  object3D::objType* objectType2;

  unsigned int k = 0;
  unsigned int l = 0;

  unsigned int amount_affected  = 0;
  unsigned int amount_onSurf    = 0;
  unsigned int amount_behSurf   = 0;

  for(unsigned int i = 0; i < size_scanHistory; i++)
  {
    if(i == scan_nr_corrupted[l])
    {
        /*
         * get all points of this scan
         */
        coord1      = scan[i]->getCoords1();
        coord2      = scan[i]->getCoords2();
        normals1    = scan[i]->getNormals1();
        normals2    = scan[i]->getNormals2();
        intens1     = scan[i]->getIntensity1();
        intens2     = scan[i]->getIntensity2();
        mask1       = scan[i]->getMask1();
        mask2       = scan[i]->getMask2();
        dist1       = scan[i]->getDist1();
        dist2       = scan[i]->getDist2();
        phi1        = scan[i]->getPhi1();
        phi2        = scan[i]->getPhi2();
        objectType1 = scan[i]->getObjectType1();
        objectType2 = scan[i]->getObjectType2();

      /*
       * add all points of this scan to the rest
       */
      for(unsigned int j = 0; j < scan[i]->getSize(); j++)
      {
        tmp_coord1[3*k]     = coord1[3*j];
        tmp_coord1[3*k+1]   = coord1[3*j+1];
        tmp_coord1[3*k+2]   = coord1[3*j+2];

        tmp_coord2[3*k]     = coord2[j];
        tmp_coord2[3*k+1]   = coord2[j+2];
        tmp_coord2[3*k+1]   = coord2[j+2];

        tmp_normals1[3*k]   = normals1[j];
        tmp_normals1[3*k+1] = normals1[j+2];
        tmp_normals1[3*k+1] = normals1[j+2];

        tmp_normals2[3*k]   = normals2[j];
        tmp_normals2[3*k+1] = normals2[j+2];
        tmp_normals2[3*k+1] = normals2[j+2];

        tmp_intens1[k]      = intens1[j];
        tmp_intens2[k]      = intens2[j];
        tmp_mask1[k]        = mask1[j];
        tmp_mask2[k]        = mask2[j];
        tmp_dist1[k]        = dist1[j];
        tmp_dist2[k]        = dist2[j];
        tmp_phi1[k]         = phi1[j];
        tmp_phi2[k]         = phi2[j];


        if((objectType1[j] == object3D::errorSurface) or (objectType1[j] == object3D::transpSurface) or (objectType1[j] == object3D::reflectiveSurface))
        {
          tmp_objectType1[k]  = object3D::errorSurface;
        }
        else if((objectType1[j] == object3D::behindErrorS) or (objectType1[j] == object3D::behindReflective) or (objectType1[j] == object3D::behindTransparent))
        {
          tmp_objectType1[k]  = object3D::behindErrorS;
        }
        else
        {
          tmp_objectType1[k]  = objectType1[j];
        }


        if((objectType2[j] == object3D::errorSurface) or (objectType2[j] == object3D::transpSurface) or (objectType2[j] == object3D::reflectiveSurface))
        {
          tmp_objectType2[k]  = object3D::errorSurface;
        }
        else if((objectType2[j] == object3D::behindErrorS) or (objectType2[j] == object3D::behindReflective) or (objectType2[j] == object3D::behindTransparent))
        {
          tmp_objectType2[k]  = object3D::behindErrorS;
        }
        else
        {
          tmp_objectType2[k]  = objectType2[j];
        }


        /*
         * count amount of affected
         */
        if((objectType1[j] == object3D::errorSurface) or (objectType1[j] == object3D::transpSurface) or (objectType1[j] == object3D::reflectiveSurface) or (objectType1[j] == object3D::behindErrorS) or (objectType1[j] == object3D::behindReflective) or (objectType1[j] == object3D::behindTransparent))
         amount_affected++;

        if((objectType1[j] == object3D::errorSurface) or (objectType1[j] == object3D::transpSurface) or (objectType1[j] == object3D::reflectiveSurface))
          amount_onSurf++;
        if((objectType1[j] == object3D::behindErrorS) or (objectType1[j] == object3D::behindReflective) or (objectType1[j] == object3D::behindTransparent))
          amount_behSurf++;

        k++;
      }
      // next corrupted
      l++;
    }
  }
//  if(_pub_Results)
//    cout << "affected = on / behind " << amount_affected << " = "  << amount_onSurf << " / " << amount_behSurf << endl;

//  //TOTOEST:
//  cout << "print objects" << endl;
//  int tm= 0;
//  for(int i=0; i < size_points_corrupted; i++)
//  {
//    //cout << tmp_objectType1[i] << "/";
//    if (tmp_objectType1[i] == object3D::errorSurface)
//    {
//      tm++;
//    }
//  }
//  cout << endl;
//  cout << "error S1 " << tm << endl;
//  cout << "amount total / affected points " << size_points_corrupted << " / " << amount_affected << endl;
//  //ENDTOTEST:

  mirrordetector::scan3D* scan_tmp = new mirrordetector::scan3D(
      seq,
      stamp,
      frame_id,
      fragmentid,
      fragmentsize,
      size_points_corrupted,
      stepsPhi,
      tmp_coord1,
      tmp_intens1,
      tmp_coord2,
      tmp_intens2);

  scan_tmp->setNormals1(tmp_normals1);
  scan_tmp->setMask1(tmp_mask1);
  scan_tmp->setDist1(tmp_dist1);
  scan_tmp->setPhi1(tmp_phi1);
  scan_tmp->setObjectType1(tmp_objectType1);

  scan_tmp->setNormals2(tmp_normals2);
  scan_tmp->setMask2(tmp_mask2);
  scan_tmp->setDist2(tmp_dist2);
  scan_tmp->setPhi2(tmp_phi2);
  scan_tmp->setObjectType2(tmp_objectType2);

  corrupted_scans = scan_tmp;

//  cout << "cor " << endl;
//  printCoords2File(corrupted_scans->getCoords1(), corrupted_scans->getSize(), "/home/rainer/workspace/c1.txt");
//  cout << "cor done" << endl;


//  /*
//   * clean up
//   */
//  double* tmp_coord1            = new double[3*size_points_corrupted];
//  double* tmp_coord2            = new double[3*size_points_corrupted];
//  double* tmp_normals1          = new double[3*size_points_corrupted];
//  double* tmp_normals2          = new double[3*size_points_corrupted];
//  unsigned int* tmp_intens1     = new unsigned int[size_points_corrupted];
//  unsigned int* tmp_intens2     = new unsigned int[size_points_corrupted];
//  bool* tmp_mask1               = new bool[size_points_corrupted];
//  bool* tmp_mask2               = new bool[size_points_corrupted];
//  double* tmp_dist1             = new double[size_points_corrupted];
//  double* tmp_dist2             = new double[size_points_corrupted];
//  double* tmp_phi1              = new double[size_points_corrupted];
//  double* tmp_phi2              = new double[size_points_corrupted];
//
//  object3D::objType* tmp_objectType1 = new object3D::objType[size_points_corrupted];
//  object3D::objType* tmp_objectType2 = new object3D::objType[size_points_corrupted];

}

void cleanScan(std::vector<mirrordetector::scan3D*> scan, std::vector<mirrordetector::object3D*> object, unsigned int amount_objects, double thres_dist, double thres_visionCone)
{
  double* object_corners;
  double* object_coefficents;

  for(unsigned int l=0; l < _lastScan; l++)
  {
    double* scan_points1       = scan[l]->getCoords1();
    double* scan_points2       = scan[l]->getCoords2();

    int scanSize              = scan[l]->getSize();

    object3D::objType* tmp_objectType1 = new object3D::objType[scanSize];
    object3D::objType* tmp_objectType2 = new object3D::objType[scanSize];

    object3D::objType planeAssignedObjectType = object3D::unchecked;

    // TOTEST
  //  object3D::objType* tmp_objectType4 = scan[0]->getObjectType1();
  //  printObjectTypeAmounts(tmp_objectType4, scanSize, "input");
    for(int i=0; i < scanSize; i++)
    {
      tmp_objectType1[i] = object3D::unchecked;
      tmp_objectType2[i] = object3D::unchecked;
    }

    std::vector<int*> locationPoint1(amount_objects);
    std::vector<int*> locationPoint2(amount_objects);

    /*
     * clean scan
     */
    for(int i=0; i < amount_objects; i++) // amount_objects
    {
      locationPoint1[i] = new int[scanSize];
      locationPoint2[i] = new int[scanSize];

      object_corners      = object[i]->getObjectCorners();
      object_coefficents  = object[i]->getCoefficents();
  //    // TOTEST:
  //    cout << object_corners[0] << "/" << object_corners[1] << "/" << object_corners[2] << endl;
  //    cout << object_corners[3] << "/" << object_corners[4] << "/" << object_corners[5] << endl;
  //    cout << object_corners[6] << "/" << object_corners[7] << "/" << object_corners[8] << endl;
  //    cout << object_corners[9] << "/" << object_corners[10] << "/" << object_corners[11] << endl;

      /*
       * define location of point
       * (0 = valid point, 1 = on surface, 2 = behind surface)
       */
      pointLocation(scan_points1, scanSize, object_corners, object_coefficents, thres_dist, thres_visionCone, locationPoint1[i]);
      pointLocation(scan_points2, scanSize, object_corners, object_coefficents, thres_dist, thres_visionCone, locationPoint2[i]);

    }
    /*
     * assign objectType of point
     */
    for(int i=0; i < amount_objects; i++) // amount_objects
    {
      planeAssignedObjectType = object[i]->getObjectType();

      for(int n=0; n < scanSize; n++)
      {
        /*
         * assign unknown objects
         */
          /*
           * assign objectType1
           */
          // if point is already assigned from another object as a errorSurface point or an error point => do not change the objectType
          if((locationPoint1[i][n] == 0) && ((tmp_objectType1[n] == object3D::unchecked) or (tmp_objectType1[n] == object3D::validPoint)))
          {
            tmp_objectType1[n] = object3D::validPoint;  // valid point
          }
          else if(locationPoint1[i][n] == 1)
          {
            if(planeAssignedObjectType == object3D::unchecked)
              tmp_objectType1[n] = object3D::errorSurface;   // other points
            if(planeAssignedObjectType == object3D::reflectiveSurface)
              tmp_objectType1[n] = object3D::reflectiveSurface;
            if(planeAssignedObjectType == object3D::transpSurface)
              tmp_objectType1[n] = object3D::transpSurface;
          }
          else if(locationPoint1[i][n] == 2)
          {
            if(planeAssignedObjectType == object3D::unchecked)
              tmp_objectType1[n] = object3D::behindErrorS;   // other points
            if(planeAssignedObjectType == object3D::reflectiveSurface)
              tmp_objectType1[n] = object3D::behindReflective;
            if(planeAssignedObjectType == object3D::transpSurface)
              tmp_objectType1[n] = object3D::behindTransparent;
          }
          // else leave old value

          /*
           * assign objectType2
           */
          // if point is already assigned from another object as a errorSurface point or an error point => do not change the objectType
          if((locationPoint2[i][n] == 0) && ((tmp_objectType2[n] == object3D::unchecked) or (tmp_objectType2[n] == object3D::validPoint)))
          {
            tmp_objectType2[n] = object3D::validPoint;  // valid point
          }
          else if(locationPoint2[i][n] == 1)
          {
            if(planeAssignedObjectType == object3D::unchecked)
              tmp_objectType2[n] = object3D::errorSurface;   // other points
            if(planeAssignedObjectType == object3D::reflectiveSurface)
              tmp_objectType2[n] = object3D::reflectiveSurface;
            if(planeAssignedObjectType == object3D::transpSurface)
              tmp_objectType2[n] = object3D::transpSurface;
          }
          else if(locationPoint2[i][n] == 2)
          {
            if(planeAssignedObjectType == object3D::unchecked)
              tmp_objectType2[n] = object3D::behindErrorS;   // other points
            if(planeAssignedObjectType == object3D::reflectiveSurface)
              tmp_objectType2[n] = object3D::behindReflective;
            if(planeAssignedObjectType == object3D::transpSurface)
              tmp_objectType2[n] = object3D::behindTransparent;
          }
          // else leave old value
      }
    }

    scan[l]->setObjectType1(tmp_objectType1);
    scan[l]->setObjectType2(tmp_objectType2);

  //  printCoords2File(scan_points1, tmp_objectType1, object3D::validPoint, scanSize, "/home/rainer/workspace/valid.txt");
  //  printCoords2File(scan_points1, tmp_objectType1, object3D::behindErrorS, scanSize, "/home/rainer/workspace/behind.txt");
  //  printCoords2File(scan_points1, tmp_objectType1, object3D::errorSurface, scanSize, "/home/rainer/workspace/on.txt");


    //TOTEST:
    //printObjectTypeAmounts(tmp_objectType1, scanSize, "changed");

  //   //TOTEST:
  //    object3D::objType* tmp_objectType3 = scan[0]->getObjectType1();
  //    printObjectTypeAmounts(tmp_objectType3, scanSize, "reread");
  }
}


/*
 * Init
 */
void init()
{
  // config marker
    // POINTS markers use x and y scale for width/height respectively
    _marker_points.scale.x = 0.05;
    _marker_points.scale.y = 0.05;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    _marker_line_strip.scale.x = 0.02;
    _marker_line_list.scale.x = 0.02;
    // Points are light blue
    _marker_points.color.r = 0.0;
    _marker_points.color.g = 1.0;
    _marker_points.color.b = 1.0;
    _marker_points.color.a = 1.0;
    // Line strip is red
    _marker_line_strip.color.r = 1.0;
    _marker_line_strip.color.g = 0.0;
    _marker_line_strip.color.b = 0.0;
    _marker_line_strip.color.a = 1.0;
    // Line list is red
    _marker_line_list.color.r = 1.0;
    _marker_line_list.color.g = 0.0;
    _marker_line_list.color.b = 0.0;
    _marker_line_list.color.a = 1.0;

  //ROS_INFO("Init viewer intensity");
  //ROS_INFO("Running with threshold %f", _substract_threshold_dist);
}

int main(int argc, char **argv)
{
  double dVar = 0;
  int iVar    = 0;
  bool bVar   = false;

  ros::init(argc, argv, "ohm_echo_filter");
  ros::NodeHandle nh_sub("~");
  ros::NodeHandle nh_pub("~");
//
  /*
   * Parameters for launch file
   */
  std::string sub_inputScan;
  std::string sub_activatePub;

  std::string pub_scan;
  std::string pub_object;
  std::string pub_transp;
  std::string pub_mirror;
  std::string pub_affected;
  std::string pub_behTransp;
  std::string pub_behMirror;
  std::string pub_backprojMirror;
  std::string pub_cleaned;

  std::string pub_marker;

  /*
   * sub and publisher
   */
  nh_sub.param<std::string>("sub_input_scan",           sub_inputScan,            "scan3d_prefiltered");
  nh_sub.param<std::string>("sub_activatePub",          sub_activatePub,          "activatePub");                // subscriber to start filter

  nh_pub.param<std::string>("pub_scan",                 pub_scan,                 "post_scan3d_filtered");       // prefiltered  3D scan

  nh_pub.param<std::string>("pub_object",               pub_object,               "post_object");                // points assigned to object
  nh_pub.param<std::string>("pub_transp",               pub_transp,               "post_obj_transp");            // points assigned to a transparent object
  nh_pub.param<std::string>("pub_mirror",               pub_mirror,               "post_obj_mirror");            // points assigned to a mirror

  nh_pub.param<std::string>("pub_affected",             pub_affected,             "post_affected");              // points assigned to be affected by the object
  nh_pub.param<std::string>("pub_behTransp",            pub_behTransp,            "post_behTransp");             // points assigned to be behind a transparent object
  nh_pub.param<std::string>("pub_behMirror",            pub_behMirror,            "post_behMirror");             // points assigned to be behind a mirror object
  nh_pub.param<std::string>("pub_backprojMirror",       pub_backprojMirror,       "post_backprojMirror");        // points assigned to be behind a mirror object and been back projected to their origin

  nh_pub.param<std::string>("pub_cleaned",              pub_cleaned,              "post_cleaned");               // points marked by their object type

  nh_pub.param<std::string>("pub_marker",               pub_marker,               "marker");                // maker to show in rviz

  nh_sub.param<int>("newContainerElements",                       iVar, 100);         // amount of new allocated container elements
  _newContainerElements = static_cast<unsigned int>(iVar);

  /*
   * variables for post-filter
   */
//  nh_sub.param<int>("outlierFilter_minNeighbors",                 iVar, 10);            // minimal number of neighbors
//  _outlierFilter_minNeighbors = static_cast<int>(iVar);
//  nh_sub.param<double>("outlierFilter_searchRadius",              dVar, 0.1);           // search radius for neighbors
//  _outlierFilter_searchRadius = static_cast<float>(dVar);

  /*
   * variables for object detection
   */
//  nh_sub.param<double>("substract_threshold_distance",            dVar, 0.05);         // allowed distance between first and last echo [m]
//  _substract_thres_dist = static_cast<float>(dVar);

  nh_sub.param<bool>("publish_Results",                          bVar, false);            // show the results of the pre-filter
  _pub_Results = static_cast<bool>(bVar);

  nh_sub.param<bool>("activateFilters",                          bVar, true);            // activate Filters ( distance-, box-filter)
  _activateFilters = static_cast<bool>(bVar);

  nh_sub.param<double>("min_measure_distance",                    dVar, 0.023);       // minimal distance of a point
  _min_dist = static_cast<double>(dVar);
  nh_sub.param<double>("max_measure_distance",                    dVar, 60.0);        // maximal distance of a point
  _max_dist = static_cast<double>(dVar);

  nh_sub.param<double>("box_min_x_distance",                      dVar, -0.01);       // minimal x distance of box filter
  _box_min_x = static_cast<double>(dVar);
  nh_sub.param<double>("box_min_y_distance",                      dVar, -0.01);       // minimal y distance of box filter
  _box_min_y = static_cast<double>(dVar);
  nh_sub.param<double>("box_min_z_distance",                      dVar, -0.01);       // minimal z distance of box filter
  _box_min_z = static_cast<double>(dVar);
  nh_sub.param<double>("box_max_x_distance",                      dVar, 0.01);        // maximal x distance of box filter
  _box_max_x = static_cast<double>(dVar);
  nh_sub.param<double>("box_max_y_distance",                      dVar, 0.01);        // maximal y distance of box filter
  _box_max_y = static_cast<double>(dVar);
  nh_sub.param<double>("box_max_z_distance",                      dVar, 0.01);        // maximal z distance of box filter


  nh_sub.param<double>("planeDetection_threshold_distance",       dVar, 0.005);         // threshold, if point is assigned from pcl to the plane or not [m]
  _planeDetection_thres_dist = static_cast<float>(dVar);
  nh_sub.param<int>("planeDetection_minAmountPoints",             iVar, 200);           // threshold, how many points are needed to detect a plane
  _planeDetection_minPoints = static_cast<int>(iVar);

  nh_sub.param<double>("visionCone_threshold",                    dVar, 0.05);          // additional room around the object plane (extending object plane) [m]
  _visionCone_thres = static_cast<float>(dVar);


  nh_sub.param<double>("max_Dist_ICP",                            dVar, 0.01);          // max. allowed distance between matching points of ICP [m]
  _maxDistICP = static_cast<float>(dVar);
  nh_sub.param<double>("max_Rot_ICP",                             dVar, 0.175);          // max. allowed rotation between matching points of ICP [rad]
  _maxRotICP = static_cast<float>(dVar);
  nh_sub.param<double>("fitness_Fct_ICP",                         dVar, 0.005);          // max. allowed result of fitness function of ICP
  _fitnessICP = static_cast<float>(dVar);
  nh_sub.param<double>("threshold_mean_intens_factor",            dVar, 0.55);           // value do determine mean intensity factor
  _thres_mean = static_cast<float>(dVar);
  nh_sub.param<double>("sizeVarFactor",                           dVar, 0.2);           // value to determine max. amount of variation of intensity to count as median or not
  _sizeVarFactor = static_cast<float>(dVar);
  nh_sub.param<double>("threshold_variation_intensity",           dVar, 0.3);           // threshold to determine if object has a high variation or not
  _thres_variationInt = static_cast<float>(dVar);

  nh_sub.param<int>("intensity_wh_Paper_rise",                    iVar, 5270);           // intensity of white paper curve rising
  _normIntensWhPaper_rise = static_cast<int>(iVar);
  nh_sub.param<int>("intensity_wh_Paper_offset",                  iVar, 2020);           // intensity of white paper curve offset
  _normIntensWhPaper_offset = static_cast<int>(iVar);

  /*
   * initialize subscriber
   */
  ros::Subscriber sub   = nh_sub.subscribe(sub_inputScan, 2, subscriberFunc);
  ros::Subscriber sub1   = nh_sub.subscribe(sub_activatePub, 2, subscriberActivatePub);

  /*
   * initialize publisher
   */
  _pub_scan               = nh_pub.advertise<ohm_hokuyo3d::CoordDistMulti>(pub_scan, 1);
  _pub_object             = nh_pub.advertise<ohm_hokuyo3d::CoordDist>(pub_object, 1);
  _pub_obj_Transp         = nh_pub.advertise<ohm_hokuyo3d::CoordDist>(pub_transp, 1);
  _pub_obj_Mirror         = nh_pub.advertise<ohm_hokuyo3d::CoordDist>(pub_mirror, 1);

  _pub_affected           = nh_pub.advertise<ohm_hokuyo3d::CoordDist>(pub_affected, 1);
  _pub_behTransp          = nh_pub.advertise<ohm_hokuyo3d::CoordDist>(pub_behTransp, 1);
  _pub_behMirror          = nh_pub.advertise<ohm_hokuyo3d::CoordDist>(pub_behMirror, 1);
  _pub_backprojMirror     = nh_pub.advertise<ohm_hokuyo3d::CoordDist>(pub_backprojMirror, 1);

  _pub_cleaned            = nh_pub.advertise<ohm_hokuyo3d::CoordDist>(pub_cleaned, 1);

  _pub_marker             = nh_pub.advertise<visualization_msgs::Marker>(pub_marker, 1);

  _transformListener = new (tf::TransformListener);

  /*
   * run initialization
   */
  init();

  ros::Rate r(50);

  while(ros::ok())
  {
    if(_start_filter)
    {
//      cout << "Publish results " << _pub_Results << endl;

      if(_pub_Results)
        ROS_INFO("************************* Post-Filter pub *************************");

      mirrordetector::scan3D* tmp_combined_scan;
      std::vector<mirrordetector::object3D*> tmp_object;

//      //TOTEST:
//      cout << "Amount of scans: " << _scan_counter << endl;
//      std::stringstream s;
//      for(int i=0; i < _scan_counter; i++)
//      {
//        double* coord1      = _scan_in[i]->getCoords1();
//        object3D::objType* objectType1 = _scan_in[i]->getObjectType1();
//
//        double* coord2      = _scan_in[i]->getCoords2();
//       s << "/home/rainer/workspace/post1_" << i << ".txt";
//       printCoords2File(coord1, _scan_in[i]->getSize(), s.str().c_str());
//       //printCoords2File(coord1, objectType1, object3D::validPoint,  _scan_in[i]->getSize(), s.str().c_str());
//
//       s.str("");
//       s << "/home/rainer/workspace/post2_" << i << ".txt";
//       printCoords2File(coord2, _scan_in[i]->getSize(), s.str().c_str());
//       s.str("");
//      }
//       //ENDTOTEST

      /*
       * extract corrupted points
       */
      separateAffectedPoints(_scan_in, _scan_counter, _scan_nr_corrupted_counter, _scan_nr_corrupted, tmp_combined_scan);

//      s.str("");
//      s << "/home/rainer/workspace/post_separated.txt";
//      tmp_combined_scan[0].printScanCoords2File(s.str().c_str());

//      //TOTOEST:
//      cout << "separation done" << endl;
//
//      cout << "print combined scan" << endl;
//      printCoords2File(tmp_combined_scan->getCoords1(), tmp_combined_scan->getSize(), "/home/rainer/workspace/sep1.txt");
//
//      cout << "print objects" << endl;
//      printCoords2File(tmp_combined_scan->getCoords1(), tmp_combined_scan->getObjectType1(), object3D::errorSurface, tmp_combined_scan->getSize(), "/home/rainer/workspace/sur.txt");
//      printCoords2File(tmp_combined_scan->getCoords1(), tmp_combined_scan->getObjectType1(), object3D::behindErrorS, tmp_combined_scan->getSize(), "/home/rainer/workspace/beh.txt");
//      printCoords2File(tmp_combined_scan->getCoords1(), tmp_combined_scan->getObjectType1(), object3D::validPoint, tmp_combined_scan->getSize(), "/home/rainer/workspace/val.txt");
//
//      //ENDTOTEST:


      /*
       * identify objects
       */
      tmp_combined_scan->indentifyObjects(tmp_object, _planeDetection_thres_dist, _planeDetection_minPoints);
//      cout << "identification done" << endl;
//
//      s.str("");
//      s << "/home/rainer/workspace/post_identified.txt";
//      tmp_combined_scan[0].printScanCoords2File(s.str().c_str());
//
//      cout << "identification done" << endl;

//      //TOTEST:
//      std::stringstream s;
//      if(_pub_Results)
//      {
//        cout << "Post Objects found : "<< tmp_object.size() << endl;
//        for(int i=0; i < tmp_object.size(); i++)
//        {
//          s << "/home/rainer/workspace/post_corner_" << i << ".txt";
//          cout << "Corners " << i  << endl;
//          printCoords2File(tmp_object[i]->getObjectCorners(), 4, s.str().c_str());
//        }
//      }
//      //ENDTOTEST:


      if(tmp_object.size() != 0)
      {
        cout << "objects found" << endl;
        /*
         * resize Buffer for objects
         */
        if(_object.size() < tmp_object.size())
        {
          ROS_INFO("resize object buffer");
          _object.resize(tmp_object.size());
        }

        if(_pub_Results)
          cout << "Amount of identified objects: " << tmp_object.size() << endl;

        /*
         * analyze types of objects
         */
        for(int i=0; i < tmp_object.size(); i++)
        {
          tmp_object[i]->analyzeObjectType(tmp_combined_scan->getCoords1(), tmp_combined_scan->getCoords2(), tmp_combined_scan->getIntensity1(), tmp_combined_scan->getIntensity2(), tmp_combined_scan->getSize(), _planeDetection_thres_dist, _visionCone_thres, _maxDistICP, _maxRotICP, _fitnessICP, _thres_mean, _normIntensWhPaper_rise, _normIntensWhPaper_offset, _sizeVarFactor, _thres_variationInt);

          if(_pub_Results)
            cout << "object rated as: " << tmp_object[i]->getObjectType() << endl;

        }
      //  cout << "analyzation done" << endl;

        /*
         * save objects
         */
        for(int i = 0; i < (tmp_object.size()); i++)
        {
          _object[i] = tmp_object[i];
        }
        _object_size = tmp_object.size();

        /*
         * clean scans
         */
        cleanScan(_scan_in, _object, _object_size, _planeDetection_thres_dist, _visionCone_thres);

//        //TOTEST:
//        std::stringstream s;
//        for(int i=0; i < _lastScan; i++)
//        {
//          s.str("");
//          s << "/home/rainer/workspace/post_out_sur_e1_" << i << ".txt";
//          printCoords2File(_scan_in[i]->getCoords1(), _scan_in[i]->getObjectType1(), object3D::reflectiveSurface, _scan_in[i]->getSize(), s.str().c_str());
//          s.str("");
//          s << "/home/rainer/workspace/post_out_beh_e1_" << i << ".txt";
//          printCoords2File(_scan_in[i]->getCoords1(), _scan_in[i]->getObjectType1(), object3D::behindReflective, _scan_in[i]->getSize(), s.str().c_str());
//          s.str("");
//          s << "/home/rainer/workspace/post_out_val_e1_" << i << ".txt";
//          printCoords2File(_scan_in[i]->getCoords1(), _scan_in[i]->getObjectType1(), object3D::validPoint, _scan_in[i]->getSize(), s.str().c_str());
//          s.str("");
//          s << "/home/rainer/workspace/post_out_sur_e2_" << i << ".txt";
//          printCoords2File(_scan_in[i]->getCoords2(), _scan_in[i]->getObjectType2(), object3D::reflectiveSurface, _scan_in[i]->getSize(), s.str().c_str());
//          s.str("");
//          s << "/home/rainer/workspace/post_out_beh_e2_" << i << ".txt";
//          printCoords2File(_scan_in[i]->getCoords2(), _scan_in[i]->getObjectType2(), object3D::behindReflective, _scan_in[i]->getSize(), s.str().c_str());
//          s.str("");
//          s << "/home/rainer/workspace/post_out_val_e2_" << i << ".txt";
//          printCoords2File(_scan_in[i]->getCoords2(), _scan_in[i]->getObjectType2(), object3D::validPoint, _scan_in[i]->getSize(), s.str().c_str());
//        }
//        cout << "print post-out final files" << endl;
//        unsigned int test_total = 0;
////        for(int i=0; i < _lastScan; i++)
////        {
////          test_total += _scan_in[i]->getSize();
////        }
////        double* test_coord = new double[3*test_total];
////        unsigned int count = 0;
////        object3D::objType* test_mask = new object3D::objType[test_total];
////        for(int i=0; i < _lastScan; i++)
////        {
////          double* coor = _scan_in[i]->getCoords1();
////          object3D::objType* mas = _scan_in[i]->getObjectType1();
////          for(int j=0; j < _scan_in[i]->getSize(); j++)
////          {
////            test_coord[count + 3*j]  = coor[3*j];
////            test_coord[count + 3*j+1]  = coor[3*j+1];
////            test_coord[count + 3*j+2]  = coor[3*j+2];
////            test_mask[count + j]   = mas[3*j];
////          }
////          count += _scan_in[i]->getSize();
////        }
////        printCoords2File(test_coord, test_mask, object3D::reflectiveSurface, test_total, "/home/rainer/workspace/sur_final.txt");
////        printCoords2File(test_coord, test_mask, object3D::behindReflective , test_total, "/home/rainer/workspace/beh_final.txt");
////        printCoords2File(test_coord, test_mask, object3D::validPoint, test_total, "/home/rainer/workspace/val_final.txt");
////        cout << "done" << endl;
////        s << "/home/rainer/workspace/post_corner_final.txt";
//        double *corner_print = new double[12*_object_size];
//        for(int i=0; i < _object_size; i++)
//        {
//           double* show_corner = _object[i]->getObjectCorners();
//           corner_print[i*12] = show_corner[0];
//           corner_print[i*12+1] = show_corner[1];
//           corner_print[i*12+2] = show_corner[2];
//           corner_print[i*12+3] = show_corner[3];
//           corner_print[i*12+4] = show_corner[4];
//           corner_print[i*12+5] = show_corner[5];
//           corner_print[i*12+6] = show_corner[6];
//           corner_print[i*12+7] = show_corner[7];
//           corner_print[i*12+8] = show_corner[8];
//           corner_print[i*12+9] = show_corner[9];
//           corner_print[i*12+10] = show_corner[10];
//           corner_print[i*12+11] = show_corner[11];
//        }
//        printCoords2File(corner_print, 4*_object_size, s.str().c_str());
//        //ENDTOTEST


        /*
         * publish
         */
        publisherFunc();
      }
      if(_pub_Results)
      ROS_INFO("************************* Finish Post-Filter *************************");
      _start_filter = false;
    }
    _new_object = false;

    ros::spinOnce();
    r.sleep();
  }
  ROS_INFO("Shutting down Post-Filter");

  return 0;
}
