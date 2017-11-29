/**
* @file   ohm_mirror_prefilter3D
* @author Rainer Koch
* @date   20.03.2016
*
*
*/

#include "ohm_mirror_prefilter3D.h"


//#include <cmath>
//#include <float.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//
//#include "obcore/base/Time.h"

//#include <sstream>

//#define _USE_MATH_DEFINES

//using std::vector;
using namespace std;
using namespace mirrordetector;

//using namespace cv;

bool _new_dataset = false;
bool _received_first_scan = false;

bool _test_once = false;

std::vector<mirrordetector::scan3D*> _scan_in(1);
std::vector<mirrordetector::object3D*> _object(5);
unsigned int _object_size = 0;

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

//int _outlierFilter_kMean            = 100;
//double _outlierFilter_stdDeviation  = 0.1;
int _outlierFilter_minNeighbors     = 10;
double _outlierFilter_searchRadius  = 0.1;    // [m]
double _substract_thres_dist        = 0.01;   // [m]
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

bool _ExecuteOneTime = false;

////Totest
//int testcounter = 0;
//std::stringstream s;


//  check timing
//  double time_sum;
//  int time_counter;
//  double time_sum1;
//  int time_counter1;

/*
 * Subscriber & Publisher
 */
void subscriberFunc(const ohm_hokuyo3d::CoordDistMulti& msg)
{
  //cout << "receive in" << endl;
  if(!_received_first_scan)
  {
    if(_pub_Results)
    ROS_INFO("Mirror Detector 3D-Prefilter: Receive first scan");
    _received_first_scan = true;
    /*
     * check if intensity is broadcasted
     */
    if(msg.coorddist[0].coord.intensity.size() == 0)
    {
      ROS_ERROR("No intensities broadcasted!");
      exit;
    }
  }

  /*
   *
   */
  if(!_new_dataset)
  {
   // cout << "subscribed" << endl;

    _new_dataset = true;
//    ROS_INFO("***********************************************************************");
//    ROS_INFO("Mirror Detector 3D-Prefilter: Receive scan");

    //cout << sizeCoord << "/ " << sizeOthers << endl;
    unsigned int seq          = msg.coorddist[0].coord.header.seq;
    unsigned long stamp       = msg.coorddist[0].coord.header.stamp.toNSec();
    string frame_id           = msg.coorddist[0].coord.header.frame_id;
    unsigned int fragmentid   = msg.coorddist[0].coord.fragmentid;
    unsigned int fragmentsize = msg.coorddist[0].coord.fragmentsize;

    unsigned int sizeCoord    = 3*fragmentsize;
    unsigned int sizeOthers   = fragmentsize;
    unsigned int stepsPhi     = msg.coorddist[1].dist.phi.size();

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

    double* coord1            = new double[sizeCoord];
    double* coord2            = new double[sizeCoord];

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

    object3D::objType* objectType1 = new object3D::objType[sizeOthers];
    object3D::objType* objectType2 = new object3D::objType[sizeOthers];

    int count_phi = 0;
    int m = 0;
    for(int i = 0; i < sizeOthers; i++)
    {
      mask1[i]        = msg.coorddist[0].coord.mask[i];
      mask2[i]        = msg.coorddist[1].coord.mask[i];

      if((mask1[i] == 1) && (mask2[i] == 1)
          && !isnan(msg.coorddist[0].coord.coords[3*i]) && !isnan(msg.coorddist[0].coord.coords[3*i+1]) && !isnan(msg.coorddist[0].coord.coords[3*i+2])
          && !isnan(msg.coorddist[1].coord.coords[3*i]) && !isnan(msg.coorddist[1].coord.coords[3*i+1]) && !isnan(msg.coorddist[1].coord.coords[3*i+2]))
      {
        coord1[3*i]       = msg.coorddist[0].coord.coords[3*i];
        coord1[3*i+1]     = msg.coorddist[0].coord.coords[3*i+1];
        coord1[3*i+2]     = msg.coorddist[0].coord.coords[3*i+2];
//        cout << msg.coorddist[0].coord.coords[3*i] << "-";

        coord2[3*i]       = msg.coorddist[1].coord.coords[3*i];
        coord2[3*i+1]     = msg.coorddist[1].coord.coords[3*i+1];
        coord2[3*i+2]     = msg.coorddist[1].coord.coords[3*i+2];

        intens1[i]        = (unsigned int)msg.coorddist[0].coord.intensity[i];
        intens2[i]        = (unsigned int)msg.coorddist[1].coord.intensity[i];
//        cout << (unsigned int)msg.coorddist[0].coord.intensity[i] << "-";
//        cout << (unsigned int)msg.coorddist[1].coord.intensity[i] << "/";

        normals1[3*i]     = msg.coorddist[0].coord.normals[3*i];
        normals1[3*i+1]   = msg.coorddist[0].coord.normals[3*i+1];
        normals1[3*i+2]   = msg.coorddist[0].coord.normals[3*i+2];

        normals2[3*i]     = msg.coorddist[1].coord.normals[3*i];
        normals2[3*i+1]   = msg.coorddist[1].coord.normals[3*i+1];
        normals2[3*i+2]   = msg.coorddist[1].coord.normals[3*i+2];

        objectType1[i]    = (object3D::objType) msg.coorddist[0].coord.objectType[i];
        objectType2[i]    = (object3D::objType) msg.coorddist[1].coord.objectType[i];

        dist1[i]          = msg.coorddist[0].dist.dists[i];
        dist2[i]          = msg.coorddist[1].dist.dists[i];

        phi1[i]           = msg.coorddist[0].dist.phi[count_phi];
        phi2[i]           = msg.coorddist[1].dist.phi[count_phi];
      }
      else
      {
        coord1[3*i]     = NAN;
        coord1[3*i+1]   = NAN;
        coord1[3*i+2]   = NAN;

        coord2[3*i]     = NAN;
        coord2[3*i+1]   = NAN;
        coord2[3*i+2]   = NAN;

        intens1[i]      = 0;
        intens2[i]      = 0;

        normals1[i]     = NAN;
        normals2[i]     = NAN;

        objectType1[i]  = object3D::nanPoint;
        objectType2[i]  = object3D::nanPoint;

        dist1[i]        = NAN;
        dist2[i]        = NAN;

        phi1[i]         = NAN;
        phi2[i]         = NAN;
      }
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
//    cout << endl;

//    //TOTEST:
//      cout << "in test" << endl;
//      for (int i=0; i < 100; i++)
//      {
//        cout << objectType1[i] << " ";
//      }
//      cout << endl;
//      cout << "in test done " << endl;
//    //TEST
//    cout << "msg: " << sizeOthers << endl;
//    for(int i = 0; i < 10; i++)
//    {
//      cout << msg.coorddist[0].coord.coords[i] << " ";
//    }
//    cout << endl;
//    cout << "msg copied:" << endl;
//    for(int i = 0; i < 10; i++)
//    {
//      cout << dist1[i] << " ";
//    }
//    cout << endl;

    mirrordetector::scan3D* scan_in_tmp = new mirrordetector::scan3D(
        seq,
        stamp,
        frame_id,
        fragmentid,
        fragmentsize,
        sizeOthers,
        stepsPhi,
        coord1,
        intens1,
        coord2,
        intens2);

//    printIntens2File(intens1, sizeOthers, "/home/rainer/workspace/i_1.txt");
//    printIntens2File(intens2, sizeOthers, "/home/rainer/workspace/i_2.txt");
    scan_in_tmp->setPubResults(_pub_Results);
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


    _scan_in[0] = scan_in_tmp;

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
//    cout << "finish receive first scan" << endl;
  }

//  //TEST
//  if((!_test_once))
//  {
//    cout << "t" << endl;
//   // _test_once = true;
//  cout << "scan_in out " << _scan_in[0]->getSize() << endl;
//  double* coord2test            = new double[_scan_in[0]->getSize()];
//  coord2test = _scan_in[0]->getDist1();
//  for (int i=0; i < 10; i++)
//  {
//    cout << coord2test[i] << " ";
//  }
//  cout << endl;
//  }
//  cout << "receive out " << endl;
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
  _marker_points.action = visualization_msgs::Marker::ADD; // 0 = add/modify, 1 = (deprecated), 2 = delete, New in Indigo 3 = deleteall ;

  _marker_points.color.a = 1.0; // Don't forget to set the alpha!
  _marker_points.color.r = 0.0;
  _marker_points.color.g = 0.0;
  _marker_points.color.b = 0.0;

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
  /*
   * read scan3D data
   */
  unsigned int sizeScan         = _scan_in[0]->getSize();
  double* object_corners;
  unsigned int affSize          = 0;


  /*
   * prepare ROS msgs
   */
  ohm_hokuyo3d::CoordDistMulti msg_scan;
  ohm_hokuyo3d::CoordDist      msg_object;
  ohm_hokuyo3d::CoordDist      msg_obj_Transp;
  ohm_hokuyo3d::CoordDist      msg_obj_Mirror;
  ohm_hokuyo3d::CoordDist      msg_affected;
  ohm_hokuyo3d::CoordDist      msg_behTransp;
  ohm_hokuyo3d::CoordDist      msg_behMirror;
  ohm_hokuyo3d::CoordDist      msg_backprojMirror;
  ohm_hokuyo3d::CoordDist      msg_cleaned;

//  //TOTEST:
//  cout << "test type" << endl;
//  object3D::objType* oType = NULL;
//  oType = _scan_in[0]->getObjectType1();
//  for (int i=0; i < 10; i++)
//  {
//    cout << oType[i] << " ";
//  }
//  cout << endl;
//  cout << "test done " << endl;

  copyScanClass3D_MultiScan(*_scan_in[0], msg_scan);

  /*
   * copy scan data into msgs
   */
  //cout << "publish valid: " << endl;
  copyScanClass3D_ScanSelected(*_scan_in[0], msg_cleaned, object3D::validPoint);   // points with are free of influences

  /*
   * if object was found, copy data of object, affected data and cleand scan data
   */
  if(_object_size > 0)
  {
    /*
     * copy object, affected and cleaned data into msgs
     */

    /*
     * if object mirror -> back project
     */
    // TODO: Have to be done for each object and its corresponding points
    copyReprojectedScan(*_scan_in[0], msg_backprojMirror, *_object[0]);   // points which are influenced by the object are reprojected

    //cout << "publish surface: " << endl;
    copyScanClass3D_ScanSelected(*_scan_in[0], msg_object, object3D::errorSurface);   // points which are located on the object
    copyScanClass3D_ScanSelected(*_scan_in[0], msg_obj_Mirror, object3D::reflectiveSurface);   // points which are located on the object
    copyScanClass3D_ScanSelected(*_scan_in[0], msg_obj_Transp, object3D::transpSurface);   // points which are located on the object

    //cout << "publish error: " << endl;
    copyScanClass3D_ScanSelected(*_scan_in[0], msg_affected, object3D::behindErrorS);             // points which are influenced by the object
    copyScanClass3D_ScanSelected(*_scan_in[0], msg_behMirror, object3D::behindReflective);        // points which are influenced by the object
    copyScanClass3D_ScanSelected(*_scan_in[0], msg_behTransp, object3D::behindTransparent);       // points which are influenced by the object

    /*
     * write corner data of object
     */
    //cout << "publish " << _object_size << " corners" << endl;
    for(int i=0; i < _object_size; i++)
    {
      object_corners = _object[i]->getObjectCorners();
      //BEGIN TOTEST:
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

//      cout << "object:" << i << "size: " << a << " / " << b << endl;
//      cout << object_corners[0] << " " << object_corners[1] << " " <<object_corners[2] << endl;
//      cout << object_corners[3] << " " << object_corners[4] << " " <<object_corners[5] << endl;
//      cout << object_corners[6] << " " << object_corners[7] << " " <<object_corners[8] << endl;
//      cout << object_corners[9] << " " << object_corners[10] << " " <<object_corners[11] << endl;
//      cout << endl;
      // END TOTEST

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

//  //TOTEST:
//  std::stringstream s;
//  s.str("");
//  s << "/home/rainer/workspace/pre_out_sur_" << _scan_in[0]->getSeqenz() << ".txt";
//  printCoords2File(_scan_in[0]->getCoords1(), _scan_in[0]->getObjectType1(), object3D::reflectiveSurface, _scan_in[0]->getSize(), s.str().c_str());
//  s.str("");
//  s << "/home/rainer/workspace/pre_out_beh_" << _scan_in[0]->getSeqenz() << ".txt";
//  printCoords2File(_scan_in[0]->getCoords1(), _scan_in[0]->getObjectType1(), object3D::behindReflective, _scan_in[0]->getSize(), s.str().c_str());
//  s.str("");
//  s << "/home/rainer/workspace/pre_out_val_" << _scan_in[0]->getSeqenz() << ".txt";
//  printCoords2File(_scan_in[0]->getCoords1(), _scan_in[0]->getObjectType1(), object3D::validPoint, _scan_in[0]->getSize(), s.str().c_str());
//  s.str("");
//  s << "/home/rainer/workspace/pre_out_corner_" << _scan_in[0]->getSeqenz() << ".txt";
//  double* tmp_o = new double[_object_size*12];
//  double* tmp;
//  for(int i=0; i < _object_size; i++)
//  {
//    tmp = _object[i]->getObjectCorners();
//    tmp_o[i*_object_size+0] = tmp[0];
//    tmp_o[i*_object_size+1] = tmp[1];
//    tmp_o[i*_object_size+2] = tmp[2];
//    tmp_o[i*_object_size+3] = tmp[3];
//    tmp_o[i*_object_size+4] = tmp[4];
//    tmp_o[i*_object_size+5] = tmp[5];
//    tmp_o[i*_object_size+6] = tmp[6];
//    tmp_o[i*_object_size+7] = tmp[7];
//    tmp_o[i*_object_size+8] = tmp[8];
//    tmp_o[i*_object_size+9] = tmp[9];
//    tmp_o[i*_object_size+10] = tmp[10];
//    tmp_o[i*_object_size+11] = tmp[11];
//  }
//  printCoords2File(tmp_o, (_object_size*4), s.str().c_str());
//
//  cout << "print pre-out final files" << endl;



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

/*
 * other functions
 */
void separateObjectPoints(mirrordetector::scan3D* scan, std::vector<mirrordetector::object3D*>& object, unsigned int amount_affected, unsigned int& objectCounter, double planeDetection_threshold, double visionCone_threshold, unsigned int planeDetectin_minPoints, double maxDist, double maxRot, double fitnessICP, double threshold_median, unsigned int normIntensWhPaper_rise,  unsigned int normIntensWhPaper_offset)
{
  double* affected_coords1        = NULL;
  double* affected_coords2        = NULL;
  unsigned int* affected_intens1  = NULL;
  unsigned int* affected_intens2  = NULL;
  /*
   * copy affected data
   */
  if(amount_affected > 0)
  {
    double* tmp_coords1             = scan->getCoords1();
    double* tmp_coords2             = scan->getCoords2();
    unsigned int* tmp_intens1       = scan->getIntensity1();
    unsigned int* tmp_intens2       = scan->getIntensity2();
    object3D::objType* tmp_objectType1   = scan->getObjectType1();
    object3D::objType* tmp_objectType2   = scan->getObjectType2();

    affected_coords1  = new double[3*amount_affected];
    affected_coords2  = new double[3*amount_affected];
    affected_intens1  = new unsigned int[amount_affected];
    affected_intens2  = new unsigned int[amount_affected];

    std::vector<mirrordetector::object3D*> tmp_objects;
    /*
     * create object
     */
    int n = 0;
    /*
     * copy out affected data from scan
     */
    for(unsigned int i=0; i< scan->getSize(); i++)
    {
//      cout << tmp_intens1[i] << "-";
//      cout << tmp_intens2[i] << "/";
      if(tmp_objectType1[i] == 2)
      {
        affected_coords1[3*n]   = tmp_coords1[3*i];
        affected_coords1[3*n+1] = tmp_coords1[3*i+1];
        affected_coords1[3*n+2] = tmp_coords1[3*i+2];

        affected_coords2[3*n]   = tmp_coords2[3*i];
        affected_coords2[3*n+1] = tmp_coords2[3*i+1];
        affected_coords2[3*n+2] = tmp_coords2[3*i+2];

        affected_intens1[n]     = tmp_intens1[i];
        affected_intens2[n]     = tmp_intens2[i];

        //objectType[n]           = tmp_objectType[i];
        n++;
      }
      else if(tmp_objectType1[i] == 3)
      {
        affected_coords1[3*n]   = tmp_coords2[3*i];
        affected_coords1[3*n+1] = tmp_coords2[3*i+1];
        affected_coords1[3*n+2] = tmp_coords2[3*i+2];

        affected_coords2[3*n]   = tmp_coords1[3*i];
        affected_coords2[3*n+1] = tmp_coords1[3*i+1];
        affected_coords2[3*n+2] = tmp_coords1[3*i+2];

        affected_intens1[n]     = tmp_intens2[i];
        affected_intens2[n]     = tmp_intens1[i];

        //objectType[n]           = tmp_objectType[i];
        n++;
      }
    }
//    cout << endl;

//    printCoords2File(affected_coords1, n, "/home/rainer/workspace/aff1.txt");
//    printCoords2File(affected_coords2, n, "/home/rainer/workspace/aff2.txt");


//    cout << "identifyObjects->Fkt" << endl;
    //identify object planes and corners in scan
    scan->indentifyObjects(tmp_objects, planeDetection_threshold, planeDetectin_minPoints);
//    cout << "identifyObjects<-Fkt" << endl;

//    //ToTest

//    /*
//     * look up tf
//     */
//
    /*
     * identify point cloud for objects and determine the corner points
     */

    if(_pub_Results)
    cout << "Amount detected objects: " << tmp_objects.size() << endl;

    if(tmp_objects.size() != 0)
    {
      double* test = tmp_objects[0]->getObjectCorners();
//      cout << "object tmp" << endl;
//      cout << test[0] << "/" << test[1] << "/" << test[2] << endl;
//      cout << test[3] << "/" << test[4] << "/" << test[5] << endl;
//      cout << test[6] << "/" << test[7] << "/" << test[8] << endl;
//      cout << test[9] << "/" << test[10] << "/" << test[11] << endl;

      /*
       * resize Buffer for objects
       */
      if(_object.size() < tmp_objects.size())
      {
        if(_pub_Results)
        ROS_INFO("resize object buffer");
        _object.resize(tmp_objects.size());
      }

//      //ToTest
//      unsigned int* tmp_intens    = new unsigned int[scan->getSize()];
//      tmp_intens = scan->getIntensity1();
//      for(int i=0; i++; scan->getSize())
//      {
//        cout << tmp_intens << "/";
//      }
//      cout << endl;

      /*
       * analyze Types of objects
       */
      for(int i=0; i < tmp_objects.size(); i++) //
      {
        //printCoords2File(tmp_coords1, scan->getSize(), "/home/rainer/workspace/coords_orig.txt");
//        if(!_ExecuteOneTime)
//        {
//        cout << "************************************************************************************************" << endl;
//        cout << "Enter object type identification" << endl;

           tmp_objects[i]->analyzeObjectType(scan->getCoords1(), scan->getCoords2(), scan->getIntensity1(), scan->getIntensity2(), scan->getSize(), planeDetection_threshold, visionCone_threshold, maxDist, maxRot, fitnessICP, threshold_median, _normIntensWhPaper_rise, _normIntensWhPaper_offset, _sizeVarFactor, _thres_variationInt);

        //TOTEST:
//        if(tmp_objects[i]->analyzeObjectType(scan->getCoords1(), scan->getCoords2(), scan->getIntensity1(), scan->getIntensity2(), scan->getSize(), planeDetection_threshold, visionCone_threshold, maxDist, maxRot, fitnessICP, threshold_median, normIntensWhPaper_rise, normIntensWhPaper_offset, _sizeVarFactor, _thres_variationInt))
//        {
//          cout << "found object type " << tmp_objects[i]->getObjectType() << " for object " << (i+1) << endl;
//          cout << "0 = not defined, 2 = eror, 4 = refl. surface, 6 = transp. surface " << endl;
//        }
//        else
//        {
//          cout << "no object type for object " << i << endl;
//        }
//        cout << "finish identification" << endl;
//        cout << "************************************************************************************************" << endl;
//        _ExecuteOneTime = true;
//        }
      }

      /*
       * save objects
       */
      for(int i = 0; i < (tmp_objects.size()); i++)
      {
        object[i] = tmp_objects[i];
      }
       objectCounter = tmp_objects.size();
       //cout << "object counter: " << objectCounter << endl;

    }
//    else
//    {
//      cout << "No corners found!" << endl;
//    }
  }

  delete [] affected_coords1;
  delete [] affected_coords2;
  delete [] affected_intens1;
  delete [] affected_intens2;
}

void cleanScan(std::vector<mirrordetector::scan3D*> scan, std::vector<mirrordetector::object3D*>& object, unsigned int amount_objects, double thres_dist, double thres_visionCone)
{

  double* object_corners = new double[12];
  double* object_coefficents = new double[4];

  double* scan_points1       = scan[0]->getCoords1();
  double* scan_points2       = scan[0]->getCoords2();

  int scanSize              = scan[0]->getSize();

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

//  // To write in file
//  ofstream file_1;
//  ofstream file_2;
//  ofstream file_3;
//  ofstream file_4;
//  ofstream file_5;
//  ofstream file_6;
//  char filename1[64] = "/home/rainer/workspace/clean_1.txt";
//  file_1.open(filename1);
//  char filename2[64] = "/home/rainer/workspace/clean_2.txt";
//  file_2.open(filename2);
//  char filename3[64] = "/home/rainer/workspace/clean_3.txt";
//  file_3.open(filename3);
//  char filename4[64] = "/home/rainer/workspace/clean_4.txt";
//  file_4.open(filename4);
//  char filename5[64] = "/home/rainer/workspace/clean_5.txt";
//  file_5.open(filename5);
//  char filename6[64] = "/home/rainer/workspace/clean_6.txt";
//  file_6.open(filename6);


  /*
   * clean scan
   */
  for(int i=0; i < amount_objects; i++) // amount_objects
  {
    //cout << " object number " << i << endl;
    locationPoint1[i] = new int[scanSize];
    locationPoint2[i] = new int[scanSize];

    object_corners      = object[i]->getObjectCorners();
    object_coefficents  = object[i]->getCoefficents();
//    // TOTEST:
//    cout << object_corners[0] << "/" << object_corners[1] << "/" << object_corners[2] << endl;
//    cout << object_corners[3] << "/" << object_corners[4] << "/" << object_corners[5] << endl;
//    cout << object_corners[6] << "/" << object_corners[7] << "/" << object_corners[8] << endl;
//    cout << object_corners[9] << "/" << object_corners[10] << "/" << object_corners[11] << endl;

    //            file_5 << this->_coords_1[3*i] << " " << this->_coords_1[3*i+1] << " " << this->_coords_1[3*i+2] << " " << endl;
    //            file_5 << this->_coords_1[3*i] << " " << this->_coords_1[3*i+1] << " " << this->_coords_1[3*i+2] << " " << endl;


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

  // if no objects are detected => all unchecked objects are valid
  if(amount_objects == 0) // if no objects found => all unchecked are assumed to be valid
  {
    if(_pub_Results)
      cout << "No object found" << endl;

    for(int i=0; i < _scan_in[0]->getSize(); i++)
    {
       if(tmp_objectType1[i] == object3D::unchecked)
       {
         tmp_objectType1[i] = object3D::validPoint;
         tmp_objectType2[i] = object3D::validPoint;
       }
    }
  }

  scan[0]->setObjectType1(tmp_objectType1);
  scan[0]->setObjectType2(tmp_objectType2);

//  printCoords2File(scan_points1, tmp_objectType1, object3D::validPoint, scanSize, "/home/rainer/workspace/valid.txt");
//  printCoords2File(scan_points1, tmp_objectType1, object3D::behindErrorS, scanSize, "/home/rainer/workspace/behind.txt");
//  printCoords2File(scan_points1, tmp_objectType1, object3D::errorSurface, scanSize, "/home/rainer/workspace/on.txt");

//  // To write in file
//  file_1.close();
//  file_2.close();
//  file_3.close();
//  file_4.close();
//  file_5.close();
//  file_6.close();

  //TOTEST:
  //printObjectTypeAmounts(tmp_objectType1, scanSize, "changed");

//   //TOTEST:
//    object3D::objType* tmp_objectType3 = scan[0]->getObjectType1();
//    printObjectTypeAmounts(tmp_objectType3, scanSize, "reread");

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

  ROS_INFO("Init viewer intensity");
  //ROS_INFO("Running with threshold %f", _substract_threshold_dist);
}

int main(int argc, char **argv)
{
  double dVar = 0;
  int iVar = 0;
  bool bVar = 0;
  ros::init(argc, argv, "ohm_mirror_detector3D_prefilter");
  ros::NodeHandle nh_sub("~");
  ros::NodeHandle nh_pub("~");

  /*
   * Parameters for launch file
   */
  std::string sub_inputScan;

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
  nh_sub.param<std::string>("sub_input_scan",           sub_inputScan,            "scan3d_raw");

  nh_pub.param<std::string>("pub_scan",                 pub_scan,                 "scan3d");                // prefiltered  3D scan
  nh_pub.param<std::string>("pub_object",               pub_object,               "object");                // points assigned to an object
  nh_pub.param<std::string>("pub_transp",               pub_transp,               "obj_transp");            // points assigned to a transparent object
  nh_pub.param<std::string>("pub_mirror",               pub_mirror,               "obj_mirror");            // points assigned to a mirror

  nh_pub.param<std::string>("pub_affected",             pub_affected,             "affected");              // points assigned to be affected by the object
  nh_pub.param<std::string>("pub_behTransp",            pub_behTransp,            "behTransp");             // points assigned to be behind a transparent object
  nh_pub.param<std::string>("pub_behMirror",            pub_behMirror,            "behMirror");             // points assigned to be behind a mirror object
  nh_pub.param<std::string>("pub_backprojMirror",       pub_backprojMirror,       "backprojMirror");        // points assigned to be behind a mirror object and been back projected to their origin

  nh_pub.param<std::string>("pub_cleaned",              pub_cleaned,              "cleaned");               // points marked by their object type

  nh_pub.param<std::string>("pub_marker",               pub_marker,               "marker");                // maker to show in rviz

  /*
   * variables for pre-filter
   */
  nh_sub.param<bool>("publish_Results",                          bVar, false);            // show the results of the pre-filter
  _pub_Results = static_cast<bool>(bVar);

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
  _box_max_z = static_cast<double>(dVar);

  nh_sub.param<int>("outlierFilter_minNeighbors",                 iVar, 10);            // minimal number of neighbors
  _outlierFilter_minNeighbors = static_cast<int>(iVar);
  nh_sub.param<double>("outlierFilter_searchRadius",              dVar, 0.1);           // search radius for neighbors
  _outlierFilter_searchRadius = static_cast<float>(dVar);

  //nh_sub.param<int>("outlierFilter_kMean",                        iVar, 100);         // number of neighbors to analyze for each point
  //_outlierFilter_kMean = static_cast<int>(iVar);
  //nh_sub.param<double>("outlierFilter_stdDeviation",              dVar, 0.1);         // standard deviation multiplier. What this means is that all points who have a distance larger than kMean standard deviation of the mean distance to the query point will be marked as outliers and removed
  //_outlierFilter_kMean = static_cast<float>(dVar);

  /*
   * variables for object detection
   */
  nh_sub.param<double>("substract_threshold_distance",            dVar, 0.05);         // allowed distance between first and last echo [m]
  _substract_thres_dist = static_cast<float>(dVar);

  nh_sub.param<double>("planeDetection_threshold_distance",       dVar, 0.005);         // threshold, if point is assigned from pcl to the plane or not [m]
  _planeDetection_thres_dist = static_cast<float>(dVar);
  nh_sub.param<int>("planeDetection_minAmountPoints",             iVar, 200);           // threshold, how many points are needed to detect a plane
  _planeDetection_minPoints = static_cast<int>(iVar);

  nh_sub.param<double>("visionCone_threshold",                    dVar, 0.05);          // additional room around the object plane (extending object plane) [m]
  _visionCone_thres = static_cast<float>(dVar);

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
  ros::Subscriber sub   = nh_sub.subscribe(sub_inputScan, 10, subscriberFunc);
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

  /*
   * run initialization
   */
  init();

  ros::Rate r(50);

  int chooseScanCounter = 0;

  while(ros::ok())
  {
    //obvious::Time start_time = obvious::Time::now();
    if(_new_dataset)
    {
      if(_pub_Results)
      ROS_INFO("********************* Prefilter check new data *********************");

//      s.str("");
//      s << "/home/rainer/workspace/pre_beforeFilter_" << testcounter << ".txt";
//      _scan_in[0]->printScanCoords2File(s.str().c_str());

      _object_size = 0;
      /*
       * erases points, which are to close to scanner or to far away
       */
      //_scan_in[0]->printScanCoords2File("/home/rainer/workspace/0.txt");

      _scan_in[0]->distanceThresFilter(_min_dist, _max_dist);
      //_scan_in[0]->printScanCoords2File("/home/rainer/workspace/1.txt");

//      s.str("");
//      s << "/home/rainer/workspace/pre_afterFilterDist_" << testcounter << ".txt";
//      _scan_in[0]->printScanCoords2File(s.str().c_str());

//      _scan_in[0]->validPointFilter();
//
      //_scan_in[0]->printScanCoords2File("/home/rainer/workspace/2.txt");
//      ROS_INFO("Prefilter: DistanceThresFilter done");

      _scan_in[0]->boxFilter(_box_min_x, _box_max_x, _box_min_y, _box_max_y, _box_min_z, _box_max_z);
      //_scan_in[0]->printScanCoords2File("/home/rainer/workspace/3.txt");

//      s.str("");
//      s << "/home/rainer/workspace/pre_afterFilterBox_" << testcounter << ".txt";
//      _scan_in[0]->printScanCoords2File(s.str().c_str());

      /*
       * filter outliers
       */
//      ROS_INFO("Prefilter: BoxFilter done");
      _scan_in[0]->outlierFilterRadius(_outlierFilter_minNeighbors, _outlierFilter_searchRadius, object3D::unchecked);
      //_scan_in[0]->printScanCoords2File("/home/rainer/workspace/4.txt");

//      s.str("");
//      s << "/home/rainer/workspace/pre_afterFilterOutlier_" << testcounter << ".txt";
//      _scan_in[0]->printScanCoords2File(s.str().c_str());


      /*
       *  check for affected data and mark the object_type
       */
      unsigned int amount_affected = _scan_in[0]->identifyReflections(_substract_thres_dist);

//      s.str("");
//      s << "/home/rainer/workspace/pre_identified_" << testcounter << ".txt";
//      _scan_in[0]->printScanCoords2File(s.str().c_str());

//      ROS_INFO("Prefilter: OutlierFitler done");

//      //To write in file
//      for (size_t i = 0; i < _size; ++i)
//      {
//        file_1 << this->_coords_1[3*i] << " " << this->_coords_1[3*i+1] << " " << this->_coords_1[3*i+2] << " " << endl;
//      }

      if(amount_affected > 0)
      {
//        ROS_INFO("Prefilter: Start to check for influences");

        //TODO: Eleminate chooseScanCounter, just for Debug
        if(chooseScanCounter >= 0)
        {
//          cout << "Scan counter # " << chooseScanCounter << endl;
          /*
           * filter outliers
           */
          _scan_in[0]->outlierFilterRadius(_outlierFilter_minNeighbors, _outlierFilter_searchRadius, object3D::errorSurface);

//          s.str("");
//          s << "/home/rainer/workspace/pre_afterOutlier2_" << testcounter << ".txt";
//          _scan_in[0]->printScanCoords2File(s.str().c_str());

//          ROS_INFO("Prefilter: OutlierFilter done");

          /*
           * separate affected points from scan and identify objects
           */
           separateObjectPoints(_scan_in[0], _object, amount_affected, _object_size, _planeDetection_thres_dist, _visionCone_thres, _planeDetection_minPoints, _maxDistICP, _maxRotICP, _fitnessICP, _thres_mean, _normIntensWhPaper_rise, _normIntensWhPaper_offset);
           //_scan_in[0]->printScanCoords2File("/home/rainer/workspace/6.txt");
//           ROS_INFO("Prefilter: SeparateObjectPoints done");

//           s.str("");
//           s << "/home/rainer/workspace/pre_seperated_" << testcounter << ".txt";
//           _scan_in[0]->printScanCoords2File(s.str().c_str());
////           //TOTEST:
////           object3D::objType* tmp_objectType1 = _scan_in[0]->getObjectType1();
////           double* tmp_coord = _scan_in[0]->getCoords1();
////
////           printCoords2File(tmp_coord, tmp_objectType1, object3D::validPoint, _scan_in[0]->getSize(), "/home/rainer/workspace/valid.txt");
////           printCoords2File(tmp_coord, tmp_objectType1, object3D::behindErrorS, _scan_in[0]->getSize(), "/home/rainer/workspace/behind.txt");
////           printCoords2File(tmp_coord, tmp_objectType1, object3D::errorSurface, _scan_in[0]->getSize(), "/home/rainer/workspace/on.txt");
//           s.str("");
//           s << "/home/rainer/workspace/pre_seperated_" << testcounter << "_corner.txt";
//           double *corner_print = new double[12*_object_size];
//           for(int i=0; i < _object_size; i++)
//           {
//             double* show_corner = _object[i]->getObjectCorners();
//             corner_print[i*12] = show_corner[0];
//             corner_print[i*12+1] = show_corner[1];
//             corner_print[i*12+2] = show_corner[2];
//             corner_print[i*12+3] = show_corner[3];
//             corner_print[i*12+4] = show_corner[4];
//             corner_print[i*12+5] = show_corner[5];
//             corner_print[i*12+6] = show_corner[6];
//             corner_print[i*12+7] = show_corner[7];
//             corner_print[i*12+8] = show_corner[8];
//             corner_print[i*12+9] = show_corner[9];
//             corner_print[i*12+10] = show_corner[10];
//             corner_print[i*12+11] = show_corner[11];
//           }
//           printCoords2File(corner_print, 4*_object_size, s.str().c_str());


  //           //TOTEST:
  //           printObjectTypeAmounts(tmp_objectType1, _scan_in[0]->getSize(), "before Clean");

            /*
             * clean scan based on founded objects
             */
           cleanScan(_scan_in, _object, _object_size, _planeDetection_thres_dist, _visionCone_thres);
             //_scan_in[0]->printScanCoords2File("/home/rainer/workspace/scan_cleaned.txt");
//             ROS_INFO("Prefilter: CleanScan done");


           for(int i=0; i < _object_size; i++)
           {
//             //TOTEST:
//             double* test = _object[i]->getObjectCorners();
//             printObjectCorners(test, i);

             if(_pub_Results)
             {
               double* show_corner = _object[i]->getObjectCorners();
               cout << "object dimension: " << _object[i]->getMajorDim() << " m /" << _object[i]->getMinorDim() << " m" << endl;
               cout << "c 1: " <<   show_corner[0] << " / " << show_corner[1]  << " / " << show_corner[2] << endl;
               cout << "c 2: " <<   show_corner[3] << " / " << show_corner[4]  << " / " << show_corner[5] << endl;
               cout << "c 3: " <<   show_corner[6] << " / " << show_corner[7]  << " / " << show_corner[8] << endl;
               cout << "c 4: " <<   show_corner[9] << " / " << show_corner[10]  << " / " << show_corner[11] << endl;

               delete show_corner;
             }
           }


//           s.str("");
//           s << "/home/rainer/workspace/pre_out_" << testcounter << ".txt";
//           _scan_in[0]->printScanCoords2File(s.str().c_str());

//           //TOTEST:
//            object3D::objType* tmp_objectType2 = _scan_in[0]->getObjectType1();
//            printObjectTypeAmounts(tmp_objectType2, _scan_in[0]->getSize(), "after Clean");

        }
        chooseScanCounter++;

      }

//      testcounter++;
        //cout << "amount affected: " << amount_affected << endl;

      /*
       * publish cleaned scan
       */
      //cout << "go into publish" << endl;
      publisherFunc();
      //cout << "out publish" << endl;
      //ROS_INFO("Prefilter: Published");

      _new_dataset = false;
      if(_pub_Results)
      ROS_INFO("******************* Prefilter wait for new data *******************");
      //cout << endl;
    }



    //    Output timing
//        obvious::Time end_time = obvious::Time::now();
//        if(end_time - start_time > 0)
//        {
//          time_sum1 = time_sum1 + (end_time - start_time);
//          time_counter1++;
//          if(time_counter1 == 100)
//          {
//            cout << "Time: " << (time_sum1/100) << endl;
//            time_counter1 = 0;
//            time_sum1 = 0;
//          }
//        }
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Shutting down Pre-Filter");

  return 0;
}
