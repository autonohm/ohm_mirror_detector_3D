/**
* @file   ohm_2d_extractor
* @author Rainer Koch
* @date   13.02.2017
*
*
*/

#include "ohm_2d_extractor.h"

using namespace std;

unsigned int _choosenScanLinePoint  = 180;
unsigned int _pointsLaserScanPlane  = 1081;

double _angle_min  = -3.1416;
double _angle_max  = 3.1416;
double _range_min  = 0.0;
double _range_max  = 30.0;

// ROS variables
ros::Publisher _pub_scan;
sensor_msgs::LaserScan _2dscan;

/*
 * Subscriber & Publisher
 */
void subscriberFunc(const ohm_hokuyo3d::CoordDistMulti& msg)
{
  unsigned int seq          = msg.coorddist[0].coord.header.seq;
  unsigned long stamp       = msg.coorddist[0].coord.header.stamp.toNSec();
  string frame_id           = msg.coorddist[0].coord.header.frame_id;
  unsigned int fragmentid   = msg.coorddist[0].coord.fragmentid;
  unsigned int fragmentsize = msg.coorddist[0].coord.fragmentsize;

  unsigned int sizeCoord    = 3*fragmentsize;
  unsigned int sizeOthers   = fragmentsize;
  unsigned int stepsPhi     = msg.coorddist[1].dist.phi.size();

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

//      cout << "fragmentsize " << fragmentsize << endl;
//      cout << "fragmentid " << fragmentid << endl;
//      cout << "size coords1 " << msg.coorddist[0].coord.coords.size() << endl;
//      cout << "size coords2 " << msg.coorddist[1].coord.coords.size() << endl;
//      cout << "size normals1 " << msg.coorddist[0].coord.normals.size() << endl;
//      cout << "size normals2 " << msg.coorddist[1].coord.normals.size() << endl;
//      cout << "size intens 1 " << msg.coorddist[0].coord.intensity.size() << endl;
//      cout << "size intens 2 " << msg.coorddist[1].coord.intensity.size() << endl;
//      cout << "size obj 1 " << msg.coorddist[0].coord.objectType.size() << endl;
//      cout << "size obj 2 " << msg.coorddist[1].coord.objectType.size() << endl;
//      cout << "size dist 1 " << msg.coorddist[0].dist.dists.size() << endl;
//      cout << "size dist 2 " << msg.coorddist[1].dist.dists.size() << endl;
//      cout << "size phi 1 " << msg.coorddist[0].dist.phi.size() << endl;
//      cout << "size phi 2 " << msg.coorddist[1].dist.phi.size() << endl;

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

      coord2[3*i]       = msg.coorddist[1].coord.coords[3*i];
      coord2[3*i+1]     = msg.coorddist[1].coord.coords[3*i+1];
      coord2[3*i+2]     = msg.coorddist[1].coord.coords[3*i+2];

      intens1[i]        = (unsigned int)msg.coorddist[0].coord.intensity[i];
      intens2[i]        = (unsigned int)msg.coorddist[1].coord.intensity[i];

      normals1[3*i]     = msg.coorddist[0].coord.normals[3*i];
      normals1[3*i+1]   = msg.coorddist[0].coord.normals[3*i+1];
      normals1[3*i+2]   = msg.coorddist[0].coord.normals[3*i+2];

      normals2[3*i]     = msg.coorddist[1].coord.normals[3*i];
      normals2[3*i+1]   = msg.coorddist[1].coord.normals[3*i+1];
      normals2[3*i+2]   = msg.coorddist[1].coord.normals[3*i+2];

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

  /*
   * calculate desired scan points
   */
  unsigned int size_Scanplane  = fragmentsize / stepsPhi;
  unsigned int size2d          = 2*stepsPhi;
  unsigned int size2d_half     = stepsPhi;


   /*
    * copy desired points from 3D cloud into 2D scan
    */
  _2dscan.header.frame_id = "laser";
  _2dscan.header.stamp    = msg.coorddist[0].coord.header.stamp;
  _2dscan.header.seq      = msg.coorddist[0].coord.header.seq;

  _2dscan.angle_min       = _angle_min;
  _2dscan.angle_max       = _angle_max;
  _2dscan.angle_increment = (_angle_max - _angle_min) / size2d;
  _2dscan.range_min       = _range_min;
  _2dscan.range_max       = _range_max;
    //_2dscan.scan_time       = source.scan_time;

  _2dscan.ranges.resize(size2d);
  _2dscan.intensities.resize(size2d);
//  cout << "--------------------------------------------------------------" << endl;
//  cout << size2d << "/" << size_Scanplane << "/" << sizeOthers << endl;

  unsigned int n = 0;
  unsigned int j = 0;
  unsigned int g = 0;
  for(unsigned int i=0; i < sizeOthers; i++)
  {
    // copy the first point in the scan plane
    if(i == ((size_Scanplane * g) + _choosenScanLinePoint))
    {
      _2dscan.ranges[size2d_half+j]       = dist1[i];
      _2dscan.intensities[size2d_half+j]  = intens1[i];

      //cout << j << "-" << _2dscan.ranges[j] << "-" << dist1[i] << "/" << endl;
    }
    // copy the second point in the scan plane
    if(i == (size_Scanplane * (g+1) - _choosenScanLinePoint))
    {
      _2dscan.ranges[j]       = dist1[i];
      _2dscan.intensities[j]  = intens1[i];

      //cout << j << "-" << _2dscan.ranges[j] << "-" << dist1[i] << "/" << endl;
      j++;
    }

    // count the scan planes
    n++;
    if(n == size_Scanplane)
    {
      g++;
      n = 0;
    }
  }

//  cout << endl;
//  cout << "--------------------------------------------------------------" << endl;


  _pub_scan.publish(_2dscan);
}

int main(int argc, char **argv)
{
  double dVar = 0;
  int iVar = 0;
  ros::init(argc, argv, "ohm_2d_extractor");
  ros::NodeHandle nh_sub("~");
  ros::NodeHandle nh_pub("~");

  /*
   * Parameters for launch file
   */
  std::string sub_inputScan;

  std::string pub_scan;

  /*
   * sub and publisher
   */
  nh_sub.param<std::string>("sub_input_scan",           sub_inputScan,            "scan3d");
  nh_pub.param<std::string>("pub_scan",                 pub_scan,                 "scan2d");

  /*
   * variables
   */
  nh_sub.param<int>("choosenScanLinePoint",                      iVar, 180);         // point which are used to build the 2D scan
  _choosenScanLinePoint = static_cast<int>(iVar);
  nh_sub.param<double>("angle_min",                      dVar, -3.1416);
  _angle_min = static_cast<double>(dVar);
  nh_sub.param<double>("angle_max",                      dVar, 3.1416);
  _angle_max = static_cast<double>(dVar);
  nh_sub.param<double>("range_min",                      dVar, 0.0);
  _range_min = static_cast<double>(dVar);
  nh_sub.param<double>("range_max",                      dVar, 30.0);
  _range_max = static_cast<double>(dVar);

  /*
   * initialize subscriber
   */
  ros::Subscriber sub   = nh_sub.subscribe(sub_inputScan, 10, subscriberFunc);
  /*
   * initialize publisher
   */
  _pub_scan               = nh_pub.advertise<sensor_msgs::LaserScan>(pub_scan, 1);

  ros::Rate r(50);

  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Shutting down");

  return 0;
}
