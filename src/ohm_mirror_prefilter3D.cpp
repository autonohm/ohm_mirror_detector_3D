/**
* @file   ohm_mirror_prefilter3D
* @author Rainer Koch
* @date   20.03.2016
*
*
*/

#include "ohm_mirror_prefilter3D.h"

using namespace std;
using namespace mirrordetector;

bool _new_dataset = false;
bool _received_first_scan = false;

bool _test_once = false;

std::vector<mirrordetector::scan3D*> _scan_in(1);
std::vector<mirrordetector::object3D*> _object(5);
unsigned int _object_size = 0;

double _min_dist                    = 0.023;
double _max_dist                    = 60.0;
int _outlierFilter_minNeighbors     = 10;
double _outlierFilter_searchRadius  = 0.1;    // [m]
double _substract_thres_dist        = 0.01;   // [m]
double _planeDetection_thres_dist   = 0.005;  // [m]
int _planeDetection_minPoints       = 200;
double _visionCone_thres            = 0.01;   // [m]
double _maxDistICP                  = 0.01;   // [m]
double _fitnessICP                  = 0.005;


//marker variables
visualization_msgs::Marker _marker_points;
visualization_msgs::Marker _marker_line_strip;
visualization_msgs::Marker _marker_line_list;

// ROS variables
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

/*
 * Subscriber & Publisher
 */
void subscriberFunc(const ohm_hokuyo3d::CoordDistMulti& msg)
{
  if(!_received_first_scan)
  {
    _received_first_scan = true;
    /*
     * check if intensity is broadcasted
     */
    if(msg.coorddist[0].coord.intensity.size() == 0)
    {
      exit;
    }
  }

  /*
   *
   */
  if(!_new_dataset)
  {
    _new_dataset = true;

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

    mirrordetector::scan3D* scan_in_tmp = new mirrordetector::scan3D(
        seq,
        stamp,
        frame_id,
        fragmentid,
        fragmentsize,
        sizeOthers,
        coord1,
        intens1,
        coord2,
        intens2);

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
  }
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

  copyScanClass3D_MultiScan(*_scan_in[0], msg_scan);

  /*
   * copy scan data into msgs
   */
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
    copyReprojectedScan(*_scan_in[0], msg_backprojMirror, *_object[0]);   // points which are influenced by the object are reprojected

    copyScanClass3D_ScanSelected(*_scan_in[0], msg_object, object3D::errorSurface);   // points which are located on the object
    copyScanClass3D_ScanSelected(*_scan_in[0], msg_obj_Mirror, object3D::reflectiveSurface);   // points which are located on the object
    copyScanClass3D_ScanSelected(*_scan_in[0], msg_obj_Transp, object3D::transpSurface);   // points which are located on the object

    copyScanClass3D_ScanSelected(*_scan_in[0], msg_affected, object3D::behindErrorS);             // points which are influenced by the object
    copyScanClass3D_ScanSelected(*_scan_in[0], msg_behMirror, object3D::behindReflective);        // points which are influenced by the object
    copyScanClass3D_ScanSelected(*_scan_in[0], msg_behTransp, object3D::behindTransparent);       // points which are influenced by the object

    /*
     * write corner data of object
     */
    for(int i=0; i < _object_size; i++)
    {
      object_corners = _object[i]->getObjectCorners();

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

/*
 * other functions
 */
void separateObjectPoints(mirrordetector::scan3D* scan, std::vector<mirrordetector::object3D*>& object, unsigned int amount_affected, unsigned int& objectCounter, double planeDetection_threshold, double visionCone_threshold, unsigned int planeDetectin_minPoints, double maxDist, double fitnessICP)
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

        n++;
      }
    }

    //identify object planes and corners in scan
    scan->indentifyObjects(tmp_objects, planeDetection_threshold, planeDetectin_minPoints);

    /*
     * identify point cloud for objects and determine the corner points
     */
    if(tmp_objects.size() != 0)
    {
      /*
       * resize Buffer for objects
       */
      if(_object.size() < tmp_objects.size())
      {
        _object.resize(tmp_objects.size());
      }

      /*
       * analyze Types of objects
       */
      for(int i=0; i < tmp_objects.size(); i++)
      {
        if(tmp_objects[i]->analyzeObjectType(scan->getCoords1(), scan->getCoords2(), scan->getIntensity1(), scan->getIntensity2(), scan->getSize(), planeDetection_threshold, visionCone_threshold, maxDist, fitnessICP))
        {
//          cout << "found object type " << tmp_objects[i]->getObjectType() << " for object " << i << endl;
//          cout << "0 = not defined, 2 = eror, 4 = refl. surface, 6 = transp. surface " << endl;
        }
      }

      /*
       * save objects
       */
      for(int i = 0; i < (tmp_objects.size()); i++)
      {
        object[i] = tmp_objects[i];
      }
       objectCounter = tmp_objects.size();
    }
  }

  delete [] affected_coords1;
  delete [] affected_coords2;
  delete [] affected_intens1;
  delete [] affected_intens2;
}

void cleanScan(std::vector<mirrordetector::scan3D*>& scan, std::vector<mirrordetector::object3D*> object, unsigned int amount_objects, double thres_dist, double thres_visionCone)
{

  double* object_corners;
  double* object_coefficents;

  double* scan_points1       = scan[0]->getCoords1();
  double* scan_points2       = scan[0]->getCoords2();

  int scanSize              = scan[0]->getSize();

  object3D::objType* tmp_objectType1 = new object3D::objType[scanSize];
  object3D::objType* tmp_objectType2 = new object3D::objType[scanSize];

  object3D::objType planeAssignedObjectType = object3D::unchecked;

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

  scan[0]->setObjectType1(tmp_objectType1);
  scan[0]->setObjectType2(tmp_objectType2);
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
}

int main(int argc, char **argv)
{
  double dVar = 0;
  int iVar = 0;
  ros::init(argc, argv, "ohm_echo_filter");
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
   * variables for prefilter
   */
  nh_sub.param<double>("min_measure_distance",                    dVar, 0.023);        // minimal distance of a point
  _min_dist = static_cast<double>(dVar);
  nh_sub.param<double>("max_measure_distance",                    dVar, 60.0);         // maximal distance of a point
  _max_dist = static_cast<double>(dVar);

  nh_sub.param<int>("outlierFilter_minNeighbors",                 iVar, 10);            // minimal number of neighbors
  _outlierFilter_minNeighbors = static_cast<int>(iVar);
  nh_sub.param<double>("outlierFilter_searchRadius",              dVar, 0.1);           // search radius for neighbors
  _outlierFilter_searchRadius = static_cast<float>(dVar);

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

  nh_sub.param<double>("fitness_Fct_ICP",                         dVar, 0.005);          // max. allowed result of fitness function of ICP
  _fitnessICP = static_cast<float>(dVar);


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
    if(_new_dataset)
    {
      /*
       * erases points, which are to close to scanner or to far away
       */
      _scan_in[0]->distanceThresFilter(_min_dist, _max_dist);
      _scan_in[0]->validPointFilter();
      /*
       * filter outliers
       */
      _scan_in[0]->outlierFilterRadius(_outlierFilter_minNeighbors, _outlierFilter_searchRadius, object3D::unchecked);

      /*
       *  check for affected data and mark the object_type
       */
      unsigned int amount_affected = _scan_in[0]->identifyReflections(_substract_thres_dist);

      if(amount_affected > 0)
      {
        if(chooseScanCounter >= 0)
        {
          /*
           * filter outliers
           */
          _scan_in[0]->outlierFilterRadius(_outlierFilter_minNeighbors, _outlierFilter_searchRadius, object3D::errorSurface);

          /*
           * separate affected points from scan and identify objects
           */
           separateObjectPoints(_scan_in[0], _object, amount_affected, _object_size, _planeDetection_thres_dist, _visionCone_thres, _planeDetection_minPoints, _maxDistICP, _fitnessICP);
           for(int i=0; i < _object_size; i++)
           {
            /*
             * clean scan based on founded objects
             */
             cleanScan(_scan_in, _object, _object_size, _planeDetection_thres_dist, _visionCone_thres);
           }
        }
        chooseScanCounter++;
      }

      /*
       * publish cleaned scan
       */
      publisherFunc();
      _new_dataset = false;
    }
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Shutting down");

  return 0;
}
