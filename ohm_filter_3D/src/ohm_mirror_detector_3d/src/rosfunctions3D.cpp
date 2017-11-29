/**
* @file   rosfunctions3D.cpp
* @author Rainer Koch
* @date   21.03.2016
*
*
*/

//#include <vector>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//
//#include <sensor_msgs/LaserScan.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <nav_msgs/Path.h>
//#include <nav_msgs/OccupancyGrid.h>
//#include <nav_msgs/GetMap.h>
//#include <sstream>

#include "rosfunctions3D.h"

//
//void convertPath2ROS(std::vector<cv::Point3f>& coords_path, std::vector<cv::Vec4f>& path_orient, nav_msgs::Path& ros_path)
//{
//  // resize
//  if(ros_path.poses.size() < coords_path.size())
//  {
//    ros_path.poses.resize(coords_path.size());
//  }
//  // copy
//  //cout << "convertPath2ROS" << endl;
//
//
//  for(int i=0; i < coords_path.size(); i++)
//  {
//    ros_path.poses[i].header.seq          = ros_path.header.seq;
//    ros_path.poses[i].header.stamp        = ros_path.header.stamp;
//    ros_path.poses[i].header.frame_id     = ros_path.header.frame_id;
//
//    ros_path.poses[i].pose.position.x = coords_path[i].x;
//    ros_path.poses[i].pose.position.y = coords_path[i].y;
//    ros_path.poses[i].pose.position.z = coords_path[i].z;
//
//    ros_path.poses[i].pose.orientation.x = path_orient[i].val[0];
//    ros_path.poses[i].pose.orientation.y = path_orient[i].val[1];
//    ros_path.poses[i].pose.orientation.z = path_orient[i].val[2];
//    ros_path.poses[i].pose.orientation.w = path_orient[i].val[3];
//
////    cout << "PathR: " << ros_path.poses[i].pose.position.x << "/" << ros_path.poses[i].pose.position.y << "/" << ros_path.poses[i].pose.position.z << endl;
////    cout << "PathC: " << coords_path[i].x << "/" << coords_path[i].y << "/" << coords_path[i].z << endl;
////    cout << "OriR: " << ros_path.poses[i].pose.orientation.x << "/" << ros_path.poses[i].pose.orientation.y << "/" << ros_path.poses[i].pose.orientation.z << "/" << ros_path.poses[i].pose.orientation.w << endl;
////    cout << "OriC: " << path_orient[i].val[0] << "/" << path_orient[i].val[1] << "/" << path_orient[i].val[2] << "/" << path_orient[i].val[3] << endl;
//  }
//}
//
//void convertPath2xyz(nav_msgs::Path& ros_path, std::vector<cv::Point3f>& coords_path, std::vector<cv::Vec4f>& path_orient)
//{
//  // resize
//  if(coords_path.size() < ros_path.poses.size())
//  {
//    coords_path.resize(ros_path.poses.size());
//    path_orient.resize(ros_path.poses.size());
//
//  }
//  // copy
//  //cout << "convertPath2xyz" << endl;
//  for(int i=0; i < ros_path.poses.size(); i++)
//  {
//    coords_path[i].x     = ros_path.poses[i].pose.position.x;
//    coords_path[i].y     = ros_path.poses[i].pose.position.y;
//    coords_path[i].z     = ros_path.poses[i].pose.position.z;
//
//    path_orient[i].val[0] = ros_path.poses[i].pose.orientation.x;
//    path_orient[i].val[1] = ros_path.poses[i].pose.orientation.y;
//    path_orient[i].val[2] = ros_path.poses[i].pose.orientation.z;
//    path_orient[i].val[3] = ros_path.poses[i].pose.orientation.w;
//
////    cout << "PathR: " << ros_path.poses[i].pose.position.x << "/" << ros_path.poses[i].pose.position.y << "/" << ros_path.poses[i].pose.position.z << endl;
////    cout << "PathC: " << coords_path[i].x << "/" << coords_path[i].y << "/" << coords_path[i].z << endl;
////    cout << "OriR: " << ros_path.poses[i].pose.orientation.x << "/" << ros_path.poses[i].pose.orientation.y << "/" << ros_path.poses[i].pose.orientation.z << "/" << ros_path.poses[i].pose.orientation.w << endl;
////    cout << "OriC: " << path_orient[i].val[0] << "/" << path_orient[i].val[1] << "/" << path_orient[i].val[2] << "/" << path_orient[i].val[3] << endl;
//  }
//
//}
//
//void convertMap2ROS(std::vector<cv::Point2f>& coords, std::vector<int>& mask, cv::Vec4f& map_orient, cv::Point3f& map_pos, nav_msgs::OccupancyGrid& map)
//{
//  // resize
//  int mapsize = map.info.width * map.info.height;
//  int width = map.info.width;
//  int height = map.info.height;
//
//  int validPointCounter = 0;
//
//  map.data.resize(mapsize);
//  int index = 0;
//  int pos_x = 0;
//  int pos_y = 0;
//
//  map.info.origin.position.x = map_pos.x;
//  map.info.origin.position.y = map_pos.y;
//  map.info.origin.position.z = 0.0;
//
//  map.info.origin.orientation.x = map_orient.val[0];
//  map.info.origin.orientation.y = map_orient.val[1];
//  map.info.origin.orientation.z = map_orient.val[2];
//  map.info.origin.orientation.w = map_orient.val[3];
//
//
////  	cout << "Map2Ros_mask" << endl;
////  	for(int i=0; i < mask.size(); i++)
////  	{
////  		if((mask[i] > 0))
////  			cout << mask[i] << " / ";
////  	}
////  	cout << endl;
//
//  // copy
//  //cout << "Map2ROS" << endl;
//  for(int i=0; i < mask.size(); i++)
//  {
//	  if(mask[i] != -1)
//	  {
//		  map.data[i] = 0;
//	  }
//	  else
//	  {
//		  map.data[i] = -1;
//	  }
//  }
////  cout << "Convert Ros2Map" << endl;
//  for(int i=0; i < (coords.size()); i++)
//  {
//	  if((coords[i].x != 0) && (coords[i].y != 0))
//	  {
//		  //TODO: Orientation is not used yet.
//		  index = (((coords[i].x-map_pos.x) / map.info.resolution) + (((coords[i].y-map_pos.y) / map.info.resolution) * map.info.width));
//		 // cout << index << "/";
//		  map.data[index] = 100;
//	  }
//  }
////cout << endl;
//}
//
//void convertxy2RosSort(double* coords, double* distance, int size, double angle_inc)
//{
//  for (unsigned int i = 0; i < size; i++)
//  {
//    distance[i] = 0;
//  }
//  int j = 0;
//  int max_inc = 2*M_PI / angle_inc;         // 360° = max_inc
//  double scanangle = 2*M_PI*0.75; // :360*270 = *0.75
//  //cout << "Max" << max_inc << endl;
//   // resize scan to get 360°
//  for (unsigned int i = 0; i < size; i++)
//  {
//    if(!(isnan(coords[2*i+1])))
//    {
//      if(coords[2*i] != 0.0)
//      {
//        // calculate new angle (Book 02/04/15)
//        j = (atan(coords[2*i] / -coords[2*i+1]) / angle_inc);// - scanangle/6/angle_inc);
//        if(j > max_inc)
//        {
//          j =  j - max_inc;
//        }
//        else if(j < 0)
//        {
//          j = j + max_inc;
//        }
//        if(j > 540)
//        {
//          j = j - 540;
//      //    cout << coords[2*i] << " / " << coords[2*i+1] << " i: " << i << " j: " << j  << endl;
//          distance[j] = sqrt(pow((coords[2*i+1]),2.0) + pow((coords[2*i]),2.0));
//       //   cout << (j) << "-" << distance[j] << " / ";
//        }
////        else
////        {
////          j = max_inc -j;
////          cout << coords[2*i+1] << " / " << coords[2*i] << " i: " << i << " j: " << j  << endl;
////          distance[j] = sqrt(pow((coords[2*i+1]),2.0) + pow((coords[2*i]),2.0));
////          cout << (j) << "-" << distance[j] << " / ";
////       }
//      }
//      else
//      {
//        j = max_inc;// - scanangle/6/angle_inc;
//        distance[j] = coords[2*i];
//      }
//
//
//    }
//  }
// // cout << endl;
//}
//
//void convertMap2xy(nav_msgs::OccupancyGrid& map, std::vector<cv::Point2f>& coords, std::vector<int>& mask)
//{
//  // resize
//  unsigned int width = map.info.width;
//  unsigned int height = map.info.height;
//
//
//  int validPointCounter = 0;
//
//  if(coords.size() < map.data.size())
//  {
//    coords.resize(map.data.size());
//    mask.resize(map.data.size());
//  }
//  // copy
////  cout << "Datasize" << map.data.size() << endl;
////  cout << "w/h: " << map.info.width << "/" << map.info.height << endl;
//  int index = 0;
//
//  for(unsigned int i=0; i < height; i++)
//  {
//    for(unsigned int j=0; j < width; j++)
//    {
//      mask[j+i*width] = map.data[j+i*width];
//      if((map.data[j+i*width] != -1) && map.data[j+i*width] != 0)
//      {
//		  //TODO: Orientation is not used yet.
//        coords[index].x = map.info.resolution * j + map.info.origin.position.x;
//        coords[index].y = map.info.resolution * i + map.info.origin.position.y;
//        index++;
//      }
//    }
//  }
//
////  cout << "Map_xy: res:" << map.info.resolution << " h/w: " << map.info.width << "/" << map.info.height << " origin x/y: " << map.info.origin.position.x << "/" << map.info.origin.position.y << endl;
////  for(int i=0; i < coords.size(); i++)
////  {
////	  if((coords[i].x != 0) or (coords[i].y != 0))
////		  cout << coords[i].x << "_" << coords[i].y << " / ";
////  }
////  cout << endl;
//
////  cout << "Mask" << endl;
////  for(int i=0; i < mask.size(); i++)
////  {
////	  if((mask[i] > 0))
////		  cout << mask[i] << " / ";
////  }
////  cout << endl;
//}
//
//void convertROS2xy(sensor_msgs::LaserScan &scan, std::vector<cv::Point2f> &data, float angle_increment, float scanangle)
//{
//  cv::Point2f tmp_point;
//  //cout << "in" << endl;
//
//  for (unsigned int i = 0; i < (scan.ranges.size()); i++)
//  {
////  cout << " angle_diff " << scanangle << endl;
////  cout << " i*angle_inc" << i*angle_increment << endl;
//
//    tmp_point.x = -scan.ranges[i] * sin(angle_increment*i + scanangle/2);
//    tmp_point.y = -scan.ranges[i] * cos(angle_increment*i + scanangle/2);
//    data[i] = tmp_point;
//    //cout << " " << data[i].x << ", " << data[i].y << " / ";
//  }
//  //cout << endl;
//}
//
//void convertxy2ROS(std::vector<cv::Point2f> &data, sensor_msgs::LaserScan &scan, float angle_increment, float scanangle)
//{
//
//  int j = 0;
//  int max_inc = 2*M_PI / angle_increment;         // 360° = max_inc
//  //cout << "xy2ROS:" << endl;
//   // resize scan to get 360°
//  scan.ranges.resize(max_inc);
//  for (unsigned int i = 0; i < (data.size()); i++)
//  {
//    if(data[i].x != 0.0)
//    {
//      // calculate new angle (Book 02/04/15)
//      j = (atan(data[i].y / data[i].x) / angle_increment) - (scanangle/6/angle_increment);
//      if(j > max_inc)
//      {
//        j =  j - max_inc;
//      }
//      else if(j < 0)
//      {
//        j = j + max_inc;
//      }
//     // cout << data[i].x << " / " << data[i].y << " i: " << i << " j: " << j  << endl;
//      scan.ranges[j] = sqrt(pow((data[i].x),2.0) + pow((data[i].y),2.0));
//      //cout << j << "-" << scan.ranges[j] << " / ";
//    }
//    else
//    {
//      j = max_inc - (scanangle/6/angle_increment);
//      scan.ranges[j] = data[i].y;
//    }
//  }
//  //cout << endl;
//}

void copyHeaderScan2Scan(ohm_hokuyo3d::CoordDistMulti scan_in, ohm_hokuyo3d::CoordDistMulti& scan_out)
{
  //TODO:
//  //copy scan
//  scan_out.coorddist[0].coord.header.seq          = scan_in.coorddist[0].coord.header.seq;
//	scan_out.coorddist[0].coord.header.stamp 		    = scan_in.coorddist[0].coord.header.stamp;
//  scan_out.coorddist[0].coord.header.frame_id     = scan_in.coorddist[0].coord.header.frame_id;
//
//	scan_out.coorddist[0].coord.fragmentid        	= scan_in.coorddist[0].coord.fragmentid;
//	scan_out.coorddist[0].coord.fragmentsize        = scan_in.coorddist[0].coord.fragmentsize;
//
//  scan_out.coorddist[0].dist.header.seq           = scan_in.coorddist[0].dist.header.seq;
//  scan_out.coorddist[0].dist.header.stamp         = scan_in.coorddist[0].dist.header.stamp;
//  scan_out.coorddist[0].dist.header.frame_id      = scan_in.coorddist[0].dist.header.frame_id;
//
//  // copy echo 1
//  scan_out.coorddist[1].coord.header.seq          = scan_in.coorddist[1].coord.header.seq;
//  scan_out.coorddist[1].coord.header.stamp        = scan_in.coorddist[1].coord.header.stamp;
//  scan_out.coorddist[1].coord.header.frame_id     = scan_in.coorddist[1].coord.header.frame_id;
//
//  scan_out.coorddist[1].coord.fragmentid          = scan_in.coorddist[1].coord.fragmentid;
//  scan_out.coorddist[1].coord.fragmentsize        = scan_in.coorddist[1].coord.fragmentsize;
//
//  scan_out.coorddist[1].dist.header.seq           = scan_in.coorddist[1].dist.header.seq;
//  scan_out.coorddist[1].dist.header.stamp         = scan_in.coorddist[1].dist.header.stamp;
//  scan_out.coorddist[1].dist.header.frame_id      = scan_in.coorddist[1].dist.header.frame_id;
//
////  // copy echo 2 (not used yet)
////  scan_out.coorddist[2].coord.header.seq          = scan_in.coorddist[2].coord.header.seq;
////  scan_out.coorddist[2].coord.header.stamp        = scan_in.coorddist[2].coord.header.stamp;
////  scan_out.coorddist[2].coord.header.frame_id     = scan_in.coorddist[2].coord.header.frame_id;
////
////  scan_out.coorddist[2].coord.fragmentid          = scan_in.coorddist[2].coord.fragmentid;
////  scan_out.coorddist[2].coord.fragmentsize        = scan_in.coorddist[2].coord.fragmentsize;
////
////  scan_out.coorddist[2].dist.header.seq           = scan_in.coorddist[2].dist.header.seq;
////  scan_out.coorddist[2].dist.header.stamp         = scan_in.coorddist[2].dist.header.stamp;
////  scan_out.coorddist[2].dist.header.frame_id      = scan_in.coorddist[2].dist.header.frame_id;
}

void copyScanClass3D_MultiScan(mirrordetector::scan3D& scan_in, ohm_hokuyo3d::CoordDistMulti& scan_out)
{
  ohm_hokuyo3d::Coord     msgCoord;
  ohm_hokuyo3d::Dist      msgDist;
  ohm_hokuyo3d::CoordDist msgCoordDist;

  ros::Time t = ros::Time::now();

  unsigned int size         = scan_in.getSize();

  double* coord1            = scan_in.getCoords1();
  double* coord2            = scan_in.getCoords2();

  double* normals1          = scan_in.getNormals1();
  double* normals2          = scan_in.getNormals1();

  double* dist1             = scan_in.getDist1();
  double* dist2             = scan_in.getDist2();

  unsigned int* intens1     = scan_in.getIntensity1();
  unsigned int* intens2     = scan_in.getIntensity2();

  bool* mask1               = scan_in.getMask1();
  bool* mask2               = scan_in.getMask2();

  double* phi1              = scan_in.getPhi1();
  double* phi2              = scan_in.getPhi2();

  object3D::objType* objectType1  = scan_in.getObjectType1();
  object3D::objType* objectType2  = scan_in.getObjectType2();
  /*
   * resize msg arrays
   */
  scan_out.coorddist.resize(3);

  msgCoord.coords.resize(3*size);
  msgCoord.normals.resize(3*size);
  msgCoord.mask.resize(size);
  msgCoord.intensity.resize(size);
  msgCoord.objectType.resize(size);


  msgDist.dists.resize(size);
  msgDist.phi.resize(size);
  msgDist.mask.resize(size);

  /* copy header */
  msgCoord.header.seq        = scan_in.getSeqenz();
  msgCoord.header.frame_id   = scan_in.getFrame_id();
  msgCoord.header.stamp      = t.fromNSec(scan_in.getTimeStamp());

  msgCoord.fragmentid        = scan_in.getFragment_id();
  msgCoord.fragmentsize      = scan_in.getFragment_size();

  msgDist.header.seq         = scan_in.getSeqenz();
  msgDist.header.stamp       = t.fromNSec(scan_in.getTimeStamp());
  msgDist.header.frame_id    = scan_in.getFrame_id();

//  //TOTEST:
//  std::stringstream s;
//
//   s << "/home/rainer/workspace/pre1_" << 0 << ".txt";
//   printCoords2File(coord1, size, s.str().c_str());
//   //printCoords2File(coord1, objectType1, object3D::validPoint,  _scan_in[0]->getSize(), s.str().c_str());
//
//   s.str("");
//   s << "/home/rainer/workspace/pre2_" << 0 << ".txt";
//   printCoords2File(coord2, size, s.str().c_str());
//   s.str("");
//
//  //ENDTOTEST

  /*
   * copy data of echo 1
   */
  for(unsigned int i=0; i< size; i++)
  {
    //  /*
    //  *            0: not defined/checked yet
    //  *            1: regular scan point
    //  *            2: error
    //  *            3: behind error
    //  *            4: point on reflective object surface
    //  *            5: reflected point
    //  *            6: point on transparent object surface
    //  *            7: behind transparent object surface
    //  *            8: not a valid point
    //  */

    switch(objectType1[i])
    {
    case object3D::unchecked:
      msgCoord.objectType[i] = 0;
      break;
    case object3D::validPoint:
      msgCoord.objectType[i] = 1;
      break;
    case object3D::errorSurface:
      msgCoord.objectType[i] = 2;
      break;
    case object3D::behindErrorS:
      msgCoord.objectType[i] = 3;
      break;
    case object3D::reflectiveSurface:
      msgCoord.objectType[i] = 4;
      break;
    case object3D::behindReflective:
      msgCoord.objectType[i] = 5;
      break;
    case object3D::transpSurface:
      msgCoord.objectType[i] = 6;
      break;
    case object3D::behindTransparent:
      msgCoord.objectType[i] = 7;
      break;
    case object3D::nanPoint:
      msgCoord.objectType[i] = 8;
      break;
    }

    msgCoord.coords[3*i]          = coord1[3*i];
    msgCoord.coords[3*i+1]        = coord1[3*i+1];
    msgCoord.coords[3*i+2]        = coord1[3*i+2];

    msgCoord.normals[3*i]         = normals1[3*i];
    msgCoord.normals[3*i+1]       = normals1[3*i+1];
    msgCoord.normals[3*i+2]       = normals1[3*i+2];

    msgCoord.mask[i]              = mask1[i];
    msgCoord.intensity[i]         = intens1[i];

    msgDist.dists[i]              = dist1[i];
    msgDist.phi[i]                = phi1[i];
    msgDist.mask[i]               = mask1[i];
  }

  /* fuse msgs */
  msgCoordDist.coord      = msgCoord;
  msgCoordDist.dist       = msgDist;

  scan_out.coorddist[0]   = msgCoordDist;

  /*
   * copy data of echo 2
   */
  for(unsigned int i=0; i< size; i++)
  {
    switch(objectType2[i])
    {
    case object3D::unchecked:
      msgCoord.objectType[i] = 0;
      break;
    case object3D::validPoint:
      msgCoord.objectType[i] = 1;
      break;
    case object3D::errorSurface:
      msgCoord.objectType[i] = 2;
      break;
    case object3D::behindErrorS:
      msgCoord.objectType[i] = 3;
      break;
    case object3D::reflectiveSurface:
      msgCoord.objectType[i] = 4;
      break;
    case object3D::behindReflective:
      msgCoord.objectType[i] = 5;
      break;
    case object3D::transpSurface:
      msgCoord.objectType[i] = 6;
      break;
    case object3D::behindTransparent:
      msgCoord.objectType[i] = 7;
      break;
    case object3D::nanPoint:
      msgCoord.objectType[i] = 8;
      break;
    }
    msgCoord.coords[3*i]      = coord2[3*i];
    msgCoord.coords[3*i+1]    = coord2[3*i+1];
    msgCoord.coords[3*i+2]    = coord2[3*i+2];

    msgCoord.normals[3*i]     = normals2[3*i];
    msgCoord.normals[3*i+1]   = normals2[3*i+1];
    msgCoord.normals[3*i+2]   = normals2[3*i+2];

    msgCoord.mask[i]          = mask2[i];
    msgCoord.intensity[i]     = intens2[i];

    msgDist.dists[i]          = dist2[i];
    msgDist.phi[i]            = phi2[i];
    msgDist.mask[i]           = mask2[i];
  }

  /* fuse msgs */
  msgCoordDist.coord      = msgCoord;
  msgCoordDist.dist       = msgDist;

  scan_out.coorddist[1]   = msgCoordDist;

  /*
   * copy data of echo 3
   */
  for(unsigned int i=0; i< size; i++)
  {
    msgCoord.objectType[i] = 0;

    msgCoord.coords[3*i]      = NAN;
    msgCoord.coords[3*i+1]    = NAN;
    msgCoord.coords[3*i+2]    = NAN;

    msgCoord.normals[3*i]     = NAN;
    msgCoord.normals[3*i+1]   = NAN;
    msgCoord.normals[3*i+2]   = NAN;

    msgCoord.mask[i]          = NAN;
    msgCoord.intensity[i]     = NAN;

    msgDist.dists[i]          = NAN;
    msgDist.phi[i]            = NAN;
    msgDist.mask[i]           = 0;
  }

  /* fuse msgs */
  msgCoordDist.coord      = msgCoord;
  msgCoordDist.dist       = msgDist;

  scan_out.coorddist[2]   = msgCoordDist;

//  //TOTEST:
//  int ob0 = 0;
//  int ob1 = 1;
//  int ob2 = 2;
//  int ob3 = 3;
//  int ob4 = 4;
//  int ob5 = 5;
//  int ob6 = 6;
//  int ob7 = 7;
//  int ob8 = 8;
//  double* c1;
//  for(int i = 0; i < size; i++)
//  {
//    switch(scan_out.coorddist[0].coord.objectType[i])
//    {
//    case 0:
//      ob0++;
//      break;
//    case 1:
//      ob1++;
//      break;
//    case 2:
//      ob2++;
//      break;
//    case 3:
//      ob3++;
//      break;
//    case 4:
//      ob4++;
//      break;
//    case 5:
//      ob5++;
//      break;
//    case 6:
//      ob6++;
//      break;
//    case 7:
//      ob7++;
//      break;
//    case 8:
//      ob8++;
//      break;
//    }
//  }
//  cout << " OBS: " << ob0 << "/" << ob1 << "/" << ob2 << "/" << ob3 << "/" << ob4 << "/" << ob5 << "/" << ob6 << "/" << ob7 << "/" << ob8 << endl;

}

void copyScanClass3D_ScanSelected(mirrordetector::scan3D& scan_in, ohm_hokuyo3d::CoordDist& scan_out, object3D::objType type)
{
  ohm_hokuyo3d::Coord     msgCoord;
  ohm_hokuyo3d::Dist      msgDist;

  ros::Time t = ros::Time::now();

  unsigned int size         = scan_in.getSize();

  double* coord1            = scan_in.getCoords1();
  double* coord2            = scan_in.getCoords2();

  double* normals1          = scan_in.getNormals1();
  double* normals2          = scan_in.getNormals2();

  double* dist1             = scan_in.getDist1();
  double* dist2             = scan_in.getDist2();

  unsigned int* intens1     = scan_in.getIntensity1();
  unsigned int* intens2     = scan_in.getIntensity2();

  bool* mask1               = scan_in.getMask1();
  bool* mask2               = scan_in.getMask2();

  double* phi1              = scan_in.getPhi1();
  double* phi2              = scan_in.getPhi2();

  object3D::objType* objectType1  = scan_in.getObjectType1();
  object3D::objType* objectType2  = scan_in.getObjectType2();

  //TOTEST:
//    cout << "pub test" << endl;
//    for(int i=0; i < 100; i++)
//    {
//      cout << objectType1[i] << " ";
//    }
//    cout << endl;
//    cout << "pub test done " << endl;

  /*
   * resize msg arrays
   */
  msgCoord.objectType.resize(size);
  msgCoord.coords.resize(3*size);
  msgCoord.normals.resize(3*size);
  msgCoord.mask.resize(size);
  msgCoord.intensity.resize(size);

  msgDist.dists.resize(size);
  msgDist.phi.resize(size);
  msgDist.mask.resize(size);

  /* copy header */
  msgCoord.header.seq        = scan_in.getSeqenz();
  msgCoord.header.frame_id   = scan_in.getFrame_id();
  msgCoord.header.stamp      = t.fromNSec(scan_in.getTimeStamp());

  msgCoord.fragmentid        = scan_in.getFragment_id();
  msgCoord.fragmentsize      = scan_in.getFragment_size();

  msgDist.header.seq         = scan_in.getSeqenz();
  msgDist.header.stamp       = t.fromNSec(scan_in.getTimeStamp());
  msgDist.header.frame_id    = scan_in.getFrame_id();

//  //TOTEST:
//  int c1 = 0;
//  int c2 = 0;
//  int c3 = 0;

  /*
   * copy data of choosen objectType
   */
  if(type == object3D::validPoint) 	    	  // points are free of influences
  {
    for(unsigned int i=0; i< size; i++)  // points are free of influences
    {
      if(objectType1[i] == type)
      {
        msgCoord.objectType[i]        = 1;

        msgCoord.coords[3*i]          = coord1[3*i];
        msgCoord.coords[3*i+1]        = coord1[3*i+1];
        msgCoord.coords[3*i+2]        = coord1[3*i+2];

        msgCoord.normals[3*i]         = normals1[3*i];
        msgCoord.normals[3*i+1]       = normals1[3*i+1];
        msgCoord.normals[3*i+2]       = normals1[3*i+2];

        msgCoord.mask[i]              = mask1[i];
        msgCoord.intensity[i]         = intens1[i];

        msgDist.dists[i]              = dist1[i];
        msgDist.phi[i]                = phi1[i];
        msgDist.mask[i]               = mask1[i];
      }
      else                              // points are not free of influences
      {
        msgCoord.objectType[i]        = 0;

        msgCoord.coords[3*i]          = NAN;
        msgCoord.coords[3*i+1]        = NAN;
        msgCoord.coords[3*i+2]        = NAN;

        msgCoord.normals[3*i]         = NAN;
        msgCoord.normals[3*i+1]       = NAN;
        msgCoord.normals[3*i+2]       = NAN;

        msgCoord.mask[i]              = NAN;
        msgCoord.intensity[i]         = NAN;

        msgDist.dists[i]              = NAN;
        msgDist.phi[i]                = NAN;
        msgDist.mask[i]               = 0;
      }
    }
  }
  else if(type == object3D::errorSurface)  	// points are located on the object
  {
    for(unsigned int i=0; i< size; i++)
    {
      if(objectType1[i] == object3D::errorSurface) // echo 0 is on the object
      {
        msgCoord.objectType[i]        = 2;

        msgCoord.coords[3*i]          = coord1[3*i];
        msgCoord.coords[3*i+1]        = coord1[3*i+1];
        msgCoord.coords[3*i+2]        = coord1[3*i+2];

        msgCoord.normals[3*i]         = normals1[3*i];
        msgCoord.normals[3*i+1]       = normals1[3*i+1];
        msgCoord.normals[3*i+2]       = normals1[3*i+2];

        msgCoord.mask[i]              = mask1[i];
        msgCoord.intensity[i]         = intens1[i];

        msgDist.dists[i]              = dist1[i];
        msgDist.phi[i]                = phi1[i];
        msgDist.mask[i]               = mask1[i];
      }
      else if(objectType2[i] == object3D::errorSurface) // echo 0 is behind the object => echo 1 is on the object
      {
        msgCoord.objectType[i]        = 2;

        msgCoord.coords[3*i]          = coord2[3*i];
        msgCoord.coords[3*i+1]        = coord2[3*i+1];
        msgCoord.coords[3*i+2]        = coord2[3*i+2];

        msgCoord.normals[3*i]         = normals2[3*i];
        msgCoord.normals[3*i+1]       = normals2[3*i+1];
        msgCoord.normals[3*i+2]       = normals2[3*i+2];

        msgCoord.mask[i]              = mask2[i];
        msgCoord.intensity[i]         = intens2[i];

        msgDist.dists[i]              = dist2[i];
        msgDist.phi[i]                = phi2[i];
        msgDist.mask[i]               = mask2[i];

      }
      else                            // echo 0 is not on or behind the object
      {
        msgCoord.objectType[i]        = 0;

        msgCoord.coords[3*i]          = NAN;
        msgCoord.coords[3*i+1]        = NAN;
        msgCoord.coords[3*i+2]        = NAN;

        msgCoord.normals[3*i]         = NAN;
        msgCoord.normals[3*i+1]       = NAN;
        msgCoord.normals[3*i+2]       = NAN;

        msgCoord.mask[i]              = NAN;
        msgCoord.intensity[i]         = NAN;

        msgDist.dists[i]              = NAN;
        msgDist.phi[i]                = NAN;
        msgDist.mask[i]               = 0;
      }
    }
  }
  else if(type == object3D::behindErrorS)  	      // points are located behind the object (influenced point)
  {
    for(unsigned int i=0; i< size; i++)
    {
      if(objectType1[i] == object3D::behindErrorS) // echo 0 is behind the object (influenced point)
      {
        msgCoord.objectType[i]        = 3;

        msgCoord.coords[3*i]          = coord1[3*i];
        msgCoord.coords[3*i+1]        = coord1[3*i+1];
        msgCoord.coords[3*i+2]        = coord1[3*i+2];

        msgCoord.normals[3*i]         = normals1[3*i];
        msgCoord.normals[3*i+1]       = normals1[3*i+1];
        msgCoord.normals[3*i+2]       = normals1[3*i+2];

        msgCoord.mask[i]              = mask1[i];
        msgCoord.intensity[i]         = intens1[i];

        msgDist.dists[i]              = dist1[i];
        msgDist.phi[i]                = phi1[i];
        msgDist.mask[i]               = mask1[i];
      }
      else if(objectType2[i] == object3D::behindErrorS) // echo 0 is on the object => echo 1 is behind the object (influenced)
      {
        msgCoord.objectType[i]        = 3;

        msgCoord.coords[3*i]          = coord2[3*i];
        msgCoord.coords[3*i+1]        = coord2[3*i+1];
        msgCoord.coords[3*i+2]        = coord2[3*i+2];

        msgCoord.normals[3*i]         = normals2[3*i];
        msgCoord.normals[3*i+1]       = normals2[3*i+1];
        msgCoord.normals[3*i+2]       = normals2[3*i+2];

        msgCoord.mask[i]              = mask2[i];
        msgCoord.intensity[i]         = intens2[i];

        msgDist.dists[i]              = dist2[i];
        msgDist.phi[i]                = phi2[i];
        msgDist.mask[i]               = mask2[i];
      }
      else                            // echo 0 is not behind or on the object
      {
        msgCoord.objectType[i]        = 0;

        msgCoord.coords[3*i]          = NAN;
        msgCoord.coords[3*i+1]        = NAN;
        msgCoord.coords[3*i+2]        = NAN;

        msgCoord.normals[3*i]         = NAN;
        msgCoord.normals[3*i+1]       = NAN;
        msgCoord.normals[3*i+2]       = NAN;

        msgCoord.mask[i]              = NAN;
        msgCoord.intensity[i]         = NAN;

        msgDist.dists[i]              = NAN;
        msgDist.phi[i]                = NAN;
        msgDist.mask[i]               = 0;
      }
    }
  }
  else if(type == object3D::reflectiveSurface)   // points are located on the object
  {
    for(unsigned int i=0; i< size; i++)
    {
      if(objectType1[i] == object3D::reflectiveSurface) // echo 0 is on the object
      {
        msgCoord.objectType[i]        = 2;

        msgCoord.coords[3*i]          = coord1[3*i];
        msgCoord.coords[3*i+1]        = coord1[3*i+1];
        msgCoord.coords[3*i+2]        = coord1[3*i+2];

        msgCoord.normals[3*i]         = normals1[3*i];
        msgCoord.normals[3*i+1]       = normals1[3*i+1];
        msgCoord.normals[3*i+2]       = normals1[3*i+2];

        msgCoord.mask[i]              = mask1[i];
        msgCoord.intensity[i]         = intens1[i];

        msgDist.dists[i]              = dist1[i];
        msgDist.phi[i]                = phi1[i];
        msgDist.mask[i]               = mask1[i];
      }
      else if(objectType2[i] == object3D::reflectiveSurface) // echo 0 is behind the object => echo 1 is on the object
      {
        msgCoord.objectType[i]        = 2;

        msgCoord.coords[3*i]          = coord2[3*i];
        msgCoord.coords[3*i+1]        = coord2[3*i+1];
        msgCoord.coords[3*i+2]        = coord2[3*i+2];

        msgCoord.normals[3*i]         = normals2[3*i];
        msgCoord.normals[3*i+1]       = normals2[3*i+1];
        msgCoord.normals[3*i+2]       = normals2[3*i+2];

        msgCoord.mask[i]              = mask2[i];
        msgCoord.intensity[i]         = intens2[i];

        msgDist.dists[i]              = dist2[i];
        msgDist.phi[i]                = phi2[i];
        msgDist.mask[i]               = mask2[i];

      }
      else                            // echo 0 is not on or behind the object
      {
        msgCoord.objectType[i]        = 0;

        msgCoord.coords[3*i]          = NAN;
        msgCoord.coords[3*i+1]        = NAN;
        msgCoord.coords[3*i+2]        = NAN;

        msgCoord.normals[3*i]         = NAN;
        msgCoord.normals[3*i+1]       = NAN;
        msgCoord.normals[3*i+2]       = NAN;

        msgCoord.mask[i]              = NAN;
        msgCoord.intensity[i]         = NAN;

        msgDist.dists[i]              = NAN;
        msgDist.phi[i]                = NAN;
        msgDist.mask[i]               = 0;
      }
    }
  }
  else if(type == object3D::behindReflective)          // points are located behind the object (influenced point)
  {
    for(unsigned int i=0; i< size; i++)
    {
      if(objectType1[i] == object3D::behindReflective) // echo 0 is behind the object (influenced point)
      {
        msgCoord.objectType[i]        = 3;

        msgCoord.coords[3*i]          = coord1[3*i];
        msgCoord.coords[3*i+1]        = coord1[3*i+1];
        msgCoord.coords[3*i+2]        = coord1[3*i+2];

        msgCoord.normals[3*i]         = normals1[3*i];
        msgCoord.normals[3*i+1]       = normals1[3*i+1];
        msgCoord.normals[3*i+2]       = normals1[3*i+2];

        msgCoord.mask[i]              = mask1[i];
        msgCoord.intensity[i]         = intens1[i];

        msgDist.dists[i]              = dist1[i];
        msgDist.phi[i]                = phi1[i];
        msgDist.mask[i]               = mask1[i];
      }
      else if(objectType2[i] == object3D::behindReflective) // echo 0 is on the object => echo 1 is behind the object (influenced)
      {
        msgCoord.objectType[i]        = 3;

        msgCoord.coords[3*i]          = coord2[3*i];
        msgCoord.coords[3*i+1]        = coord2[3*i+1];
        msgCoord.coords[3*i+2]        = coord2[3*i+2];

        msgCoord.normals[3*i]         = normals2[3*i];
        msgCoord.normals[3*i+1]       = normals2[3*i+1];
        msgCoord.normals[3*i+2]       = normals2[3*i+2];

        msgCoord.mask[i]              = mask2[i];
        msgCoord.intensity[i]         = intens2[i];

        msgDist.dists[i]              = dist2[i];
        msgDist.phi[i]                = phi2[i];
        msgDist.mask[i]               = mask2[i];
      }
      else                            // echo 0 is not behind or on the object
      {
        msgCoord.objectType[i]        = 0;

        msgCoord.coords[3*i]          = NAN;
        msgCoord.coords[3*i+1]        = NAN;
        msgCoord.coords[3*i+2]        = NAN;

        msgCoord.normals[3*i]         = NAN;
        msgCoord.normals[3*i+1]       = NAN;
        msgCoord.normals[3*i+2]       = NAN;

        msgCoord.mask[i]              = NAN;
        msgCoord.intensity[i]         = NAN;

        msgDist.dists[i]              = NAN;
        msgDist.phi[i]                = NAN;
        msgDist.mask[i]               = 0;
      }
    }
  }
  else if(type == object3D::transpSurface)   // points are located on the object
    {
      for(unsigned int i=0; i< size; i++)
      {
        if(objectType1[i] == object3D::transpSurface) // echo 0 is on the object
        {
          msgCoord.objectType[i]        = 2;

          msgCoord.coords[3*i]          = coord1[3*i];
          msgCoord.coords[3*i+1]        = coord1[3*i+1];
          msgCoord.coords[3*i+2]        = coord1[3*i+2];

          msgCoord.normals[3*i]         = normals1[3*i];
          msgCoord.normals[3*i+1]       = normals1[3*i+1];
          msgCoord.normals[3*i+2]       = normals1[3*i+2];

          msgCoord.mask[i]              = mask1[i];
          msgCoord.intensity[i]         = intens1[i];

          msgDist.dists[i]              = dist1[i];
          msgDist.phi[i]                = phi1[i];
          msgDist.mask[i]               = mask1[i];
        }
        else if(objectType2[i] == object3D::transpSurface) // echo 0 is behind the object => echo 1 is on the object
        {
          msgCoord.objectType[i]        = 2;

          msgCoord.coords[3*i]          = coord2[3*i];
          msgCoord.coords[3*i+1]        = coord2[3*i+1];
          msgCoord.coords[3*i+2]        = coord2[3*i+2];

          msgCoord.normals[3*i]         = normals2[3*i];
          msgCoord.normals[3*i+1]       = normals2[3*i+1];
          msgCoord.normals[3*i+2]       = normals2[3*i+2];

          msgCoord.mask[i]              = mask2[i];
          msgCoord.intensity[i]         = intens2[i];

          msgDist.dists[i]              = dist2[i];
          msgDist.phi[i]                = phi2[i];
          msgDist.mask[i]               = mask2[i];

        }
        else                            // echo 0 is not on or behind the object
        {
          msgCoord.objectType[i]        = 0;

          msgCoord.coords[3*i]          = NAN;
          msgCoord.coords[3*i+1]        = NAN;
          msgCoord.coords[3*i+2]        = NAN;

          msgCoord.normals[3*i]         = NAN;
          msgCoord.normals[3*i+1]       = NAN;
          msgCoord.normals[3*i+2]       = NAN;

          msgCoord.mask[i]              = NAN;
          msgCoord.intensity[i]         = NAN;

          msgDist.dists[i]              = NAN;
          msgDist.phi[i]                = NAN;
          msgDist.mask[i]               = 0;
        }
      }
    }
    else if(type == object3D::behindTransparent)          // points are located behind the object (influenced point)
    {
      for(unsigned int i=0; i< size; i++)
      {
        if(objectType1[i] == object3D::behindTransparent) // echo 0 is behind the object (influenced point)
        {
          msgCoord.objectType[i]        = 3;

          msgCoord.coords[3*i]          = coord1[3*i];
          msgCoord.coords[3*i+1]        = coord1[3*i+1];
          msgCoord.coords[3*i+2]        = coord1[3*i+2];

          msgCoord.normals[3*i]         = normals1[3*i];
          msgCoord.normals[3*i+1]       = normals1[3*i+1];
          msgCoord.normals[3*i+2]       = normals1[3*i+2];

          msgCoord.mask[i]              = mask1[i];
          msgCoord.intensity[i]         = intens1[i];

          msgDist.dists[i]              = dist1[i];
          msgDist.phi[i]                = phi1[i];
          msgDist.mask[i]               = mask1[i];
        }
        else if(objectType2[i] == object3D::behindTransparent) // echo 0 is on the object => echo 1 is behind the object (influenced)
        {
          msgCoord.objectType[i]        = 3;

          msgCoord.coords[3*i]          = coord2[3*i];
          msgCoord.coords[3*i+1]        = coord2[3*i+1];
          msgCoord.coords[3*i+2]        = coord2[3*i+2];

          msgCoord.normals[3*i]         = normals2[3*i];
          msgCoord.normals[3*i+1]       = normals2[3*i+1];
          msgCoord.normals[3*i+2]       = normals2[3*i+2];

          msgCoord.mask[i]              = mask2[i];
          msgCoord.intensity[i]         = intens2[i];

          msgDist.dists[i]              = dist2[i];
          msgDist.phi[i]                = phi2[i];
          msgDist.mask[i]               = mask2[i];
        }
        else                            // echo 0 is not behind or on the object
        {
          msgCoord.objectType[i]        = 0;

          msgCoord.coords[3*i]          = NAN;
          msgCoord.coords[3*i+1]        = NAN;
          msgCoord.coords[3*i+2]        = NAN;

          msgCoord.normals[3*i]         = NAN;
          msgCoord.normals[3*i+1]       = NAN;
          msgCoord.normals[3*i+2]       = NAN;

          msgCoord.mask[i]              = NAN;
          msgCoord.intensity[i]         = NAN;

          msgDist.dists[i]              = NAN;
          msgDist.phi[i]                = NAN;
          msgDist.mask[i]               = 0;
        }
      }
    }
  else                                  // point is not chosen
  {
    for(unsigned int i=0; i< size; i++)
    {
      msgCoord.objectType[i]        = 0;

      msgCoord.coords[3*i]          = NAN;
      msgCoord.coords[3*i+1]        = NAN;
      msgCoord.coords[3*i+2]        = NAN;

      msgCoord.normals[3*i]         = NAN;
      msgCoord.normals[3*i+1]       = NAN;
      msgCoord.normals[3*i+2]       = NAN;

      msgCoord.mask[i]              = NAN;
      msgCoord.intensity[i]         = NAN;

      msgDist.dists[i]              = NAN;
      msgDist.phi[i]                = NAN;
      msgDist.mask[i]               = 0;
    }
  }

  /* fuse msgs */
  scan_out.coord      = msgCoord;
  scan_out.dist       = msgDist;
}

void copyReprojectedScan(mirrordetector::scan3D& scan_in, ohm_hokuyo3d::CoordDist& scan_out, mirrordetector::object3D& object)
{
  ohm_hokuyo3d::Coord     msgCoord;
  ohm_hokuyo3d::Dist      msgDist;

  ros::Time t = ros::Time::now();

  unsigned int sizeScan     = scan_in.getSize();

  double* coords1           = scan_in.getCoords1();
  double* coords2           = scan_in.getCoords2();

  double* normals1          = scan_in.getNormals1();
  double* normals2          = scan_in.getNormals2();

  double* dist1             = scan_in.getDist1();
  double* dist2             = scan_in.getDist2();

  unsigned int* intens1     = scan_in.getIntensity1();
  unsigned int* intens2     = scan_in.getIntensity2();

  bool* mask1               = scan_in.getMask1();
  bool* mask2               = scan_in.getMask2();

  double* phi1              = scan_in.getPhi1();
  double* phi2              = scan_in.getPhi2();

  bool* maskBehindObject1   = new bool[sizeScan];
  bool* maskBehindObject2   = new bool[sizeScan];

  unsigned int sizeAff1     = 0;
  unsigned int sizeAff2     = 0;

  object3D::objType* objectType1  = scan_in.getObjectType1();
  object3D::objType* objectType2  = scan_in.getObjectType2();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_org1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_org2 (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans2 (new pcl::PointCloud<pcl::PointXYZ>);

  /*
   * resize msg arrays
   */
  msgCoord.objectType.resize(sizeScan);
  msgCoord.coords.resize(3*sizeScan);
  msgCoord.normals.resize(3*sizeScan);
  msgCoord.mask.resize(sizeScan);
  msgCoord.intensity.resize(sizeScan);

  msgDist.dists.resize(sizeScan);
  msgDist.phi.resize(sizeScan);
  msgDist.mask.resize(sizeScan);

  /* copy header */
  msgCoord.header.seq        = scan_in.getSeqenz();
  msgCoord.header.frame_id   = scan_in.getFrame_id();
  msgCoord.header.stamp      = t.fromNSec(scan_in.getTimeStamp());

  msgCoord.fragmentid        = scan_in.getFragment_id();
  msgCoord.fragmentsize      = scan_in.getFragment_size();

  msgDist.header.seq         = scan_in.getSeqenz();
  msgDist.header.stamp       = t.fromNSec(scan_in.getTimeStamp());
  msgDist.header.frame_id    = scan_in.getFrame_id();

  /*
   * copy empty data
   */
  for(unsigned int j = 0; j < sizeScan; j++)
  {
      msgCoord.objectType[j]        = 0;

      msgCoord.coords[3*j]          = NAN;
      msgCoord.coords[3*j+1]        = NAN;
      msgCoord.coords[3*j+2]        = NAN;

      msgCoord.normals[3*j]         = NAN;
      msgCoord.normals[3*j+1]       = NAN;
      msgCoord.normals[3*j+2]       = NAN;

      msgCoord.mask[j]              = NAN;
      msgCoord.intensity[j]         = NAN;

      msgDist.dists[j]              = NAN;
      msgDist.phi[j]                = NAN;
      msgDist.mask[j]               = 0;
  }

  /*
   * count amount of points behind the reflective area
   */
  for(unsigned int j = 0; j < sizeScan; j++)
  {
    maskBehindObject1[j] = false;
    maskBehindObject2[j] = false;

    if((!isnan(coords1[3*j])) && (objectType1[j] == object3D::behindReflective))
    {
      if(behindPlane(coords1[3*j], coords1[3*j+1], coords1[3*j+2], object.getCoefficents()))
      {
        maskBehindObject1[j] = true;
        sizeAff1++;
      }
    }

    if((!isnan(coords2[3*j])) && (objectType2[j] == object3D::behindReflective))
    {
      if(behindPlane(coords2[3*j], coords2[3*j+1], coords2[3*j+2], object.getCoefficents()))
      {
        maskBehindObject2[j] = true;
        sizeAff2++;
      }
    }
  }
//  cout << "Aff: " << sizeAff1 << "/" << sizeAff2 << endl;

  /*
   * only, if there are points behind this plane
   */
  if((sizeAff1) > 0 or (sizeAff2 > 0))
  {
    cloud_org1->points.resize(sizeAff1);
    cloud_trans1->points.resize(sizeAff1);

    cloud_org2->points.resize(sizeAff2);
    cloud_trans2->points.resize(sizeAff2);

    unsigned int* indizes1  = new unsigned int[sizeAff1];
    unsigned int* indizes2  = new unsigned int[sizeAff2];

    double* tmp_coords1     = new double[3*sizeAff1];
    double* tmp_coords2     = new double[3*sizeAff2];

    double* coords1_mirr    = new double[3*sizeAff1];
    double* coords2_mirr    = new double[3*sizeAff2];

    unsigned int m1         = 0;
    unsigned int m2         = 0;

    /*
     * copy coords and indizes
     */
    for(unsigned int j = 0; j < sizeScan; j++)
    {
      /*
       * copy coords 1
       */
      if((maskBehindObject1[j] == true))
      {
        tmp_coords1[3*m1]   = coords1[3*j];
        tmp_coords1[3*m1+1] = coords1[3*j+1];
        tmp_coords1[3*m1+2] = coords1[3*j+2];
        indizes1[m1] = j;
        m1++;
      }

      /*
       * copy coords 2
       */
      if((maskBehindObject2[j] == true))
      {
        tmp_coords2[3*m2]   = coords2[3*j];
        tmp_coords2[3*m2+1] = coords2[3*j+1];
        tmp_coords2[3*m2+2] = coords2[3*j+2];
        indizes2[m2] = j;
        m2++;
      }
    }

    /*
     * mirror temporary coords on reflective area
     */
    if(sizeAff1 > 0)
      mirrorOnPlane(tmp_coords1, coords1_mirr, object.getCoefficents(), sizeAff1);
    if(sizeAff2 > 0)
      mirrorOnPlane(tmp_coords2, coords2_mirr, object.getCoefficents(), sizeAff2);

    /*
     * put them into point cloud to transform them
     */
    /*
     * copy coords 1
     */
    for(unsigned int j = 0; j < sizeAff1; j++)
    {
        cloud_org1->points[j].x = coords1_mirr[3*j];
        cloud_org1->points[j].y = coords1_mirr[3*j+1];
        cloud_org1->points[j].z = coords1_mirr[3*j+2];
  //      if(m1 < 10)
  //      {
  //        cout << cloud_org1->points[m1].x << " " << cloud_org1->points[m1].y << " " << cloud_org1->points[m1].z << endl;
  //      }
    }
    /*
     * copy coords 2
     */
    for(unsigned int j = 0; j < sizeAff2; j++)
    {
      cloud_org2->points[j].x = coords2_mirr[3*j];
      cloud_org2->points[j].y = coords2_mirr[3*j+1];
      cloud_org2->points[j].z = coords2_mirr[3*j+2];
  //      if(m2 < 10)
  //      {
  //        cout << cloud_org2->points[m2].x << " " << cloud_org2->points[m2].y << " " << cloud_org2->points[m2].z << endl;
  //      }
    }

    /*
     * get transformation and transform point clouds
     */
    Eigen::Matrix4f Trans = object.getTransformation();

//    cout << "Trans:" << endl;
//    cout << Trans << endl;

    pcl::transformPointCloud (*cloud_org1, *cloud_trans1, Trans);
    pcl::transformPointCloud (*cloud_org2, *cloud_trans2, Trans);

    /*
     * copy transformed coords 1 into msg
     */
    for(int j = 0; j < sizeAff1; j++)
    {

  //      if(j < 10)
  //      {
  //        cout << cloud_trans1->points[j].x << " " << cloud_trans1->points[j].y << " " << cloud_trans1->points[j].z << endl;
  //      }

        msgCoord.objectType[indizes1[j]]        = 3;

        msgCoord.coords[3*indizes1[j]]          = cloud_trans1->points[j].x;
        msgCoord.coords[3*indizes1[j]+1]        = cloud_trans1->points[j].y;
        msgCoord.coords[3*indizes1[j]+2]        = cloud_trans1->points[j].z;

        //TODO_: Have to calculated new
        msgCoord.normals[3*indizes1[j]]         = normals1[3*indizes1[j]];
        msgCoord.normals[3*indizes1[j]+1]       = normals1[3*indizes1[j]+1];
        msgCoord.normals[3*indizes1[j]+2]       = normals1[3*indizes1[j]+2];

        msgCoord.mask[indizes1[j]]              = mask1[indizes1[j]];
        msgCoord.intensity[indizes1[j]]         = intens1[indizes1[j]];

        msgDist.dists[indizes1[j]]              = dist1[indizes1[j]];
        msgDist.phi[indizes1[j]]                = phi1[indizes1[j]];
        msgDist.mask[indizes1[j]]               = mask1[indizes1[j]];
    }
    /*
     * copy transformed coords 2 into msg
     */
    for(int j = 0; j < sizeAff2; j++)
    {
  //      if(j < 10)
  //      {
  //        cout << cloud_trans2->points[j].x << " " << cloud_trans2->points[j].y << " " << cloud_trans2->points[j].z << endl;
  //      }

      msgCoord.objectType[indizes2[j]]        = 3;

      msgCoord.coords[3*indizes2[j]]          = cloud_trans2->points[j].x;
      msgCoord.coords[3*indizes2[j]+1]        = cloud_trans2->points[j].y;
      msgCoord.coords[3*indizes2[j]+2]        = cloud_trans2->points[j].z;

      //TODO_: Have to calculated new
      msgCoord.normals[3*indizes2[j]]         = normals2[3*indizes2[j]];
      msgCoord.normals[3*indizes2[j]+1]       = normals2[3*indizes2[j]+1];
      msgCoord.normals[3*indizes2[j]+2]       = normals2[3*indizes2[j]+2];

      msgCoord.mask[indizes2[j]]              = mask2[indizes2[j]];
      msgCoord.intensity[indizes2[j]]         = intens2[indizes2[j]];

      msgDist.dists[indizes2[j]]              = dist2[indizes2[j]];
      msgDist.phi[indizes2[j]]                = phi2[indizes2[j]];
      msgDist.mask[indizes2[j]]               = mask2[indizes2[j]];
    }
  }
  /* fuse msgs */
  scan_out.coord      = msgCoord;
  scan_out.dist       = msgDist;
}
