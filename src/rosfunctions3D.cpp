/**
* @file   rosfunctions3D.cpp
* @author Rainer Koch
* @date   21.03.2016
*
*
*/

#include "rosfunctions3D.h"

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
  /*
   * copy data of echo 1
   */
  for(unsigned int i=0; i< size; i++)
  {
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
    }
    /*
     * copy coords 2
     */
    for(unsigned int j = 0; j < sizeAff2; j++)
    {
      cloud_org2->points[j].x = coords2_mirr[3*j];
      cloud_org2->points[j].y = coords2_mirr[3*j+1];
      cloud_org2->points[j].z = coords2_mirr[3*j+2];
    }

    /*
     * get transformation and transform point clouds
     */
    Eigen::Matrix4f Trans = object.getTransformation();

    pcl::transformPointCloud (*cloud_org1, *cloud_trans1, Trans);
    pcl::transformPointCloud (*cloud_org2, *cloud_trans2, Trans);

    /*
     * copy transformed coords 1 into msg
     */
    for(int j = 0; j < sizeAff1; j++)
    {
        msgCoord.objectType[indizes1[j]]        = 3;

        msgCoord.coords[3*indizes1[j]]          = cloud_trans1->points[j].x;
        msgCoord.coords[3*indizes1[j]+1]        = cloud_trans1->points[j].y;
        msgCoord.coords[3*indizes1[j]+2]        = cloud_trans1->points[j].z;

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
      msgCoord.objectType[indizes2[j]]        = 3;

      msgCoord.coords[3*indizes2[j]]          = cloud_trans2->points[j].x;
      msgCoord.coords[3*indizes2[j]+1]        = cloud_trans2->points[j].y;
      msgCoord.coords[3*indizes2[j]+2]        = cloud_trans2->points[j].z;

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
