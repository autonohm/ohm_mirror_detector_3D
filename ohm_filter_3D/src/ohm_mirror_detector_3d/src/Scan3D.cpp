/**
* @file   Scan3D.cpp
* @author Rainer Koch
* @date   21.03.2016
*
*
*/

#include "Scan3D.h"


namespace mirrordetector
{
  scan3D::scan3D(unsigned int seq, unsigned long timestamp, std::string frame_id, unsigned int fragmentid, unsigned int fragement_size, unsigned int size, unsigned int stepsPhi, double* coords_1, unsigned int* intens_1, double* coords_2, unsigned int* intens_2)
  {
    _seq            = seq;
    _time_stamp     = timestamp;
    _frame_id       = frame_id;
    _fragmentid     = fragmentid;
    _fragmet_size   = fragement_size;
    _pub_Results    = 0;

    _size           = size;
    _stepsPhi       = stepsPhi;


    _objectType_1   = NULL;
    _objectType_2   = NULL;

    _coords_1       = new double[3*_size];
    _normals_1      = NULL;

    _mask_1         = NULL;
    _dist_1         = NULL;
    _phi_1          = NULL;
    _intensity_1    = new unsigned int[_size];

    _coords_2       = new double[3*_size];
    _normals_2      = NULL;

    _mask_2         = NULL;
    _dist_2         = NULL;
    _phi_2          = NULL;
    _intensity_2    = new unsigned int[_size];

    memcpy(_coords_1, coords_1, 3*_size*sizeof(double));
    memcpy(_intensity_1, intens_1, _size*sizeof(unsigned int));

    memcpy(_coords_2, coords_2, 3*_size*sizeof(double));
    memcpy(_intensity_2, intens_2, _size*sizeof(unsigned int));

    _T = new obvious::Matrix(4,4);
    _T->setIdentity();
  }

  scan3D::~scan3D()
  {
    delete [] _objectType_1;
    delete [] _objectType_2;

    delete [] _dist_1;
    delete [] _coords_1;
    delete [] _normals_1;
    delete [] _mask_1;
    delete [] _intensity_1;

    delete [] _dist_2;
    delete [] _coords_2;
    delete [] _normals_2;
    delete [] _mask_2;
    delete [] _intensity_2;

    //cout << "delete scan3D" << endl;
  }

  void scan3D::setCoords1(double* coords)
  {
    memcpy(_coords_1, coords, 3*_size*sizeof(double));
  }
  double* scan3D::getCoords1()
  {
    return _coords_1;
  }

  void scan3D::setNormals1(double* normals)
  {
    if(!_normals_1) _normals_1     = new double[3*_size];
    memcpy(_normals_1, normals, 3*_size*sizeof(double));
  }
  double* scan3D::getNormals1()
  {
    return _normals_1;
  }

  void scan3D::setMask1(bool* mask)
  {
    if(!_mask_1) _mask_1     = new bool[_size];
    memcpy(_mask_1, mask, _size*sizeof(bool));
  }
  bool* scan3D::getMask1()
  {
    return _mask_1;
  }

  void scan3D::setIntensity1(unsigned int* intens)
  {
    memcpy(_intensity_1, intens, _size*sizeof(unsigned int));
  }
  unsigned int* scan3D::getIntensity1()
  {
    return _intensity_1;
  }

  void scan3D::setDist1(double* dist)
  {
    if(!_dist_1) _dist_1     = new double[_size];
    memcpy(_dist_1, dist, _size*sizeof(*dist));
  }
  double* scan3D::getDist1()
  {
    return _dist_1;
  }

  void scan3D::setPhi1(double* phi)
  {
    if(!_phi_1) _phi_1     = new double[_size];
    memcpy(_phi_1, phi, _size*sizeof(double));
  }
  double* scan3D::getPhi1()
  {
    return _phi_1;
  }

  void scan3D::setObjectType1(object3D::objType* objectType)
  {
    if(!_objectType_1) _objectType_1     = new object3D::objType[_size];
    memcpy(_objectType_1, objectType, _size*sizeof(object3D::objType));
  }
  object3D::objType* scan3D::getObjectType1()
  {
    return _objectType_1;
  }

  void scan3D::setCoords2(double* coords)
  {
    memcpy(_coords_2, coords, 3*_size*sizeof(double));
  }
  double* scan3D::getCoords2()
  {
    return _coords_2;
  }

  void scan3D::setNormals2(double* normals)
  {
    if(!_normals_2) _normals_2     = new double[3*_size];
    memcpy(_normals_2, normals, 3*_size*sizeof(double));
  }
  double* scan3D::getNormals2()
  {
    return _normals_2;
  }

  void scan3D::setMask2(bool* mask)
  {
    if(!_mask_2) _mask_2     = new bool[_size];
    memcpy(_mask_2, mask, _size*sizeof(bool));
  }
  bool* scan3D::getMask2()
  {
    return _mask_2;
  }

  void scan3D::setIntensity2(unsigned int* intens)
  {
    memcpy(_intensity_2, intens, _size*sizeof(unsigned int));
  }
  unsigned int* scan3D::getIntensity2()
  {
    return _intensity_2;
  }

  void scan3D::setDist2(double* dist)
  {
    if(!_dist_2) _dist_2     = new double[_size];
    memcpy(_dist_2, dist, _size*sizeof(double));
  }
  double* scan3D::getDist2()
  {
    return _dist_2;
  }

  void scan3D::setPhi2(double* phi)
  {
    if(!_phi_2) _phi_2     = new double[_size];
    memcpy(_phi_2, phi, _size*sizeof(double));
  }
  double* scan3D::getPhi2()
  {
    return _phi_2;
  }

  void scan3D::setObjectType2(object3D::objType* objectType)
  {
    if(!_objectType_2) _objectType_2     = new object3D::objType[_size];
    memcpy(_objectType_2, objectType, _size*sizeof(object3D::objType));
  }
  object3D::objType* scan3D::getObjectType2()
  {
    return _objectType_2;
  }

  void scan3D::setSeqenz(unsigned int seq)
  {
    _seq = seq;
  }
  unsigned int scan3D::getSeqenz()
  {
    return _seq;
  }

  void scan3D::setTimeStamp(unsigned long time)
  {
    _time_stamp = time;
  }
  unsigned long scan3D::getTimeStamp()
  {
    return _time_stamp;
  }

  void scan3D::setFrame_id(std::string frame_id)
  {
    _frame_id = frame_id;
  }
  std::string scan3D::getFrame_id()
  {
    return _frame_id;
  }

  void scan3D::setFragment_id(unsigned int fragment_id)
  {
    _fragmentid = fragment_id;
  }
  unsigned int scan3D::getFragment_id()
  {
    return _fragmentid;
  }

  void scan3D::setFragment_size(unsigned int fragment_size)
  {
    _fragmet_size = fragment_size;
  }
  unsigned int scan3D::getFragment_size()
  {
    return _fragmet_size;
  }

  void scan3D::setSize(unsigned int size)
  {
    _size = size;
  }
  unsigned int scan3D::getSize()
  {
    return _size;
  }

  void scan3D::setPubResults(bool pubResults)
  {
    _pub_Results = pubResults;
  }
  bool scan3D::getPubResults()
  {
    return _pub_Results;
  }

  /*
   * setTransformation
   * @param[in] T: Transformation matrix (4x4)
   */
  void scan3D::setTransformation(obvious::Matrix T)
  {
    *_T = T;
  }

  /*
   * getTransformation
   * @param[out] T: Transformation matrix (4x4)
   */
  obvious::Matrix scan3D::getTransformation()
  {
    obvious::Matrix T = *_T;
    return T;
  }

  void scan3D::setStepsPhi(unsigned int steps)
  {
    _stepsPhi = steps;
  }
  unsigned int scan3D::getStepsPhi()
  {
    return _stepsPhi;
  }

  void scan3D::removePoint(int idx)
  {
    this->_coords_1[idx]      = NAN;
    this->_normals_1[idx]     = NAN;

    this->_mask_1[idx]        = false;
    this->_dist_1[idx]        = NAN;
    this->_phi_1[idx]         = NAN;
    this->_intensity_1[idx]   = NAN;

    this->_objectType_1[idx]  = object3D::nanPoint;

    this->_coords_2[idx]      = NAN;
    this->_normals_2[idx]     = NAN;

    this->_mask_2[idx]        = false;
    this->_dist_2[idx]        = NAN;
    this->_phi_2[idx]         = NAN;
    this->_intensity_2[idx]   = NAN;

    this->_objectType_2[idx]  = object3D::nanPoint;
  }

  void scan3D::distanceThresFilter(double min_dist, double max_dist)
  {
    //cout << "Distance Filter" << endl;
    /*
     * prefilter data (erase all points, which are to close or to far away from the scanner)
     */

    //printCoords2File(this->getCoords1(), _size, "/home/rainer/workspace/p_t1.txt");

    //double tmp_max = 0;
    for(unsigned int i=0; i<_size; i++)
    {
//      if(tmp_max < this->_dist_1[i])
//        tmp_max = this->_dist_1[i];

      if(((this->_dist_1[i] < min_dist) or (this->_dist_1[i] > max_dist)) or
          (abs(this->_coords_1[3*i]) > 1.5*max_dist) or
          (abs(this->_coords_1[3*i+1]) > 1.5*max_dist) or
          (abs(this->_coords_1[3*i+2]) > 1.5*max_dist) or
          (abs(this->_coords_2[3*i]) > 1.5*max_dist) or
          (abs(this->_coords_2[3*i+1]) > 1.5*max_dist) or
          (abs(this->_coords_2[3*i+2]) > 1.5*max_dist))
      {
        //cout << i << "/";
        this->_dist_1[i]          = NAN;
        this->_coords_1[3*i]      = NAN;
        this->_coords_1[3*i+1]    = NAN;
        this->_coords_1[3*i+2]    = NAN;
        this->_normals_1[3*i]     = NAN;
        this->_normals_1[3*i+1]   = NAN;
        this->_normals_1[3*i+2]   = NAN;
        this->_intensity_1[i]     = NAN;
        this->_phi_1[i]           = NAN;

        this->_objectType_1[i]    = object3D::nanPoint;
        this->_mask_1[i]          = 0;

        this->_dist_2[i]          = NAN;
        this->_coords_2[3*i]      = NAN;
        this->_coords_2[3*i+1]    = NAN;
        this->_coords_2[3*i+2]    = NAN;
        this->_normals_2[3*i]     = NAN;
        this->_normals_2[3*i+1]   = NAN;
        this->_normals_2[3*i+2]   = NAN;
        this->_intensity_2[i]     = NAN;
        this->_phi_2[i]           = NAN;

        this->_objectType_2[i]    = object3D::nanPoint;
        this->_mask_2[i]          = 0;
      }
      if((this->_dist_2[i] < min_dist) or (this->_dist_2[i] > max_dist))
      {
        this->_dist_2[i]          = NAN;
        this->_coords_2[3*i]      = NAN;
        this->_coords_2[3*i+1]    = NAN;
        this->_coords_2[3*i+2]    = NAN;
        this->_normals_2[3*i]     = NAN;
        this->_normals_2[3*i+1]   = NAN;
        this->_normals_2[3*i+2]   = NAN;
        this->_intensity_2[i]     = NAN;
        this->_phi_2[i]           = NAN;

        this->_objectType_2[i]    = object3D::nanPoint;
        this->_mask_2[i]          = 0;
      }

    }

    //printCoords2File(this->getCoords1(), _size, "/home/rainer/workspace/p_t2.txt");

    //cout << "Max Value: " << tmp_max << endl;
    //cout <<  this->_dist_1[5] << endl;
    //cout << "End Distance Filter" << endl;
  }

  void scan3D::boxFilter(double box_min_x, double box_max_x, double box_min_y, double box_max_y, double box_min_z, double box_max_z)
  {
//    cout << "Box Filter" << endl;
//    cout << box_min_x << "/" << box_max_x << "/" << box_min_y << "/" << box_max_y << "/" << box_min_z << "/" << box_max_z << endl;


    //TOTEST
    double* tmp_coords_box = new double[3*_size];
    //printCoords2File(this->_coords_1, _size, "/home/rainer/workspace/coords_box.txt");

    /*
     * prefilter data (erase all points, which are located inside the box)
     */
    for(unsigned int i=0; i<_size; i++)
    {
      tmp_coords_box[3*i]   = NAN;
      tmp_coords_box[3*i+1] = NAN;
      tmp_coords_box[3*i+2] = NAN;

     if((this->_coords_1[3*i] >= box_min_x and this->_coords_1[3*i] <= box_max_x) and (this->_coords_1[3*i+1] >= box_min_y and this->_coords_1[3*i+1] <= box_max_y) and (this->_coords_1[3*i+2] >= box_min_z and this->_coords_1[3*i+2] <= box_max_z))
     {
       //cout << _coords_1[3*i] <<  "/" << _coords_1[3*i+1] <<  "/" << _coords_1[3*i+2] <<  " - WEG" << endl;
       tmp_coords_box[3*i]   = _coords_1[3*i];
       tmp_coords_box[3*i+1] = _coords_1[3*i+1];
       tmp_coords_box[3*i+2] = _coords_1[3*i+2];

       this->_dist_1[i]          = NAN;
       this->_coords_1[3*i]      = NAN;
       this->_coords_1[3*i+1]    = NAN;
       this->_coords_1[3*i+2]    = NAN;
       this->_normals_1[3*i]     = NAN;
       this->_normals_1[3*i+1]   = NAN;
       this->_normals_1[3*i+2]   = NAN;
       this->_intensity_1[i]     = NAN;
       this->_phi_1[i]           = NAN;

       this->_dist_2[i]          = NAN;
       this->_coords_2[3*i]      = NAN;
       this->_coords_2[3*i+1]    = NAN;
       this->_coords_2[3*i+2]    = NAN;
       this->_normals_2[3*i]     = NAN;
       this->_normals_2[3*i+1]   = NAN;
       this->_normals_2[3*i+2]   = NAN;
       this->_intensity_2[i]     = NAN;
       this->_phi_2[i]           = NAN;

       this->_objectType_1[i]    = object3D::nanPoint;
       this->_objectType_2[i]    = object3D::nanPoint;

       this->_mask_1[i]          = 0;
       this->_mask_2[i]          = 0;
     }
    }

    //TOTEST
    //printCoords2File(tmp_coords_box, _size, "/home/rainer/workspace/coords_box_in.txt");

    //cout << "End Box Filter" << endl;
  }

//  void scan3D::validPointFilter()
//  {
//    //cout << "Distance Filter" << endl;
//    /*
//     * prefilter data (erase all points, which are to close or to far away from the scanner)
//     */
//    for(unsigned int i=0; i<_size; i++)
//    {
//      if(isnan(_coords_1[3*i]) or isnan(_coords_1[3*i+1]) or isnan(_coords_1[3*i+2]) or isnan(_coords_2[3*i]) or isnan(_coords_2[3*i+1]) or isnan(_coords_2[3*i+2]))
//      {
//        //cout << "erase " << i << endl;
//        this->_dist_1[i]          = NAN;
//        this->_coords_1[3*i]      = NAN;
//        this->_coords_1[3*i+1]    = NAN;
//        this->_coords_1[3*i+2]    = NAN;
//        this->_normals_1[3*i]     = NAN;
//        this->_normals_1[3*i+1]   = NAN;
//        this->_normals_1[3*i+2]   = NAN;
//        this->_intensity_1[i]     = NAN;
//        this->_phi_1[i]           = NAN;
//
//        this->_dist_2[i]          = NAN;
//        this->_coords_2[3*i]      = NAN;
//        this->_coords_2[3*i+1]    = NAN;
//        this->_coords_2[3*i+2]    = NAN;
//        this->_normals_2[3*i]     = NAN;
//        this->_normals_2[3*i+1]   = NAN;
//        this->_normals_2[3*i+2]   = NAN;
//        this->_intensity_2[i]     = NAN;
//        this->_phi_2[i]           = NAN;
//
//        this->_objectType_1[i]    = object3D::nanPoint;
//        this->_objectType_2[i]    = object3D::nanPoint;
//
//        this->_mask_1[i]          = 0;
//        this->_mask_2[i]          = 0;
//      }
//    }
//    //cout <<  this->_dist_1[5] << endl;
//    //cout << "End Distance Filter" << endl;
//  }

  void scan3D::outlierFilterKMean(int kMean, double stdDeviation, object3D::objType objectType)
  {
    /*
     * pcl-version
     * http://pointclouds.org/documentation/tutorials/statistical_outlier.php
     */
    cout << "Outlier Filter: " << kMean << "/" << stdDeviation << endl;
//      // To write in file
//      char filename1[64] = "/home/rainer/workspace/outfilter_1.txt";
//      file_1.open(filename1);
//      char filename2[64] = "/home/rainer/workspace/outfilter_2.txt";
//      file_2.open(filename2);
//      char filename3[64] = "/home/rainer/workspace/outfilter_3.txt";
//      file_3.open(filename3);


      object3D::objType* tmp_objectType = this->getObjectType1();
      unsigned int* tmp_pointLocation_1 = new unsigned int[this->getSize()];

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

      /*
       * check amount of points assigned to the object
       */
      int object_size = 0;
      for(unsigned int i = 0; i < this->getSize(); ++i)
      {
        if(tmp_objectType[i] == objectType)
          object_size++;
      }

      cloud->points.resize(object_size);

      /*
       * fill cloud
       */
      int m = 0;
      for(int i = 0; i < this->getSize(); i++)
      {
        if((tmp_objectType[i] == objectType))
        {
          cloud->points[m].x = this->_coords_1[3*i];
          cloud->points[m].y = this->_coords_1[3*i+1];
          cloud->points[m].z = this->_coords_1[3*i+2];

          tmp_pointLocation_1[m] = i;

          // To write in file
          //file_1 << cloud->points[m].x << " " << cloud->points[m].y << " " << cloud->points[m].z << " " << endl;

          m++;
        }

      }
      //cout << "cloud filled" << endl;
      // Create the filtering object
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (cloud);
      sor.setMeanK (kMean);
      sor.setStddevMulThresh (stdDeviation);
      sor.filter (*cloud_filtered);

      //cout << "Size filtered: " << cloud_filtered->points.size() << "/" << object_size << endl;

//      for(int i = 0; i < cloud_filtered->points.size(); i++)
//      {
//        // To write in file
//        file_2 << cloud_filtered->points[i].x << " " << cloud_filtered->points[i].y << " " << cloud_filtered->points[i].z << " " << endl;
//      }

      sor.setNegative (true);
      sor.filter (*cloud_filtered);

      //cout << "Size error: " << cloud_filtered->points.size() << "/" << object_size << endl;

//      for(int i = 0; i < cloud_filtered->points.size(); i++)
//      {
//        // To write in file
//        file_3 << cloud_filtered->points[i].x << " " << cloud_filtered->points[i].y << " " << cloud_filtered->points[i].z << " " << endl;
//      }

//        // To write in file
//        file_1.close();
//        file_2.close();
//        file_3.close();

  }

  void scan3D::outlierFilterRadius(int neightbors, double radius, object3D::objType objectType)
  {
    /*
     * pcl-version
     * http://pointclouds.org/documentation/tutorials/statistical_outlier.php
     * http://docs.pointclouds.org/1.7.0/classpcl_1_1_radius_outlier_removal.html
     */

//      cout << "Outlier Filter: " << radius << "/" << neightbors << endl;
//      // To write in file
//      ofstream file_1;
//      ofstream file_2;
//      ofstream file_3;
//      char filename1[64] = "/home/rainer/workspace/outfilter_1.txt";
//      file_1.open(filename1);
//      char filename2[64] = "/home/rainer/workspace/outfilter_2.txt";
//      file_2.open(filename2);
//      char filename3[64] = "/home/rainer/workspace/outfilter_3.txt";
//      file_3.open(filename3);


      object3D::objType* tmp_objectType = this->getObjectType1();
      unsigned int* tmp_pointLocation_1 = new unsigned int[this->getSize()];

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

      /*
       * check amount of points assigned to the object
       */
      int object_size = 0;
      for(unsigned int i = 0; i < this->getSize(); ++i)
      {
        if((tmp_objectType[i] == objectType) && !(isnan(this->_coords_1[3*i]) or isnan(this->_coords_1[3*i+1]) or isnan(this->_coords_1[3*i+2])))
          object_size++;
      }

      cloud->points.resize(object_size);

      /*
       * fill cloud to filter
       */
      int m = 0;
      for(int i = 0; i < this->getSize(); i++)
      {
        if((tmp_objectType[i] == objectType)  && !(isnan(this->_coords_1[3*i]) or isnan(this->_coords_1[3*i+1]) or isnan(this->_coords_1[3*i+2])))
        {
          cloud->points[m].x = this->_coords_1[3*i];
          cloud->points[m].y = this->_coords_1[3*i+1];
          cloud->points[m].z = this->_coords_1[3*i+2];

//          if((isnan(this->_coords_1[3*i]) or isnan(this->_coords_1[3*i+1]) or isnan(this->_coords_1[3*i+2])))
//            cout << "NAN" << endl;
          tmp_pointLocation_1[m] = i;

          // To write in file
          //file_1 << cloud->points[m].x << " " << cloud->points[m].y << " " << cloud->points[m].z << " " << endl;
          m++;
        }

      }
      /*
       * setup filter
       */
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem(true);
      pcl::PointIndicesPtr inliers (new pcl::PointIndices);

      outrem.setInputCloud(cloud);
      outrem.setRadiusSearch(radius);
      outrem.setMinNeighborsInRadius(neightbors);
      outrem.setNegative (false);

      /*
       * apply filter
       */
      outrem.filter(*cloud_filtered);
      outrem.getRemovedIndices(*inliers);

      /*
       * remove points of original cloud
       */
      for(unsigned int i = 0; i < inliers->indices.size(); ++i)
      {
        this->removePoint(tmp_pointLocation_1[inliers->indices[i]]);
//        // To write in file
//        file_3 << cloud->points[inliers->indices[i]].x << " " << cloud->points[inliers->indices[i]].y << " " << cloud->points[inliers->indices[i]].z << " " << endl;
      }


//      //To write in File
//      double* test = this->getCoords1();
//      for(int i = 0; i < this->getSize(); i++)
//      {
//        if((!isnan(test[3*i])) && (!isnan(test[3*i+1])) && (!isnan(test[3*i+2])))
//          if((tmp_objectType[i] == objectType))
//            file_1 << test[3*i] << " " << test[3*i+1] << " " << test[3*i+2] << " " << endl;
//
//      }
//      cout << "Size filtered: " << cloud_filtered->points.size() << "/" << object_size << endl;
//      for(int i = 0; i < cloud_filtered->points.size(); i++)
//      {
//        // To write in file
//       // file_1 << cloud_filtered->points[i].x << " " << cloud_filtered->points[i].y << " " << cloud_filtered->points[i].z << " " << endl;
//        file_2 << cloud_filtered->points[i].x << " " << cloud_filtered->points[i].y << " " << cloud_filtered->points[i].z << " " << endl;
//
//      }
//      //TOSHOW
//      cout << "Size error: " << inliers->indices.size() << "/" << object_size << endl;
//      for(int i = 0; i < inliers->indices.size(); i++)
//      {
//        // To write in file
//       // file_1 << cloud->points[inliers->indices[i]].x << " " << cloud->points[inliers->indices[i]].y << " " << cloud->points[inliers->indices[i]].z << " " << endl;
//        file_3 << cloud->points[inliers->indices[i]].x << " " << cloud->points[inliers->indices[i]].y << " " << cloud->points[inliers->indices[i]].z << " " << endl;
//      }
//      // To write in file
//      cout << "test"<< endl;
//      file_1.close();
//      file_2.close();
//      file_3.close();
  }

  unsigned int scan3D::identifyReflections(double thres_substract_distance)
  {
    object3D::objType* objectType1  = new object3D::objType[this->_size];
    object3D::objType* objectType2   = new object3D::objType[this->_size];

    unsigned int reflectionFound       = 0;

//    cout << "begin identifyReflections " << thres_substract_distance << endl;
    //TOTEST:
//    for(int i=0; i <= 10; i++)
//    {
//      cout << _dist_2[i] << " - " << _dist_1[i] << " = " << (_dist_2[i] - _dist_1[i]) << " / " << abs(this->_dist_2[i]-this->_dist_1[i]) << endl;
//    }
    /*
     * check each point, if dist_1 and dist_2 are different
     */
//    int tmp_i_mask = 0;
//    int tmp_i_val = 0;
//    int tmp_i_refl = 0;
//    int tmp_i_noExch = 0;
//    int tmp_i_Exch = 0;
//    int tmp_no = 0;

//    // To write in file
//    ofstream file_1;
//    ofstream file_2;
//    ofstream file_3;
//    ofstream file_4;
//    ofstream file_5;
//    ofstream file_6;
//    char filename1[64] = "/home/rainer/workspace/identify_1.txt";
//    file_1.open(filename1);
//    char filename2[64] = "/home/rainer/workspace/identify_2.txt";
//    file_2.open(filename2);
//    char filename3[64] = "/home/rainer/workspace/identify_3.txt";
//    file_3.open(filename3);
//    char filename4[64] = "/home/rainer/workspace/identify_4.txt";
//    file_4.open(filename4);
//    char filename5[64] = "/home/rainer/workspace/identify_5.txt";
//    file_5.open(filename5);
//    char filename6[64] = "/home/rainer/workspace/identify_6.txt";
//    file_6.open(filename6);

    for(int i=0; i<this->_size; i++)
    {
      if((this->_mask_1[i]) and !isnan(this->_coords_1[3*i]) and !isnan(this->_coords_1[3*i+1]) and !isnan(this->_coords_1[3*i+2]))
      {
        //tmp_i_mask++;
        //cout << abs(_dist_1[i] - _dist_2[i]) << "/";
        if((thres_substract_distance >= abs(this->_dist_2[i] - this->_dist_1[i])) or isnan(abs(this->_dist_2[i] - this->_dist_1[i])))//or (this->_dist_2[i] == 0))
        {
          objectType1[i] = object3D::validPoint;         // valid scan point
          objectType2[i] = object3D::validPoint;

//          // To write in File
//          file_1 << this->_coords_1[3*i] << " " << this->_coords_1[3*i+1] << " " << this->_coords_1[3*i+2] << " " << endl;
//          file_2 << this->_coords_2[3*i] << " " << this->_coords_2[3*i+1] << " " << this->_coords_2[3*i+2] << " " << endl;
//          tmp_i_val++;
        }
        else  // reflective influenced
        {
//          tmp_i_refl++;
//          file_1 << this->_coords_1[3*i] << " " << this->_coords_1[3*i+1] << " " << this->_coords_1[3*i+2] << " " << endl;

          //cout << thres_substract_distance << "-" << _dist_1[i] << "-" << _dist_2[i] << "-" << abs(_dist_2[i] - _dist_1[i]) << " / ";
          if(this->_dist_1[i] < this->_dist_2[i])  // echo 1 in front of echo 2
          {
            objectType1[i] = object3D::errorSurface;        // point on the object
            objectType2[i] = object3D::behindErrorS;        // point influenced by the object
//            // To write in File
//            file_3 << this->_coords_1[3*i] << " " << this->_coords_1[3*i+1] << " " << this->_coords_1[3*i+2] << " " << endl;
//            file_4 << this->_coords_2[3*i] << " " << this->_coords_2[3*i+1] << " " << this->_coords_2[3*i+2] << " " << endl;

//            tmp_i_noExch++;

          }
          else // echo 2 in front of echo 1 => have to exchange echo 1 and echo 2
          {
//            tmp_i_Exch++;
            /*
             * exchange point, so that the errorSurface is stored in objectType1 and the error in objectType2
             */
            double tmp = 0;
            // coords
            tmp = _coords_1[3*i];
            _coords_1[3*i] = _coords_2[3*i];
            _coords_2[3*i] = tmp;
            tmp = _coords_1[3*i+1];
            _coords_1[3*i+1] = _coords_2[3*i+1];
            _coords_2[3*i+1] = tmp;
            tmp = _coords_1[3*i+2];
            _coords_1[3*i+2] = _coords_2[3*i+2];
            _coords_2[3*i+2] = tmp;

            // normals
            tmp = _normals_1[3*i];
            _normals_1[3*i] = _normals_2[3*i];
            _normals_2[3*i] = tmp;
            tmp = _normals_1[3*i+1];
            _normals_1[3*i+1] = _normals_2[3*i+1];
            _normals_2[3*i+1] = tmp;
            tmp = _normals_1[3*i+2];
            _normals_1[3*i+2] = _normals_2[3*i+2];
            _normals_2[3*i+2] = tmp;

            // mask
            tmp = _mask_1[i];
            _mask_1[i] = _mask_2[i];
            _mask_2[i] = tmp;

            // dist
            tmp = _dist_1[i];
            _dist_1[i] = _dist_2[i];
            _dist_2[i] = tmp;

            // phi
            tmp = _phi_1[i];
            _phi_1[i] = _phi_2[i];
            _phi_2[i] = tmp;

            // intensity
            tmp = _intensity_1[i];
            _intensity_1[i] = _intensity_2[i];
            _intensity_2[i] = tmp;

            objectType1[i] = object3D::errorSurface;              // point influenced by the object
            objectType2[i] = object3D::behindErrorS;                // point on the object
//            // To write in File
//            file_5 << this->_coords_1[3*i] << " " << this->_coords_1[3*i+1] << " " << this->_coords_1[3*i+2] << " " << endl;
//            file_6 << this->_coords_2[3*i] << " " << this->_coords_2[3*i+1] << " " << this->_coords_2[3*i+2] << " " << endl;

          }
          reflectionFound++;
        }
      }
      else
      {
        this->_mask_1[i] = false;
        objectType1[i] = object3D::nanPoint;
        objectType2[i] = object3D::nanPoint;
//        tmp_no++;
      }
    }

//    // To write in file
//    file_1.close();
//    file_2.close();
//    file_3.close();
//    file_4.close();
//    file_5.close();
//    file_6.close();

//    cout << "size/mask true/no: " <<  this->_size << " / " << tmp_i_mask << " / " << tmp_no << endl;
//    cout << "mask true/valid/reflected: "  <<  tmp_i_mask << " / " <<   tmp_i_val << " / "<<  tmp_i_refl <<  endl;
//    cout << "reflected/noExchange/Exchange/: "  <<  tmp_i_refl << " / "<<  tmp_i_noExch << " / "<<  tmp_i_Exch << endl;


    /*
     * set new objectTypes
     */
    this->setObjectType1(objectType1);
    this->setObjectType2(objectType2);
   // cout << "end identifyReflections - points found:  " << reflectionFound << endl;

    return reflectionFound;
  }

  void scan3D::indentifyObjects(std::vector<mirrordetector::object3D*>& object, double planeDetection_threshold, unsigned int planeDetectin_minPoints)
  {

//    /*
//     * clustering
//     */
//    // k-mean clustering
//    // minimum spacing tree
//    // PCL: planar_segmentation
//    // http://pointclouds.org/documentation/tutorials/planar_segmentation.php
//    // => see Miniprogramms
//    //
//    // Hough Transformation
//    //
//    // UNI Wü
//    //http://slam6d.sourceforge.net/doc/shapes.html
//    //
//    // VTK
//    // https://github.com/daviddoria/VTKHoughPlanes/blob/master/src/HoughPlanes.cpp

   // std::vector<mirrordetector::object3D*> newObjects;
    unsigned int amountObjects  = 0;

    //std::vector<double*> newCorners(12);
    double* newCorners    = new double[12];
    std::vector<double*> centroid(1);
    double* tmp_centroid  = new double[3];


    double* newCoeff    = new double[4];

    object3D::objType* tmp_objectType  = this->getObjectType1();
    bool* tmp_belonging                = new bool[this->getSize()];
    unsigned int fragment_id           = this->getFragment_id();

    bool exit = false;

    for(unsigned int i=0; i < this->getSize(); i++)
    {
      tmp_belonging[i] = false;
    }

    /*
     * point cloud version
     */
    // check amount of points assigned to the object
    int object_size = 0;
    for(unsigned int i = 0; i < this->getSize(); ++i)
    {
      if(tmp_objectType[i] == object3D::errorSurface)
        object_size++;
    }

    int new_inliers = 0;
    int new_size = object_size;


    /*
     * copy into cloud for clustering
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    int* tmp_pLocation_1    = NULL;
    int cloudIn_size = 0;

    /*
     * get size for cloud
     */
    for(unsigned int i = 0; i < this->getSize(); ++i)
    {
      if((tmp_objectType[i] == object3D::errorSurface) &&
          !isnan(this->_coords_1[3*i]) && !isnan(this->_coords_1[3*i+1])  && !isnan(this->_coords_1[3*i+2])  &&
          !isinf(this->_coords_1[3*i]) && !isinf(this->_coords_1[3*i+1])  && !isinf(this->_coords_1[3*i+2]))
      {
        cloudIn_size++;
      }
    }
    //cout << "cloud for cluster size" << cloudIn_size<< endl;

    tmp_pLocation_1         = new int[cloudIn_size];
    cloudIn->points.resize(cloudIn_size);

    /*
     * copy all points together which are not assigned to an object yet.
     */
    int n = 0;
    for(unsigned int i = 0; i < this->getSize(); ++i)
    {
      if((tmp_objectType[i] == object3D::errorSurface) &&
          !isnan(this->_coords_1[3*i]) && !isnan(this->_coords_1[3*i+1])  && !isnan(this->_coords_1[3*i+2])  &&
          !isinf(this->_coords_1[3*i]) && !isinf(this->_coords_1[3*i+1])  && !isinf(this->_coords_1[3*i+2]))
      {
        cloudIn->points[n].x    = this->_coords_1[3*i];
        cloudIn->points[n].y    = this->_coords_1[3*i+1];
        cloudIn->points[n].z    = this->_coords_1[3*i+2];
//        tmp_normals_1[3*m]    = this->_normals_1[3*i];
//        tmp_normals_1[3*m+1]  = this->_normals_1[3*i+1];
//        tmp_normals_1[3*m+2]  = this->_normals_1[3*i+2];
//
//        tmp_dist_1[m]         = this->_dist_1[i];
//        tmp_phi_1[m]          = this->_phi_1[i];
//        tmp_intens_1[m]       = this->_intensity_1[i];
        tmp_pLocation_1[n]= i;          // saves the index where the point was located in the original scan, the index of tmp_pointLocation_1 is refering to the pcl-cloud
        n++;
      }
    }

//    // To write in file
//    double* test_input    = new double[3*cloudIn_size];
//    for(unsigned int i = 0; i < cloudIn_size; ++i)
//    {
//      test_input[3*i]   = cloudIn->points[i].x;
//      test_input[3*i+1] = cloudIn->points[i].y;
//      test_input[3*i+2] = cloudIn->points[i].z;
//    }
//    printCoords2File(test_input, cloudIn_size, "/home/rainer/workspace/input.txt");

/*
 * cluster growing
 * http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php
 */
//    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
//    pcl::PointCloud <pcl::Normal>::Ptr normals_cluster (new pcl::PointCloud <pcl::Normal>);
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
//    normal_estimator.setSearchMethod (tree);
//    normal_estimator.setInputCloud (cloud_cluster);
//    normal_estimator.setKSearch (50);
//    normal_estimator.compute (*normals_cluster);
//    cout << "1" << endl;
//    pcl::IndicesPtr indices_cluster (new std::vector <int>);
//    pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setInputCloud (cloud_cluster);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (0.0, 1.0);
//    pass.filter (*indices_cluster);
//    cout << "2" << endl;
//
//    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
//    reg.setMinClusterSize (50);
//    reg.setMaxClusterSize (1000000);
//    reg.setSearchMethod (tree);
//    reg.setNumberOfNeighbours (30);
//    reg.setInputCloud (cloud_cluster);
//    //reg.setIndices (indices);
//    reg.setInputNormals (normals_cluster);
//    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
//    reg.setCurvatureThreshold (1.0);
//    cout << "3" << endl;
//
//    std::vector <pcl::PointIndices> clusters;
//    reg.extract (clusters);
//
//    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
//    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
//    std::cout << "These are the indices of the points of the initial" <<
//      std::endl << "cloud that belong to the first cluster:" << std::endl;
//    int counter = 0;
//    while (counter < clusters[0].indices.size ())
//    {
//      std::cout << clusters[0].indices[counter] << ", ";
//      counter++;
//      if (counter % 10 == 0)
//        std::cout << std::endl;
//    }

/*
 * cluster points
 * http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
 */
    unsigned int new_inliers_plane = 0;
    int plane = 0;
    // Create the filtering object: downsample the dataset using a leaf size of planeDetection_threshold/5.0
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloudIn);

    //TODO: change this size autmatically based on the opening angle and rotation speed of the scanner
    vg.setLeafSize (planeDetection_threshold/10.0, planeDetection_threshold/10.0, planeDetection_threshold/10.0);
    vg.filter (*cloud_filtered);
   // std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*


    // To write in file
    double* test_input1    = new double[3*cloud_filtered->points.size()];
    for(unsigned int i = 0; i < cloud_filtered->points.size(); ++i)
    {
      test_input1[3*i]   = cloud_filtered->points[i].x;
      test_input1[3*i+1] = cloud_filtered->points[i].y;
      test_input1[3*i+2] = cloud_filtered->points[i].z;
    }
    //printCoords2File(test_input1, cloud_filtered->points.size(), "/home/rainer/workspace/input1.txt");


    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (planeDetection_threshold);
    ec.setMinClusterSize (planeDetectin_minPoints);
    ec.setMaxClusterSize (cloudIn_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract(cluster_indices);
    int j = 0;
    //cout << "found clusters: " << cluster_indices.size() << endl;

    /*
     * go through clusters and check for plane values
     */
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      /*
       * copy data of cluster into cloud_cluster2 cloud
       */
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster2 (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        cloud_cluster2->points.push_back (cloud_filtered->points[*pit]);
      }
      cloud_cluster2->width = cloud_cluster2->points.size ();
      cloud_cluster2->height = 1;
      cloud_cluster2->is_dense = true;

//      // To write in file
//      unsigned int new_inliers_cluster = (unsigned int) cloud_cluster2->points.size();
//      if(j == 0)
//      {
//       cout << "cluster 0: " << new_inliers_cluster << endl;
//       double* test_obj0     = new double[3*new_inliers_cluster];
//       for(unsigned int i = 0; i < new_inliers_cluster; ++i)
//       {
//         test_obj0[3*i]     = cloud_cluster2->points[i].x;
//         test_obj0[3*i+1]   = cloud_cluster2->points[i].y;
//         test_obj0[3*i+2]   = cloud_cluster2->points[i].z;
//       }
//       printCoords2File(test_obj0, new_inliers_cluster, "/home/rainer/workspace/object0.txt");
//      }
//      if(j == 1)
//      {
//       cout << "cluster 1: " << new_inliers_cluster << endl;
//
//       double* test_obj1     = new double[3*new_inliers_cluster];
//       for(unsigned int i = 0; i < new_inliers_cluster; ++i)
//       {
//         test_obj1[3*i]     = cloud_cluster2->points[i].x;
//         test_obj1[3*i+1]   = cloud_cluster2->points[i].y;
//         test_obj1[3*i+2]   = cloud_cluster2->points[i].z;
//       }
//       printCoords2File(test_obj1, new_inliers_cluster, "/home/rainer/workspace/object1.txt");
//
//      }
//      if(j == 2)
//      {
//       cout << "cluster 2: " << new_inliers_cluster << endl;
//
//       double* test_obj2     = new double[3*new_inliers_cluster];
//       for(unsigned int i = 0; i < new_inliers_cluster; ++i)
//       {
//         test_obj2[3*i]     = cloud_cluster2->points[i].x;
//         test_obj2[3*i+1]   = cloud_cluster2->points[i].y;
//         test_obj2[3*i+2]   = cloud_cluster2->points[i].z;
//       }
//       printCoords2File(test_obj2, new_inliers_cluster, "/home/rainer/workspace/object2.txt");
//      }
//      if(j == 3)
//      {
//       cout << "cluster 3: " << new_inliers_cluster << endl;
//
//       double* test_obj3     = new double[3*new_inliers_cluster];
//       for(unsigned int i = 0; i < new_inliers_cluster; ++i)
//       {
//         test_obj3[3*i]     = cloud_cluster2->points[i].x;
//         test_obj3[3*i+1]   = cloud_cluster2->points[i].y;
//         test_obj3[3*i+2]   = cloud_cluster2->points[i].z;
//       }
//       printCoords2File(test_obj3, new_inliers_cluster, "/home/rainer/workspace/object3.txt");
//      }


      /*
       * check for plane values
       */
      // Create the segmentation object for the planar model and set all the parameters
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      // TODO: set them to launch file variables
      seg.setMaxIterations (100);
      seg.setDistanceThreshold (0.1);

      int i=0, nr_points = (int) cloud_cluster2->points.size ();

      // Segment the planar component from the remaining cloud
      seg.setInputCloud (cloud_cluster2);
      seg.segment (*inliers_plane, *coefficients_plane);
      if (inliers_plane->indices.size () == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cloud_cluster2);
      extract.setIndices (inliers_plane);
      extract.setNegative (false);


      /*
       * assign points associated with the planar surface
       */
      extract.filter (*cloud_plane);
      new_inliers_plane = (unsigned int) cloud_plane->points.size ();

      double* tmp_coords        = new double[3*new_inliers_plane];

      bool thisIsANewObject     = true;

      for(unsigned int i = 0; i < new_inliers_plane; ++i)
      {
        tmp_coords[3*i]     = cloud_plane->points[i].x;
        tmp_coords[3*i+1]   = cloud_plane->points[i].y;
        tmp_coords[3*i+2]   = cloud_plane->points[i].z;
      }

//// To write in file
//      if(j == 0)
//      {
//       cout << "plane 0: " << new_inliers_plane << endl;
//       double* test_obj0     = new double[3*new_inliers_plane];
//       for(unsigned int i = 0; i < new_inliers_plane; ++i)
//       {
//         test_obj0[3*i]     = cloud_plane->points[i].x;
//         test_obj0[3*i+1]   = cloud_plane->points[i].y;
//         test_obj0[3*i+2]   = cloud_plane->points[i].z;
//       }
//       printCoords2File(test_obj0, new_inliers_plane, "/home/rainer/workspace/plane0.txt");
//      }
//      if(j == 1)
//      {
//       cout << "plane 1: " << new_inliers_plane << endl;
//
//       double* test_obj1     = new double[3*new_inliers_plane];
//       for(unsigned int i = 0; i < new_inliers_plane; ++i)
//       {
//         test_obj1[3*i]     = cloud_plane->points[i].x;
//         test_obj1[3*i+1]   = cloud_plane->points[i].y;
//         test_obj1[3*i+2]   = cloud_plane->points[i].z;
//       }
//       printCoords2File(test_obj1, new_inliers_plane, "/home/rainer/workspace/plane1.txt");
//
//      }
//      if(j == 2)
//      {
//       cout << "plane 2: " << new_inliers_plane << endl;
//
//       double* test_obj2     = new double[3*new_inliers_plane];
//       for(unsigned int i = 0; i < new_inliers_plane; ++i)
//       {
//         test_obj2[3*i]     = cloud_plane->points[i].x;
//         test_obj2[3*i+1]   = cloud_plane->points[i].y;
//         test_obj2[3*i+2]   = cloud_plane->points[i].z;
//       }
//       printCoords2File(test_obj2, new_inliers_plane, "/home/rainer/workspace/plane2.txt");
//      }
//      if(j == 3)
//      {
//       cout << "plane 3: " << new_inliers_plane << endl;
//
//       double* test_obj3     = new double[3*new_inliers_plane];
//       for(unsigned int i = 0; i < new_inliers_plane; ++i)
//       {
//         test_obj3[3*i]     = cloud_plane->points[i].x;
//         test_obj3[3*i+1]   = cloud_plane->points[i].y;
//         test_obj3[3*i+2]   = cloud_plane->points[i].z;
//       }
//       printCoords2File(test_obj3, new_inliers_plane, "/home/rainer/workspace/plane3.txt");
//      }
//// END write in file


      /*
       * copy coefficients of plane
       */
      for(int m=0; m<4; m++)
      {
        newCoeff[m] = (double)coefficients_plane->values[m];
      }

//      cout << "coefficients plane " << endl;
//      cout << newCoeff[0] << "/" << newCoeff[1] << "/" << newCoeff[2] << "/" << newCoeff[3] << endl;

//        std::cerr << "Model coefficients " << amountObjects << " :"
//                                              << coefficients->values[0] << " "
//                                              << coefficients->values[1] << " "
//                                              << coefficients->values[2] << " "
//                                              << coefficients->values[3] << std::endl;
      double* tmp_dimensions = new double[2];
      double* tmp_centroid = new double[3];

      /*
       * identify object boarders
       */
      newCorners = this->indentifyCorners(newCoeff, centroid, tmp_coords, new_inliers_plane, tmp_dimensions);
      tmp_centroid = centroid[0];

//      cout << "newCoords plane " << endl;
//      cout << newCorners[0] << "/" << newCorners[1] << "/" << newCorners[2] << endl;
//      cout << newCorners[3] << "/" << newCorners[4] << "/" << newCorners[5] << endl;
//      cout << newCorners[6] << "/" << newCorners[7] << "/" << newCorners[8] << endl;
//      cout << newCorners[9] << "/" << newCorners[10] << "/" << newCorners[11] << endl;
//      cout << "new Centroid plane " << endl;
//      cout << tmp_centroid[0] << "/" << tmp_centroid[1] << "/" << tmp_centroid[2] << endl;


      object3D* tmpObject = new object3D(this->getSeqenz(), this->getTimeStamp(), newCorners, tmp_centroid);

      /*
       * set additional values for object
       */
      tmpObject->setCoefficents(newCoeff);
      tmpObject->setSize(new_inliers);
      tmpObject->setCoords(tmp_coords);
      tmpObject->setPubResults(_pub_Results);

      /*
       * TODO:
       */
//      tmpObject->setNormals(tmp_normals);
//      tmpObject->setDist(tmp_dist);
//      tmpObject->setPhi(tmp_phi);
//      tmpObject->setIntensity(tmp_intens);
      tmpObject->setMajorDim(tmp_dimensions[0]);
      tmpObject->setMinorDim(tmp_dimensions[1]);

      //cout << "Dimensions: " << tmp_dimensions[0] << " / " << tmp_dimensions[1] << endl;


      /*
       * check if object already exists (maybe not necessary since there are clustering)
       */
      for(int i=0; i < amountObjects; i++)
      {
        //TODO: threshold for same object identification as launch variable
        if(thisIsANewObject)
          thisIsANewObject = tmpObject->compareObject(object[i], 2*planeDetection_threshold);
      }

      if(thisIsANewObject)
      {
        /*
         * save object
         */
        object.resize(amountObjects+1);
        object[amountObjects] = tmpObject;

        //double* test = object[amountObjects]->getObjectCorners();
        amountObjects++;
      }

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_filtered = *cloud_f;
      plane++;

      delete [] tmp_dimensions;
      delete [] tmp_coords;

      //std::cout << "PointCloud representing the Cluster: " << cloud_cluster2->points.size () << " data points." << std::endl;
      j++;

    }

    delete [] newCorners;
    delete [] newCoeff;
  }

  void scan3D::indentifyObjects_old(std::vector<mirrordetector::object3D*>& object, double planeDetection_threshold, unsigned int planeDetectin_minPoints)
  {

  //    /*
  //     * clustering
  //     */
  //    // k-mean clustering
  //    // minimum spacing tree
  //    // PCL: planar_segmentation
  //    // http://pointclouds.org/documentation/tutorials/planar_segmentation.php
  //    // => see Miniprogramms
  //    //
  //    // Hough Transformation
  //    //
  //    // UNI Wü
  //    //http://slam6d.sourceforge.net/doc/shapes.html
  //    //
  //    // VTK
  //    // https://github.com/daviddoria/VTKHoughPlanes/blob/master/src/HoughPlanes.cpp

     // std::vector<mirrordetector::object3D*> newObjects;
      unsigned int amountObjects  = 0;

      //std::vector<double*> newCorners(12);
      double* newCorners    = new double[12];
      std::vector<double*> centroid(1);
      double* tmp_centroid  = new double[3];


      double* newCoeff    = new double[4];

      object3D::objType* tmp_objectType  = this->getObjectType1();
      bool* tmp_belonging                = new bool[this->getSize()];
      unsigned int fragment_id           = this->getFragment_id();

      bool exit = false;

      for(unsigned int i=0; i < this->getSize(); i++)
      {
        tmp_belonging[i] = false;
      }

      /*
       * point cloud version
       */
      // check amount of points assigned to the object
      int object_size = 0;
      for(unsigned int i = 0; i < this->getSize(); ++i)
      {
        if(tmp_objectType[i] == object3D::errorSurface)
          object_size++;
      }

      int new_inliers = 0;
      int new_size = object_size;

      /*
       * run till all object are found
       */
      while(!exit)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //      cout << "tmp belonging begin" << endl;
  //      for(unsigned int i = 0; i < this->getSize(); ++i)
  //      {
  //        if(tmp_belonging[i] == true)
  //          cout << i << "/";
  //      }

        new_size = new_size - new_inliers;
  //      cout << "size: " << object_size << "/" << new_size << "-" << new_inliers << " > " << planeDetectin_minPoints << endl;

        double* tmp_coords          = NULL;
        double* tmp_normals         = NULL;
        double* tmp_dist            = NULL;
        double* tmp_phi             = NULL;
        unsigned int* tmp_intens    = NULL;

        double* tmp_normals_1       = NULL;
        double* tmp_dist_1          = NULL;
        double* tmp_phi_1           = NULL;
        unsigned int* tmp_intens_1  = NULL;
        int* tmp_pointLocation_1    = NULL;

        cloud->points.resize(new_size);

        tmp_normals_1               = new double[3*new_size];
        tmp_dist_1                  = new double[new_size];
        tmp_phi_1                   = new double[new_size];
        tmp_intens_1                = new unsigned int[new_size];
        tmp_pointLocation_1         = new int[new_size];

        int m = 0;

        /*
         * copy all points together which are not assigned to an object yet.
         */
        for(unsigned int i = 0; i < this->getSize(); ++i)
        {
          if((tmp_objectType[i] == object3D::errorSurface) && (tmp_belonging[i] == false))
          {
            cloud->points[m].x    = this->_coords_1[3*i];
            cloud->points[m].y    = this->_coords_1[3*i+1];
            cloud->points[m].z    = this->_coords_1[3*i+2];

            tmp_normals_1[3*m]    = this->_normals_1[3*i];
            tmp_normals_1[3*m+1]  = this->_normals_1[3*i+1];
            tmp_normals_1[3*m+2]  = this->_normals_1[3*i+2];

            tmp_dist_1[m]         = this->_dist_1[i];
            tmp_phi_1[m]          = this->_phi_1[i];
            tmp_intens_1[m]       = this->_intensity_1[i];
            tmp_pointLocation_1[m]= i;          // saves the index where the point was located in the original scan, the index of tmp_pointLocation_1 is refering to the pcl-cloud
            m++;
          }
          else
          {
            tmp_belonging[i] = true;
          }

        }

//        // To write in file
//        double* test_input    = new double[3*m];
//        for(unsigned int i = 0; i < m; ++i)
//        {
//          test_input[3*i]   = cloud->points[i].x;
//          test_input[3*i+1] = cloud->points[i].y;
//          test_input[3*i+2] = cloud->points[i].z;
//        }
//        printCoords2File(test_input, m, "/home/rainer/workspace/input.txt");

        /*
         * Plane identification
         */
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        //TODO: Put as variables
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (planeDetection_threshold);
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        new_inliers = (unsigned int) inliers->indices.size();

  //      cout << "PCL modle points: " << inliers->indices.size() << " > " << planeDetectin_minPoints << "?" << endl;
        /*
         * plane found
         */
        if(new_inliers >= planeDetectin_minPoints)
        {
  //        //TOCHECK for only one identification:
  //        exit = 1;

          /*
           * assign points to its object
           */
          //TODO: copy points into tmp to store them with the object
          tmp_coords        = new double[3*new_inliers];
          tmp_normals       = new double[3*new_inliers];
          tmp_dist          = new double[new_inliers];
          tmp_phi           = new double[new_inliers];
          tmp_intens        = new unsigned int[new_inliers];

          bool thisIsANewObject     = true;
          bool thisIsAGroundObject  = false;

          for(unsigned int i = 0; i < new_inliers; ++i)
          {
            tmp_belonging[tmp_pointLocation_1[inliers->indices[i]]] = true;       // checks for the point in the pcl-cloud the original point in the scan and marks sets the belonging bit
            tmp_coords[3*i]     = cloud->points[inliers->indices[i]].x;
            tmp_coords[3*i+1]   = cloud->points[inliers->indices[i]].y;
            tmp_coords[3*i+2]   = cloud->points[inliers->indices[i]].z;

            tmp_normals[3*i]    = tmp_normals_1[3*inliers->indices[i]];
            tmp_normals[3*i+1]  = tmp_normals_1[3*inliers->indices[i]+1];
            tmp_normals[3*i+2]  = tmp_normals_1[3*inliers->indices[i]+2];

            tmp_dist[i]         = tmp_dist_1[inliers->indices[i]];
            tmp_phi[i]          = tmp_phi_1[inliers->indices[i]];
            tmp_intens[i]       = tmp_intens_1[inliers->indices[i]];
          }


          /*
           * identify object boarders
           */
          // (in ax + by + cz + d = 0 form).
          for(int j=0; j<4; j++)
          {
            newCoeff[j] = (double)coefficients->values[j];
          }

  //        std::cerr << "Model coefficients " << amountObjects << " :"
  //                                              << coefficients->values[0] << " "
  //                                              << coefficients->values[1] << " "
  //                                              << coefficients->values[2] << " "
  //                                              << coefficients->values[3] << std::endl;
          double* tmp_dimensions = new double[2];
          double* tmp_centroid = new double[3];

          newCorners = this->indentifyCorners(newCoeff, centroid, tmp_coords, new_inliers, tmp_dimensions);
          tmp_centroid = centroid[0];


  //        cout << "Centroid" << endl;
  //        cout << tmp_centroid[0] << " " << tmp_centroid[1] << " " <<  tmp_centroid[2] <<  endl;

  //        // To write in file
  //        if(amountObjects == 0)
  //        {
  //          double* test_obj0     = new double[3*new_inliers];
  //          for(unsigned int i = 0; i < new_inliers; ++i)
  //          {
  //            test_obj0[3*i]     = cloud->points[inliers->indices[i]].x;
  //            test_obj0[3*i+1]   = cloud->points[inliers->indices[i]].y;
  //            test_obj0[3*i+2]   = cloud->points[inliers->indices[i]].z;
  //          }
  //          printCoords2File(test_input, m, "/home/rainer/workspace/object0.txt");
  //          printCoords2File(newCorners, 4, "/home/rainer/workspace/corners0.txt");
  //        }
  //        if(amountObjects == 1)
  //        {
  //          double* test_obj1     = new double[3*new_inliers];
  //          for(unsigned int i = 0; i < new_inliers; ++i)
  //          {
  //            test_obj1[3*i]     = cloud->points[inliers->indices[i]].x;
  //            test_obj1[3*i+1]   = cloud->points[inliers->indices[i]].y;
  //            test_obj1[3*i+2]   = cloud->points[inliers->indices[i]].z;
  //          }
  //          printCoords2File(test_input, m, "/home/rainer/workspace/object1.txt");
  //          printCoords2File(newCorners, 4, "/home/rainer/workspace/corners1.txt");
  //
  //        }
  //        if(amountObjects == 2)
  //        {
  //          double* test_obj2     = new double[3*new_inliers];
  //          for(unsigned int i = 0; i < new_inliers; ++i)
  //          {
  //            test_obj2[3*i]     = cloud->points[inliers->indices[i]].x;
  //            test_obj2[3*i+1]   = cloud->points[inliers->indices[i]].y;
  //            test_obj2[3*i+2]   = cloud->points[inliers->indices[i]].z;
  //          }
  //          printCoords2File(test_input, m, "/home/rainer/workspace/object2.txt");
  //          printCoords2File(newCorners, 4, "/home/rainer/workspace/corners2.txt");
  //        }

  //        cout << "ob1" << endl;
  //        // TOTEST:
  //        cout << newCorners[0] << "/" << newCorners[1] << "/" << newCorners[2] << endl;
  //        cout << newCorners[3] << "/" << newCorners[4] << "/" << newCorners[5] << endl;
  //        cout << newCorners[6] << "/" << newCorners[7] << "/" << newCorners[8] << endl;
  //        cout << newCorners[9] << "/" << newCorners[10] << "/" << newCorners[11] << endl;
  //        double* test = new double[3];
  //        printCoords2File(newCorners, 4, "/home/rainer/workspace/corners.txt");
  //        test[0] = newCorners[0];
  //        test[1] = newCorners[1];
  //        test[2] = newCorners[2];
  //        printCoords2File(test, 1, "/home/rainer/workspace/c1.txt");
  //        test[0] = newCorners[3];
  //        test[1] = newCorners[4];
  //        test[2] = newCorners[5];
  //        printCoords2File(test, 1, "/home/rainer/workspace/c2.txt");
  //        test[0] = newCorners[6];
  //        test[1] = newCorners[7];
  //        test[2] = newCorners[8];
  //        printCoords2File(test, 1, "/home/rainer/workspace/c3.txt");
  //        test[0] = newCorners[9];
  //        test[1] = newCorners[10];
  //        test[2] = newCorners[11];
  //        printCoords2File(test, 1, "/home/rainer/workspace/c4.txt");

  //        cout << "dim: " << endl;
  //        cout << tmp_dimensions[0] << " " << tmp_dimensions[1] << endl;

          object3D* tmpObject = new object3D(this->getSeqenz(), this->getTimeStamp(), newCorners, tmp_centroid);

          /*
           * set additional values for object
           */
          tmpObject->setCoefficents(newCoeff);
          tmpObject->setSize(new_inliers);
          tmpObject->setCoords(tmp_coords);
          tmpObject->setNormals(tmp_normals);
          tmpObject->setDist(tmp_dist);
          tmpObject->setPhi(tmp_phi);
          tmpObject->setIntensity(tmp_intens);
          tmpObject->setMajorDim(tmp_dimensions[0]);
          tmpObject->setMinorDim(tmp_dimensions[1]);

          /*
           * check if object has already been found before
           */
          for(int i=0; i < amountObjects; i++)
          {
            //TODO: threshold for same object identification as launch variable
            thisIsANewObject = tmpObject->compareObject(object[i], 2*planeDetection_threshold);
          }

          if(thisIsANewObject && !thisIsAGroundObject)
          {
            /*
             * save object
             */
            object.resize(amountObjects+1);
            object[amountObjects] = tmpObject;

            //double* test = object[amountObjects]->getObjectCorners();
            amountObjects++;
          }

          if((new_size-new_inliers) < planeDetectin_minPoints)
            exit = true;

          delete [] tmp_dimensions;
          delete [] tmp_coords;
          delete [] tmp_normals;
          delete [] tmp_dist;
          delete [] tmp_phi;
          delete [] tmp_intens;
          delete [] tmp_normals_1;
          delete [] tmp_dist_1;
          delete [] tmp_phi_1;
          delete [] tmp_intens_1;

        }
        else
        {
          //PCL_ERROR ("Could not estimate a planar model for the given dataset.");
          exit = true;
        }
      }

      delete [] newCorners;
      delete [] newCoeff;
    }

  double* scan3D::indentifyCorners(double* coefficients, std::vector<double*>& centroid, double* coords, int amount_points, double* dimensions)
  {
    /*
     * Mirrorplane F out of coefficients
     * F: coefficients[0] * x + coefficients[1] * y + coefficients[2] * z + coefficients[3] = 0
     */
    double* tmp_corners = new double[12];
    double* tmp_coords  = new double[3*amount_points];
    double* tmp_center  = new double[3];

//    // To write in file
//    for (size_t i = 0; i < amount_points; ++i)
//    {
//      file_1 << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
//    }

//    /*
//     * copy coords of this object to tmp_coords
//     */
//    int amount_hits = 0;
//    for(int i = 0; i < this->_size; i++)
//    {
//      if(this->_objectBelonging[i] == objectNr)
//      {
//        tmp_coords[3*amount_hits]   = this->_coords_1[3*i];
//        tmp_coords[3*amount_hits+1] = this->_coords_1[3*i+1];
//        tmp_coords[3*amount_hits+2] = this->_coords_1[3*i+2];
//
//        //file_2 << tmp_coords[3*amount_hits] << " " << tmp_coords[3*amount_hits+1] << " " << tmp_coords[3*amount_hits+2] << endl;
//        amount_hits++;
//      }
//    }
//

//    /*
//     * Version 1: closest distance to origin
//     */
//    /*
//     * detect first corner (closest to the origin)
//     */
//    cout << "version 1:" << endl;
//    double tmp_dist1 = calcVectorLength(tmp_coords[0], tmp_coords[1], tmp_coords[2]);
//    int index_close = 0;
//    int index_far = 0;
//    for(int i = 1; i < amount_hits; i++)
//    {
//      if(tmp_dist1 >= calcVectorLength(tmp_coords[3*i], tmp_coords[3*i+1], tmp_coords[3*i+2]))
//         index_close = i;
//      if(tmp_dist1 <= calcVectorLength(tmp_coords[3*i], tmp_coords[3*i+1], tmp_coords[3*i+2]))
//         index_far = i;
//    }
//    corners[0] = tmp_coords[3*index_close];
//    corners[1] = tmp_coords[3*index_close+1];
//    corners[2] = tmp_coords[3*index_close+2];
//
//    corners[3] = tmp_coords[3*index_far];
//    corners[4] = tmp_coords[3*index_far+1];
//    corners[5] = tmp_coords[3*index_far+2];
//
//      for(int i=0; i < 2; i++)
//      {
//        cout << corners[3*i] << " " << corners[3*i+1] << " " << corners[3*i+2] << endl;
//        file_2 << corners[3*i] << " " << corners[3*i+1] << " " << corners[3*i+2] << endl;
//      }

//    /*
//     * Version 2: closes point of y (-y = left, y = right) and z (-z = bottom, z = top)
//     */
//    cout << "version 2:" << endl;
//    int index_left    = 0;
//    int index_right   = 0;
//    int index_top     = 0;
//    int index_bottom  = 0;
//
//    for(int i = 1; i < amount_hits; i++)
//    {
//      // check for left and right
//      if(tmp_coords[3*index_left+1] <= tmp_coords[3*i+1])
//        index_left = i;
//
//      if(tmp_coords[3*index_right+1] >= tmp_coords[3*i+1])
//        index_right = i;
//
//      // check for bottom and top
//      if(tmp_coords[3*index_top+2] <= tmp_coords[3*i+2])
//        index_top = i;
//
//      if(tmp_coords[3*index_bottom+2] >= tmp_coords[3*i+2])
//        index_bottom = i;
//    }
//
//    corners[0] = tmp_coords[3*index_left];
//    corners[1] = tmp_coords[3*index_left+1];
//    corners[2] = tmp_coords[3*index_left+2];
//
//    corners[3] = tmp_coords[3*index_right];
//    corners[4] = tmp_coords[3*index_right+1];
//    corners[5] = tmp_coords[3*index_right+2];
//
//    corners[6] = tmp_coords[3*index_top];
//    corners[7] = tmp_coords[3*index_top+1];
//    corners[8] = tmp_coords[3*index_top+2];
//
//    corners[9] = tmp_coords[3*index_bottom];
//    corners[10] = tmp_coords[3*index_bottom+1];
//    corners[11] = tmp_coords[3*index_bottom+2];
//
//      for(int i=0; i < 4; i++)
//      {
//        cout << corners[3*i] << " " << corners[3*i+1] << " " << corners[3*i+2] << endl;
//        file_2 << corners[3*i] << " " << corners[3*i+1] << " " << corners[3*i+2] << endl;
//      }

//  /*
//   * Version 3: dialoge of two farest points -> have to be the corners
//   */
//    /*
//     * detect first corner (closest to the origin)
//     */
////    cout << "version 3:" << endl;
//    double* corner_1  = new double[3];
//    double* corner_2  = new double[3];
//    double* corner_3  = new double[3];
//    double* corner_4  = new double[3];
//
//    /*
//     * assign first point and check for first corner
//     */
//    corner_1[0] = coords[0];
//    corner_1[1] = coords[1];
//    corner_1[2] = coords[2];
//
//    int index       = 0;
//    double dist_old = 0;
//    double dist_new = 0;
//    /*
//     * detect first corner
//     */
//    for(int i = 1; i < amount_points; i++)
//    {
//      corner_2[0] = coords[3*i];
//      corner_2[1] = coords[3*i+1];
//      corner_2[2] = coords[3*i+2];
//      dist_new    = calcVectorLength(corner_1, corner_2);
//
//      if(dist_old < dist_new)
//      {
//        dist_old = dist_new;
//        index = i;
//      }
//    }
//    /*
//     * assign first corner
//     */
//    corner_1[0] = coords[3*index];
//    corner_1[1] = coords[3*index+1];
//    corner_1[2] = coords[3*index+2];
//
////    // TOSHOW
////    cout << corner_1[0] << " " << corner_1[1] << " " << corner_1[2] << endl;
////    file_2 << corner_1[0] << " " << corner_1[1] << " " << corner_1[2] << endl;
//
//    /*
//     * correct first corner
//     */
//    //corner_1 = basepoint2Plane(coefficients, corner_1);
//
////    // TOSHOW
////    cout << corner_1[0] << " " << corner_1[1] << " " << corner_1[2] << endl;
////    file_2 << corner_1[0] << " " << corner_1[1] << " " << corner_1[2] << endl;
//
//    /*
//     * check for second corner
//     */
//    dist_old = 0;
//    index = 0;
//    /*
//     * detect second corner
//     */
//    for(int i = 1; i < amount_points; i++)
//    {
//      corner_2[0] = coords[3*i];
//      corner_2[1] = coords[3*i+1];
//      corner_2[2] = coords[3*i+2];
//      dist_new    = calcVectorLength(corner_1, corner_2);
//
//      if(dist_old < dist_new)
//      {
//        dist_old = dist_new;
//        index = i;
//      }
//    }
//
//    /*
//     * assign second corner
//     */
//    corner_2[0] = coords[3*index];
//    corner_2[1] = coords[3*index+1];
//    corner_2[2] = coords[3*index+2];
//
////    // TOSHOW
////    cout << corner_2[0] << " " << corner_2[1] << " " << corner_2[2] << endl;
////    file_2 << corner_2[0] << " " << corner_2[1] << " " << corner_2[2] << endl;
//
//    /*
//     * correct second corner
//     */
//    //corner_2 = basepoint2Plane(coefficients, corner_2);
//
////    // TOSHOW
////    cout << corner_2[0] << " " << corner_2[1] << " " << corner_2[2] << endl;
////    file_2 << corner_2[0] << " " << corner_2[1] << " " << corner_2[2] << endl;
//
//
//    /*
//     * create first diagonal with the two corners
//     * g: vec_x = lamda * vec_u + vec_m
//     */
//    double* vec_u = new double[3];
//    double* vec_m = new double[3];
//    double* fp    = new double[3];
//
//    vec_u[0] = corner_2[0] - corner_1[0];
//    vec_u[1] = corner_2[1] - corner_1[1];
//    vec_u[2] = corner_2[2] - corner_1[2];
//
//    vec_m = corner_1;
//
//    /*
//     * detect third corner
//     */
//    index         = 0;
//    dist_old      = 0;
//    dist_new      = 0;
//
//    double d      = 0;
//    double lamda  = 0;
//
//    for(int i = 0; i < amount_points; i++)
//    {
//      /*
//       * calculate d of plane H
//       * H: vec_u_x * coords_x + vec_u_y * coords_y + vec_u_z * coords_z = d
//       */
//      d = vec_u[0] * coords[3*i] + vec_u[1] * coords[3*i+1] + vec_u[2] * coords[3*i+2];
//
//      /*
//       * calculate base point fp
//       */
//      lamda = (d - (vec_u[0]*vec_m[0] + vec_u[1]*vec_m[1] + vec_u[2]*vec_m[2])) / (pow(vec_u[0], 2) + pow(vec_u[1], 2) + pow(vec_u[2], 2));
//
//      fp[0] = lamda * vec_u[0] + vec_m[0];
//      fp[1] = lamda * vec_u[1] + vec_m[1];
//      fp[2] = lamda * vec_u[2] + vec_m[2];
//
//      /*
//       * calculate distance between point x_i and base point to determine the farest point
//       */
//      corner_3[0] = coords[3*i];
//      corner_3[1] = coords[3*i+1];
//      corner_3[2] = coords[3*i+2];
//
//      dist_new    = calcVectorLength(corner_3, fp);
//
//      if(dist_old < dist_new)
//      {
//        dist_old = dist_new;
//        index = i;
//      }
//    }
//
//    /*
//     * assign third corner
//     */
//    corner_3[0] = coords[3*index];
//    corner_3[1] = coords[3*index+1];
//    corner_3[2] = coords[3*index+2];
//
////    // TOSHOW
////    cout << corner_3[0] << " " << corner_3[1] << " " << corner_3[2] << endl;
////    file_2 << corner_3[0] << " " << corner_3[1] << " " << corner_3[2] << endl;
//
//    /*
//     * correct third corner
//     */
//    //corner_3 = basepoint2Plane(coefficients, corner_3);
//
////    // TOSHOW
////    cout << corner_3[0] << " " << corner_3[1] << " " << corner_3[2] << endl;
////    file_2 << corner_3[0] << " " << corner_3[1] << " " << corner_3[2] << endl;
//
//    /*
//     * calculate fourth corner
//     */
//    corner_4[0] = corner_1[0] + corner_2[0] - corner_3[0];
//    corner_4[1] = corner_1[1] + corner_2[1] - corner_3[1];
//    corner_4[2] = corner_1[2] + corner_2[2] - corner_3[2];
//
////    // TOSHOW
////    cout << corner_4[0] << " " << corner_4[1] << " " << corner_4[2] << endl;
////    file_2 << corner_4[0] << " " << corner_4[1] << " " << corner_4[2] << endl;
//
//    /*
//     * resort and set corners
//     */
////    tmp_corners[0]  = corner_1[0];
////    tmp_corners[1]  = corner_1[1];
////    tmp_corners[2]  = corner_1[2];
////
////    tmp_corners[3]  = corner_3[0];
////    tmp_corners[4]  = corner_3[1];
////    tmp_corners[5]  = corner_3[2];
////
////    tmp_corners[6]  = corner_2[0];
////    tmp_corners[7]  = corner_2[1];
////    tmp_corners[8]  = corner_2[2];
////
////    tmp_corners[9]  = corner_4[0];
////    tmp_corners[10] = corner_4[1];
////    tmp_corners[11] = corner_4[2];
//
////    // TOSHOW
////    for(int i=0; i < 4; i++)
////    {
////      cout << tmp_corners[3*i] << " " << tmp_corners[3*i+1] << " " << tmp_corners[3*i+2] << endl;
////      file_2 << tmp_corners[3*i] << " " << tmp_corners[3*i+1] << " " << tmp_corners[3*i+2] << endl;
////    }
//
//    tmp_corners[0]  = corner_3[0];
//    tmp_corners[1]  = corner_3[1];
//    tmp_corners[2]  = corner_3[2];
//
//    tmp_corners[3]  = corner_1[0];
//    tmp_corners[4]  = corner_1[1];
//    tmp_corners[5]  = corner_1[2];
//
//    tmp_corners[6]  = corner_4[0];
//    tmp_corners[7]  = corner_4[1];
//    tmp_corners[8]  = corner_4[2];
//
//    tmp_corners[9]  = corner_2[0];
//    tmp_corners[10] = corner_2[1];
//    tmp_corners[11] = corner_2[2];
//
//
//    delete [] corner_1;
//    delete [] corner_2;
//    delete [] corner_3;
//    delete [] corner_4;
//
//    delete [] tmp_coords;
//    delete [] vec_u;
//    //delete [] vec_m;
//    delete [] fp;


//    /*
//     * Version 4a: principal component analysis of obvious (not finished yet)
//     *
//     * Matrix.cpp Matrix* Matrix::pcaAnalysis()
//     * void Matrix::svd(Matrix* U, double* s, Matrix* V)
//     *
//     */
//   /*
//    * create matrix
//    */
//   cout << "pcaAnalysis" << endl;
//   obvious::Matrix coords_mat(amount_points, 3);
//   obvious::Matrix* axes_mat;
//   double* axes = new double[18];
//   double* axes1 = new double[18];
//
//   for(int i=0; i < amount_points; i++)
//   {
//     coords_mat(i,0) = coords[3*i];
//     coords_mat(i,1) = coords[3*i+1];
//     coords_mat(i,2) = coords[3*i+2];
//   }
//
//   /*
//    * pca analysis
//    */
//   axes_mat = coords_mat.pcaAnalysis();
//   cout << "axes: " << endl;
//   axes_mat->print();
//   if(axes_mat != NULL)
//   {
//     cout << "check axes" << endl;
//     /*
//      * copy axes into vector
//      */
//     for(int l=0; l < 3; l++)
//     {
//       axes[6*l]      = (*axes_mat)(l,0);
//       axes[6*l+1]    = (*axes_mat)(l,1);
//       axes[6*l+2]    = (*axes_mat)(l,2);
//       axes[6*l+3]    = (*axes_mat)(l,3);
//       axes[6*l+4]    = (*axes_mat)(l,4);
//       axes[6*l+5]    = (*axes_mat)(l,5);
//     }
//
//     cout << "-" << endl;
//     for(int r=0; r < 6; r++)
//     {
//       axes1[3*r]      = (*axes_mat)(0,r);
//       axes1[3*r+1]    = (*axes_mat)(1,r);
//       axes1[3*r+2]    = (*axes_mat)(2,r);
//     }
//
//     cout << "print" << endl;
//     printCoords2File(coords, amount_points, "/home/rainer/workspace/coords.txt");
//     cout << "coords"<< endl;
//     printCoords2File(axes, 6, "/home/rainer/workspace/axes.txt");
//     cout << "coords2"<< endl;
//     printCoords2File(axes1, 6, "/home/rainer/workspace/axes1.txt");
//     cout << "pcaAnalysis end" << endl;
//   }
//   else
//   {
//     cout << "pca-Analysis unsucessful" << endl;
//   }

//  /*
//   * Version 4b: pca-analysis of pcl with pcl
//   *
//   * http://codextechnicanum.blogspot.de/2015/04/find-minimum-oriented-bounding-box-of.html
//   * http://pointclouds.org/documentation/tutorials/vfh_recognition.php
//   */
//
//    /*
//     * create matrix
//     */
//    cout << "pcaAnalysis" << endl;
//
//    double* axes_x = new double[6];
//    double* axes_y = new double[6];
//    double* axes_z = new double[6];
//    double delta_x = 0;
//    double delta_y = 0;
//    double delta_z = 0;
//    double* centroid = new double[6];
//
//    double* tf_coords = new double[amount_points];
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//    cloud->points.resize(amount_points);
//
//    /*
//    * fill cloud
//    */
//    for(int i = 0; i < amount_points; i++)
//    {
//     cloud->points[i].x = coords[3*i];
//     cloud->points[i].y = coords[3*i+1];
//     cloud->points[i].z = coords[3*i+2];
//    }
//
//    /*
//     * Compute principal directions
//     */
//    Eigen::Vector4f pcaCentroid;
//    pcl::compute3DCentroid(*cloud, pcaCentroid);
//    Eigen::Matrix3f covariance;
//    computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
//
//    /*
//     * calculate eigen-vectors
//     */
//    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
//    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
//    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));     // This line is necessary for proper orientation in some cases. The numbers come out the same without it, but the signs are different and the box doesn't get correctly oriented in some cases.
//
////    // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
////    Eigen::Matrix3f eigenVectors;
////    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
////    pcl::PCA<pcl::PointXYZ> pca;
////    pca.setInputCloud(cloud);
////    pca.project(*cloud, *cloudPCAprojection);
////
////    eigenVectors = pca.getEigenVectors();
////
////    cout << "V1: EigenVectors: " << endl;
////    cout << eigenVectorsPCA << endl;
////    cout << "V2: EigenVectors: " << endl;
////    cout << pca.getEigenVectors() << endl;
////    cout << "EigenValues: " << endl;
////    cout << pca.getEigenValues() << endl;
//
//    /*
//     * vectors for coordinate system of orientation object
//     */
//    centroid[0] = pcaCentroid(0);
//    centroid[1] = pcaCentroid(1);
//    centroid[2] = pcaCentroid(2);
//
//    /*
//     * axis of object coordinate system
//     */
//    for(int i=0; i < 3; i++)
//    {
//      axes_x[i] = 0;
//      axes_y[i] = 0;
//      axes_z[i] = 0;
//    }
//    /*
//     * print Version 1:
//     */
//    axes_x[3] = eigenVectorsPCA (0,0);
//    axes_x[4] = eigenVectorsPCA (1,0);
//    axes_x[5] = eigenVectorsPCA (2,0);
//
//    axes_y[3] = eigenVectorsPCA (0,1);
//    axes_y[4] = eigenVectorsPCA (1,1);
//    axes_y[5] = eigenVectorsPCA (2,1);
//
//    axes_z[3] = eigenVectorsPCA (0,2);
//    axes_z[4] = eigenVectorsPCA (1,2);
//    axes_z[5] = eigenVectorsPCA (2,2);
//
//    //TO SHOW
//    for(int i=0; i < 3; i++)
//    {
//      axes_x[i] = 0.000000001;
//      axes_y[i] = 0.000000001;
//      axes_z[i] = 0.000000001;
//    }
//    printCoords2File(axes_x, 2, "/home/rainer/workspace/x.txt");
//    printCoords2File(axes_y, 2, "/home/rainer/workspace/y.txt");
//    printCoords2File(axes_z, 2, "/home/rainer/workspace/z.txt");
//
////    /*
////     * print Version 2:
////     */
////    axes_x[3] = eigenVectors (0,0);
////    axes_x[4] = eigenVectors (1,0);
////    axes_x[5] = eigenVectors (2,0);
////
////    axes_y[3] = eigenVectors (0,1);
////    axes_y[4] = eigenVectors (1,1);
////    axes_y[5] = eigenVectors (2,1);
////
////    axes_z[3] = eigenVectors (0,2);
////    axes_z[4] = eigenVectors (1,2);
////    axes_z[5] = eigenVectors (2,2);
////    //TO SHOW
////   printCoords2File(axes_x, 2, "/home/rainer/workspace/x2.txt");
////   printCoords2File(axes_y, 2, "/home/rainer/workspace/y2.txt");
////   printCoords2File(axes_z, 2, "/home/rainer/workspace/z2.txt");
////   printCoords2File(coords, amount_points, "/home/rainer/workspace/coords.txt");
////   printCoords2File(centroid, 1, "/home/rainer/workspace/centroid.txt");
//   for(int i=0; i < 3; i++)
//   {
//     axes_x[i] = 0;
//     axes_y[i] = 0;
//     axes_z[i] = 0;
//   }
//
//   // Transform the original cloud to the origin where the principal components correspond to the axes.
//   Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
//   projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
//   projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
//   // Get the minimum and maximum points of the transformed cloud.
//   pcl::PointXYZ minPoint, maxPoint;
//   pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
//   const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
//
//   /*
//    * Final transform
//    */
//   const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
//   const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
//
//   /*
//    * get dimensions
//    */
//   delta_x = maxPoint.x - minPoint.x;
//   delta_y = maxPoint.y - minPoint.y;
//   delta_z = maxPoint.z - minPoint.z;
//
//   cout << "dimensions: " << endl;
//   cout << "x : " << delta_x << endl;
//   cout << "y : " << delta_y << endl;
//   cout << "z : " << delta_z << endl;
//
////   cout << "transform coords" << endl;
////   /*
////    * transformed object coords
////    */
////   for(int i = 0; i < amount_points; i++)
////   {
////    tf_coords[3*i]    = cloudPointsProjected->points[i].x;
////    tf_coords[3*i+1]  = cloudPointsProjected->points[i].y;
////    tf_coords[3*i+2]  = cloudPointsProjected->points[i].z;
////   }
////   printCoords2File(tf_coords, amount_points, "/home/rainer/workspace/tfcoords.txt");

 /*
  * Version 4c: principal component analysis of pcl with feature extractor
  * http://pointclouds.org/documentation/tutorials/moment_of_inertia.php
  *
  */
//   cout << "pcaAnalysis with feature extractor" << endl;

   double* axes_x = new double[6];
   double* axes_y = new double[6];
   double* axes_z = new double[6];
   double delta_x = 0;
   double delta_y = 0;
   double delta_z = 0;
   //double* centroid = new double[6];
   double major_dim = 0;
   double middle_dim = 0;

   double* corner_1  = new double[3];
   double* corner_2  = new double[3];
   double* corner_3  = new double[3];
   double* corner_4  = new double[3];

   double* tf_coords = new double[amount_points];

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
   cloud->points.resize(amount_points);

   /*
   * fill cloud
   */
   for(int i = 0; i < amount_points; i++)
   {
    cloud->points[i].x = coords[3*i];
    cloud->points[i].y = coords[3*i+1];
    cloud->points[i].z = coords[3*i+2];
   }

   pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
   feature_extractor.setInputCloud (cloud);
   feature_extractor.compute ();

   std::vector <float> moment_of_inertia;
   std::vector <float> eccentricity;
  // pcl::PointXYZ min_point_AABB;
  // pcl::PointXYZ max_point_AABB;
   pcl::PointXYZ min_point_OBB;
   pcl::PointXYZ max_point_OBB;
   pcl::PointXYZ position_OBB;
   Eigen::Matrix3f rotational_matrix_OBB;
   float major_value, middle_value, minor_value;
   Eigen::Vector3f major_vector, middle_vector, minor_vector;
   Eigen::Vector3f mass_center;

   feature_extractor.getMomentOfInertia (moment_of_inertia);
   feature_extractor.getEccentricity (eccentricity);
   //feature_extractor.getAABB (min_point_AABB, max_point_AABB);
   feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
   feature_extractor.getEigenValues (major_value, middle_value, minor_value);
   feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
   feature_extractor.getMassCenter (mass_center);

  // cout << "dimensions AABB: " << endl;
  // cout << "x : " << max_point_AABB.x -  min_point_AABB.x << endl;
  // cout << "y : " << max_point_AABB.y -  min_point_AABB.y << endl;
  // cout << "z : " << max_point_AABB.z -  min_point_AABB.z << endl;

//   cout << "vectors" << endl;
//   cout << mass_center << endl;
//   cout << "-" << endl;
//   cout << major_vector << endl;
//   cout << "-" << endl;
//   cout << middle_vector << endl;
//   cout << "-" << endl;
//   cout << minor_vector << endl;

   /*
    * get dimensions
    */
   delta_x = max_point_OBB.x - min_point_OBB.x;
   delta_y = max_point_OBB.y - min_point_OBB.y;
   delta_z = max_point_OBB.z - min_point_OBB.z;

//   cout << "dimensions: " << endl;
//   cout << "x : " << delta_x << endl;
//   cout << "y : " << delta_y << endl;
//   cout << "z : " << delta_z << endl;

   if(delta_x >= delta_y)
   {
     if(delta_x >= delta_z)
     {
       major_dim = (double) (max_point_OBB.x - min_point_OBB.x);
       if(delta_z >= delta_y)
       {
         middle_dim = max_point_OBB.z - min_point_OBB.z;
       }
       else
       {
         middle_dim = max_point_OBB.y - min_point_OBB.y;
       }
     }
     else
     {
       major_dim = max_point_OBB.z - min_point_OBB.z;
       middle_dim = max_point_OBB.x - min_point_OBB.x;
     }
   }
   else
   {
     if(delta_y >= delta_z)
     {
       major_dim = max_point_OBB.y - min_point_OBB.y;
       if(delta_x >= delta_z)
       {
         middle_dim = max_point_OBB.x - min_point_OBB.x;
       }
       else
       {
         middle_dim = max_point_OBB.z - min_point_OBB.z;

       }
     }
     else
     {
       major_dim = max_point_OBB.z - min_point_OBB.z;
       middle_dim = max_point_OBB.y - min_point_OBB.y;
     }
   }

//   cout << "dimensions" << endl;
//   cout << major_dim << endl;
//   cout << middle_dim << endl;

   dimensions[0] = major_dim;
   dimensions[1] = middle_dim;

   /*
    * vectors for coordinate system of orientation object
    */
   tmp_center[0] = mass_center(0);
   tmp_center[1] = mass_center(1);
   tmp_center[2] = mass_center(2);

   centroid[0] = tmp_center;

//   double* test = new double[3];
//   test = centroid[0];
//   cout << "mass center" << endl;
//   cout << tmp_center[0] << " " << tmp_center[1] << " " <<  tmp_center[2] <<  endl;
//   cout << test[0] << " " << test[1] << " " <<  test[2] <<  endl;

//   /*
//    * axes assigned to the origin
//    */
//   for(int i=0; i < 3; i++)
//   {
//     axes_x[i] = 0.000000001;
//     axes_y[i] = 0.000000001;
//     axes_z[i] = 0.000000001;
//   }
//   axes_x[3] = minor_vector(0);
//   axes_x[4] = minor_vector(1);
//   axes_x[5] = minor_vector(2);
//
//   axes_y[3] = middle_vector(0);
//   axes_y[4] = middle_vector(1);
//   axes_y[5] = middle_vector(2);
//
//   axes_z[3] = major_vector(0);
//   axes_z[4] = major_vector(1);
//   axes_z[5] = major_vector(2);

//  printCoords2File(axes_x, 2, "/home/rainer/workspace/x.txt");
//  printCoords2File(axes_y, 2, "/home/rainer/workspace/y.txt");
//  printCoords2File(axes_z, 2, "/home/rainer/workspace/z.txt");
//  printCoords2File(coords, amount_points, "/home/rainer/workspace/coords.txt");
//  printCoords2File(centroid, 1, "/home/rainer/workspace/centroid.txt");

   /*
    * axes assigned to the centroid of the object
    */
    for(int i=0; i < 3; i++)
    {
      axes_x[i] = tmp_center[i];
      axes_y[i] = tmp_center[i];
      axes_z[i] = tmp_center[i];
    }

    axes_x[3] = minor_vector(0) + tmp_center[0];
    axes_x[4] = minor_vector(1) + tmp_center[1];
    axes_x[5] = minor_vector(2) + tmp_center[2];

    axes_y[3] = middle_vector(0) + tmp_center[0];
    axes_y[4] = middle_vector(1) + tmp_center[1];
    axes_y[5] = middle_vector(2) + tmp_center[2];

    axes_z[3] = major_vector(0) + tmp_center[0];
    axes_z[4] = major_vector(1) + tmp_center[1];
    axes_z[5] = major_vector(2) + tmp_center[2];

//    //TO SHOW
//    printCoords2File(axes_x, 2, "/home/rainer/workspace/x2.txt");
//    printCoords2File(axes_y, 2, "/home/rainer/workspace/y2.txt");
//    printCoords2File(axes_z, 2, "/home/rainer/workspace/z2.txt");

    /*
     * get corners of 3D-object
     */
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

    p1 = rotational_matrix_OBB * p1 + position;
    p2 = rotational_matrix_OBB * p2 + position;
    p3 = rotational_matrix_OBB * p3 + position;
    p4 = rotational_matrix_OBB * p4 + position;
    p5 = rotational_matrix_OBB * p5 + position;
    p6 = rotational_matrix_OBB * p6 + position;
    p7 = rotational_matrix_OBB * p7 + position;
    p8 = rotational_matrix_OBB * p8 + position;

//    /*
//     * TO SHOW:
//     */
//    double* pt1 = new double[3];
//    pt1[0] = p1 (0);
//    pt1[1] = p1 (1);
//    pt1[2] = p1 (2);
//
//    double* pt2 = new double[3];
//    pt2[0] = p2 (0);
//    pt2[1] = p2 (1);
//    pt2[2] = p2 (2);
//
//    double* pt3 = new double[3];
//    pt3[0] = p3 (0);
//    pt3[1] = p3 (1);
//    pt3[2] = p3 (2);
//
//    double* pt4 = new double[3];
//    pt4[0] = p4 (0);
//    pt4[1] = p4 (1);
//    pt4[2] = p4 (2);
//
//    double* pt5 = new double[3];
//    pt5[0] = p5 (0);
//    pt5[1] = p5 (1);
//    pt5[2] = p5 (2);
//
//    double* pt6 = new double[3];
//    pt6[0] = p6 (0);
//    pt6[1] = p6 (1);
//    pt6[2] = p6 (2);
//
//    double* pt7 = new double[3];
//    pt7[0] = p7 (0);
//    pt7[1] = p7 (1);
//    pt7[2] = p7 (2);
//
//    double* pt8 = new double[3];
//    pt8[0] = p8 (0);
//    pt8[1] = p8 (1);
//    pt8[2] = p8 (2);
//
//    printCoords2File(pt1, 1, "/home/rainer/workspace/pt1.txt");
//    printCoords2File(pt2, 1, "/home/rainer/workspace/pt2.txt");
//    printCoords2File(pt3, 1, "/home/rainer/workspace/pt3.txt");
//    printCoords2File(pt4, 1, "/home/rainer/workspace/pt4.txt");
//    printCoords2File(pt5, 1, "/home/rainer/workspace/pt5.txt");
//    printCoords2File(pt6, 1, "/home/rainer/workspace/pt6.txt");
//    printCoords2File(pt7, 1, "/home/rainer/workspace/pt7.txt");
//    printCoords2File(pt8, 1, "/home/rainer/workspace/pt8.txt");

    /*
     * calculate corners of plane object
     */
    corner_1[0] = 0.5*(p2(0) + p1(0));
    corner_1[1] = 0.5*(p2(1) + p1(1));
    corner_1[2] = 0.5*(p2(2) + p1(2));

    corner_2[0] = 0.5*(p4(0) + p3(0));
    corner_2[1] = 0.5*(p4(1) + p3(1));
    corner_2[2] = 0.5*(p4(2) + p3(2));

    corner_3[0] = 0.5*(p8(0) + p7(0));
    corner_3[1] = 0.5*(p8(1) + p7(1));
    corner_3[2] = 0.5*(p8(2) + p7(2));

    corner_4[0] = 0.5*(p6(0) + p5(0));
    corner_4[1] = 0.5*(p6(1) + p5(1));
    corner_4[2] = 0.5*(p6(2) + p5(2));

//    printCoords2File(corner_1, 1, "/home/rainer/workspace/c1.txt");
//    printCoords2File(corner_2, 1, "/home/rainer/workspace/c2.txt");
//    printCoords2File(corner_3, 1, "/home/rainer/workspace/c3.txt");
//    printCoords2File(corner_4, 1, "/home/rainer/workspace/c4.txt");

    /*
     * sort corners
     */
    sortCorners(corner_1, corner_2, corner_3, corner_4, tmp_corners);

    delete [] corner_1;
    delete [] corner_2;
    delete [] corner_3;
    delete [] corner_4;

    delete [] tmp_coords;

    delete [] axes_x;
    delete [] axes_y;
    delete [] axes_z;

  return tmp_corners;
 }

  void scan3D::printScanCoords2File(const char* filename)
  {
    vector<string> splitfilename;
    std::string infilename(filename);

    /*
     * cut .txt and add the right file ending
     */
    std::size_t pos_end = infilename.find(".txt");
    std::string str2 = infilename.substr (0,pos_end);     // go from 0 till pos

    std::string sfilename_v   = str2 + "_valid.txt";
    //cout << sfilename_v << endl;
    std::string sfilename_eS  = str2 + "_errorSurf.txt";
    std::string sfilename_bE  = str2 + "_behError.txt";
    std::string sfilename_rS  = str2 + "_refSurf.txt";
    std::string sfilename_bR  = str2 + "_behRef.txt";
    std::string sfilename_mS  = str2 + "_mirSurf.txt";
    std::string sfilename_bM  = str2 + "_behMir.txt";
    std::string sfilename_u   = str2 + "_unchecked.txt";

    ofstream file_v;
    ofstream file_eS;
    ofstream file_bE;
    ofstream file_rS;
    ofstream file_bR;
    ofstream file_mS;
    ofstream file_bM;
    ofstream file_u;
    ofstream file;


    const char* filename_v  = sfilename_v.c_str();
    const char* filename_eS = sfilename_eS.c_str();
    const char* filename_bE = sfilename_bE.c_str();
    const char* filename_rS = sfilename_rS.c_str();
    const char* filename_bR = sfilename_bR.c_str();
    const char* filename_mS = sfilename_mS.c_str();
    const char* filename_bM = sfilename_bM.c_str();
    const char* filename_u  = sfilename_u.c_str();

    file_v.open(filename_v);
    file_eS.open(filename_eS);
    file_bE.open(filename_bE);
    file_rS.open(filename_rS);
    file_bR.open(filename_bR);
    file_mS.open(filename_mS);
    file_bM.open(filename_bM);
    file_u.open(filename_u);
    file.open(filename);

    object3D::objType* objectType = this->getObjectType1();
    double* coords = this->getCoords1();

    int size = this->getSize();

    // write in file
    for(int i = 0; i < size; i++)
    {
      if(!isnan(coords[3*i]) && !isnan(coords[3*i+1]) && !isnan(coords[3*i+2]) && (!(coords[3*i] == 0) && !(coords[3*i+1] == 0) && !(coords[3*i+2] == 0)) && (this->_mask_1[i])&& (!(abs(coords[3*i]) > 60.0) && !(abs(coords[3*i+1]) > 60.0) && !(abs(coords[3*i+2]) > 60.0)))
      {
        file << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;

        if(objectType[i] == object3D::validPoint)
          file_v << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
        if(objectType[i] == object3D::errorSurface)
          file_eS << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
        if(objectType[i] == object3D::behindErrorS)
          file_bE << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
        if(objectType[i] == object3D::reflectiveSurface)
          file_rS << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
        if(objectType[i] == object3D::behindReflective)
          file_bR << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
        if(objectType[i] == object3D::transpSurface)
          file_mS << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
        if(objectType[i] == object3D::behindTransparent)
          file_bM << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
        if(objectType[i] == object3D::unchecked)
          file_u << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;

      }
    }
    file_v.close();
    file_eS.close();
    file_bE.close();
    file_rS.close();
    file_bR.close();
    file_mS.close();
    file_bM.close();
    file_u.close();
    file.close();


    std::string sfilename_v2   = str2 + "_valid2.txt";
    //cout << sfilename_v << endl;
    std::string sfilename_eS2  = str2 + "_errorSurf2.txt";
    std::string sfilename_bE2  = str2 + "_behError2.txt";
    std::string sfilename_rS2  = str2 + "_refSurf2.txt";
    std::string sfilename_bR2  = str2 + "_behRef2.txt";
    std::string sfilename_mS2  = str2 + "_mirSurf2.txt";
    std::string sfilename_bM2  = str2 + "_behMir2.txt";
    std::string sfilename_u2  = str2 + "_unchecked2.txt";
    std::string sfilename2   = str2 + "2.txt";

    ofstream file_v2;
    ofstream file_eS2;
    ofstream file_bE2;
    ofstream file_rS2;
    ofstream file_bR2;
    ofstream file_mS2;
    ofstream file_bM2;
    ofstream file_u2;
    ofstream file2;


    const char* filename_v2  = sfilename_v2.c_str();
    const char* filename_eS2 = sfilename_eS2.c_str();
    const char* filename_bE2 = sfilename_bE2.c_str();
    const char* filename_rS2 = sfilename_rS2.c_str();
    const char* filename_bR2 = sfilename_bR2.c_str();
    const char* filename_mS2 = sfilename_mS2.c_str();
    const char* filename_bM2 = sfilename_bM2.c_str();
    const char* filename_u2  = sfilename_u2.c_str();
    const char* filename2  = sfilename2.c_str();

    file_v2.open(filename_v2);
    file_eS2.open(filename_eS2);
    file_bE2.open(filename_bE2);
    file_rS2.open(filename_rS2);
    file_bR2.open(filename_bR2);
    file_mS2.open(filename_mS2);
    file_bM2.open(filename_bM2);
    file_u2.open(filename_u2);
    file2.open(filename2);

    coords = this->getCoords2();
    objectType = this->getObjectType2();

    // write in file
    for(int i = 0; i < size; i++)
    {
      if(!isnan(coords[3*i]) && !isnan(coords[3*i+1]) && !isnan(coords[3*i+2]) && (!(coords[3*i] == 0) && !(coords[3*i+1] == 0) && !(coords[3*i+2] == 0)) && (this->_mask_1[i])&& (!(abs(coords[3*i]) > 60.0) && !(abs(coords[3*i+1]) > 60.0) && !(abs(coords[3*i+2]) > 60.0)))
      {
        file2 << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;

        //cout << this->_objectType_1[i] << "-" << this->_objectType_2[i] << "/";

        if(objectType[i] == object3D::validPoint)
          file_v2 << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
        if(objectType[i] == object3D::errorSurface)
          file_eS2 << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
        if(objectType[i] == object3D::behindErrorS)
          file_bE2 << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
        if(objectType[i] == object3D::reflectiveSurface)
          file_rS2 << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
        if(objectType[i] == object3D::behindReflective)
          file_bR2 << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
        if(objectType[i] == object3D::transpSurface)
          file_mS2 << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
        if(objectType[i] == object3D::behindTransparent)
          file_bM2 << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
        if(objectType[i] == object3D::unchecked)
          file_u2 << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;

      }
    }
    //cout << endl;
    file_v2.close();
    file_eS2.close();
    file_bE2.close();
    file_rS2.close();
    file_bR2.close();
    file_mS2.close();
    file_bM2.close();
    file_u2.close();
    file2.close();

  }


/* END CLASS */
}
