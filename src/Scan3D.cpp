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
  scan3D::scan3D(unsigned int seq, unsigned long timestamp, std::string frame_id, unsigned int fragmentid, unsigned int fragement_size, unsigned int size, double* coords_1, unsigned int* intens_1, double* coords_2, unsigned int* intens_2)
  {
    _seq            = seq;
    _time_stamp     = timestamp;
    _frame_id       = frame_id;
    _fragmentid     = fragmentid;
    _fragmet_size   = fragement_size;

    _size           = size;

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
    /*
     * prefilter data (erase all points, which are to close or to far away from the scanner)
     */
    for(unsigned int i=0; i<_size; i++)
    {
      if((this->_dist_1[i] < min_dist) or (this->_dist_1[i] > max_dist))
      {
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
  }

  void scan3D::validPointFilter()
  {
    /*
     * prefilter data (erase all points, which are to close or to far away from the scanner)
     */
    for(unsigned int i=0; i<_size; i++)
    {
      if(isnan(_coords_1[3*i]) or isnan(_coords_1[3*i+1]) or isnan(_coords_1[3*i+2]) or isnan(_coords_2[3*i]) or isnan(_coords_2[3*i+1]) or isnan(_coords_2[3*i+2]))
      {
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
  }

  void scan3D::outlierFilterKMean(int kMean, double stdDeviation, object3D::objType objectType)
  {
    /*
     * pcl-version
     * http://pointclouds.org/documentation/tutorials/statistical_outlier.php
     */
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
          m++;
        }

      }
      // Create the filtering object
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (cloud);
      sor.setMeanK (kMean);
      sor.setStddevMulThresh (stdDeviation);
      sor.filter (*cloud_filtered);


      sor.setNegative (true);
      sor.filter (*cloud_filtered);
  }

  void scan3D::outlierFilterRadius(int neightbors, double radius, object3D::objType objectType)
  {
    /*
     * pcl-version
     * http://pointclouds.org/documentation/tutorials/statistical_outlier.php
     * http://docs.pointclouds.org/1.7.0/classpcl_1_1_radius_outlier_removal.html
     */
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

          tmp_pointLocation_1[m] = i;
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
      }
  }

  unsigned int scan3D::identifyReflections(double thres_substract_distance)
  {
    object3D::objType* objectType1  = new object3D::objType[this->_size];
    object3D::objType* objectType2   = new object3D::objType[this->_size];

    unsigned int reflectionFound       = 0;

    /*
     * check each point, if dist_1 and dist_2 are different
     */
    for(int i=0; i<this->_size; i++)
    {
      if(this->_mask_1[i])
      {
        if((thres_substract_distance >= abs(_dist_2[i] - _dist_1[i])) or (_dist_2[i] == 0))
        {
          objectType1[i] = object3D::validPoint;         // valid scan point
          objectType2[i] = object3D::validPoint;
        }
        else
        {
          if(_dist_1[i] < _dist_2[i])
          {
            objectType1[i] = object3D::errorSurface;        // point on the object
            objectType2[i] = object3D::behindErrorS;               // point influenced by the object
          }
          else
          {
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

            objectType1[i] = object3D::errorSurface;         // point influenced by the object
            objectType2[i] = object3D::behindErrorS;                // point on the object
          }
          reflectionFound++;
        }
      }
      else
      {
        objectType1[i] = object3D::unchecked;
        objectType2[i] = object3D::unchecked;
      }
    }

    /*
     * set new objectTypes
     */
    this->setObjectType1(objectType1);
    this->setObjectType2(objectType2);
    return reflectionFound;
  }

  void scan3D::indentifyObjects(std::vector<mirrordetector::object3D*>& object, double planeDetection_threshold, unsigned int planeDetectin_minPoints)
  {
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
    cout << "cloud for cluster size" << cloudIn_size<< endl;

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
        tmp_pLocation_1[n]= i;          // saves the index where the point was located in the original scan, the index of tmp_pointLocation_1 is refering to the pcl-cloud
        n++;
      }
    }

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

    vg.setLeafSize (planeDetection_threshold/10.0, planeDetection_threshold/10.0, planeDetection_threshold/10.0);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

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
    cout << "found clusters: " << cluster_indices.size() << endl;

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

      /*
       * copy coefficients of plane
       */
      for(int m=0; m<4; m++)
      {
        newCoeff[m] = (double)coefficients_plane->values[m];
      }

      double* tmp_dimensions = new double[2];
      double* tmp_centroid = new double[3];

      /*
       * identify object boarders
       */
      newCorners = this->indentifyCorners(newCoeff, centroid, tmp_coords, new_inliers_plane, tmp_dimensions);
      tmp_centroid = centroid[0];

      object3D* tmpObject = new object3D(this->getSeqenz(), this->getTimeStamp(), newCorners, tmp_centroid);

      /*
       * set additional values for object
       */
      tmpObject->setCoefficents(newCoeff);
      tmpObject->setSize(new_inliers);
      tmpObject->setCoords(tmp_coords);

      tmpObject->setMajorDim(tmp_dimensions[0]);
      tmpObject->setMinorDim(tmp_dimensions[1]);

      /*
       * check if object already exists (maybe not necessary since there are clustering)
       */
      for(int i=0; i < amountObjects; i++)
      {
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

        amountObjects++;
      }

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_filtered = *cloud_f;
      plane++;

      delete [] tmp_dimensions;
      delete [] tmp_coords;

      j++;
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

 /*
  * principal component analysis of pcl with feature extractor
  * http://pointclouds.org/documentation/tutorials/moment_of_inertia.php
  *
  */

   double* axes_x = new double[6];
   double* axes_y = new double[6];
   double* axes_z = new double[6];
   double delta_x = 0;
   double delta_y = 0;
   double delta_z = 0;
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
   pcl::PointXYZ min_point_OBB;
   pcl::PointXYZ max_point_OBB;
   pcl::PointXYZ position_OBB;
   Eigen::Matrix3f rotational_matrix_OBB;
   float major_value, middle_value, minor_value;
   Eigen::Vector3f major_vector, middle_vector, minor_vector;
   Eigen::Vector3f mass_center;

   feature_extractor.getMomentOfInertia (moment_of_inertia);
   feature_extractor.getEccentricity (eccentricity);
   feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
   feature_extractor.getEigenValues (major_value, middle_value, minor_value);
   feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
   feature_extractor.getMassCenter (mass_center);

   /*
    * get dimensions
    */
   delta_x = max_point_OBB.x - min_point_OBB.x;
   delta_y = max_point_OBB.y - min_point_OBB.y;
   delta_z = max_point_OBB.z - min_point_OBB.z;

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

   dimensions[0] = major_dim;
   dimensions[1] = middle_dim;

   /*
    * vectors for coordinate system of orientation object
    */
   tmp_center[0] = mass_center(0);
   tmp_center[1] = mass_center(1);
   tmp_center[2] = mass_center(2);

   centroid[0] = tmp_center;

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


/* END CLASS */
}
