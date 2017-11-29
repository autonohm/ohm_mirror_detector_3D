/**
* @file   Object3D.cpp
* @author Rainer Koch
* @date   21.03.2016
*
*
*/

#include "Object3D.h"
#include "testFunctions.h"

namespace mirrordetector
{
  object3D::object3D()
  {
    _seq                = 0;
    _time_stamp         = 0;

    _objectCorners      = NULL;
    _objectType         = unchecked;
    _objectCoefficients = NULL;

    _size               = 0;
    _coords             = NULL;
    _normals            = NULL;
    _dist               = NULL;
    _phi                = NULL;
    _intensity          = NULL;

    _major_dim           = 0;
    _minor_dim           = 0;

    _transformation     = Eigen::Matrix4f::Identity();


  }

  object3D::object3D(unsigned int seq, unsigned long time, double* corners, double* centroid)
  {
    _seq                = seq;
    _time_stamp         = time;

    _objectCorners      = new double[12];
    _objectCentroid     = new double[3];

    _pub_Results        = false;

    _objectType         = unchecked;
    _objectCoefficients = NULL;

    _size               = 0;
    _coords             = NULL;
    _normals            = NULL;
    _dist               = NULL;
    _phi                = NULL;
    _intensity          = NULL;

    _transformation     = Eigen::Matrix4f::Identity();


    memcpy(_objectCorners, corners, 12*sizeof(double));
    memcpy(_objectCentroid, centroid, 3*sizeof(double));

  }

  object3D::~object3D()
  {
    delete [] _coords;
    delete [] _normals;
    delete [] _intensity;
    delete [] _dist;
    delete [] _phi;

    delete [] _objectCorners;
    delete [] _objectCoefficients;
  }

  void object3D::setCoords(double* coords)
  {
    if(!_coords)
    {
      if(_size == 0)
      {
        //HAVE TO BE CHECKED
        _size = sizeof(coords) / sizeof(*coords);
      }
      //cout << "new: " << _size << endl;
      _coords = new double[_size];
    }
    memcpy(_coords, coords, 3*_size*sizeof(double));
  }
  double* object3D::getCoords()
  {
    return _coords;
  }

  void object3D::setNormals(double* normals)
  {
    if(!_normals)
    {
      if(_size == 0)
      {
        //HAVE TO BE CHECKED

        _size = sizeof(normals) / sizeof(*normals);
      }
      _normals     = new double[3*_size];
    }
    memcpy(_normals, normals, 3*_size*sizeof(double));
  }
  double* object3D::getNormals()
  {
    return _normals;
  }

  void object3D::setIntensity(unsigned int* intens)
  {
    if(!_intensity)
    {
      if(_size == 0)
      {
        //HAVE TO BE CHECKED

        _size = sizeof(intens) / sizeof(*intens);
      }
      _intensity     = new unsigned int[3*_size];
    }
    memcpy(_intensity, intens, _size*sizeof(unsigned int));
  }
  unsigned int* object3D::getIntensity()
  {
    return _intensity;
  }

  void object3D::setDist(double* dist)
  {
    if(!_dist)
    {
      if(_size == 0)
      {
        //HAVE TO BE CHECKED

        _size = sizeof(dist) / sizeof(*dist);
      }
      _dist     = new double[3*_size];
    }
    memcpy(_dist, dist, _size*sizeof(double));
  }
  double* object3D::getDist()
  {
    return _dist;
  }

  void object3D::setPhi(double* phi)
  {
    if(!_phi)
    {
      if(_size == 0)
      {
        //HAVE TO BE CHECKED

        _size = sizeof(phi) / sizeof(*phi);
      }
      _phi     = new double[3*_size];
    }
    memcpy(_phi, phi, _size*sizeof(double));
  }
  double* object3D::getPhi()
  {
    return _phi;
  }

  void object3D::setObjectType(object3D::objType objectType)
  {
    _objectType = objectType;
  }
  object3D::objType object3D::getObjectType()
  {
    return _objectType;
  }


  void object3D::setTransformation(Eigen::Matrix4f Trans)
  {
    _transformation(0,0) = Trans(0,0);
    _transformation(0,1) = Trans(0,1);
    _transformation(0,2) = Trans(0,2);
    _transformation(0,3) = Trans(0,3);

    _transformation(1,0) = Trans(1,0);
    _transformation(1,1) = Trans(1,1);
    _transformation(1,2) = Trans(1,2);
    _transformation(1,3) = Trans(1,3);

    _transformation(2,0) = Trans(2,0);
    _transformation(2,1) = Trans(2,1);
    _transformation(2,2) = Trans(2,2);
    _transformation(2,3) = Trans(2,3);

    _transformation(3,0) = Trans(3,0);
    _transformation(3,1) = Trans(3,1);
    _transformation(3,2) = Trans(3,2);
    _transformation(3,3) = Trans(3,3);
  }

  Eigen::Matrix4f object3D::getTransformation()
  {
    return _transformation;
  }


  void object3D::setCoefficents(double* coefficients)
  {
    //int size = sizeof(coefficients) / sizeof(*coefficients);

    if(!_objectCoefficients)
      _objectCoefficients     = new double[4];

    memcpy(_objectCoefficients, coefficients, 4*sizeof(double));
  }
  double* object3D::getCoefficents()
  {
    return _objectCoefficients;
  }

  void object3D::setCentroid(double* centroid)
  {
    //int size = sizeof(coefficients) / sizeof(*coefficients);

    if(!_objectCentroid)
      _objectCentroid     = new double[3];

    memcpy(_objectCentroid, centroid, 3*sizeof(double));
  }
  double* object3D::getCentroid()
  {
    return _objectCentroid;
  }


  void object3D::setMajorDim(double dim)
  {
    _major_dim = dim;
  }
  double object3D::getMajorDim()
  {
    return _major_dim;
  }

  void object3D::setMinorDim(double dim)
  {
    _minor_dim = dim;
  }

  double object3D::getMinorDim()
  {
    return _minor_dim;
  }

  unsigned int object3D::getSeqenz()
  {
    return _seq;
  }

  unsigned long object3D::getTimeStamp()
  {
    return _time_stamp;
  }

  unsigned int object3D::getSize()
  {
    return _size;
  }

  void object3D::setSize(unsigned int size)
  {
    this->_size = size;
    this->_coords = new double[3*_size];
    this->_normals = new double[3*_size];

    this->_dist = new double[_size];
    this->_phi = new double[_size];
    this->_intensity = new unsigned int[_size];

  }

  void object3D::setPubResults(bool pubResults)
  {
    _pub_Results = pubResults;
  }
  bool object3D::getPubResults()
  {
    return _pub_Results;
  }

  double* object3D::getObjectCorners()
  {
    return _objectCorners;
  }

  bool object3D::compareObject(object3D* existingObject, double thres)
  {
    //TODO: Should be checked in a better way!
    //cout << "Check object" << endl;
    bool isNew = true;

//    double* tmp_corner_O1;
//    double* tmp_coords_O1;
//
//    double* tmp_corner_O2;
//    double* tmp_coords_O2;

    double* tmp_coeff_O1;
    double* tmp_coeff_O2;

    tmp_coeff_O1 = this->getCoefficents();
    tmp_coeff_O2 = existingObject->getCoefficents();

//    cout << "Coeff 1: " << endl;
//    cout << tmp_coeff_O1[0] << "/" << tmp_coeff_O1[1] << "/" << tmp_coeff_O1[2] << "/" << tmp_coeff_O1[3] << endl;
//    cout << tmp_coeff_O2[0] << "/" << tmp_coeff_O2[1] << "/" << tmp_coeff_O2[2] << "/" << tmp_coeff_O2[3] << endl;
//
//    tmp_corner_O1 = this->getObjectCorners();
//    tmp_corner_O2 = existingObject->getObjectCorners();
//
//    tmp_coords_O1 = this->getCoords();
//    tmp_coords_O2 = existingObject->getCoords();
//
//    printCoords2File(tmp_corner_O1, 4, "/home/rainer/workspace/corner1.txt");
//    printCoords2File(tmp_corner_O2, 4, "/home/rainer/workspace/corner2.txt");
//
//    cout<< "size" << this->getSize() << " / " << existingObject->getSize() << endl;
//    printCoords2File(tmp_coords_O1, this->getSize(), "/home/rainer/workspace/obj1.txt");
//    printCoords2File(tmp_coords_O2, existingObject->getSize(), "/home/rainer/workspace/obj2.txt");


    /*
     * if the normal vectors are equal
     */
    if((abs(tmp_coeff_O1[0] - tmp_coeff_O2[0]) < thres) && (abs(tmp_coeff_O1[1] - tmp_coeff_O2[1]) < thres) && (abs(tmp_coeff_O1[2] - tmp_coeff_O2[2]) < thres))// && (abs(tmp_coeff_O1[3] - tmp_coeff_O2[3]) < thres))
    {
      if(_pub_Results)
      cout << "Not a new object" << endl;
      isNew = false;
    }
    return isNew;
  }

//  bool object3D::checkGroundPlane()
//  {
//    //TODO: Find better way
//    //cout << "ground plane check" << endl;
//
//    bool ground = false;
//    /*
//     * if z-axis of center point is the main direction and neg.
//     */
//    double* center = new double[3];
//    double* corners = new double[12];
//    double* coeff = new double[4];
//
//    // get center point
//    center = this->getCentroid();
//    corners = this->getObjectCorners();
//
////    cout << "center" << endl;
////    cout << center[0] << " " << center[1] << " " <<  center[2] <<  endl;
////    cout << "coeff" << endl;
////    cout << coeff[0] << " " << coeff[1] << " " <<  coeff[2] << " " << coeff[3] << endl;
////
////    cout << "corners" << endl;
////    cout << corners[0] << " " << corners[1] << " " <<  corners[2] <<  endl;
////    cout << corners[3] << " " << corners[4] << " " <<  corners[5] <<  endl;
////    cout << corners[6] << " " << corners[7] << " " <<  corners[8] <<  endl;
////    cout << corners[9] << " " << corners[10] << " " <<  corners[11] <<  endl;
//
//    // get the normal vector
//    coeff = this->getCoefficents();
//
//    // check if the z is neg. and the length of the z-normal is bigger than x and y
//    if((abs(coeff[2]) > 0.8))
//    {
//      cout << "ground!!" << endl;
//      ground = true;
//    }
//    //cout << "end ground plane check" << endl;
//
//    return ground;
//
//
//  }


  bool object3D::analyzeObjectType(double* coords1, double* coords2, unsigned int* intens1, unsigned int* intens2, int scanSize, double thres_dist, double thres_visionCone, double maxDist, double maxRot, double fitnessICP, double thres_mean, unsigned int normIntensWhPaper_rise, unsigned int normIntensWhPaper_offset, double amVarFactor, double thres_VariationInt)
   {
    //TODO: CONTINIUE, not checked why only a few intensity values are available. Already the subscriber has only a few values
     object3D::objType objectType = object3D::unchecked;

     /*
      * results of check
      */
     int check_meanIntens       = 0;
     int check_transformation   = 0;
     int check_intensVariation  = 0;
//     double check_phongCurve      = 0;
//     double check_Intensity2Paper = 0;
     int check_final            = 0;

     /*
      * resorted to work with correct data
      */
     int amount_valid    = 0;
     int amount_surface  = 0;
     int amount_error    = 0;

     double* tmp_coords_valid           = new double[3*scanSize];

     //double* tmp_coords_surface;
     double* tmp_coords_surface1        = new double[3*scanSize];
     double* tmp_coords_surface2        = new double[3*scanSize];

     //double* tmp_coords_error;
     double* tmp_coords_error1          = new double[3*scanSize];
     double* tmp_coords_error2          = new double[3*scanSize];

     unsigned int* tmp_intens_valid1    = new unsigned int[scanSize];
     unsigned int* tmp_intens_valid2    = new unsigned int[scanSize];

     //unsigned int* tmp_intens_surface;
     unsigned int* tmp_intens_surface1  = new unsigned int[scanSize];
     unsigned int* tmp_intens_surface2  = new unsigned int[scanSize];

     //unsigned int* tmp_intens_error;
     unsigned int* tmp_intens_error1    = new unsigned int[scanSize];
     unsigned int* tmp_intens_error2    = new unsigned int[scanSize];

     object3D::objType* maskObjectType1 = new object3D::objType[scanSize];
     object3D::objType* maskObjectType2 = new object3D::objType[scanSize];

     for(int i=0; i < scanSize; i++)
     {
       maskObjectType1[i] = object3D::unchecked;
       maskObjectType2[i] = object3D::unchecked;
     }

     int* locationPoint1 = new int[scanSize];
     int* locationPoint2 = new int[scanSize];

     /*
      * clean scan
      * mask points belong to the object and are affected from the object
      */
       pointLocation(coords1, scanSize, this->getObjectCorners(), this->getCoefficents(), thres_dist, thres_visionCone, locationPoint1);
       pointLocation(coords2, scanSize, this->getObjectCorners(), this->getCoefficents(), thres_dist, thres_visionCone, locationPoint2);

       /*
        * assign objectType of point
        */
       for(int i=0; i < scanSize; i++)
       {
         /*
          * assign objectType1
          */
         if(locationPoint1[i] == 0)
         {
           maskObjectType1[i] = object3D::validPoint;  // valid point
           amount_valid++;
         }
         else if(locationPoint1[i] == 1)
         {
           maskObjectType1[i] = object3D::errorSurface;   // other points
           amount_surface++;
         }
         else if(locationPoint1[i] == 2)
         {
           maskObjectType1[i] = object3D::behindErrorS;   // other points
           amount_error++;
         }
         else
         {
           cout << "ERROR: Object type identification" << endl;
         }
         /*
          * assign objectType2
          */
         if(locationPoint2[i] == 0)
         {
           maskObjectType2[i] = object3D::validPoint;  // valid point
         }
         else if(locationPoint2[i] == 1)
         {
           maskObjectType2[i] = object3D::errorSurface;   // other points
           amount_surface++;
         }
         else if(locationPoint2[i] == 2)
         {
           maskObjectType2[i] = object3D::behindErrorS;   // other points
           amount_error++;
         }
         else
         {
           cout << "ERROR: Object type identification" << endl;
         }
       }

       double* tmp_coords_surface = new double[3*amount_surface];
       unsigned int* tmp_intens_surface = new unsigned int[amount_surface];

       double* tmp_coords_error   = new double[3*amount_error];
       unsigned int* tmp_intens_error   = new unsigned int[amount_error];

       /*
        * copy together points according to their object type
        */
       for(int i=0; i < scanSize; i++)
       {
         /*
          * check for valid points
          */
         if(maskObjectType1[i] == object3D::validPoint)
         {
           tmp_coords_valid[3*i]    = coords1[3*i];
           tmp_coords_valid[3*i+1]  = coords1[3*i+1];
           tmp_coords_valid[3*i+2]  = coords1[3*i+2];

           tmp_intens_valid1[i]     = intens1[i];
           tmp_intens_valid2[i]     = intens2[i];

           tmp_coords_surface1[3*i] = NAN;
           tmp_coords_surface1[3*i] = NAN;
           tmp_coords_surface1[3*i] = NAN;

           tmp_coords_surface2[3*i] = NAN;
           tmp_coords_surface2[3*i] = NAN;
           tmp_coords_surface2[3*i] = NAN;

           tmp_intens_surface1[i]   = 0;
           tmp_intens_surface2[i]   = 0;

           tmp_coords_error1[3*i]   = NAN;
           tmp_coords_error1[3*i+1] = NAN;
           tmp_coords_error1[3*i+2] = NAN;

           tmp_coords_error2[3*i]   = NAN;
           tmp_coords_error2[3*i+1] = NAN;
           tmp_coords_error2[3*i+2] = NAN;

           tmp_intens_error1[i]     = 0;
           tmp_intens_error2[i]     = 0;
         }
         /*
          * check for surface points
          */
         if((maskObjectType1[i] == object3D::errorSurface) or (maskObjectType2[i] == object3D::errorSurface))
         {
           tmp_coords_valid[3*i]   = NAN;
           tmp_coords_valid[3*i+1] = NAN;
           tmp_coords_valid[3*i+2] = NAN;

           tmp_intens_valid1[i]    = 0;
           tmp_intens_valid2[i]    = 0;

           if(maskObjectType1[i] == object3D::errorSurface)
           {
             tmp_coords_surface1[3*i]   = coords1[3*i];
             tmp_coords_surface1[3*i+1] = coords1[3*i+1];
             tmp_coords_surface1[3*i+2] = coords1[3*i+2];

             tmp_coords_surface2[3*i] = NAN;
             tmp_coords_surface2[3*i] = NAN;
             tmp_coords_surface2[3*i] = NAN;

             tmp_intens_surface1[i]     = intens1[i];
             tmp_intens_surface2[i]     = 0;
           }
           else
           {
             tmp_coords_surface1[3*i] = NAN;
             tmp_coords_surface1[3*i] = NAN;
             tmp_coords_surface1[3*i] = NAN;

             tmp_coords_surface2[3*i]   = coords2[3*i];
             tmp_coords_surface2[3*i+1] = coords2[3*i+1];
             tmp_coords_surface2[3*i+2] = coords2[3*i+2];

             tmp_intens_surface1[i]     = 0;
             tmp_intens_surface2[i]     = intens2[i];
           }

           tmp_coords_error1[3*i]   = NAN;
           tmp_coords_error1[3*i+1] = NAN;
           tmp_coords_error1[3*i+2] = NAN;

           tmp_coords_error2[3*i]   = NAN;
           tmp_coords_error2[3*i+1] = NAN;
           tmp_coords_error2[3*i+2] = NAN;

           tmp_intens_error1[i]     = 0;
           tmp_intens_error2[i]     = 0;
         }
         /*
          * check for error points
          */
         if((maskObjectType1[i] == object3D::behindErrorS) or (maskObjectType2[i] == object3D::behindErrorS))
         {
           tmp_coords_valid[3*i]   = NAN;
           tmp_coords_valid[3*i+1] = NAN;
           tmp_coords_valid[3*i+2] = NAN;

           tmp_intens_valid1[i]    = 0;
           tmp_intens_valid2[i]    = 0;

           tmp_coords_surface1[3*i] = NAN;
           tmp_coords_surface1[3*i] = NAN;
           tmp_coords_surface1[3*i] = NAN;

           tmp_coords_surface2[3*i] = NAN;
           tmp_coords_surface2[3*i] = NAN;
           tmp_coords_surface2[3*i] = NAN;

           tmp_intens_surface1[i]   = 0;
           tmp_intens_surface2[i]   = 0;

           if(maskObjectType1[i] == object3D::behindErrorS)
           {
             tmp_coords_error1[3*i]   = coords1[3*i];
             tmp_coords_error1[3*i+1] = coords1[3*i+1];
             tmp_coords_error1[3*i+2] = coords1[3*i+2];

             tmp_coords_error2[3*i]   = NAN;
             tmp_coords_error2[3*i+1] = NAN;
             tmp_coords_error2[3*i+2] = NAN;

             tmp_intens_error1[i]     = intens1[i];
             tmp_intens_error2[i]     = 0;
           }
           else
           {
             tmp_coords_error1[3*i]   = NAN;
             tmp_coords_error1[3*i+1] = NAN;
             tmp_coords_error1[3*i+2] = NAN;

             tmp_coords_error2[3*i]   = coords2[3*i];
             tmp_coords_error2[3*i+1] = coords2[3*i+1];
             tmp_coords_error2[3*i+2] = coords2[3*i+2];

             tmp_intens_error1[i]     = 0;
             tmp_intens_error2[i]     = intens2[i];
           }
         }
       }

       int j = 0;
       int n = 0;
       for(int i=0; i < scanSize; i++)
       {
         if(maskObjectType1[i] == object3D::errorSurface)
         {
           tmp_coords_surface[3*j]   = coords1[3*i];
           tmp_coords_surface[3*j+1] = coords1[3*i+1];
           tmp_coords_surface[3*j+2] = coords1[3*i+2];

           tmp_intens_surface[j]     = intens1[i];
           j++;
         }
//         if(maskObjectType2[i] == object3D::errorSurface)
//         {
//           tmp_coords_surface[3*j]   = coords2[3*i];
//           tmp_coords_surface[3*j+1] = coords2[3*i+1];
//           tmp_coords_surface[3*j+2] = coords2[3*i+2];
//
//           tmp_intens_surface[j]     = intens2[i];
//           j++;
//         }

         if(maskObjectType1[i] == object3D::behindErrorS)
         {
           tmp_coords_error[3*n]   = coords1[3*i];
           tmp_coords_error[3*n+1] = coords1[3*i+1];
           tmp_coords_error[3*n+2] = coords1[3*i+2];

           tmp_intens_error[n]     = intens1[i];
           //cout << intens1[i] << "/";

           n++;
         }
         if(maskObjectType2[i] == object3D::behindErrorS)
         {
           tmp_coords_error[3*n]   = coords2[3*i];
           tmp_coords_error[3*n+1] = coords2[3*i+1];
           tmp_coords_error[3*n+2] = coords2[3*i+2];

           tmp_intens_error[n]     = intens2[i];
           n++;
         }
       }
       //cout << endl;


//       //TOTEST
//       printCoords2File(coords1, scanSize, "/home/rainer/workspace/coords1.txt");
//       printCoords2File(coords2, scanSize, "/home/rainer/workspace/coords2.txt");
//       printCoords2File(tmp_coords_valid, scanSize, "/home/rainer/workspace/valid.txt");
//       printCoords2File(tmp_coords_surface, amount_surface, "/home/rainer/workspace/surface.txt");
//       printCoords2File(tmp_coords_surface1, scanSize, "/home/rainer/workspace/surface1.txt");
//       printCoords2File(tmp_coords_surface2, scanSize, "/home/rainer/workspace/surface2.txt");
//       printCoords2File(tmp_coords_error, amount_error, "/home/rainer/workspace/error.txt");
//       printCoords2File(tmp_coords_error1, scanSize, "/home/rainer/workspace/error1.txt");
//       printCoords2File(tmp_coords_error2, scanSize, "/home/rainer/workspace/error2.txt");
//
//       printIntens2File(intens1, scanSize, "/home/rainer/workspace/int_1.txt");
//       printIntens2File(intens2, scanSize, "/home/rainer/workspace/int_2.txt");
//       printIntens2File(tmp_intens_surface, amount_surface, "/home/rainer/workspace/int_s.txt");
//       printIntens2File(tmp_intens_surface1, scanSize, "/home/rainer/workspace/int_s1.txt");
//       printIntens2File(tmp_intens_surface2, scanSize, "/home/rainer/workspace/int_s2.txt");
//       printIntens2File(tmp_intens_error, amount_error, "/home/rainer/workspace/int_e.txt");
//       printIntens2File(tmp_intens_error1, scanSize, "/home/rainer/workspace/int_e1.txt");
//       printIntens2File(tmp_intens_error2, scanSize, "/home/rainer/workspace/int_e2.txt");
//       cout << "print files done" << endl;

//         cout << "Size object: " << this->getMajorDim() << "x" << this->getMinorDim() << endl;


     /*
      * Check 1: Median Intensity Factor
      */
//       cout << "check 1 : mean Intens" << endl;
//       /*
//        * ToShow
//        */
//       cout << "check 1 : start Intens" << endl;
//       // Start to print the intensities
//       cout << "points on surface: " << amount_surface << endl;
//       cout << "Centroid:" << endl;
//       cout << this->_objectCentroid[0] << " " << this->_objectCentroid[1] << " " << this->_objectCentroid[2] << endl;
//       cout << "d = " << (sqrt(_objectCentroid[0]*_objectCentroid[0]+_objectCentroid[1]*_objectCentroid[1]+_objectCentroid[2]*_objectCentroid[2])) << endl;
//
//
//       printCoords2File(coords1, scanSize, "/home/rainer/workspace/coords1.txt");
//       printCoords2File(this->_objectCorners, 4, "/home/rainer/workspace/surf_corners.txt");
//       printIntens2File(tmp_intens_surface, amount_surface, "/home/rainer/workspace/surf_intens.txt");
//       printCoords2File(tmp_coords_surface, amount_surface, "/home/rainer/workspace/surf_coords.txt");
//       printIntens2PlaneFile(tmp_coords_surface, tmp_intens_surface, amount_surface, this->_objectCentroid, "/home/rainer/workspace/surf_flat_intens.txt");
       check_meanIntens = meanIntensFactorCheck(tmp_intens_surface, tmp_intens_error, amount_surface, amount_error, thres_mean);
       if(_pub_Results)
       {
         if(check_meanIntens == 0)
         {
           cout << " check 1: Mean Intensity Factor -> transparent " << endl;
         }
         else
         {
           cout << " check 1: Mean Intensity Factor -> mirror " << endl;
         }
       }
       //cout << " Result Check Mean Intens: " << check_meanIntens << endl;

     /*
      * Check 2: reflected points by ICP
      */
//       cout << "check 2: start ICP" << endl;
       Eigen::Matrix4f Trans = Eigen::Matrix4f::Identity();
       //cout << Trans << endl;
       check_transformation = transformationCheck(coords1, tmp_coords_error, scanSize, amount_error, this->_objectCoefficients, maxDist, maxRot, fitnessICP, &Trans);
       if(_pub_Results)
       {
         if(check_transformation == 0)
         {
           cout << " check 2: ICP -> transparent " << endl;
         }
         else
         {
           cout << " check 2: ICP -> mirror " << endl;
         }
       }
       //cout << Trans << endl;
       //cout << "Result Check Transformation: " << check_transformation << endl;
       //cout << endl; // without cout the program crashes, don`t ask me why

       /*
        * Check 3: if intensities of echo 2 vary or be stable
        *
        */
        //cout << "check 3: IntensityVariation" << endl;
        check_intensVariation = intensVariation(tmp_intens_error,amount_error, amVarFactor, thres_VariationInt);
        if(_pub_Results)
        {
          if(check_intensVariation == 0)
          {
            cout << " check 3: IntensityVariation -> transparent " << endl;
          }
          else
          {
            cout << " check 3: IntensityVariation -> mirror " << endl;
          }
        }
        //cout << "Result Check phongCurve: " << check_intensVariation << endl;

     /*
      * Check 3: if a phong curve can be found
      *
      */
//       cout << "check 2: phongCurve" << endl;
//       check_phongCurve = phongCurveCheck(tmp_coords_surface, tmp_coords_error, tmp_intens_surface, tmp_intens_error, amount_surface, amount_error, this->_objectCentroid);
//       cout << " Result Check phongCurve: " << check_phongCurve << endl;

     /*
      * Check 4: amount of intensity for 1m and compare to white paper
      *
      */
       //check_Intensity2Paper = intensity2PaperCheck(tmp_coords_surface, tmp_intens_surface, amount_surface, this->_objectCentroid, normIntensWhPaper_rise, normIntensWhPaper_offset);


      /*
       * compare results of different checks
       */
        check_final = evaluateResults(check_meanIntens, check_transformation, check_intensVariation);
        if(_pub_Results)
        {
          if(check_final == 0)
          {
            cout << " final check -> transparent " << endl;
          }
          else
          {
            cout << " final check -> mirror " << endl;
          }
        }
        //cout << "Final: " << check_final << endl;
       if(check_final == 1)
       {
         objectType = object3D::reflectiveSurface;
         /*
          * set transformation matrix to back-project reflected points
          */
         this->setTransformation(Trans);
       }
       else if(check_final == 0)
       {
         objectType = object3D::transpSurface;
         /*
          * no transformation available, because of transparent surface
          */
         this->setTransformation(Eigen::Matrix4f::Identity());
       }
       else
       {
         objectType = object3D::unchecked;
       }
     this->setObjectType(objectType);

     return true;
   }


  /* END CLASS */
}
