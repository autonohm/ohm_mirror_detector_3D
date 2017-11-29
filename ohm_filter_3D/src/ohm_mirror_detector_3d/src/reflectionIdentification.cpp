/**
* @file   reflectionIdentification.cpp
* @author Rainer Koch
* @date   11.01.2017
*
*
*/
#include "reflectionIdentification.h"

// To test
#include "testFunctions.h"

  int meanIntensFactorCheck(unsigned int* intens_plane, unsigned int* intens_error, int size_plane, int size_error, double thres_mean)
  {
    /*
     * TOSHOW
     */
    // Start to print the intensities
//    cout << "points on surface: " << size_plane << endl;
//    cout << "points on error: " << size_error << endl;
//    printIntens2File(intens_plane, size_plane, "/home/rainer/workspace/intens_check1.txt");
//    printIntens2File(intens_error, size_error, "/home/rainer/workspace/intens_check2.txt");
    // END TOSHOW


    double mean_plane_intens  = meanIntens(intens_plane, size_plane);
    double mean_error_intens  = meanIntens(intens_error, size_error);
    double mean_factor        = 0;
 //   cout << "mean plan " << mean_plane_intens << endl;
 //   cout << "mean error " << mean_error_intens << endl;

    /*
     * create factor
     */
    mean_factor = mean_error_intens / mean_plane_intens;

    //cout << "Factor: " << mean_factor << "/" << thres_mean << endl;

    if(mean_factor > thres_mean)
    {
      //cout << " check 1 -> transparent " << endl;// greatIntensFactor << "/" << amountOfAffectScans << endl;

      return 0;       // transparent
    }
    else
    {
      //cout << " check 1 -> mirror " << endl;//greatIntensFactor << "/" << amountOfAffectScans << endl;
      return 1;      // mirror
    }

  }

  int transformationCheck(double* coords, double* coords_error, int scanSize, int errorSize, double* plane_coefficients, double maxDist, double maxRot, double fitnessICP, Eigen::Matrix4f* Trans)
  {
    //TOTEST:
    double* tmp_coords_valid = new double[3*scanSize];
    double* tmp_coords_error = new double[3*errorSize];

    for(int i=0; i < scanSize; i++)
    {
      tmp_coords_valid[3*i]   = coords[3*i];
      tmp_coords_valid[3*i+1] = coords[3*i+1];
      tmp_coords_valid[3*i+2] = coords[3*i+2];
    }
    for(int i=0; i < errorSize; i++)
    {
      tmp_coords_error[3*i]   = coords_error[3*i];
      tmp_coords_error[3*i+1] = coords_error[3*i+1];
      tmp_coords_error[3*i+2] = coords_error[3*i+2];
     }
    // End TOTEST


    /*
     * mirror points which are located behind the plane
     */
    double* mirrored_coords = new double[3*errorSize];
    mirrorOnPlane(coords_error, mirrored_coords, plane_coefficients, errorSize);

    /*
     * check for valid transformation with ICP
     * http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_coords (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mirrored (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result (new pcl::PointCloud<pcl::PointXYZ>);

    cloud_coords->points.resize (scanSize);
    cloud_mirrored->points.resize (errorSize);
    //cloud_result->points.resize (scanSize + amount_error);

    for(int i=0; i < scanSize; i++)
    {
      cloud_coords->points[i].x = coords[3*i];
      cloud_coords->points[i].y = coords[3*i+1];
      cloud_coords->points[i].z = coords[3*i+2];
    }
    for(int i=0; i < errorSize; i++)
    {
      cloud_mirrored->points[i].x = mirrored_coords[3*i];
      cloud_mirrored->points[i].y = mirrored_coords[3*i+1];
      cloud_mirrored->points[i].z = mirrored_coords[3*i+2];
     }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_mirrored);
    icp.setInputTarget(cloud_coords);
    pcl::PointCloud<pcl::PointXYZ> cloud_result;
    icp.align(cloud_result);

    Eigen::Matrix4f tmp_Trans;
    tmp_Trans = icp.getFinalTransformation();

    Eigen::Affine3f tmp_Tr;
    tmp_Tr(0,0)= tmp_Trans(0,0);
    tmp_Tr(0,1)= tmp_Trans(0,1);
    tmp_Tr(0,2)= tmp_Trans(0,2);
    tmp_Tr(0,3)= tmp_Trans(0,3);

    tmp_Tr(1,0)= tmp_Trans(1,0);
    tmp_Tr(1,1)= tmp_Trans(1,1);
    tmp_Tr(1,2)= tmp_Trans(1,2);
    tmp_Tr(1,3)= tmp_Trans(1,3);

    tmp_Tr(2,0)= tmp_Trans(2,0);
    tmp_Tr(2,1)= tmp_Trans(2,1);
    tmp_Tr(2,2)= tmp_Trans(2,2);
    tmp_Tr(2,3)= tmp_Trans(2,3);

    tmp_Tr(3,0)= tmp_Trans(3,0);
    tmp_Tr(3,1)= tmp_Trans(3,1);
    tmp_Tr(3,2)= tmp_Trans(3,2);
    tmp_Tr(3,3)= tmp_Trans(3,3);


//cout << tmp_Tr(0,0) << "/" << tmp_Tr(0,1) << "/" << tmp_Tr(0,2) << "/" << tmp_Tr(0,3) << endl;
//cout << tmp_Tr(1,0) << "/" << tmp_Tr(1,1) << "/" << tmp_Tr(1,2) << "/" << tmp_Tr(1,3) << endl;
//cout << tmp_Tr(2,0) << "/" << tmp_Tr(2,1) << "/" << tmp_Tr(2,2) << "/" << tmp_Tr(2,3) << endl;
//cout << tmp_Tr(3,0) << "/" << tmp_Tr(3,1) << "/" << tmp_Tr(3,2) << "/" << tmp_Tr(3,3) << endl;

    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    float dx = 0;
    float dy = 0;
    float dz = 0;


    //TOTEST:
//    printCoords2File(coords, scanSize, "/home/rainer/workspace/coords_valid.txt");
//    printCoords2File(coords_error, errorSize, "/home/rainer/workspace/coords_error.txt");
//    printCoords2File(mirrored_coords, errorSize, "/home/rainer/workspace/coords_mirrored.txt");
//         std::cout << "has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << "/" << fitnessICP << std::endl;
//         cout << endl;


//         std::cout << std::endl;



//         cout << "cloud: " <<  cloud_coords->points.size() << endl;
//         cout << "mirr cloud: " <<  cloud_mirrored->points.size() << endl;
    // End TOTEST

    *Trans = Eigen::Matrix4f::Identity();
    //cout << tmp_Trans(0,3) << "/" << tmp_Trans(1,3) << "/" << tmp_Trans(2,3) << "/" << maxDist << endl;


    if(icp.hasConverged())// && (icp.getFitnessScore() <= fitnessICP))
    {
      pcl::getTranslationAndEulerAngles(tmp_Tr, dx, dy, dz, roll, pitch, yaw);
      //         std::cout << icp.getFinalTransformation() << std::endl;
               //std::cout << "R= " << roll << ", P= " << pitch << ", Y= " << yaw << ", dx= " << dx << ", dy= " << dy << ", dz= " << dz << std::endl;
//      std::cout << "R= " << roll/3.14*180 << ", P= " << pitch/3.14*180 << ", Y= " << yaw/3.14*180 << ", dx= " << dx << ", dy= " << dy << ", dz= " << dz << std::endl;
      /*
       * check max rotation and distance
       */
      // check dist or Transformation matrix
      //cout << "max Dist/Rot: " << maxDist << "/" << maxRot << endl;

      if((abs(dx) <= maxDist) && (abs(dy) <= maxDist) && (abs(dz) <= maxDist))// && (abs(roll) <= maxRot) && (abs(pitch) <= maxRot) && (abs(yaw) <= maxRot))
      {
        //TODO: Include rotation check
//        double beta  = atan2(-r[2][0], sqrt(Math.pow(r[0][0], 2) + pow(r[1][0], 2)));
//        double alpha = atan2(r[1][0]/cos(beta), r[0][0]/cos(beta));
//        double gamma = atan2(r[2][1]/cos(beta), r[2][2]/cos(beta));
//
//
//
//
//        // check rotation or Transformation matrix
//        if((alpha < maxRot) && (beta < maxRot) && (gamma < maxRot))
//        {
//        }
          /*
           * found a transformation => reflective object
           */
          *Trans = icp.getFinalTransformation();

//          cout << *Trans << endl;
          //cout << " check 2 -> mirror " << endl;
          return 1;       // mirror
        }
      }
    /*
     * found no transformation => transparent object
     */
    //cout << " check 2 -> transparent " << endl;
    return 0;   // transparent

  }

  int intensVariation(unsigned int* intens_error, int size_error,  double amVarFactor, double thres_VariationInt)
  {
    unsigned int median = 0;
    double varianz = 0.0;
    /*
     * check if intensity values of echo 2 build a line or have a high variation
     * if high variation -> reflective, if line -> transparent
     *
     */

    /*
     * TOSHOW
     */
//    cout << "points on error: " << size_error << endl;
    //printIntens2File(intens_error, size_error, "/home/rainer/workspace/surf_intens_error.txt");


//    /*
//     * get min/max-values
//     */
//    unsigned int min_error    = std::numeric_limits<unsigned int>::max();
//    unsigned int max_error    = 0;
//    unsigned int arithmetic_mean = 0;
////    unsigned int min_plane = std::numeric_limits<unsigned int>::max();
////    unsigned int max_plane = 0;
//
//    for(int i = 0; i < size_error; i++)
//    {
//      if(intens_error[i] < min_error)
//        min_error = intens_error[i];
//      if(intens_error[i] > max_error)
//        max_error = intens_error[i];
//    }
//    cout << "min/max/median intensity error: " << min_error << " / " << max_error << endl;

    median =  medianIntens(intens_error, size_error);
//    cout << "median:" << median << endl;
    varianz = amVarIntens(intens_error, size_error, median, amVarFactor);

//    cout << "varianz: " << varianz << endl;
//    cout << "thres varianz: " << thres_VariationInt << endl;

    if(varianz < thres_VariationInt)
    {
      /*
       * found a transformation => reflective object
       */
     // cout << " check 3 -> mirror " << endl;
      return 1;       // mirror
    }
    else
    {
    /*
     * found no transformation => transparent object
     */
     // cout << " check 3 -> transparent " << endl;
      return 0;   // transparent
    }

  }

  int evaluateResults(int meanIntensResult, int transformationResult, int intensVariationResult)
  {
    double result = 0;
    result = (meanIntensResult + transformationResult + intensVariationResult) / 3.0;

    if(result >= 0.5)
    {
     // cout << "Final result mirror" << endl;
      return 1;      // mirror
    }
    else
    {
     // cout << "Final result transparent" << endl;
      return 0;      // transparent
    }
  }


  int phongCurveCheck(double* coords, double* coords2, unsigned int* intens_plane, unsigned int* intens_error, int size_plane, int size_error, double* centroid)
  {

//    /*
//     * check if intensity values build a phong curve or have a high variation
//     *
//     */
//
//    /*
//     * TOSHOW
//     */
//    // Start to print the intensities
//    cout << "points on surface: " << size_plane << endl;
//    cout << "points on error: " << size_error << endl;
//    cout << "Centroid:" << endl;
//    cout << centroid[0] << " " << centroid[1] << " " << centroid[2] << endl;
//    cout << "d = " << (sqrt(centroid[0]*centroid[0]+centroid[1]*centroid[1]+centroid[2]*centroid[2])) << endl;
//    printCoords2File(coords, size_plane, "/home/rainer/workspace/coords1.txt");
//    printCoords2File(coords2, size_error, "/home/rainer/workspace/coords2.txt");
////    printCoords2File(this->_objectCorners, 4, "/home/rainer/workspace/surf_corners.txt");
//    printIntens2File(intens_plane, size_plane, "/home/rainer/workspace/surf_intens1.txt");
//    printIntens2File(intens_error, size_error, "/home/rainer/workspace/surf_intens2.txt");
//    printCoords2File(coords, size_plane, "/home/rainer/workspace/surf_coords.txt");
//    // END TOSHOW
//
//
//    /*
//     * TOSHOW:
//     */
//    double * tmp_coords;
//    tmp_coords = new double[size_plane];
//    tmp_coords = coords;
//    for(int i = 0; i < size_plane; i++)
//    {
//      if((!isnan(intens_plane[i])) and (intens_plane[i] < 10000) and (intens_plane[i] > 0) and (!isnan(tmp_coords[3*i])) and (!isnan(tmp_coords[3*i+1])) and (!isnan(tmp_coords[3*i+2])))
//      {
//        /*
//         * transfer surface coords into xy-plane
//         */
//         tmp_coords[3*i] = coords[3*i] - centroid[0];
//         tmp_coords[3*i+1] = coords[3*i+1] - centroid[1];
//         tmp_coords[3*i+2] = coords[3*i+2] - centroid[2];
//
//      }
//    }
//    printIntens2PlaneFile(tmp_coords, intens_plane, size_plane, centroid, "/home/rainer/workspace/surf_flat_intens1.txt");
//    printIntens2PlaneFile(tmp_coords, intens_error, size_error, centroid, "/home/rainer/workspace/surf_flat_intens2.txt");
//    // END TOSHOW
//





//    /*
//     * ToShow
//     */
//    // Start to print the intensities
//    cout << "points on surface: " << size << endl;
//    cout << "Centroid:" << endl;
//    cout << centroid[0] << " " << centroid[1] << " " << centroid[2] << endl;
//    cout << "d = " << (sqrt(centroid[0]*centroid[0]+centroid[1]*centroid[1]+centroid[2]*centroid[2])) << endl;
//    printCoords2File(coords, size, "/home/rainer/workspace/coords1.txt");
////    printCoords2File(this->_objectCorners, 4, "/home/rainer/workspace/surf_corners.txt");
//    printIntens2File(intens, size, "/home/rainer/workspace/surf_intens.txt");
//    printCoords2File(coords, size, "/home/rainer/workspace/surf_coords.txt");
//
//    double * tmp_coords;
//    tmp_coords = new double[size];
//    tmp_coords = coords;
//
//    for(int i = 0; i < size; i++)
//    {
//      if((!isnan(intens[i])) and (intens[i] < 10000) and (intens[i] > 0) and (!isnan(tmp_coords[3*i])) and (!isnan(tmp_coords[3*i+1])) and (!isnan(tmp_coords[3*i+2])))
//      {
//        /*
//         * transfer surface coords into xy-plane
//         */
//         tmp_coords[3*i] = coords[3*i] - centroid[0];
//         tmp_coords[3*i+1] = coords[3*i+1] - centroid[1];
//         tmp_coords[3*i+2] = coords[3*i+2] - centroid[2];
//
//      }
//    }
//
//
//    printIntens2PlaneFile(tmp_coords, intens, size, centroid, "/home/rainer/workspace/surf_flat_intens.txt");

  }

  int intensity2PaperCheck(double* coords, unsigned int* intens, int size, double* centroid, unsigned int normIntensWhPaper_rise, unsigned int normIntensWhPaper_offset)
  {
    /*
     * see book 15.1.
     */
    unsigned int int_greater_wh = 0;
    unsigned int int_smaller_wh = 0;

    double* dist_center2plane       = new double[size];
    double* angles                  = new double [3];
    double* beta                    = new double[size];
    unsigned int* intens_resp_angle = new unsigned int[size];
    unsigned int* normed_intens_1m  = new unsigned int[size];
    unsigned int intens_paper_wh = 0;

    //TOCONTINIUE:
    for(int i=0; i < size; i++)
    {
      /*
       * calculate distance and angles origin scanner to  plane
       */
      dist_center2plane[i] = calcVectorLength(&coords[i]);
      angles               = calcAngleVectorAxes(&coords[i]);

      /*
       * respect angle dependency
       *  if distance to point > 1.5 m -> angle does matter
       *  if distance to point < 2m -> angle does not matter
       */
      //intens_resp_angle[i] =

      /*
       * calculate intensity of white paper at distance of point
       */
      intens_paper_wh = normIntensWhPaper_rise / ( dist_center2plane[i] * dist_center2plane[i]) + normIntensWhPaper_offset;

      /*
       * count amount of intensity points greater than white paper and amount of intensity points smaller than white paper
       */
      if(intens_paper_wh >= intens_resp_angle[i])
      {
        int_greater_wh++;
      }
      else if (intens_paper_wh < intens_resp_angle[i])
      {
        int_smaller_wh++;
      }

      /*
       * alternative check variation of points
       */
    }

    /*
     * compare values
     */





//    /*
//     * ToShow
//     */
//    // Start to print the intensities
//    cout << "points on surface: " << size << endl;
//    cout << "Centroid:" << endl;
//    cout << centroid[0] << " " << centroid[1] << " " << centroid[2] << endl;
//    cout << "d = " << (sqrt(centroid[0]*centroid[0]+centroid[1]*centroid[1]+centroid[2]*centroid[2])) << endl;
//    printCoords2File(coords, size, "/home/rainer/workspace/coords1.txt");
////    printCoords2File(this->_objectCorners, 4, "/home/rainer/workspace/surf_corners.txt");
//    printIntens2File(intens, size, "/home/rainer/workspace/surf_intens.txt");
//    printCoords2File(coords, size, "/home/rainer/workspace/surf_coords.txt");
//
    //    double * tmp_coords;
    //    tmp_coords = new double[size];
    //    tmp_coords = coords;
    //
    //    for(int i = 0; i < size; i++)
    //    {
    //      if((!isnan(intens[i])) and (intens[i] < 10000) and (intens[i] > 0) and (!isnan(tmp_coords[3*i])) and (!isnan(tmp_coords[3*i+1])) and (!isnan(tmp_coords[3*i+2])))
    //      {
    //        /*
    //         * transfer surface coords into xy-plane
    //         */
    //         tmp_coords[3*i] = coords[3*i] - centroid[0];
    //         tmp_coords[3*i+1] = coords[3*i+1] - centroid[1];
    //         tmp_coords[3*i+2] = coords[3*i+2] - centroid[2];
    //
    //      }
    //    }
    //
    //
    //    printIntens2PlaneFile(tmp_coords, intens, size, centroid, "/home/rainer/workspace/surf_flat_intens.txt");

 }

  unsigned int medianIntens(unsigned int* intens, int size)
  {
    unsigned int *newptr[size];
    unsigned int median = 0;
    for(int i = 0; i<size; i++)
    {
        newptr[i] = &intens[i];
    }
//    for(int i = 0; i<size; i++)
//    {
//        cout << intens[i] << " ";
//
//    }
//    cout << endl;
   //cout << size << endl;

    //sort pointer array
   for(int j = 0; j<(size-1); j++)
   {
       for(; j > -1 && *newptr[j] < *newptr[j+1]; j--)
       {
         //cout << size << " " << j  << "/" << size << endl;
         //cout << size << " " << j << *newptr[j] << endl;
         //cout << size << " " << j << *newptr[j] << " " << *newptr[j+1] << endl;

           unsigned int *temp = newptr[j+1];
           newptr[j+1] = newptr[j];
           newptr[j] = temp;
       }
   }

   // check if size is even or not
   if(((size/2) - int (size/2)) == 0)
   {
     median = *newptr[size/2+1];
   }
   else
   {
     median = (*newptr[int(size/2)] + *newptr[int(size/2)]) / 2;
   }
//   for(int i = 0; i<size; i++)
//   {
//       cout << *newptr[i] << " ";
//
//   }
//   cout << endl;

   //cout << "median: " << median << endl;
   return median;
  }

  double meanIntens(unsigned int* intens, int size)
  {
    unsigned int mean = 0;
    for(int i = 0; i < size; i++)
    {
    //  cout << intens[i] << "/";
      mean += intens[i];
    }
    //cout << endl;
    return (1.0*mean/size);
  }


  double amVarIntens(unsigned int* intens, int size, unsigned int median, double amVar_factor)
  {
    int count = 0;
    for(int i=0; i < size; i++)
    {
        if(double (abs(intens[i] - median)) < (amVar_factor* (double) median))
          count++;
    }

//    cout << "median*amVar = " << (amVar_factor*median) << endl;
//    cout << count << "points > " << median << " +-" << (amVar_factor*median) << " = " << (100*count/size) << "%" << endl;

    return (1.0*count/size);
  }
//
//  unsigned int arethmMeanIntens(unsigned int* intens, int size)
//  {
//    unsigned int arethmMean = 0;
//    for(int i=0; i < size; i++)
//    {
//      arethmMean += intens[i];
//    }
//    arethmMean = arethmMean / size;
//    cout << "arethmMean: " << arethmMean << endl;
//
//    return arethmMean;
//  }
//
//  double varianzIntens(unsigned int* intens, int size)
//  {
//    unsigned int arethmMean = 0;
//    for(int i=0; i < size; i++)
//    {
//      arethmMean += intens[i];
//    }
//    arethmMean = arethmMean / size;
//
//    //Varianz var
//    double var = 0;
//    for (int i = 0; i < size; i++)
//    {
//        var += (intens[i] - arethmMean) * (intens[i] - arethmMean);
//    }
//    var = sqrt(var / size);
//
//    cout << "Arethmetic Mean / Varianz: " << arethmMean << "/" << var << endl;
//
//    return var;
//  }
