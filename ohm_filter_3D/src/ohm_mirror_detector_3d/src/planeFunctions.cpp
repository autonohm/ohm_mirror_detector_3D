/**
* @file   planeFunctions.cpp
* @author Rainer Koch
* @date   30.03.2016
*
*
*/
#include "planeFunctions.h"

#include <pcl/filters/frustum_culling.h>
#include <pcl/common/transforms.h>
// To test
#include "testFunctions.h"

#define PI 3.14159265

struct data{
  double value;
  int index;
};

struct by_number {
    bool operator()(data const &left, data const &right) {
        return left.value < right.value;
    }
};


bool pointFrustumFilter(double* point, double* top_E, double* bottom_E, double* right_E, double* left_E, double thres_dist)
{
  /*
   * see book 31.3.16
   */
  bool inFrustum = false;
  double d_pointToPlane1;
  double d_pointToPlane2;
  double d_pointToPlane3;
  double d_pointToPlane4;

  /*
   * Check if point is between top_E and bottom_E
   */
  d_pointToPlane1    = distanceToPlane(top_E, point);
  d_pointToPlane2    = distanceToPlane(bottom_E, point);

  /*
   * checking the distance to the plane
   * instead of check the distance to the surface, check to the threshold
   */
  if((d_pointToPlane1 >= -thres_dist) && (d_pointToPlane2 <= (thres_dist))) // top/bottom
  {
    /*
     * Check if point is between left_E and right_E
     */
    d_pointToPlane3    = distanceToPlane(right_E,point);
    d_pointToPlane4    = distanceToPlane(left_E,point);
    if((d_pointToPlane4 >= -thres_dist) && (d_pointToPlane3 <= (thres_dist))) // left/right
      inFrustum = true;
  }

  return inFrustum;
}

void pointsInCone(double* points, int size, double* objectCorners, double thres_dist, double thres_visionCone, bool* mask)
{
  /*
   * see book 31.3.16
   */
  double* n1  = new double[3];
  double* n2  = new double[3];
  double* n3  = new double[3];
  double* n4  = new double[3];

  double* E1  = new double[4];
  double* E2  = new double[4];
  double* E3  = new double[4];
  double* E4  = new double[4];

  double* C1  = new double[3];
  double* C2  = new double[3];
  double* C3  = new double[3];
  double* C4  = new double[3];

  double* p   = new double[3];

  double d_E1toC4 = 0;
  double d_E2toC3 = 0;
  double d_E3toC2 = 0;
  double d_E4toC3 = 0;

  bool bottomE1     = false;
  bool leftE4   = false;

  C1[0]       = objectCorners[0];
  C1[1]       = objectCorners[1];
  C1[2]       = objectCorners[2];

  C2[0]       = objectCorners[3];
  C2[1]       = objectCorners[4];
  C2[2]       = objectCorners[5];

  C3[0]       = objectCorners[6];
  C3[1]       = objectCorners[7];
  C3[2]       = objectCorners[8];

  C4[0]       = objectCorners[9];
  C4[1]       = objectCorners[10];
  C4[2]       = objectCorners[11];

  /*
   * calculate normal vectors of E1, E2, E3 and E4
   * e.g. C1 is vector between origin and C1
   */
  n1 = calcNormal(C3,C2);
  n2 = calcNormal(C4,C1);
  n3 = calcNormal(C3,C4);
  n4 = calcNormal(C2,C1);

  /*
   * create planes E1, E2, E3 and E4
   */
  E1[0]       = n1[0];
  E1[1]       = n1[1];
  E1[2]       = n1[2];
  E1[3]       = 0;

  E2[0]       = n2[0];
  E2[1]       = n2[1];
  E2[2]       = n2[2];
  E2[3]       = 0;

  E3[0]       = n3[0];
  E3[1]       = n3[1];
  E3[2]       = n3[2];
  E3[3]       = 0;

  E4[0]       = n4[0];
  E4[1]       = n4[1];
  E4[2]       = n4[2];
  E4[3]       = 0;

  /*
   * check position of E2 compared to E1
   */
  d_E1toC4    = distanceToPlane(E1,C4);
  d_E2toC3    = distanceToPlane(E2,C3);
  d_E3toC2    = distanceToPlane(E3,C2);
  d_E4toC3    = distanceToPlane(E4,C3);

  // TOTEST:
// cout << C1[0] << "/" << C1[1] << "/" << C1[2] << endl;
// cout << C2[0] << "/" << C2[1] << "/" << C2[2] << endl;
// cout << C3[0] << "/" << C3[1] << "/" << C3[2] << endl;
// cout << C4[0] << "/" << C4[1] << "/" << C4[2] << endl;
// cout << endl;
// printCoords2File(objectCorners, 4, "/home/rainer/workspace/corners.txt");
// printCoords2File(C1, 1, "/home/rainer/workspace/c1.txt");
// printCoords2File(C2, 1, "/home/rainer/workspace/c2.txt");
// printCoords2File(C3, 1, "/home/rainer/workspace/c3.txt");
// printCoords2File(C4, 1, "/home/rainer/workspace/c4.txt");
//  cout << d_E1toC4 << " && " << d_E2toC3 << endl;
//  cout << d_E3toC2 << " && " << d_E4toC3 << endl;

  if((d_E1toC4 > 0) && (d_E2toC3 < 0))
    bottomE1 = true;

  if((d_E3toC2 < 0) && (d_E4toC3 > 0))
    leftE4 = true;


  /*
   * Check if point is between E1, E2, E3 and E4
   */

  if(bottomE1) // E1 bottom, E2 top
  {
    if(leftE4) // E3 right, E4 left
    {
      //cout << d_E1toC4 << "/" << d_E4toC3 << "/" << endl;

      for(int i= 0; i< size; i++)    //TOCHANGE
      {
        p[0] = points[3*i];
        p[1] = points[3*i+1];
        p[2] = points[3*i+2];
        mask[i] = pointFrustumFilter(p, E1, E2, E3, E4, thres_visionCone);
      }
    }
    else // E3 left, E4 right
    {

      for(int i= 0; i< size; i++)
      {
        p[0] = points[3*i];
        p[1] = points[3*i+1];
        p[2] = points[3*i+2];

        mask[i] = pointFrustumFilter(p, E1, E2, E4, E3, thres_visionCone);
      }
    }
  }
  else // E1 top, E2 bottom
  {
    if(leftE4) // E3 right, E4 left
    {
      //cout << d_E1toC4 << "/" << d_E4toC3 << "/" << endl;

      for(int i= 0; i< size; i++)    //TOCHANGE
      {
        p[0] = points[3*i];
        p[1] = points[3*i+1];
        p[2] = points[3*i+2];
        mask[i] = pointFrustumFilter(p, E2, E1, E3, E4, thres_visionCone);
      }
    }
    else // E3 left, E4 right
    {

      for(int i= 0; i< size; i++)
      {
        p[0] = points[3*i];
        p[1] = points[3*i+1];
        p[2] = points[3*i+2];

        mask[i] = pointFrustumFilter(p, E2, E1, E4, E3, thres_visionCone);
      }
    }
  }
  //printCoords2File(points, mask, size, "/home/rainer/workspace/inCone.txt");


  delete [] n1;
  delete [] n2;
  delete [] n3;
  delete [] n4;
  delete [] E1;
  delete [] E2;
  delete [] E3;
  delete [] E4;
}

void pointLocation(double* points, int size, double* objectCorners, double* planeValues, double thres_dist, double thres_visionCone, int* pointLocMask)
{
  /*
   * see book 30.3.16 /
   */
  bool* mask    = new bool[size];
  double* point = new double[3];

  double* intersectionPoint;

  double d_point = 0;
  double d_intersectionPoint = 0;

// //TOTEST
//  double* test_intersection = new double[3*size];
//  double* test_incone = new double[3*size];
//  double* test_onplane = new double[3*size];
//  double* test_behind = new double[3*size];
//  for(int i = 0; i < 3*size; i++)
//  {
//    test_intersection[i] = 0;
//    test_incone[i] = 0;
//    test_onplane[i] = 0;
//    test_behind[i] = 0;
//  }

  bool set = 0;
  //printCoords2File(objectCorners, 4, "/home/rainer/workspace/corners.txt");

  /*
   * check if point is in vision cone (frustum filter)
   */
  pointsInCone(points, size, objectCorners, thres_dist, thres_visionCone, mask);

//    cout << "objectCorners: " << endl;
//    for(int i = 0; i < 4; i++)
//    {
//      cout << objectCorners[3*i] << "/" << objectCorners[3*i+1] << "/" << objectCorners[3*i+2] << endl;
//    }

    /*
     * create plane equation in cartesian form
     */


//    printCoords2File(objectCorners, 4, "/home/rainer/workspace/corners.txt");

//    cout << "Plane values" << endl;
//    cout << planeValues[0] << "/" << planeValues[1] << "/" << planeValues[2] << "/" << planeValues[3] << endl;

  /*
   * check points in the cone, if there are before or behind the plane
   */
  for(int i=0; i < size; i++)
  {
    if(mask[i])
    {
      // in the vision cone
      point[0] = points[3*i];
      point[1] = points[3*i+1];
      point[2] = points[3*i+2];

      /*
       * calculate intersection point with plane
       */
      intersectionPoint = intersectionWithPlane(planeValues, point);

//      if(set == 0)
//      {
//        printCoords2File(intersectionPoint, 1, "/home/rainer/workspace/base.txt");
//        printCoords2File(point, 1, "/home/rainer/workspace/point.txt");
//        set = 1;
//      }

     // cout << basePoint[0] << "/" << basePoint[1] << "/" << basePoint[2] << endl;

      d_intersectionPoint = calcVectorLength(intersectionPoint);
      d_point             = calcVectorLength(point);

//      test_intersection[3*i] = intersectionPoint[0];
//      test_intersection[3*i+1] = intersectionPoint[1];
//      test_intersection[3*i+2] = intersectionPoint[2];
//      test_incone[3*i] = point[0];
//      test_incone[3*i+1] = point[1];
//      test_incone[3*i+2] = point[2];

      /*
       * check point location
       */
//        0 = in front the object or out of the vision cone
//        1 = on the object
//        2 = behind the object
        // cout << d_point << "-" << d_basePoint << " / ";
        if(d_point > (d_intersectionPoint+thres_dist))
        {
          // behind object
          pointLocMask[i] = 2;

//          test_behind[3*i] = point[0];
//          test_behind[3*i+1] = point[1];
//          test_behind[3*i+2] = point[2];
        }
        else if(d_point < (d_intersectionPoint-thres_dist))
        {
          // before object
          pointLocMask[i] = 0;

        }
        else
        {

          // on object
          pointLocMask[i] = 1;

//          test_onplane[3*i] = point[0];
//          test_onplane[3*i+1] = point[1];
//          test_onplane[3*i+2] = point[2];
        }
    }
    else
    {
      // out of vision cone
      pointLocMask[i] = 0;
    }
  }

//  cout << endl;
//
//  printCoords2File(test_intersection, size, "/home/rainer/workspace/intersection.txt");
//  printCoords2File(test_incone, size, "/home/rainer/workspace/incone.txt");
//  printCoords2File(test_onplane, size, "/home/rainer/workspace/onplane.txt");
//  printCoords2File(test_behind, size, "/home/rainer/workspace/behind.txt");
//  printCoords2File(points, size, "/home/rainer/workspace/allpoints.txt");

  delete [] mask;
  delete [] point;
}

bool behindPlane(double point_x, double point_y, double point_z, double* planeValues)
{
  bool result                 = false;
  double* point               = new double[3];
  double d_point              = 0;
  double d_intersectionPoint  = 0;

  double* intersectionPoint;

  point[0] = point_x;
  point[1] = point_y;
  point[2] = point_z;

  /*
  * calculate intersection point with plane
  */
  intersectionPoint = intersectionWithPlane(planeValues, point);

  d_intersectionPoint = calcVectorLength(intersectionPoint);
  d_point             = calcVectorLength(point);

  /*
  * check if point is further away than the intersection point
  */
  if(d_point > d_intersectionPoint)
   result = true;

  delete [] point;

  return result;
}


void cornersToPlaneEqu(double* objectCorners, double* cartesianEquationValues)
{
  /*
   * see book 30.3.16
   *
   * C1 => objectCorners[0] - [2]
   * C2 => objectCorners[3] - [5]
   * C3 => objectCorners[6] - [8]
   */
  double* a = new double[3];
  double* b = new double[3];
  double* n = new double[3];

  a[0] = objectCorners[3] - objectCorners[0];
  a[1] = objectCorners[4] - objectCorners[1];
  a[2] = objectCorners[5] - objectCorners[2];

  b[0] = objectCorners[6] - objectCorners[0];
  b[1] = objectCorners[7] - objectCorners[1];
  b[2] = objectCorners[8] - objectCorners[2];

  n = calcNormal(a,b);

  cartesianEquationValues[0] = n[0];
  cartesianEquationValues[1] = n[1];
  cartesianEquationValues[2] = n[2];
  cartesianEquationValues[3] = n[0]* objectCorners[0] + n[1]* objectCorners[1]+ n[2]*objectCorners[2];

  delete [] a;
  delete [] b;
  delete [] n;

}

double* calcNormal(double* v1, double* v2)
{
  double* n = new double[3];

  n[0] = v1[1]*v2[2] - v1[2]*v2[1];
  n[1] = v1[2]*v2[0] - v1[0]*v2[2];
  n[2] = v1[0]*v2[1] - v1[1]*v2[0];

  return n;
}
double distanceToPlane(double* planeEqu, double* point)
{
  /*
   * see book 30.3.16
   */
  return (planeEqu[0]*point[0] + planeEqu[1]*point[1] + planeEqu[2]*point[2]) / sqrt(pow(planeEqu[0],2) + pow(planeEqu[1],2) + pow(planeEqu[2],2));
}

double calcVectorLength(double* vector)
{
  double length = sqrt(pow(vector[0],2) + pow(vector[1],2) + pow(vector[2],2));
  return length;
}

double calcVectorLength(double* v1, double* v2)
{
  double length = sqrt(pow(v1[0]-v2[0],2) + pow(v1[1]-v2[1],2) + pow(v1[2]-v2[2],2));
  return length;
}

double calcVectorLength(double x, double y, double z)
{
  double length = sqrt(x*x + y*y + z*z);
  return length;
}

double calcAngleVectors(double* v1, double* v2)
{
  double dot = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
  double lenSq1 = v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2];
  double lenSq2 = v2[0]*v2[0] + v2[1]*v2[1] + v1[2]*v1[2];
  double angle = acos(dot/sqrt(lenSq1 * lenSq2));
  return (angle/PI*180);
}

double* calcAngleVectorAxes(double* v1)
{
  double* angle = new double[3];

  /*
   * angle x
   */
  double dot_x = v1[0]*1;
  double dot_y = v1[1]*1;
  double dot_z = v1[2]*1;

  double lenSq1 = v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2];

  angle[0] = acos(dot_x/sqrt(lenSq1))/PI*180;
  angle[1] = acos(dot_y/sqrt(lenSq1))/PI*180;
  angle[2] = acos(dot_z/sqrt(lenSq1))/PI*180;

  return angle;
}

double* basepoint2Plane(double* plane, double* point)
{
  /*
   * plane E: a * x + b * y + c * y + d = 0
   * a = plane[0]
   * b = plane[1]
   * c = plane[2]
   * d = plane[3]
   * vec_n = (a/b/c)
   * line l: x = lamda * vec_n + point
   */
  double lamda = 0;
  double* fp = new double[3];

  lamda = (-plane[3] - (plane[0]*point[0] + plane[1]*point[1] + plane[2]*point[2])) / (pow(plane[0], 2) + pow(plane[1], 2) + pow(plane[2], 2));

  fp[0] = lamda * plane[0] + point[0];
  fp[1] = lamda * plane[1] + point[1];
  fp[2] = lamda * plane[2] + point[2];

  return fp;
}

double* intersectionWithPlane(double* plane, double* point)
{
  /*
   * plane E: a * x + b * y + c * y + d = 0
   * a = plane[0]
   * b = plane[1]
   * c = plane[2]
   * d = plane[3]
   * line trough point g: x_vec = t * p_vec + v_vec
   * p_vec = coordinate of point.
   * v_vec = is 0, because the line goes through the center
   *
   * => E = g
   */
  double* intersection = new double[3];
  double t = 0;

  t = -plane[3] / (plane[0]*point[0] + plane[1]*point[1] + plane[2]*point[2]);

  intersection[0] = t * point[0];
  intersection[1] = t * point[1];
  intersection[2] = t * point[2];

  return intersection;
}

void sortCorners(double* c1, double* c2, double* c3, double* c4, double* c_sorted)
{
  /*
   * copy together points to sort x, y and z values
   */
  std::vector<data> x_values(4);
  std::vector<data> y_values(4);
  std::vector<data> z_values(4);

  x_values[0].value = c1[0];
  x_values[1].value = c2[0];
  x_values[2].value = c3[0];
  x_values[3].value = c4[0];

  y_values[0].value = c1[1];
  y_values[1].value = c2[1];
  y_values[2].value = c3[1];
  y_values[3].value = c4[1];

  z_values[0].value = c1[2];
  z_values[1].value = c2[2];
  z_values[2].value = c3[2];
  z_values[3].value = c4[2];

  double* c_unsorted = new double[12];

  c_unsorted[0]  = c1[0];
  c_unsorted[1]  = c1[1];
  c_unsorted[2]  = c1[2];

  c_unsorted[3]  = c2[0];
  c_unsorted[4]  = c2[1];
  c_unsorted[5]  = c2[2];

  c_unsorted[6]  = c3[0];
  c_unsorted[7]  = c3[1];
  c_unsorted[8]  = c3[2];

  c_unsorted[9]  = c4[0];
  c_unsorted[10] = c4[1];
  c_unsorted[11] = c4[2];


  /*
   * write indices
   */
  for(int i = 0; i < 4; i++)
  {
    x_values[i].index = i;
    y_values[i].index = i;
    z_values[i].index = i;
  }

//  cout << "unsorted corners" << endl;
//  cout << c1[0] << " " << c1[1] << " " << c1[2] << endl;
//  cout << c2[0] << " " << c2[1] << " " << c2[2] << endl;
//  cout << c3[0] << " " << c3[1] << " " << c3[2] << endl;
//  cout << c4[0] << " " << c4[1] << " " << c4[2] << endl;
//  cout << "----" << endl;

//  cout << " unsorted z" << endl;
//
//  for(int i = 0; i < 4; i++)
//  {
//    cout << z_values[i].index << " :" << z_values[i].value << endl;
//  }

  /*
   * sort z-values
   */
  std::sort(z_values.begin(), z_values.end(), by_number());

//  cout << " sorted z" << endl;
//  for(int i = 0; i < 4; i++)
//  {
//    cout << z_values[i].index << " :" << z_values[i].value << endl;
//  }

  /*
   * sort corners according to their indices
   * left top corner first, then left bottom, then right bottom and then right top
   */
    /*
     * check for objects which are placed on the walls
     */
    if(y_values[z_values[2].index].value > y_values[z_values[3].index].value)
    {
      //z_values[2].index is left top corner
      c_sorted[0]   = c_unsorted[3*z_values[2].index];
      c_sorted[1]   = c_unsorted[3*z_values[2].index+1];
      c_sorted[2]   = c_unsorted[3*z_values[2].index+2];

      // is right top corner
      c_sorted[9]   = c_unsorted[3*z_values[3].index];
      c_sorted[10]  = c_unsorted[3*z_values[3].index+1];
      c_sorted[11]  = c_unsorted[3*z_values[3].index+2];
    }
    else
    {
      //z_values[3].index is left top corner
      c_sorted[0]   = c_unsorted[3*z_values[3].index];
      c_sorted[1]   = c_unsorted[3*z_values[3].index+1];
      c_sorted[2]   = c_unsorted[3*z_values[3].index+2];

      // is right top corner
      c_sorted[9]   = c_unsorted[3*z_values[2].index];
      c_sorted[10]  = c_unsorted[3*z_values[2].index+1];
      c_sorted[11]  = c_unsorted[3*z_values[2].index+2];
    }

    if(y_values[z_values[0].index].value > y_values[z_values[1].index].value)
    {
      //z_values[0].index is left bottom corner
      c_sorted[3] = c_unsorted[3*z_values[0].index];
      c_sorted[4] = c_unsorted[3*z_values[0].index+1];
      c_sorted[5] = c_unsorted[3*z_values[0].index+2];

      // is right bottom corner
      c_sorted[6] = c_unsorted[3*z_values[1].index];
      c_sorted[7] = c_unsorted[3*z_values[1].index+1];
      c_sorted[8] = c_unsorted[3*z_values[1].index+2];
    }
    else
    {
      //z_values[1].index is left bottom corner
      c_sorted[3] = c_unsorted[3*z_values[1].index];
      c_sorted[4] = c_unsorted[3*z_values[1].index+1];
      c_sorted[5] = c_unsorted[3*z_values[1].index+2];

      // is right bottom corner
      c_sorted[6] = c_unsorted[3*z_values[0].index];
      c_sorted[7] = c_unsorted[3*z_values[0].index+1];
      c_sorted[8] = c_unsorted[3*z_values[0].index+2];
    }

    /*
     * check if left and right plane are exchanged
     */

  /*
     * TODO: Check for objects which are placed above the scanner => parallel to xy-plane
   */

//  cout << "sorted corners" << endl;
//  for(int i=0; i < 4; i++)
//  {
//    cout << c_sorted[3*i] << " " << c_sorted[3*i+1] << " " << c_sorted[3*i+2] << endl;
//
//  }

}
//unsigned int pointLocationCheck(double* coords, double* dist, double corners)
//{
//  int location = 0;  // location unknown
//
//  /*
//   * determine, if point is in the vision cone
//   */
//  if(cone)
//  {
//    /*
//     * determine, if point is in front, on or behind the plane
//     */
//
//  }
//  else
//  {
//    /*
//     * point is a regular scan point
//     */
//    location = 1;
//  }
//
//
//  return location;
//}

void mirrorOnPlane(double* coords, double* mirrored_coords, double* plane_Coefficients, unsigned int size_coords)
{
  /*
   * check book 09/15/16
   */
  double* vec_P     = new double[3];
  double* vec_P_mir = new double[3];
  double* vec_SP    = new double[3];
  double* vec_S     = new double[3];

  for(unsigned int i=0; i < size_coords; i++)
  {
    /*
     * create vector P out of point i
     */
    vec_P[0] = coords[3*i];
    vec_P[1] = coords[3*i+1];
    vec_P[2] = coords[3*i+2];

    /*
     * calculate perpendicular point s
     */
    vec_S = basepoint2Plane(plane_Coefficients, vec_P);

    /*
     * create vector SP => move coordinate system to S, so that the mirroring can be done by negation of each direction
     */
    vec_SP[0] = vec_P[0] - vec_S[0];
    vec_SP[1] = vec_P[1] - vec_S[1];
    vec_SP[2] = vec_P[2] - vec_S[2];

    /*
     * mirror vec_SP
     */
    vec_P_mir[0] = -vec_SP[0];
    vec_P_mir[1] = -vec_SP[1];
    vec_P_mir[2] = -vec_SP[2];

    /*
     * move back into original coordinate system
     */
    mirrored_coords[3*i]    = vec_S[0] + vec_P_mir[0];
    mirrored_coords[3*i+1]  = vec_S[1] + vec_P_mir[1];
    mirrored_coords[3*i+2]  = vec_S[2] + vec_P_mir[2];

  }
//  //TOTEST:
//  printCoords2File(corners_in, 4, "/home/rainer/workspace/corners.txt");
//  printCoords2File(coords, size_coords, "/home/rainer/workspace/coords.txt");
//  printCoords2File(mirrored_coords, size_coords, "/home/rainer/workspace/coords_mirrored.txt");


}


