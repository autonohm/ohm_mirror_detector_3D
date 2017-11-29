/**
* @file   testFunctions.cpp
* @author Rainer Koch
* @date   07.09.2016
*
*
*/
#include "testFunctions.h"


void printObjectTypeAmounts(object3D::objType* objectTypes, int size, string infotext)
{
  int m0 = 0;
  int m1 = 0;
  int m2 = 0;
  int m3 = 0;
  int m4 = 0;
  int m5 = 0;
  int m6 = 0;
  int m7 = 0;

  for(int i=0; i <= size; i++)
  {
    if(objectTypes[i] == 0)
    m0++;
    if(objectTypes[i] == 1)
    m1++;
    if(objectTypes[i] == 2)
    m2++;
    if(objectTypes[i] == 3)
    m3++;
    if(objectTypes[i] == 4)
    m4++;
    if(objectTypes[i] == 5)
    m5++;
    if(objectTypes[i] == 6)
    m6++;
    if(objectTypes[i] == 7)
    m7++;
  }
  cout << infotext << ": " << size << "=" << m0 << "-" << m1 << "-" << m2 << "-" << m3 << "-" << m4 << "-" << m5 << "-" << m6 << "-" << m7 << endl;

}


void printObjectCorners(double* corners, string infotext)
{
   cout << "corner: " << infotext << endl;
   cout << corners[0] << "/" << corners[1] << "/" << corners[2] << endl;
   cout << corners[3] << "/" << corners[4] << "/" << corners[5] << endl;
   cout << corners[6] << "/" << corners[7] << "/" << corners[8] << endl;
   cout << corners[9] << "/" << corners[10] << "/" << corners[11] << endl;
     //To write in file
//     for (int j = 0; j < 4; ++j)
//     {
//       file_1 << test[3*j] << " " << test[3*j+1] << " " << test[3*j+2] << " " << endl;
//     }

}
void printObjectCorners(double* corners, int nr)
{
   cout << "corner: " << nr << endl;
   cout << corners[0] << "/" << corners[1] << "/" << corners[2] << endl;
   cout << corners[3] << "/" << corners[4] << "/" << corners[5] << endl;
   cout << corners[6] << "/" << corners[7] << "/" << corners[8] << endl;
   cout << corners[9] << "/" << corners[10] << "/" << corners[11] << endl;
     //To write in file
//     for (int j = 0; j < 4; ++j)
//     {
//       file_1 << test[3*j] << " " << test[3*j+1] << " " << test[3*j+2] << " " << endl;
//     }

}

void printCoords2File(double* coords, int size, const char* filename)
{
  ofstream file;
  file.open(filename);

  // write in file
  for(int i = 0; i < size; i++)
  {
    if(!isnan(coords[3*i]) && !isnan(coords[3*i+1]) && !isnan(coords[3*i+2]) or ((coords[3*i] == 0) && (coords[3*i+1] == 0) && (coords[3*i+2] == 0)))
      file << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
  }
  file.close();

}
void printCoords2File(double* coords, bool* mask, int size, const char* filename)
{
  ofstream file;
  file.open(filename);

  // write in file
  for(int i = 0; i < size; i++)
  {
    if(mask[i])
    {
      if(!isnan(coords[3*i]) && !isnan(coords[3*i+1]) && !isnan(coords[3*i+2]) or ((coords[3*i] == 0) && (coords[3*i+1] == 0) && (coords[3*i+2] == 0)))
        file << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
    }
  }
  file.close();

}
void printCoords2File(double* coords, object3D::objType* mask, object3D::objType filter, int size, const char* filename)
{
  ofstream file;
  file.open(filename);

  // write in file
  for(int i = 0; i < size; i++)
  {
    if(mask[i] == filter)
    {
      if((!isnan(coords[3*i]) and !isnan(coords[3*i+1]) and !isnan(coords[3*i+2])))// and (!(coords[3*i] == 0) && !(coords[3*i+1] == 0) && !(coords[3*i+2] == 0)) and ((abs(coords[3*i]) > 0.00001) && (abs(coords[3*i+1]) > 0.00001) && !(abs(coords[3*i+2]) > 0.00001)))
        file << coords[3*i] << " " << coords[3*i+1] << " " << coords[3*i+2] << " " << endl;
    }
  }
  file.close();

}

void printIntens2File(unsigned int* intens, int size, const char* filename)
{
  ofstream file;
  file.open(filename);

  // write in file
  for(int i = 0; i < size; i++)
  {
    if((!isnan(intens[i])) and (intens[i] != 0))
      file << intens[i] << endl;
  }
  file.close();

}

void printIntens2PlaneFile(double* coords, unsigned int* intens, int size, double* centroid, const char* filename)
{
  ofstream file;
  file.open(filename);
  double * tmp_coords;
  tmp_coords = new double[size];
  tmp_coords = coords;

  for(int i = 0; i < size; i++)
  {
    if((!isnan(intens[i])) and (intens[i] < 10000) and (intens[i] > 0) and (!isnan(tmp_coords[3*i])) and (!isnan(tmp_coords[3*i+1])) and (!isnan(tmp_coords[3*i+2])))
    {
      /*
       * transfer surface coords into xy-plane
       */
       tmp_coords[3*i] = coords[3*i] - centroid[0];
       tmp_coords[3*i+1] = coords[3*i+1] - centroid[1];
       tmp_coords[3*i+2] = coords[3*i+2] - centroid[2];

       /*
        * write into file
        */
       //TODO: check which coords need to use to have plane
       file << tmp_coords[3*i+1] << " " << tmp_coords[3*i+2] << " " << intens[i] << endl;
    }
  }
  //printCoords2File(tmp_coords, size, "/home/rainer/workspace/surf_tmp_coords.txt");

  file.close();

}

