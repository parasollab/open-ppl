/////////////////////////////////////////////////////////////////////
//
//  Project.h
//
//  General Description
//      CS645 project
//      Function: generate cubic bspline curve.
//
//
//  Created
//      02/19/2001 Guang Song
//
//  Last Modified By:
//      xx/xx/xx  <Name>
/////////////////////////////////////////////////////////////////////
#ifndef Project_h
#define Project_h

#include<iostream.h>
#include<fstream.h>
#include<vector.h>
#include"Vectors.h"

void knotInsertion(double t, int d, vector<double> & knots, vector<Vector3D> &p);
void getCurve(int d, int n, vector<double> &knots, vector<Vector3D> &v,
              vector<Vector3D> &path, vector<double> &curvature, vector<double> &, double &);
void getPath(vector<Vector3D> &path, Vector3D d[4]);
void getCurvature(double &, vector<double> &, Vector3D d[4]);
void output(vector<Vector3D> &v, vector<Vector3D> &path, vector<double>&
     curvature, vector<double>&maxCurvature, char *bzCP, char *curve, char *cvtr, char*);
double Bernstein(int, int, double);
double B1(int, int, double);
double B2(int, int, double);

void ProjectGetPath(vector<Vector3D> &path, const vector<Vector3D> &controlPoints,
double &MaximumCvt);
#endif
