// $Id$
////////////////////////////////////////////////////
//
//  MyDistanceMetrics.h
//
//  derived class of DistanceMetrics.h
//
/////////////////////////////////////////////////////
#ifndef MyDistanceMetrics_h
#define MyDistanceMetrics_h


#include"DistanceMetrics.h"
#include"Vectors.h"

class MyDistanceMetrics : public DistanceMetric {

public:
   MyDistanceMetrics();
   ~MyDistanceMetrics();

   virtual double Distance(Environment *env, Cfg _c1, Cfg _c2, SID _dmsetid);
   //virtual double RMSD(vector<Vector3D> x, const vector<Vector3D> &y, int);
   virtual double RMSD(vector<Vector3D> x, vector<Vector3D> y, int);
   virtual double WeightedEuclidianDistance(Cfg _c1, Cfg _c2);

};

#endif
