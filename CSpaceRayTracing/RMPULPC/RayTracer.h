#ifndef _RAYTRACER_H_INCLUDED
#define _RAYTRACER_H_INCLUDED
///////////////////////////////////////
// RayTracer.h
// Created
//    Traces Rays in CSpace
//////////////////////////////////////

#include "RayCSpace.h"
#include <Environment.h>
#include <CollisionDetection.h>

#define MAX_BOUNCINGS 10000
#define MAX_RAY_LENGTH 10000
//The following constant defines the maximum number of rays to be used
//Eventually it'll have to be changed by a better approach to know when
//to stop shooting out rays, by the moment it'll be 1
#define MAX_RAYS 1
#define RT_RANDOMLY 0
#define RT_HEURISTIC 1
#define RT_TARGET_ORIENTED 2
#define RT_NORMAL 3

class RayTracer {
 public:
/*    RayTracer(Environment *environment, Cfg source, Cfg target); */
  RayTracer(Roadmap *rdmp, CollisionDetection *cd, SID cdsetid, CDInfo cdinfo, DistanceMetric * dm, SID dmsetid);
  void connectCCs();
  bool findPath(Cfg &source, Cfg &target);
  void setSource(Cfg configuration);
  void setTarget(Cfg configuration);
/*    void setEnvironment(Environment *environment); */
  void setDirection(const int policy);//set direction of the ray to trace
  bool trace();//trace a ray in the established direction
  void newDirection(/*policy*/);//set a new direction according to some policy
  bool exhausted(); //returns true if all the possibilities have been explored
  void printPath ();//print the path found

 private:
  RayCSpace ray; // Ray to be traced
  Cfg source, target;
  Cfg direction; //
  Environment *environment;
  int rays_tested; // Number of rays tested so far
  bool all_explored; // True if all possible rays have been traced
  int policy; // Policy to shoot rays, possible values in setDirection

  Roadmap *rdmp;
  CollisionDetection *cd;
  SID cdsetid;
  CDInfo cdinfo;
  DistanceMetric *dm;
  SID dmsetid;
  
};

#endif /*_RAYTRACER_H_INCLUDED*/







