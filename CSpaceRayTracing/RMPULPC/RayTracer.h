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

#include <string>

class RayTracer {
 public:
  enum BouncingMode {TARGET_ORIENTED, RANDOM, HEURISTIC, NORMAL}; 
/*    RayTracer(Environment *environment, Cfg source, Cfg target); */
  RayTracer(Roadmap *rdmp, CollisionDetection *cd, SID cdsetid, DistanceMetric * dm, SID dmsetid);
  void setOptions(string bouncing_mode, int max_rays, int max_bounces, int max_ray_length);
  void connectCCs();
  bool findPath(Cfg &source, Cfg &target);
  void setSource(Cfg configuration);
  void setTarget(Cfg configuration);
/*    void setEnvironment(Environment *environment); */
  void setInitialDirection();//set direction of the ray to trace
  bool trace();//trace a ray in the established direction
  void newDirection(/*policy*/);//set a new direction according to some policy
  bool exhausted(); //returns true if all the possibilities have been explored
  void printPath ();//print the path found

 private:
  RayCSpace ray; // Ray to be traced
  Cfg source, target;
  Cfg direction; //
  Environment *environment;
  int max_rays; // Maximum number of rays to test when finding a path
  int max_bounces; // Maximum number of bounces for each ray
  int max_ray_length; // Maximum length for each ray
  int rays_tested; // Number of rays tested so far
  bool all_explored; // True if all possible rays have been traced
  BouncingMode bouncing_policy; // Policy to shoot rays, possible values in setDirection

  Roadmap *rdmp;
  CollisionDetection *cd;
  SID cdsetid;
  DistanceMetric *dm;
  SID dmsetid;
  
};

#endif /*_RAYTRACER_H_INCLUDED*/







