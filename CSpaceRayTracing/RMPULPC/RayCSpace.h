/////////////////////////////////////////////////////////////////////////
// RayCSpace.h
// Created 2/9/01
//    A Ray in C-Space
/////////////////////////////////////////////////////////////////////////

#ifndef _RAYCSPACE_H_INCLUDED
#define _RAYCSPACE_H_INCLUDED

#include <Cfg.h>
#include <Environment.h>
#include <Roadmap.h>
#include <OBPRM.h>
#include <DistanceMetrics.h>


class RayCSpace {
 private:
  Cfg direction; //direction to shoot a ray

  Cfg origin; //origin of the ray

  Cfg collisionConfiguration;
  double rayLength;
  int numberOfBounces;
  double collisionDistance;
  Cfg target;

  vector<Cfg> path;
  Roadmap trace;

  //list of points contained in the ray
 public:
  void init(Cfg origin, Cfg direction, Cfg target); //initialize the ray
  void finish();
  void bounce(Cfg direction);//Stores point in list and changes direction

  //This function has to call a collision detection routine (throug cfg)
  bool collide(Environment *env, CollisionDetection *cd,
			SID cdsetid, CDInfo& cdInfo, DistanceMetric *dm,
			SID dmsetid, double maxLength);//Check for collision in the environment
  bool connectTarget (Environment *env, CollisionDetection *cd,
			SID cdsetid, CDInfo& cdInfo, DistanceMetric *dm,
		      SID dmsetid);
  double length(void); //Length of the ray, I still have to figure out the units
  void writePath(Environment *env);
  void writePathConfigurations(char output_file[80],
			       vector<Cfg> path, Environment *env);
  void save(Environment *env);
};
#endif /*_RAYCSPACE_H_INCLUDED*/


