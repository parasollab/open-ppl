#ifndef _scheduling_mode_
#define _scheduling_mode_
//This enum should go inside the ConnectMapComponent collection
enum SCHEDULING_MODE {LARGEST_TO_SMALLEST, SMALLEST_TO_LARGEST, CLOSEST_TO_FARTHEST, FARTHEST_TO_CLOSEST};
#endif


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
#include <GraphAlgo.h>
/*#include "ConnectCCs.h"*/

#include "ConnectCCMethod.h"

#include <string>

class RayTracerConnectionMethod;
class ConnectCCs;
class RayTracer {
 public:
  enum BouncingMode {TARGET_ORIENTED, RANDOM, HEURISTIC, NORMAL}; 
/*    RayTracer(Environment *environment, Cfg source, Cfg target); */
  RayTracer(Roadmap *rdmp, CollisionDetection *cd, SID cdsetid, DistanceMetric * dm, SID dmsetid, ConnectMapNodes *cn);
  ~RayTracer();
  void setOptions(string bouncing_mode, int max_rays, int max_bounces, int max_ray_length);
  void connectCCs(SCHEDULING_MODE scheduling_mode, unsigned int schedule_max_size, unsigned int sample_max_size);
/*    bool connectCCs(Roadmap &cci, VID cci_id, vector<Cfg> &rep_cci_cfgs, Roadmap &ccj, VID ccj_id, vector<Cfg> &rep_ccj_cfgs, Roadmap &target_rdmp); */
  bool connectCCs(VID cci_id, vector<Cfg> &rep_cci_cfgs, VID ccj_id, vector<Cfg> &rep_ccj_cfgs, Roadmap &target_rdmp, bool try_backwards=false);
  bool findPath(Cfg &source, Cfg &target, Roadmap &ray_rdmp);
  bool findPath(Cfg &source, Cfg &target, vector<Cfg> *target_cfgs, Roadmap &ray_rdmp);
  void setSource(Cfg configuration);
  void setTarget(Cfg configuration);
  void setTargetCfgs(vector<Cfg> *target_cfgs);
/*    void setEnvironment(Environment *environment); */
  void setInitialDirection();//set direction of the ray to trace
  bool trace(Roadmap &ray_rdmp);//trace a ray in the established direction
  void newDirection(/*policy*/);//set a new direction according to some policy
  bool exhausted(); //returns true if all the possibilities have been explored
  void printPath ();//print the path found

 private:
  void getBoundaryCfgs(const vector<Cfg> &input, vector<Cfg> &output, unsigned int k);
  RayCSpace ray; // Ray to be traced
  Cfg source, target;
  vector<Cfg> *target_cfgs;
  bool using_target_vector;
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
  ConnectMapNodes *cn;

  int cd_counts;
  
};

#endif /*_RAYTRACER_H_INCLUDED*/

