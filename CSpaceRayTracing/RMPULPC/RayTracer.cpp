///////////////////////////////////////////
// RayTracer.cpp
//    Implementation of RayTracer class
///////////////////////////////////////////

#include "RayTracer.h"
#include <iostream.h>

//  RayTracer::RayTracer(Environment *environment, Cfg source, Cfg target) {
//    this->environment = environment;
//    this->source = source;
//    this->target = target;
//    all_explored = false; //Used by this->exhausted to decide if there are no more rays to trace
//    rays_tested = 0;
//  }

RayTracer::RayTracer(Roadmap *rdmp, CollisionDetection *cd, SID cdsetid, CDInfo cdinfo, DistanceMetric * dm, SID dmsetid) {
  this->rdmp = rdmp;
  this->cd = cd;
  this->cdsetid = cdsetid;
  this->cdinfo = cdinfo;
  this->dm = dm;
  this->dmsetid = dmsetid;
  environment = rdmp->GetEnvironment();
  all_explored = false;
  rays_tested = 0;
}

void RayTracer::connectCCs() {
  //the following lines are to be replaced with a clever
  //way to pick a pair of configurations in two different connected components
  //to try to connect. This is to be called afterwards
  Cfg source = Cfg::GetFreeRandomCfg(environment, cd, cdsetid, cdinfo);
  Cfg target = Cfg::GetFreeRandomCfg(environment, cd, cdsetid, cdinfo);
  findPath(source, target);
}

bool RayTracer::findPath(Cfg &source, Cfg &target) {
  bool path_found = false;
  setSource(source);
  setTarget(target);  
  setDirection(RT_TARGET_ORIENTED);
  
  while (!path_found && !exhausted()) {
    //Trace the ray
    cout<< "Trying new direction for ray"<<endl;
    path_found= trace();
    newDirection();
  }
  if (path_found)
    return true;
  return false;
}

// Set source of the ray (it is a configuration)
void RayTracer::setSource(Cfg configuration) {
  cout << "Setting the source of the ray\n";
  source = configuration;
}

// Set target of the ray (it is a configuration)
void  RayTracer::setTarget(Cfg configuration) {
  cout << "Setting the target (goal)\n";
  target = configuration;
}

// Set environment
//  void  RayTracer::setEnvironment(Environment *environment) {
//    cout << "Setting the environment\n";
//    this->environment = environment;
//  }

// Set direction in which the ray to be traced is going to be
// shot for the first time, a policy can be defined to select
// how the direction is generated. In this first implementation
// of the ray tracer I am going to shut rays in the direction
// of the target and when a ray hits an object it is going to 
// bounce in a random direction from it (of course this has to
// change if I want to have a coherent tracer).
void RayTracer::setDirection(const int policy) {
  cout << "Setting direction of the first ray: ";
  this->policy = policy;
  switch (policy) {
  case RT_TARGET_ORIENTED: direction = target;
    cout << "Direction of the target\n";
    break;
  case RT_HEURISTIC:
    cout << "Heuristic method.\n";
    break;
  case RT_RANDOMLY:
    cout << "Random,\n";
    break;
  default: direction = target;
    cout << "Direction of the target\n";
    break;
  }
}

//  bool RayTracer::trace(CollisionDetection *cd, SID cdsetid, CDInfo& cdinfo, 
//  		      DistanceMetric * dm, SID dmsetid) {
bool RayTracer::trace() {
  bool path_found = false; //true if a path has been found, false otherwise
  long number_bouncings = 0; //number of bouncings of the ray
  double ray_length = 0; //length of the ray (depending on metrics, I suppose)

  ray.init(source, direction, target); //initialize the ray
  while (!path_found && number_bouncings < MAX_BOUNCINGS &&
	 ray.length() < MAX_RAY_LENGTH) {
    if (ray.connectTarget(environment, cd, cdsetid, cdinfo, dm, dmsetid)) {
      ray.finish();
      path_found = true;
    }
    else
    if (ray.collide(environment, cd, cdsetid, cdinfo, dm, dmsetid,MAX_RAY_LENGTH)) { // if there is a collision
      //cout<< "\tif the collided object is the target's screen\n";
      //cout << "\t\tpath_found=true;\n";
      //cout << "\telse\n";
      //cout << "\t\tray.bounce(collisionPoint, newdirection);\n";
      cout << "there was a collision, the ray is going to bounce \n";
      ray.bounce(Cfg::GetRandomCfg(environment));//consider the two cases commented out
      cout << "the ray has bounced \n";
      ++number_bouncings;
    }
    //    cout << "Bouncings: "<< number_bouncings << ", Length of the ray so far: " << ray.length() << "\n";
    //cout << "MaxBouncings: " << MAX_BOUNCINGS << ", MaxRayLength: " << MAX_RAY_LENGTH;
  }
  if (path_found) {
    ray.save(environment);
    ray.writePath(environment);
  }
  return path_found;
}

//This function sets a direction to trace a new ray. This direction
//is not related with bouncings at all!.
void RayTracer::newDirection() {
  cout << "sets a new Direction according to the policy defined in setDirection\n";
  rays_tested++;
  if (rays_tested>=MAX_RAYS)
    all_explored = true;
  //by the momment this is going to be just a random direction
  direction = direction.GetRandomCfg(environment);
  cout << "new direction set" << endl;
}

bool RayTracer::exhausted() {
  cout << "all_explored status:" << all_explored << endl;
  return all_explored;
}

void RayTracer::printPath() {
  cout << "print the path found, remember that it is stored in the roadmap"<<endl;
}




