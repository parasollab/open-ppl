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

RayTracer::RayTracer(Roadmap *rdmp, CollisionDetection *cd, SID cdsetid, DistanceMetric * dm, SID dmsetid) {
  this->rdmp = rdmp;
  this->cd = cd;
  this->cdsetid = cdsetid;
  this->dm = dm;
  this->dmsetid = dmsetid;
  environment = rdmp->GetEnvironment();
  all_explored = false;
  rays_tested = 0;
  setOptions(string("targetOriented"), 1, 10000, 10000);
}

void RayTracer::setOptions(string bouncing_mode, int max_rays, int max_bounces, int max_ray_length) {
  this->max_rays = max_rays;
  this->max_bounces = max_bounces;
  this->max_ray_length = max_ray_length;
  //set the bouncing policy
  if (bouncing_mode == string("targetOriented"))
    bouncing_policy =TARGET_ORIENTED;
  else if (bouncing_mode == string("random"))
    bouncing_policy = RANDOM;
  else if (bouncing_mode == string("heuristic"))
    bouncing_policy = HEURISTIC;
  else if (bouncing_mode == string("normal"))
    bouncing_policy = NORMAL;
  else
    bouncing_policy = TARGET_ORIENTED;

}

void RayTracer::connectCCs() {
//    //the following lines are to be replaced with a clever
//    //way to pick a pair of configurations in two different connected components
//    //to try to connect. This is to be called afterwards
//    Cfg source = Cfg::GetFreeRandomCfg(environment, cd, cdsetid, cd->cdInfo);
//    Cfg target = Cfg::GetFreeRandomCfg(environment, cd, cdsetid, cd->cdInfo);
//    findPath(source, target);

//    vector< pair<int,VID> > ccs;
//    GetCCStats(*(rdmp->m_pRoadmap), ccs);//get the CCs

//    vector< pair<int,VID> >::iterator cci = ccs.begin();
//    Roadmap submap1 = Roadmap::Roadmap();
//    submap1.environment = rdmp->GetEnvironment();

//    vector<VID> cc; // a vector with the configs of cci
//    GetCC(*(rdmp->m_pRoadmap), cci->second, cc);
//    ModifyRoadMap(&submap, rdmp, cc);
//    vector< pair<int,VID> >::iterator ccj = cci+1;

//    //Order CCs to attempt to connect them
//    OrderCCByCloseness(rdmp, dm, info, ccvec);

//    VID cciid = cci->second;//maybe the VID of the "flag" Cfg of the CC?
//    while (ccj <= ccvec.end()) {
//      Roadmap submap2 = Roadmap::Roadmap();
//      submap2.environment = rdmp->GetEnvironment();

//      vector<VID> cctj;
//      GetCC(*(rdmp->m_pRoadmap), ccj->second, cctj);//copy the nodes in ccj->second to cct2
//      ModifyRoadMap(&submap2, rdmp, cct2);//submap2nodes (and incident edges) = nodes cct2 from rdmp
//      VID ccjid = ccj->second;
//      int d = 0;
//      GetCCcount(*(rdmp->m_pRoadmap));

//      vector<VID> cct3;
//      GetCC(*(rdmp->m_pRoadmap), ccj->second, cct3);
//      if (cct3.size()>= MAX_SMALL_CC_SIZE)
//        while ( !IsSameCC(*(rdmp->m_pRoadmap),  cciid, ccjid) && d < MAX_D && i < MAX_ITERATIONS) {
//  	//	 go through each configuration of submap1 and try to connect to submap2
//  	Roadmap * ray_rdmp = connectCCs(submap1, submap2);
//  	// put the ray obtained into the roadmap
//  	if (ray_rdmp != NULL){
//  	  vector<VID> ray_vertices;
//  	  ray_rdmp->m_pRoadmap->GetVerticesVID(ray_vertices);
//  	  ModifyRoadMap(rdmp, &ray_rdmp, ray_vertices);
//  	}
//  	d++;
//        }
//      ccj++; d=0;
//      submap2.environment = NULL;
//      submap2.m_pRoadmap = NULL;
//    }
//    submap1.environment = NULL;
//    submap1.m_pRoadmap = NULL; 
}

//  Roadmap * RayTracer::connectCCs(roadmap &ccA, roadmap &ccB) {
//    //first approach (to be replaced by a better one):
//    // findPath(config from ccA, config from ccB)
//    // return ray.roadmap();
//  }

bool RayTracer::findPath(Cfg &source, Cfg &target) {
  bool path_found = false;
  setSource(source);
  setTarget(target);  
  setInitialDirection();
  
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
void RayTracer::setInitialDirection() {
  cout << "Setting direction of the first ray: ";
  switch (bouncing_policy) {
  case TARGET_ORIENTED: direction = target;
    cout << "Direction of the target\n";
    break;
  case HEURISTIC:
    cout << "Heuristic method.\n";
    break;
  case RANDOM:
    cout << "Random,\n";
    break;
  case NORMAL:
    cout << "Normal\n";
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
  while (!path_found && number_bouncings < max_bounces &&
	 ray.length() < max_ray_length) {
    if (ray.connectTarget(environment, cd, cdsetid, cd->cdInfo, dm, dmsetid)) {
      ray.finish();
      path_found = true;
    }
    else
    if (ray.collide(environment, cd, cdsetid, cd->cdInfo, dm, dmsetid,max_ray_length)) { // if there is a collision
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
    //cout << "MaxBouncings: " << max_bounces << ", MaxRayLength: " << max_ray_length;
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
  if (rays_tested>=max_rays)
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




