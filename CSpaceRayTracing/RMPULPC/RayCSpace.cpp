///////////////////////////////////////////////
// RayCSpace.cpp
//    Implementation of class RayCSpace
///////////////////////////////////////////////

#include "RayCSpace.h"
#include <iostream.h>

RayCSpace::RayCSpace() {
}
RayCSpace::~RayCSpace() {
}

void RayCSpace::init(Cfg origin, Cfg direction, Cfg target) {
  this->origin = origin;
  this->direction = direction;
  this->target = target;
  rayLength = 0;
  numberOfBounces = 0;
  // path.reserve(10000);//its not good to have this hard coded. (needed?)
  path.clear();
  path.push_back(origin);
  traveledDistance=0;
  collisionDistance=0;
  using_target_vector = false;
  target_vector = NULL;
}

void RayCSpace::finish(void) {
  rayLength += traveledDistance;
  path.push_back(reached_target);
}

void RayCSpace::bounce(Cfg direction) {
  this->direction = direction;
  //rayLength += collisionDistance;
  //path.push_back(collisionConfiguration);//includes collisionConfiguration in list
  //origin = collisionConfiguration;
  rayLength += traveledDistance;
  path.push_back(lastFreeConfiguration);
  origin = lastFreeConfiguration;
  numberOfBounces++;
}

//Based on code by Shawna (Cfg::cAproxCspaceClearance:
bool RayCSpace::collide(Environment *env, CollisionDetection *cd,
			SID cdsetid, CDInfo& cdInfo, DistanceMetric *dm,
			SID dmsetid, double maxLength) {
  bool collision=false; 
  
  Cfg cfg = origin; //Cfg of first collision
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  int n_ticks;
  double tmpDist;

  Cfg dir = direction; //direction of the ray
  Cfg tick = cfg;//to walk through a straight line heading direction
  Cfg incr = cfg.FindIncrement(dir,&n_ticks,positionRes,orientationRes);
  int tk = 0; // controls the number of ticks generated

  while(!collision && (dm->Distance(env,cfg,tick,dmsetid) < maxLength) ) {
    lastFreeConfiguration = tick;
    tick.Increment(incr); //next configuration to check
    if( (tick.isCollision(env,cd,cdsetid,cdInfo)) || !(tick.InBoundingBox(env)) ) {
      collisionConfiguration = tick;
      collisionDistance = dm->Distance(env, cfg, tick, dmsetid);
      collision = true;
    }
    tk++;// increases the tick
  }
  if (collision) {
    traveledDistance = dm->Distance(env, cfg, collisionConfiguration, dmsetid);
  }
  else
    traveledDistance += dm->Distance(env, cfg, tick, dmsetid);
  return collision;
}

bool RayCSpace::connectTarget(Environment *env, CollisionDetection *cd,
			SID cdsetid, CDInfo& cdInfo, DistanceMetric *dm,
			      SID dmsetid) {
  if (using_target_vector && target_vector != NULL) {
    //    cout << "looking for connections with " << target_vector->size() << " confs" << endl;
    for (unsigned int i = 0; i < target_vector->size(); ++i)
      if (connectTarget(env, cd, cdsetid, cdInfo, dm, dmsetid, (*target_vector)[i]))
	return true;
    return false;
  }
  else
    return connectTarget(env, cd, cdsetid, cdInfo, dm, dmsetid, target);
  return false;
}
//Based on code by Shawna (Cfg::cAproxCspaceClearance:
bool RayCSpace::connectTarget(Environment *env, CollisionDetection *cd,
			SID cdsetid, CDInfo& cdInfo, DistanceMetric *dm,
			SID dmsetid, Cfg &dir) {
  bool collision=false; 
  
  Cfg cfg = origin; //Cfg of first collision
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  int n_ticks;
  double tmpDist;

  //  Cfg dir = target; //direction of the ray
  Cfg tick = cfg;//to walk through a straight line heading direction
  Cfg incr = cfg.FindIncrement(dir,&n_ticks,positionRes,orientationRes);
  int tk = 0; // controls the number of ticks generated
  
  while(tk < n_ticks && !collision) {
    lastFreeConfiguration = tick;
    tick.Increment(incr); //next configuration to check
    if( (tick.isCollision(env,cd,cdsetid,cdInfo)) || !(tick.InBoundingBox(env)) ) {
      tmpDist = dm->Distance(env, cfg, tick, dmsetid);//distance % the ticks
      collisionConfiguration = tick;
      collisionDistance = dm->Distance(env, cfg, tick, dmsetid);
      collision = true; 
    }    
    tk++;// increases the tick
  }
  if (collision) {
    traveledDistance = dm->Distance(env, cfg, collisionConfiguration, dmsetid);
  }
  else
    traveledDistance += dm->Distance(env, cfg, tick, dmsetid);
  if (!collision)
    reached_target = dir;
  return !collision;
}

double RayCSpace::length(void) {
  return rayLength;
}

void RayCSpace::writePath(Environment *env) {
  writePathConfigurations("RayCSpace.path", path, env);
}

void RayCSpace::writePathConfigurations(char output_file[80],
					vector<Cfg> path, Environment *env ) {
  FILE *fp;	
  if((fp = fopen(output_file,"w")) == NULL){
    printf("\n\t Can't open file %s \n",output_file);
    exit(1);
  } 
  //Cfg::print_preamble_to_file(env, fp, path.size());        
  fprintf(fp,"VIZMO_PATH_FILE   Path Version %d\n", PATHVER_20001125);
  fprintf(fp,"%d\n", 1);
  fprintf(fp,"%d \n", path.size());
  for(int i = 0 ; i < path.size() ; i++){
    vector<double> tmp=path[i].GetData();
    // Translate all path configurations such that their resulting
    // center of gravity is what was saved (ie, the rover original)
    for(int j=0; j<tmp.size(); ++j) {
      fprintf(fp,"%f ",tmp[j]);
    }
    if(i!=(path.size()-1))fprintf(fp,"\n");
    
  }
  fprintf(fp,"\n");
  fclose(fp);
}

//This works assuming that the ray is a single thread that can be stretched
void RayCSpace::addRoadmapNodes(Roadmap &rdmp) {
  VID current, previous;

  for (int i= 1, previous = rdmp.m_pRoadmap->AddVertex(path[0]); i < path.size(); i++, previous = current) {
    current = rdmp.m_pRoadmap->AddVertex(path[i]);
    cout << endl << "Adding edge from " << previous << " to " << current<< endl;
    rdmp.m_pRoadmap->AddEdge(previous, current);
  }
}
void RayCSpace::setTargetVector(vector<Cfg> *target_vector) {
  this->target_vector = target_vector;
  using_target_vector = true;
}
