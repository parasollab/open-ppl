///////////////////////////////////////////////
// RayCSpace.cpp
//    Implementation of class RayCSpace
///////////////////////////////////////////////

#include "RayCSpace.h"
#include <iostream.h>

void RayCSpace::init(Cfg origin, Cfg direction, Cfg target) {
  this->origin = origin;
  this->direction = direction;
  this->target = target;
  rayLength = 0;
  numberOfBounces = 0;
  path.reserve(10000);//its not good to have this hard coded. (I wonder if it is needed)
  path.push_back(origin);
  cout << "Initializing the ray in C-space\n";
}

void RayCSpace::finish(void) {
  path.push_back(target);
}

void RayCSpace::bounce(Cfg direction) {
  //cout << "the ray is bouncing in point in direction\n";
  this->direction = direction;
  rayLength += collisionDistance;
  path.push_back(collisionConfiguration);//includes collisionConfiguration in list
  origin = collisionConfiguration;
  numberOfBounces++;
}

bool RayCSpace::collide(Environment *env, CollisionDetection *cd,
			SID cdsetid, CDInfo& cdInfo, DistanceMetric *dm,
			SID dmsetid, double maxLength) {
  bool collision=false; 
  //Based on code by Shawna (Cfg::cAproxCspaceClearance:
  //Cfg of first collision
  Cfg cfg = origin;
 
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  int n_ticks;
  double tmpDist;

  Cfg dir;//check again this and the next line
  dir = direction; //direction of the ray
  Cfg tick = cfg;//to walk through a straight line heading direction
  Cfg incr = cfg.FindIncrement(dir,&n_ticks,positionRes,orientationRes);//I have to figure out what positionRes and orientationRes are
  //it seems that FindIncrement updates n_ticks in cover the space accordingly to positionRes and orientationRes (resolutions both)
   
  int tk = 0; // controls the number of ticks generated
  //the while is repeated for a maximum ray length of 'clear' and for a maximum number of ticks=n_ticks
  //it is stopped when a collision is found

  while(!collision && (dm->Distance(env,cfg,tick,dmsetid) < maxLength) ) {
    tick.Increment(incr); //next configuration to check
    //check if the tick is still in the bounding box and or it is in collision
    //any case it is colliding something
    if( (tick.isCollision(env,cd,cdsetid,cdInfo)) || !(tick.InBoundingBox(env)) ) {
      tmpDist = dm->Distance(env, cfg, tick, dmsetid);//gets the distance between the two ticks
      if(tmpDist < maxLength) { //if the distance is less than clear it means the clearance is the new distance
	collisionConfiguration = tick;
	collisionDistance = tmpDist; 
	//find a way to store the id of the colliding object and lastCollisionFree configuration
      }
      collision = true; //turns on the collision flag to stop looking in this direction
    }
    
    //if it is testing the last tick allowed and there hasn't been any collision
    //it assumes the distance of that ray as the distance walked so far
    //if(!collision){
    //cout << "maximum number of ticks allowed to test reached\n";
    //tmpDist = dm->Distance(env, cfg, tick, dmsetid);//gets the distance
      //if the distance gotten is less than the previously computed clearance it has to update the clearance
    //if(tmpDist < maxLength){
	//in this case no collision was found, but the function cannot keep
	//going farther because it reached its maxLength specified
	//find a way to say so.
    //}
    //}
    tk++;// increases the tick
  }
  return collision;
  //End of Code based on Shawna's work
  // It is also important to know what happens with the bounding box,
  //does it count as an object also?, I think so, but what if it does not?
}

bool RayCSpace::connectTarget(Environment *env, CollisionDetection *cd,
			SID cdsetid, CDInfo& cdInfo, DistanceMetric *dm,
			SID dmsetid) {
  bool collision=false; 
  //Based on code by Shawna (Cfg::cAproxCspaceClearance:
  //Cfg of first collision
  Cfg cfg = origin;
 
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  int n_ticks;
  double tmpDist;

  Cfg dir;//check again this and the next line
  dir = target; //direction of the ray
  Cfg tick = cfg;//to walk through a straight line heading direction

  Cfg incr = cfg.FindIncrement(dir,&n_ticks,positionRes,orientationRes);//I have to figure out what positionRes and orientationRes are
  //it seems that FindIncrement updates n_ticks in cover the space accordingly to positionRes and orientationRes (resolutions both)
   
  int tk = 0; // controls the number of ticks generated
  //the while is repeated for a maximum ray length of 'clear' and for a maximum number of ticks=n_ticks
  //it is stopped when a collision is found
  while(tk < n_ticks && !collision) {
    tick.Increment(incr); //next configuration to check
    //check if the tick is still in the bounding box and or it is in collision
    //any case it is colliding something
    if( (tick.isCollision(env,cd,cdsetid,cdInfo)) || !(tick.InBoundingBox(env)) ) {
      tmpDist = dm->Distance(env, cfg, tick, dmsetid);//gets the distance between the two ticks
      collisionConfiguration = tick;
      collisionDistance = tmpDist; 
      //find a way to store the id of the colliding object and lastCollisionFree configuration
      collision = true; //turns on the collision flag to stop looking in this direction
    }
    
    //if it is testing the last tick allowed and there hasn't been any collision
    //it assumes the distance of that ray as the distance walked so far
    if(tk == n_ticks - 1 && !collision){
      //cout << "maximum number of ticks allowed to test reached\n";
      tmpDist = dm->Distance(env, cfg, tick, dmsetid);//gets the distance
      //if the distance gotten is less than the previously computed clearance it has to update the clearance
    }
    tk++;// increases the tick
  }
  return !collision;
  //End of Code based on Shawna's work
  // It is also important to know what happens with the bounding box,
  //does it count as an object also?, I think so, but what if it does not?
}

double RayCSpace::length(void) {
  return rayLength;
}

void RayCSpace::writePath(Environment *env) {
  writePathConfigurations("RayCSpace.path", path, env);
}
void RayCSpace::writePathConfigurations(char output_file[80],
						vector<Cfg> path,
						Environment *env ) {
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

void RayCSpace::save(Environment *environment) {
  trace.environment = environment;
  //the vertices are the configurations in the path
  trace.m_pRoadmap->AddVertex(path);
  //the edges are from each configuration to the next one
  for (int i = 0; i+1 < path.size(); i++)
    trace.m_pRoadmap->AddEdge(i, i+1);//here I should put the distance between the pair
  trace.m_pRoadmap->WriteGraph("salida.map");
}
