  Cfg cfg = *this;
 
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  int n_ticks;
  double clear, tmpDist;
  clear = 1e10;

  Cfg dir;
  dir = GetRandomCfg(env); //This is not a random direction by any chance,
  //this is the direction of the parameter
  Cfg tick = cfg;//to walk through a straight line heading direction

  Cfg incr = cfg.FindIncrement(dir,&n_ticks,positionRes,orientationRes);//I have to figure out what positionRes and orientationRes are
  //it seems that FindIncrement updates n_ticks in cover the space accordingly to positionRes and orientationRes (resolutions both)
   
  int tk = 0; // controls the number of ticks generated
  int collisionFlag = false;
  //the while is repeated for a maximum ray length of 'clear' and for a maximum number of ticks=n_ticks
  //it is stopped when a collision is found
  while(tk < n_ticks && !collisionFlag && (dm->Distance(env,cfg,tick,dmsetid) < clear) ) {
    tick.Increment(incr); //next configuration to check
    //check if the tick is still in the bounding box and or it is in collision
    //any case it is colliding something
    if( (tick.isCollision(env,cd,cdsetid,cdInfo)) || !(tick.InBoundingBox(env)) ) {
      tmpDist = dm->Distance(env, cfg, tick, dmsetid);//gets the distance between the two ticks
      if(tmpDist < clear) { //if the distance is less than clear it means the clearance is the new distance
	clear = tmpDist; //updates the clearance
	direction = &dir; //keeps the direction of the clearance
      }
      collisionFlag = true; //turns on the collision flag to stop looking in this direction
    }
    
    //if it is testing the last tick allowed and there hasn't been any collision
    //it assumes the distance of that ray as the distance walked so far
    if(tk == n_ticks-1 && !collisionFlag){
      tmpDist = dm->Distance(env, cfg, tick, dmsetid);//gets the distance
      //if the distance gotten is less than the previously computed clearance it has to update the clearance
      if(tmpDist < clear){
	clear = tmpDist;//updates the clearance
	direction = &dir;//keeps the direction of the clearance
      }
    }
    tk++;// increases the tick
  }
  return clear;//returns the clearance

Class CObstacleDistance {
 public:
    CObstacleDistance(){}
    ~CObstacleDistance(){}
    set(id closest, double distance, Cfg cfg){
      closestObject = closest;
      distanceClosest = distance;
      lastCollisionFreeConfiguration= cfg;
    id closestObject;
    double distanceClosest;
    Cfg lastCollisionFreeConfiguration;
    private;
};

CObstacleDistance ApproxCObstacleDistance (Cfg dir, double maxLength) {

  Cfg cfg = *this;
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  int n_ticks;
  double tmpDist;

  CObstacleDistance result; //In my case this is collisionConfiguration
  Cfg tick = cfg;//to walk through a straight line heading direction


  Cfg incr = cfg.FindIncrement(dir,&n_ticks,positionRes,orientationRes);//I have to figure out what positionRes and orientationRes are
  //it seems that FindIncrement updates n_ticks in cover the space accordingly to positionRes and orientationRes (resolutions both)
  int tk = 0; // controls the number of ticks generated
  int collisionFlag = false;
  //the while is repeated for a maximum ray length (maxLength) and for a maximum number of ticks=n_ticks
  //it is stopped when a collision is found
  while(tk < n_ticks && !collisionFlag && 
	(dm->Distance(env,cfg,tick,dmsetid) < maxLength) ) {
    tick.Increment(incr); //next configuration to check
    //check if the tick is still in the bounding box and or it is in collision
    //any case it is colliding something
    if( (tick.isCollision(env,cd,cdsetid,cdInfo)) || !(tick.InBoundingBox(env)) ) {
      tmpDist = dm->Distance(env, cfg, tick, dmsetid);//gets the distance between the two ticks
      if(tmpDist < maxLength) //if the distance is less than clear it means the clearance is the new distance
	result.set(tmpDist,collidingObstacle = ....,lastCollisionFreeConf = ....);
      collisionFlag = true; //turns on the collision flag to stop looking in this direction
    }

    //if it is testing the last tick allowed and there hasn't been any collision
    //it assumes the distance of that ray as the distance walked so far
    //but it cannot tell anything about a collidingObstacle
    if(tk == n_ticks-1 && !collisionFlag){
      tmpDist = dm->Distance(env, cfg, tick, dmsetid);//gets the distance
      //if the distance gotten is less than the previously computed clearance it has to update the clearance
      if(tmpDist < maxLength)
	result.set(tmpDist, NUL, lastCollisionFreeConf = ...);
    }
    tk++;// increases the tick
  } 
}
 

 double ApproxCspaceClearance(){
   double clear;
   CObstacleDistance clearObst;
   CObstacleDistance clearDir;
   clear = 1e10;
   for(int i = 0 ; i < n ; i++){
     dir = GetRandomCfg(env);
     collisionObst = ApproxCObstacleDistanc(dir, clear);
     if (collisionObst.distanceClosest < clear){
       clear = collisionObst.distanceClosest;
       clearDir = collisionObst;
     }
   }
   return clear;
 }
 
