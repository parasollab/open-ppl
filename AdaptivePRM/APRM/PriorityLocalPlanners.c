/////////////////////////////////////////////////////////////////////
//
//  PriorityLocalPlanners.c
//
//  General Description 
//	A derived class of class LocalPlanners. Edge weights are 
//      the probability of being connected. Always returns true.
//
//  Created
//	07/24/2002 Shawna Thomas
//
//  Last Modified By:
//      xx/xx/xx  <Name>
/////////////////////////////////////////////////////////////////////


#include "PriorityLocalPlanners.h"
#include "PriorityWeight.h"

double 
PriorityLocalPlanners::CalWeight(Environment* env, Cfg& _c1, Cfg& _c2, DistanceMetric* dm, LPInfo *info) {
  
  double length = dm->Distance(env, _c1, _c2, info->dmsetid);
 
  int level = ((PriorityWeight*)info->edge.first.GetIWeight())->Level();

  double power = pow(2, level);
  
  int max_level = 0;
  int n_ticks;
  Cfg incr = _c1.FindIncrement(_c2,&n_ticks,info->positionRes,info->orientationRes);
  double n_ticks_dbl = n_ticks;
  while(n_ticks_dbl > 1) {
    max_level++;
    n_ticks_dbl = n_ticks_dbl/2;
  }
  
  double probability;
  if(level <= max_level) { //only partially checked
    probability = pow((cosh(lambda * length / power)), (power*-1));
  } else { //completely checked
    probability = 1;
  }
  return (-1 * log(probability));
}


bool 
PriorityLocalPlanners::IsConnected_straightline_simple(Environment* env,
						    CollisionDetection* cd, DistanceMetric* dm, 
						    Cfg& _c1, Cfg& _c2, LP& _lp, 
						    LPInfo* info) {
  
  int n_ticks = pow(2, ((PriorityWeight*)info->edge.first.GetIWeight())->Level());
  Cfg tick = _c1;
  Cfg incr = _c1.FindIncrement(_c2, n_ticks);
  
  tick.Increment(-incr);
  for(int i=1; i<n_ticks; i+=2) { //only need to check every other one, the others have already been checked
    tick.Increment(incr);
    tick.Increment(incr);

    if(info->checkCollision) {	
      info->cd_cntr++;
      if(tick.isCollision(env,cd,info->cdsetid,info->cdInfo)) {
	tick.Increment(-incr);
	info->savedEdge = pair<Cfg,Cfg>(_c1, tick);
	info->edge.first.Weight() = 0;
	info->edge.first.Weight() = 0;
	
	return false;
      }
    }
    if(info->savePath || info->saveFailedPath) {
      info->path.push_back(tick);
    }
  }

  double edgeWeight = CalWeight(env,_c1,_c2,dm,info);
  info->edge.first.Weight() = edgeWeight;
  info->edge.second.Weight() = edgeWeight;
  
  return true;
}


bool 
PriorityLocalPlanners::IsConnected_rotate_at_s(Environment* env,
					    CollisionDetection* cd, DistanceMetric* dm, 
					    Cfg& _c1, Cfg& _c2, LP& _lp, 
					    LPInfo* info) {
  return false;
}


bool 
PriorityLocalPlanners::IsConnected_astar(Environment* env,
				      CollisionDetection* cd, DistanceMetric* dm, 
				      Cfg& _c1, Cfg& _c2, LP& _lp, 
				      LPInfo* info) {
  return false;
}


int
PriorityLocalPlanners::GetTicks(vector<Cfg>& ticks, Cfg _c1, Cfg _c2, SID _lpsetid, LPInfo *info) {
	
  vector<LP> lpset = planners.GetLPSet(_lpsetid); 
  int  lpcnt = 0;
   		
  return GetTicks(lpset[lpcnt].GetPlanner(), ticks, _c1, _c2, info);
}


int
PriorityLocalPlanners::GetTicks(PLANNER lpName, vector<Cfg>& ticks, Cfg& _c1, Cfg& _c2, LPInfo *info) {
	
  switch(lpName) {
  case STRAIGHTLINE:
    return GetTicks_straightline_simple(ticks,_c1,_c2,info);
    break;
  case ROTATE_AT_S:
    return GetTicks_rotate_at_s(ticks,_c1,_c2,info);
    break;
  case ASTAR_DISTANCE:
  case ASTAR_CLEARANCE:
    return GetTicks_astar(ticks,_c1,_c2,info);
    break;
  case APPROX_SPHERES:
    return GetTicks_approx_spheres(ticks,_c1,_c2,info);
    break;
  }
  cout << "Error: in PriorityLocalPlanners::GetTicks(PLANNER lpName, ...), invalid lp option!" << endl;
  exit(1);
}


int
PriorityLocalPlanners::GetTicks_straightline_simple(vector<Cfg>& ticks, Cfg& _c1, Cfg& _c2, 
						 LPInfo* info) {  
  int n_ticks = pow(2, ((PriorityWeight*)info->edge.first.GetIWeight())->Level());
  Cfg tick = _c1;
  Cfg incr = _c1.FindIncrement(_c2, n_ticks);

  ticks.push_back(tick);
  for(int i=0; i<n_ticks; i++) {
    tick.Increment(incr);
    ticks.push_back(tick);
  }
  
  return ticks.size();
}


int
PriorityLocalPlanners::GetTicks_rotate_at_s(vector<Cfg>& ticks, Cfg& _c1, Cfg& _c2,
					 LPInfo* info) {
  return ticks.size();
}
   

int
PriorityLocalPlanners::GetTicks_astar(vector<Cfg>& ticks, Cfg& _c1, Cfg& _c2,
				   LPInfo* info) {
  return ticks.size();
}
    

int
PriorityLocalPlanners::GetTicks_approx_spheres(vector<Cfg>& ticks, Cfg& _c1, Cfg& _c2,
					    LPInfo* info) {
  return ticks.size();
}
