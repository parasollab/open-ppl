// $Id$
/////////////////////////////////////////////////////////////////////
//
//  CfgManager.c
//
//  General Description
//      This class provides some implmentations which could be
//      different for different Cfg types, and some methods do require
//      a derived Cfg provide its own version(pure virtual methods).
//      This is an abstract class.
//
//                CfgManager   <--------  Cfg
//                /   |     \
//              _/    |      \_
//      Cfg_free      |         Cfg_free_serial<int>
//                Cfg_fixed_PRR
//
//  BE AWARE: 
//      It is ASSUMED that all position values are packed together
//      in the first 'posDof' spots of a Cfg data. Other choices 
//      would require the corrrsponding derived class have its own
//      version of most member functions listed here.
//
//  Created
//      08/31/99        Guang Song
//
//  Last Modified By:
//
/////////////////////////////////////////////////////////////////////
#include "util.h"
#include "Vectors.h"
#include "Cfg.h"
#include "CfgManager.h"
#include "Environment.h"

#define EQ(a,b)  (fabs(a-b)<0.0001)



CfgManager::~CfgManager() {}

// Normalize the orientation to the some range.
void CfgManager::Normalize_orientation(Cfg& c) {
    for(int i=posDof; i<c.v.size(); ++i)
	c.v[i] = c.v[i] - floor(c.v[i]);
}


Cfg CfgManager::InvalidData(){
  vector<double> tmp;
  for(int i=0; i<dof; ++i) {
     if(i<posDof)
        tmp.push_back(MAXFLOAT);
     else
        tmp.push_back(0.0);
  }
  return Cfg(tmp);
}


// Return the range of a single parameter of the configuration (i.e., range of x)
// param = the parameter to get the range for
// In the future, this function should get the range for x,y,z by the bounding box
// Currently it assumes the range for a position parameter to be -10000 to 10000
// and the range for an orientation parameter to be 0 to 1, which should
// also be changed to reflect any self collision in a linked robot
pair<double,double> CfgManager::SingleParamRange(int param) {
  pair<double,double> range;

  if ((param>=0) && (param<dof)) {
    if ((param>=0) && (param<posDof)) {
      range.first=-10000;
      range.second=10000;
    } else {
      range.first=0;
      range.second=1;
    }
  } else {
	cout << "Error in CfgManager::SingleParamRange, out of range! \n";
	exit(1);
  }
  return range;
 
}
 
#if 0
Cfg CfgManager::GetRandomCfg(
	double *boundingBox,
	double rangeFactor) {
   vector<double> tmp;
   for(int i=0; i<dof; ++i) {
      if(i<posDof) {
	 double p = boundingBox[i]-(boundingBox[i+1]-boundingBox[i])*((rangeFactor-1)/2)+
                    rangeFactor*(boundingBox[i+1]-boundingBox[i])*drand48();
	 tmp.push_back(p);
      } else
	 tmp.push_back(drand48());
   }
   return Cfg(tmp);
	
}
#endif



void CfgManager::IncrementTowardsGoal(
	Cfg &c, 
	const Cfg &goal, 
	const Cfg &increment)
{
  double tmp;
  int i;
  for(i=0; i<posDof; ++i) {
        if( fabs(goal.v[i]-c.v[i]) < fabs(increment.v[i]))
            c.v[i] = goal.v[i];
        else
            c.v[i] += increment.v[i];
  }
  for(i=posDof; i<dof; ++i) {
      if(c.v[i] != goal.v[i]) {
	 double orientationIncr = increment.v[i] < 0.5 ? increment.v[i] : 1-increment.v[i];
         tmp=DirectedAngularDistance(c.v[i],goal.v[i]);
         if(fabs(tmp) < orientationIncr) {
            c.v[i]=goal.v[i];
         } else {
            c.v[i] += increment.v[i];
            c.v[i]=Normalize_wrt_b(c.v[i],1);
         }
      }
   }
  

}


vector<Cfg> CfgManager::FindNeighbors(
	const Cfg& start,
	Environment *_env,
	const Cfg &increment,
  	CollisionDetection *cd,
	int noNeighbors,
	SID  _cdsetid,
	CDInfo& _cdInfo){

   vector<Cfg> ret;
   vector<Cfg> nList;
 
   nList.push_back(increment); 
   vector<double> posOnly, oriOnly;
   int i;
   for(i=0; i<dof; ++i) {
	if(i<posDof) {
	    posOnly.push_back(increment.v[i]);
	    oriOnly.push_back(0.0);
	} else {
	    posOnly.push_back(0.0);
	    oriOnly.push_back(increment.v[i]);
	}
   }
   nList.push_back(Cfg(posOnly));
   nList.push_back(Cfg(oriOnly));

   // find close neighbour in every dimension.
   vector<double> oneDim;
   for(i=0; i< dof; i++)
	oneDim.push_back(0.0);
   for(i=0; i< dof; i++) { 
        oneDim[i] = increment.v[i];
        nList.push_back(Cfg(oneDim));
        nList.push_back(-Cfg(oneDim));
        oneDim[i] = 0.0;  // reset to 0.0
   }
        

   if(noNeighbors > nList.size()) noNeighbors = nList.size(); 
   for(i=0;i<(noNeighbors);i++) {
       Cfg tmp= start + nList[i];
       if(!AlmostEqual(start, tmp) && !tmp.isCollision(_env, cd, _cdsetid,_cdInfo) ) 
            ret.push_back(tmp);
   }

   return ret;
}

vector<Cfg> CfgManager::FindNeighbors(
	const Cfg& start,
	Environment *_env,
	const Cfg& goal,
	const Cfg& increment,
        CollisionDetection *cd,
	int noNeighbors,
	SID  _cdsetid,
	CDInfo& _cdInfo) {

   vector<Cfg> ret;
   vector<Cfg> nList;
     
   nList.push_back(increment);
   vector<double> posOnly, oriOnly;
   int i;
   for(i=0; i<dof; ++i) {
        if(i<posDof) {
            posOnly.push_back(increment.v[i]);
            oriOnly.push_back(0.0);
        } else {
            posOnly.push_back(0.0);
            oriOnly.push_back(increment.v[i]);
        }
   }
   nList.push_back(Cfg(posOnly));
   nList.push_back(Cfg(oriOnly));

   // find close neighbour in every dimension.
   vector<double> oneDim;
   for(i=0; i< dof; i++)
        oneDim.push_back(0.0);
   for(i=0; i< dof; i++) {
        oneDim[i] = increment.v[i];
        nList.push_back(Cfg(oneDim));
        nList.push_back(-Cfg(oneDim));
        oneDim[i] = 0.0;  // reset to 0.0
   }


   /* Need to modify following code for the future cfgs */
   if(noNeighbors > nList.size()) noNeighbors = nList.size();
   for(i=0;i<noNeighbors;++i) {
       Cfg tmp = start;
       IncrementTowardsGoal(tmp, goal, nList[i]);
       if(!AlmostEqual(start, tmp) && !tmp.isCollision(_env,cd,_cdsetid,_cdInfo) ) {
            ret.push_back(tmp);
       }
   }
 
   return ret;

}

Cfg CfgManager::FindIncrement( 
	const Cfg& _start,
	const Cfg& _goal, 
	int * n_ticks,
	double positionRes,
	double orientationRes)
{
    Cfg diff = _goal - _start; 

    // adding two basically makes this a rough ceiling...
    *n_ticks= (int)max(PositionMagnitude(diff)/positionRes, 
		       OrientationMagnitude(diff)/orientationRes) + 2;

    return FindIncrement(_start, _goal, *n_ticks);

}

Cfg CfgManager::FindIncrement( 
	const Cfg& c,
	const Cfg& _goal, 
	int  n_ticks)
{
    vector<double> incr;
    for(int i=0; i<dof; ++i) {
	if(i<posDof) 
	    incr.push_back((_goal.v[i] - c.v[i])/n_ticks);
	else
	    incr.push_back(DirectedAngularDistance(c.v[i], _goal.v[i])/n_ticks);
    }
	    
    return Cfg(incr);
}



bool CfgManager::AlmostEqual(const Cfg&c1, const Cfg &c2)
{
  for(int i=0; i<dof; ++i) {
     if(i<posDof) {
	if(!EQ(c1.v[i], c2.v[i])) return false;
     } else {
	if(!EQ(DirectedAngularDistance(c1.v[i], c2.v[i]), 0.0)) return false;
     }
  }
  return true;
}

bool CfgManager::isWithinResolution(
	const Cfg&c1, 
	const Cfg&c2, 
	double positionRes,
	double orientationRes)
{
    Cfg diff = c1 - c2;

    return PositionMagnitude(diff) <= positionRes &&
           OrientationMagnitude(diff) <= orientationRes;

}


double  CfgManager::OrientationMagnitude(const Cfg& c)
{
  double result = 0.0;
  for(int i=posDof; i<dof; ++i) {
	result += c.v[i] > 0.5 ? sqr(1.0 - c.v[i]) : sqr(c.v[i]);
  }
  return sqrt(result);
}


double CfgManager::PositionMagnitude(const Cfg& c)
{
  double result = 0.0;
  for(int i=0; i<posDof; ++i) 
	result += sqr(c.v[i]);
  return sqrt(result);
}


Cfg CfgManager::GetPositionOrientationFrom2Cfg(
	const Cfg &c1,
	const Cfg &c2)
{
  vector<double> tmp;
  for(int i=0; i<dof; ++i) {
     if(i<posDof)
        tmp.push_back(c1.v[i]);
     else
        tmp.push_back(c2.v[i]);
  }
  return Cfg(tmp);
  
}

vector<Cfg> CfgManager::GetMovingSequenceNodes(const Cfg &c1, const Cfg &c2, double s) {
    vector<Cfg> result;
    Cfg tmp = Cfg::WeightedSum(c1, c2, s);
    Cfg s1  = GetPositionOrientationFrom2Cfg(tmp,c1);
    Cfg s2  = GetPositionOrientationFrom2Cfg(tmp,c2);
    result.push_back(c1);
    result.push_back(s1);
    result.push_back(s2);
    result.push_back(c2);

    return result;
}

void CfgManager::writeTransformation(FILE *_fp, Transformation & tmp) {
        Orientation ori = tmp.orientation;
        ori.ConvertType(Orientation::FixedXYZ);
        fprintf(_fp,"\n%f %f %f %f %f %f",
               tmp.position[0], tmp.position[1], tmp.position[2],
               ori.gamma/6.2832, ori.beta/6.2832, ori.alpha/6.2832);
}


void CfgManager::print_vizmo_format_to_file(const Cfg &c, Environment *env, FILE *_fp) {
  ConfigEnvironment(c, env);
  int robot = env->GetRobotIndex();
  int numofLink = env->GetMultiBody(robot)->GetFreeBodyCount();
  for(int i=0; i<numofLink; i++) {
     Transformation tmp = env->GetMultiBody(robot)->GetFreeBody(i)
                          ->WorldTransformation();
     writeTransformation(_fp, tmp);
  }

}

void CfgManager::print_preamble_to_file(Environment *env, FILE *_fp, int numofCfg) {
    fprintf(_fp,"VIZMO_PATH_FILE\n");

    int numofLink = env->GetMultiBody(env->GetRobotIndex())->GetFreeBodyCount();
    fprintf(_fp,"%d\n", numofLink);

    fprintf(_fp,"%d ",numofCfg);

}

bool CfgManager::isCollision(const Cfg &c, Environment *env, CollisionDetection *cd,
                           SID _cdsetid, CDInfo& _cdInfo, MultiBody * onflyRobot) {
        //ConfigEnvironment(c, onflyRobot);
	ConfigEnvironment(c, env);
        bool result = cd->IsInCollision(env, _cdsetid, _cdInfo, onflyRobot);
        return result;
}

bool CfgManager::isCollision(const Cfg &c, Environment *env, CollisionDetection *cd,
                             SID _cdsetid, CDInfo& _cdInfo){
     if(!ConfigEnvironment(c, env))
         return true;

     // after updating the environment(multibodies), Ask ENVIRONMENT
     // to check collision! (this is more nature.)
     bool answerFromEnvironment = cd->IsInCollision(env, _cdsetid, _cdInfo);
     return answerFromEnvironment;
}

bool CfgManager::isCollision(const Cfg &c, Environment *env, CollisionDetection *cd,
                int robot, int obs, SID _cdsetid, CDInfo& _cdInfo){
     if(!ConfigEnvironment(c, env))
          return true;

     // ask CollisionDetection class directly.
     bool answerFromCD = cd->IsInCollision(env, _cdsetid, _cdInfo, robot, obs);
     return answerFromCD;
}

