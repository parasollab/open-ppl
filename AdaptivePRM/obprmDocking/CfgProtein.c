/////////////////////////////////////////////////////////////////////
//
//  CfgProtein.c
//
//  General Description
//	A derived template class from Cfg_fixed_tree. It provides some 
//	specific implementation directly related to a multiple joints
//	serial robot.
//
//  Created
//	08/31/99	Guang Song
//
//  Last Modified By:
//
/////////////////////////////////////////////////////////////////////


#include "CfgProtein.h"
#include "Environment.h"
#include "BioPotentials.h"
#include "MultiBody.h"

#define DefaultRange 1.0 
//#define MAXPOTENTIAL 999999.9

double CfgProtein::std = 60; // units: degree
double CfgProtein::range = 60;

// normally(gaussian) distributed random number generator.
double randn() {
  double v1, v2, rsq;
  static int iset = 0;
  static double gset;
  if(iset == 0) {
     do {
        v1 = 2*drand48() - 1.0;
	v2 = 2*drand48() - 1.0;
	rsq = v1*v1 + v2*v2;
     } while (rsq >= 1.0 || rsq == 0.0);
     double fac = sqrt(-2.0*log(rsq)/rsq);
     gset = v1*fac;
     iset = 1;
     return v2*fac;
  } else {
     iset = 0;
     return gset;
  }
}

CfgProtein::CfgProtein(int _numofJoints) : Cfg_fixed_tree(_numofJoints) {}

CfgProtein::~CfgProtein() {}

Vector3D CfgProtein::GetRobotCenterPosition(const Cfg &c) const {
   return Vector3D(0, 0, 0);
}


Cfg CfgProtein::GetRandomCfg(double R, double rStep){

   vector<double> result;
   double jointAngle;

   for(int i=0; i<NumofJoints; i++) {
	jointAngle = (2.0*rStep)*drand48() - rStep;
        jointAngle = jointAngle*DefaultRange;
	result.push_back(jointAngle);
   }

   return Cfg(result);

}

Cfg CfgProtein::GetRandomRay(double incr) {

   incr = 0.005;

   vector<double> result;
   for(int i=0; i<NumofJoints; i++)
        result.push_back(drand48()*DefaultRange*incr);

   return Cfg(result);

}

Cfg CfgProtein::GetRandomCfg_CenterOfMass(double *boundingBox) {
// this is not EXACTLY accurate, ok with most cases ... TO DO
// To be accurate, one has to make sure every link is inside the given BB,
// but here only the base link is taken care of. It is almost fine since
// a little 'bigger' BB will contain all links. 

   //static double std = 60.0;
   //static double range = 5.0; // 60.0;

   vector<double> tmp;
   //for(int i=0; i<dof; ++i) 
   //   tmp.push_back(drand48()*DefaultRange);
   for(int i=0; i<dof; ++i) {
 	int angle = BioPotentials::torsionAngle[i];
	if(abs(angle) == 90) // turns 
	   tmp.push_back((angle+range*(2*drand48()-1))/360); // angle +/- 60, uniform distr. 
	else // alpha or betas
	   tmp.push_back((angle+std*randn())/360.0); // normal distribution.
   }
   return Cfg(tmp);

}

vector<Cfg> CfgProtein::GetMovingSequenceNodes(const Cfg &c1, const Cfg &c2, double s) {

   vector<Cfg> result;
   vector<double> tmp;
   for(int i=0; i<dof; i++) {
      if(i<2)
         tmp.push_back(c1.GetData()[i]);
      else
         tmp.push_back(c2.GetData()[i]);
   }

   result.push_back(c1);
   result.push_back(Cfg(tmp));
   result.push_back(c2);

   return result;
}


bool CfgProtein::isInRange(const Cfg &c) {
     //Normalize_orientation();
     vector<double> v = c.GetData();

     for(int i=0; i<dof; i++) {
        if(v[i] > DefaultRange)
          return false;
     }
     return true;
}


bool CfgProtein::ConfigEnvironment(const Cfg &c, Environment *_env) {
     if(! isInRange(c)) return false;

     vector<double> v = c.GetData();
     int robot = _env->GetRobotIndex();

     double jointAngle;
     int i;
     for(i=0; i<_env->GetMultiBody(robot)->GetFreeBodyCount(); i++) {
	if((i+1)%3 == 0) 
	    jointAngle = 180.0;   // joint angles for 3k+2 is always set to 180.
	else
	    jointAngle = v[i/3*2 + i%3]*360; // index mapping: 3k->2k, 3k+1->2k+1
        _env->GetMultiBody(robot)->GetFreeBody(i)
            ->GetBackwardConnection(0)->GetDHparameters().theta = jointAngle;
     }  // config the robot

     for(i=0; i<_env->GetMultiBody(robot)->GetFreeBodyCount(); i++) {
        FreeBody * afb = _env->GetMultiBody(robot)->GetFreeBody(i);
        if(afb->ForwardConnectionCount() == 0)  // tree tips: leaves.
             afb->GetWorldTransformation();
     }

     // since Transformation is calculated in recursive manner, only
     // let the last links(or Freebody) call getWorldTransformation will
     // automatically calculate the transformations for all previous links.

     // when all worldTransformations are recalculated by using new cfg, the
     // config of the whole robot is updated.
     return true;


}

bool CfgProtein::GenerateOverlapCfg(
		Environment *env,  // although env and robot is not used here,
		int robot,            // they are needed in other Cfg classes.
		Vector3D robot_start, 
		Vector3D robot_goal, 
		Cfg *resultCfg){

     vector<double> treeData;
     for(int i=0; i<NumofJoints; i++)
	treeData.push_back(drand48()*DefaultRange);
     // pass back the Cfg for this pose.
     *resultCfg = Cfg(treeData);
     return true;

}


bool CfgProtein::isCollision(Cfg &c, Environment *env, CollisionDetection *cd, 
                             SID _cdsetid, CDInfo& _cdInfo, bool notUsedHere) {
	return BioPotentials::isInPotentialRangeOfNodeGeneration(c, env);
}
