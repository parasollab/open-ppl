// $Id$
/////////////////////////////////////////////////////////////////////
/**@file  Cfg_fixed_tree.cpp
  *
  * General Description
  *	A derived template class from CfgManager. It provides some 
  *	specific implementation directly related to a multiple joints
  *	serial robot.
  *
  * Created
  *	@date08/31/99	
  * @author Guang Song
  */
/////////////////////////////////////////////////////////////////////


#include "Cfg_fixed_tree.h"

#include "Cfg.h"
#include "MultiBody.h"
#include "Environment.h"
#include "GenerateMapNodes.h"
#include "util.h"

///@todo Document this!!
#define DefaultRange 0.25

Cfg_fixed_tree::Cfg_fixed_tree(int _numofJoints) : CfgManager(_numofJoints, 0), NumofJoints(_numofJoints) {}

Cfg_fixed_tree::~Cfg_fixed_tree() {}

Vector3D Cfg_fixed_tree::GetRobotCenterPosition(const Cfg &c) const {
   return Vector3D(0, 0, 0);
}


Cfg Cfg_fixed_tree::GetRandomCfg(double R, double rStep){

   vector<double> result;
   double jointAngle;

   for(int i=0; i<NumofJoints; i++) 
   {
		jointAngle = (2.0*rStep)*drand48() - rStep;
        jointAngle = jointAngle*DefaultRange;
		result.push_back(jointAngle);
   }

   return Cfg(result);

}

Cfg Cfg_fixed_tree::GetRandomRay(double incr) {

   incr = 0.005;

   vector<double> result;
   for(int i=0; i<NumofJoints; i++)
        result.push_back(drand48()*DefaultRange*incr);

   return Cfg(result);

}

Cfg Cfg_fixed_tree::GetRandomCfg_CenterOfMass(double *boundingBox) {

// Why following comments are here? This method suppose will generate
// Cfg whose center of mass will inside a given bounding box....

// this is not EXACTLY accurate, ok with most cases ... TO DO
// To be accurate, one has to make sure every link is inside the given BB,
// but here only the base link is taken care of. It is almost fine since
// a little 'bigger' BB will contain all links. 

   vector<double> tmp;
   for(int i=0; i<dof; ++i) 
      tmp.push_back(drand48()*DefaultRange);
   return Cfg(tmp);

}

vector<Cfg> Cfg_fixed_tree::GetMovingSequenceNodes(const Cfg &c1, const Cfg &c2, double s) {

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


bool Cfg_fixed_tree::isInRange(const Cfg &c) {
     //Normalize_orientation();
     vector<double> v = c.GetData();

     for(int i=0; i<dof; i++) {
        if(v[i] > DefaultRange)
          return false;
     }
     return true;
}


bool Cfg_fixed_tree::ConfigEnvironment(const Cfg &c, Environment *_env) {
     if(! isInRange(c)) return false;

     vector<double> v = c.GetData();
     int robot = _env->GetRobotIndex();

     int i;
     for(i=0; i<NumofJoints; i++) {
        _env->GetMultiBody(robot)->GetFreeBody(i)
            ->GetBackwardConnection(0)->GetDHparameters().theta = v[i]*360.0;
     }  // config the robot

     for(i=0; i<NumofJoints; i++) {
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

bool Cfg_fixed_tree::GenerateOverlapCfg(
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

//===================================================================
// GenSurfaceCfgs4ObstNORMAL
//      generate nodes by overlapping two triangles' normal.
// Guang Song 08/24/99
//===================================================================
vector<Cfg> Cfg_fixed_tree::GenSurfaceCfgs4ObstNORMAL
(Environment * env,CollisionDetection* cd, int obstacle, int nCfgs, SID _cdsetid, CDInfo& _cdInfo){
   cout << "Error in Cfg_fixed_tree::GenSurfaceCfgs4ObstNORMAL(), not implemented yet" << endl;
   exit(10);
}

