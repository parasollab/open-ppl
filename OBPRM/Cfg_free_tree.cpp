/////////////////////////////////////////////////////////////////////
//
//  Cfg_free_tree.c
//
//  General Description
//	A derived template class from CfgManager. It provides some
//	specific implementation directly related to a multiple joints
//	serial robot.
//
//  Created
//	08/31/99	Guang Song
//
//  Last Modified By:
//
/////////////////////////////////////////////////////////////////////

#include "util.h"
#include "Vectors.h"

#include "Cfg_free.h"
#include "Cfg_free_tree.h"
#include "Environment.h"
#include "GenerateMapNodes.h"

Cfg_free_tree::Cfg_free_tree(int _numofJoints) : CfgManager(6+_numofJoints, 3),
						     NumofJoints(_numofJoints) {}

Cfg_free_tree::~Cfg_free_tree() {}

Vector3D Cfg_free_tree::GetRobotCenterPosition(const Cfg &c) const {
   vector<double> tmp = c.GetData();
   return Vector3D(tmp[0], tmp[1], tmp[2]);
}


Cfg Cfg_free_tree::GetRandomCfg(double R, double rStep){
   double alpha,beta,z, z1;
   double jointAngle;

   alpha = 2.0*M_PI*drand48();
   beta  = 2.0*M_PI*drand48();
   z = R*cos(beta);
   z1 = R*sin(beta);

   double roll, pitch, yaw;
   roll = (2.0*rStep)*drand48() - rStep;
   pitch = (2.0*rStep)*drand48() - rStep;
   yaw = (2.0*rStep)*drand48() - rStep;

   Vector6<double> base(z1*cos(alpha),z1*sin(alpha),z,roll,pitch,yaw);
   vector<double> result;
   int i;
   for( i=0; i<6; ++i)
	result.push_back(base[i]);
   for(i=0; i<NumofJoints; i++) {
        jointAngle = (2.0*rStep)*drand48() - rStep;
        // or: jointAngle = 0.0; I am not sure which is more reasonable now. Guang
	result.push_back(jointAngle);
   }

   return Cfg(result);

}

Cfg Cfg_free_tree::GetRandomRay(double incr) {

   double alpha,beta,z, z1;
   double jointAngle;

   alpha = 2.0*M_PI*drand48();
   beta  = 2.0*M_PI*drand48();
   z = incr*cos(beta);
   z1 = incr*sin(beta);

   Vector6<double> base(z1*cos(alpha),z1*sin(alpha),z,0.0,0.0,0.0);
   vector<double> result;
   int i;
   for(i=0; i<6; ++i)
        result.push_back(base[i]);
   for(i=0; i<NumofJoints; i++) {
        jointAngle = 0.0;
        // or: jointAngle = drand48();
	result.push_back(jointAngle);
   }

   return Cfg(result);


}

Cfg Cfg_free_tree::GetRandomCfg_CenterOfMass(double *boundingBox) {
// this is not EXACTLY accurate, ok with most cases ... TO DO
// To be accurate, one has to make sure every link is inside the given BB,
// but here only the base link is taken care of. It is almost fine since
// a little 'bigger' BB will contain all links.

   vector<double> tmp;
   for(int i=0; i<dof; ++i) {
      if(i<3) {
         int k = 2*i;
         double p = boundingBox[k] +
                        (boundingBox[k+1]-boundingBox[k])*drand48();
         tmp.push_back(p);
      } else
         tmp.push_back(drand48());
   }
   return Cfg(tmp);

}


bool Cfg_free_tree::ConfigEnvironment(const Cfg &c, Environment *_env) {
     vector<double> v = c.GetData();
     int robot = _env->GetRobotIndex();

     // configure the robot according to current Cfg: joint parameters
     // (and base locations/orientations for free flying robots.)
     Transformation T1 = Transformation(
          Orientation(Orientation::FixedXYZ, v[5]*TWOPI, v[4]*TWOPI, v[3]*TWOPI), // RPY
          Vector3D(v[0],v[1],v[2]));

     _env->GetMultiBody(robot)->GetFreeBody(0)->Configure(T1);  // update link 1.
     int i;
     for( i=0; i<NumofJoints; i++) {
        _env->GetMultiBody(robot)->GetFreeBody(i+1)
          ->GetBackwardConnection(0)->GetDHparameters().theta = v[i+6]*360.0;
     }  // config the robot


     for(i=0; i<_env->GetMultiBody(robot)->GetFreeBodyCount(); i++) {
        FreeBody * afb = _env->GetMultiBody(robot)->GetFreeBody(i);
        if(afb->ForwardConnectionCount() == 0)  // tree tips: leaves.
             afb->GetWorldTransformation();
     }

     // since Transformation is calculated in recursive manner, only
     // let the last link(or Freebody) call getWorldTransformation will
     // automatically calculate the transformations for all previous links.

     // when all worldTransformations are recalculated by using new cfg, the
     // config of the whole robot is updated.
     return true;

}

bool Cfg_free_tree::GenerateOverlapCfg(
		Environment *env,  // although env and robot is not used here,
		int robot,            // they are needed in other Cfg classes.
		Vector3D robot_start,
		Vector3D robot_goal,
		Cfg *resultCfg){

     int i;
     Vector3D diff = robot_goal - robot_start;

     vector<double> result;
     for(i=0; i<3; ++i)
	result.push_back(diff[i]);
     for(i=3; i<dof; ++i)
	result.push_back(drand48());

     // pass back the Cfg for this pose.
     *resultCfg = Cfg(result);
     return true;

}


//===================================================================
// GenSurfaceCfgs4ObstNORMAL
//      generate nodes by overlapping two triangles' normal.
//===================================================================
vector<Cfg> Cfg_free_tree::GenSurfaceCfgs4ObstNORMAL
(Environment * env,CollisionDetection* cd, int obstacle, int nCfgs, 
SID _cdsetid,CDInfo& _cdInfo){
    static const int SIZE = 1;
    //static double jointAngles[SIZE][3] = {{0.0, 0.0, 0.0}, {0.25, 0.25, 0.25}, {0.0, 0.4, 0.0},
    //                                   {0.4, 0.6, 0.4},};
    vector<Cfg> surface;
    int robot = env->GetRobotIndex();
    GMSPolyhedron &polyRobot = env->GetMultiBody(robot)->GetFreeBody(0)->GetPolyhedron();
    GMSPolyhedron &polyObst = env->GetMultiBody(obstacle)->GetFixedBody(0)
                                  ->GetWorldPolyhedron();
    int num = 0;
    MultiBody * base = new MultiBody(env);
    base->AddBody(env->GetMultiBody(robot)->GetFreeBody(0));
    Cfg_free cfgFreeManager;
    while(num < nCfgs) {
          int robotTriIndex = (int)(drand48()*polyRobot.numPolygons);
          int obstTriIndex = (int)(drand48()*polyObst.numPolygons);
          vector<Cfg> cfgFree = cfgFreeManager.GetCfgByOverlappingNormal(
				env, cd, 
				polyRobot, polyObst, 
				robotTriIndex, obstTriIndex, 
				_cdsetid, _cdInfo,
				base);
          if(!cfgFree.empty()) {

             vector<double> basePose = cfgFree[0].GetData();
             for(int j=0; j<SIZE; ++j) {
                vector<double> serialData = basePose;  // for clearness, have basePose tmp variable.
                for(int i=0; i<NumofJoints; ++i) {  // now add joint angles.
		   serialData.push_back(drand48());
                }
                Cfg serial(serialData);
                if(!serial.isCollision(env,cd,_cdsetid,_cdInfo) && serial.InBoundingBox(env)) {
                   surface.push_back(serial);
                   ++num;
                }
             }
          }
    }
    return surface;
}

