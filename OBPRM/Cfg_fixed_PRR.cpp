// $Id$
/////////////////////////////////////////////////////////////////////
//
//  Cfg_fixed_PRR.c
//
//  General Description
//      A derived class from CfgManager. It provides some specific
//      implementation closely related to a PRR robot.
//      Degree of Freedom: 3
//
//  Created
//      08/31/99        Guang Song
//
/////////////////////////////////////////////////////////////////////

#include "Cfg_fixed_PRR.h"

#include "Cfg.h"
#include "MultiBody.h"
#include "Environment.h"
#include "GenerateMapNodes.h"
#include "util.h"

Cfg_fixed_PRR::Cfg_fixed_PRR() : CfgManager(3, 1) {}

Cfg_fixed_PRR::~Cfg_fixed_PRR() {}

CfgManager * Cfg_fixed_PRR::clone() const {
   return (new Cfg_fixed_PRR());
}



Vector3D Cfg_fixed_PRR::GetRobotCenterPosition(const Cfg &c) const {
   vector<double> tmp = c.GetData();
   return Vector3D(0, 0, tmp[0]);
}

Cfg Cfg_fixed_PRR::GetRandomCfg(double R, double rStep){
   double zz;
   if(drand48() > 0.5)
        zz = R;
   else
        zz = -R;

   double ceta1, ceta2;
   ceta1 = (2.0*rStep)*drand48() - rStep;
   ceta2 = (2.0*rStep)*drand48() - rStep;

   return Cfg(zz, ceta1, ceta2);

}

Cfg Cfg_fixed_PRR::GetRandomRay(double incr) {

   double alpha,beta,z, z1;

   alpha = 2.0*M_PI*drand48();
   beta  = 2.0*M_PI*drand48();
   z = incr*cos(beta);
   z1 = incr*sin(beta);

   return Cfg(z1*cos(alpha),z1*sin(alpha),z);

}

Cfg Cfg_fixed_PRR::GetRandomCfg_CenterOfMass(double *boundingBox) {

   double zz = boundingBox[4] +
                        (boundingBox[5]-boundingBox[4])*drand48();

   double ceta1 =1.0 * drand48();
   double ceta2 =1.0 * drand48();

   return Cfg(zz, ceta1, ceta2);
}


bool Cfg_fixed_PRR::ConfigEnvironment(const Cfg &c, Environment *_env) {
     vector<double> v = c.GetData();
     int robot = _env->GetRobotIndex();

     // configure the robot according to current Cfg: joint parameters
     // (and base locations/orientations for free flying robots.)
     double zz = v[0];
     double ceta1 = v[1]*360.0;
     double ceta2 = v[2]*360.0;
     _env->GetMultiBody(robot)->GetFreeBody(0)
          ->GetBackwardConnection(0)->GetDHparameters().theta = ceta1;
     _env->GetMultiBody(robot)->GetFreeBody(0)
          ->GetBackwardConnection(0)->GetDHparameters().d = zz;
     _env->GetMultiBody(robot)->GetFreeBody(0)
          ->GetForwardConnection(0)->GetDHparameters().theta = ceta2;

     // calculate WorldTransformation recursively from the very last link.
     _env->GetMultiBody(robot)->GetFreeBody(1)->GetWorldTransformation();
     return true;

}


bool Cfg_fixed_PRR::GenerateOverlapCfg(
		Environment *env,  // although env and robot is not used here,
		int robot,            // they are needed in other Cfg classes.
		Vector3D robot_start,
		Vector3D robot_goal,
		Cfg *resultCfg){

     // formulas here is from Craig book P128 with a little modifications.
     double x,y,a,b,r,L1,c2,s2; // known variables
     double  ceta1,ceta2, zz; // unknown ones.

     x = robot_goal[0];  // world frame
     y = robot_goal[1];
     a = robot_start[0]; // robot frame
     b = robot_start[1];
     r = sqrt(a*a+b*b);
     L1 = env->GetMultiBody(robot)->GetFreeBody(1)
          ->GetBackwardConnection(0)->GetDHparameters().a;
     c2 = ((x*x+y*y)-L1*L1-r*r)/(2.*L1*r);
     if(c2 >= 1.0 || c2 <= -1.0 ) return false;
     s2 = sqrt(1.-c2*c2);

     // find solution to this inverse kinematics problem.
     ceta1 = atan2(y,x) - atan2(r*s2, L1+r*c2);
     ceta2 = atan2(s2,c2) - atan2(b,a);
     zz = robot_goal[2];

     // pass back the Cfg for this pose.
     *resultCfg = Cfg(zz, ceta1/6.2832, ceta2/6.2832); // normalize to [0, 1]
     return true;

}


//===================================================================
// GenSurfaceCfgs4ObstNORMAL
//      generate nodes by overlapping two triangles' normal.
//===================================================================
vector<Cfg>
Cfg_fixed_PRR::GenSurfaceCfgs4ObstNORMAL
(Environment * env,CollisionDetection* cd, int obstacle, int nCfgs, 
	SID _cdsetid,CDInfo& _cdInfo){
   cout << "Error in Cfg_fixed_PRR::GenSurfaceCfgs4ObstNORMAL(), not implemented yet" << endl;
   exit(10);
}

