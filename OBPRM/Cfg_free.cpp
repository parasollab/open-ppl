// $Id$
/////////////////////////////////////////////////////////////////////
//
//  Cfg_free.c
//
//  General Description
//	A derived class from CfgManager. It provides some specific
//	implementation for a 6-dof rigid-body moving in a 3-D
//	work space.
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
#include "Environment.h"
#include "GenerateMapNodes.h"

Cfg_free::Cfg_free() : CfgManager(6, 3) {}

Cfg_free::~Cfg_free() {}

Vector3D Cfg_free::GetRobotCenterPosition(const Cfg &c) const {
   vector<double> tmp = c.GetData();
   return Vector3D(tmp[0], tmp[1], tmp[2]);
}


Cfg Cfg_free::GetRandomCfg(double R, double rStep){
   double alpha,beta,z, z1;

   alpha = 2.0*M_PI*drand48();
   beta  = 2.0*M_PI*drand48();
   z = R*cos(beta);
   z1 = R*sin(beta);

   double roll, pitch, yaw;
   roll = (2.0*rStep)*drand48() - rStep;
   pitch = (2.0*rStep)*drand48() - rStep;
   yaw = (2.0*rStep)*drand48() - rStep;

   return Cfg(z1*cos(alpha),z1*sin(alpha),z,roll,pitch,yaw);
}

Cfg Cfg_free::GetRandomRay(double incr) {
   double alpha,beta,z, z1;

   alpha = 2.0*M_PI*drand48();
   beta  = 2.0*M_PI*drand48();
   z = incr*cos(beta);
   z1 = incr*sin(beta);

   return Cfg(z1*cos(alpha),z1*sin(alpha),z,0.0,0.0,0.0);

   // return GetRandomCfg(incr, 0.0);
   // this implementation is only true for this kind of Cfgs.

}

Cfg Cfg_free::GetRandomCfg_CenterOfMass(double *boundingBox) {
   vector<double> tmp;
   for(int i=0; i<6; ++i) {
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


bool Cfg_free::ConfigEnvironment(const Cfg &c, Environment *env) {
     vector<double> v = c.GetData();
     int robot = env->GetRobotIndex();

     // configure the robot according to current Cfg: joint parameters
     // (and base locations/orientations for free flying robots.)
     Transformation T1 = Transformation(
          Orientation(Orientation::FixedXYZ, v[5]*TWOPI, v[4]*TWOPI, v[3]*TWOPI)
          ,
          Vector3D(v[0],v[1],v[2])
     );
     // update link 1.
     env->GetMultiBody(robot)->GetFreeBody(0)->Configure(T1);
     return true;
}


bool Cfg_free::GenerateOverlapCfg(
		Environment *env,  // although env and robot is not used here,
		int robot,            // they are needed in other Cfg classes.
		Vector3D robot_start,
		Vector3D robot_goal,
		Cfg *resultCfg){

     Vector3D diff = robot_goal - robot_start;

     // pass back the Cfg for this pose.
     *resultCfg =
        Cfg(diff[0], diff[1], diff[2],
			drand48(), drand48(),drand48());
     return true;
}


vector<Cfg>
Cfg_free::GetCfgByOverlappingNormal(
	Environment* env,CollisionDetection* cd, 
	const GMSPolyhedron &polyRobot, const GMSPolyhedron &polyObst, 
	int robTri, int obsTri, 
	SID _cdsetid, CDInfo& _cdInfo,
	MultiBody * onflyRobot){

	static const double posRes = env->GetPositionRes();
        vector<Cfg> surface;
        Vector3D robotVertex[3], obstVertex[3], robotPoint, obstPoint, robotNormal, obstNormal;
	if(robTri < 0 || robTri >= polyRobot.numPolygons ||
	   obsTri < 0 || obsTri >= polyObst.numPolygons ) {
		cout << "out of range: Cfg_free::GetCfgByOverlappingNormal() " << endl;
		exit(10);
	}
	GMSPolygon *pRobot = &polyRobot.polygonList[robTri];
	GMSPolygon *pObst = &polyObst.polygonList[obsTri];
	for(int i=0; i<3; i++) {
	   robotVertex[i] = polyRobot.vertexList[pRobot->vertexList[i]];
	   obstVertex[i]  = polyObst.vertexList[pObst->vertexList[i]];
	}
	// find normals for both triangles (on robot and obstacle):
	robotNormal = pRobot->normal;
	obstNormal = pObst->normal;

	// Overlap these two normals, solve for alpha, beta, gamma of FixedXYZ rotation.
	Orientation orient;
	double dot = robotNormal.dotProduct(obstNormal);
	if(abs(dot) == 1) { // two normals parallel to each other.
	   orient = Orientation(IdentityMatrix);
	} else {
	   double cV = sqrt((1+dot)/2.0);
	   double sV = sqrt((1-dot)/2.0);
	   Vector3D vertical = robotNormal.crossProduct(obstNormal);
	   vertical.normalize();
	   orient = Orientation(cV, vertical*sV);
	}
	//double randomAngle = drand48()*3.14159*2.0;
	//orient = orient*Orientation(cos(randomAngle/2.0), obstNormal*sin(randomAngle/2.0));
	orient.ConvertType(Orientation::FixedXYZ);
	double alpha = orient.alpha, beta = orient.beta, gamma = orient.gamma;

        int trials = 0;
        while(trials++ < 10) {
	   // find a point on robot's facet and one on obstacle's facet(one of the triangles).
	   // points on edge.
	   double ran1 = drand48();
	   double ran2 = drand48();
	   robotPoint = robotVertex[rand()%3]*ran1 + robotVertex[rand()%3]*(1.-ran1);
	   obstPoint = obstVertex[rand()%3]*ran2 + obstVertex[rand()%3]*(1.-ran2);
	   //obstPoint = obstVertex[rand()%3];

	   Vector3D robotCMS = obstPoint - ( orient * robotPoint);
	   Vector3D direction(0,0,0);
	   Vector3D disp = obstNormal*(posRes*0.5);  //0.01;
	   Cfg displacement = Cfg(Vector6<double>(disp[0], disp[1], disp[2], 0, 0, 0));
	   Cfg cfgIn = Cfg(Vector6<double>(robotCMS[0], robotCMS[1], robotCMS[2],
			gamma/TWOPI, beta/TWOPI, alpha/TWOPI));
	   cfgIn.Increment(displacement);
	   if(! isCollision(cfgIn, env,cd,_cdsetid, _cdInfo, onflyRobot) ) {
	      direction = obstNormal;
	   } else {
	      cfgIn = cfgIn - displacement - displacement;
	      if(! isCollision(cfgIn, env,cd,_cdsetid, _cdInfo, onflyRobot) ) {
		  direction = -obstNormal;
	      } else {
	          orient = Orientation(Orientation::FixedXYZ, alpha+PI, beta+PI, gamma);
	          robotCMS = obstPoint - ( orient * robotPoint);
	          cfgIn = Cfg(Vector6<double>(robotCMS[0], robotCMS[1], robotCMS[2],
                        gamma/TWOPI, (beta+PI)/TWOPI, (alpha+PI)/TWOPI));
	          cfgIn.Increment(displacement);
	          if(! isCollision(cfgIn, env,cd,_cdsetid, _cdInfo, onflyRobot) ) {
            	       direction = obstNormal;
                  } else {
		       cfgIn = cfgIn - displacement - displacement;
                       if(! isCollision(cfgIn, env,cd,_cdsetid, _cdInfo, onflyRobot) ) {
                            direction = -obstNormal;
		       }
                  }
	      }
	   }
	   if(direction.magnitude() > 0) { // this means free Cfg is found.
	      surface.push_back(cfgIn);
	      break;
	   }
        }
	return surface;
}


bool Cfg_free::InNarrowPassage(
	const Cfg&c, Environment * env,CollisionDetection* cd,
	SID _cdsetid, CDInfo& _cdInfo, 
	MultiBody * onflyRobot){

	    if(c.GetData().size() != 6) {
		cout << "Error in Cfg_free::InNarrowPassage, Cfg must be rigidbody type. " << endl;
		exit(1);
	    }
            // add filter here
	    static const double posRes = env->GetPositionRes();
            double width = 2.0*posRes;
            int narrowpassageWeight = 0;
	    Vector6<double> tmp(0,0,0,0,0,0);
            for(int i=0; i<3; i++) {
                tmp[i] = width;
                Cfg incr(tmp);
                Cfg shiftL = c - incr;
                Cfg shiftR = c + incr;
                tmp[i] = 0.0;
                if(isCollision(shiftL,env,cd,_cdsetid, _cdInfo, onflyRobot) &&
                   isCollision(shiftR,env,cd,_cdsetid, _cdInfo, onflyRobot) ) {  // Inside Narrow Passage !
                     narrowpassageWeight++;
                }
            }
            double THROWpercentage = 0.5; // (0.5:walls) (0.97:alpha) (1.0:flange)
            if(narrowpassageWeight < 2  && drand48() < THROWpercentage)
                return false; // throw most of No-inside-narrow nodes away.
	    return true;
}



//===================================================================
// GenSurfaceCfgs4ObstNORMAL
//      generate nodes by overlapping two triangles' normal.
//===================================================================
vector<Cfg>
Cfg_free::GenSurfaceCfgs4ObstNORMAL
(Environment * env,CollisionDetection* cd, int obstacle, int nCfgs, 
SID _cdsetid,CDInfo& _cdInfo){

      vector<Cfg> surface;
      int robot = env->GetRobotIndex();

      GMSPolyhedron &polyRobot = env->GetMultiBody(robot)->GetFreeBody(0)->GetPolyhedron();
      GMSPolyhedron &polyObst = env->GetMultiBody(obstacle)->GetFixedBody(0)
				  ->GetWorldPolyhedron();

      int num = 0;

      while(num < nCfgs) {
	  int robotTriIndex = (int)(drand48()*polyRobot.numPolygons);
	  int obstTriIndex = (int)(drand48()*polyObst.numPolygons);
	  vector<Cfg> tmp = GetCfgByOverlappingNormal(env, cd, 
				polyRobot, polyObst,
				robotTriIndex, obstTriIndex, 
				_cdsetid, _cdInfo,
				env->GetMultiBody(robot));

	  if(!tmp.empty() && tmp[0].InBoundingBox(env)) {
		//if(! InNarrowPassage(tmp[0],env,cd,_cdsetid, env->GetMultiBody(robot))) continue;
		surface.push_back(tmp[0]);
		++num;
	  }
      }
      return surface;
}

