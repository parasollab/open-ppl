/////////////////////////////////////////////////////////////////////
//
//  Cfg_2D.cpp
//
//  General Description
//	Derived from Cfg_free. Take the simplest approach to
//	implement a 3-dof rigid-body moving in a 2-D	
//	work space.
//      This class was created by first copying Cfg_free class
//      and simply setting z, pitch, roll to zero
//
//  Created
//	12/21/01	Jinsuck Kim
//
/////////////////////////////////////////////////////////////////////


#include "Cfg_2D.h"
#include "Cfg.h"
#include "MultiBody.h"
#include "Environment.h"
#include "GenerateMapNodes.h"
#include "util.h"



Cfg Cfg_2D::ForceItTo2D(Cfg c)
{
 return Cfg(c.GetSingleParam(0), c.GetSingleParam(1), 0, 0, 0, c.GetSingleParam(5));
}

// for safety & compatiaility, use 6 elements for cfg.
Cfg_2D::Cfg_2D() : Cfg_free() {}

Cfg_2D::~Cfg_2D() {}


Cfg Cfg_2D::GetRandomCfg(double R, double rStep){
   double alpha,beta,z, z1;

   alpha = 2.0*M_PI*drand48();
   beta  = 2.0*M_PI*drand48();
   z = R*cos(beta);
   z1 = R*sin(beta);

   double roll, pitch, yaw;
   roll = (2.0*rStep)*drand48() - rStep;
   pitch = (2.0*rStep)*drand48() - rStep;
   yaw = (2.0*rStep)*drand48() - rStep;

   return ForceItTo2D(Cfg(z1*cos(alpha),z1*sin(alpha),z,roll,pitch,yaw));
}


Cfg Cfg_2D::GetRandomRay(double incr) {
   double alpha,beta,z, z1;

   alpha = 2.0*M_PI*drand48();
   beta  = 2.0*M_PI*drand48();
   z = incr*cos(beta);
   z1 = incr*sin(beta);

   return ForceItTo2D(Cfg(z1*cos(alpha),z1*sin(alpha),z,0.0,0.0,0.0));

   // this implementation is only true for this kind of Cfgs.
}

Cfg Cfg_2D::GetRandomCfg_CenterOfMass(double *boundingBox) {
   vector<double> tmp;
   
   for(int i=0; i<6; ++i) {
      if(i<3) {
		 int k = 2*i;
         double p = boundingBox[k] +(boundingBox[k+1]-boundingBox[k])*drand48();
         tmp.push_back(p);
      }
	  else
         tmp.push_back(drand48());
   }

   return ForceItTo2D(Cfg(tmp));
}

bool Cfg_2D::GenerateOverlapCfg(
		Environment *env,  // although env and robot is not used here,
		int robot,         // they are needed in other Cfg classes.
		Vector3D robot_start,
		Vector3D robot_goal,
		Cfg *resultCfg){

     Vector3D diff = robot_goal - robot_start;

     // pass back the Cfg for this pose.
     *resultCfg =
        ForceItTo2D(Cfg(diff[0], diff[1], diff[2],
			drand48(), drand48(),drand48()));
     return true;
}

vector<Cfg>
Cfg_2D::GetCfgByOverlappingNormal(
	Environment* env,CollisionDetection* cd, 
	const GMSPolyhedron &polyRobot, const GMSPolyhedron &polyObst, 
	int robTri, int obsTri, 
	SID _cdsetid, CDInfo& _cdInfo,
	MultiBody * onflyRobot){

	static const double posRes = env->GetPositionRes();
    vector<Cfg> surface;
    Vector3D robotVertex[3], obstVertex[3], robotPoint, obstPoint, robotNormal, obstNormal;

	/////////////////////////////////////////////////////////////////////////////////////////////
	//Check if robTri and obsTri are in side the range
	if(robTri < 0 || robTri >= polyRobot.numPolygons ||
	   obsTri < 0 || obsTri >= polyObst.numPolygons ) {
		cout << "out of range: Cfg_free::GetCfgByOverlappingNormal() " << endl;
		exit(10);
	}

	/////////////////////////////////////////////////////////////////////////////////////////////
	//Get polygon accroding to index
	GMSPolygon *pRobot = &polyRobot.polygonList[robTri];
	GMSPolygon *pObst = &polyObst.polygonList[obsTri];

	/////////////////////////////////////////////////////////////////////////////////////////////
	//Get first three vertices of polygon, as a triangular
	for(int i=0; i<3; i++) {
	   //Get vertex location
	   robotVertex[i] = polyRobot.vertexList[pRobot->vertexList[i]/*vertex index*/];
	   obstVertex[i]  = polyObst.vertexList[pObst->vertexList[i]/*vertex index*/];
	}

	/////////////////////////////////////////////////////////////////////////////////////////////
	// find normals for both triangles (on robot and obstacle):
	robotNormal = pRobot->normal;
	obstNormal = pObst->normal;

	/////////////////////////////////////////////////////////////////////////////////////////////
	// Overlap these two normals, solve for alpha, beta, gamma of FixedXYZ rotation.
	Orientation orient;
	double dot = robotNormal.dotProduct(obstNormal);
	if(abs(dot) == 1) { // two normals parallel to each other.
	   orient = Orientation(IdentityMatrix);
	} 
	else 
	{
	   double cV = sqrt((1+dot)/2.0);
	   double sV = sqrt((1-dot)/2.0);
	   Vector3D vertical = robotNormal.crossProduct(obstNormal);
	   vertical.normalize();
	   orient = Orientation(cV, vertical*sV);
	}
	orient.ConvertType(Orientation::FixedXYZ);

	double alpha = orient.alpha, beta = orient.beta, gamma = orient.gamma;

	/////////////////////////////////////////////////////////////////////////////////////////////

    int trials = 0;
    while(trials++ < 10)
	{
	   // find a point on robot's facet and one on obstacle's facet(one of the triangles).
	   
		// points on edge.
	   double ran1 = drand48();
	   double ran2 = drand48();
	   //random interpolation between two random points (vertices) (robot)
	   robotPoint = robotVertex[rand()%3]*ran1 + robotVertex[rand()%3]*(1.-ran1);
	   //random interpolation between two random points (vertices) (obstacle)
	   obstPoint = obstVertex[rand()%3]*ran2 + obstVertex[rand()%3]*(1.-ran2);

	   ///I can't see what's goning on next???
	   Vector3D robotCMS = obstPoint - ( orient * robotPoint);
	   Vector3D direction(0,0,0);
	   Vector3D disp = obstNormal*(posRes*0.5);  //0.01;

	   Cfg displacement = Cfg(Vector6<double>(disp[0], disp[1], disp[2], 0, 0, 0));
	   Cfg cfgIn = Cfg(Vector6<double>(robotCMS[0], robotCMS[1], robotCMS[2],
			gamma/TWOPI, beta/TWOPI, alpha/TWOPI));
	   cfgIn.Increment(displacement);

	   if(! isCollision(cfgIn, env,cd,_cdsetid, _cdInfo, onflyRobot) ) 
	   {
	      direction = obstNormal;
	   } 
	   else 
	   {
	      cfgIn = cfgIn - displacement - displacement;
	      if(! isCollision(cfgIn, env,cd,_cdsetid, _cdInfo, onflyRobot) ) 
		  {
			direction = -obstNormal;
	      } 
		  else 
		  {
	          orient = Orientation(Orientation::FixedXYZ, alpha+PI, beta+PI, gamma);
	          robotCMS = obstPoint - ( orient * robotPoint);
	          cfgIn = Cfg(Vector6<double>(robotCMS[0], robotCMS[1], robotCMS[2],
                        gamma/TWOPI, (beta+PI)/TWOPI, (alpha+PI)/TWOPI));
	          cfgIn.Increment(displacement);
	          if(! isCollision(cfgIn, env,cd,_cdsetid, _cdInfo, onflyRobot) ) 
			  {
            	       direction = obstNormal;
              }
			  else 
			  {
		       cfgIn = cfgIn - displacement - displacement;
                       if(! isCollision(cfgIn, env,cd,_cdsetid, _cdInfo, onflyRobot) ) 
					   {
                            direction = -obstNormal;
					   }
              }
	      }
	   }

	   if(direction.magnitude() > 0) { // this means free Cfg is found.
	      surface.push_back(ForceItTo2D(cfgIn));
	      break;
	   }
    }
	return surface;
}


vector<Cfg>
Cfg_2D::GenSurfaceCfgs4ObstNORMAL
(Environment * env,CollisionDetection* cd, int obstacle, int nCfgs, 
SID _cdsetid,CDInfo& _cdInfo){

      vector<Cfg> surface;
      int robot = env->GetRobotIndex();

      GMSPolyhedron &polyRobot = env->GetMultiBody(robot)->GetFreeBody(0)->GetPolyhedron();
      GMSPolyhedron &polyObst = env->GetMultiBody(obstacle)->GetFixedBody(0)->GetWorldPolyhedron();

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
			surface.push_back(ForceItTo2D(tmp[0]));
			++num;
		  }
      }

      return surface;
}