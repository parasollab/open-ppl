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
/////////////////////////////////////////////////////////////////////

#include "Cfg_free.h"

#include "MultiBody.h"
#include "Environment.h"
#include "util.h"

Cfg_free::Cfg_free() {
  dof = 6;
  posDof = 3;

  v.clear();
  for(int i=0; i<dof; i++)
    v.push_back(0);

  obst = -1;
  tag = -1;
  clearance = -1;
}


Cfg_free::Cfg_free(double x, double y, double z, 
		   double roll, double pitch, double yaw) {
  dof = 6;
  posDof = 3;

  v.clear();
  v.push_back(x);
  v.push_back(y);
  v.push_back(z);
  v.push_back(roll);
  v.push_back(pitch);
  v.push_back(yaw);

  Normalize_orientation();

  obst = -1;
  tag = -1;
  clearance = -1;
}


Cfg_free::Cfg_free(const Vector6<double>& _v) {
  dof = 6;
  posDof = 3;
  v.clear();
  for(int i=0; i<dof; i++)
    v.push_back(_v[i]);
  Normalize_orientation();

  obst = -1;
  tag = -1;
  clearance = -1;
}


Cfg_free::Cfg_free(const Cfg& _c) {
  dof = 6;
  posDof = 3;
  vector<double> _v;
  _v = _c.GetData();
  if(_v.size() < dof) {
    cout << "\n\nERROR in Cfg_free::Cfg_free(Cfg&), ";
    cout << "size of vector is less than 6\n";
    exit(-1);
  }
  v.clear();
  for(int i=0; i<dof; i++)
    v.push_back(_v[i]);
  Normalize_orientation();

  obst = _c.obst;
  tag = _c.tag;
  clearance = _c.clearance;
}


Cfg_free::~Cfg_free() {}


void Cfg_free::equals(const Cfg& c) {
  vector<double> _v;
  _v = c.GetData();
  if(_v.size() < dof) {
    cout << "\n\nERROR in Cfg_free::equals(Cfg&), ";
    cout << "size of vector is less than 6\n";
    exit(-1);
  }
  v.clear();
  for(int i=0; i<dof; i++)
    v.push_back(_v[i]);

  obst = c.obst;
  tag = c.tag;
  clearance = c.clearance;
}


Vector3D Cfg_free::GetRobotCenterPosition() const {
   return Vector3D(v[0], v[1], v[2]);
}


const char* Cfg_free::GetName() const {
  return "Cfg_free";
}


bool Cfg_free::ConfigEnvironment(Environment* env) const {
  int robot = env->GetRobotIndex();
  
  // configure the robot according to current Cfg: joint parameters
  // (and base locations/orientations for free flying robots.)
  Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
						 v[5]*TWOPI, 
						 v[4]*TWOPI, 
						 v[3]*TWOPI),
				     Vector3D(v[0],v[1],v[2]));
  // update link 1.
  env->GetMultiBody(robot)->GetFreeBody(0)->Configure(T1);
  return true;
}


void Cfg_free::GetRandomCfg(double R, double rStep) {
  double alpha, beta, z, z1;
  
  alpha = 2.0*M_PI*drand48();
  beta  = 2.0*M_PI*drand48();
  z = R*cos(beta);
  z1 = R*sin(beta);
  
  double roll, pitch, yaw;
  roll = (2.0*rStep)*drand48() - rStep;
  pitch = (2.0*rStep)*drand48() - rStep;
  yaw = (2.0*rStep)*drand48() - rStep;
  
  v.clear();
  v.push_back(z1*cos(alpha));
  v.push_back(z1*sin(alpha));
  v.push_back(z);
  v.push_back(roll);
  v.push_back(pitch);
  v.push_back(yaw);

  obst = -1;
  tag = -1;
  clearance = -1;
}


void Cfg_free::GetRandomCfg(Environment* env) {
  Cfg::GetRandomCfg(env);
}


void Cfg_free::GetRandomRay(double incr) {
  double alpha, beta, z, z1;
  
  alpha = 2.0*M_PI*drand48();
  beta  = 2.0*M_PI*drand48();
  z = incr*cos(beta);
  z1 = incr*sin(beta);
  
  v.clear();
  v.push_back(z1*cos(alpha));
  v.push_back(z1*sin(alpha));
  v.push_back(z);
  v.push_back(0.0);
  v.push_back(0.0);
  v.push_back(0.0);

  obst = -1;
  tag = -1;
  clearance = -1;
}


bool Cfg_free::GenerateOverlapCfg(Environment *env,  // although env and robot is not used here,
				  int robot,         // they are needed in other Cfg classes.
				  Vector3D robot_start, Vector3D robot_goal,
				  Cfg *resultCfg) {
  Vector3D diff = robot_goal - robot_start;
  
  // pass back the Cfg for this pose.
  *resultCfg = Cfg_free(diff[0], diff[1], diff[2], 
			drand48(), drand48(), drand48());
  return true;
}


//===================================================================
// GenSurfaceCfgs4ObstNORMAL
//      generate nodes by overlapping two triangles' normal.
//===================================================================
void Cfg_free::GenSurfaceCfgs4ObstNORMAL(Environment* env, CollisionDetection* cd, 
					 int obstacle, int nCfgs, 
					 SID _cdsetid, CDInfo& _cdInfo, 
					 vector<Cfg*>& surface) const {
  surface.clear();
  int robot = env->GetRobotIndex();
  
  GMSPolyhedron& polyRobot = env->GetMultiBody(robot)->GetFreeBody(0)->GetPolyhedron();
  GMSPolyhedron& polyObst = env->GetMultiBody(obstacle)->GetFixedBody(0)->GetWorldPolyhedron();
  
  int num = 0;
  
  while(num < nCfgs) {
    int robotTriIndex = (int)(drand48()*polyRobot.numPolygons);
    int obstTriIndex = (int)(drand48()*polyObst.numPolygons);
  
    vector<Cfg*> tmp;  
    GetCfgByOverlappingNormal(env, cd, polyRobot, polyObst,
			      robotTriIndex, obstTriIndex, 
			      _cdsetid, _cdInfo, env->GetMultiBody(robot), 
			      tmp);
    
    if(!tmp.empty() && tmp[0]->InBoundingBox(env)) {
      surface.push_back(tmp[0]);
      for(int i=1; i<tmp.size(); i++)
	delete tmp[i];
      ++num;
    }
  }
}


void Cfg_free::GetCfgByOverlappingNormal(Environment* env, CollisionDetection* cd, 
					 const GMSPolyhedron &polyRobot, 
					 const GMSPolyhedron &polyObst, 
					 int robTri, int obsTri, 
					 SID _cdsetid, CDInfo& _cdInfo,
					 MultiBody* onflyRobot, vector<Cfg*>& surface) const {
  surface.clear();
  static const double posRes = env->GetPositionRes();
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
  GMSPolygon* pRobot = &polyRobot.polygonList[robTri];
  GMSPolygon* pObst = &polyObst.polygonList[obsTri];
  
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
  if(abs((int)dot) == 1) { // two normals parallel to each other.
    orient = Orientation(IdentityMatrix);
  } else {
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
  while(trials++ < 10) {
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
    
    Cfg_free displacement = Cfg_free(disp[0], disp[1], disp[2], 
				     0, 0, 0);
    Cfg_free cfgIn = Cfg_free(robotCMS[0], robotCMS[1], robotCMS[2],
			      gamma/TWOPI, beta/TWOPI, alpha/TWOPI);
    cfgIn.Increment(displacement);
    
    if(! cfgIn.isCollision(env, cd,_cdsetid, _cdInfo, onflyRobot) ) {
      direction = obstNormal;
    } else {
      //cfgIn = cfgIn - displacement - displacement;
      cfgIn.subtract(cfgIn, displacement);
      cfgIn.subtract(cfgIn, displacement);  
      if(! cfgIn.isCollision(env, cd,_cdsetid, _cdInfo, onflyRobot) ) {
	direction = -obstNormal;
      } else {
	orient = Orientation(Orientation::FixedXYZ, alpha+PI, beta+PI, gamma);
	robotCMS = obstPoint - ( orient * robotPoint);
	cfgIn = Cfg_free(robotCMS[0], robotCMS[1], robotCMS[2],
			 gamma/TWOPI, (beta+PI)/TWOPI, (alpha+PI)/TWOPI);
	cfgIn.Increment(displacement);
	if(! cfgIn.isCollision(env, cd,_cdsetid, _cdInfo, onflyRobot) ) {
	  direction = obstNormal;
	} else {
	  //cfgIn = cfgIn - displacement - displacement;
	  cfgIn.subtract(cfgIn, displacement);
	  cfgIn.subtract(cfgIn, displacement);
	  if(! cfgIn.isCollision(env, cd,_cdsetid, _cdInfo, onflyRobot) ) {
	    direction = -obstNormal;
	  }
	}
      }
    }
    
    if(direction.magnitude() > 0) { // this means free Cfg is found.
      Cfg* pCfgIn = cfgIn.CreateNewCfg();
      surface.push_back(pCfgIn);
      break;
    }
  }
}


bool Cfg_free::InNarrowPassage(Environment* env, CollisionDetection* cd,
			       SID _cdsetid, CDInfo& _cdInfo, 
			       MultiBody* onflyRobot) const {
  if(v.size() != 6) {
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
    Cfg_free incr(tmp);
    Cfg_free shiftL;
    shiftL.subtract(*this, incr);
    Cfg_free shiftR;
    shiftR.add(*this, incr);
    tmp[i] = 0.0;
    if(shiftL.isCollision(env, cd,_cdsetid, _cdInfo, onflyRobot) &&
       shiftR.isCollision(env, cd,_cdsetid, _cdInfo, onflyRobot) ) { // Inside Narrow Passage !
      narrowpassageWeight++;
    }
  }
  
  double THROWpercentage = 0.5; // (0.5:walls) (0.97:alpha) (1.0:flange)
  if(narrowpassageWeight < 2  && drand48() < THROWpercentage)
    return false; // throw most of No-inside-narrow nodes away.
  return true;
}


Cfg* Cfg_free::CreateNewCfg() const {
  Cfg* tmp = new Cfg_free();
  tmp->equals(*this);
  return tmp;
}


Cfg* Cfg_free::CreateNewCfg(vector<double>& data) const {
  Vector6<double> _data;
  if(data.size() < dof) {
    cout << "\n\nERROR in Cfg_free::CreateNewCfg(vector<double>), ";
    cout << "size of vector is less than 6\n";
    exit(-1);
  }
  for(int i=0; i<6; i++)
    _data[i] = data[i];
  Cfg* tmp = new Cfg_free(_data);
  return tmp;
}


void Cfg_free::GetRandomCfg_CenterOfMass(Environment *env) {
  double* boundingBox = env->GetBoundingBox();
  
  v.clear();
  for(int i=0; i<6; ++i) {
    if(i<3) {
      int k = 2*i;
      double p = boundingBox[k] +(boundingBox[k+1]-boundingBox[k])*drand48();
      v.push_back(p);
    } else
      v.push_back(drand48());
  }

  obst = -1;
  tag = -1;
  clearance = -1;
}

