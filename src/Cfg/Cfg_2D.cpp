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
#include "util.h"


// for safety & compatiaility, use 6 elements for cfg.
Cfg_2D::Cfg_2D(){
  dof = 6;
  posDof = 2;
  v.clear();
  for(int i=0; i<6; i++)
    v.push_back(0);

  obst = -1;
  tag = -1;
  clearance = -1;
}

Cfg_2D::Cfg_2D(double x, double y, double z, 
	       double roll, double pitch, double yaw) {
  dof = 6;
  posDof = 2;
  v.clear();
  v.push_back(x);
  v.push_back(y);
  v.push_back(0.0);
  v.push_back(0.0);
  v.push_back(0.0);
  v.push_back(roll);

  obst = -1;
  tag = -1;
  clearance = -1;
}

Cfg_2D::Cfg_2D(const Cfg& _c) {
  dof = 6;
  posDof = 2;
  vector<double> _v;
  _v = _c.GetData();
  v.clear();
  for (int i = 0; i < 6; i ++)
    v.push_back(_v[i]);
  Normalize_orientation();

  obst = _c.obst;
  tag = _c.tag;
  clearance = _c.clearance;
}

Cfg_2D::Cfg_2D(const Vector6D& _v) {
  dof = 6;
  posDof = 2;
  v.clear();
  for (int i = 0; i < 6; i ++)
    v.push_back(_v[i]);
  Normalize_orientation();

  obst = -1;
  tag = -1;
  clearance = -1;
}

Cfg_2D::~Cfg_2D() {}

Cfg* Cfg_2D::CreateNewCfg() const {
  Cfg* tmp = new Cfg_2D();
  tmp->equals(*this);
  return tmp;
}


Cfg* Cfg_2D::CreateNewCfg(vector<double>& data) const {
  Vector6D _data;
  if(data.size() < 6) {
    cout << "\n\nERROR in Cfg_2D::CreateNewCfg(vector<double>), ";
    cout << "size of vector is less than 6\n";
    exit(-1);
  }
  for(int i=0; i<6; i++)
    _data[i] = data[i];
  Cfg* tmp = new Cfg_2D(_data);
  return tmp;
}

void Cfg_2D::equals(const Cfg& c) {
  vector<double> _v;
  _v = c.GetData();
  if(_v.size() < 6) {
    cout << "\n\nERROR in Cfg_2D::equals(Cfg&), ";
    cout << "size of Cfg data is less than 6\n";
    exit(-1);
  }
  v.clear();
  for(int i=0; i<6; i++)
    v.push_back(_v[i]);
  obst = c.obst;
  tag = c.tag;
  clearance = c.clearance;
}

const char* Cfg_2D::GetName() const {
  return "Cfg_2D";
}

Vector3D Cfg_2D::GetRobotCenterPosition() const {
     return Vector3D(v[0], v[1], v[2]);
}

bool Cfg_2D::ConfigEnvironment(Environment* env) const {
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

void Cfg_2D::GetRandomCfg(Environment* env) {
  Cfg::GetRandomCfg(env);
}

void Cfg_2D::GetRandomCfg(double R, double rStep){
  double alpha,beta,z, z1;
  
  alpha = 2.0*M_PI*OBPRM_drand();
  beta  = 2.0*M_PI*OBPRM_drand();
  z = R*cos(beta);
  z1 = R*sin(beta);
  
  double roll, pitch, yaw;
  roll = (2.0*rStep)*OBPRM_drand() - rStep;
  pitch = (2.0*rStep)*OBPRM_drand() - rStep;
  yaw = (2.0*rStep)*OBPRM_drand() - rStep;
  
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
  ForceItTo2D();
}

void Cfg_2D::GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm) {
   Cfg_free::GetRandomRay(incr, env, dm);
   ForceItTo2D();
}

void Cfg_2D::GetRandomCfg_CenterOfMass(Environment *env) {
  BoundingBox *boundingBox = env->GetBoundingBox();
  v.clear();
  
  for(int i=0; i<dof; ++i)
    v.push_back(boundingBox->GetRandomValueInParameter(i));

  obst = -1;
  tag = -1;
  clearance = -1;
  ForceItTo2D();
}

bool Cfg_2D::GenerateOverlapCfg(Environment *env,  // although env and robot is not used here,
				int robot,         // they are needed in other Cfg classes.
				Vector3D robot_start,
				Vector3D robot_goal,
				Cfg *resultCfg) {

  Vector3D diff = robot_goal - robot_start;
  double rand[3];
  rand[0] = OBPRM_drand();
  rand[1] = OBPRM_drand();
  rand[2] = OBPRM_drand();
  
  // pass back the Cfg for this pose.
  // Cfg_2D cfg2d(diff[0], diff[1], diff[2],
  //      	  OBPRM_drand(), OBPRM_drand(),OBPRM_drand());
  Cfg_2D cfg2d(diff[0], diff[1], diff[2],
	       rand[0], rand[1], rand[2]);
  
  cfg2d.ForceItTo2D();     
  *resultCfg = cfg2d;
  
  return true;
}

void
Cfg_2D::GetCfgByOverlappingNormal(Environment* env, Stat_Class& Stats,
				  CollisionDetection* cd, 
				  const GMSPolyhedron &polyRobot, 
				  const GMSPolyhedron &polyObst, 
				  int robTri, int obsTri, 
				  CDInfo& _cdInfo,
				  shared_ptr<MultiBody> onflyRobot,
				  vector<Cfg*> surface) {
  surface.clear();
  static const double posRes = env->GetPositionRes();
  std::string Callee(GetName()), CallCnt("1");
  {std::string Method("-cfg_2d::GetCfgByOverlappingNormal"), Callee = Callee + Method;}
 
  Vector3D robotVertex[3], obstVertex[3], robotPoint, obstPoint, robotNormal, obstNormal;
  
  /////////////////////////////////////////////////////////////////////////////////////////////
  //Check if robTri and obsTri are in side the range
  if(robTri < 0 || robTri >= (int)polyRobot.polygonList.size() ||
     obsTri < 0 || obsTri >= (int)polyObst.polygonList.size() ) {
    cout << "out of range: Cfg_2D::GetCfgByOverlappingNormal() " << endl;
    exit(10);
  }
  
  /////////////////////////////////////////////////////////////////////////////////////////////
  //Get polygon accroding to index
  const GMSPolygon *pRobot = &polyRobot.polygonList[robTri];
  const GMSPolygon *pObst = &polyObst.polygonList[obsTri];
  
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
    vertical = vertical.normalize();
    orient = Orientation(cV, vertical*sV);
  }
  orient.ConvertType(Orientation::FixedXYZ);
  
  double alpha = orient.alpha, beta = orient.beta, gamma = orient.gamma;
  
  /////////////////////////////////////////////////////////////////////////////////////////////
  
  int trials = 0;
  while(trials++ < 10) {
    // find a point on robot's facet and one on obstacle's facet(one of the triangles).
    
    // points on edge.
    double ran1 = OBPRM_drand();
    double ran2 = OBPRM_drand();
    //random interpolation between two random points (vertices) (robot)
    robotPoint = robotVertex[OBPRM_lrand()%3]*ran1 + robotVertex[OBPRM_lrand()%3]*(1.-ran1);
    //random interpolation between two random points (vertices) (obstacle)
    obstPoint = obstVertex[OBPRM_lrand()%3]*ran2 + obstVertex[OBPRM_lrand()%3]*(1.-ran2);
    
    ///I can't see what's goning on next???
    Vector3D robotCMS = obstPoint - ( orient * robotPoint);
    Vector3D direction(0,0,0);
    Vector3D disp = obstNormal*(posRes*0.5);  //0.01;
    
    Cfg_2D displacement = Cfg_2D(Vector6D(disp[0], disp[1], disp[2], 0, 0, 0));
    Cfg_2D cfgIn = Cfg_2D(Vector6D(robotCMS[0], robotCMS[1], robotCMS[2],
					  gamma/TWOPI, beta/TWOPI, alpha/TWOPI));
    cfgIn.Increment(displacement);
    
    std::string tmpStr = Callee+CallCnt;
    if(! cfgIn.isCollision(env, Stats, cd, _cdInfo, onflyRobot,true, &tmpStr) ) {
      direction = obstNormal;
    } else {
      cfgIn.subtract(cfgIn,displacement);
      cfgIn.subtract(cfgIn,displacement);
      
      CallCnt="2";
      tmpStr = Callee+CallCnt;
      if(! cfgIn.isCollision(env, Stats, cd, _cdInfo, onflyRobot,true, &tmpStr) ) {
	direction = -obstNormal;
      } else {
	orient = Orientation(Orientation::FixedXYZ, alpha+PI, beta+PI, gamma);
	robotCMS = obstPoint - ( orient * robotPoint);
	cfgIn = Cfg_2D(Vector6D(robotCMS[0], robotCMS[1], robotCMS[2],
				       gamma/TWOPI, (beta+PI)/TWOPI, (alpha+PI)/TWOPI));
	cfgIn.Increment(displacement);
	CallCnt="3";
	tmpStr = Callee+CallCnt;
	if(! cfgIn.isCollision(env, Stats, cd, _cdInfo, onflyRobot, true, &tmpStr) ) {
	  direction = obstNormal;
	} else {
	  cfgIn.subtract(cfgIn,displacement);
	  cfgIn.subtract(cfgIn,displacement);
	  CallCnt="4";
	  tmpStr = Callee+CallCnt;
	  if(! cfgIn.isCollision(env, Stats, cd, _cdInfo, onflyRobot,true, &tmpStr) ) {
	    direction = -obstNormal;
	  }
	}
      }
    }
    
    if(direction.magnitude() > 0) { // this means free Cfg is found.
      cfgIn.ForceItTo2D();
      Cfg* pCfgIn = cfgIn.CreateNewCfg();
      surface.push_back(pCfgIn);
      break;
    }
  }
}



void Cfg_2D::GenSurfaceCfgs4ObstNORMAL(Environment * env, Stat_Class& Stats,
				       CollisionDetection* cd, 
				       int obstacle, int nCfgs, 
				       CDInfo& _cdInfo,
				       vector<Cfg*>& surface){
  
  surface.clear();
  int robot = env->GetRobotIndex();
  
  GMSPolyhedron &polyRobot = env->GetMultiBody(robot)->GetFreeBody(0)->GetPolyhedron();
  GMSPolyhedron &polyObst = env->GetMultiBody(obstacle)->GetFixedBody(0)->GetWorldPolyhedron();
  
  int num = 0;
  
  while(num < nCfgs) {
    int robotTriIndex = (int)(OBPRM_drand()*polyRobot.polygonList.size());
    int obstTriIndex = (int)(OBPRM_drand()*polyObst.polygonList.size());
    
    vector<Cfg*> tmp;
    GetCfgByOverlappingNormal(env, Stats, cd, 
			      polyRobot, polyObst,
			      robotTriIndex, obstTriIndex, 
			      _cdInfo,
			      env->GetMultiBody(robot),
			      tmp);
    
    if(!tmp.empty() && tmp[0]->InBoundingBox(env)) {
      ((Cfg_2D*)tmp[0])->ForceItTo2D();
      
      
      surface.push_back(tmp[0]);
      for (size_t i= 1; i < tmp.size(); i ++)
	delete tmp[i];
      ++num;			
    }
  }
}


void Cfg_2D::ForceItTo2D() {
  SetSingleParam(2, 0);
  SetSingleParam(3, 0);
  SetSingleParam(4, 0);
}
