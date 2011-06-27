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
#include "DistanceMetricMethod.h"
#include "MPProblem.h"
#include "ValidityChecker.hpp"
#include "DistanceMetrics.h"


// for safety & compatiaility, use 6 elements for cfg.
Cfg_2D::Cfg_2D():p(0,0){
  dof = 2;
  posDof = 2;
  v.clear();
  for(int i=0; i<2; i++)
    v.push_back(0);
  obst = -1;
  tag = -1;
  clearance = -1;
}

Cfg_2D::Cfg_2D(double x, double y):p(x,y){
  dof = 2;
  posDof = 2;
  v.clear();
  v.push_back(x);
  v.push_back(y);
  obst = -1;
  tag = -1;
  clearance = -1;
}

Cfg_2D::Cfg_2D(const Cfg& _c){
  dof = 2;
  posDof = 2;
  vector<double> _v;
  _v = _c.GetData();
  v.clear();
  for (int i = 0; i < 2; i ++)
    v.push_back(_v[i]);
  p = Point2d(_v[0], _v[1]);
  obst = _c.obst;
  tag = _c.tag;
  clearance = _c.clearance;
}

Cfg_2D::Cfg_2D(const Vector2d& _v){
  dof = 2;
  posDof = 2;
  v.clear();
  for (int i = 0; i < 2; i ++)
    v.push_back(_v[i]);
  p = Point2d(_v[0], _v[1]);
  obst = -1;
  tag = -1;
  clearance = -1;
}

Cfg_2D::Cfg_2D(const Point2d _p){
  dof = 2;
  posDof = 2;
  v.clear();
  v.push_back(_p[0]);
  v.push_back(_p[1]);
  p = _p;
  obst = -1;
  tag = -1;
  clearance = -1;
}

Cfg_2D::~Cfg_2D() {}

void Cfg_2D::Read(istream& is){
  double x, y, tmp;
  is>>x>>y>>tmp>>tmp>>tmp>>tmp;
  v.clear();
  v.push_back(x);
  v.push_back(y);
  p = Point2d(x,y);
}

void Cfg_2D::Write(ostream& os) const{
  os<<setw(4)<<v[0]<<" "<<setw(4)<<v[1]<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" ";
}

void Cfg_2D::add(const Cfg& c1, const Cfg& c2) {
  Cfg::add(c1, c2);
  setPos(Point2d(v[0],v[1]));
}

void Cfg_2D::subtract(const Cfg& c1, const Cfg& c2) {
  Cfg::subtract(c1, c2);
  setPos(Point2d(v[0],v[1]));
}

void Cfg_2D::negative(const Cfg& c) {
  Cfg::negative(c);
  setPos(Point2d(v[0],v[1]));
}

void Cfg_2D::multiply(const Cfg& c, double s) {
  Cfg::multiply(c, s);
  setPos(Point2d(v[0],v[1]));
}

void Cfg_2D::divide(const Cfg& c, double s) {
  Cfg::divide(c, s);
  setPos(Point2d(v[0],v[1]));
}

void Cfg_2D::WeightedSum(const Cfg& first, const Cfg& second, double weight) {
  Cfg::WeightedSum(first, second, weight);
  setPos(Point2d(v[0],v[1]));
}

// Set a single parameter in the configuration (i.e., x,y,z,roll...)
// param = the parameter number to set
// value = the value to set the parameter as
int Cfg_2D::SetSingleParam(int param, double value) {    
  if ((param>=0) && (param<dof)) {
    Cfg::SetSingleParam(param, value);
    if(param<posDof)
      p[param] = value;
    return 1;
  } else {
    return 0;
  } 
}

// Increment a single parameter in the configuration (i.e., x,y,z,roll...)
// param = the parameter number to set
// value = the value to increment the parameter by
int Cfg_2D::IncSingleParam(int param, double value) {    
  if ((param>=0) && (param<dof)) {
    Cfg::IncSingleParam(param, value);
    if(param<posDof)
      p[param] += value;
    return 1;
  } else {
    return 0;
  } 
}

void Cfg_2D::Increment(const Cfg& _increment) {
  Cfg::Increment(_increment);
  setPos(Point2d(v[0], v[1]));
}

void Cfg_2D::IncrementTowardsGoal(const Cfg &goal, const Cfg &increment) {
  Cfg::IncrementTowardsGoal(goal, increment);
  setPos(Point2d(v[0], v[1]));
}
  
void Cfg_2D::FindIncrement(const Cfg& _start, const Cfg& _goal, int* n_ticks, double positionRes, double orientationRes){
  Cfg::FindIncrement(_start, _goal, n_ticks, positionRes, orientationRes);
}

void Cfg_2D::FindIncrement(const Cfg& _start, const Cfg& _goal, int n_ticks) {
  Cfg::FindIncrement(_start, _goal, n_ticks);
  setPos(Point2d(v[0], v[1]));
}


Cfg* Cfg_2D::CreateNewCfg() const {
  Cfg* tmp = new Cfg_2D();
  tmp->equals(*this);
  return tmp;
}


Cfg* Cfg_2D::CreateNewCfg(vector<double>& data) const {
  Vector2d _data;
  if(data.size() < 2) {
    cout << "\n\nERROR in Cfg_2D::CreateNewCfg(vector<double>), ";
    cout << "size of vector is less than 2\n";
    exit(-1);
  }
  for(int i=0; i<2; i++)
    _data[i] = data[i];
  Cfg* tmp = new Cfg_2D(_data);
  return tmp;
}

void Cfg_2D::equals(const Cfg& c) {
  vector<double> _v;
  _v = c.GetData();
  if(_v.size() < 2) {
    cout << "\n\nERROR in Cfg_2D::equals(Cfg&), ";
    cout << "size of Cfg data is less than 2\n";
    exit(-1);
  }
  v.clear();
  for(int i=0; i<2; i++)
    v.push_back(_v[i]);
  
  setPos(Point2d(v[0], v[1]));

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
						 0, 
						 0, 
						 0),
				     Vector3D(v[0],v[1],0));
  // update link 1.
  env->GetMultiBody(robot)->GetFreeBody(0)->Configure(T1);
  
  return true;
}

void Cfg_2D::GetRandomCfg(Environment* env) {
  Cfg::GetRandomCfg(env);
}

void Cfg_2D::GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm) {
  //randomly sample params
  v.clear();
  for(int i=0; i<DOF(); ++i)
    v.push_back( double(2.0)*OBPRM_drand() - double(1.0) );

  //scale to appropriate length
  Cfg_2D origin;
  dm->ScaleCfg(env, incr, origin, *this);

  setPos(Point2d(v[0], v[1]));

  obst = -1;
  tag = -1;
  clearance = -1;
}

void Cfg_2D::GetRandomCfg(double R, double rStep){
  double alpha, beta, z1;
  
  alpha = 2.0*M_PI*OBPRM_drand();
  beta  = 2.0*M_PI*OBPRM_drand();
  z1 = R*sin(beta);
  
  v.clear();
  v.push_back(z1*cos(alpha));
  v.push_back(z1*sin(alpha));

  setPos(Point2d(v[0], v[1]));

  obst = -1;
  tag = -1;
  clearance = -1;
}

void Cfg_2D::GetRandomCfg_CenterOfMass(Environment *env) {
  shared_ptr<BoundingBox> boundingBox = env->GetBoundingBox();
  v.clear();
  
  for(int i=0; i<dof; ++i)
    v.push_back(boundingBox->GetRandomValueInParameter(i));

  setPos(Point2d(v[0], v[1]));

  obst = -1;
  tag = -1;
  clearance = -1;
}

bool Cfg_2D::GenerateOverlapCfg(Environment *env,  // although env and robot is not used here,
				int robot,         // they are needed in other Cfg classes.
				Vector3D robot_start,
				Vector3D robot_goal,
				Cfg *resultCfg) {

  Vector3D diff = robot_goal - robot_start;
  
  // pass back the Cfg for this pose.
  Cfg_2D cfg2d(diff[0], diff[1]);
  
  *resultCfg = cfg2d;
  
  return true;
}

void
Cfg_2D::GetCfgByOverlappingNormal(MPProblem* mp, Environment* env, Stat_Class& Stats, 
				  string vc_method,
				  const GMSPolyhedron &polyRobot, 
				  const GMSPolyhedron &polyObst, 
				  int robTri, int obsTri, 
				  CDInfo& _cdInfo,
				  shared_ptr<MultiBody> onflyRobot,
				  vector<Cfg*> surface) const {
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
    exit(1);
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
    
    Cfg_2D displacement = Cfg_2D(Vector2d(disp[0], disp[1]));
    Cfg_2D cfgIn = Cfg_2D(Vector2d(robotCMS[0], robotCMS[1]));
    cfgIn.Increment(displacement);
    
    std::string tmpStr = Callee+CallCnt;
    if(mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc_method), cfgIn, env, Stats, _cdInfo, true, &tmpStr)) {
      direction = obstNormal;
    } else {
      cfgIn.subtract(cfgIn,displacement);
      cfgIn.subtract(cfgIn,displacement);
      
      CallCnt="2";
      tmpStr = Callee+CallCnt;
      if(mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc_method), cfgIn, env, Stats, _cdInfo, true, &tmpStr)) {
	direction = -obstNormal;
      } else {
	orient = Orientation(Orientation::FixedXYZ, alpha+PI, beta+PI, gamma);
	robotCMS = obstPoint - ( orient * robotPoint);
	cfgIn = Cfg_2D(Vector2d(robotCMS[0], robotCMS[1]));
	cfgIn.Increment(displacement);
	CallCnt="3";
	tmpStr = Callee+CallCnt;
        if(mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc_method), cfgIn, env, Stats, _cdInfo, true, &tmpStr)) {
	  direction = obstNormal;
	} else {
	  cfgIn.subtract(cfgIn,displacement);
	  cfgIn.subtract(cfgIn,displacement);
	  CallCnt="4";
	  tmpStr = Callee+CallCnt;
	  if(mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc_method), cfgIn, env, Stats, _cdInfo, true, &tmpStr)) {
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



void Cfg_2D::GenSurfaceCfgs4ObstNORMAL(MPProblem* mp, Environment * env, Stat_Class& Stats,
				       string vc_method,
				       int obstacle, int nCfgs, 
				       CDInfo& _cdInfo,
				       vector<Cfg*>& surface) const{
  
  surface.clear();
  int robot = env->GetRobotIndex();
  
  GMSPolyhedron &polyRobot = env->GetMultiBody(robot)->GetFreeBody(0)->GetPolyhedron();
  GMSPolyhedron &polyObst = env->GetMultiBody(obstacle)->GetFixedBody(0)->GetWorldPolyhedron();
  
  int num = 0;
  
  while(num < nCfgs) {
    int robotTriIndex = (int)(OBPRM_drand()*polyRobot.polygonList.size());
    int obstTriIndex = (int)(OBPRM_drand()*polyObst.polygonList.size());
    
    vector<Cfg*> tmp;
    GetCfgByOverlappingNormal(mp, env, Stats, vc_method, 
			      polyRobot, polyObst,
			      robotTriIndex, obstTriIndex, 
			      _cdInfo,
			      env->GetMultiBody(robot),
			      tmp);
    
    if(!tmp.empty() && tmp[0]->InBoundingBox(env)) {
      surface.push_back(tmp[0]);
      for (size_t i= 1; i < tmp.size(); i ++)
	delete tmp[i];
      ++num;			
    }
  }
}

bool Cfg_2D::InNarrowPassage(MPProblem* mp, Environment* env, Stat_Class& Stats,
			       string vc_method,
			       CDInfo& _cdInfo, 
			       shared_ptr<MultiBody> onflyRobot) const {
  if(v.size() != 6) {
    cout << "Error in Cfg_free::InNarrowPassage, Cfg must be rigidbody type. " << endl;
    exit(1);
  }
  
  // add filter here
  static const double posRes = env->GetPositionRes();
  double width = 2.0*posRes;
  int narrowpassageWeight = 0;
  Vector2d tmp(0,0);
  
  std::string Callee(GetName()), CallL("(L)"), CallR("(R)");
  {std::string Method("-cfg_free::InNarrowPassage"); Callee = Callee+Method;}

  for(int i=0; i<2; i++) {
    tmp[i] = width;
    Cfg_2D incr(tmp);
    Cfg_2D shiftL;
    shiftL.subtract(*this, incr);
    Cfg_2D shiftR;
    shiftR.add(*this, incr);
    tmp[i] = 0.0;
    std::string tmpStr1 = Callee+CallL;
    std::string tmpStr2 = Callee+CallR;
    if((!(mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc_method), shiftL, env, Stats, _cdInfo, true, &tmpStr1))) &&
       (!(mp->GetValidityChecker()->IsValid(mp->GetValidityChecker()->GetVCMethod(vc_method), shiftR, env, Stats, _cdInfo, true, &tmpStr2)))) { // Inside Narrow Passage
      narrowpassageWeight++;
    }
  }
  
  double THROWpercentage = 0.5; // (0.5:walls) (0.97:alpha) (1.0:flange)
  if(narrowpassageWeight < 2  && OBPRM_drand() < THROWpercentage)
    return false; // throw most of No-inside-narrow nodes away.
  return true;
}
