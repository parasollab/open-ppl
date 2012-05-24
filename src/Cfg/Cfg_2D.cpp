/////////////////////////////////////////////////////////////////////
//
//  Cfg_2D.cpp
//
//  General Description
//	Derived from Cfg_free. Take the simplest approach to
//	implement a 3-m_dof rigid-body moving in a 2-D	
//	work space.
//      This class was created by first copying Cfg_free class
//      and simply setting z, pitch, roll to zero
//
//  Created
//	12/21/01	Jinsuck Kim
//
/////////////////////////////////////////////////////////////////////


#include "Cfg_2D.h"
#include "MultiBody.h"
#include "Environment.h"
#include "DistanceMetricMethod.h"


// for safety & compatiaility, use 6 elements for cfg.
Cfg_2D::Cfg_2D():p(0,0){
  m_v.clear();
  for(int i=0; i<2; i++)
    m_v.push_back(0);
}

Cfg_2D::Cfg_2D(const Cfg& _c){
  vector<double> _v;
  _v = _c.GetData();
  m_v.clear();
  for (int i = 0; i < 2; i ++)
    m_v.push_back(_v[i]);
  p = Point2d(_v[0], _v[1]);
}

Cfg_2D::Cfg_2D(const Point2d _p){
  m_v.clear();
  m_v.push_back(_p[0]);
  m_v.push_back(_p[1]);
  p = _p;
}

Cfg_2D::~Cfg_2D() {}

vector<Robot> Cfg_2D::GetRobots(int _numJoints){
  vector<Robot> robots;
  robots.push_back(Robot(Robot::PLANAR, Robot::TRANSLATIONAL, Robot::JointMap(), 0));
  return robots;
}

void Cfg_2D::Read(istream& is){
  double x, y, tmp;
  is>>x>>y>>tmp>>tmp>>tmp>>tmp;
  m_v.clear();
  m_v.push_back(x);
  m_v.push_back(y);
  p = Point2d(x,y);
}

void Cfg_2D::Write(ostream& os) const{
  os<<setw(4)<<m_v[0]<<" "<<setw(4)<<m_v[1]<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" ";
}

void Cfg_2D::add(const Cfg& c1, const Cfg& c2) {
  Cfg::add(c1, c2);
  setPos(Point2d(m_v[0],m_v[1]));
}

void Cfg_2D::subtract(const Cfg& c1, const Cfg& c2) {
  Cfg::subtract(c1, c2);
  setPos(Point2d(m_v[0],m_v[1]));
}

void Cfg_2D::negative(const Cfg& c) {
  Cfg::negative(c);
  setPos(Point2d(m_v[0],m_v[1]));
}

void Cfg_2D::multiply(const Cfg& c, double s, bool _norm) {
  Cfg::multiply(c, s, _norm);
  setPos(Point2d(m_v[0],m_v[1]));
}

void Cfg_2D::divide(const Cfg& c, double s) {
  Cfg::divide(c, s);
  setPos(Point2d(m_v[0],m_v[1]));
}

void Cfg_2D::WeightedSum(const Cfg& first, const Cfg& second, double weight) {
  Cfg::WeightedSum(first, second, weight);
  setPos(Point2d(m_v[0],m_v[1]));
}

// Set a single parameter in the configuration (i.e., x,y,z,roll...)
// param = the parameter number to set
// value = the value to set the parameter as
int Cfg_2D::SetSingleParam(size_t param, double value, bool _norm) {    
  if ((param>=0) && (param<m_dof)) {
    Cfg::SetSingleParam(param, value, _norm);
    p[param] = value;
    return 1;
  } else {
    return 0;
  } 
}

// Increment a single parameter in the configuration (i.e., x,y,z,roll...)
// param = the parameter number to set
// value = the value to increment the parameter by
int Cfg_2D::IncSingleParam(size_t param, double value) {    
  if ((param>=0) && (param<m_dof)) {
    Cfg::IncSingleParam(param, value);
    p[param] += value;
    return 1;
  } else {
    return 0;
  } 
}

void Cfg_2D::Increment(const Cfg& _increment) {
  Cfg::Increment(_increment);
  setPos(Point2d(m_v[0], m_v[1]));
}

void Cfg_2D::IncrementTowardsGoal(const Cfg &goal, const Cfg &increment) {
  Cfg::IncrementTowardsGoal(goal, increment);
  setPos(Point2d(m_v[0], m_v[1]));
}
  
void Cfg_2D::FindIncrement(const Cfg& _start, const Cfg& _goal, int* n_ticks, double positionRes, double orientationRes){
  Cfg::FindIncrement(_start, _goal, n_ticks, positionRes, orientationRes);
}

void Cfg_2D::FindIncrement(const Cfg& _start, const Cfg& _goal, int n_ticks) {
  Cfg::FindIncrement(_start, _goal, n_ticks);
  setPos(Point2d(m_v[0], m_v[1]));
}

Cfg& Cfg_2D::operator=(const Cfg& _c) {
  setPos(Point2d(_c.GetData()[0], _c.GetData()[1]));
  return Cfg::operator=(_c);
}

const string Cfg_2D::GetName() const {
  return "Cfg_2D";
}

Vector3D Cfg_2D::GetRobotCenterPosition() const {
     return Vector3D(m_v[0], m_v[1], m_v[2]);
}

bool Cfg_2D::ConfigEnvironment(Environment* env) const {
  int robot = env->GetRobotIndex();
  
  // configure the robot according to current Cfg: joint parameters
  // (and base locations/orientations for free flying robots.)
  Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
						 0, 
						 0, 
						 0),
				     Vector3D(m_v[0],m_v[1],0));
  // update link 1.
  env->GetMultiBody(robot)->GetFreeBody(0)->Configure(T1);
  
  return true;
}

void Cfg_2D::GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm, bool _norm) {
  //randomly sample params
  double dist=0.0;
  m_v.clear();
  for(size_t i=0; i<m_dof; ++i) {
    m_v.push_back( double(2.0)*DRand() - double(1.0) );
    dist += pow(m_v[i],2);
  }

  //scale to appropriate length
  Cfg_2D origin;
  dm->ScaleCfg(env, incr, origin, *this, _norm);
  setPos(Point2d(m_v[0], m_v[1]));
}

void Cfg_2D::GetRandomCfgCenterOfMass(Environment *_env, shared_ptr<Boundary> _bb) {
  m_v.clear();
  Point3d p = _bb->GetRandomPoint();
  for(size_t i=0 ;i<m_dof;i++){
    m_v.push_back(p[i]);
  }
  setPos(Point2d(m_v[0], m_v[1]));
}

