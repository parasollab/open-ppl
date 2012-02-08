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
  m_dof = 2;
  m_posDof = 2;
  m_v.clear();
  for(int i=0; i<2; i++)
    m_v.push_back(0);
}

Cfg_2D::Cfg_2D(double x, double y):p(x,y){
  m_dof = 2;
  m_posDof = 2;
  m_v.clear();
  m_v.push_back(x);
  m_v.push_back(y);
}

Cfg_2D::Cfg_2D(const Cfg& _c){
  m_dof = 2;
  m_posDof = 2;
  vector<double> _v;
  _v = _c.GetData();
  m_v.clear();
  for (int i = 0; i < 2; i ++)
    m_v.push_back(_v[i]);
  p = Point2d(_v[0], _v[1]);
}

Cfg_2D::Cfg_2D(const Vector2d& _v){
  m_dof = 2;
  m_posDof = 2;
  m_v.clear();
  for (int i = 0; i < 2; i ++)
    m_v.push_back(_v[i]);
  p = Point2d(_v[0], _v[1]);
}

Cfg_2D::Cfg_2D(const Point2d _p){
  m_dof = 2;
  m_posDof = 2;
  m_v.clear();
  m_v.push_back(_p[0]);
  m_v.push_back(_p[1]);
  p = _p;
}

Cfg_2D::~Cfg_2D() {}

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

void Cfg_2D::multiply(const Cfg& c, double s) {
  Cfg::multiply(c, s);
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
int Cfg_2D::SetSingleParam(int param, double value) {    
  if ((param>=0) && (param<m_dof)) {
    Cfg::SetSingleParam(param, value);
    if(param<m_posDof)
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
  if ((param>=0) && (param<m_dof)) {
    Cfg::IncSingleParam(param, value);
    if(param<m_posDof)
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

const char* Cfg_2D::GetName() const {
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

void Cfg_2D::GetRandomCfg(Environment* _env,shared_ptr<Boundary> _bb) {
  Cfg::GetRandomCfg(_env,_bb);
}

void Cfg_2D::GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundingBox());
}

void Cfg_2D::GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm) {
  //randomly sample params
  m_v.clear();
  for(int i=0; i<m_dof; ++i)
    m_v.push_back( double(2.0)*DRand() - double(1.0) );

  //scale to appropriate length
  Cfg_2D origin;
  dm->ScaleCfg(env, incr, origin, *this);

  setPos(Point2d(m_v[0], m_v[1]));
}

void Cfg_2D::GetRandomCfg(double R, double rStep){
  double alpha, beta, z1;
  
  alpha = 2.0*M_PI*DRand();
  beta  = 2.0*M_PI*DRand();
  z1 = R*sin(beta);
  
  m_v.clear();
  m_v.push_back(z1*cos(alpha));
  m_v.push_back(z1*sin(alpha));

  setPos(Point2d(m_v[0], m_v[1]));
}

void Cfg_2D::GetRandomCfg_CenterOfMass(Environment *_env, shared_ptr<Boundary> _bb) {
  m_v.clear();
  Point3d p = _bb->GetRandomPoint();
  for(int i=0 ;i<m_posDof;i++){
    m_v.push_back(p[i]);
  }
  
  for(int i=m_posDof; i<m_dof; ++i)
    m_v.push_back(_bb->GetRandomValueInParameter(i-m_posDof));

  setPos(Point2d(m_v[0], m_v[1]));
}

void Cfg_2D::GetRandomCfg_CenterOfMass(Environment *_env) {
  GetRandomCfg_CenterOfMass(_env, _env->GetBoundingBox());
}

