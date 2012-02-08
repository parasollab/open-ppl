/////////////////////////////////////////////////////////////////////
//
//  Cfg_2D_withRot.c
//
//  General Description
//	A derived class from CfgManager. It provides some specific
//	implementation for a 6-m_dof rigid-body moving in a 3-D
//	work space.
//
//  Created
//	08/31/99	Guang Song
//
/////////////////////////////////////////////////////////////////////

#include "Cfg_2D_withRot.h"
#include "MultiBody.h"
#include "Environment.h"
#include "DistanceMetricMethod.h"


Cfg_2D_withRot::Cfg_2D_withRot(){
  m_dof = 3;
  m_posDof = 2;

  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(0);

  setPos(Point2d(0,0));
}


Cfg_2D_withRot::Cfg_2D_withRot(double x, double y, double theta) {
  m_dof = 3;
  m_posDof = 2;

  m_v.clear();
  m_v.push_back(x);
  m_v.push_back(y);
  m_v.push_back(theta);

  setPos(Point2d(x,y));
  
  NormalizeOrientation();
}


Cfg_2D_withRot::Cfg_2D_withRot(const Vector3d& _v) {
  m_dof = 3;
  m_posDof = 2;
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  
  setPos(Point2d(_v[0],_v[1]));
  NormalizeOrientation();
}


Cfg_2D_withRot::Cfg_2D_withRot(const Cfg& _c) {
  m_dof = 3;
  m_posDof = 2;
  vector<double> _v;
  _v = _c.GetData();
  if((int)_v.size() < m_dof) {
    cout << "\n\nERROR in Cfg_2D_withRot::Cfg_2D_withRot(Cfg&), ";
    cout << "size of vector is less than " << m_dof << endl;
    exit(-1);
  }
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);

  setPos(Point2d(m_v[0], m_v[1]));
  NormalizeOrientation();
}

Cfg_2D_withRot::Cfg_2D_withRot(const Point2d _p, double theta){
  m_dof = 3;
  m_posDof = 2;
  m_v.clear();
  m_v.push_back(_p[0]);
  m_v.push_back(_p[1]);
  m_v.push_back(theta);

  setPos(_p);

  NormalizeOrientation();
}

Cfg_2D_withRot::~Cfg_2D_withRot() {}

//Write configuration to output stream
void Cfg_2D_withRot::Write(ostream& os) const{
  os<<setw(4)<<m_v[0]<<" "<<setw(4)<<m_v[1]<<" "<<0<<" "<<0<<" "<<0<<" "<<setw(4)<<m_v[2]<<" ";
}
  
//Read configuration from input stream
void Cfg_2D_withRot::Read(istream& is){
  double x, y, theta, tmp;
  is>>x>>y>>tmp>>tmp>>tmp>>theta;
  m_v.clear();
  m_v.push_back(x);
  m_v.push_back(y);
  m_v.push_back(theta);

  setPos(Point2d(m_v[0], m_v[1]));
}

Vector3D Cfg_2D_withRot::GetRobotCenterPosition() const {
   return Vector3D(m_v[0], m_v[1], 0);
}


const char* Cfg_2D_withRot::GetName() const {
  return "Cfg_2D_withRot";
}


bool Cfg_2D_withRot::ConfigEnvironment(Environment* env) const {
  shared_ptr<MultiBody> mb = env->GetMultiBody(env->GetRobotIndex());
  
  // configure the robot according to current Cfg: joint parameters
  // (and base locations/orientations for free flying robots.)
  Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
  						 m_v[2]*TWOPI, 
						 0, 
						 0),
				     Vector3D(m_v[0],m_v[1],0));
  // update link i
  mb->GetFreeBody(0)->Configure(T1);
  
  return true;
}


void Cfg_2D_withRot::GetRandomCfg(double R, double rStep) {
  double alpha, beta, z1;
  
  alpha = 2.0*M_PI*DRand();
  beta  = 2.0*M_PI*DRand();
  z1 = R*sin(beta);
  
  double theta;
  theta = (2.0*rStep)*DRand() - rStep;
  
  m_v.clear();
  m_v.push_back(z1*cos(alpha));
  m_v.push_back(z1*sin(alpha));
  m_v.push_back(theta);

  setPos(Point2d(m_v[0], m_v[1]));
}


void Cfg_2D_withRot::GetRandomCfg(Environment* _env,shared_ptr<Boundary> _bb) {
  Cfg::GetRandomCfg(_env,_bb);
}

void Cfg_2D_withRot::GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundingBox());
}


void Cfg_2D_withRot::GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm) {
  //randomly sample params
  m_v.clear();
  for(int i=0; i<m_dof; ++i)
    m_v.push_back( double(2.0)*DRand() - double(1.0) );

  //scale to appropriate length
  Cfg_2D_withRot origin;
  dm->ScaleCfg(env, incr, origin, *this);

  setPos(Point2d(m_v[0], m_v[1]));
  
  NormalizeOrientation();
}

void Cfg_2D_withRot::GetRandomCfg_CenterOfMass(Environment *_env, shared_ptr<Boundary> _bb) {
    
  m_v.clear();
  Point3d p = _bb->GetRandomPoint();
  for(int i=0 ;i<m_posDof;i++){
    m_v.push_back(p[i]);
  }

  for(int i=m_posDof; i<m_dof; ++i)
    m_v.push_back(_bb->GetRandomValueInParameter(i-m_posDof));
  
  setPos(Point2d(m_v[0], m_v[1]));
}

void Cfg_2D_withRot::GetRandomCfg_CenterOfMass(Environment *_env) {
  GetRandomCfg_CenterOfMass(_env, _env->GetBoundingBox());
}

