/////////////////////////////////////////////////////////////////////
//
//  Cfg_free.c
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

#include "Cfg_free.h"
#include "MultiBody.h"
#include "Environment.h"
#include "DistanceMetricMethod.h"

Cfg_free::Cfg_free() {
  m_dof = 6;
  m_posDof = 3;

  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(0);
}


Cfg_free::Cfg_free(double x, double y, double z, 
		   double roll, double pitch, double yaw) {
  m_dof = 6;
  m_posDof = 3;

  m_v.clear();
  m_v.push_back(x);
  m_v.push_back(y);
  m_v.push_back(z);
  m_v.push_back(roll);
  m_v.push_back(pitch);
  m_v.push_back(yaw);

  NormalizeOrientation();
}


Cfg_free::Cfg_free(const Vector6D& _v) {
  m_dof = 6;
  m_posDof = 3;
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  NormalizeOrientation();
}


Cfg_free::Cfg_free(const Cfg& _c) {
  m_dof = 6;
  m_posDof = 3;
  vector<double> _v;
  _v = _c.GetData();
  if((int)_v.size() < m_dof) {
    cout << "\n\nERROR in Cfg_free::Cfg_free(Cfg&), ";
    cout << "size of vector is less than " << m_dof << endl;
    exit(-1);
  }
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  NormalizeOrientation();
}



Cfg_free::~Cfg_free() {}

Vector3D Cfg_free::GetRobotCenterPosition() const {
   return Vector3D(m_v[0], m_v[1], m_v[2]);
}


const char* Cfg_free::GetName() const {
  return "Cfg_free";
}


bool Cfg_free::ConfigEnvironment(Environment* env) const {
  shared_ptr<MultiBody> mb = env->GetMultiBody(env->GetRobotIndex());
  
  // configure the robot according to current Cfg: joint parameters
  // (and base locations/orientations for free flying robots.)
  Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
  						 m_v[5]*TWOPI, 
						 m_v[4]*TWOPI, 
						 m_v[3]*TWOPI),
				     Vector3D(m_v[0],m_v[1],m_v[2]));
  // update link i
  mb->GetFreeBody(0)->Configure(T1);
  
  return true;
}


void Cfg_free::GetRandomCfg(double R, double rStep) {
  double alpha, beta, z, z1;
  
  alpha = 2.0*M_PI*DRand();
  beta  = 2.0*M_PI*DRand();
  z = R*cos(beta);
  z1 = R*sin(beta);
  
  double roll, pitch, yaw;
  roll = (2.0*rStep)*DRand() - rStep;
  pitch = (2.0*rStep)*DRand() - rStep;
  yaw = (2.0*rStep)*DRand() - rStep;
  
  m_v.clear();
  m_v.push_back(z1*cos(alpha));
  m_v.push_back(z1*sin(alpha));
  m_v.push_back(z);
  m_v.push_back(roll);
  m_v.push_back(pitch);
  m_v.push_back(yaw);
}


void Cfg_free::GetRandomCfg(Environment* _env,shared_ptr<BoundingBox> _bb) {
  Cfg::GetRandomCfg(_env,_bb);
}

void Cfg_free::GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundingBox());
}


void Cfg_free::GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm) {
  //randomly sample params
  m_v.clear();
  for(int i=0; i<DOF(); ++i)
    m_v.push_back(2.0*DRand()-1.0);

  //scale to appropriate length
  CfgType origin;
  dm->ScaleCfg(env, incr, origin, *this);

  NormalizeOrientation();
}

void Cfg_free::GetRandomCfg_CenterOfMass(Environment *_env, shared_ptr<BoundingBox> _bb) {
    
  m_v.clear();
  for(int i=0; i<m_dof; ++i)
    m_v.push_back(_bb->GetRandomValueInParameter(i));
}

void Cfg_free::GetRandomCfg_CenterOfMass(Environment *_env) {
  GetRandomCfg_CenterOfMass(_env, _env->GetBoundingBox());
}

