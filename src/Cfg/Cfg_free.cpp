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

Cfg_free::Cfg_free() {
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(0);
}

Cfg_free::Cfg_free(const Cfg& _c) {
  vector<double> _v;
  _v = _c.GetData();
  if(_v.size() < m_dof) {
    cout << "\n\nERROR in Cfg_free::Cfg_free(Cfg&), ";
    cout << "size of vector is less than " << m_dof << endl;
    exit(-1);
  }
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  NormalizeOrientation();
}

Cfg_free::~Cfg_free() {}

vector<Robot> Cfg_free::GetRobots(int _num){
  vector<Robot> robots;
  robots.push_back(Robot(Robot::VOLUMETRIC, Robot::ROTATIONAL, Robot::JointMap(), 0));
  return robots;
}

Vector3D Cfg_free::GetRobotCenterPosition() const {
   return Vector3D(m_v[0], m_v[1], m_v[2]);
}


const string Cfg_free::GetName() const {
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

Cfg*
Cfg_free::CreateNewCfg() const {
  Cfg* tmp = new Cfg_free();
  *tmp = *this;
  return tmp;
}

void 
Cfg_free::GetRandomCfgCenterOfMass(Environment* _env, shared_ptr<Boundary> _bb) {
  m_v.clear();
  Point3d p = _bb->GetRandomPoint();
  for(int i=0 ;i<3;i++){
    m_v.push_back(p[i]);   
  }
  for(size_t i=3; i<m_dof; ++i)
    m_v.push_back(_bb->GetRandomValueInParameter(i-3));
}


Vector3D 
Cfg_free::GetRobotCenterofMass(Environment* _env) const {
  ConfigEnvironment(_env);

  Vector3D com(0,0,0);
  GMSPolyhedron poly =
_env->GetMultiBody(_env->GetRobotIndex())->GetFreeBody(0)->GetWorldPolyhedron();
  for(vector<Vector3D>::const_iterator vit = poly.m_vertexList.begin(); vit != poly.m_vertexList.end(); ++vit)
    com = com + (*vit);
  com = com / poly.m_vertexList.size();
  return com;
}
