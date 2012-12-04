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
#include "MPProblem/Geometry/MultiBody.h"
#include "MPProblem/Environment.h"

Cfg_2D_withRot::Cfg_2D_withRot(){
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(0);

  setPos(Point2d(0,0));
}

Cfg_2D_withRot::Cfg_2D_withRot(const Cfg& _c) {
  vector<double> _v;
  _v = _c.GetData();
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);

  setPos(Point2d(m_v[0], m_v[1]));
  NormalizeOrientation();
}

Cfg_2D_withRot::Cfg_2D_withRot(const Point2d _p, double theta){
  m_v.clear();
  m_v.push_back(_p[0]);
  m_v.push_back(_p[1]);
  m_v.push_back(theta);

  setPos(_p);

  NormalizeOrientation();
}

Cfg_2D_withRot::~Cfg_2D_withRot() {}

vector<Robot> Cfg_2D_withRot::GetRobots(int _numJoints){
  vector<Robot> robots;
  robots.push_back(Robot(Robot::PLANAR, Robot::ROTATIONAL, Robot::JointMap(), 0));
  return robots;
}

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


const string Cfg_2D_withRot::GetName() const {
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

Cfg*
Cfg_2D_withRot::CreateNewCfg() const {
  Cfg* tmp = new Cfg_2D_withRot();
  *tmp = *this;
  return tmp;
}

void Cfg_2D_withRot::GetRandomCfgCenterOfMass(Environment* _env, shared_ptr<Boundary> _bb) {
  m_v.clear();
  Point3d p = _bb->GetRandomPoint();
  for(size_t i=0 ;i<PosDOF();i++){
    m_v.push_back(p[i]);
  }

  for(size_t i=PosDOF(); i<m_dof; ++i)
    m_v.push_back(_bb->GetRandomValueInParameter(i-PosDOF()));
  setPos(Point2d(m_v[0], m_v[1]));
}
