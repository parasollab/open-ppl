/////////////////////////////////////////////////////////////////////
//
//  Cfg_free_tree.c
//
//  General Description
//	A derived template class from CfgManager. It provides some
//	specific implementation directly related to a multiple joints
//	serial robot.
//
//  Created
//	08/31/99	Guang Song
/////////////////////////////////////////////////////////////////////

#include "Cfg_free_tree.h"
#include "MultiBody.h"
#include "Environment.h"

int Cfg_free_tree::m_numOfJoints;

Cfg_free_tree::Cfg_free_tree(){
  m_dof = 6 + m_numOfJoints;
  m_posDof = 3;
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(0);
};

Cfg_free_tree::~Cfg_free_tree(){}

Cfg_free_tree::Cfg_free_tree(int _numofJoints) {
  if(m_numOfJoints != _numofJoints ) {
    cout << "\n\nERROR in Cfg_free_tree::Cfg_free_tree(int), cannot change numofJoints\n";
    exit(-1);
  }
  
  m_dof = 6 + m_numOfJoints;
  m_posDof = 3;
  
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(0);
  
  NormalizeOrientation(); 
}

Cfg_free_tree::Cfg_free_tree(double x, double y, double z, 
		   double roll, double pitch, double yaw) {
  cout << "\n\nERROR in Cfg_free_tree::Cfg_free_tree(double,double,double,double,double,double), not implement yet.\n";
  exit(-1);
}

Cfg_free_tree::Cfg_free_tree(const Vector6D& _v) {
  cout << "\n\nERROR in Cfg_free_tree::Cfg_free_tree(Vector6D), not implement yet.\n";
  exit(-1);
}

Cfg_free_tree::Cfg_free_tree(const vector<double> &_v){
  m_dof = 6 + m_numOfJoints;
  m_posDof = 3;
  if((int)_v.size() < m_dof) {
    cout << "\n\nERROR in Cfg_free_tree::Cfg_free_tree(vector<double>), ";
    cout << "size of vector less than m_dof\n";
    exit(-1);
  }
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  NormalizeOrientation();

};

Cfg_free_tree::Cfg_free_tree(const Cfg& _c) {
  m_dof = 6 + m_numOfJoints;
  m_posDof = 3;
  vector<double> _v;
  _v = _c.GetData();
  if((int)_v.size() < m_dof) {
    cout << "\n\nERROR in Cfg_free_tree::Cfg_free_tree(Cfg&), ";
    cout << "size of cfg data less than m_dof\n";
    exit(-1);
  }
  m_v.clear();
  for(int i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  NormalizeOrientation();
}

void Cfg_free_tree::GetRandomCfg(Environment* env) {
  Cfg::GetRandomCfg(env);
}

const char* Cfg_free_tree::GetName() const {
  return "Cfg_free_tree";
}

Vector3D Cfg_free_tree::GetRobotCenterPosition() const {
  return Vector3D(m_v[0], m_v[1], m_v[2]);
}


Vector3D Cfg_free_tree::GetRobotCenterofMass(Environment* env) const {
  ConfigEnvironment(env);

  Vector3D com(0,0,0);
  shared_ptr<MultiBody> mb = env->GetMultiBody(env->GetRobotIndex());
  for(int i=0; i<=m_numOfJoints; ++i) {
    GMSPolyhedron poly = mb->GetFreeBody(i)->GetWorldPolyhedron();
    Vector3D poly_com(0,0,0);
    for(vector<Vector3D>::const_iterator V = poly.vertexList.begin(); V != poly.vertexList.end(); ++V)
      poly_com = poly_com + (*V);
    poly_com = poly_com / poly.vertexList.size();
    com = com + poly_com;
  }
  com = com / (m_numOfJoints+1);

  return com;
}


void Cfg_free_tree::GetRandomCfg(double R, double rStep){
  double alpha,beta,z, z1;
  double jointAngle;
  
  alpha = 2.0*M_PI*DRand();
  beta  = 2.0*M_PI*DRand();
  z = R*cos(beta);
  z1 = R*sin(beta);

  double roll, pitch, yaw;
  roll = (2.0*rStep)*DRand() - rStep;
  pitch = (2.0*rStep)*DRand() - rStep;
  yaw = (2.0*rStep)*DRand() - rStep;
  
  Vector6D base(z1*cos(alpha),z1*sin(alpha),z,roll,pitch,yaw);
  
  int i;
  m_v.clear();
  for( i=0; i<6; ++i)
    m_v.push_back(base[i]);
  for(i=0; i<m_numOfJoints; i++) {
    jointAngle = (2.0*rStep)*DRand() - rStep;
    // or: jointAngle = 0.0; I am not sure which is more reasonable now. Guang
    m_v.push_back(jointAngle);
  }
}

void Cfg_free_tree::GetRandomCfg_CenterOfMass(Environment *env) {
  // this is not EXACTLY accurate, ok with most cases ... TO DO
  // To be accurate, one has to make sure every link is inside the given BB,
  // but here only the base link is taken care of. It is almost fine since
  // a little 'bigger' BB will contain all links.
  
  shared_ptr<BoundingBox> boundingBox = env->GetBoundingBox();
  m_v.clear();
  for (int i = 0 ; i < m_dof ; ++i)
    m_v.push_back(boundingBox->GetRandomValueInParameter(i));
}


bool Cfg_free_tree::ConfigEnvironment(Environment *_env) const {
  int robot = _env->GetRobotIndex();
     
  // configure the robot according to current Cfg: joint parameters
  // (and base locations/orientations for free flying robots.)
  Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
						 m_v[5]*TWOPI, 
						 m_v[4]*TWOPI, 
						 m_v[3]*TWOPI), // RPY
				     Vector3D(m_v[0],m_v[1],m_v[2]));
  
  _env->GetMultiBody(robot)->GetFreeBody(0)->Configure(T1);  // update link 1.
  int i;
  for( i=0; i<m_numOfJoints; i++) {
    _env->GetMultiBody(robot)->GetFreeBody(i+1)
      ->GetBackwardConnection(0).GetDHparameters().theta = m_v[i+6]*360.0;
  }  // config the robot
  
  
  for(i=0; i<_env->GetMultiBody(robot)->GetFreeBodyCount(); i++) {
    shared_ptr<FreeBody> afb = _env->GetMultiBody(robot)->GetFreeBody(i);
    if(afb->ForwardConnectionCount() == 0)  // tree tips: leaves.
      afb->GetWorldTransformation();
  }
  
  // since Transformation is calculated in recursive manner, only
  // let the last link(or Freebody) call getWorldTransformation will
  // automatically calculate the transformations for all previous links.
  
  // when all worldTransformations are recalculated by using new cfg, the
  // config of the whole robot is updated.
  return true;
}

