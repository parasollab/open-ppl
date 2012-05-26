/////////////////////////////////////////////////////////////////////
//
//  ManifoldCfg.cpp
//
//  General Description
//	A derived class from Cfg. It provides for a generalized 
//      d-dimension configuration space and any robot system
//	should be representable within it.
//
/////////////////////////////////////////////////////////////////////

#include "ManifoldCfg.h"
#include "MultiBody.h"
#include "Environment.h"
#include "DistanceMetricMethod.h"

ManifoldCfg::ManifoldCfg() {
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(0);
}

ManifoldCfg::ManifoldCfg(const Cfg& _c) {
  vector<double> v;
  v = _c.GetData();
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(v[i]);
  NormalizeOrientation();
}

ManifoldCfg::~ManifoldCfg() {}

Vector3D ManifoldCfg::GetRobotCenterPosition() const {
  double x=0, y=0, z=0;
  int numRobots = m_robots.size();
  int index = 0;
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots.begin(); rit!=m_robots.end(); rit++) {
    x += m_v[index];
    y += m_v[index+1];
    if(rit->m_base==Robot::VOLUMETRIC)
      z += m_v[index+2];
    index+=2;
    if(rit->m_base==Robot::VOLUMETRIC)
      index+=1;
    index+=rit->m_joints.size();
  }

  return Vector3D(x/numRobots, y/numRobots, z/numRobots);
}

const string ManifoldCfg::GetName() const {
  return "ManifoldCfg";
}

bool ManifoldCfg::ConfigEnvironment(Environment* _env) const {
  shared_ptr<MultiBody> mb = _env->GetMultiBody(_env->GetRobotIndex());
  int index=0; 
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots.begin(); rit!=m_robots.end(); rit++) {
    int posIndex=index;
    double x=0, y=0, z=0, alpha=0, beta=0, gamma=0;
    if(rit->m_base != Robot::FIXED){
      x = m_v[posIndex];
      y = m_v[posIndex+1];
      index+=2;
      if(rit->m_base==Robot::VOLUMETRIC){
        index++;
        z = m_v[posIndex+2];
      }
      if(rit->m_baseMovement == Robot::ROTATIONAL){
        if(rit->m_base==Robot::PLANAR){
          index++;
          gamma = m_v[posIndex+2];
        }
        else{
          index+=3;
          alpha = m_v[posIndex+3];
          beta = m_v[posIndex+4];
          gamma = m_v[posIndex+5];
        }
      }
    }
    // configure the robot according to current Cfg: joint parameters
    // (and base locations/orientations for free flying robots.)
    Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
          gamma*TWOPI, beta*TWOPI, alpha*TWOPI), Vector3D(x,y,z));
    // update link i
    mb->GetFreeBody(rit->m_bodyIndex)->Configure(T1);
    typedef Robot::JointMap::iterator MIT;
    for(MIT mit = rit->m_joints.begin(); mit!=rit->m_joints.end(); mit++){
      size_t second = mit->first.second;
      mb->GetFreeBody(second)->GetBackwardConnection(0).GetDHparameters().theta = m_v[index]*TWOPI;
      index++;
      if(mit->second==Robot::SPHERICAL){
        mb->GetFreeBody(second)->GetBackwardConnection(0).GetDHparameters().alpha = m_v[index]*TWOPI;
        index++;
      }
    }  // config the robot
  }
  for(int i=0; i<mb->GetFreeBodyCount(); i++) {
    shared_ptr<FreeBody> afb = mb->GetFreeBody(i);
    if(afb->ForwardConnectionCount() == 0)  // tree tips: leaves.
      afb->GetWorldTransformation();
  }
  return true;
}

void ManifoldCfg::GetRandomRay(double _incr, Environment* _env, shared_ptr<DistanceMetricMethod> _dm, bool _norm) {
  //randomly sample params
  m_v.clear();
  for(size_t i=0; i<DOF(); ++i)
    m_v.push_back(2.0*DRand()-1.0);

  //scale to appropriate length
  ManifoldCfg origin;
  _dm->ScaleCfg(_env, _incr, origin, *this);
  if(_norm)
    NormalizeOrientation();
}

void ManifoldCfg::GetRandomCfgCenterOfMass(Environment* _env, shared_ptr<Boundary> _bb) {
  m_v.clear();
  size_t index = 0;
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots.begin(); rit!=m_robots.end(); rit++){
    if(rit->m_base == Robot::PLANAR || rit->m_base == Robot::VOLUMETRIC){
      Point3d p = _bb->GetRandomPoint();
      size_t posDOF = rit->m_base == Robot::VOLUMETRIC ? 3 : 2;
      for(size_t i=0 ;i<posDOF;i++){
        m_v.push_back(p[i]); 
      }
      if(rit->m_baseMovement == Robot::ROTATIONAL){
        size_t oriDOF = rit->m_base == Robot::VOLUMETRIC ? 3 : 1;
        for(size_t i=0 ;i<oriDOF;i++){
          m_v.push_back(DRand()); 
        }
      }
    }
    for(Robot::JointIT i=rit->m_joints.begin(); i!=rit->m_joints.end(); ++i){
      if(i->second == Robot::REVOLUTE){
        //double angle = _bb->GetRandomValueInParameter(i+index);
        m_v.push_back(DRand());
        index++;
      }
      else if(i->second == Robot::SPHERICAL){
        m_v.push_back(DRand());
        m_v.push_back(DRand());
        index++;
      }
    }
  }
}

