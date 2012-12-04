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
#include "MPProblem/Geometry/MultiBody.h"
#include "MPProblem/Environment.h"

ManifoldCfg::ManifoldCfg() {
  m_v.clear();
  for(size_t i = 0; i < m_dof; i++)
    m_v.push_back(0);
}

ManifoldCfg::ManifoldCfg(const Cfg& _c) {
  vector<double> v;
  v = _c.GetData();
  m_v.clear();
  for(size_t i = 0; i < m_dof; i++)
    m_v.push_back(v[i]);
  NormalizeOrientation();
}

ManifoldCfg::~ManifoldCfg() {}

Vector3D
ManifoldCfg::GetRobotCenterPosition() const {
  double x = 0, y = 0, z = 0;
  int numRobots = m_robots.size();
  int index = 0;
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots.begin(); rit != m_robots.end(); rit++) {
    x += m_v[index];
    y += m_v[index + 1];
    if(rit->m_base == Robot::VOLUMETRIC)
      z += m_v[index + 2];
    index += 2;
    if(rit->m_base == Robot::VOLUMETRIC)
      index += 1;
    index += rit->m_joints.size();
  }

  return Vector3D(x/numRobots, y/numRobots, z/numRobots);
}


Vector3D 
ManifoldCfg::GetRobotCenterofMass(Environment* _env) const {
  ConfigEnvironment(_env);
  
  typedef vector<Robot>::iterator RIT;
  Vector3D com(0,0,0);
  int numbodies=0;
  shared_ptr<MultiBody> mb = _env->GetMultiBody(_env->GetRobotIndex());
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots.begin(); rit != m_robots.end(); rit++) {
    GMSPolyhedron poly = mb->GetFreeBody(rit->m_bodyIndex)->GetWorldPolyhedron();
    Vector3D polycom(0,0,0);
    for(vector<Vector3D>::const_iterator  vit = poly.m_vertexList.begin(); vit != poly.m_vertexList.end(); ++vit)
      polycom = polycom + (*vit);
    polycom = polycom / poly.m_vertexList.size();
    com = com + polycom;
    numbodies++;  

   for(Robot::JointIT i = rit->m_joints.begin(); i != rit->m_joints.end(); ++i){
     GMSPolyhedron poly1 = mb->GetFreeBody(i->first.second)->GetWorldPolyhedron();
     Vector3D polycom1(0,0,0);
   for(vector<Vector3D>::const_iterator vit1 = poly1.m_vertexList.begin(); vit1 != poly1.m_vertexList.end(); ++vit1)
      polycom1 = polycom1 + (*vit1);
    polycom1 = polycom1 / poly1.m_vertexList.size();
    com = com + polycom1;
    numbodies++;

}
  }

  com = com/numbodies;
  return com;
}



const string
ManifoldCfg::GetName() const {
  return "ManifoldCfg";
}

bool
ManifoldCfg::ConfigEnvironment(Environment* _env) const {
  shared_ptr<MultiBody> mb = _env->GetMultiBody(_env->GetRobotIndex());
  int index = 0;
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots.begin(); rit != m_robots.end(); rit++) {
    int posIndex = index;
    double x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0;
    if(rit->m_base != Robot::FIXED) {
      x = m_v[posIndex];
      y = m_v[posIndex + 1];
      index += 2;
      if(rit->m_base == Robot::VOLUMETRIC) {
        index++;
        z = m_v[posIndex + 2];
      }
      if(rit->m_baseMovement == Robot::ROTATIONAL) {
        if(rit->m_base == Robot::PLANAR) {
          index++;
          gamma = m_v[posIndex + 2];
        }
        else {
          index += 3;
          alpha = m_v[posIndex + 3];
          beta = m_v[posIndex + 4];
          gamma = m_v[posIndex + 5];
        }
      }
      // configure the robot according to current Cfg: joint parameters
      // (and base locations/orientations for free flying robots.)
      Transformation t1 = Transformation(Orientation(Orientation::FixedXYZ, gamma*TWOPI, beta*TWOPI, alpha*TWOPI), Vector3D(x,y,z));
      // update link i
      mb->GetFreeBody(rit->m_bodyIndex)->Configure(t1);
    }
    typedef Robot::JointMap::iterator MIT;
    for(MIT mit = rit->m_joints.begin(); mit != rit->m_joints.end(); mit++) {
      if(mit->second!=Robot::NONACTUATED) {
        size_t second = mit->first.second;
        mb->GetFreeBody(second)->GetBackwardConnection(0).GetDHparameters().theta = m_v[index]*TWOPI;
        index++;
        if(mit->second==Robot::SPHERICAL){
          mb->GetFreeBody(second)->GetBackwardConnection(0).GetDHparameters().alpha = m_v[index]*TWOPI;
          index++;
        }
      } 
    }  // config the robot
  }
  for(int i = 0; i < mb->GetFreeBodyCount(); i++) {
    shared_ptr<FreeBody> afb = mb->GetFreeBody(i);
    if(afb->ForwardConnectionCount() == 0)  // tree tips: leaves.
      afb->GetWorldTransformation();
  }
  return true;
}

Cfg*
ManifoldCfg::CreateNewCfg() const {
  Cfg* tmp = new ManifoldCfg();
  *tmp = *this;
  return tmp;
}

void
ManifoldCfg::GetRandomCfgCenterOfMass(Environment* _env, shared_ptr<Boundary> _bb) {
  m_v.clear();
  size_t index = 0;
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots.begin(); rit != m_robots.end(); rit++) {
    if(rit->m_base == Robot::PLANAR || rit->m_base == Robot::VOLUMETRIC) {
      Point3d p = _bb->GetRandomPoint();
      size_t posDOF = rit->m_base == Robot::VOLUMETRIC ? 3 : 2;
      for(size_t i = 0; i < posDOF; i++) {
        m_v.push_back(p[i]); 
      }
      if(rit->m_baseMovement == Robot::ROTATIONAL) {
        size_t oriDOF = rit->m_base == Robot::VOLUMETRIC ? 3 : 1;
        for(size_t i = 0; i < oriDOF; i++) {
          m_v.push_back(DRand()); 
        }
      }
    }
    for(Robot::JointIT i = rit->m_joints.begin(); i != rit->m_joints.end(); ++i) {
      if(i->second == Robot::REVOLUTE) {
        //double angle = _bb->GetRandomValueInParameter(i+index);
        m_v.push_back(DRand());
        index++;
      }
      else if(i->second == Robot::SPHERICAL) {
        m_v.push_back(DRand());
        m_v.push_back(DRand());
        index++;
      }
    }
  }
}

