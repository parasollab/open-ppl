#include "Cfg_reach_cc_fixed.h"
#include "Environment.h"
#include <numeric>

#define TWO_PI 6.2831853072

Cfg_reach_cc_fixed::
Cfg_reach_cc_fixed() : Cfg_reach_cc() {
  m_posDof = 0;
  m_dof = m_numOfJoints;
  m_v.resize(m_dof);
}

Cfg_reach_cc_fixed::
Cfg_reach_cc_fixed(int _numofjoints) : Cfg_reach_cc(_numofjoints) {
  m_posDof = 0;
  m_dof = m_numOfJoints;
  m_v.resize(m_dof);
}

Cfg_reach_cc_fixed::
Cfg_reach_cc_fixed(const Vector6D& _v) : Cfg_reach_cc(_v) {
  m_posDof = 0;
  m_dof = m_numOfJoints;
  m_v.resize(m_dof);
}

Cfg_reach_cc_fixed::
Cfg_reach_cc_fixed(const vector<double>& _v) : Cfg_reach_cc(_v) {
  m_posDof = 0;
  m_dof = m_numOfJoints;
  m_v.resize(m_dof);
}

Cfg_reach_cc_fixed::
Cfg_reach_cc_fixed(const Cfg& c) : Cfg_reach_cc(c) {
  m_posDof = 0;
  m_dof = m_numOfJoints;
  m_v.resize(m_dof);
}

Cfg_reach_cc_fixed::
Cfg_reach_cc_fixed(double x, double y, double z, 
	     double roll, double pitch, double yaw) 
  : Cfg_reach_cc(x, y, z, roll, pitch, yaw) {
  m_posDof = 0;
  m_dof = m_numOfJoints;
  m_v.resize(m_dof);
}

Cfg_reach_cc_fixed::
~Cfg_reach_cc_fixed() {
}

bool
Cfg_reach_cc_fixed::
ConfigEnvironment(Environment* _env) const {
  int robot = _env->GetRobotIndex();

  for(int i=0; i<m_numOfJoints; ++i) {
    _env->GetMultiBody(robot)->GetFreeBody(i)->GetBackwardConnection(0).GetDHparameters().theta = m_v[i]*360.0;
  } //config the robot

  vector<int> link_ids;
  for(int i=0; i<_env->GetMultiBody(robot)->GetFreeBodyCount(); ++i)
    link_ids.push_back(_env->GetMultiBody(robot)->GetFreeBodyIndex(_env->GetMultiBody(robot)->GetFreeBody(i)));
  set<int> visited;
  for(vector<int>::const_iterator L = link_ids.begin(); L != link_ids.end(); ++L)
    _env->GetMultiBody(robot)->GetFreeBody(*L)->ComputeWorldTransformation(visited);

  return true;
}

void 
Cfg_reach_cc_fixed::
GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm, bool _norm) {
  cerr << "Warning GetRandomRay not implemented yet\n";
}

