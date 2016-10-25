#include "Cfg_reach_cc_fixed.h"
#include "Environment.h"
#include <numeric>

#define TWO_PI 6.2831853072

Cfg_reach_cc_fixed::
Cfg_reach_cc_fixed() : Cfg_reach_cc() {
  m_dof = m_numOfJoints;
  m_v.resize(m_dof);
}

Cfg_reach_cc_fixed::
Cfg_reach_cc_fixed(const Cfg& c) {
  m_dof = m_numOfJoints;

  vector<double> _v = c.GetData();
  if(_v.size() < m_dof) {
    cout << "\n\nERROR in Cfg_reach_cc_fixed::Cfg_reach_cc_fixed(Cfg&), ";
    cout << "size of cfg data less than m_dof\n";
    exit(-1);
  }
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  NormalizeOrientation();

  link_lengths = ((Cfg_reach_cc_fixed&)c).link_lengths;
  link_orientations = ((Cfg_reach_cc_fixed&)c).link_orientations;
}

Cfg_reach_cc_fixed::
~Cfg_reach_cc_fixed() {
}

/*bool
Cfg_reach_cc_fixed::
ConfigureRobot(Environment* _env) const {
  int robot = _env->GetRobotIndex();

  for(int i=0; i<m_numOfJoints; ++i) {
    _env->GetMultiBody(robot)->GetFreeBody(i)->GetBackwardConnection(0).GetDHparameters().theta
    = m_v[i]*TWOPI;
  } //config the robot

  vector<int> link_ids;
  for(int i=0; i<_env->GetMultiBody(robot)->GetFreeBodyCount(); ++i)
    link_ids.push_back(_env->GetMultiBody(robot)->GetFreeBodyIndex(_env->GetMultiBody(robot)->GetFreeBody(i)));
  set<int> visited;
  for(vector<int>::const_iterator L = link_ids.begin(); L != link_ids.end(); ++L)
    _env->GetMultiBody(robot)->GetFreeBody(*L)->ComputeWorldTransformation(visited);

  return true;
}
*/
void
Cfg_reach_cc_fixed::
GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm, bool _norm) {
  cerr << "Warning GetRandomRay not implemented yet\n";
}

