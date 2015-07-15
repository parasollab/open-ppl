#include "State.h"

#include "Matrix.h"
#include "Quaternion.h"
using namespace mathtool;

#include "Environment/ActiveMultiBody.h"
#include "Environment/Environment.h"
#include "Environment/NonHolonomicMultiBody.h"

State::
State(size_t _index) : Cfg(_index) {
  if(m_dof.size() > 0)
    m_vel.resize(m_dof[m_robotIndex], 0.0);
}

State::
State(const Cfg& _other) : Cfg(_other), m_vel(m_dof[m_robotIndex], 0) {
}

State::
State(const State& _other) : Cfg(_other), m_vel(_other.m_vel) {
}

State
State::operator=(const State& _other) {
  Cfg::operator=(_other);
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i)
    m_vel[i] = _other.m_vel[i];
  return *this;
}

bool
State::
operator==(const State& _other) const {
  bool cfgequal = Cfg::operator==(_other);
  if(!cfgequal)
    return false;

  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    double eps = Epsilon(m_vel[i], _other.m_vel[i]);
    if(abs(m_vel[i] - _other.m_vel[i]) > eps)
      return false;
  }
  return true;
}

State
State::
operator+(const State& _s) const {
  State result = *this;
  result += _s;
  return result;
}

State&
State::
operator+=(const State& _s) {
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    m_v[i] += _s.m_v[i];
    m_vel[i] += _s.m_vel[i];
  }
  NormalizeOrientation();
  return *this;
}

State
State::
operator-(const State& _s) const {
  State result = *this;
  result -= _s;
  return result;
}

State&
State::
operator-=(const State& _s) {
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == DofType::Positional || m_dofTypes[m_robotIndex][i] == DofType::Joint)
      m_v[i] -= _s.m_v[i];
    else
      m_v[i] = DirectedAngularDistance(m_v[i], _s.m_v[i]);
    m_vel[i] -= _s.m_vel[i];
  }
  NormalizeOrientation();
  return *this;
}

State
State::
operator-() const {
  State result = *this;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    result.m_v[i] = -result.m_v[i];
    result.m_vel[i] = -result.m_vel[i];
  }
  result.NormalizeOrientation();
  return result;
}

State
State::
operator*(double _d) const {
  State result = *this;
  result *= _d;
  return result;
}

State&
State::
operator*=(double _d) {
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    m_v[i] *= _d;
    m_vel[i] *= _d;
  }
  NormalizeOrientation();
  return *this;
}

State
State::
operator/(double _d) const {
  State result = *this;
  result /= _d;
  return result;
}

State&
State::
operator/=(double _d) {
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    m_v[i] /= _d;
    m_vel[i] /= _d;
  }
  NormalizeOrientation();
  return *this;
}

State
State::Apply(Environment* _env, const vector<double>& _u, double _dt) {
  Vector3d force(_u[0], _u[1], _u[2]);
  Vector3d torque(_u[3], _u[4], _u[5]);
  //cout << "Apply: " << force << " " << torque << endl;
  //cout << "dt: " << _dt << endl;
  //RK4 integration
  State x0 = F(_env, *this, force, torque);
  //cout << "x0: " << x0 << endl;
  //cout << "this + x0: " << (*this + x0) << endl;
  State x1 = F(_env, *this + x0 * (_dt / 2.0), force, torque);
  //cout << "x1: " << x1 << endl;
  State x2 = F(_env, *this + x1 * (_dt / 2.0), force, torque);
  //cout << "x2: " << x2 << endl;
  State x3 = F(_env, *this + x2 * _dt, force, torque);
  //cout << "x3: " << x3 << endl;
  return *this + (x0 + x1 * 2 + x2 * 2 + x3) * (_dt / 6.0);
}

void
State::
Read(istream& _is) {
  //first read in robot index, and then read in DOF values
  _is >> m_robotIndex;
  if(_is.fail())
    return;
  for(auto& i : m_v) {
    _is >> i;
    if(_is.fail())
      throw ParseException(WHERE, "Failed reading values for all dofs");
  }
  for(auto& i : m_vel) {
    _is >> i;
    if(_is.fail())
      throw ParseException(WHERE, "Failed reading values for all vels");
  }
}

void
State::
Write(ostream& _os) const {
  //write out robot index, and then dofs
  _os << setw(4) << m_robotIndex << ' ';
  _os << scientific << setprecision(17);
  for(const auto& i : m_v)
    _os << setw(25) << i << ' ';
  for(const auto& i : m_vel)
    _os << setw(25) << i << ' ';
  _os.unsetf(ios_base::floatfield);
}

void
State::
GetRandomCfgImpl(Environment* _env, shared_ptr<Boundary> _bb) {
  Cfg::GetRandomCfgImpl(_env, _bb);
  m_vel = static_pointer_cast<NonHolonomicMultiBody>(m_robots[m_robotIndex])->GetRandomVelocity();
}

State
State::
F(Environment* _env, const State& _s,
    const Vector3d& _force, const Vector3d& _torque) {
  shared_ptr<FreeBody> body =_env->GetRobot(_s.m_robotIndex)->GetFreeBody(0);
  State xdot;
  xdot.m_v = _s.m_vel;

  EulerAngle eq(_s.m_v[5]*PI, _s.m_v[4]*PI, _s.m_v[3]*PI);
  Matrix3x3 r;
  convertFromEuler(r, eq);

  Quaternion q, w(0, Vector3d(_s.m_vel[3], _s.m_vel[4], _s.m_vel[5]));
  convertFromMatrix(q, r);
  Quaternion qdot = 0.5 * w * q;
  EulerAngle edot;
  convertFromQuaternion(edot, qdot);
  xdot.m_v[3] = edot.gamma()/PI;
  xdot.m_v[4] = edot.beta()/PI;
  xdot.m_v[5] = edot.alpha()/PI;

  Vector3d force = r * _force;
  Vector3d torque = r * _torque;
  for(size_t i = 0; i < m_posdof[_s.m_robotIndex]; ++i)
    xdot.m_vel[i] = force[i] / body->GetMass();

  Matrix3x3 rt = r.transpose();
  Vector3d wdot = r * body->GetMoment() * rt * torque;
  xdot.m_vel[3] = wdot[0];
  xdot.m_vel[4] = wdot[1];
  xdot.m_vel[5] = wdot[2];

  return xdot;
}
