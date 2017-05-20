#include "Cfg.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/MetricUtils.h"


/*-------------------------------- Construction ------------------------------*/

Cfg::
Cfg() : Cfg(inputRobot) { }


Cfg::
Cfg(Robot* const _robot) : m_robot(_robot) {
  if(m_robot) {
    m_dofs.resize(DOF(), 0);
    if(m_robot->IsNonholonomic())
      m_vel.resize(DOF(), 0);
  }
}


Cfg::
Cfg(const Vector3d& _v, Robot* const _robot) : m_robot(_robot) {
  if(m_robot) {
    m_dofs.resize(DOF(), 0);
    for(size_t i = 0; i < PosDOF(); ++i)
      m_dofs[i] = _v[i];
    if(m_robot->IsNonholonomic())
      m_vel.resize(DOF(), 0);
  }
}


Cfg::
Cfg(const Cfg& _other) :
    m_clearanceInfo(_other.m_clearanceInfo),
    m_witnessCfg(_other.m_witnessCfg),
    m_dofs(_other.m_dofs),
    m_vel(_other.m_vel),
    m_robot(_other.m_robot),
    m_labelMap(_other.m_labelMap),
    m_statMap(_other.m_statMap) { }


Cfg::
Cfg(Cfg&& _other) :
    m_clearanceInfo(std::move(_other.m_clearanceInfo)),
    m_witnessCfg(std::move(_other.m_witnessCfg)),
    m_dofs(std::move(_other.m_dofs)),
    m_vel(std::move(_other.m_vel)),
    m_robot(_other.m_robot),
    m_labelMap(std::move(_other.m_labelMap)),
    m_statMap(std::move(_other.m_statMap)) { }

/*------------------------------- Assignment ---------------------------------*/

Cfg&
Cfg::
operator=(const Cfg& _cfg) {
  if(this != &_cfg) {
    m_dofs.clear();
    m_dofs = _cfg.GetData();
    m_vel.clear();
    m_vel = _cfg.GetVelocity();
    m_labelMap = _cfg.m_labelMap;
    m_statMap = _cfg.m_statMap;
    m_robot = _cfg.m_robot;
    m_clearanceInfo = _cfg.m_clearanceInfo;
    m_witnessCfg = _cfg.m_witnessCfg;
  }
  return *this;
}


Cfg&
Cfg::
operator=(Cfg&& _cfg) {
  if(this != &_cfg) {
    m_dofs.clear();
    m_dofs = std::move(_cfg.m_dofs);
    m_vel.clear();
    m_vel = std::move(_cfg.m_vel);
    m_labelMap = std::move(_cfg.m_labelMap);
    m_statMap = std::move(_cfg.m_statMap);
    m_robot = _cfg.m_robot;
    m_clearanceInfo = std::move(_cfg.m_clearanceInfo);
    m_witnessCfg = std::move(_cfg.m_witnessCfg);
  }
  return *this;
}


Cfg&
Cfg::
operator+=(const Cfg& _cfg) {
  for(size_t i = 0; i < DOF(); ++i)
    m_dofs[i] += _cfg[i];
  for(size_t i = 0; i < m_vel.size(); ++i)
    m_vel[i] += _cfg.m_vel[i];

  NormalizeOrientation();
  m_witnessCfg.reset();
  return *this;
}


Cfg&
Cfg::
operator-=(const Cfg& _cfg) {
  for(size_t i = 0; i < DOF(); ++i) {
    switch(GetMultiBody()->GetDOFType(i)) {
      case DofType::Positional:
      case DofType::Joint:
        m_dofs[i] -= _cfg[i];
        break;
      case DofType::Rotational:
        m_dofs[i] = DirectedAngularDistance(m_dofs[i], _cfg[i]);
        break;
    }
  }

  for(size_t i = 0; i < m_vel.size(); ++i)
    m_vel[i] -= _cfg.m_vel[i];

  NormalizeOrientation();
  m_witnessCfg.reset();
  return *this;
}


Cfg&
Cfg::
operator*=(const double _d) {
  for(size_t i = 0; i < DOF(); ++i)
    m_dofs[i] *= _d;
  for(size_t i = 0; i < m_vel.size(); ++i)
    m_vel[i] *= _d;

  NormalizeOrientation();
  m_witnessCfg.reset();
  return *this;
}


Cfg&
Cfg::
operator/=(const double _d) {
  for(size_t i = 0; i < DOF(); ++i)
    m_dofs[i] /= _d;
  for(size_t i = 0; i < m_vel.size(); ++i)
    m_vel[i] /= _d;

  NormalizeOrientation();
  m_witnessCfg.reset();
  return *this;
}

/*------------------------------- Arithmetic ---------------------------------*/

Cfg
Cfg::
operator+(const Cfg& _cfg) const {
  Cfg result = *this;
  return result += _cfg;
}


Cfg
Cfg::
operator-(const Cfg& _cfg) const {
  Cfg result = *this;
  return result -= _cfg;
}


Cfg
Cfg::
operator-() const {
  Cfg result = *this;
  for(size_t i = 0; i < DOF(); ++i)
    result[i] *= -1;
  for(size_t i = 0; i < m_vel.size(); ++i)
    result.m_vel[i] *= -1;

  result.NormalizeOrientation();
  result.m_witnessCfg.reset();
  return result;
}


Cfg
Cfg::
operator*(const double _d) const {
  Cfg result = *this;
  return result *= _d;
}


Cfg
Cfg::
operator/(const double _d) const {
  Cfg result = *this;
  return result /= _d;
}

/*-------------------------------- Equality ----------------------------------*/

bool
Cfg::
operator==(const Cfg& _cfg) const {
  // First check for same robot pointer.
  if(m_robot != _cfg.m_robot)
    return false;

  // Check all the DOFs.
  for(size_t i = 0; i < DOF(); ++i) {
    const double eps = Epsilon(m_dofs[i], _cfg[i]);
    switch(GetMultiBody()->GetDOFType(i)) {
      //regular types map to manifold R thus have no "wrap around"
      case DofType::Positional:
      case DofType::Joint:
        if(abs(m_dofs[i] - _cfg[i]) > eps)
          return false;
        break;
      //rotational types map to manifold S thus have a "wrap around"
      case DofType::Rotational:
        if(abs(DirectedAngularDistance(m_dofs[i], _cfg[i])) > eps)
          return false;
        break;
      default:
        throw RunTimeException(WHERE, "Unknown joint type found");
    }
  }

  // Check the velocities.
  for(size_t i = 0; i < m_vel.size(); ++i) {
    const double eps = Epsilon(m_vel[i], _cfg.m_vel[i]);
    if(std::abs(m_vel[i] - _cfg.m_vel[i]) > eps)
      return false;
  }

  // If we're still here, the Cfgs are equal.
  return true;
}


bool
Cfg::
operator!=(const Cfg& _cfg) const {
  return !(*this == _cfg);
}

/*------------------------------- Robot Info ---------------------------------*/

size_t
Cfg::
DOF() const {
  return GetMultiBody()->DOF();
}


size_t
Cfg::
PosDOF() const {
  return GetMultiBody()->PosDOF();
}


size_t
Cfg::
OriDOF() const {
  return GetMultiBody()->OrientationDOF();
}


size_t
Cfg::
JointDOF() const {
  return GetMultiBody()->JointDOF();
}



Robot*
Cfg::
GetRobot() const noexcept {
  return m_robot;
}


ActiveMultiBody*
Cfg::
GetMultiBody() const noexcept {
  return m_robot->GetMultiBody();
}

/*----------------------------- DOF Accessors --------------------------------*/

double&
Cfg::
operator[](const size_t _dof) noexcept {
  m_witnessCfg.reset();
  return m_dofs[_dof];
}


double
Cfg::
operator[](const size_t _dof) const noexcept {
  return m_dofs[_dof];
}


const std::vector<double>&
Cfg::
GetData() const noexcept {
  return m_dofs;
}


double&
Cfg::
Velocity(const size_t _dof) noexcept {
  return m_vel[_dof];
}


double
Cfg::
Velocity(const size_t _dof) const noexcept {
  return m_vel[_dof];
}


const std::vector<double>&
Cfg::
GetVelocity() const noexcept {
  return m_vel;
}


void
Cfg::
SetData(const vector<double>& _data) {
  // Assert that we got the correct number of DOFs.
  if(_data.size() != DOF())
    throw RunTimeException(WHERE, "Tried to set data for " +
        std::to_string(_data.size()) + " DOFs, but robot has " +
        std::to_string(DOF()) + " DOFs!");

  m_dofs = _data;
  m_witnessCfg.reset();
}


void
Cfg::
SetData(vector<double>&& _data) {
  // Assert that we got the correct number of DOFs.
  if(_data.size() != DOF())
    throw RunTimeException(WHERE, "Tried to set data for " +
        std::to_string(_data.size()) + " DOFs, but robot has " +
        std::to_string(DOF()) + " DOFs!");

  m_dofs = std::move(_data);
  m_witnessCfg.reset();
}


void
Cfg::
SetJointData(const vector<double>& _data) {
  // Assert that we got the correct number of DOFs.
  if(_data.size() != JointDOF())
    throw RunTimeException(WHERE, "Tried to set data for " +
        std::to_string(_data.size()) + " joint DOFs, but robot has " +
        std::to_string(JointDOF()) + " joint DOFs!");

  for(size_t i = 0, j = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) == DofType::Joint)
      m_dofs[i] = _data[j++];
  m_witnessCfg.reset();
}


vector<double>
Cfg::
GetNormalizedData(const Boundary* const _b) const {
  pair<vector<double>, vector<double>> range = GetMultiBody()->GetCfgLimits(_b);
  vector<double> normed;
  for(size_t i = 0; i < DOF(); ++i) {
    const double radius = (range.second[i] - range.first[i]) / 2.;
    const double center = range.first[i] + radius;
    normed.push_back((m_dofs[i] - center) / radius);
  }
  return move(normed);
}


void
Cfg::
SetNormalizedData(const vector<double>& _data, const Boundary* const _b) {
  // Assert that we got the right number of DOFs.
  if(_data.size() != DOF())
    throw RunTimeException(WHERE, "Tried to set data for " +
        std::to_string(_data.size()) + " DOFs, but robot has " +
        std::to_string(DOF()) + " DOFs!");

  pair<vector<double>, vector<double>> range = GetMultiBody()->GetCfgLimits(_b);
  for(size_t i = 0; i < DOF(); ++i) {
    const double radius = (range.second[i] - range.first[i]) / 2.;
    const double center = range.first[i] + radius;
    m_dofs[i] = _data[i] * radius + center;
  }
}


Point3d
Cfg::
GetPoint() const noexcept {
  return Point3d(m_dofs[0], m_dofs[1], PosDOF() == 3 ? m_dofs[2] : 0);
}


vector<double>
Cfg::
GetPosition() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) == DofType::Positional)
      ret.push_back(m_dofs[i]);
  return ret;
}


vector<double>
Cfg::
GetRotation() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) == DofType::Rotational)
      ret.push_back(m_dofs[i]);
  return ret;
}


vector<double>
Cfg::
GetOrientation() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) != DofType::Positional)
      ret.push_back(m_dofs[i]);
  return ret;
}


vector<double>
Cfg::
GetJoints() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetMultiBody()->GetDOFType(i) == DofType::Joint)
      ret.push_back(m_dofs[i]);
  }
  return ret;
}


vector<double>
Cfg::
GetNonJoints() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) != DofType::Joint)
      ret.push_back(m_dofs[i]);
  return ret;
}


double
Cfg::
Magnitude() const {
  double result = 0;
  for(size_t i = 0; i < DOF(); ++i)
    result += m_dofs[i] * m_dofs[i];
  return std::sqrt(result);
}


double
Cfg::
PositionMagnitude() const {
  double result = 0;
  for(size_t i = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) == DofType::Positional)
      result += m_dofs[i] * m_dofs[i];
  return std::sqrt(result);
}


double
Cfg::
OrientationMagnitude() const {
  double result = 0;
  for(size_t i = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) != DofType::Positional)
      result += m_dofs[i] * m_dofs[i];
  return std::sqrt(result);
}


Vector3d
Cfg::
LinearPosition() const {
  Vector3d out;
  for(size_t i = 0; i < PosDOF(); ++i)
    out[i] = m_dofs[i];
  return out;
}


Vector3d
Cfg::
AngularPosition() const {
  const size_t i = PosDOF();
  switch(OriDOF()) {
    case 1:
      return Vector3d(0, 0, m_dofs[i]);
    case 3:
      return Vector3d(m_dofs[i], m_dofs[i + 1], m_dofs[i + 2]);
    default:
      return Vector3d();
  }
}


Vector3d
Cfg::
LinearVelocity() const {
  if(m_vel.empty())
    return Vector3d();

  Vector3d out;
  for(size_t i = 0; i < PosDOF(); ++i)
    out[i] = m_vel[i];
  return out;
}


Vector3d
Cfg::
AngularVelocity() const {
  if(m_vel.empty())
    return Vector3d();

  const size_t i = PosDOF();
  switch(OriDOF()) {
    case 1:
      return Vector3d(0, 0, m_vel[i]);
    case 3:
      return Vector3d(m_vel[i], m_vel[i + 1], m_vel[i + 2]);
    default:
      return Vector3d();
  }
}

/*------------------------- Labels and Stats ---------------------------------*/

bool
Cfg::
GetLabel(const std::string& _label) const {
  return m_labelMap.at(_label);
}


bool
Cfg::
IsLabel(const std::string& _label) const noexcept {
  return m_labelMap.count(_label);
}


void
Cfg::
SetLabel(const std::string& _label, const bool _value) noexcept {
  m_labelMap[_label] = _value;
}


double
Cfg::
GetStat(const std::string& _stat) const {
  return m_statMap.at(_stat);
}


bool
Cfg::
IsStat(const std::string& _stat) const noexcept {
  return m_statMap.count(_stat) > 0;
}


void
Cfg::
SetStat(const std::string& _stat, const double _value) noexcept {
  m_statMap[_stat] = _value;
}


void
Cfg::
IncrementStat(const std::string& _stat, const double _value) noexcept {
  m_statMap[_stat] += _value;
}

/*--------------------------- Generation Methods -----------------------------*/

void
Cfg::
GetRandomCfg(Environment* _env, const Boundary* const _b) {
  m_witnessCfg.reset();
  size_t tries = 100;
  while(tries-- > 0) {
    m_dofs = GetMultiBody()->GetRandomCfg(_b);
    if(_env->InBounds(*this, _b)) {
      GetRandomVelocity();
      return;
    }
  }

  // throw error message and some helpful statistics
  ostringstream oss;
  oss << "GetRandomCfg not able to find anything in boundary: "
      << *_b << ". Robot radius is "
      << GetMultiBody()->GetBoundingSphereRadius() << ".";
  throw PMPLException("Boundary too small", WHERE, oss.str());
}


void
Cfg::
GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundary());
}


void
Cfg::
GetRandomVelocity() {
  // Do nothing if there is no velocity for this cfg.
  if(m_vel.empty())
    return;

  const double maxLinearVel = GetRobot()->GetMaxLinearVelocity();
  const double maxAngularVel = GetRobot()->GetMaxAngularVelocity();

  // Sample the base velocity.
  const auto baseType = GetMultiBody()->GetBaseType();
  const auto mvmtType = GetMultiBody()->GetBaseMovementType();

  auto dofRand = []() {return 2. * DRand() - 1.;};

  switch(baseType) {
    case FreeBody::BodyType::Fixed:
      break;
    case FreeBody::BodyType::Planar:
      {
        Vector2d l(dofRand);
        l = l.normalize() * DRand() * maxLinearVel;
        m_vel[0] = l[0];
        m_vel[1] = l[1];

        // If the base rotates, also sample its angular velocity.
        if(mvmtType == FreeBody::MovementType::Rotational)
          m_vel[2] = dofRand() * maxAngularVel;
      }
      break;
    case FreeBody::BodyType::Volumetric:
      {
        Vector3d l(dofRand);
        l = l.normalize() * DRand() * maxLinearVel;
        m_vel[0] = l[0];
        m_vel[1] = l[1];
        m_vel[2] = l[2];

        // If the base rotates, also sample its angular velocity.
        if(mvmtType == FreeBody::MovementType::Rotational) {
          Vector3d a(dofRand);
          a = a.normalize() * DRand() * maxAngularVel;
          m_vel[3] = a[0];
          m_vel[4] = a[1];
          m_vel[5] = a[2];
        }
      }
      break;
    default:
      throw RunTimeException(WHERE, "Unrecognized base type.");
  }

  const size_t firstJointIndex = DOF() - JointDOF();
  const size_t lastJointIndex = DOF();
  for(size_t i = firstJointIndex; i != lastJointIndex; ++i) {
    /// @TODO Extract max joint velocity from robot properly.
    const double maxJointVelocity = 1;
    m_vel[i] = dofRand() * maxJointVelocity;
  }
}


void
Cfg::
ConfigureRobot() const {
  GetMultiBody()->Configure(m_dofs);
}


void
Cfg::
IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment) {
  for(size_t i = 0; i < DOF(); ++i) {
    // Don't adjust DOFs that already have the exact goal value.
    if(m_dofs[i] == _goal[i])
      continue;

    // Adjust DOFs up to the goal value.
    switch(GetMultiBody()->GetDOFType(i)) {
      case DofType::Positional:
      case DofType::Joint:
        {
          const bool overshootsGoal = std::abs(_goal[i] - m_dofs[i]) <
                                      std::abs(_increment[i]);
          m_dofs[i] = overshootsGoal ? _goal[i] : m_dofs[i] += _increment[i];
        }
        break;
      case DofType::Rotational:
        {
          const double dist = DirectedAngularDistance(m_dofs[i], _goal[i]);
          const bool overshootsGoal = std::abs(dist) < std::abs(_increment[i]);
          m_dofs[i] = overshootsGoal ? _goal[i] :
                                       Normalize(m_dofs[i] += _increment[i]);
        }
        break;
    }
  }

  m_witnessCfg.reset();
}


void
Cfg::
FindIncrement(const Cfg& _start, const Cfg& _goal, const int _nTicks) {
  // Need positive number of ticks.
  if(_nTicks <= 0)
    throw RunTimeException(WHERE, "Divide by 0");

  // Compute the increment value for each DOF needed to go from _start to _goal
  // in _nTicks steps.
  for(size_t i = 0; i < DOF(); ++i) {
    switch(GetMultiBody()->GetDOFType(i)) {
      case DofType::Positional:
      case DofType::Joint:
        m_dofs[i] = (_goal[i] - _start[i]) / _nTicks;
        break;
      case DofType::Rotational:
        m_dofs[i] = DirectedAngularDistance(_start[i], _goal[i]) / _nTicks;
        break;
    }
  }

  NormalizeOrientation();
  m_witnessCfg.reset();
}


void
Cfg::
FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks,
    double _positionRes, double _orientationRes) {
  const Cfg diff = _goal - _start;

  *_nTicks = max(1., ceil(max(diff.PositionMagnitude() / _positionRes,
        diff.OrientationMagnitude() / _orientationRes)));

  this->FindIncrement(_start, _goal, *_nTicks);
}


void
Cfg::
WeightedSum(const Cfg& _first, const Cfg& _second, const double _weight) {
  const double firstWeight = 1 - _weight;

  for(size_t i = 0; i < DOF(); ++i)
    m_dofs[i] = _first[i] * firstWeight + _second[i] * _weight;
  for(size_t i = 0; i < m_vel.size(); ++i)
    m_dofs[i] = _first.m_vel[i] * firstWeight + _second.m_vel[i] * _weight;

  NormalizeOrientation();
  m_witnessCfg.reset();
}


void
Cfg::
GetPositionOrientationFrom2Cfg(const Cfg& _pos, const Cfg& _ori) {
  for(size_t i = 0; i < DOF(); ++i)
    m_dofs[i] = GetMultiBody()->GetDOFType(i) == DofType::Positional ? _pos[i] :
                                                                       _ori[i];

  NormalizeOrientation();
  m_witnessCfg.reset();
}

/*----------------------------------- I/O ------------------------------------*/

Robot* Cfg::inputRobot = nullptr;

void
Cfg::
Read(istream& _is) {
  // Require the Cfg to already have a robot pointer for now.
  if(!m_robot)
    throw RunTimeException(WHERE, "Cannot read in a Cfg without knowing the "
        "robot. Use inputRobot member to specify the default robot pointer.");

  // Read one DOF first. If that fails, return and rely on checking _is.fail()
  // from the call site to determine that no more Cfg's are available.
  _is >> m_dofs[0];
  if(_is.fail())
    return;

  // Read remaining DOFs.
  for(size_t i = 1; i < DOF(); ++i) {
    _is >> m_dofs[i];
    if(_is.fail()) {
      // If we fail, print the DOFS we actually read with the error message.
      string dofs;
      for(size_t k = 0; k < i; ++k)
        dofs += to_string(m_dofs[i]) + " ";

      throw ParseException(WHERE, "Failed reading values for all dofs: expected "
          + std::to_string(m_dofs.size()) + ", but read " + std::to_string(i) +
          ":\n\t" + dofs);
    }
  }

  // Read velocities, if any.
  for(size_t i = 0; i < m_vel.size(); ++i) {
    _is >> m_vel[i];
    if(_is.fail()) {
      // If we fail, print the DOFS we actually read with the error message.
      string err;
      for(size_t k = 0; k < i; ++k)
        err += to_string(m_vel[i]) + " ";

      throw ParseException(WHERE, "Failed reading values for all velocities: "
          "expected " + std::to_string(m_vel.size()) + ", but read " +
          std::to_string(i) + ":\n\t" + err);
    }

  }

  m_witnessCfg.reset();
}


void
Cfg::
Write(ostream& _os) const {
#ifdef VIZMO_MAP
  _os << "0 "; // uncomment for vizmo?
#endif
  // Write DOFs.
  _os << scientific << setprecision(17);
  for(auto i : m_dofs)
    _os << setw(25) << i << ' ';
#ifndef VIZMO_MAP
  for(auto i : m_vel)
    _os << setw(25) << i << ' ';
#endif

  // Unset scientific/precision options.
  _os.unsetf(ios_base::floatfield);
  if(_os.fail())
    throw RunTimeException(WHERE, "Failed to write to file.");
}


istream&
operator>>(istream& _is, Cfg& _cfg) {
  _cfg.Read(_is);
  return _is;
}


ostream&
operator<<(ostream& _os, const Cfg& _cfg) {
  _cfg.Write(_os);
  return _os;
}

/*--------------------------------- Helpers ----------------------------------*/

void
Cfg::
NormalizeOrientation(const int _index) noexcept {
  if(_index == -1) {
    for(size_t i = 0; i < DOF(); ++i)
      if(GetMultiBody()->GetDOFType(i) == DofType::Rotational)
        m_dofs[i] = Normalize(m_dofs[i]);
  }
  else if(GetMultiBody()->GetDOFType(_index) == DofType::Rotational) {
    // orientation index
    m_dofs[_index] = Normalize(m_dofs[_index]);
  }
}

/*----------------------------------------------------------------------------*/
