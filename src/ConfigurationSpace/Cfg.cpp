#include "Cfg.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/MetricUtils.h"

/*---------------------------- Clearance Info --------------------------------*/

ClearanceInfo::
ClearanceInfo(Cfg* _direction, const double _clearance) :
    m_clearance(_clearance), m_direction(_direction) { }


ClearanceInfo::
~ClearanceInfo() {
  delete m_direction;
}


double
ClearanceInfo::
GetClearance() const noexcept {
  return m_clearance;
}


void
ClearanceInfo::
SetClearance(const double _clearance) noexcept {
  m_clearance = _clearance;
}


Cfg*
ClearanceInfo::
GetDirection() const noexcept {
  return m_direction;
}


void
ClearanceInfo::
SetDirection(Cfg* _direction) noexcept {
  m_direction = _direction;
}


int
ClearanceInfo::
GetObstacleId() const noexcept {
  return m_obstacleId;
}


void
ClearanceInfo::
SetObstacleId(const int _id) noexcept {
  m_obstacleId = _id;
}

/*-------------------------------- Cfg ---------------------------------------*/

Cfg::
Cfg(Robot* const _robot) : m_robot(_robot) {
  if(m_robot)
    m_v.resize(DOF(), 0);
}


Cfg::
Cfg(const Vector3d& _v, Robot* const _robot) : m_robot(_robot) {
  if(m_robot) {
    m_v.resize(DOF(), 0);
    for(size_t i = 0; i < PosDOF(); ++i)
      m_v[i] = _v[i];
  }
}


Cfg::
Cfg(const Cfg& _other) :
    m_clearanceInfo(_other.m_clearanceInfo),
    m_witnessCfg(_other.m_witnessCfg),
    m_v(_other.m_v),
    m_robot(_other.m_robot),
    m_labelMap(_other.m_labelMap),
    m_statMap(_other.m_statMap) { }


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
GetNumOfJoints() const {
  return GetMultiBody()->NumJoints();
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


Cfg&
Cfg::
operator=(const Cfg& _cfg) {
  if(this != &_cfg) {
    m_v.clear();
    m_v = _cfg.GetData();
    m_labelMap = _cfg.m_labelMap;
    m_statMap = _cfg.m_statMap;
    m_robot = _cfg.m_robot;
    m_clearanceInfo = _cfg.m_clearanceInfo;
    m_witnessCfg = _cfg.m_witnessCfg;
  }
  return *this;
}


bool
Cfg::
operator==(const Cfg& _cfg) const {
  // First check for same robot pointer.
  if(m_robot != _cfg.m_robot)
    return false;

  // If not comparing to self or to a Cfg for a different robot, we need to
  // check all the DOFs.
  for(size_t i = 0; i < DOF(); ++i) {
    const double eps = Epsilon(m_v[i], _cfg[i]);
    switch(GetMultiBody()->GetDOFType(i)) {
      //regular types map to manifold R thus have no "wrap around"
      case DofType::Positional:
      case DofType::Joint:
        if(abs(m_v[i] - _cfg[i]) > eps)
          return false;
        break;
      //rotational types map to manifold S thus have a "wrap around"
      case DofType::Rotational:
        if(abs(DirectedAngularDistance(m_v[i], _cfg[i])) > eps)
          return false;
        break;
      default:
        throw RunTimeException(WHERE, "Unknown joint type found");
    }
  }

  // If we're still here, the Cfgs are equal.
  return true;
}


bool
Cfg::
operator!=(const Cfg& _cfg) const {
  return !(*this == _cfg);
}


Cfg
Cfg::
operator+(const Cfg& _cfg) const {
  Cfg result = *this;
  result += _cfg;
  return result;
}


Cfg&
Cfg::
operator+=(const Cfg& _cfg) {
  for(size_t i = 0; i < DOF(); ++i)
    m_v[i] += _cfg[i];
  NormalizeOrientation();
  m_witnessCfg.reset();
  return *this;
}


Cfg
Cfg::
operator-(const Cfg& _cfg) const {
  Cfg result = *this;
  result -= _cfg;
  return result;
}


Cfg&
Cfg::
operator-=(const Cfg& _cfg) {
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetMultiBody()->GetDOFType(i) == DofType::Positional ||
        GetMultiBody()->GetDOFType(i) == DofType::Joint)
      m_v[i] -= _cfg[i];
    else
      m_v[i] = DirectedAngularDistance(m_v[i], _cfg.m_v[i]);
  }
  NormalizeOrientation();
  m_witnessCfg.reset();
  return *this;
}


Cfg
Cfg::
operator-() const {
  Cfg result = *this;
  for(size_t i = 0; i < DOF(); ++i)
    result[i] = -result[i];
  result.NormalizeOrientation();
  result.m_witnessCfg.reset();
  return result;
}


Cfg
Cfg::
operator*(double _d) const {
  Cfg result = *this;
  result *= _d;
  return result;
}


Cfg&
Cfg::
operator*=(double _d) {
  for(size_t i = 0; i < DOF(); ++i)
    m_v[i] *= _d;
  NormalizeOrientation();
  m_witnessCfg.reset();
  return *this;
}


Cfg
Cfg::
operator/(double _d) const {
  Cfg result = *this;
  result /= _d;
  return result;
}


Cfg&
Cfg::
operator/=(double _d) {
  for(size_t i = 0; i < DOF(); ++i)
    m_v[i] /= _d;
  NormalizeOrientation();
  m_witnessCfg.reset();
  return *this;
}


double&
Cfg::
operator[](size_t _dof) {
  assert(_dof >= 0 && _dof <= DOF());
  m_witnessCfg.reset();
  return m_v[_dof];
}


const double&
Cfg::
operator[](size_t _dof) const {
  assert(_dof >= 0 && _dof <= DOF());
  return m_v[_dof];
}


void
Cfg::
Read(istream& _is) {
  // Require the Cfg to already have a robot pointer for now.
  if(!m_robot)
    throw RunTimeException(WHERE, "Can't read in a Cfg without knowing what "
        "robot it represents. Please set the robot pointer first.");

  // Read one DOF first. If that fails, return and rely on checking _is.fail()
  // from the call site to determine that no more Cfg's are available.
  _is >> m_v[0];
  if(_is.fail())
    return;

  for(size_t i = 1; i < DOF(); ++i) {
    _is >> m_v[i];
    if(_is.fail()) {
      // If we fail, print the DOFS we actually read with the error message.
      string dofs;
      for(size_t k = 0; k < i; ++k)
        dofs += to_string(m_v[i]) + " ";
      throw ParseException(WHERE, "Failed reading values for all dofs: expected "
          + to_string(m_v.size()) + ", but read " + to_string(i) + ":\n\t" +
          dofs);
    }
  }
}


void
Cfg::
Write(ostream& _os) const {
  // Write DOFs.
  _os << scientific << setprecision(17);
  for(auto i : m_v)
    _os << setw(25) << i << ' ';

  // Unset scientific/precision options.
  _os.unsetf(ios_base::floatfield);
  if(_os.fail())
    throw RunTimeException(WHERE, "Failed to write to file.");
}


istream&
operator>>(istream& _is, Cfg& _cfg) {
  _cfg.Read(_is);
  _cfg.m_witnessCfg.reset();
  return _is;
}


ostream&
operator<<(ostream& _os, const Cfg& _cfg) {
  _cfg.Write(_os);
  return _os;
}


Point3d
Cfg::
GetPoint() const noexcept {
  return Point3d(m_v[0], m_v[1], PosDOF() == 3 ? m_v[2] : 0);
}


const std::vector<double>&
Cfg::
GetData() const noexcept {
  return m_v;
}


void
Cfg::
SetData(const vector<double>& _data) {
  // Assert that we got the correct number of DOFs.
  if(_data.size() != DOF())
    throw RunTimeException(WHERE, "Tried to set data for " +
        std::to_string(_data.size()) + " DOFs, but robot has " +
        std::to_string(DOF()) + " DOFs!");

  m_v = _data;
  m_witnessCfg.reset();
}


void
Cfg::
SetJointData(const vector<double>& _data) {
  // Assert that we got the correct number of DOFs.
  if(_data.size() != GetNumOfJoints())
    throw RunTimeException(WHERE, "Tried to set data for " +
        std::to_string(_data.size()) + " joints, but robot has " +
        std::to_string(GetNumOfJoints()) + " joints!");

  for(size_t i = 0, j = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) == DofType::Joint)
      m_v[i] = _data[j++];
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
    normed.push_back((m_v[i] - center) / radius);
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
    m_v[i] = _data[i] * radius + center;
  }
}


bool
Cfg::
GetLabel(const std::string& _label) const {
  if(!IsLabel(_label))
    throw RunTimeException(WHERE, "No label\'" + _label + "\' found.");
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
  if(!IsStat(_stat))
    throw RunTimeException(WHERE, "No stat \'" + _stat + "\' found.");
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
IncStat(const std::string& _stat, const double _value) noexcept {
  m_statMap[_stat] += _value;
}


vector<double>
Cfg::
GetPosition() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetMultiBody()->GetDOFType(i) == DofType::Positional)
      ret.push_back(m_v[i]);
  }
  return ret;
}


vector<double>
Cfg::
GetOrientation() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetMultiBody()->GetDOFType(i) != DofType::Positional)
      ret.push_back(m_v[i]);
  }
  return ret;
}


vector<double>
Cfg::
GetNonJoints() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetMultiBody()->GetDOFType(i) != DofType::Joint)
      ret.push_back(m_v[i]);
  }
  return ret;
}


vector<double>
Cfg::
GetJoints() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetMultiBody()->GetDOFType(i) == DofType::Joint)
      ret.push_back(m_v[i]);
  }
  return ret;
}


vector<double>
Cfg::
GetRotation() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetMultiBody()->GetDOFType(i) == DofType::Rotational)
      ret.push_back(m_v[i]);
  }
  return ret;
}


void
Cfg::
ResetRigidBodyCoordinates() {
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetMultiBody()->GetDOFType(i) != DofType::Joint)
      m_v[i]=0;
  }
}


double
Cfg::
Magnitude() const {
  double result = 0.0;
  for(size_t i = 0; i < DOF(); ++i)
    result += m_v[i]*m_v[i];
  return sqrt(result);
}


double
Cfg::
PositionMagnitude() const {
  double result = 0.0;
  for(size_t i = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) == DofType::Positional)
      result += m_v[i]*m_v[i];
  return sqrt(result);
}


double
Cfg::
OrientationMagnitude() const {
  double result = 0.0;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetMultiBody()->GetDOFType(i) != DofType::Positional)
      result += m_v[i]*m_v[i];
  }
  return sqrt(result);
}


Vector3d
Cfg::
GetRobotCenterPosition() const {
  Vector3d v;
  for(size_t i = 0; i < 3 && i < DOF(); i++)
    if(GetMultiBody()->GetDOFType(i) == DofType::Positional)
      v[i] = m_v[i];
  return v;
}


Vector3d
Cfg::
GetRobotCenterofMass() const {
  ConfigureRobot();
  return GetMultiBody()->GetCenterOfMass();
}

void
Cfg::
GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundary());
}

void
Cfg::
GetRandomCfg(Environment* _env, const Boundary* const _b) {
  m_witnessCfg.reset();
  // Probably should do something smarter than 3 strikes and exit.
  // eg, if it fails once, check size of bounding box vs robot radius
  // and see if user has an impossibly small (for this robot) bounding
  // box specified
  size_t tries = 100;
  while(tries-- > 0) {
    this->GetRandomCfgImpl(_env, _b);

    if(_env->InBounds(*this, _b))
      return;
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
ConfigureRobot() const {
  GetMultiBody()->Configure(m_v);
}


void
Cfg::
GetResolutionCfg(Environment* _env) {
  m_v.clear();
  double posRes = _env->GetPositionRes();
  double oriRes = _env->GetOrientationRes();

  for(size_t i = 0; i < DOF(); i++)
    if(GetMultiBody()->GetDOFType(i) == DofType::Positional)
      m_v.push_back(posRes);
    else
      m_v.push_back(oriRes);

  NormalizeOrientation();
  m_witnessCfg.reset();
}


void
Cfg::
IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment) {
  ///For Position
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetMultiBody()->GetDOFType(i) == DofType::Positional ||
        GetMultiBody()->GetDOFType(i) == DofType::Joint) {
      //If the diff between _goal and c is smaller than _increment
      if(fabs(_goal.m_v[i]-m_v[i]) < fabs(_increment.m_v[i]))
        m_v[i] = _goal.m_v[i];
      else
        m_v[i] += _increment.m_v[i];
    }
  }

  ///For Oirentation
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetMultiBody()->GetDOFType(i) == DofType::Rotational) {
      if(m_v[i] != _goal.m_v[i]) {
        double orientationIncr = _increment.m_v[i];
        double tmp = DirectedAngularDistance(m_v[i], _goal.m_v[i]);
        if(fabs(tmp) < orientationIncr) {
          m_v[i] = _goal.m_v[i];
        }
        else {
          m_v[i] += _increment.m_v[i];
          m_v[i] = Normalize(m_v[i]);
        }
      }
    }
  }

  m_witnessCfg.reset();
}


void
Cfg::
FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks,
    double _positionRes, double _orientationRes) {
  Cfg diff = _goal - _start;

  // adding two basically makes this a rough ceiling...
  *_nTicks = max(1., ceil(max(diff.PositionMagnitude() / _positionRes,
        diff.OrientationMagnitude() / _orientationRes)));

  this->FindIncrement(_start, _goal, *_nTicks);
}


void
Cfg::
FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks) {
  vector<double> incr;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetMultiBody()->GetDOFType(i) == DofType::Positional)
      incr.push_back((_goal.m_v[i] - _start.m_v[i])/_nTicks);
    else if(GetMultiBody()->GetDOFType(i) == DofType::Joint) {
      double a = _start.m_v[i];
      double b = _goal.m_v[i];
      if(_nTicks == 0)
        throw PMPLException("Divide by 0", WHERE, "Divide by 0");
      incr.push_back((b-a)/_nTicks);
    }
    else if(GetMultiBody()->GetDOFType(i) == DofType::Rotational) {
      incr.push_back(DirectedAngularDistance(_start.m_v[i], _goal.m_v[i]) /
          _nTicks);
    }
  }

  m_v = incr;
  NormalizeOrientation();
  m_witnessCfg.reset();
}


void
Cfg::
WeightedSum(const Cfg& _first, const Cfg& _second, double _weight) {
  vector<double> v;
  for(size_t i = 0; i < DOF(); ++i)
    v.push_back(_first.m_v[i]*(1.-_weight) + _second.m_v[i]*_weight);
  m_v = v;
  NormalizeOrientation();
  m_witnessCfg.reset();
}


void
Cfg::
GetPositionOrientationFrom2Cfg(const Cfg& _c1, const Cfg& _c2) {
  vector<double> v;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetMultiBody()->GetDOFType(i) == DofType::Positional)
      v.push_back(_c1.m_v[i]);
    else
      v.push_back(_c2.m_v[i]);
  }
  m_v = v;
  NormalizeOrientation();
  m_witnessCfg.reset();
}


vector<Vector3d>
Cfg::
PolyApprox() const {
  vector<Vector3d> result;
  ConfigureRobot();
  GetMultiBody()->PolygonalApproximation(result);
  return result;
}


void
Cfg::
NormalizeOrientation(int _index) {
  //Normalize the orientation to the range [-1, 1)
  if(_index == -1) {
    for(size_t i = 0; i < DOF(); ++i) {
      if(GetMultiBody()->GetDOFType(i) == DofType::Rotational)
        m_v[i] = Normalize(m_v[i]);
    }
  }
  else if(GetMultiBody()->GetDOFType(_index) == DofType::Rotational) {
    // orientation index
    m_v[_index] = Normalize(m_v[_index]);
  }
}


void
Cfg::
GetRandomCfgImpl(Environment* _env, const Boundary* const _b) {
  m_v = GetMultiBody()->GetRandomCfg(_b);
  m_witnessCfg.reset();
}

/*----------------------------------------------------------------------------*/
