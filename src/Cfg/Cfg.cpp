#include "Cfg.h"

#include "Environment/ActiveMultiBody.h"
#include "Environment/Environment.h"
#include "Utilities/MetricUtils.h"

/*---------------------------- Clearance Info --------------------------------*/

ClearanceInfo::
~ClearanceInfo() {
  delete m_direction;
}

/*-------------------------------- Cfg ---------------------------------------*/

vector<shared_ptr<ActiveMultiBody>> Cfg::m_robots;
shared_ptr<ActiveMultiBody> Cfg::m_pointRobot;


Cfg::
Cfg(size_t _robotIndex) : m_robotIndex(_robotIndex) {
  if(!m_robots.empty())
    m_v.resize(DOF(), 0);
}


Cfg::
Cfg(const Vector3d& _v, size_t _robotIndex) : m_robotIndex(_robotIndex) {
  if(!m_robots.empty())
    m_v.resize(DOF(), 0);
  for(size_t i = 0; i < PosDOF(); ++i)
    m_v[i] = _v[i];
}


Cfg::
Cfg(const Cfg& _other) :
    m_v(_other.m_v),
    m_robotIndex(_other.m_robotIndex),
    m_labelMap(_other.m_labelMap),
    m_statMap(_other.m_statMap),
    m_clearanceInfo(_other.m_clearanceInfo),
    m_witnessCfg(_other.m_witnessCfg) {}


size_t
Cfg::
DOF() const {
  return GetRobot()->DOF();
}


size_t
Cfg::
PosDOF() const {
  return GetRobot()->PosDOF();
}


size_t
Cfg::
GetNumOfJoints() const {
  return GetRobot()->NumJoints();
}


void
Cfg::
SetSize(size_t _size) {
  m_robots.resize(_size);
}


void
Cfg::
InitRobots(shared_ptr<ActiveMultiBody>& _robots, size_t _index) {
  if(_index == size_t(-1))
    m_pointRobot = _robots;
  else
    m_robots[_index] = _robots;
}


Cfg&
Cfg::
operator=(const Cfg& _cfg) {
  if(this != &_cfg) {
    m_v.clear();
    m_v = _cfg.GetData();
    m_labelMap = _cfg.m_labelMap;
    m_statMap = _cfg.m_statMap;
    m_robotIndex = _cfg.m_robotIndex;
    m_clearanceInfo = _cfg.m_clearanceInfo;
    m_witnessCfg = _cfg.m_witnessCfg;
  }
  return *this;
}


bool
Cfg::
operator==(const Cfg& _cfg) const {
  if(m_robotIndex != _cfg.m_robotIndex)
    return false;
  for(size_t i = 0; i < DOF(); ++i) {
    double eps = Epsilon(m_v[i], _cfg[i]);
    switch(GetRobot()->GetDOFType(i)) {
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
    if(GetRobot()->GetDOFType(i) == DofType::Positional ||
        GetRobot()->GetDOFType(i) == DofType::Joint)
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

bool 
Cfg::
operator<(const Cfg& _cfg) const {
  for(size_t i = 0; i < DOF(); ++i) {
    if(m_v[i] < _cfg.m_v[i])
      return true;
    else if(m_v[i] > _cfg.m_v[i])
      return false;
  }
  return false;
}


void
Cfg::
Read(istream& _is) {
  //first read in robot index, and then read in DOF values
  _is >> m_robotIndex;
  //if this failed, then we're done reading Cfgs
  if (_is.fail())
    return;
  for(size_t i = 0; i < m_v.size(); ++i) {
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
  //write out robot index, and then dofs
  _os << setw(4) << m_robotIndex << ' ' << scientific << setprecision(17);
  for(vector<double>::const_iterator i = m_v.begin(); i != m_v.end(); ++i)
    _os << setw(25) << *i << ' ';
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
GetPoint() const {
  return Point3d(m_v[0], m_v[1], PosDOF() == 3 ? m_v[2] : 0);
}


void
Cfg::
SetData(const vector<double>& _data) {
  if(_data.size() != DOF()) {
    string msg = "Tried to set data for " + to_string(_data.size()) +
        " DOFs, but robot has " + to_string(DOF()) + " DOFs!";
    throw RunTimeException(WHERE, msg);
  }
  m_v = _data;
  m_witnessCfg.reset();
}


void
Cfg::
SetJointData(const vector<double>& _data) {
  if(_data.size() != GetNumOfJoints()) {
    string msg = "Tried to set data for " + to_string(_data.size()) +
        " joints, but robot has " + to_string(GetNumOfJoints()) + " joints!";
    throw RunTimeException(WHERE, msg);
  }
  unsigned int j = 0;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetRobot()->GetDOFType(i) == DofType::Joint) {
      m_v[i]=_data[j];
      j++;
    }
  }
  m_witnessCfg.reset();
}


vector<double>
Cfg::
GetNormalizedData(const shared_ptr<const Boundary> _b) const {
  pair<vector<double>, vector<double>> range = GetRobot()->GetCfgLimits(_b);
  vector<double> normed;
  for(size_t i = 0; i < DOF(); ++i) {
    double radius = (range.second[i] - range.first[i]) / 2.;
    double center = range.first[i] + radius;
    normed.push_back((m_v[i] - center) / radius);
  }
  return move(normed);
}


void
Cfg::
SetNormalizedData(const vector<double>& _data,
    const shared_ptr<const Boundary> _b) {
  if(_data.size() != DOF()) {
    string msg = "Tried to set data for " + to_string(_data.size()) +
        " DOFs, but robot has " + to_string(DOF()) + " DOFs!";
    throw RunTimeException(WHERE, msg);
  }
  pair<vector<double>, vector<double>> range = GetRobot()->GetCfgLimits(_b);
  for(size_t i = 0; i < DOF(); ++i) {
    double radius = (range.second[i] - range.first[i]) / 2.;
    double center = range.first[i] + radius;
    m_v[i] = _data[i] * radius + center;
  }
}


bool
Cfg::
GetLabel(string _label) {
  if(!IsLabel(_label))
    throw RunTimeException(WHERE, "No label\'" + _label + "\' found.");
  return m_labelMap[_label];
}


bool
Cfg::
IsLabel(string _label) {
  return m_labelMap.count(_label) > 0;
}


void
Cfg::
SetLabel(string _label, bool _value) {
  m_labelMap[_label] = _value;
}


double
Cfg::
GetStat(string _stat) const {
  if(!IsStat(_stat))
    throw RunTimeException(WHERE, "No stat \'" + _stat + "\' found.");
  return m_statMap.at(_stat);
}


bool
Cfg::
IsStat(string _stat) const {
  return m_statMap.count(_stat) > 0;
}


void
Cfg::
SetStat(string _stat, double _value) {
  m_statMap[_stat] = _value;
}


void
Cfg::
IncStat(string _stat, double _value) {
  m_statMap[_stat] += _value;
}


vector<double>
Cfg::
GetPosition() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetRobot()->GetDOFType(i) == DofType::Positional)
      ret.push_back(m_v[i]);
  }
  return ret;
}


vector<double>
Cfg::
GetOrientation() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetRobot()->GetDOFType(i) != DofType::Positional)
      ret.push_back(m_v[i]);
  }
  return ret;
}


vector<double>
Cfg::
GetNonJoints() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetRobot()->GetDOFType(i) != DofType::Joint)
      ret.push_back(m_v[i]);
  }
  return ret;
}


vector<double>
Cfg::
GetJoints() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetRobot()->GetDOFType(i) == DofType::Joint)
      ret.push_back(m_v[i]);
  }
  return ret;
}


vector<double>
Cfg::
GetRotation() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetRobot()->GetDOFType(i) == DofType::Rotational)
      ret.push_back(m_v[i]);
  }
  return ret;
}


void
Cfg::
ResetRigidBodyCoordinates() {
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetRobot()->GetDOFType(i) != DofType::Joint)
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
    if(GetRobot()->GetDOFType(i) == DofType::Positional)
      result += m_v[i]*m_v[i];
  return sqrt(result);
}


double
Cfg::
OrientationMagnitude() const {
  double result = 0.0;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetRobot()->GetDOFType(i) != DofType::Positional)
      result += m_v[i]*m_v[i];
  }
  return sqrt(result);
}


Vector3d
Cfg::
GetRobotCenterPosition() const {
  Vector3d v;
  for(size_t i = 0; i < 3 && i < DOF(); i++)
    if(GetRobot()->GetDOFType(i) == DofType::Positional)
      v[i] = m_v[i];
  return v;
}


Vector3d
Cfg::
GetRobotCenterofMass() const {
  ConfigureRobot();
  return GetRobot()->GetCenterOfMass();
}

// generates random configuration where workspace robot's EVERY VERTEX
// is guaranteed to lie within the environment specified bounding box
void
Cfg::
GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundary());
}

void
Cfg::
GetRandomCfg(Environment* _env, shared_ptr<Boundary> _bb) {
  m_witnessCfg.reset();
  // Probably should do something smarter than 3 strikes and exit.
  // eg, if it fails once, check size of bounding box vs robot radius
  // and see if user has an impossibly small (for this robot) bounding
  // box specified
  size_t tries = 100;
  while(tries-- > 0) {
    this->GetRandomCfgImpl(_env, _bb);

    if(_env->InBounds(*this, _bb))
      return;
  }

  // throw error message and some helpful statistics
  ostringstream oss;
  oss << "GetRandomCfg not able to find anything in boundary: "
    << *_bb << ". Robot radius is "
    << GetRobot()->GetBoundingSphereRadius() << ".";
  throw PMPLException("Boundary too small", WHERE, oss.str());
}


void
Cfg::
ConfigureRobot() const {
  GetRobot()->Configure(m_v);
}


void
Cfg::
GetResolutionCfg(Environment* _env) {
  m_v.clear();
  double posRes = _env->GetPositionRes();
  double oriRes = _env->GetOrientationRes();

  for(size_t i = 0; i < DOF(); i++)
    if(GetRobot()->GetDOFType(i) == DofType::Positional)
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
    if(GetRobot()->GetDOFType(i) == DofType::Positional ||
        GetRobot()->GetDOFType(i) == DofType::Joint) {
      //If the diff between _goal and c is smaller than _increment
      if(fabs(_goal.m_v[i]-m_v[i]) < fabs(_increment.m_v[i]))
        m_v[i] = _goal.m_v[i];
      else
        m_v[i] += _increment.m_v[i];
    }
  }

  ///For Oirentation
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetRobot()->GetDOFType(i) == DofType::Rotational) {
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
  *_nTicks = max(1., ceil(max(diff.PositionMagnitude()/_positionRes,
        diff.OrientationMagnitude()/_orientationRes)));

  this->FindIncrement(_start, _goal, *_nTicks);
}


void
Cfg::
FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks) {
  vector<double> incr;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetRobot()->GetDOFType(i) == DofType::Positional)
      incr.push_back((_goal.m_v[i] - _start.m_v[i])/_nTicks);
    else if(GetRobot()->GetDOFType(i) == DofType::Joint) {
      double a = _start.m_v[i];
      double b = _goal.m_v[i];
      if(_nTicks == 0)
        throw PMPLException("Divide by 0", WHERE, "Divide by 0");
      incr.push_back((b-a)/_nTicks);
    }
    else if(GetRobot()->GetDOFType(i) == DofType::Rotational) {
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
    if(GetRobot()->GetDOFType(i) == DofType::Positional)
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
  GetRobot()->PolygonalApproximation(result);
  return result;
}


void
Cfg::
NormalizeOrientation(int _index) {
  //Normalize the orientation to the range [-1, 1)
  if(_index == -1) {
    for(size_t i = 0; i < DOF(); ++i) {
      if(GetRobot()->GetDOFType(i) == DofType::Rotational)
        m_v[i] = Normalize(m_v[i]);
    }
  }
  else if(GetRobot()->GetDOFType(_index) == DofType::Rotational) {
    // orientation index
    m_v[_index] = Normalize(m_v[_index]);
  }
}


void
Cfg::
GetRandomCfgImpl(Environment* _env, shared_ptr<Boundary> _b) {
  m_v = GetRobot()->GetRandomCfg(_b);
  m_witnessCfg.reset();
}

/*----------------------------------------------------------------------------*/
