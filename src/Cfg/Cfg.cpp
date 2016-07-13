/////////////////////////////////////////////////////////////////////
//
//  Cfg.c
//
//  General Description
//      Configuration Data Class, it has all the interface needed
//      by other Motion Planning classes. Since it is abstract, it
//      will have to 'ask' a helper class called CfgManager to
//      provide implementation to some specific functions.
/////////////////////////////////////////////////////////////////////

#include "Cfg.h"

#include "Environment/ActiveMultiBody.h"
#include "Environment/Environment.h"
#include "Utilities/MetricUtils.h"

ClearanceInfo::~ClearanceInfo() {
  if(m_direction != NULL)
    delete m_direction;
}

////////////////////////////////////////////////////////////////////
vector<size_t> Cfg::m_dof;
vector<size_t> Cfg::m_posdof;
vector<size_t> Cfg::m_numJoints;
vector<vector<DofType>> Cfg::m_dofTypes;
vector<shared_ptr<ActiveMultiBody>> Cfg::m_robots;

Cfg::Cfg(size_t _robotIndex) {
  m_v.clear();
  m_robotIndex = _robotIndex;
  if(m_dof.size() > 0)
    m_v.resize(m_dof[m_robotIndex], 0.0);
  m_witnessCfg.reset();
}

Cfg::Cfg(const Cfg& _other) :
  m_v(_other.m_v),
  m_robotIndex(_other.m_robotIndex),
  m_labelMap(_other.m_labelMap),
  m_statMap(_other.m_statMap),
  m_clearanceInfo(_other.m_clearanceInfo),
  m_witnessCfg(_other.m_witnessCfg) {}

void
Cfg::SetSize(size_t _size) {
  m_dof.resize(_size);
  m_numJoints.resize(_size);
  m_posdof.resize(_size);
  m_dofTypes.resize(_size);
  m_robots.resize(_size);
}

void
Cfg::InitRobots(shared_ptr<ActiveMultiBody>& _robots, size_t _index) {
  m_robots[_index] = _robots;
  m_dof[_index] = _robots->DOF();
  m_numJoints[_index] = _robots->NumJoints();
  m_posdof[_index] = _robots->PosDOF();
  m_dofTypes[_index] = _robots->GetDOFTypes();
}

Cfg&
Cfg::operator=(const Cfg& _cfg) {
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
Cfg::operator==(const Cfg& _cfg) const {
  if (m_robotIndex != _cfg.m_robotIndex)
    return false;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    double eps = Epsilon(m_v[i], _cfg[i]);
    switch(m_dofTypes[m_robotIndex][i]) {
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
Cfg::operator!=(const Cfg& _cfg) const {
  return !(*this == _cfg);
}

Cfg
Cfg::operator+(const Cfg& _cfg) const {
  Cfg result = *this;
  result += _cfg;
  return result;
}

Cfg&
Cfg::operator+=(const Cfg& _cfg) {
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i)
    m_v[i] += _cfg[i];
  NormalizeOrientation();
  m_witnessCfg.reset();
  return *this;
}

Cfg
Cfg::operator-(const Cfg& _cfg) const {
  Cfg result = *this;
  result -= _cfg;
  return result;
}

Cfg&
Cfg::operator-=(const Cfg& _cfg) {
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == DofType::Positional || m_dofTypes[m_robotIndex][i] == DofType::Joint)
      m_v[i] -= _cfg[i];
    else
      m_v[i] = DirectedAngularDistance(m_v[i], _cfg.m_v[i]);
  }
  NormalizeOrientation();
  m_witnessCfg.reset();
  return *this;
}

Cfg
Cfg::operator-() const {
  Cfg result = *this;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i)
    result[i] = -result[i];
  result.NormalizeOrientation();
  result.m_witnessCfg.reset();
  return result;
}

Cfg
Cfg::operator*(double _d) const {
  Cfg result = *this;
  result *= _d;
  return result;
}

Cfg&
Cfg::operator*=(double _d) {
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i)
    m_v[i] *= _d;
  NormalizeOrientation();
  m_witnessCfg.reset();
  return *this;
}

Cfg
Cfg::operator/(double _d) const {
  Cfg result = *this;
  result /= _d;
  return result;
}

Cfg&
Cfg::operator/=(double _d) {
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i)
    m_v[i] /= _d;
  NormalizeOrientation();
  m_witnessCfg.reset();
  return *this;
}

double&
Cfg::operator[](size_t _dof) {
  assert(_dof >= 0 && _dof <= m_dof[m_robotIndex]);
  m_witnessCfg.reset();
  return m_v[_dof];
}

const double&
Cfg::operator[](size_t _dof) const {
  assert(_dof >= 0 && _dof <= m_dof[m_robotIndex]);
  return m_v[_dof];
}

//---------------------------------------------
// Input/Output operators for Cfg
//---------------------------------------------
void
Cfg::Read(istream& _is) {
  //first read in robot index, and then read in DOF values
  _is >> m_robotIndex;
  //if this failed, then we're done reading Cfgs
  if (_is.fail())
    return;
  for(vector<double>::iterator i = m_v.begin(); i != m_v.end(); ++i) {
    _is >> *i;
    if (_is.fail()) {
      cerr << "Cfg::operator>> error - failed reading values for all dofs" << endl;
      exit(1);
    }
  }
}

void
Cfg::
Write(ostream& _os) const{
  //write out robot index, and then dofs
  _os << setw(4) << m_robotIndex << ' ' << setprecision(4);
  for(vector<double>::const_iterator i = m_v.begin(); i != m_v.end(); ++i)
    _os << setw(6) << *i << ' ';
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
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == DofType::Joint) {
      m_v[i]=_data[j];
      j++;
    }
  }
  m_witnessCfg.reset();
}

vector<double>
Cfg::
GetNormalizedData(const shared_ptr<const Boundary> _b) const {
  pair<vector<double>, vector<double>> range = m_robots[m_robotIndex]->
      GetCfgLimits(_b);
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
  pair<vector<double>, vector<double>> range = m_robots[m_robotIndex]->
      GetCfgLimits(_b);
  for(size_t i = 0; i < DOF(); ++i) {
    double radius = (range.second[i] - range.first[i]) / 2.;
    double center = range.first[i] + radius;
    m_v[i] = _data[i] * radius + center;
  }
}

bool
Cfg::GetLabel(string _label) {
  if(IsLabel(_label)) {
    return m_labelMap[_label];
  }
  else {
    cout << "Cfg::GetLabel -- I cannot find Label =  " << _label << endl;
    exit(-1);
  }
}

bool
Cfg::IsLabel(string _label) {
  bool label = false;
  if(m_labelMap.count(_label) > 0)
    label = true;
  else
    label = false;
  return label;
}

void
Cfg::SetLabel(string _label, bool _value) {
  m_labelMap[_label] = _value;
}


double
Cfg::GetStat(string _stat) {
  if(IsStat(_stat)) {
    return m_statMap[_stat];
  }
  else {
    cout << "Cfg::GetStat -- I cannot find Stat =  " << _stat << endl;
    exit(-1);
  }
}

bool
Cfg::IsStat(string _stat) {
  bool stat = false;
  if(m_statMap.count(_stat) > 0)
    stat = true;
  else
    stat = false;
  return stat;
}

void
Cfg::SetStat(string _stat, double _value) {
  m_statMap[_stat] = _value;
}

void
Cfg::IncStat(string _stat, double _value) {
  m_statMap[_stat] += _value;
}

vector<double>
Cfg::GetPosition() const {
  vector<double> ret;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == DofType::Positional)
      ret.push_back(m_v[i]);
  }
  return ret;
}

vector<double>
Cfg::GetOrientation() const {
  vector<double> ret;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] != DofType::Positional)
      ret.push_back(m_v[i]);
  }
  return ret;
}

///////////////////////////////////
vector<double>
Cfg::GetNonJoints() const {
  vector<double> ret;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] != DofType::Joint)
      ret.push_back(m_v[i]);
  }
  return ret;
}

vector<double>
Cfg::GetJoints() const {
  vector<double> ret;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == DofType::Joint)
      ret.push_back(m_v[i]);
  }
  return ret;
}

vector<double>
Cfg::GetRotation() const {
  vector<double> ret;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == DofType::Rotational)
      ret.push_back(m_v[i]);
  }
  return ret;
}

void
Cfg::ResetRigidBodyCoordinates() {
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] != DofType::Joint)
      m_v[i]=0;
  }
}
///////////////////////////////////


double
Cfg::Magnitude() const {
  double result = 0.0;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i)
    result += m_v[i]*m_v[i];
  return sqrt(result);
}

double
Cfg::PositionMagnitude() const {
  double result = 0.0;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i)
    if(m_dofTypes[m_robotIndex][i] == DofType::Positional)
      result += m_v[i]*m_v[i];
  return sqrt(result);
}

double
Cfg::OrientationMagnitude() const {
  double result = 0.0;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] != DofType::Positional)
      result += m_v[i]*m_v[i];
  }
  return sqrt(result);
}

Vector3d
Cfg::GetRobotCenterPosition() const {
  Vector3d v;
  for(size_t i = 0; i < 3 && i < DOF(); i++)
    if(m_dofTypes[m_robotIndex][i] == DofType::Positional)
      v[i] = m_v[i];
  return v;
}

Vector3d
Cfg::
GetRobotCenterofMass() const {
  ConfigEnvironment();
  return m_robots[m_robotIndex]->GetCenterOfMass();
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
    << m_robots[m_robotIndex]->GetBoundingSphereRadius() << ".";
  throw PMPLException("Boundary to small", WHERE, oss.str());
}

void
Cfg::
ConfigEnvironment() const {
  m_robots[m_robotIndex]->Configure(m_v);
}

void
Cfg::GetResolutionCfg(Environment* _env) {
  m_v.clear();
  double posRes = _env->GetPositionRes();
  double oriRes = _env->GetOrientationRes();

  for(size_t i = 0; i < m_dof[m_robotIndex]; i++)
    if(m_dofTypes[m_robotIndex][i] == DofType::Positional)
      m_v.push_back(posRes);
    else
      m_v.push_back(oriRes);

  NormalizeOrientation();
  m_witnessCfg.reset();
}

void
Cfg::IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment) {
  ///For Position
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == DofType::Positional || m_dofTypes[m_robotIndex][i] == DofType::Joint) {
      //If the diff between _goal and c is smaller than _increment
      if(fabs(_goal.m_v[i]-m_v[i]) < fabs(_increment.m_v[i]))
        m_v[i] = _goal.m_v[i];
      else
        m_v[i] += _increment.m_v[i];
    }
  }

  ///For Oirentation
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == DofType::Rotational) {
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
Cfg::FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks, double _positionRes, double _orientationRes) {
  Cfg diff = _goal - _start;

  // adding two basically makes this a rough ceiling...
  *_nTicks = max(1., ceil(max(diff.PositionMagnitude()/_positionRes,
        diff.OrientationMagnitude()/_orientationRes)));

  this->FindIncrement(_start, _goal, *_nTicks);
}

void
Cfg::FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks) {
  vector<double> incr;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == DofType::Positional)
      incr.push_back((_goal.m_v[i] - _start.m_v[i])/_nTicks);
    else if(m_dofTypes[m_robotIndex][i] == DofType::Joint) {
      double a = _start.m_v[i];
      double b = _goal.m_v[i];
      if(_nTicks == 0)
        throw PMPLException("Divide by 0", WHERE, "Divide by 0");
      incr.push_back((b-a)/_nTicks);
    }
    else if(m_dofTypes[m_robotIndex][i] == DofType::Rotational) {
      incr.push_back(DirectedAngularDistance(_start.m_v[i], _goal.m_v[i])/_nTicks);
    }
  }

  m_v = incr;
  NormalizeOrientation();
  m_witnessCfg.reset();
}

void
Cfg::WeightedSum(const Cfg& _first, const Cfg& _second, double _weight) {
  vector<double> v;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i)
    v.push_back(_first.m_v[i]*(1.-_weight) + _second.m_v[i]*_weight);
  m_v = v;
  NormalizeOrientation();
  m_witnessCfg.reset();
}

void
Cfg::GetPositionOrientationFrom2Cfg(const Cfg& _c1, const Cfg& _c2) {
  vector<double> v;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == DofType::Positional)
      v.push_back(_c1.m_v[i]);
    else
      v.push_back(_c2.m_v[i]);
  }
  m_v = v;
  NormalizeOrientation();
  m_witnessCfg.reset();
}

vector<Vector3d>
Cfg::PolyApprox() const {
  vector<Vector3d> result;
  ConfigEnvironment();
  m_robots[m_robotIndex]->PolygonalApproximation(result);
  return result;
}

//Normalize the orientation to the range [-1, 1)
void
Cfg::NormalizeOrientation(int _index) {
  if(_index == -1) {
    for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
      if(m_dofTypes[m_robotIndex][i] == DofType::Rotational) {
        m_v[i] = Normalize(m_v[i]);
      }
    }
  }
  else if(m_dofTypes[m_robotIndex][_index] == DofType::Rotational) {  // orientation index
    m_v[_index] = Normalize(m_v[_index]);
  }
}

//generates random configuration within C-space
void
Cfg::
GetRandomCfgImpl(Environment* _env, shared_ptr<Boundary> _bb) {
  m_v = m_robots[m_robotIndex]->GetRandomCfg(_bb);
  m_witnessCfg.reset();
}
