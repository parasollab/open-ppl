/////////////////////////////////////////////////////////////////////
//
//  CfgMultiRobotMultiRobot.cpp
//
//  General Description
//
//  Created
//
//  Last Modified By:
//
/////////////////////////////////////////////////////////////////////

#include "CfgMultiRobot.h"

#include "Environment/Environment.h"
#include "Environment/MultiBody.h"
#include "Utilities/MetricUtils.h"

size_t CfgMultiRobot::m_numRobot;
typedef vector<Cfg>::iterator CIter;
typedef vector<Cfg>::const_iterator ConstCIter;

CfgMultiRobot::
CfgMultiRobot() {
  m_robotIndex = -1;
  for(size_t i = 0; i < m_numRobot; ++i)
    this->m_robotsCollect.push_back(Cfg(i));
}

CfgMultiRobot::
CfgMultiRobot(const Cfg& _c) {
  this->m_robotsCollect.clear();
  this->m_robotsCollect.push_back(_c);
}

CfgMultiRobot::
CfgMultiRobot(const CfgMultiRobot& _other) :
  m_robotsCollect(_other.m_robotsCollect) {}

CfgMultiRobot&
CfgMultiRobot::
operator=(const CfgMultiRobot& _cfg) {
  if(this != &_cfg)
    m_robotsCollect = _cfg.m_robotsCollect;
  return *this;
}

bool
CfgMultiRobot::
operator==(const CfgMultiRobot& _cfg) const {
  bool result = true;
  for(size_t i = 0; i < m_robotsCollect.size(); ++i)
    result &= (this->m_robotsCollect[i] == _cfg.m_robotsCollect[i]);
  return result;
}

bool
CfgMultiRobot::
operator!=(const CfgMultiRobot& _cfg) const {
  return !(*this == _cfg);
}

CfgMultiRobot
CfgMultiRobot::
operator+(const CfgMultiRobot& _cfg) const {
  CfgMultiRobot result = *this;
  result += _cfg;
  return result;
}

CfgMultiRobot&
CfgMultiRobot::
operator+=(const CfgMultiRobot& _cfg) {
  for(size_t i = 0; i < m_robotsCollect.size(); ++i)
    this->m_robotsCollect[i] += _cfg.m_robotsCollect[i];
  return *this;
}

CfgMultiRobot
CfgMultiRobot::
operator-(const CfgMultiRobot& _cfg) const {
  CfgMultiRobot result = *this;
  result -= _cfg;
  return result;
}

CfgMultiRobot&
CfgMultiRobot::
operator-=(const CfgMultiRobot& _cfg) {
  for(size_t i = 0; i < m_robotsCollect.size(); ++i)
    this->m_robotsCollect[i] -= _cfg.m_robotsCollect[i];
  return *this;
}

CfgMultiRobot
CfgMultiRobot::
operator-() const {
  CfgMultiRobot result = *this;
  for(size_t i = 0; i< result.m_robotsCollect.size(); ++i)
    result.m_robotsCollect[i] = -(result.m_robotsCollect[i]);
  return result;
}

CfgMultiRobot
CfgMultiRobot::
operator*(double _d) const {
  CfgMultiRobot result = *this;
  result *= _d;
  return result;
}

CfgMultiRobot&
CfgMultiRobot::
operator*=(double _d) {
  for(size_t i = 0; i < m_robotsCollect.size(); ++i)
    this->m_robotsCollect[i] *= _d;
  return *this;
}

CfgMultiRobot
CfgMultiRobot::
operator/(double _d) const {
  CfgMultiRobot result = *this;
  result /= _d;
  return result;
}

CfgMultiRobot&
CfgMultiRobot::
operator/=(double _d) {
  for(size_t i = 0; i< m_robotsCollect.size(); ++i)
    this->m_robotsCollect[i] /= _d;
  return *this;
}

double&
CfgMultiRobot::
operator[](size_t _dof) {
  for(size_t i = 0; i< m_robotsCollect.size(); ++i) {
    size_t dof = this->m_robotsCollect[i].DOF();
    if(_dof > dof) {
      _dof -= dof;
    } else {
      return this->m_robotsCollect[i][_dof];
    }
  }
  throw PMPLException("DOF Out of Bound", WHERE, "DOF Out of Bound");
}

const double&
CfgMultiRobot::
operator[](size_t _dof) const {
  for(size_t i = 0; i< m_robotsCollect.size(); ++i) {
    size_t dof = this->m_robotsCollect[i].DOF();
    if(_dof > dof) {
      _dof -= dof;
    } else {
      return this->m_robotsCollect[i][_dof];
    }
  }
  throw PMPLException("DOF Out of Bound", WHERE, "DOF Out of Bound");
}

//---------------------------------------------
// Input/Output operators for CfgMultiRobot
//---------------------------------------------
void
CfgMultiRobot::
Read(istream& _is) {
  //first read in robot index, and then read in DOF values
  for(CIter cIter= m_robotsCollect.begin(); cIter != m_robotsCollect.end(); cIter++)
    cIter->Read(_is);
}

void
CfgMultiRobot::
Write(ostream& _os) const{
  //write out robot index, and then dofs
  for(ConstCIter cIter= m_robotsCollect.begin(); cIter != m_robotsCollect.end(); cIter++)
    cIter->Write(_os);
}

istream&
operator>>(istream& _is, CfgMultiRobot& _cfg) {
  vector<Cfg>& c = _cfg.m_robotsCollect;
  for(size_t i = 0; i< c.size(); ++i)
    _is >> c[i];
  return _is;
}

// not implemented, should never be used
const vector<double>&
CfgMultiRobot::
GetData() const {
  throw PMPLException("Not Implemented", WHERE, "Not Implemented");
}

void
CfgMultiRobot::
SetData(const vector<double>& _data) {
  size_t dataSize = _data.size();
  size_t begin = 0;

  for(CIter cIter= m_robotsCollect.begin(); cIter != m_robotsCollect.end(); cIter++) {
    size_t dof = cIter->DOF();
    vector<double>::const_iterator first = _data.begin() + begin;
    vector<double>::const_iterator last = _data.begin() + begin + dof;
    vector<double> newVec(first, last);
    cIter->SetData(newVec);

    begin += dof;
    dataSize -= dof;
  }
  if(dataSize != 0)
    throw PMPLException("DOF Out of Bound", WHERE, "DOF Out of Bound");
}

vector<double>
CfgMultiRobot::
GetPosition() const {
  vector<double> ret;
  for(ConstCIter cIter= m_robotsCollect.begin(); cIter != m_robotsCollect.end(); cIter++) {
    for(size_t j = 0; j < cIter->GetPosition().size(); ++j)
      ret.push_back(cIter->GetPosition()[j]);
  }
  return ret;
}

vector<double>
CfgMultiRobot::
GetOrientation() const {
  vector<double> ret;
  for(ConstCIter cIter= m_robotsCollect.begin(); cIter != m_robotsCollect.end(); cIter++) {
    for(size_t j = 0; j < cIter->GetOrientation().size(); ++j)
      ret.push_back(cIter->GetOrientation()[j]);
  }
  return ret;
}

double
CfgMultiRobot::
Magnitude() const {
  double result = 0.0;
  for(ConstCIter cIter= m_robotsCollect.begin(); cIter != m_robotsCollect.end(); cIter++)
    result += sqr(cIter->Magnitude());
  return sqrt(result);
}

double
CfgMultiRobot::
PositionMagnitude() const {
  double result = 0.0;
  for(ConstCIter cIter= m_robotsCollect.begin(); cIter != m_robotsCollect.end(); cIter++)
    result += sqr(cIter->PositionMagnitude());
  return sqrt(result);
}

double
CfgMultiRobot::
OrientationMagnitude() const {
  double result = 0.0;
  for(ConstCIter cIter= m_robotsCollect.begin(); cIter != m_robotsCollect.end(); cIter++)
    result += sqr(cIter->OrientationMagnitude());
  return sqrt(result);
}

Vector3d
CfgMultiRobot::
GetRobotCenterPosition() const {
  Vector3d pos;
  for(ConstCIter cIter= m_robotsCollect.begin(); cIter != m_robotsCollect.end(); cIter++)
    pos += cIter->GetRobotCenterPosition();
  return pos/m_robotsCollect.size();
}

//come back again
Vector3d
CfgMultiRobot::
GetRobotCenterofMass() const {
  Vector3d com;
  for(size_t i = 0; i < m_robotsCollect.size(); ++i)
    com += this->m_robotsCollect[i].GetRobotCenterofMass();
  return com/m_robotsCollect.size();
}

void
CfgMultiRobot::
ConfigureRobot() const {
  for(ConstCIter cIter= m_robotsCollect.begin(); cIter != m_robotsCollect.end(); cIter++)
    cIter->ConfigureRobot();
}

void
CfgMultiRobot::
GetResolutionCfg(Environment* _env) {
  for(CIter cIter= m_robotsCollect.begin(); cIter != m_robotsCollect.end(); cIter++)
    cIter->GetResolutionCfg(_env);
}

void
CfgMultiRobot::
IncrementTowardsGoal(const CfgMultiRobot& _goal, const CfgMultiRobot& _increment) {
  for(size_t i = 0; i < m_robotsCollect.size(); ++i)
    this->m_robotsCollect[i].IncrementTowardsGoal(_goal.m_robotsCollect[i], _increment.m_robotsCollect[i]);
}

void
CfgMultiRobot::
FindIncrement(const CfgMultiRobot& _start, const CfgMultiRobot& _goal, int* _nTicks, double _positionRes, double _orientationRes) {
  // if multiple cfg, nTicks = max(nTicks[])
  *_nTicks = 0;
  int nTicks = 0;
  for(size_t i = 0; i < m_robotsCollect.size(); ++i) {
    this->m_robotsCollect[i].FindIncrement(_start.m_robotsCollect[i], _goal.m_robotsCollect[i], &nTicks, _positionRes, _orientationRes);
    if(*_nTicks < nTicks)
      *_nTicks = nTicks;
  }
  this->FindIncrement(_start, _goal, *_nTicks);
}

void
CfgMultiRobot::
FindIncrement(const CfgMultiRobot& _start, const CfgMultiRobot& _goal, int _nTicks) {
  for(size_t i = 0; i < m_robotsCollect.size(); ++i) {
    this->m_robotsCollect[i].FindIncrement(_start.m_robotsCollect[i], _goal.m_robotsCollect[i], _nTicks);
  }
}

void
CfgMultiRobot::
WeightedSum(const CfgMultiRobot& _first, const CfgMultiRobot& _second, double _weight) {
  for(size_t i = 0; i < m_robotsCollect.size(); ++i)
    this->m_robotsCollect[i].WeightedSum(_first.m_robotsCollect[i], _second.m_robotsCollect[i], _weight);
}

void
CfgMultiRobot::
GetPositionOrientationFrom2Cfg(const CfgMultiRobot& _c1, const CfgMultiRobot& _c2) {
  for(size_t i = 0; i < m_robotsCollect.size(); ++i)
    this->m_robotsCollect[i].GetPositionOrientationFrom2Cfg(_c1.m_robotsCollect[i], _c2.m_robotsCollect[i]);
}

vector<Vector3d>
CfgMultiRobot::
PolyApprox(Environment* _env) const {
  throw PMPLException("Not Implemented", WHERE, "Not Implemented");
  /* vector<Vector3d> result; */
  /* ConfigureRobot(); */
  /* _env->GetMultiBody(m_robotIndex)->PolygonalApproximation(result); */
  /* return result; */
}

void
CfgMultiRobot::
GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundary());
}

void
CfgMultiRobot::
GetRandomCfg(Environment* _env, shared_ptr<Boundary> _bb) {
  for(CIter cIter= m_robotsCollect.begin(); cIter != m_robotsCollect.end(); cIter++)
    cIter->GetRandomCfg(_env, _bb);
}

size_t
CfgMultiRobot::
DOF() const {
  size_t dof = 0;
  for(ConstCIter cIter= m_robotsCollect.begin(); cIter != m_robotsCollect.end(); cIter++)
    dof += cIter->DOF();
  return dof;
}
