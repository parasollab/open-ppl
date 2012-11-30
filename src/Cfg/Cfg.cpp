/////////////////////////////////////////////////////////////////////
//
//  Cfg.c
//
//  General Description
//      Configuration Data Class, it has all the interface needed
//      by other Motion Planning classes. Since it is abstract, it
//      will have to 'ask' a helper class called CfgManager to
//      provide implementation to some specific functions.
//
//  Created
//      08/31/99        Guang Song
//
/////////////////////////////////////////////////////////////////////

#include "Cfg.h"
#include "MPProblem/Geometry/MultiBody.h"
#include "MPProblem/Environment.h"
#include "Utilities/MetricUtils.h"
#include "ValidityCheckers/CollisionDetection/CDInfo.h"

#define EQ(a,b)  (fabs(a-b)<0.0001)

ClearanceInfo::~ClearanceInfo() {
  if(m_direction != NULL)
    delete m_direction;
}

////////////////////////////////////////////////////////////////////
size_t Cfg::m_dof;
vector<Cfg::DofType> Cfg::m_dofTypes;
vector<Robot> Cfg::m_robots;

Cfg::Cfg(const Cfg& _other) {
  m_v = _other.m_v;
  m_dof = _other.m_dof;
  m_labelMap = _other.m_labelMap;
  m_statMap = _other.m_statMap;
}

void
Cfg::InitRobots(vector<Robot>& _robots) {
  m_robots = _robots;
  for(vector<Robot>::iterator rit = m_robots.begin(); rit != m_robots.end(); rit++) {
    if(rit->m_base == Robot::PLANAR) {
      m_dofTypes.push_back(POS);
      m_dofTypes.push_back(POS);
      if(rit->m_baseMovement == Robot::ROTATIONAL)
        m_dofTypes.push_back(ROT);
    }
    if(rit->m_base == Robot::VOLUMETRIC) {
      m_dofTypes.push_back(POS);
      m_dofTypes.push_back(POS);
      m_dofTypes.push_back(POS);
      if(rit->m_baseMovement == Robot::ROTATIONAL) {
        m_dofTypes.push_back(ROT);
        m_dofTypes.push_back(ROT);
        m_dofTypes.push_back(ROT);
      }
    }
    for(Robot::JointIT jit = rit->m_joints.begin(); jit != rit->m_joints.end(); jit++) {
      if(jit->second == Robot::REVOLUTE) {
        m_dofTypes.push_back(JOINT);
      }
      else if(jit->second == Robot::SPHERICAL) {
        m_dofTypes.push_back(JOINT);
        m_dofTypes.push_back(JOINT);
      }
      else if(jit->second == Robot::NONACTUATED){
        //skip, do nothing
      }
    }
  }
  m_dof = m_dofTypes.size();
}

bool
Cfg::operator==(const Cfg& _tmp) const {
  return AlmostEqual(_tmp);
}

bool
Cfg::operator!=(const Cfg& _tmp) const {
  return !(*this == _tmp);
}

bool
Cfg::AlmostEqual(const Cfg& _c) const {
  for(size_t i = 0; i < m_dof; ++i) {
    if(m_dofTypes[i] == POS || m_dofTypes[i] == JOINT) {
      if(!EQ(m_v[i], _c.m_v[i]))
        return false;
    }
    else {
      if(!EQ(DirectedAngularDistance(m_v[i], _c.m_v[i]), 0.0)) 
        return false;
    }
  }
  return true;
}

void
Cfg::add(const Cfg& _c1, const Cfg& _c2) {
  vector<double> v;
  for(size_t i = 0; i < m_dof; ++i)
    v.push_back(_c1.m_v[i] + _c2.m_v[i]);
  m_v = v;
  NormalizeOrientation();
}

void
Cfg::subtract(const Cfg& _c1, const Cfg& _c2) {
  vector<double> v;
  for(size_t i = 0; i < m_dof; ++i) {
    if(m_dofTypes[i] == POS || m_dofTypes[i] == JOINT)
      v.push_back(_c1.m_v[i]-_c2.m_v[i]);
    else
      v.push_back(DirectedAngularDistance(_c1.m_v[i], _c2.m_v[i]));
  }
  m_v = v;
  NormalizeOrientation();
}

void
Cfg::negative(const Cfg& _c) {
  vector<double> v;    
  for(vector<double>::const_iterator i = _c.m_v.begin(); i != _c.m_v.end(); ++i) 
    v.push_back(-(*i));
  m_v = v;
  NormalizeOrientation();
}

void
Cfg::multiply(const Cfg& _c, double _s, bool _norm) {
  vector<double> v;
  for(vector<double>::const_iterator i = _c.m_v.begin(); i != _c.m_v.end(); ++i)
    v.push_back((*i)*_s);
  m_v = v;
  if(_norm)
    NormalizeOrientation();
}

void
Cfg::divide(const Cfg& _c, double _s) {
  vector<double> v;
  for(vector<double>::const_iterator i = _c.m_v.begin(); i != _c.m_v.end(); ++i)
    v.push_back((*i)/_s);
  m_v = v;
  NormalizeOrientation();
}

Cfg&
Cfg::operator=(const Cfg& _c) {
  m_v.clear();
  m_v = _c.GetData();
  m_labelMap = _c.m_labelMap;
  m_statMap = _c.m_statMap;

  return *this;
}

void
Cfg::WeightedSum(const Cfg& _first, const Cfg& _second, double _weight) {
  vector<double> v;
  for(size_t i = 0; i < m_dof; ++i)
    v.push_back(_first.m_v[i]*(1.-_weight) + _second.m_v[i]*_weight);
  m_v = v;
  NormalizeOrientation();
}

bool
Cfg::IsWithinResolution(const Cfg& _c, double _positionRes, double _orientationRes) const {
  Cfg* diff = this->CreateNewCfg();
  diff->subtract(*this, _c);

  bool result = ((diff->PositionMagnitude() <= _positionRes) &&
      (diff->OrientationMagnitude() <= _orientationRes));
  delete diff;
  return result;
}	

//---------------------------------------------
// Input/Output operators for Cfg
//---------------------------------------------
istream&
operator>>(istream& _s, Cfg& _pt) {
  _pt.Read(_s);
  return _s;
}


ostream&
operator<<(ostream& _s, const Cfg& _pt) {
  _pt.Write(_s);
  return _s;
}


void
Cfg::Write(ostream& _os) const {
  for(vector<double>::const_iterator i = m_v.begin(); i != m_v.end(); ++i)
    _os << setw(4) << *i << ' ';
}

void
Cfg::Read(istream &_is) {
  for(vector<double>::iterator i = m_v.begin(); i != m_v.end(); ++i)
    _is >> (*i);
}

void
Cfg::PrintLinkConfigurations(Environment* _env, vector<Vector6D>& _cfigs) const {
  ConfigEnvironment(_env);
  int robot = _env->GetRobotIndex();
  int numOfLink = _env->GetMultiBody(robot)->GetFreeBodyCount();

  _cfigs.erase(_cfigs.begin(), _cfigs.end());
  for(int i = 0; i < numOfLink; i++) {
    Transformation tmp = _env->GetMultiBody(robot)->GetFreeBody(i)->WorldTransformation();
    Orientation ori = tmp.m_orientation;
    ori.ConvertType(Orientation::FixedXYZ);
    Vector6D v6 = Vector6D(tmp.m_position[0], tmp.m_position[1], tmp.m_position[2],
        ori.gamma/TWOPI, ori.beta/TWOPI, ori.alpha/TWOPI);
    _cfigs.push_back(v6);        
  }
}

const vector<double>&
Cfg::GetData() const {
  return m_v;
}

size_t
Cfg::DOF() {
  return m_dof;
}

size_t
Cfg::PosDOF() {
  size_t posDof = 0;
  for(vector<Robot>::const_iterator rit = m_robots.begin(); rit != m_robots.end(); rit++) {
    if(rit->m_base == Robot::PLANAR)
      posDof += 2;
    else if(rit->m_base == Robot::VOLUMETRIC)
      posDof += 3;
  }
  return posDof;
}

void
Cfg::SetData(vector<double>& _data) {
  if(_data.size() != m_dof) {
    cout << "\n\nERROR in Cfg::SetData, ";
    cout << "DOF of data and Cfg are not equal " << _data.size() << "\t!=\t" << m_dof << endl; 
    exit(-1);
  }
  m_v = _data;
}

// Set a single parameter in the configuration (i.e., x,y,z,roll...)
// _param = the parameter number to set
// _value = the value to set the parameter as
int
Cfg::SetSingleParam(size_t _param, double _value, bool _norm) {   
  if((_param >= 0) && (_param < m_dof)) {
    m_v[_param] = _value;
    if(_norm)
      NormalizeOrientation(_param);
    return 1;
  }
  else {
    return 0;
  } 
}

// Increment a single parameter in the configuration (i.e., x,y,z,roll...)
// _param = the parameter number to set
// _value = the value to increment the parameter by
int
Cfg::IncSingleParam(size_t _param, double _value) {    
  if((_param >= 0) && (_param < m_dof)) {
    m_v[_param] += _value;
    NormalizeOrientation(_param);
    return 1;
  }
  else {
    return 0;
  } 
}

// Retreive a single parameter in the configuration (i.e., x,y,z,roll...)
// _param = the parameter number to retreive
double
Cfg::GetSingleParam(size_t _param) const {
  if((_param >= 0) && (_param < m_dof)) {
    return m_v[_param];
  }
  else {
    return 0;
  }
}

vector<double>
Cfg::GetPosition() const {
  vector<double> ret;  
  for(size_t i = 0; i < m_dof; ++i) {
    if(m_dofTypes[i] == POS)
      ret.push_back(m_v[i]);
  }
  return ret;
}

vector<double>
Cfg::GetOrientation() const {
  vector<double> ret;
  for(size_t i = 0; i < m_dof; ++i) {
    if(m_dofTypes[i] != POS)
      ret.push_back(m_v[i]);
  }     
  return ret;
}

double
Cfg::PositionMagnitude() const {
  double result = 0.0;
  for(size_t i = 0; i < m_dof; ++i) 
    if(m_dofTypes[i] == POS)
      result += sqr(m_v[i]);
  return sqrt(result);
}

double
Cfg::OrientationMagnitude() const {
  double result = 0.0;
  for(size_t i = 0; i < m_dof; ++i) {
    if(m_dofTypes[i] != POS)
      result += m_v[i] > 0.5 ? sqr(1.0 - m_v[i]) : sqr(m_v[i]);
  }
  return sqrt(result);
}

// tests whether or not robot in this configuration has every vertex inside
// the environment specified bounding box
bool
Cfg::InBoundary(Environment* _env, shared_ptr<Boundary> _bb) const {
  return _bb->InBoundary(*this, _env); 
}

bool
Cfg::InBoundary(Environment* _env) const {
  return InBoundary(_env,_env->GetBoundary());
}

void
Cfg::GetResolutionCfg(Environment* _env) {
  m_v.clear();
  double posRes = _env->GetPositionRes();
  double oriRes = _env->GetOrientationRes();

  for(size_t i = 0; i < m_dof; i++)
    if(m_dofTypes[i] == POS) 
      m_v.push_back(posRes);
    else 
      m_v.push_back(oriRes);

  NormalizeOrientation();
}

void
Cfg::GetPositionOrientationFrom2Cfg(const Cfg& _c1, const Cfg& _c2) {
  vector<double> v;
  for(size_t i = 0; i < m_dof; ++i) {
    if(m_dofTypes[i] == POS)
      v.push_back(_c1.m_v[i]);
    else
      v.push_back(_c2.m_v[i]);
  }
  m_v = v;
  NormalizeOrientation();
}

// _p1 & _p2 are two endpts of a line segment
// find the closest point to the current cfg on that line segment
// it could be one of the two endpoints of course
void
Cfg::ClosestPtOnLineSegment(const Cfg& _current, const Cfg& _p1, const Cfg& _p2) {

  Cfg* b = _p2.CreateNewCfg();
  b->subtract(_p2, _p1);

  Cfg* c = _current.CreateNewCfg();
  c->subtract(_current, _p1);
  
  double bDotC = 0;
  double bSquared = 0;

  vector<double>::const_iterator itb, itc;
  for (itb = b->m_v.begin(), itc = c->m_v.begin(); itb < b->m_v.end(); ++itb, ++itc) {
    bDotC += (*itb)*(*itc);
    bSquared += (*itb)*(*itb);
  }

  if (bDotC <= 0) {
    //return p1;
    *this = _p1;
  } 
  else if (bDotC >= bSquared) {
    //return p2;
    *this = _p2;
  } 
  else {
    this->multiply(*b, bDotC/bSquared);
    this->add(_p1, *this);
  }

  delete b;
  delete c;
}

void
Cfg::Increment(const Cfg& _increment) {
  for(size_t i = 0; i < m_dof; ++i)
    m_v[i] += _increment.m_v[i];
  NormalizeOrientation();
}

void
Cfg::IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment) {
  double tmp;
  size_t i;

  ///For Position
  for(i = 0; i < m_dof; ++i) {
    if(m_dofTypes[i] == POS || m_dofTypes[i] == JOINT) {
      //If the diff between _goal and c is smaller than _increment
      if(fabs(_goal.m_v[i]-m_v[i]) < fabs(_increment.m_v[i]))
        m_v[i] = _goal.m_v[i];
      else
        m_v[i] += _increment.m_v[i];
    }
  }

  ///For Oirentation
  for(i = 0; i < m_dof; ++i) {
    if(m_dofTypes[i] == ROT) {
      if(m_v[i] != _goal.m_v[i]) {
        double orientationIncr = _increment.m_v[i] < 0.5 ? _increment.m_v[i] : 1-_increment.m_v[i];
        tmp = DirectedAngularDistance(m_v[i], _goal.m_v[i]);
        if(fabs(tmp) < orientationIncr) {
          m_v[i] = _goal.m_v[i];
        }
        else {
          m_v[i] += _increment.m_v[i];
          m_v[i] = m_v[i] - floor(m_v[i]);
        }
      }
    }
  }
}

void
Cfg::FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks, double _positionRes, double _orientationRes) {
  Cfg* diff = _goal.CreateNewCfg();
  diff->subtract(_goal, _start);

  // adding two basically makes this a rough ceiling...
  *_nTicks = floor(max(diff->PositionMagnitude()/_positionRes, 
      diff->OrientationMagnitude()/_orientationRes) + 0.5);
  delete diff;

  this->FindIncrement(_start, _goal, *_nTicks);
}

void
Cfg::FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks) {
  vector<double> incr;
  for(size_t i = 0; i < m_dof; ++i) {
    if(m_dofTypes[i] == POS) 
      incr.push_back((_goal.m_v[i] - _start.m_v[i])/_nTicks);
    else if(m_dofTypes[i] == JOINT) {
      double a = _start.m_v[i];
      double b = _goal.m_v[i];
      // normalize both a and b to [0, 1)
      a = a - floor(a);
      b = b - floor(b);
      if(a >= 0.5) a -= 1.0;
      if(b >= 0.5) b -= 1.0;
      incr.push_back((b-a)/_nTicks);
    }
    else if(m_dofTypes[i] == ROT) {
      incr.push_back(DirectedAngularDistance(_start.m_v[i], _goal.m_v[i])/_nTicks);
    }
  }

  m_v = incr;
  NormalizeOrientation();
}

// generates random configuration where workspace robot's EVERY VERTEX
// is guaranteed to lie within the environment specified bounding box
void
Cfg::GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundary());
}

void
Cfg::GetRandomCfg(Environment* _env, shared_ptr<Boundary> _bb) {
  
  // Probably should do something smarter than 3 strikes and exit.
  // eg, if it fails once, check size of bounding box vs robot radius
  // and see if user has an impossibly small (for this robot) bounding
  // box specified
  size_t tries = 100;
  while(tries-- > 0) {
    this->GetRandomCfgCenterOfMass(_env, _bb);
   
    if(this->InBoundary(_env, _bb))
      return;
  }//endwhile

  // Print error message and some helpful (I hope!) statistics and exit...
  cout << "\n\nERROR: GetRandomCfg not able to find anything in bounding box."
    <<   "\n       robot radius is "
    << _env->GetMultiBody(_env->GetRobotIndex())->GetBoundingSphereRadius();
  _bb->Print(cout);
  exit(-1);
}

Cfg*
Cfg::CreateNewCfg(vector<double>& _data) const {
  if(_data.size() != m_dof) {
    cout << "\n\nERROR in Cfg::CreateNewCfg(vector<double>), ";
    cout << "size of vector does not equal dof " << _data.size() << "\t!=\t" << m_dof << endl;
    exit(-1);
  }
  Cfg* tmp = this->CreateNewCfg();
  tmp->m_v = _data;
  return tmp;
}

// Normalize the orientation to the some range.
void
Cfg::NormalizeOrientation(int _index) {
  if(_index == -1) {
    for(size_t i = 0; i < m_dof; ++i) {
      if(m_dofTypes[i] != POS) {
        m_v[i] = m_v[i] - floor(m_v[i]);
      }
    }
  } 
  else if(m_dofTypes[_index] != POS) {  // orientation index
    m_v[_index] = m_v[_index] - floor(m_v[_index]);
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

vector<Vector3D>
Cfg::PolyApprox(Environment* _env) const {
  vector<Vector3D> result;
  ConfigEnvironment(_env);
  _env->GetMultiBody(_env->GetRobotIndex())->PolygonalApproximation(result);
  return result;
}

/*double
Cfg::GetSmoothingValue(StatClass& _stats, CDInfo& _cdInfo, const ClearanceParams& _cParams) {
  CfgType tmp;
  GetApproxCollisionInfo(*((CfgType*)this), tmp, _stats, _cdInfo, _cParams);
  return _cdInfo.m_minDist;
}*/
