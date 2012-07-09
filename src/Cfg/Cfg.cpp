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

/////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "CfgTypes.h"
#include "Cfg.h"
#include "MultiBody.h"
#include "Environment.h"
#include "DistanceMetrics.h"
#include "CollisionDetection.h"
#include "MetricUtils.h"
#include "ValidityChecker.hpp"
#include "MPProblem.h"

#define EQ(a,b)  (fabs(a-b)<0.0001)

ClearanceInfo::
  ~ClearanceInfo() {
    if(m_direction != NULL)
      delete m_direction;
  }

////////////////////////////////////////////////////////////////////
size_t Cfg::m_dof;
vector<Cfg::DofType> Cfg::m_dofTypes;
vector<Robot> Cfg::m_robots;

Cfg::Cfg(const Cfg& _other){
  m_v = _other.m_v;
  m_dof = _other.m_dof;
  m_labelMap = _other.m_labelMap;
  m_statMap = _other.m_statMap;
}

void Cfg::InitRobots(vector<Robot>& _robots){
  m_robots=_robots;
  for(vector<Robot>::iterator rit = m_robots.begin(); rit!=m_robots.end(); rit++){
    if(rit->m_base==Robot::PLANAR){
      m_dofTypes.push_back(POS);
      m_dofTypes.push_back(POS);
      if(rit->m_baseMovement == Robot::ROTATIONAL)
        m_dofTypes.push_back(ROT);
    }
    if(rit->m_base==Robot::VOLUMETRIC){
      m_dofTypes.push_back(POS);
      m_dofTypes.push_back(POS);
      m_dofTypes.push_back(POS);
      if(rit->m_baseMovement == Robot::ROTATIONAL){
        m_dofTypes.push_back(ROT);
        m_dofTypes.push_back(ROT);
        m_dofTypes.push_back(ROT);
      }
    }
    for(Robot::JointIT jit = rit->m_joints.begin(); jit!=rit->m_joints.end(); jit++){
      if(jit->second==Robot::REVOLUTE){
        m_dofTypes.push_back(JOINT);
      }
      else if(jit->second==Robot::SPHERICAL){
        m_dofTypes.push_back(JOINT);
        m_dofTypes.push_back(JOINT);
      }
    }
  }
  m_dof = m_dofTypes.size();
}

bool Cfg::operator==(const Cfg &_tmp) const {
  return AlmostEqual(_tmp);
}

bool Cfg::operator!=( const Cfg& _tmp) const {
  return !(*this==_tmp);
}

bool Cfg::AlmostEqual(const Cfg& _c) const {
  for(size_t i=0; i<m_dof; ++i) {
    if(m_dofTypes[i]==POS || m_dofTypes[i]==JOINT) {
      if(!EQ(m_v[i], _c.m_v[i]))
        return false;
    } else {
      if(!EQ(DirectedAngularDistance(m_v[i], _c.m_v[i]), 0.0)) 
        return false;
    }
  }
  return true;
}

void Cfg::add(const Cfg& _c1, const Cfg& _c2) {
  vector<double> v;
  for(size_t i=0; i<m_dof; ++i)
    v.push_back(_c1.m_v[i]+_c2.m_v[i]);
  m_v = v;
  NormalizeOrientation();
}

void Cfg::subtract(const Cfg& _c1, const Cfg& _c2) {
  vector<double> v;
  for(size_t i=0; i<m_dof; ++i){
    if(m_dofTypes[i] == POS || m_dofTypes[i]== JOINT)
      v.push_back(_c1.m_v[i]-_c2.m_v[i]);
    else
      v.push_back(DirectedAngularDistance(_c1.m_v[i], _c2.m_v[i]));
  }
  m_v = v;
  NormalizeOrientation();
}

void Cfg::negative(const Cfg& _c) {
  vector<double> v;    
  for(vector<double>::const_iterator V = _c.m_v.begin(); V != _c.m_v.end(); ++V)
    v.push_back(-(*V));
  m_v = v;
  NormalizeOrientation();
}

void Cfg::multiply(const Cfg& _c, double _s, bool _norm) {
  vector<double> v;
  for(vector<double>::const_iterator V = _c.m_v.begin(); V != _c.m_v.end(); ++V)
    v.push_back((*V)*_s);
  m_v = v;
  if ( _norm )
    NormalizeOrientation();
}

void Cfg::divide(const Cfg& _c, double _s) {
  vector<double> v;
  for(vector<double>::const_iterator V = _c.m_v.begin(); V != _c.m_v.end(); ++V)
    v.push_back((*V)/_s);
  m_v = v;
  NormalizeOrientation();
}

Cfg& Cfg::operator=(const Cfg& _c){
  m_v.clear();
  m_v = _c.GetData();
  m_labelMap = _c.m_labelMap;
  m_statMap = _c.m_statMap;

  return *this;
}

void Cfg::WeightedSum(const Cfg& _first, const Cfg& _second, double _weight) {
  vector<double> v;
  for(size_t i=0; i<m_dof; ++i)
    v.push_back(_first.m_v[i]*(1.-_weight) + _second.m_v[i]*_weight);
  m_v = v;
  NormalizeOrientation();
}

bool Cfg::IsWithinResolution(const Cfg& _c, double _positionRes, double _orientationRes) const {
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
istream& operator>> (istream& _s, Cfg& _pt) {
  _pt.Read(_s);
  return _s;
}


ostream& operator<< (ostream& _s, const Cfg& _pt) {
  _pt.Write(_s);
  return _s;
}


void Cfg::Write(ostream& _os) const {
  for(vector<double>::const_iterator V = m_v.begin(); V != m_v.end(); ++V)
    _os << setw(4)<<*V<<' ';
}

void Cfg::Read(istream &_is) {
  for(vector<double>::iterator V = m_v.begin(); V != m_v.end(); ++V)
    _is >> (*V);
}

void Cfg::PrintLinkConfigurations(Environment* _env, vector<Vector6D>& _cfigs) const {
  ConfigEnvironment(_env);
  int robot = _env->GetRobotIndex();
  int numOfLink = _env->GetMultiBody(robot)->GetFreeBodyCount();

  _cfigs.erase(_cfigs.begin(), _cfigs.end());
  for(int i=0; i<numOfLink; i++) {
    Transformation tmp = _env->GetMultiBody(robot)->GetFreeBody(i)->WorldTransformation();
    Orientation ori = tmp.m_orientation;
    ori.ConvertType(Orientation::FixedXYZ);
    Vector6D v6 = Vector6D(tmp.m_position[0], tmp.m_position[1], tmp.m_position[2],
        ori.gamma/TWOPI, ori.beta/TWOPI, ori.alpha/TWOPI);
    _cfigs.push_back(v6);        
  }
}

const vector<double>& Cfg::GetData() const {
  return m_v;
}

size_t Cfg::DOF(){
  return m_dof;
}

size_t Cfg::PosDOF(){
  size_t posDof = 0;
  for(vector<Robot>::const_iterator rit = m_robots.begin(); rit!=m_robots.end(); rit++){
    if(rit->m_base==Robot::PLANAR)
      posDof+=2;
    else if (rit->m_base==Robot::VOLUMETRIC)
      posDof+=3;
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
int Cfg::SetSingleParam(size_t _param, double _value, bool _norm) {    
  if ((_param>=0) && (_param<m_dof)) {
    m_v[_param]=_value;
    if ( _norm )
      NormalizeOrientation(_param);
    return 1;
  } else {
    return 0;
  } 
}

// Increment a single parameter in the configuration (i.e., x,y,z,roll...)
// _param = the parameter number to set
// _value = the value to increment the parameter by
int Cfg::IncSingleParam(size_t _param, double _value) {    
  if ((_param>=0) && (_param<m_dof)) {
    m_v[_param]+=_value;
    NormalizeOrientation(_param);
    return 1;
  } else {
    return 0;
  } 
}

// Retreive a single parameter in the configuration (i.e., x,y,z,roll...)
// _param = the parameter number to retreive
double Cfg::GetSingleParam(size_t _param) const {
  if ((_param>=0) && (_param<m_dof)) {
    return m_v[_param];
  } else {
    return 0;
  }
}

vector<double> Cfg::GetPosition() const {
  vector<double> ret;  
  for(size_t i=0; i<m_dof; ++i) {
    if(m_dofTypes[i]==POS)
      ret.push_back(m_v[i]);
  }
  return ret;
}

vector<double> Cfg::GetOrientation() const {
  vector<double> ret;
  for(size_t i=0; i<m_dof; ++i) {
    if(m_dofTypes[i]!=POS)
      ret.push_back(m_v[i]);
  }     
  return ret;
}

double Cfg::PositionMagnitude() const {
  double result = 0.0;
  for(size_t i=0; i<m_dof; ++i) 
    if(m_dofTypes[i]==POS)
      result += sqr(m_v[i]);
  return sqrt(result);
}

double Cfg::OrientationMagnitude() const {
  double result = 0.0;
  for(size_t i=0; i<m_dof; ++i) {
    if(m_dofTypes[i]!=POS)
      result += m_v[i] > 0.5 ? sqr(1.0 - m_v[i]) : sqr(m_v[i]);
  }
  return sqrt(result);
}

// tests whether or not robot in this configuration has every vertex inside
// the environment specified bounding box
bool Cfg::InBoundary(Environment* _env,shared_ptr<Boundary> _bb) const {
  return _bb->InBoundary(*this); 
}

bool Cfg::InBoundary(Environment* _env) const {
  return InBoundary(_env,_env->GetBoundary());
}

void Cfg::GetResolutionCfg(Environment* _env) {
  m_v.clear();
  double posRes = _env->GetPositionRes();
  double oriRes = _env->GetOrientationRes();

  for(size_t i=0;i<m_dof;i++)
    if(m_dofTypes[i]==POS) 
      m_v.push_back(posRes);
    else 
      m_v.push_back(oriRes);

  NormalizeOrientation();
}


void Cfg::GetPositionOrientationFrom2Cfg(const Cfg& _c1, const Cfg& _c2) {
  vector<double> v;
  for(size_t i=0; i<m_dof; ++i) {
    if(m_dofTypes[i]==POS)
      v.push_back(_c1.m_v[i]);
    else
      v.push_back(_c2.m_v[i]);
  }
  m_v = v;
  NormalizeOrientation();
}

void Cfg::GetMovingSequenceNodes(const Cfg& _other, vector<double> _sValue, vector<Cfg*>& _result) const {
  Cfg* thisCopy = this->CreateNewCfg();
  _result.push_back(thisCopy);

  Cfg* weightedSum = this->CreateNewCfg();
  double oWeight = 0.0;
  for(vector<double>::const_iterator S = _sValue.begin(); S != _sValue.end(); ++S) {
    weightedSum->WeightedSum(*this, _other, *S);

    Cfg* s1 = this->CreateNewCfg();
    s1->GetPositionOrientationFrom2Cfg(*weightedSum, *_result.back());
    _result.push_back(s1);

    oWeight += 1.0 / _sValue.size();
    weightedSum->WeightedSum(*this, _other, oWeight);

    Cfg* s2 = this->CreateNewCfg();
    s2->GetPositionOrientationFrom2Cfg(*s1, *weightedSum);
    _result.push_back(s2);
  }

  Cfg* otherCopy = _other.CreateNewCfg();
  _result.push_back(otherCopy);

  delete weightedSum;
} 

void Cfg::GetMovingSequenceNodes(const Cfg& _other, double _s, vector<Cfg*>& _result) const {
  Cfg* thisCopy = this->CreateNewCfg();
  _result.push_back(thisCopy);

  Cfg* weightedSum = this->CreateNewCfg();
  weightedSum->WeightedSum(*this, _other, _s);

  Cfg* s1 = this->CreateNewCfg();
  s1->GetPositionOrientationFrom2Cfg(*weightedSum, *this);
  _result.push_back(s1);

  Cfg* s2 = this->CreateNewCfg();
  s2->GetPositionOrientationFrom2Cfg(*weightedSum, _other);
  _result.push_back(s2);

  Cfg* otherCopy = _other.CreateNewCfg();
  _result.push_back(otherCopy);

  delete weightedSum;
}


// _pt1 & _pt2 are two endpts of a line segment
// find the closest point to the current cfg on that line segment
// it could be one of the two endpoints of course
void Cfg::ClosestPtOnLineSegment(const Cfg& _current, const Cfg& _pt1, const Cfg& _pt2) {

  Cfg* B = _pt2.CreateNewCfg();
  B->subtract(_pt2, _pt1);

  Cfg* C = _current.CreateNewCfg();
  C->subtract(_current, _pt1);

  double B_dot_C = 0;
  double B_squared = 0;

  vector<double>::const_iterator b, c;
  for (b = B->m_v.begin(), c = C->m_v.begin(); b < B->m_v.end(); ++b, ++c) {
    B_dot_C += (*b)*(*c);
    B_squared += (*b)*(*b);
  }

  if (B_dot_C <= 0) {
    //return pt1;
    *this = _pt1;
  } else if (B_dot_C >= B_squared) {
    //return pt2;
    *this = _pt2;
  } else {
    this->multiply(*B, B_dot_C/B_squared);
    this->add(_pt1, *this);
  }

  delete B;
  delete C;
}

void Cfg::Increment(const Cfg& _increment) {
  for(size_t i=0; i<m_dof; ++i)
    m_v[i] += _increment.m_v[i];
  NormalizeOrientation();
}

void Cfg::IncrementTowardsGoal(const Cfg &_goal, const Cfg &_increment) {
  double tmp;
  size_t i;

  ///For Position
  for(i=0; i<m_dof; ++i) {
    if(m_dofTypes[i]==POS || m_dofTypes[i]==JOINT){
      //If the diff between _goal and c is smaller than _increment
      if( fabs(_goal.m_v[i]-m_v[i]) < fabs(_increment.m_v[i]))
        m_v[i] = _goal.m_v[i];
      else
        m_v[i] += _increment.m_v[i];
    }
  }

  ///For Oirentation
  for(i=0; i<m_dof; ++i) {
    if(m_dofTypes[i]==ROT){
      if(m_v[i] != _goal.m_v[i]) {
        double orientationIncr = _increment.m_v[i] < 0.5 ? _increment.m_v[i] : 1-_increment.m_v[i];
        tmp = DirectedAngularDistance(m_v[i], _goal.m_v[i]);
        if(fabs(tmp) < orientationIncr) {
          m_v[i]=_goal.m_v[i];
        } else {
          m_v[i] += _increment.m_v[i];
          m_v[i] = m_v[i] - floor(m_v[i]);
        }
      }
    }
  }
}

void Cfg::FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks, double _positionRes, double _orientationRes) {
  Cfg* diff = _goal.CreateNewCfg();
  diff->subtract(_goal, _start);

  // adding two basically makes this a rough ceiling...
  *_nTicks = max(diff->PositionMagnitude()/_positionRes, 
      diff->OrientationMagnitude()/_orientationRes) + 2;
  delete diff;

  this->FindIncrement(_start, _goal, *_nTicks);
}

void Cfg::FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks) {
  vector<double> incr;
  for(size_t i=0; i<m_dof; ++i) {
    if(m_dofTypes[i]==POS) 
      incr.push_back((_goal.m_v[i] - _start.m_v[i])/_nTicks);
    else if(m_dofTypes[i]==JOINT){
      double a = _start.m_v[i];
      double b = _goal.m_v[i];
      // normalize both a and b to [0, 1)
      a = a - floor(a);
      b = b - floor(b);
      if(a>=0.5)a-=1.0;
      if(b>=0.5)b-=1.0;
      incr.push_back((b-a)/_nTicks);
    }
    else if(m_dofTypes[i]==ROT){
      incr.push_back(DirectedAngularDistance(_start.m_v[i], _goal.m_v[i])/_nTicks);
    }
  }

  m_v = incr;
  NormalizeOrientation();
}

// generates random configuration where workspace robot's EVERY VERTEX
// is guaranteed to lie within the environment specified bounding box
void Cfg::GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundary());
}

void Cfg::GetRandomCfg(Environment* _env, shared_ptr<Boundary> _bb) {
  
  // Probably should do something smarter than 3 strikes and exit.
  // eg, if it fails once, check size of bounding box vs robot radius
  // and see if user has an impossibly small (for this robot) bounding
  // box specified
  size_t tries = 100;
  while ( tries-- > 0) {
    this->GetRandomCfgCenterOfMass(_env, _bb);
   
    if (this->InBoundary(_env,_bb))
      return;
  }//endwhile

  // Print error message and some helpful (I hope!) statistics and exit...
  cout << "\n\nERROR: GetRandomCfg not able to find anything in bounding box."
    <<   "\n       robot radius is "
    << _env->GetMultiBody(_env->GetRobotIndex())->GetBoundingSphereRadius();
  _bb->Print(cout);
  exit(-1);
}

// generates a random cfg with a given length 
void Cfg::GetRandomCfg(Environment* _env, shared_ptr<DistanceMetricMethod> _dm, double _length) {
  m_v = vector<double>(m_dof, 0);
  Cfg* origin = this->CreateNewCfg();

  GetRandomCfg(_env);
  _dm->ScaleCfg(_env, _length, *origin, *this);

  NormalizeOrientation();
}

Cfg* Cfg::CreateNewCfg() const{
  Cfg* tmp = new CfgType();
  *tmp = *this;
  return tmp;
}

Cfg* Cfg::CreateNewCfg(vector<double>& _data) const {
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
void Cfg::NormalizeOrientation(int _index) {
  if(_index == -1) {
    for(size_t i=0; i<m_dof; ++i){
      if(m_dofTypes[i]!=POS){
        m_v[i] = m_v[i] - floor(m_v[i]);
      }
    }
  } 
  else if(m_dofTypes[_index]!=POS) {  // orientation index
    m_v[_index] = m_v[_index] - floor(m_v[_index]);
  } 
}

bool Cfg::GetLabel(string _label) {
  if(IsLabel(_label)){
    return m_labelMap[_label];
  }
  else{
    cout << "Cfg::GetLabel -- I cannot find Label =  " << _label << endl;
    exit(-1);
  } 
}

bool Cfg::IsLabel(string _label) {
  bool label = false;
  if(m_labelMap.count(_label) > 0)
    label = true;
  else
    label= false;
  return label;
}

void Cfg::SetLabel(string _label, bool _value) {
  m_labelMap[_label] = _value;
}


double Cfg::GetStat(string _stat) {
  if(IsStat(_stat)) {
    return m_statMap[_stat];
  }
  else {
    cout << "Cfg::GetStat -- I cannot find Stat =  " << _stat << endl;
    exit(-1);
  }
}

bool Cfg::IsStat(string _stat) {
  bool stat = false;
  if(m_statMap.count(_stat) > 0)
    stat= true ;
  else
    stat=false;
  return stat;
}

void Cfg::SetStat(string _stat,double _value) {
  m_statMap[_stat] = _value;
}

vector<Vector3D> Cfg::PolyApprox(Environment* _env) const {
  vector<Vector3D> result;
  ConfigEnvironment(_env);
  _env->GetMultiBody(_env->GetRobotIndex())->PolygonalApproximation(result);
  return result;
}

double Cfg::GetSmoothingValue(MPProblem* _mp, Environment *_env,StatClass& _stats,
    string _vc, CDInfo& _cdInfo, string _dm, int _n, bool _bl ){
  CfgType tmp;
  GetApproxCollisionInfo(_mp, *((CfgType*)this), tmp, _env, _stats, _cdInfo, _vc, _dm, _n, _n, true, true);
  return _cdInfo.min_dist;
}
