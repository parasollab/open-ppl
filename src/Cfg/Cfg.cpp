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
#include "MPProblem/Geometry/MultiBody.h"
#include "MPProblem/Environment.h"
#include "Utilities/MetricUtils.h"

ClearanceInfo::~ClearanceInfo() {
  if(m_direction != NULL)
    delete m_direction;
}

////////////////////////////////////////////////////////////////////
size_t Cfg::m_dof;
size_t Cfg::m_posdof;
size_t Cfg::m_numJoints;
vector<Cfg::DofType> Cfg::m_dofTypes;
vector<Robot> Cfg::m_robots;

Cfg::Cfg() {
  m_v.clear();
  m_v.resize(m_dof, 0.0);
  m_robotIndex = 0;
  m_witnessCfg.reset();
}

Cfg::Cfg(const Cfg& _other) :
  m_v(_other.m_v), 
  m_robotIndex(_other.m_robotIndex),
  m_labelMap(_other.m_labelMap),
  m_statMap(_other.m_statMap),
  m_witnessCfg(_other.m_witnessCfg){}

void
Cfg::InitRobots(vector<Robot>& _robots) {
  m_robots = _robots;
  m_posdof = 0;
  m_numJoints = 0;
  for(vector<Robot>::iterator rit = m_robots.begin(); rit != m_robots.end(); rit++) {
    if(rit->m_base == Robot::PLANAR) {
      m_dofTypes.push_back(POS);
      m_dofTypes.push_back(POS);
      m_posdof += 2;
      if(rit->m_baseMovement == Robot::ROTATIONAL)
        m_dofTypes.push_back(ROT);
    }
    if(rit->m_base == Robot::VOLUMETRIC) {
      m_dofTypes.push_back(POS);
      m_dofTypes.push_back(POS);
      m_dofTypes.push_back(POS);
      m_posdof += 3;
      if(rit->m_baseMovement == Robot::ROTATIONAL) {
        m_dofTypes.push_back(ROT);
        m_dofTypes.push_back(ROT);
        m_dofTypes.push_back(ROT);
      }
    }
    for(Robot::JointIT jit = rit->m_joints.begin(); jit != rit->m_joints.end(); jit++) {
      if(jit->get<0>() == Robot::REVOLUTE) {
        m_dofTypes.push_back(JOINT);
        m_numJoints++;
      }
      else if(jit->get<0>() == Robot::SPHERICAL) {
        m_dofTypes.push_back(JOINT);
        m_dofTypes.push_back(JOINT);
        m_numJoints+=2;
      }
      else if(jit->get<0>() == Robot::NONACTUATED){
        //skip, do nothing
      }
    }
  }
  m_dof = m_dofTypes.size();
}

Cfg&
Cfg::operator=(const Cfg& _cfg) {
  if(this != &_cfg){
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
  for(size_t i = 0; i < m_dof; ++i) {
    if(m_dofTypes[i] == POS || m_dofTypes[i] == JOINT) {
      if(fabs(m_v[i]-_cfg[i]) > numeric_limits<float>::epsilon())
        return false;
    }
    else {
      if(fabs(DirectedAngularDistance(m_v[i], _cfg[i])) > numeric_limits<float>::epsilon()) 
        return false;
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
  for(size_t i = 0; i < m_dof; ++i)
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
  for(size_t i = 0; i < m_dof; ++i){
    if(m_dofTypes[i] == POS || m_dofTypes[i] == JOINT)
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
  for(size_t i = 0; i < m_dof; ++i)
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
  for(size_t i = 0; i < m_dof; ++i)
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
  for(size_t i = 0; i < m_dof; ++i)
    m_v[i] /= _d;
  NormalizeOrientation();
  m_witnessCfg.reset();
  return *this;
}

double&
Cfg::operator[](size_t _dof){
  assert(_dof >= 0 && _dof <= m_dof);
  m_witnessCfg.reset();
  return m_v[_dof];
}

const double&
Cfg::operator[](size_t _dof) const {
  assert(_dof >= 0 && _dof <= m_dof);
  return m_v[_dof];
}

//---------------------------------------------
// Input/Output operators for Cfg
//---------------------------------------------
void
Cfg::Read(istream& _is){
  //first read in robot index, and then read in DOF values
  _is >> m_robotIndex;
  //if this failed, then we're done reading Cfgs
  if (_is.fail())
    return;
  for(vector<double>::iterator i = m_v.begin(); i != m_v.end(); ++i){
    _is >> (*i);
    cout << "Read dof: " << *i << endl;
    if (_is.fail()){
      cerr << "Cfg::operator>> error - failed reading values for all dofs" << endl;
      exit(1);
    }
  }
}

void
Cfg::Write(ostream& _os) const{
  //write out robot index, and then dofs
  _os << setw(4) << m_robotIndex << ' ';
  for(vector<double>::const_iterator i = m_v.begin(); i != m_v.end(); ++i)
    _os << setw(4) << *i << ' ';
  if (_os.fail()){
    cerr << "Cfg::Write error - failed to write to file" << endl;
    exit(1);
  }
}

istream&
operator>>(istream& _is, Cfg& _cfg){
  _cfg.Read(_is);
  _cfg.m_witnessCfg.reset();
  return _is;
}


ostream&
operator<<(ostream& _os, const Cfg& _cfg){
  _cfg.Write(_os);
  return _os;
}

void
Cfg::SetData(const vector<double>& _data) {
  if(_data.size() != m_dof) {
    cout << "\n\nERROR in Cfg::SetData, ";
    cout << "DOF of data and Cfg are not equal " << _data.size() << "\t!=\t" << m_dof << endl; 
    exit(-1);
  }
  m_v = _data;
  m_witnessCfg.reset();
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
Cfg::Magnitude() const {
  double result = 0.0;
  for(size_t i = 0; i < m_dof; ++i) 
    result += m_v[i]*m_v[i];
  return sqrt(result);
}

double
Cfg::PositionMagnitude() const {
  double result = 0.0;
  for(size_t i = 0; i < m_dof; ++i) 
    if(m_dofTypes[i] == POS)
      result += m_v[i]*m_v[i];
  return sqrt(result);
}

double
Cfg::OrientationMagnitude() const {
  double result = 0.0;
  for(size_t i = 0; i < m_dof; ++i) {
    if(m_dofTypes[i] != POS)
      result += m_v[i]*m_v[i];
  }
  return sqrt(result);
}

Vector3D
Cfg::GetRobotCenterPosition() const {
  double x = 0, y = 0, z = 0;
  int numRobots = m_robots.size();
  int index = 0;
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots.begin(); rit != m_robots.end(); rit++) {
    x += m_v[index];
    y += m_v[index + 1];
    if(rit->m_base == Robot::VOLUMETRIC)
      z += m_v[index + 2];
    index += 2;
    if(rit->m_base == Robot::VOLUMETRIC)
      index += 1;
    index += rit->m_joints.size();
  }

  return Vector3D(x/numRobots, y/numRobots, z/numRobots);
}

Vector3D 
Cfg::GetRobotCenterofMass(Environment* _env) const {
  ConfigEnvironment(_env);

  typedef vector<Robot>::iterator RIT;
  Vector3D com(0,0,0);
  int numbodies=0;
  shared_ptr<MultiBody> mb = _env->GetMultiBody(m_robotIndex);
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots.begin(); rit != m_robots.end(); rit++) {
    GMSPolyhedron poly = mb->GetFreeBody(rit->m_bodyIndex)->GetWorldPolyhedron();
    Vector3D polycom(0,0,0);
    for(vector<Vector3D>::const_iterator  vit = poly.m_vertexList.begin(); vit != poly.m_vertexList.end(); ++vit)
      polycom = polycom + (*vit);
    polycom = polycom / poly.m_vertexList.size();
    com = com + polycom;
    numbodies++;  

    for(Robot::JointIT i = rit->m_joints.begin(); i != rit->m_joints.end(); ++i){
      GMSPolyhedron poly1 = mb->GetFreeBody(i->get<1>().second)->GetWorldPolyhedron();
      Vector3D polycom1(0,0,0);
      for(vector<Vector3D>::const_iterator vit1 = poly1.m_vertexList.begin(); vit1 != poly1.m_vertexList.end(); ++vit1)
        polycom1 = polycom1 + (*vit1);
      polycom1 = polycom1 / poly1.m_vertexList.size();
      com = com + polycom1;
      numbodies++;

    }
  }

  com = com/numbodies;
  return com;
}

// generates random configuration where workspace robot's EVERY VERTEX
// is guaranteed to lie within the environment specified bounding box
void
Cfg::GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundary());
}

void
Cfg::GetRandomCfg(Environment* _env, shared_ptr<Boundary> _bb) {
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

  // Print error message and some helpful (I hope!) statistics and exit...
  cerr << "\n\nERROR: GetRandomCfg not able to find anything in boundary: " 
    << *_bb << ".\n       robot radius is "
    << _env->GetMultiBody(m_robotIndex)->GetBoundingSphereRadius() << ".";
  exit(-1);
}

bool
Cfg::ConfigEnvironment(Environment* _env) const {
  shared_ptr<MultiBody> mb = _env->GetMultiBody(m_robotIndex);
  int index = 0;
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots.begin(); rit != m_robots.end(); rit++) {
    int posIndex = index;
    double x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0;
    if(rit->m_base != Robot::FIXED) {
      x = m_v[posIndex];
      y = m_v[posIndex + 1];
      index += 2;
      if(rit->m_base == Robot::VOLUMETRIC) {
        index++;
        z = m_v[posIndex + 2];
      }
      if(rit->m_baseMovement == Robot::ROTATIONAL) {
        if(rit->m_base == Robot::PLANAR) {
          index++;
          gamma = m_v[posIndex + 2];
        }
        else {
          index += 3;
          alpha = m_v[posIndex + 3];
          beta = m_v[posIndex + 4];
          gamma = m_v[posIndex + 5];
        }
      }
      // configure the robot according to current Cfg: joint parameters
      // (and base locations/orientations for free flying robots.)
      Transformation t1 = Transformation(Orientation(Orientation::FixedXYZ, gamma*PI, beta*PI, alpha*PI), Vector3D(x,y,z));
      // update link i
      mb->GetFreeBody(rit->m_bodyIndex)->Configure(t1);
    }
    typedef Robot::JointMap::iterator MIT;
    for(MIT mit = rit->m_joints.begin(); mit != rit->m_joints.end(); mit++) {
      if(mit->get<0>() != Robot::NONACTUATED) {
        size_t second = mit->get<1>().second;
        mb->GetFreeBody(second)->GetBackwardConnection(0).GetDHparameters().theta = m_v[index]*PI;
        index++;
        if(mit->get<0>() == Robot::SPHERICAL){
          mb->GetFreeBody(second)->GetBackwardConnection(0).GetDHparameters().alpha = m_v[index]*PI;
          index++;
        }
      } 
    }  // config the robot
  }
  for(int i = 0; i < mb->GetFreeBodyCount(); i++) {
    shared_ptr<FreeBody> afb = mb->GetFreeBody(i);
    if(afb->ForwardConnectionCount() == 0)  // tree tips: leaves.
      afb->GetWorldTransformation();
  }
  return true;
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
  m_witnessCfg.reset();
}

void
Cfg::IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment) {
  ///For Position
  for(size_t i = 0; i < m_dof; ++i) {
    if(m_dofTypes[i] == POS || m_dofTypes[i] == JOINT) {
      //If the diff between _goal and c is smaller than _increment
      if(fabs(_goal.m_v[i]-m_v[i]) < fabs(_increment.m_v[i]))
        m_v[i] = _goal.m_v[i];
      else
        m_v[i] += _increment.m_v[i];
    }
  }

  ///For Oirentation
  for(size_t i = 0; i < m_dof; ++i) {
    if(m_dofTypes[i] == ROT) {
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
  *_nTicks = floor(max(diff.PositionMagnitude()/_positionRes, 
        diff.OrientationMagnitude()/_orientationRes) + 0.5);

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
      // normalize both a and b to [-1, 1)
      a = Normalize(a);
      b = Normalize(b);
      incr.push_back((b-a)/_nTicks);
    }
    else if(m_dofTypes[i] == ROT) {
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
  for(size_t i = 0; i < m_dof; ++i)
    v.push_back(_first.m_v[i]*(1.-_weight) + _second.m_v[i]*_weight);
  m_v = v;
  NormalizeOrientation();
  m_witnessCfg.reset();
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
  m_witnessCfg.reset();
}

vector<Vector3D>
Cfg::PolyApprox(Environment* _env) const {
  vector<Vector3D> result;
  ConfigEnvironment(_env);
  _env->GetMultiBody(m_robotIndex)->PolygonalApproximation(result);
  return result;
}

//Normalize the orientation to the range [-1, 1)
void
Cfg::NormalizeOrientation(int _index) {
  if(_index == -1) {
    for(size_t i = 0; i < m_dof; ++i) {
      if(m_dofTypes[i] != POS) {
        m_v[i] = Normalize(m_v[i]);
      }
    }
  } 
  else if(m_dofTypes[_index] != POS) {  // orientation index
    m_v[_index] = Normalize(m_v[_index]);
  } 
}

//generates random configuration within C-space
void
Cfg::GetRandomCfgImpl(Environment* _env, shared_ptr<Boundary> _bb) {
  m_v.clear();
  size_t index = 0;
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots.begin(); rit != m_robots.end(); rit++) {
    if(rit->m_base == Robot::PLANAR || rit->m_base == Robot::VOLUMETRIC) {
      Point3d p = _bb->GetRandomPoint();
      size_t posDOF = rit->m_base == Robot::VOLUMETRIC ? 3 : 2;
      for(size_t i = 0; i < posDOF; i++) {
        m_v.push_back(p[i]); 
      }
      if(rit->m_baseMovement == Robot::ROTATIONAL) {
        size_t oriDOF = rit->m_base == Robot::VOLUMETRIC ? 3 : 1;
        for(size_t i = 0; i < oriDOF; i++) {
          m_v.push_back(2.0*DRand()-1.0); 
        }
      }
    }
    for(Robot::JointIT i = rit->m_joints.begin(); i != rit->m_joints.end(); ++i) {
      if(i->get<0>() == Robot::REVOLUTE) {
        pair<double, double> r = i->get<2>();
        double t = DRand()*(r.second-r.first)+r.first;
        m_v.push_back(t);
        index++;
      }
      else if(i->get<0>() == Robot::SPHERICAL) {
        pair<double, double> r = i->get<2>();
        double t = DRand()*(r.second-r.first)+r.first;
        r = i->get<3>();
        double a = DRand()*(r.second-r.first)+r.first;
        m_v.push_back(t);
        m_v.push_back(a);
        index++;
      }
    }
  }
  m_witnessCfg.reset();
}
