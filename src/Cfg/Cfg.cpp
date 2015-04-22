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
vector<size_t> Cfg::m_dof;
vector<size_t> Cfg::m_posdof;
vector<size_t> Cfg::m_numJoints;
vector<vector<Cfg::DofType> > Cfg::m_dofTypes;
vector<vector<Robot> > Cfg::m_robots;

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
Cfg::InitRobots(vector<Robot>& _robots, size_t _index, ostream& _os) {
  m_robots[_index] = _robots;
  _os << "DoF List: " << endl;

  int dof=0;
  size_t posdof = 0;
  size_t numJoints = 0;
  vector<DofType> dofTypes;
  for(vector<Robot>::iterator rit = m_robots[_index].begin(); rit != m_robots[_index].end(); rit++) {

    _os << "\tRobot with base index " << rit->m_bodyIndex;
    _os << " (" << rit->m_body->GetFileName() << "):" << endl;

    if(rit->m_base == Robot::PLANAR) {
      dofTypes.push_back(POS);
      dofTypes.push_back(POS);
      posdof += 2;

      _os << "\t\t" << dof++ << ": X position" << endl;
      _os << "\t\t" << dof++ << ": Y position" << endl;

      if(rit->m_baseMovement == Robot::ROTATIONAL) {
        dofTypes.push_back(ROT);

        _os << "\t\t" << dof++ << ": Rotation about Z" << endl;
      }
    }
    if(rit->m_base == Robot::VOLUMETRIC) {
      dofTypes.push_back(POS);
      dofTypes.push_back(POS);
      dofTypes.push_back(POS);
      posdof += 3;

      _os << "\t\t" << dof++ << ": X position" << endl;
      _os << "\t\t" << dof++ << ": Y position" << endl;
      _os << "\t\t" << dof++ << ": Z position" << endl;
      if(rit->m_baseMovement == Robot::ROTATIONAL) {
        dofTypes.push_back(ROT);
        dofTypes.push_back(ROT);
        dofTypes.push_back(ROT);

        _os << "\t\t" << dof++ << ": Rotation about X" << endl;
        _os << "\t\t" << dof++ << ": Rotation about Y" << endl;
        _os << "\t\t" << dof++ << ": Rotation about Z" << endl;
      }
    }
    for(Robot::JointIT jit = rit->m_joints.begin(); jit != rit->m_joints.end(); jit++) {
      if((*jit)->GetConnectionType() == Connection::REVOLUTE) {
        dofTypes.push_back(JOINT);
        numJoints++;

        _os << "\t\t" << dof++ << ": ";
        _os << "Rotational joint from body " << (*jit)->GetPreviousBodyIndex();
        _os << " (" << (*jit)->GetPreviousBody()->GetFileName() << ")";
        _os << " to body " << (*jit)->GetNextBodyIndex();
        _os << " (" << (*jit)->GetNextBody()->GetFileName() << ")" << endl;
      }
      else if((*jit)->GetConnectionType() == Connection::SPHERICAL) {
        dofTypes.push_back(JOINT);
        dofTypes.push_back(JOINT);
        numJoints+=2;

        _os << "\t\t" << dof++;
        _os << "/" << dof++ << ": ";
        _os << "Spherical joint from body " << (*jit)->GetPreviousBodyIndex();
        _os << " (" << (*jit)->GetPreviousBody()->GetFileName() << ")";
        _os << " to body " << (*jit)->GetNextBodyIndex();
        _os << " (" << (*jit)->GetNextBody()->GetFileName() << ")" << endl;
      }
      else if((*jit)->GetConnectionType() == Connection::NONACTUATED) {
        //skip, do nothing
      }
    }
  }
  m_dof[_index] = dofTypes.size();
  m_numJoints[_index] = numJoints;
  m_posdof[_index] = posdof;
  m_dofTypes[_index] = dofTypes;
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
    if(m_dofTypes[m_robotIndex][i] == POS || m_dofTypes[m_robotIndex][i] == JOINT) {
      if(abs(m_v[i] - _cfg[i]) > abs(min(m_v[i], _cfg[i]))*numeric_limits<double>::epsilon())
        return false;
    }
    else {
      if(abs(DirectedAngularDistance(m_v[i], _cfg[i])) > abs(min(m_v[i], _cfg[i])) * numeric_limits<double>::epsilon())
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
    if(m_dofTypes[m_robotIndex][i] == POS || m_dofTypes[m_robotIndex][i] == JOINT)
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
Cfg::Write(ostream& _os) const{
  //write out robot index, and then dofs
  _os << setw(4) << m_robotIndex << ' ';
  _os << scientific << setprecision(17);
  for(vector<double>::const_iterator i = m_v.begin(); i != m_v.end(); ++i)
    _os << setw(25) << *i << ' ';
  _os.unsetf(ios_base::floatfield);
  if (_os.fail()) {
    cerr << "Cfg::Write error - failed to write to file" << endl;
    exit(1);
  }
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
Cfg::SetData(const vector<double>& _data) {
  if(_data.size() != m_dof[m_robotIndex]) {
    cout << "\n\nERROR in Cfg::SetData, ";
    cout << "DOF of data and Cfg are not equal " << _data.size() << "\t!=\t" << m_dof[m_robotIndex] << endl;
    exit(-1);
  }
  m_v = _data;
  m_witnessCfg.reset();
}

//sets joint angle coordinates to be coordinates from _data + leaves other the same
void
Cfg::SetJointData(const vector<double>& _data) {
  /*
  if(_data.size() != m_dof) {
  cout << "\n\nERROR in Cfg::SetData, ";
  cout << "DOF of data and Cfg are not equal " << _data.size() << "\t!=\t" << m_dof << endl;
    exit(-1);
  }
  */

  
  unsigned int j=0;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == JOINT){
      if(j>=_data.size()){
        cout << "\n\nERROR in Cfg::SetJointData, ";
        cout << "DOF of data:"<<_data.size()<<" not equal to number of joints"<< endl;
        exit(-1);
      }

      m_v[i]=_data[j];
      j++;
    } 
  }
  
  //m_v = _data;
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
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == POS)
      ret.push_back(m_v[i]);
  }
  return ret;
}

vector<double>
Cfg::GetOrientation() const {
  vector<double> ret;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] != POS)
      ret.push_back(m_v[i]);
  }
  return ret;
}

///////////////////////////////////
vector<double>
Cfg::GetNonJoints() const {
  vector<double> ret;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] != JOINT)
      ret.push_back(m_v[i]);
  }
  return ret;
}

vector<double>
Cfg::GetJoints() const {
  vector<double> ret;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == JOINT)
      ret.push_back(m_v[i]);
  }
  return ret;
}

vector<double>
Cfg::GetRotation() const {
  vector<double> ret;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == ROT)
      ret.push_back(m_v[i]);
  }
  return ret;
}

void
Cfg::ResetRigidBodyCoordinates() {
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] != JOINT)
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
    if(m_dofTypes[m_robotIndex][i] == POS)
      result += m_v[i]*m_v[i];
  return sqrt(result);
}

double
Cfg::OrientationMagnitude() const {
  double result = 0.0;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] != POS)
      result += m_v[i]*m_v[i];
  }
  return sqrt(result);
}

Vector3d
Cfg::GetRobotCenterPosition() const {
  double x = 0, y = 0, z = 0;
  int numRobots = m_robots[m_robotIndex].size();
  int index = 0;
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots[m_robotIndex].begin(); rit != m_robots[m_robotIndex].end(); rit++) {
    x += m_v[index];
    y += m_v[index + 1];
    if(rit->m_base == Robot::VOLUMETRIC)
      z += m_v[index + 2];
    index += 2;
    if(rit->m_base == Robot::VOLUMETRIC)
      index += 1;
    index += rit->m_joints.size();
  }

  return Vector3d(x/numRobots, y/numRobots, z/numRobots);
}

Vector3d
Cfg::GetRobotCenterofMass(Environment* _env) const {
  ConfigEnvironment(_env);

  typedef vector<Robot>::iterator RIT;
  Vector3d com(0,0,0);
  int numbodies=0;
  shared_ptr<MultiBody> mb = _env->GetMultiBody(m_robotIndex);
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots[m_robotIndex].begin(); rit != m_robots[m_robotIndex].end(); rit++) {
    GMSPolyhedron poly = mb->GetFreeBody(rit->m_bodyIndex)->GetWorldPolyhedron();
    Vector3d polycom(0,0,0);
    for(vector<Vector3d>::const_iterator  vit = poly.m_vertexList.begin(); vit != poly.m_vertexList.end(); ++vit)
      polycom = polycom + (*vit);
    polycom = polycom / poly.m_vertexList.size();
    com = com + polycom;
    numbodies++;

    for(Robot::JointIT i = rit->m_joints.begin(); i != rit->m_joints.end(); ++i) {
      GMSPolyhedron poly1 = mb->GetFreeBody((*i)->GetNextBodyIndex())->GetWorldPolyhedron();
      Vector3d polycom1(0,0,0);
      for(vector<Vector3d>::const_iterator vit1 = poly1.m_vertexList.begin(); vit1 != poly1.m_vertexList.end(); ++vit1)
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

  // throw error message and some helpful statistics
  ostringstream oss;
  oss << "GetRandomCfg not able to find anything in boundary: "
    << *_bb << ". Robot radius is "
    << _env->GetMultiBody(m_robotIndex)->GetBoundingSphereRadius() << ".";
  throw PMPLException("Boundary to small", WHERE, oss.str());
}

bool
Cfg::ConfigEnvironment(Environment* _env) const {
  shared_ptr<MultiBody> mb = _env->GetMultiBody(m_robotIndex);
  int index = 0;
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots[m_robotIndex].begin(); rit != m_robots[m_robotIndex].end(); rit++) {
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
      Transformation t1(Vector3d(x, y, z), Orientation(EulerAngle(gamma*PI, beta*PI, alpha*PI)));
      // update link i
      mb->GetFreeBody(rit->m_bodyIndex)->Configure(t1);
    }
    typedef Robot::JointMap::iterator MIT;
    for(MIT mit = rit->m_joints.begin(); mit != rit->m_joints.end(); mit++) {
      if((*mit)->GetConnectionType() != Connection::NONACTUATED) {
        size_t second = (*mit)->GetNextBodyIndex();
        mb->GetFreeBody(second)->GetBackwardConnection(0).GetDHparameters().m_theta = m_v[index]*PI;
        index++;
        if((*mit)->GetConnectionType() == Connection::SPHERICAL) {
          mb->GetFreeBody(second)->GetBackwardConnection(0).GetDHparameters().m_alpha = m_v[index]*PI;
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

  for(size_t i = 0; i < m_dof[m_robotIndex]; i++)
    if(m_dofTypes[m_robotIndex][i] == POS)
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
    if(m_dofTypes[m_robotIndex][i] == POS || m_dofTypes[m_robotIndex][i] == JOINT) {
      //If the diff between _goal and c is smaller than _increment
      if(fabs(_goal.m_v[i]-m_v[i]) < fabs(_increment.m_v[i]))
        m_v[i] = _goal.m_v[i];
      else
        m_v[i] += _increment.m_v[i];
    }
  }

  ///For Oirentation
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == ROT) {
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
  *_nTicks = max(1., floor(max(diff.PositionMagnitude()/_positionRes,
        diff.OrientationMagnitude()/_orientationRes) + 0.5));

  this->FindIncrement(_start, _goal, *_nTicks);
}

void
Cfg::FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks) {
  vector<double> incr;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    if(m_dofTypes[m_robotIndex][i] == POS)
      incr.push_back((_goal.m_v[i] - _start.m_v[i])/_nTicks);
    else if(m_dofTypes[m_robotIndex][i] == JOINT) {
      double a = _start.m_v[i];
      double b = _goal.m_v[i];
      // normalize both a and b to [-1, 1)
      //a = Normalize(a);
      //b = Normalize(b);
      if(_nTicks == 0)
        throw PMPLException("Divide by 0", WHERE, "Divide by 0");
      incr.push_back((b-a)/_nTicks);
    }
    else if(m_dofTypes[m_robotIndex][i] == ROT) {
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
    if(m_dofTypes[m_robotIndex][i] == POS)
      v.push_back(_c1.m_v[i]);
    else
      v.push_back(_c2.m_v[i]);
  }
  m_v = v;
  NormalizeOrientation();
  m_witnessCfg.reset();
}

vector<Vector3d>
Cfg::PolyApprox(Environment* _env) const {
  vector<Vector3d> result;
  ConfigEnvironment(_env);
  _env->GetMultiBody(m_robotIndex)->PolygonalApproximation(result);
  return result;
}

//Normalize the orientation to the range [-1, 1)
void
Cfg::NormalizeOrientation(int _index) {
  if(_index == -1) {
    for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
      if(m_dofTypes[m_robotIndex][i] == ROT) {
        m_v[i] = Normalize(m_v[i]);
      }
    }
  }
  else if(m_dofTypes[m_robotIndex][_index] == ROT) {  // orientation index
    m_v[_index] = Normalize(m_v[_index]);
  }
}

//generates random configuration within C-space
void
Cfg::GetRandomCfgImpl(Environment* _env, shared_ptr<Boundary> _bb) {
  m_v.clear();
  size_t index = 0;
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots[m_robotIndex].begin(); rit != m_robots[m_robotIndex].end(); rit++) {
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
      if((*i)->GetConnectionType() == Connection::REVOLUTE) {
        pair<double, double> r = (*i)->GetJointLimits(0);
        double t = DRand()*(r.second-r.first)+r.first;
        m_v.push_back(t);
        index++;
      }
      else if((*i)->GetConnectionType() == Connection::SPHERICAL) {
        pair<double, double> r = (*i)->GetJointLimits(0);
        double t = DRand()*(r.second-r.first)+r.first;
        r = (*i)->GetJointLimits(1);
        double a = DRand()*(r.second-r.first)+r.first;
        m_v.push_back(t);
        m_v.push_back(a);
        index++;
      }
    }
  }
  m_witnessCfg.reset();
}
