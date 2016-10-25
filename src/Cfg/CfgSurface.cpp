/////////////////////////////////////////////////////////////////////
//
//  CfgSurface.cpp
//
//  General Description
//	A derived class from Cfg. It provides some specific
//	implementation for a 3-dof rigid-body moving in a 3-D
//	work space on surfaces.
/////////////////////////////////////////////////////////////////////

#include "CfgSurface.h"

#include "Environment/ActiveMultiBody.h"
#include "Environment/Environment.h"
#include "Environment/FixedBody.h"
#include "Environment/FreeBody.h"
#include "Environment/SurfaceMultiBody.h"

CfgSurface::CfgSurface() :
  m_pt(Point2d(0,0)), m_h(0), m_surfaceID(INVALID_SURFACE) {
    m_v.resize(3,0);
    InitCfgSurface();
  }

CfgSurface::CfgSurface(double _x, double _y, double _h, int _sid) :
  m_pt(Point2d(_x, _y)), m_h(_h), m_surfaceID(_sid) {
    m_v.resize(3);
    InitCfgSurface();
    SetDataFromThis();
  }

CfgSurface::CfgSurface(const Vector3d& _v) :
  m_pt(Point2d(_v[0], _v[2])), m_h(_v[1]), m_surfaceID(INVALID_SURFACE) {
    m_v.resize(3);
    InitCfgSurface();
    SetDataFromThis();
  }

CfgSurface::CfgSurface(const CfgSurface& _c) :
  m_pt(_c.m_pt), m_h(_c.m_h), m_surfaceID(_c.m_surfaceID){
    m_labelMap = _c.m_labelMap;
    m_statMap = _c.m_statMap;
    m_clearanceInfo = _c.m_clearanceInfo;
    m_witnessCfg = _c.m_witnessCfg;
    m_v = _c.m_v;
    InitCfgSurface();
  }

CfgSurface::CfgSurface(const Point2d& _p, double _h, int _sid) :
  m_pt(_p), m_h(_h), m_surfaceID(_sid) {
    m_v.resize(3);
    InitCfgSurface();
    SetDataFromThis();
  }

CfgSurface::CfgSurface(const Cfg& _c) :
  m_pt(Point2d(_c[0], _c[2])), m_h(_c[1]), m_surfaceID(INVALID_SURFACE) {
    m_v.resize(3);
    InitCfgSurface();
    SetDataFromThis();
  }

CfgSurface::~CfgSurface() {}

CfgSurface&
CfgSurface::operator=(const CfgSurface& _cfg) {
  InitCfgSurface();
  if(this != &_cfg){
    m_pt = _cfg.m_pt;
    m_h = _cfg.m_h;
    m_surfaceID = _cfg.m_surfaceID;
    m_labelMap = _cfg.m_labelMap;
    m_statMap = _cfg.m_statMap;
    m_robotIndex = _cfg.m_robotIndex;
    m_clearanceInfo = _cfg.m_clearanceInfo;
    m_witnessCfg = _cfg.m_witnessCfg;
    m_v = _cfg.m_v;
    SetDataFromThis();
  }
  return *this;
}

void
CfgSurface::InitCfgSurface() {
  m_dof.clear();
  m_dof.push_back( size_t(3) );
  m_posdof.push_back( size_t(3) );
  m_numJoints.push_back( size_t(0) );
  vector<DofType> dofTypes;
  dofTypes.push_back( DofType::Positional );
  dofTypes.push_back( DofType::Positional );
  dofTypes.push_back( DofType::Positional );
  m_dofTypes.push_back( dofTypes );
}

bool
CfgSurface::operator==(const CfgSurface& _cfg) const {
  return (m_pt == _cfg.m_pt) && (fabs(m_h-_cfg.m_h) < numeric_limits<double>::epsilon()) &&
  m_robotIndex == _cfg.m_robotIndex;
}

bool
CfgSurface::operator!=(const CfgSurface& _cfg) const {
  return !(*this == _cfg);
}

CfgSurface
CfgSurface::operator+(const CfgSurface& _cfg) const {
  CfgSurface result = *this;
  result += _cfg;
  result.SetDataFromThis();
  return result;
}

CfgSurface&
CfgSurface::operator+=(const CfgSurface& _cfg) {
  m_pt[0] += _cfg.m_pt[0];
  m_pt[1] += _cfg.m_pt[1];
  m_h += _cfg.m_h;
  //m_surfaceID = _cfg.m_surfaceID;
  m_witnessCfg.reset();
  SetDataFromThis();
  return *this;
}

CfgSurface
CfgSurface::operator-(const CfgSurface& _cfg) const {
  CfgSurface result = *this;
  result -= _cfg;
  result.SetDataFromThis();
  return result;
}

CfgSurface&
CfgSurface::operator-=(const CfgSurface& _cfg) {
  m_pt[0] -= _cfg.m_pt[0];
  m_pt[1] -= _cfg.m_pt[1];
  m_h -= _cfg.m_h;
  //m_surfaceID = _cfg.m_surfaceID;
  m_witnessCfg.reset();
  SetDataFromThis();
  return *this;
}

CfgSurface
CfgSurface::operator-() const {
  CfgSurface result = *this;
  result.m_pt[0] = -m_pt[0];
  result.m_pt[1] = -m_pt[1];
  result.m_h = -m_h;
  result.m_witnessCfg.reset();
  result.SetDataFromThis();
  return result;
}

CfgSurface
CfgSurface::operator*(double _d) const {
  CfgSurface result = *this;
  result *= _d;
  result.SetDataFromThis();
  return result;
}

CfgSurface&
CfgSurface::operator*=(double _d) {
  m_pt[0] *= _d;
  m_pt[1] *= _d;
  m_h *= _d;
  m_witnessCfg.reset();
  SetDataFromThis();
  return *this;
}

CfgSurface
CfgSurface::operator/(double _d) const {
  CfgSurface result = *this;
  result /= _d;
  result.SetDataFromThis();
  return result;
}

CfgSurface&
CfgSurface::operator/=(double _d) {
  m_pt[0] /= _d;
  m_pt[1] /= _d;
  m_h /= _d;
  m_witnessCfg.reset();
  SetDataFromThis();
  return *this;
}

double&
CfgSurface::operator[](size_t _dof){
  m_witnessCfg.reset();
  assert(_dof >= 0 && _dof <= m_dof[m_robotIndex]);
  switch(_dof){
    case 0 : return m_pt[0];
    case 1 : return m_h;
    case 2 : return m_pt[1];
    default :
      cerr << "Cfg Surface Invalid access to index " << _dof << ". Exiting." << endl;
      exit(1);
  }
}

const double&
CfgSurface::operator[](size_t _dof) const {
  assert(_dof >= 0 && _dof <= m_dof[m_robotIndex]);
  switch(_dof){
    case 0 : return m_pt[0];
    case 1 : return m_h;
    case 2 : return m_pt[1];
    default :
      cerr << "Cfg Surface Invalid access to index " << _dof << ". Exiting." << endl;
      exit(1);
  }
}

//---------------------------------------------
// Input/Output operators for CfgSurface
//---------------------------------------------

void
CfgSurface::Read(istream& _is){
  m_witnessCfg.reset();
  _is >> m_robotIndex >> m_surfaceID >> m_pt[0] >> m_h >> m_pt[1];
  SetDataFromThis();
}

void
CfgSurface::Write(ostream& _os) const{
  _os << setw(4) << m_robotIndex << " "
      << setw(4) << m_surfaceID << " "
      << setw(4) << m_pt[0] << " "
      << setw(4) << m_h << " "
      << setw(4) << m_pt[1] << " ";

  if (_os.fail()){
    cerr << "CfgSurface::Write error - failed to write to file" << endl;
    exit(1);
  }
}

istream&
operator>>(istream& _is, CfgSurface& _cfg) {
  _cfg.Read(_is);
  return _is;
}


ostream&
operator<<(ostream& _os, const CfgSurface& _cfg){
  _cfg.Write(_os);
  return _os;
}

void
CfgSurface::SetData(const vector<double>& _data) {
  if(_data.size() != m_dof[m_robotIndex]) {
    cout << "\n\nERROR in CfgSurface::SetData, ";
    cout << "DOF of data and Cfg are not equal " << _data.size() << "\t!=\t" << m_dof[m_robotIndex] << endl;
    exit(-1);
  }
  if( m_v.size() != 3 )
    m_v.resize(3);
  m_pt[0] = _data[0];
  m_h = _data[1];
  m_pt[1] = _data[2];

  SetDataFromThis();
  m_witnessCfg.reset();
}

void
CfgSurface::SetDataFromThis() {
  m_v[0] = m_pt[0];
  m_v[1] = m_h;
  m_v[2] = m_pt[1];
}

vector<double>
CfgSurface::GetPosition() const {
  return GetData();
}

double
CfgSurface::Magnitude() const {
  return sqrt(m_pt[0]*m_pt[0] + m_h*m_h + m_pt[1]*m_pt[1]);
}

double
CfgSurface::PositionMagnitude() const {
  return Magnitude();
}

Vector3d
CfgSurface::GetRobotCenterPosition() const {
  return Vector3d(m_pt[0], m_h, m_pt[1]);
}

Vector3d
CfgSurface::GetRobotCenterofMass() const {
  ConfigureRobot();

  Vector3d com(0,0,0);
  GMSPolyhedron poly =
    m_robots[m_robotIndex]->GetFreeBody(0)->GetWorldPolyhedron();
  for(vector<Vector3d>::const_iterator vit = poly.m_vertexList.begin(); vit
      != poly.m_vertexList.end(); ++vit)
    com = com + (*vit);
  com = com / poly.m_vertexList.size();
  return com;
}

void
CfgSurface::
ConfigureRobot() const {
  shared_ptr<ActiveMultiBody> mb = m_robots[m_robotIndex];

  // configure the robot according to current Cfg: joint parameters
  // (and base locations/orientations for free flying robots.)
  Transformation T1 = Transformation(
      Vector3d(m_pt[0], m_h, m_pt[1]),
      Orientation());
  // update link i
  mb->GetFreeBody(0)->Configure(T1);
}

void
CfgSurface::GetResolutionCfg(Environment* _env) {
  double posRes = _env->GetPositionRes();
  m_pt[0] = posRes;
  m_h = posRes;
  m_pt[1] = posRes;
  SetDataFromThis();
  m_witnessCfg.reset();
}

void
CfgSurface::IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment) {
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    //If the diff between _goal and c is smaller than _increment
    if(fabs(((const CfgSurface&)_goal)[i]-operator[](i)) < fabs(((const CfgSurface&)_increment)[i]))
      operator[](i) = ((const CfgSurface&)_goal)[i];
    else
      operator[](i) += ((const CfgSurface&)_increment)[i];
  }
  SetDataFromThis();
  m_witnessCfg.reset();
}

void
CfgSurface::FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks, double _positionRes, double _orientationRes) {
  CfgSurface diff = (const CfgSurface&)_goal - (const CfgSurface&)_start;

  // adding two basically makes this a rough ceiling...
  *_nTicks = floor(((const CfgSurface&)diff).PositionMagnitude()/_positionRes + 0.5);

  this->FindIncrement(_start, _goal, *_nTicks);
}

void
CfgSurface::FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks){
  vector<double> incr;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i) {
    incr.push_back((((const CfgSurface&)_goal)[i] - ((const CfgSurface&)_start)[i])/_nTicks);
  }
  SetData(incr);
}

void
CfgSurface::WeightedSum(const Cfg& _first, const Cfg& _second, double _weight) {
  vector<double> v;
  for(size_t i = 0; i < m_dof[m_robotIndex]; ++i)
    v.push_back(((const CfgSurface&)_first)[i]*(1.-_weight) + ((const CfgSurface&)_second)[i]*_weight);
  SetData(v);
  m_surfaceID = ((const CfgSurface&)_first).GetSurfaceID();
}

void
CfgSurface::GetPositionOrientationFrom2Cfg(const Cfg& _c1, const Cfg& _c2) {
  *this = (const CfgSurface&)_c1;
}

void
CfgSurface::GetRandomCfgImpl(Environment* _env, shared_ptr<Boundary> _bb) {
  if( m_surfaceID<-1 ) {
    m_surfaceID = INVALID_SURFACE;
  }
  if( m_surfaceID == INVALID_SURFACE ) { // need to set appropriate surface id
    int rindex = _env->GetRandomSurfaceIndex();
    m_surfaceID = rindex;
  }

  if( m_surfaceID == BASE_SURFACE ) {
    Point3d rpt = _bb->GetRandomPoint();
    m_pt[0] = rpt[0];
    m_h = 0.0;
    m_pt[1] = rpt[2];
  }
  else { //surface id points to something valid
    //////////////////////////////////////////////////////////////////////////////
    shared_ptr<SurfaceMultiBody> surface_body = _env->GetSurface(m_surfaceID);
    shared_ptr<FixedBody> fb = surface_body->GetFixedBody(0);
    GMSPolyhedron& polyhedron = fb->GetWorldPolyhedron();
    Point3d surfPt3d = polyhedron.GetRandPtOnSurface();
    m_pt[0] = surfPt3d[0];
    m_h = surfPt3d[1];
    m_pt[1] = surfPt3d[2];
    //////////////////////////////////////////////////////////////////////////////
  }
  m_witnessCfg.reset();

  SetDataFromThis();
}

