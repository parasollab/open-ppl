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
#include "MPProblem/Geometry/MultiBody.h"
#include "MPProblem/Environment.h"

CfgSurface::CfgSurface(){
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(0);

  if (m_dof >= 3){
    SetPos(Point2d(-999,-999));
    SetHeight(-999.0);
    SetSurfaceID(-999); //default to base surface
  }
}

CfgSurface::CfgSurface(double _x, double _y, double _h, int _sid) {
  m_v.clear();
  m_v.push_back(_x);
  m_v.push_back(_h);
  m_v.push_back(_y);

  SetPos(Point2d(_x,_y));
  SetHeight(_h);
  SetSurfaceID(_sid);

}

CfgSurface::CfgSurface(const Vector3d& _v) {
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);

  if (m_dof >= 3){
  UpdatePrivateVariables();
  SetSurfaceID(-999); //default to base surface
  }
}

CfgSurface::CfgSurface(const Cfg& _c) {
  vector<double> _v;
  _v = _c.GetData();
  if(_v.size() < m_dof) {
    cerr << "\n\nERROR in CfgSurface::CfgSurface(Cfg&), ";
    cerr << "size of vector is less than " << m_dof << endl;
    exit(-1);
  }
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);

  if (m_dof >= 3){
  UpdatePrivateVariables();
  SetSurfaceID( ((CfgSurface&)_c).GetSurfaceID());
  }
}

CfgSurface::CfgSurface(const Point2d _p, double _h, int _sid){
  m_v.clear();
  m_v.push_back(_p[0]);
  m_v.push_back(_h);
  m_v.push_back(_p[1]);

  SetPos(_p);
  SetHeight(_h);
  SetSurfaceID(_sid);
}

CfgSurface::~CfgSurface() { }

//Write configuration to output stream
void CfgSurface::Write(ostream& _os) const{
  for (size_t i=0; i<m_dof;i++){
    _os<<setw(4)<<m_v[i];
    if (i < m_dof-1)
      _os<<" "; //place a space after each dof except the last one
  }
}

//Read configuration from input stream
void CfgSurface::Read(istream& _is){
  double x, y, height;
  _is>>x>>height>>y;
  if (_is.fail()){
    cerr << "\n\nError in CfgSurface:: Failed to read configuration from stream" << endl;
  }
  m_v.clear();
  m_v.push_back(x);
  m_v.push_back(height);
  m_v.push_back(y);

  UpdatePrivateVariables();
  //HMM, missing surface id here (should this become another DOF?)
}

void CfgSurface::GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundary());
}

void CfgSurface::GetRandomCfg(Environment* _env,shared_ptr<Boundary> _bb) {
  Cfg::GetRandomCfg(_env,_bb);
}

CfgSurface& CfgSurface::operator=(const CfgSurface& _c){
  if(_c.DOF() != m_dof) {
    cerr << "\n\nERROR in CfgSurface::operator=(CfgSurface&), ";
    cerr << "DOF of Cfgs are not equal " << _c.DOF() << "\t!=\t" << m_dof << endl; 
    exit(-1);
  }
  m_v.clear();
  m_v = _c.GetData();
  SetPos( _c.GetPos() );
  SetHeight( _c.GetHeight() );
  SetSurfaceID( _c.GetSurfaceID() );
#ifndef _PARALLEL
  m_labelMap = _c.m_labelMap;
  m_statMap = _c.m_statMap;
#endif
  return *this;
}

void CfgSurface::add(const Cfg& _c1, const Cfg& _c2) {
  Cfg::add(_c1, _c2);
  UpdatePrivateVariables();
  SetSurfaceID( ((CfgSurface&) _c1).GetSurfaceID() );
}

void CfgSurface::subtract(const Cfg& _c1, const Cfg& _c2) {
  Cfg::subtract(_c1, _c2);
  UpdatePrivateVariables();
  SetSurfaceID( ((CfgSurface&) _c1).GetSurfaceID() );
}

void CfgSurface::negative(const Cfg& _c) {
  Cfg::negative(_c);
  UpdatePrivateVariables();
  SetSurfaceID( ((CfgSurface&) _c).GetSurfaceID() );
}

void CfgSurface::multiply(const Cfg& _c, double _s, bool _norm) {
  Cfg::multiply(_c, _s, _norm);
  UpdatePrivateVariables();
  SetSurfaceID( ((CfgSurface&) _c).GetSurfaceID() );
}

void CfgSurface::divide(const Cfg& _c, double _s) {
  Cfg::divide(_c, _s);
  UpdatePrivateVariables();
  SetSurfaceID( ((CfgSurface&) _c).GetSurfaceID() );
}

void CfgSurface::WeightedSum(const Cfg& _first, const Cfg& _second, double _weight) {
  Cfg::WeightedSum(_first, _second, _weight);
  UpdatePrivateVariables();
  SetSurfaceID( ((CfgSurface&) _first).GetSurfaceID() );
}

// Set a single parameter in the configuration (i.e., x,y,z,roll...)
// param = the parameter number to set
// value = the value to set the parameter as
int CfgSurface::SetSingleParam(size_t _param, double _value, bool _norm) {    
  if(Cfg::SetSingleParam(_param, _value, _norm)){
    if(_param<3){
      if(_param==0) m_p[_param] = _value;
      else if(_param==1) m_h = _value;
      else //param is 2
        m_p[1] = _value;
    }
    return 1;
  } 

  return 0;

}

// Increment a single parameter in the configuration (i.e., x,y,z,roll...)
// param = the parameter number to set
// value = the value to increment the parameter by
int CfgSurface::IncSingleParam(size_t _param, double _value) {    
  if(Cfg::IncSingleParam(_param, _value)){
    if(_param<3){
      if(_param==0)
        m_p[_param] += _value;
      else if(_param==1)
        m_h += _value;
      else //param is 2
        m_p[1] += _value;
    }
    return 1;
  } 
  return 0;
}

void CfgSurface::Increment(const Cfg& _increment) {
  Cfg::Increment(_increment);
  UpdatePrivateVariables();
}

void CfgSurface::IncrementTowardsGoal(const Cfg &goal, const Cfg &increment) {
  Cfg::IncrementTowardsGoal(goal, increment);
  UpdatePrivateVariables();
}

void CfgSurface::FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks, double _positionRes, double _orientationRes){
  Cfg::FindIncrement(_start, _goal, _nTicks, _positionRes, _orientationRes);
}

void CfgSurface::FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks) {
  Cfg::FindIncrement(_start, _goal, _nTicks);
  UpdatePrivateVariables();
}

Vector3D CfgSurface::GetRobotCenterPosition() const {
  return Vector3D(m_v[0], m_v[1], m_v[2]);
}

const string CfgSurface::GetName() const {
  return "CfgSurface";
}

bool CfgSurface::ConfigEnvironment(Environment* _env) const {
  shared_ptr<MultiBody> mb = _env->GetMultiBody(_env->GetRobotIndex());

  // configure the robot according to current Cfg: joint parameters
  // (and base locations/orientations for free flying robots.)
  Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 
        0, 
        0, 
        0),
      Vector3D( m_v[0],m_v[1],m_v[2] ));
  // update link i
  mb->GetFreeBody(0)->Configure(T1);

  return true;
}

Cfg* 
CfgSurface::CreateNewCfg() const{
  Cfg* tmp = new CfgSurface();
  ((CfgSurface*)tmp)->SetPos( this->GetPos() );
  ((CfgSurface*)tmp)->SetHeight( this->GetHeight() );
  ((CfgSurface*)tmp)->SetSurfaceID( this->GetSurfaceID() );
  return tmp;
}

Vector3D
CfgSurface::GetRobotCenterofMass(Environment* _env) const {
  ConfigEnvironment(_env);

  Vector3D com(0,0,0);
  GMSPolyhedron poly =
    _env->GetMultiBody(_env->GetRobotIndex())->GetFreeBody(0)->GetWorldPolyhedron();
  for(vector<Vector3D>::const_iterator vit = poly.m_vertexList.begin(); vit
      != poly.m_vertexList.end(); ++vit)
    com = com + (*vit);
  com = com / poly.m_vertexList.size();
  return com;
}

void CfgSurface::GetRandomCfgCenterOfMass(Environment *_env, shared_ptr<Boundary> _bb) {

  if( m_surfaceID < -1 ) { // need to set appropriate surface id
    int rindex = _env->GetRandomNavigableSurfaceIndex();
    m_surfaceID = rindex;
  }

  //this implementation will be wrong for now 
  m_v.clear();
  if( m_surfaceID == -1 ) {
    Point3d rpt = _bb->GetRandomPoint();
    for(size_t i=0; i<m_dof; ++i)
      if( i== 1 )
        m_v.push_back(0.0);
      else {
        m_v.push_back(rpt[i]);
      }
  }
  else { //surface id points to something valid
    //////////////////////////////////////////////////////////////////////////////
    shared_ptr<MultiBody> surface_body = _env->GetNavigableSurface(m_surfaceID);
    shared_ptr<FixedBody> fb = surface_body->GetFixedBody(0);
    GMSPolyhedron & polyhedron = fb->GetWorldPolyhedron();
    Point3d surfPt3d = polyhedron.GetRandPtOnSurface();
    for(size_t i=0; i<m_dof; ++i)
      m_v.push_back( surfPt3d[i] );
    //////////////////////////////////////////////////////////////////////////////
  }

  UpdatePrivateVariables();
}

void CfgSurface::GetRandomCfgCenterOfMass(Environment *_env) {
  GetRandomCfgCenterOfMass(_env, _env->GetBoundary());
}

void CfgSurface::UpdatePrivateVariables(){
  SetPos(Point2d(m_v[0], m_v[2]));
  SetHeight( m_v[1] );
}
