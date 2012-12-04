/////////////////////////////////////////////////////////////////////
//
//  Cfg_surface.cpp
//
//  General Description
//	A derived class from Cfg. It provides some specific
//	implementation for a 3-dof rigid-body moving in a 3-D
//	work space on surfaces.
/////////////////////////////////////////////////////////////////////

#include "Cfg_surface.h"
#include "MPProblem/Geometry/MultiBody.h"
#include "MPProblem/Environment.h"

Cfg_surface::Cfg_surface(){
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(0);

  setPos(Point2d(-999,-999));
  setHeight(-999.0);
  setSurfaceID(-999); //default to base surface
}


Cfg_surface::Cfg_surface(double _x, double _y, double _h, int _sid) {
  m_v.clear();
  m_v.push_back(_x);
  m_v.push_back(_h);
  m_v.push_back(_y);

  setPos(Point2d(_x,_y));
  setHeight(_h);
  setSurfaceID(_sid);
  
}


Cfg_surface::Cfg_surface(const Vector3d& _v) {
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);
  
  setPos(Point2d(_v[0],_v[2]));
  setHeight(_v[1]);
  setSurfaceID(-999); //default to base surface
}


Cfg_surface::Cfg_surface(const Cfg& _c) {
  vector<double> _v;
  _v = _c.GetData();
  if(_v.size() < m_dof) {
    cout << "\n\nERROR in Cfg_surface::Cfg_surface(Cfg&), ";
    cout << "size of vector is less than " << m_dof << endl;
    exit(-1);
  }
  m_v.clear();
  for(size_t i=0; i<m_dof; i++)
    m_v.push_back(_v[i]);

  setPos(Point2d(m_v[0], m_v[2]));
  setHeight(m_v[1]);
  setSurfaceID( ((Cfg_surface&)_c).getSurfaceID());
}

Cfg_surface::Cfg_surface(const Point2d _p, double _h, int _sid){
  m_v.clear();
  m_v.push_back(_p[0]);
  m_v.push_back(_h);
  m_v.push_back(_p[1]);

  setPos(_p);
  setHeight(_h);
  setSurfaceID(_sid);
}

Cfg_surface::~Cfg_surface() { }

//Write configuration to output stream
void Cfg_surface::Write(ostream& _os) const{
  _os<<setw(4)<<m_v[0]<<" "<<setw(4)<<m_v[1]<<" "<<setw(4)<<m_v[2]<<" "<<0<<" "<<0<<" "<<0<<" ";
}
  
//Read configuration from input stream
void Cfg_surface::Read(istream& _is){
  double x, y, H, theta, tmp;
  _is>>x>>H>>y>>tmp>>tmp>>theta;
  m_v.clear();
  m_v.push_back(x);
  m_v.push_back(H);
  m_v.push_back(y);

  setPos(Point2d(m_v[0], m_v[2]));
  setHeight(H);
  //HMM, missing surface id here
}




void Cfg_surface::GetRandomCfg(double _r, double _rStep) {
  double alpha, beta, z1;
  
  alpha = 2.0*M_PI*DRand();
  beta  = 2.0*M_PI*DRand();
  z1 = _r*sin(beta);
  
  double H;
  H = (2.0*_r)*DRand() - _r; // (2.0*rStep)*DRand() - rStep;
  
  m_v.clear();
  m_v.push_back(z1*cos(alpha));
  m_v.push_back(H);
  m_v.push_back(z1*sin(alpha));

  setPos(Point2d(m_v[0], m_v[2]));
  setHeight( m_v[1] );
}


void Cfg_surface::GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env, _env->GetBoundary());
}

void Cfg_surface::GetRandomCfg(Environment* _env,shared_ptr<Boundary> _bb) {
  Cfg::GetRandomCfg(_env,_bb);
}

Cfg_surface& Cfg_surface::operator=(const Cfg_surface& _c){
  if(_c.DOF() != m_dof) {
    cout << "\n\nERROR in Cfg_surface::operator=(Cfg_surface&), ";
    cout << "DOF of Cfgs are not equal " << _c.DOF() << "\t!=\t" << m_dof << endl; 
    exit(-1);
  }
  m_v.clear();
  m_v = _c.GetData();
  setPos( _c.getPos() );
  setHeight( _c.getHeight() );
  setSurfaceID( _c.getSurfaceID() );
#ifndef _PARALLEL
  m_labelMap = _c.m_labelMap;
  m_statMap = _c.m_statMap;
#endif
  return *this;
}

void Cfg_surface::add(const Cfg& _c1, const Cfg& _c2) {
  Cfg::add(_c1, _c2);
  setPos(Point2d(m_v[0],m_v[2]));
  setHeight(m_v[1]);
  setSurfaceID( ((Cfg_surface&) _c1).getSurfaceID() );
}

void Cfg_surface::subtract(const Cfg& _c1, const Cfg& _c2) {
  Cfg::subtract(_c1, _c2);
  setPos(Point2d(m_v[0],m_v[2]));
  setHeight(m_v[1]);
  setSurfaceID( ((Cfg_surface&) _c1).getSurfaceID() );
}

void Cfg_surface::negative(const Cfg& _c) {
  Cfg::negative(_c);
  setPos(Point2d(m_v[0],m_v[2]));
  setHeight(m_v[1]);
  setSurfaceID( ((Cfg_surface&) _c).getSurfaceID() );
}

void Cfg_surface::multiply(const Cfg& _c, double _s, bool _norm) {
  Cfg::multiply(_c, _s, _norm);
  setPos(Point2d(m_v[0],m_v[2]));
  setHeight(m_v[1]);
  setSurfaceID( ((Cfg_surface&) _c).getSurfaceID() );
}

void Cfg_surface::divide(const Cfg& _c, double _s) {
  Cfg::divide(_c, _s);
  setPos(Point2d(m_v[0],m_v[2]));
  setHeight(m_v[1]);
  setSurfaceID( ((Cfg_surface&) _c).getSurfaceID() );
}

void Cfg_surface::WeightedSum(const Cfg& _first, const Cfg& _second, double _weight) {
  Cfg::WeightedSum(_first, _second, _weight);
  setPos(Point2d(m_v[0],m_v[2]));
  setHeight(m_v[1]);
  setSurfaceID( ((Cfg_surface&) _first).getSurfaceID() );
}

// Set a single parameter in the configuration (i.e., x,y,z,roll...)
// param = the parameter number to set
// value = the value to set the parameter as
int Cfg_surface::SetSingleParam(size_t _param, double _value, bool _norm) {    
  if ((_param>=0) && (_param<m_dof)) {
    Cfg::SetSingleParam(_param, _value, _norm);
    if(_param<3){
      if(_param==0) m_p[_param] += _value;
      else if(_param==1) m_H += _value;
      else if(_param==2) m_p[1] += _value;
    }
    return 1;
  } else {
    return 0;
  } 
}

// Increment a single parameter in the configuration (i.e., x,y,z,roll...)
// param = the parameter number to set
// value = the value to increment the parameter by
int Cfg_surface::IncSingleParam(size_t _param, double _value) {    
  if ((_param>=0) && (_param<m_dof)) {
    Cfg::IncSingleParam(_param, _value);
    if(_param<3){
      if(_param==0) m_p[_param] += _value;
      else if(_param==1) m_H += _value;
      else if(_param==2) m_p[1] += _value;
    }
    return 1;
  } else {
    return 0;
  } 
}

void Cfg_surface::Increment(const Cfg& _increment) {
  Cfg::Increment(_increment);
  setPos(Point2d(m_v[0], m_v[2]));
  setHeight(m_v[1]);
}

void Cfg_surface::IncrementTowardsGoal(const Cfg &goal, const Cfg &increment) {
  Cfg::IncrementTowardsGoal(goal, increment);
  setPos(Point2d(m_v[0], m_v[2]));
  setHeight(m_v[1]);
}
  
void Cfg_surface::FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks, double _positionRes, double _orientationRes){
  Cfg::FindIncrement(_start, _goal, _nTicks, _positionRes, _orientationRes);
}

void Cfg_surface::FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks) {
  Cfg::FindIncrement(_start, _goal, _nTicks);
  setPos(Point2d(m_v[0], m_v[2]));
  setHeight(m_v[1]);
}



Vector3D Cfg_surface::GetRobotCenterPosition() const {
   return Vector3D(m_v[0], m_v[1], m_v[2]);
}


const string Cfg_surface::GetName() const {
  return "Cfg_surface";
}

bool Cfg_surface::ConfigEnvironment(Environment* _env) const {
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
Cfg_surface::CreateNewCfg() const{
  Cfg* tmp = new Cfg_surface();
  ((Cfg_surface*)tmp)->setPos( this->getPos() );
  ((Cfg_surface*)tmp)->setHeight( this->getHeight() );
  ((Cfg_surface*)tmp)->setSurfaceID( this->getSurfaceID() );
  return tmp;
}

void Cfg_surface::GetRandomCfgCenterOfMass(Environment *_env, shared_ptr<Boundary> _bb) {
  
  if( m_SurfaceID < -1 ) { // need to set appropriate surface id
    int rindex = _env->GetRandomNavigableSurfaceIndex();
    m_SurfaceID = rindex;
  }

  //this implementation will be wrong for now 
  m_v.clear();
  if( m_SurfaceID == -1 ) {
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
    shared_ptr<MultiBody> surface_body = _env->GetNavigableSurface(m_SurfaceID);
    shared_ptr<FixedBody> fb = surface_body->GetFixedBody(0);
    GMSPolyhedron & polyhedron = fb->GetWorldPolyhedron();
    Point3d surfPt3d = polyhedron.GetRandPtOnSurface();
   // cout << " surfaceid: " << m_SurfaceID << " computed pt: " << surfPt3d << endl;
    for(size_t i=0; i<m_dof; ++i)
      m_v.push_back( surfPt3d[i] );
    //////////////////////////////////////////////////////////////////////////////
  }
  
  setPos(Point2d(m_v[0], m_v[2]));
  setHeight( m_v[1] );
}

void Cfg_surface::GetRandomCfgCenterOfMass(Environment *_env) {
  GetRandomCfgCenterOfMass(_env, _env->GetBoundary());
}


Vector3D 
Cfg_surface::GetRobotCenterofMass(Environment* _env) const {
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
