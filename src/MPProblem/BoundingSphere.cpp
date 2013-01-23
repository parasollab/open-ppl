#include "BoundingSphere.h"
#include "Cfg/Cfg.h"

BoundingSphere::BoundingSphere(int _DOFs, int _posDOFs ) 
{ 
  m_posDOFs=_posDOFs;
  m_DOFs=_DOFs;
  m_boundingSphere.clear();
  for (int i = 0; i < m_DOFs; i++) {
    if (i < m_posDOFs)
      m_parType.push_back(TRANSLATIONAL);
    else
      m_parType.push_back(REVOLUTE);
  }
  for (int i = m_posDOFs; i < m_DOFs; i++) {    
    m_jointLimits.push_back(pair<double,double>(-1.0,1.0));
  }
}

BoundingSphere::
BoundingSphere(XMLNodeReader& _node): Boundary(_node) { 

  m_posDOFs = Cfg::PosDOF();
  m_DOFs = Cfg::DOF();
  _node.verifyName(string("boundary"));
  m_boundingSphere.clear();
  for (int i = 0; i < m_DOFs; i++) {

    if (i < m_posDOFs)
      m_parType.push_back(TRANSLATIONAL);
    else
      m_parType.push_back(REVOLUTE);
  }
  for (int i = m_posDOFs; i < m_DOFs; i++) {    
    m_jointLimits.push_back(pair<double,double>(-1.0,1.0));
  }

  XMLNodeReader::childiterator citr;
  for(citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
    if (citr->getName() == "parameter") {

      int par_id = citr->numberXMLParameter("id",true,0,0,MAX_INT,"id");
      string par_label = citr->stringXMLParameter("label",true,"","Label");
      //@todo par_label is not used in bSphere parameters, may want to use it
      double val = citr->numberXMLParameter("value",true,0.0,-1.0*MAX_DBL,MAX_DBL,"value");


      //if(m_debug) cout<<"BoundingSphere:: setting parameter par_id="<<par_id<<" value=" <<val;

      SetParameter(par_id,val);
      string type = citr->stringXMLParameter("type",true,"","type");
      if (type == "translational")
        m_parType[par_id] = TRANSLATIONAL;
      else
        m_parType[par_id] = REVOLUTE;
    } else {
      citr->warnUnknownNode();
    }
  }
  double translational_scale = _node.numberXMLParameter(string("translational_scale"),true,double(0),double(-1*MAX_INT),double(MAX_INT),string("translational_scale"));

  TranslationalScale(translational_scale);
}

BoundingSphere::
BoundingSphere(const BoundingSphere& _bSphere)  {
  m_DOFs = _bSphere.GetDOFs();
  m_posDOFs = _bSphere.GetPosDOFs();
  m_boundingSphere.clear();
  for (int i = 0; i < m_DOFs; i++) {
    if(i<=m_posDOFs)
      m_boundingSphere.push_back(_bSphere.GetParameter(i));
    m_parType.push_back(_bSphere.GetType(i));
  }
  for (int i = m_posDOFs; i < m_DOFs; i++) {
    m_jointLimits.push_back(_bSphere.m_jointLimits[i-m_posDOFs]);
  }
}

BoundingSphere::
BoundingSphere() { }

BoundingSphere::
~BoundingSphere() {
  //cout << " ~BoundingSphere(). TODO ALL " << endl;
}

bool
BoundingSphere::
operator==(const Boundary& _bb) const
{
  cerr<<"BoundingSphere::== not implemented"<<endl;
  exit(-1);
}

Point3d
BoundingSphere::GetRandomPoint(){
  Point3d p;
  double radius = m_boundingSphere[m_posDOFs];
  for(int i=0;i<m_posDOFs;i++){
    p[i] = (m_boundingSphere[i] - radius) +
      (2 * radius)*DRand();
  }
  return p;

}

void 
BoundingSphere::
SetParameter(int _par, double _value) {
  m_boundingSphere[_par] =_value;
}

std::vector<BoundingSphere > 
BoundingSphere::
Partition(int _par, double _point, double _epsilon) {
  std::vector<BoundingSphere > result;
  cerr<<"BoundingSphere:: Partition not implemented as bsphere may not have partitions.";
  exit(-1);
  return result;
}

BoundingSphere
BoundingSphere::
GetCombination(BoundingSphere& _boundingSphere) {
  cout<<"BoundingSphere:: GetCombinationnot not implemented. none call this function.";
  exit(-1);
  return _boundingSphere;
}

int
BoundingSphere::
FindSplitParameter(BoundingSphere& _boundingSphere) {
  cout<<"BoundingSphere:: FindSplitParameter not implemented. don't know the use"<<endl;
  exit(-1);
  return 0; 

}
const double
BoundingSphere::
GetParameter(int _par) const{
  return m_boundingSphere[_par];
}

double
BoundingSphere::
GetClearance(Vector3D _point3d) const {
  double minClearance = -1;
  double clearance = 0;
  double radius = m_boundingSphere[m_posDOFs];
  for (int i = 0; i < m_posDOFs; ++i) {
    clearance = min((_point3d[i] - (m_boundingSphere[i] -radius) ),
        ((m_boundingSphere[i] + radius) - _point3d[i]));
    if (clearance < minClearance || i == 0)
      minClearance = clearance;
  }
  return minClearance;
}

bool
BoundingSphere::
IfWrap(int _par) {
  if(m_parType[_par] == REVOLUTE) { // orientation angle
    return true;   
    //may need to consider other cases of second < first ?
  }
  return false;
}

bool BoundingSphere::InBoundary(const Cfg& _cfg, Environment* _env){
  vector <double> m_v= _cfg.GetData();
  if(!IfSatisfiesConstraints(m_v)) 
    return false;

  // @todo: if there are multiple robots, this needs to be changed.
  shared_ptr<MultiBody> robot = _env->GetMultiBody(_env->GetRobotIndex());  
  if (GetClearance(_cfg.GetRobotCenterPosition()) < robot->GetBoundingSphereRadius()) { //faster, loose check
    // Robot is close to wall, have a strict check.
    _cfg.ConfigEnvironment(_env); // Config the Environment(robot indeed).

    for(int m=0; m<robot->GetFreeBodyCount(); ++m) {
      Transformation &worldTransformation = robot->GetFreeBody(m)->GetWorldTransformation();

      GMSPolyhedron &bb_poly = robot->GetFreeBody(m)->GetBoundingBoxPolyhedron();
      bool bSphere_check = true;
      for(vector<Vector3D>::const_iterator V = bb_poly.m_vertexList.begin(); V != bb_poly.m_vertexList.end(); ++V)
        if(!IfSatisfiesConstraints(worldTransformation * (*V))) {
          bSphere_check = false;
          break;
        }
      if(bSphere_check) 
        continue;

      GMSPolyhedron &poly = robot->GetFreeBody(m)->GetPolyhedron();
      for(vector<Vector3D>::const_iterator V = poly.m_vertexList.begin(); V != poly.m_vertexList.end(); ++V)
        if(!IfSatisfiesConstraints(worldTransformation * (*V)))
          return false;      
    }
  }
  return true; 
}

void
BoundingSphere::
Print(std::ostream& _os, char _rangeSep, char _parSep) const {
  std::vector< double >::const_iterator itrb;
  _os << "[ " ;
  for (itrb = m_boundingSphere.begin(); itrb < m_boundingSphere.end(); ++itrb) {
    if (itrb+1 != m_boundingSphere.end())
      _os << *itrb << _parSep << " ";
    else
      _os << *itrb << "";
  }
  _os << " ]";
}

void
BoundingSphere::
TranslationalScale(double _scaleFactor) {
  if(_scaleFactor != 1.0) {    	
    SetParameter(m_posDOFs, _scaleFactor * m_boundingSphere[m_posDOFs]);

  }
}

bool 
BoundingSphere::
IfEnoughRoom(int _par, double _room) {
  int radius =  m_boundingSphere[m_posDOFs]; 
  if ( radius > 0) { // regularly
    if ((2* radius) >= _room)
      return true;
    else
      return false;
  } 
  else 
    return false;
}

bool
BoundingSphere::
IfSatisfiesConstraints(Vector3D _point3d) const {
  double radius =  m_boundingSphere[m_posDOFs];
  for (int i = 0; i < m_posDOFs; i++) {
    if ( _point3d[i] < m_boundingSphere[i] - radius || _point3d[i] > m_boundingSphere[i] + radius)
      return false;
  }
  return true;
}

bool
BoundingSphere::
IfSatisfiesConstraints(vector<double> _point) const {
  cout<<"IfSatisfiesConstraints not implemented\n";
  double radius =  m_boundingSphere[m_posDOFs];
  for (size_t i = 0; i < _point.size() && i < m_boundingSphere.size(); i++) {
    if (m_parType[i] == REVOLUTE) {
      if (_point[i] < -1.0 || _point[i] > 1.0) {
        cout << "Invalid range on REVOLUTE dof." << endl;
        exit(-1);
        return false;
      }
    }
    else { //no wrap around in other kinds of parameters
      if (_point[i] < m_boundingSphere[i] - radius || _point[i] > m_boundingSphere[i] + radius)
        return false;
      // may still need to consider physical constraints not explicitely set by the bounding Sphere
    }
  }
  return true;
}
