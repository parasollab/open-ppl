#include "BoundingSphere.h"
#include "MPProblem.h"


BoundingSphere::BoundingSphere(int _iDofs, int _iPosDofs ) 
  { 
  pos_dofs=_iPosDofs;
  dofs=_iDofs;
  boundingSphere.clear();
  for (int i = 0; i < dofs; i++) {
    if (i < pos_dofs)
      parType.push_back(TRANSLATIONAL);
    else
      parType.push_back(REVOLUTE);
  }
  for (int i = pos_dofs; i < dofs; i++) {    
    m_jointLimits.push_back(pair<double,double>(0.0,1.0));
  }
}

BoundingSphere::
BoundingSphere(XMLNodeReader& _inNode,MPProblem* _inPproblem): Boundary(_inNode, _inPproblem) { 

  pos_dofs = Cfg::PosDOF();
  dofs = Cfg::DOF();
  _inNode.verifyName(string("boundary"));
  boundingSphere.clear();
  for (int i = 0; i < dofs; i++) {
    
    if (i < pos_dofs)
      parType.push_back(TRANSLATIONAL);
    else
      parType.push_back(REVOLUTE);
  }
  for (int i = pos_dofs; i < dofs; i++) {    
    m_jointLimits.push_back(pair<double,double>(0.0,1.0));
  }

  XMLNodeReader::childiterator citr;
  for(citr = _inNode.children_begin(); citr!= _inNode.children_end(); ++citr) {
    if (citr->getName() == "parameter") {
     
      int par_id = citr->numberXMLParameter("id",true,0,0,MAX_INT,"id");
      string par_label = citr->stringXMLParameter("Label",true,"","Label");
          //@todo par_label is not used in bSphere parameters, may want to use it
      double val = citr->numberXMLParameter("value",true,0.0,-1.0*MAX_DBL,MAX_DBL,"value");
    
      
      if(m_debug) cout<<"BoundingSphere:: setting parameter par_id="<<par_id<<" value=" <<val;
      
      SetParameter(par_id,val);
      string type = citr->stringXMLParameter("type",true,"","type");
      if (type == "translational")
        parType[par_id] = TRANSLATIONAL;
      else
        parType[par_id] = REVOLUTE;
    } else {
      citr->warnUnknownNode();
    }
  }
  double translational_scale = _inNode.numberXMLParameter(string("translational_scale"),true,double(0),double(-1*MAX_INT),double(MAX_INT),string("translational_scale"));

  TranslationalScale(translational_scale);
}

BoundingSphere::
BoundingSphere(const BoundingSphere &from_bSphere)  {
  dofs = from_bSphere.GetDOFs();
  pos_dofs = from_bSphere.GetPosDOFs();
  boundingSphere.clear();
  SetMPProblem(from_bSphere.GetMPProblem());
  for (int i = 0; i < dofs; i++) {
    if(i<=pos_dofs)
      boundingSphere.push_back(from_bSphere.GetParameter(i));
    parType.push_back(from_bSphere.GetType(i));
  }
  for (int i = pos_dofs; i < dofs; i++) {
    m_jointLimits.push_back(from_bSphere.m_jointLimits[i-pos_dofs]);
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
operator==(const Boundary& bb) const
{
  cerr<<"BoundingSphere::== not implemented"<<endl;
  exit(-1);
}

Point3d
BoundingSphere::GetRandomPoint(){
Point3d p;
double radius = boundingSphere[pos_dofs];
for(int i=0;i<pos_dofs;i++){
  p[i] = (boundingSphere[i] - radius) +
         (2 * radius)*DRand();
}
return p;

}

void 
BoundingSphere::
SetParameter(int par, double p_value) {
  boundingSphere[par] =p_value;
}

std::vector<BoundingSphere > 
BoundingSphere::
Partition(int par, double p_point, double epsilon) {
  std::vector<BoundingSphere > result;
  cerr<<"BoundingSphere:: Partition not implemented as bsphere may not have partitions.";
  exit(-1);
  return result;
}

BoundingSphere
BoundingSphere::
GetCombination(BoundingSphere &o_bounding_sphere) {
  cout<<"BoundingSphere:: GetCombinationnot not implemented. none call this function.";
  exit(-1);
  return o_bounding_sphere;
}

int
BoundingSphere::
FindSplitParameter(BoundingSphere &o_bounding_sphere) {
  cout<<"BoundingSphere:: FindSplitParameter not implemented. don't know the use"<<endl;
  exit(-1);
  return 0; 

}
const double
BoundingSphere::
GetParameter(int _par) const{
 return boundingSphere[_par];
}

double
BoundingSphere::
GetClearance(Vector3D _point3d) const {
  double minClearance = -1;
  double clearance = 0;
  double radius = boundingSphere[pos_dofs];
  for (int i = 0; i < pos_dofs; ++i) {
    clearance = min((_point3d[i] - (boundingSphere[i] -radius) ),
                      ((boundingSphere[i] + radius) - _point3d[i]));
    if (clearance < minClearance || i == 0)
     minClearance = clearance;
  }
  return minClearance;
}

BoundingSphere::parameter_type
BoundingSphere::
GetType(int _par) const {
  return parType[_par];
}


bool
BoundingSphere::
IfWrap(int par) {
  if(parType[par] == REVOLUTE) { // orientation angle
    return true;   
  //may need to consider other cases of second < first ?
  }
  return false;
}

bool BoundingSphere::InBoundary(const Cfg& _cfg){
 vector <double> m_v= _cfg.GetData();
 Environment *_env=GetMPProblem()->GetEnvironment();
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
Print(std::ostream& _os, char range_sep, char par_sep) const {
  std::vector< double >::const_iterator itrb;
  _os << "[ " ;
  for (itrb = boundingSphere.begin(); itrb < boundingSphere.end(); ++itrb) {
    if (itrb+1 != boundingSphere.end())
      _os << *itrb << par_sep << " ";
    else
      _os << *itrb << "";
  }
  _os << " ]";
}

void
BoundingSphere::
TranslationalScale(double scale_factor) {
  if(scale_factor != 1.0) {    	
    SetParameter(pos_dofs, scale_factor * boundingSphere[pos_dofs]);
    
  }
}

bool 
BoundingSphere::
IfEnoughRoom(int par, double room) {
  int radius =  boundingSphere[pos_dofs]; 
  if ( radius > 0) { // regularly
    if ((2* radius) >= room)
      return true;
    else
      return false;
  } 
  else 
    return false;
}

bool
BoundingSphere::
IfSatisfiesConstraints(Vector3D point3d) const {
  double radius =  boundingSphere[pos_dofs];
  for (int i = 0; i < pos_dofs; i++) {
    if ( point3d[i] < boundingSphere[i] - radius || point3d[i] > boundingSphere[i] + radius)
      return false;
  }
  return true;
}

bool
BoundingSphere::
IfSatisfiesConstraints(vector<double> point) const {
  cout<<"IfSatisfiesConstraints not implemented\n";
  double radius =  boundingSphere[pos_dofs];
  for (size_t i = 0; i < point.size() && i < boundingSphere.size(); i++) {
    if (parType[i] == REVOLUTE) {
      if (point[i] < 0 || point[i] > 1) {
	cout << "Invalid range on REVOLUTE dof." << endl;
	exit(-1);
	return false;
      }
   }
   else { //no wrap around in other kinds of parameters
     if (point[i] < boundingSphere[i] - radius || point[i] > boundingSphere[i] + radius)
       return false;
      // may still need to consider physical constraints not explicitely set by the bounding Sphere
    }
  }
  return true;
}
