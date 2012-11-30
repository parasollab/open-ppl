#include "BoundingBox.h"
#include "MPProblem/Environment.h"
#include "Cfg/Cfg.h"

BoundingBox::BoundingBox(int _DOFs, int _posDOFs ) { 
  m_posDOFs = _posDOFs;
  m_DOFs=_DOFs ;
  m_boundingBox.clear();
  for (int i = 0; i < m_DOFs; i++) {
    m_boundingBox.push_back(pair<double,double>(0.0,1.0));
    if (i < m_posDOFs)
      m_parType.push_back(TRANSLATIONAL);
    else
      m_parType.push_back(REVOLUTE);
  }
  for (int i = m_posDOFs; i < m_DOFs; i++) {    
    m_jointLimits.push_back(pair<double,double>(0.0,1.0));
  }
}

BoundingBox::BoundingBox(XMLNodeReader& _node) { 

  m_posDOFs = Cfg::PosDOF();
  m_DOFs = Cfg::DOF();
  _node.verifyName(string("boundary"));
  m_boundingBox.clear();
  for (int i = 0; i < m_DOFs; i++) {    
    m_jointLimits.push_back(pair<double,double>(0.0,1.0));
    m_boundingBox.push_back(pair<double,double>(0.0,1.0));
    m_parType.push_back(REVOLUTE);
  }

  XMLNodeReader::childiterator citr;
  for(citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
    if (citr->getName() == "parameter") {

      int par_id = citr->numberXMLParameter("id",true,0,0,MAX_INT,"id");
      string par_label = citr->stringXMLParameter("label",true,"","Label");
      //@todo par_label is not used in bbox parameters, may want to use it
      double par_min = citr->numberXMLParameter("min",true,0.0,-1.0*MAX_DBL,MAX_DBL,"min");
      double par_max = citr->numberXMLParameter("max",true,0.0,-1.0*MAX_DBL,MAX_DBL,"max");

      //if(m_debug) cout<<"BoundingBox:: setting parameter par_id="<<par_id<<" par_min=" <<par_min<<" par_max="<<par_max;

      SetParameter(par_id,par_min,par_max);
      string type = citr->stringXMLParameter("type",true,"","type");
      if (type == "translational")
        m_parType[par_id] = TRANSLATIONAL;
      else
        m_parType[par_id] = REVOLUTE;
    } else {
      citr->warnUnknownNode();
    }
  }
  double translational_scale =
    _node.numberXMLParameter(string("translational_scale"),true,0.0,-MAX_DBL,MAX_DBL,string("translational_scale"));

  TranslationalScale(translational_scale);
}

BoundingBox::
BoundingBox(const BoundingBox& _bbox)  {
  m_DOFs = _bbox.GetDOFs();
  m_posDOFs = _bbox.GetPosDOFs();
  m_boundingBox.clear();
  for (int i = 0; i < m_DOFs; i++) {
    m_boundingBox.push_back(_bbox.GetRange(i));
    m_parType.push_back(_bbox.GetType(i));
  }
  m_jointLimits.clear();
  m_jointLimits.reserve((int)(m_DOFs-m_posDOFs));
  for (int i = m_posDOFs; i < m_DOFs; i++) {
    m_jointLimits.push_back(_bbox.m_jointLimits[i-m_posDOFs]);
  }
}

BoundingBox::~BoundingBox() {
}

bool
BoundingBox::
operator==(const Boundary& _b) const
{
  const BoundingBox* bbox = dynamic_cast<const BoundingBox*>(&_b);
  return (m_boundingBox == bbox->m_boundingBox) &&
    (m_parType == bbox->m_parType) &&
    (m_posDOFs == bbox->m_posDOFs) &&
    (m_DOFs == bbox->m_DOFs);
}

double 
BoundingBox::
GetClearance(Vector3D _point3d) const {
  double minClearance = -1;
  double clearance = 0;
  for (int i = 0; i < m_posDOFs; ++i) {
    clearance = min((_point3d[i] - m_boundingBox[i].first ), 
        (m_boundingBox[i].second - _point3d[i]));
    if (clearance < minClearance || i == 0)
      minClearance = clearance;
  }
  return minClearance;
}

void
BoundingBox::
Clear(){ m_parType.clear(); }

Point3d
BoundingBox::GetRandomPoint(){
  Point3d p;
  for(int i=0;i<min(3, m_posDOFs);i++){
    p[i] = m_boundingBox[i].first +
      (m_boundingBox[i].second - m_boundingBox[i].first)*DRand();
  }
  return p;
}

void 
BoundingBox::
SetParameter(int _par, double _first, double _second) {
  m_boundingBox[_par].first = _first;
  m_boundingBox[_par].second = _second;
}

std::vector<BoundingBox > 
BoundingBox::
Partition(int _par, double _point, double _epsilon) {
  std::vector<BoundingBox > result;
  BoundingBox leftBB(*this);
  leftBB.SetParameter(_par, (leftBB.GetRange(_par)).first, _point+_epsilon);
  result.push_back(leftBB);
  BoundingBox rightBB(*this);
  rightBB.SetParameter(_par, _point-_epsilon, rightBB.GetRange(_par).second);
  result.push_back(rightBB);
  return result;
}

BoundingBox
BoundingBox::
GetCombination(BoundingBox& _boundingBox) {
  BoundingBox combination(*this);
  for (int par=0; par < _boundingBox.GetDOFs() && par < combination.GetDOFs(); par++) {
    combination.SetParameter(par,
        min(combination.m_boundingBox[par].first,
          _boundingBox.m_boundingBox[par].first),
        max(combination.m_boundingBox[par].second,
          _boundingBox.m_boundingBox[par].second));
  }
  return combination;
}

int
BoundingBox::
FindSplitParameter(BoundingBox& _boundingBox) {
  int split_par = -1;
  for (size_t par=0; par < m_boundingBox.size(); par++) {
    if (m_boundingBox[par].first != _boundingBox.m_boundingBox[par].first ||
        m_boundingBox[par].second != _boundingBox.m_boundingBox[par].second) {
      if (split_par == -1) { // check to see that only one parameter varies
        split_par = par;
      } else { // a split par was previously found, not right.
        split_par = -1;
        break;
      }
    }
  }
  return split_par;
}

bool
BoundingBox::
IfWrap(int _par) {
  if (m_parType[_par] == REVOLUTE) { // orientation angle
    if (m_boundingBox[_par].first == 0 && m_boundingBox[_par].second == 1)
      return true;
    if (m_boundingBox[_par].first > m_boundingBox[_par].second)
      return true;
    //may need to consider other cases of second < first ?
  }
  return false;
}

bool 
BoundingBox::InBoundary(const Cfg& _cfg, Environment* _env){
  vector <double> m_v= _cfg.GetData();
  if(!IfSatisfiesConstraints(m_v)) 
    return false;

  // @todo: if there are multiple robots, this needs to be changed.
  shared_ptr<MultiBody> robot = _env->GetMultiBody(_env->GetRobotIndex()); 
  if (GetClearance(_cfg.GetRobotCenterPosition()) < robot->GetBoundingSphereRadius()) { //faster, loose check
    // Robot is close to wall, have a strict check.
    _cfg.ConfigEnvironment(_env); // Config the Environment(robot indeed).

    for(int m=0; m<robot->GetFreeBodyCount(); ++m) {
      Transformation &worldTransformation = robot->GetFreeBody(m)->WorldTransformation();
      GMSPolyhedron &bb_poly = robot->GetFreeBody(m)->GetBoundingBoxPolyhedron();

      bool bbox_check = true;
      for(vector<Vector3D>::const_iterator V = bb_poly.m_vertexList.begin(); V !=
          bb_poly.m_vertexList.end(); ++V){
        if(!IfSatisfiesConstraints(worldTransformation * (*V))) {
          bbox_check = false;
          break;
        }
      }
      if(bbox_check) 
        continue;

      GMSPolyhedron &poly = robot->GetFreeBody(m)->GetPolyhedron();
      for(vector<Vector3D>::const_iterator V = poly.m_vertexList.begin(); V !=
          poly.m_vertexList.end(); ++V){
        if(!IfSatisfiesConstraints(worldTransformation * (*V)))
          return false;      
      }
    }
  }
  return true; 
}

void
BoundingBox::
Print(std::ostream& _os, char _sep, char _psep) const {
  std::vector< std::pair<double, double> >::const_iterator itrb;
  _os << "[ " ;
  for (itrb = m_boundingBox.begin(); itrb < m_boundingBox.end(); ++itrb) {
    if (itrb+1 != m_boundingBox.end())
      _os << itrb->first << _sep << itrb->second << " " << _psep << " ";
    else
      _os << itrb->first << _sep << itrb->second << "";
  }
  _os << " ]";
}

/*
   void
   BoundingBox::
   Parse(std::stringstream &i_bbox) {
   vector<double> boundingBox;
   char c;
   bool done = false;

   try {
   do {
   c = i_bbox.peek();
   if ( c==' ' || c=='\n' || c==',' || c=='[' || c==']') {
   if (c==']')
   done = true;
   if (!i_bbox.get())
   done=true;
   } else if ((c>='0' && c<='9') || c=='-' || c=='.') {
   double dtmp;
   if (i_bbox >> dtmp)
   boundingBox.push_back(dtmp);
   else {
   cout << "Character not accepted" << endl;
   throw BadUsage();
   }
   } else {
   cout << "Character not accepted" << endl;
   throw BadUsage();
   }
   } while (!done);
   if (boundingBox.size()%2 != 0) {
   cout << "Expecting even number of ranges" << endl;
   throw BadUsage();
   }
   if (boundingBox.size() < m_posDOFs*2) {
   cout << "Insufficient number of ranges" << endl;
   throw BadUsage();
   } 
   } catch (BadUsage) {
   cout << "BoundingBox:Parse: Err in BoundingBox parameters...expecting even number parameters...no letters...etc" << endl;
   exit(-1);
   }
   SetRange(boundingBox);
   }
   */
void
BoundingBox::
TranslationalScale(double _scaleFactor) {
  double center, new_first, new_second;
  if (_scaleFactor != 1.0) {    	
    for (int i = 0; i < m_posDOFs; i++) {
      center = (m_boundingBox[i].first+m_boundingBox[i].second)/2;
      new_first = (m_boundingBox[i].first-center)*_scaleFactor+center;
      new_second = (m_boundingBox[i].second-center)*_scaleFactor+center;
      SetParameter(i,new_first,new_second);
    }
  }
}

bool 
BoundingBox::
IfEnoughRoom(int _par, double _room) {
  if (m_boundingBox[_par].second > m_boundingBox[_par].first) { // regularly
    if ((m_boundingBox[_par].second - m_boundingBox[_par].first) >= _room)
      return true;
    else
      return false;
  } else { // only in orientation parameters that turn around
    if ((1-m_boundingBox[_par].first - m_boundingBox[_par].second) >= _room)
      return true;
    else
      return false;
  }
}

bool
BoundingBox::
IfSatisfiesConstraints(Vector3D _point3d) const {
  for (int i = 0; i < 3; i++) {
    if ( _point3d[i] < m_boundingBox[i].first || _point3d[i] > m_boundingBox[i].second)
      return false;
  }
  return true;
}

bool
BoundingBox::
IfSatisfiesConstraints(vector<double> _point) const {
  for (size_t i = 0; i < _point.size() && i < m_boundingBox.size(); i++) {
    if (m_parType[i] == REVOLUTE) {
      if (_point[i] < 0 || _point[i] > 1) {
        cout << "Invalid range on REVOLUTE dof." << endl;
        exit(-1);
        return false;
      }
      if (m_boundingBox[i].first > m_boundingBox[i].second) { // wrap around parameter
        if (_point[i] > m_boundingBox[i].second && _point[i] < m_boundingBox[i].first)
          return false;
      }
    } else { //no wrap around in other kinds of parameters
      if (_point[i] < m_boundingBox[i].first || _point[i] > m_boundingBox[i].second)
        return false;
      // may still need to consider physical constraints not explicitely set by the bounding box
    }
  }
  return true;
}

