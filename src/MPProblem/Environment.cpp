#include "Environment.h"
#include <fstream>
#include <iostream>

#include "boost/pointer_cast.hpp"

#include "MPProblem/BoundingBox.h"
#include "MPProblem/BoundingSphere.h"
#include "Cfg/Cfg.h"

#ifdef PMPCfgSurface
#include "Cfg/CfgSurface.h"
#endif

#define ENV_RES_DEFAULT                    0.05

//===================================================================
//  Constructors
//===================================================================


Environment::
Environment():
  m_usableExternalbodyCount(0),
  positionRes(ENV_RES_DEFAULT),
  orientationRes(ENV_RES_DEFAULT),
  positionResFactor(ENV_RES_DEFAULT),
  orientationResFactor(ENV_RES_DEFAULT),
  minmax_BodyAxisRange(0),
  m_filename("")
{}


Environment::
Environment(shared_ptr<Boundary> _bb):
  m_usableExternalbodyCount(0),
  m_boundaries(_bb),
  positionRes(ENV_RES_DEFAULT),
  orientationRes(ENV_RES_DEFAULT),
  positionResFactor(ENV_RES_DEFAULT),
  orientationResFactor(ENV_RES_DEFAULT),
  minmax_BodyAxisRange(0),
  m_filename("")
{}


/**
 * Copy Constructor
 */
Environment::
Environment(const Environment &_env) :
  m_usableExternalbodyCount(_env.m_usableExternalbodyCount),
  positionRes(_env.positionRes),
  orientationRes(_env.orientationRes),
  positionResFactor(_env.positionResFactor),
  orientationResFactor(_env.orientationResFactor),
  minmax_BodyAxisRange(_env.minmax_BodyAxisRange),
  m_robotGraph(_env.m_robotGraph),
  robotVec(_env.robotVec),
  m_filename(_env.m_filename)
{
  if(_env.m_boundaries) 
    m_boundaries=_env.m_boundaries;

  // only usable bodies in _env will be copied
  for (size_t i = 0; i < _env.GetUsableMultiBodyCount(); i++) {
    m_usableMultiBodies.push_back(_env.GetMultiBody(i));
  }

  // copy of surfaces 
  for (size_t i = 0; i < _env.GetNavigableSurfacesCount(); i++) {
    m_navigableSurfaces.push_back(_env.GetNavigableSurface(i));
  }
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  rd_res=_env.GetRdRes();
#endif
  SelectUsableMultibodies();
}

/**
 * Copy Constructor
 * receiving a bounding box (not necessarily the same as the original
 * environment
 */
Environment::
Environment(const Environment &_env, const Boundary &_boundary) :
  positionRes(_env.positionRes),
  orientationRes(_env.orientationRes),
  positionResFactor(_env.positionResFactor),
  orientationResFactor(_env.orientationResFactor),
  minmax_BodyAxisRange(_env.minmax_BodyAxisRange),
  m_robotGraph(_env.m_robotGraph),
  robotVec(_env.robotVec),
  m_filename(_env.m_filename)
{
  if(_env.m_boundaries) 
    m_boundaries=_env.m_boundaries;
  for (size_t i = 0; i < _env.GetUsableMultiBodyCount(); i++) {
    m_usableMultiBodies.push_back(_env.GetMultiBody(i));
  }
  // copy of surfaces 
  for (size_t i = 0; i < _env.GetNavigableSurfacesCount(); i++) {
    m_navigableSurfaces.push_back(_env.GetNavigableSurface(i));
  }
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  rd_res=_env.GetRdRes();
#endif
  SelectUsableMultibodies(); // select usable multibodies
}

///\brief Constructor taking in an XML node for parsing
///\todo Fix hack to input Env file ... Shouln't use Input Class.
///\todo Fix boundaries init

Environment::
Environment(XMLNodeReader& _node) : 
  m_usableExternalbodyCount(0),
  positionRes(ENV_RES_DEFAULT),
  orientationRes(ENV_RES_DEFAULT),
  positionResFactor(ENV_RES_DEFAULT),
  orientationResFactor(ENV_RES_DEFAULT),
  minmax_BodyAxisRange(0),
  m_filename("")
{
  _node.verifyName(string("Environment"));

  double pos_res = -1.0, ori_res=-1.0l;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  rd_res = 0.005;
#endif
 
  m_filename = _node.stringXMLParameter("input_env", true, "", "env filename");
  Read(m_filename);

  XMLNodeReader::childiterator citr;
  size_t num_joints = 0;
  for ( citr = _node.children_begin(); citr!= _node.children_end(); ++citr ) {
    if ( citr->getName() == "robot") {
      string cfg_type = citr->stringXMLParameter("Cfg_name", true, "", "type of robot");

      num_joints = citr->numberXMLParameter(string("num_joints"),true,0,0,MAX_INT,string("num_joints"));

      vector<Robot> robots = robotVec;
      Cfg::InitRobots(robots);

      XMLNodeReader::childiterator citr2;
      for ( citr2 = citr->children_begin(); citr2!= citr->children_end(); ++citr2 ) {
        if ( citr2->getName() == "boundary" ) {
          string type = citr2->stringXMLParameter("type",true,"","type");
          if(type == "bbox"){
            m_boundaries = shared_ptr<BoundingBox>(new BoundingBox(*citr2));
          }
          else if(type == "bshpere") {
            m_boundaries = shared_ptr<BoundingSphere>(new BoundingSphere(*citr2));
          }

         //@todo assumption of input bbox not strong. When no bbox provided call FindBoundingBox()
        } else {
          citr2->warnUnknownNode();
        }
      }
    } else if ( citr->getName() == "resolution" ) {
      pos_res = citr->numberXMLParameter("pos_res", false, -1.0, -1.0, MAX_DBL, "position resolution");
      ori_res = citr->numberXMLParameter("ori_res", false, -1.0, -1.0, MAX_DBL, "orientation resolution");
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
      rd_res = citr->numberXMLParameter("rd_res", false, .005, .00001, MAX_DBL, "reachable distance resolution");
#endif
      positionResFactor    = citr->numberXMLParameter("pos_res_factor", false, 0.05, 0.0, MAX_DBL, "position resolution factor");
      orientationResFactor = citr->numberXMLParameter("ori_res_factor", false, 0.05, 0.0, MAX_DBL, "orientation resolution factor");
    } else {
      citr->warnUnknownNode();
    }
  }
 
  // Compute RESOLUTION
  ComputeResolution(pos_res, ori_res, positionResFactor, orientationResFactor, num_joints);
  
  SelectUsableMultibodies();
}



///////////////////////////////

void Environment::
PrintOptions(ostream& out_os) {
  out_os << "  Environment" << endl;
  out_os << "    positionRes = " << positionRes << "; orientationRes = " << orientationRes << endl;
  out_os << "    positionResFactor = " << positionResFactor << "; orientationResFactor = " << orientationResFactor << endl;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  out_os << "    rd_res= " << rd_res << "; rd_res=" <<rd_res << endl;
#endif
  out_os << "    bbox = ";
  m_boundaries->Print(out_os);
  out_os << endl;
}

//===================================================================
//  ComputeResolution, if _posRes and _oriRes are <0, auto compute
//  the resolutions based on min_max body spans.
//===================================================================
void 
Environment::ComputeResolution(double _posRes, double _oriRes, 
    double _posResFactor, double _oriResFactor, size_t _numJoints){
  // NOTE: orientationResFactor is valid input, but not used.
  if (m_activeBodies.empty()){
    cerr << "Environment::ComputeResolution error - no active multibodies in the environment!" << endl;
    exit(-1);
  }

  m_activeBodies[0]->FindBoundingBox();
  double bodiesMinSpan = m_activeBodies[0]->GetMaxAxisRange();
  for(size_t i = 1 ; i < m_activeBodies.size() ; i++){
      m_activeBodies[i]->FindBoundingBox();
      bodiesMinSpan = min(bodiesMinSpan,m_activeBodies[i]->GetMaxAxisRange());
  }

  for(size_t i = 0 ; i < m_otherMultiBodies.size() ; i++){
      m_otherMultiBodies[i]->FindBoundingBox();
      bodiesMinSpan = min(bodiesMinSpan,m_otherMultiBodies[i]->GetMaxAxisRange());
  }
 
  // Set to XML input resolution if specified, else compute resolution factor
  if ( _posRes > 0 ) positionRes = _posRes;
  else                   positionRes = bodiesMinSpan * _posResFactor;

  if ( _oriRes > 0 ) orientationRes = _oriRes;
  else                   orientationRes = 0.05;
  
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  //make sure to calculate the rdRes based upon the DOF of the robot
  rd_res = _numJoints * rd_res;
#endif
  
  minmax_BodyAxisRange = bodiesMinSpan;
}

//===================================================================
//  Destructor
//===================================================================
Environment::
~Environment() {
}


//============================================
//SortBodies so that the external bodies appear first in the array
//============================================
void 
Environment::
SortMultiBodies(){
  if(!m_otherMultiBodies.empty()) { //the robot is not the only object
    size_t i = 0;
    size_t j = m_otherMultiBodies.size()-1;
    while (i < j) {
      //Quicksort
      while((i<m_otherMultiBodies.size()) && !m_otherMultiBodies[i]->IsInternal()) 
        i++;
      while ((j>0) && (m_otherMultiBodies[j]->IsInternal()))
        j--;
      if (i<j) {
        shared_ptr<MultiBody> pMidBody = m_otherMultiBodies[j];//switch multibody[i] & multibody[j]
        m_otherMultiBodies[j] = m_otherMultiBodies[i];
        m_otherMultiBodies[i] = pMidBody;
      }
    }
    if (i != j+1)
      cerr << "Wrong sorting in void Environment::SortMultiBodies(){}"<<endl;
      exit(-1);
  }
}


//Get rid of obstacles outside the bounding box
void 
Environment::
SelectUsableMultibodies() {
  m_usableMultiBodies = m_activeBodies;
  m_usableExternalbodyCount = 0;

  // get workspace bounding box
  double minx, maxx, miny, maxy, minz, maxz;

  minx = m_boundaries->GetRange(0).first; 
  maxx = m_boundaries->GetRange(0).second;
  miny = m_boundaries->GetRange(1).first; 
  maxy = m_boundaries->GetRange(1).second;
  
  if(m_boundaries->GetPosDOFs() < 3){
    minz = 0;
    maxz = 0;
  }
  else{
    minz = m_boundaries->GetRange(2).first; 
    maxz = m_boundaries->GetRange(2).second;
  }

  for (size_t i = 0; i < m_otherMultiBodies.size(); i++) {
      m_otherMultiBodies.at(i)->FindBoundingBox();
      const double *obb = m_otherMultiBodies.at(i)->GetBoundingBox();
        if (((obb[0] <= maxx && obb[0] >= minx) || (obb[1] <= maxx && obb[1] >= minx)) &&
            ((obb[2] <= maxy && obb[2] >= miny) || (obb[3] <= maxy && obb[3] >= miny)) &&
            ((obb[4] <= maxz && obb[4] >= minz) || (obb[5] <= maxz && obb[5] >= minz))) {
          // any point in obstacle's bbox inside boundaries => obstacle is usable
          m_usableMultiBodies.push_back(m_otherMultiBodies[i]);
          if (!(m_otherMultiBodies[i]->IsInternal()))
            m_usableExternalbodyCount++;
        } else { // bounding boxes cross each other 
          if (!(obb[0] > maxx || obb[1] < minx || 
                obb[2] > maxy || obb[3] < miny || 
                obb[4] > maxz || obb[5] < minz)) {
            m_usableMultiBodies.push_back(m_otherMultiBodies[i]);
            if (!(m_otherMultiBodies[i]->IsInternal()))
              m_usableExternalbodyCount++;
          }
        }
  }
}


//===================================================================
//  Write
//
//  Function: Write the Input data for an environment into a 
//  file
//
//===================================================================
void 
Environment::
Write(ostream & _os) {
    _os << m_usableMultiBodies.size() << endl;
    for (size_t i=0; i < m_usableMultiBodies.size(); i++)
        m_usableMultiBodies[i]->Write(_os);
}

/*void 
Environment::
FindBoundingBox(){
  if (m_multibody.empty()){
    cerr << "Environment::FindBoundingBox() error - no multibodies exist." << endl;
    exit(1);
  }

  const double * tmp;
  double minx, miny, minz, maxx, maxy, maxz;
  minx = miny = minz = maxx = maxy = maxz = 0;

  m_multibody[0]->FindBoundingBox();
  double bodiesMinSpan = m_multibody[0]->GetMaxAxisRange();
  tmp = m_multibody[0]->GetBoundingBox();
  minx = tmp[0]; maxx = tmp[1];
  miny = tmp[2]; maxy = tmp[3];
  minz = tmp[4]; maxz = tmp[5];

  //loop over rest of multibody vec
  for(size_t i = 1 ; i < m_multibody.size() ; i++){
      m_multibody[i]->FindBoundingBox();
      tmp = m_multibody[i]->GetBoundingBox();
      minx = min(minx,tmp[0]); maxx = max(maxx,tmp[1]);
      miny = min(miny,tmp[2]); maxy = max(maxy,tmp[3]);
      minz = min(minz,tmp[4]); maxz = max(maxz,tmp[5]);
      bodiesMinSpan = min(bodiesMinSpan,m_multibody[i]->GetMaxAxisRange());
  }
  //loop over nav surfaces
  for(size_t i = 0 ; i < m_navigableSurfaces.size() ; i++){
    m_navigableSurfaces[i]->FindBoundingBox();
    tmp = m_navigableSurfaces[i]->GetBoundingBox();
    minx = min(minx,tmp[0]); maxx = max(maxx,tmp[1]);
    miny = min(miny,tmp[2]); maxy = max(maxy,tmp[3]);
    minz = min(minz,tmp[4]); maxz = max(maxz,tmp[5]);
    bodiesMinSpan = min(bodiesMinSpan,m_navigableSurfaces[i]->GetMaxAxisRange());
  }

  //Also, now that environment does not track a "robot" index and there are potentially many robots,
  //something more sophisticated needs to be done here
  double min_clearance = robotSpan/3.0;
  vector<double> boundingBox;
  boundingBox.push_back(minx-min_clearance); 
  boundingBox.push_back(maxx+min_clearance);
  boundingBox.push_back(miny-min_clearance); 
  boundingBox.push_back(maxy+min_clearance);
  boundingBox.push_back(minz-min_clearance); 
  boundingBox.push_back(maxz+min_clearance);

  m_boundaries->SetRange(boundingBox);
  m_boundaries->TranslationalScale(2); ///\todo fix this default.
  //defaults bbox_scale to 2 when no bbox is defined.

  positionRes = bodiesMinSpan * positionResFactor;
  minmax_BodyAxisRange = bodiesMinSpan;
}*/

void
Environment::
ResetBoundingBox(double _d, size_t _robotIndex){
  if (m_activeBodies.empty()){
    cerr << "Environmenet::ResetBoundingBox error - no active multibodies present." << endl;
    exit(1);
  }
  const double * tmp;
  double minx, miny, minz, maxx, maxy, maxz;
  double origin_minx, origin_miny, origin_minz, origin_maxx, origin_maxy, origin_maxz;

  double robot_radius = GetMultiBody(_robotIndex)->GetBody(0)->GetPolyhedron().m_maxRadius;
  _d += robot_radius;

  origin_minx = m_boundaries->GetRange(0).first;
  origin_maxx = m_boundaries->GetRange(0).second;
  origin_miny = m_boundaries->GetRange(1).first;
  origin_maxy = m_boundaries->GetRange(1).second;
  origin_minz = m_boundaries->GetRange(2).first;
  origin_maxz = m_boundaries->GetRange(2).second;
  minx = miny = minz = maxx = maxy = maxz = 0;

  //set initial values of tmp and mins
  m_activeBodies[0]->FindBoundingBox();
  tmp = m_activeBodies[0]->GetBoundingBox();
  minx = tmp[0];  maxx = tmp[1];
  miny = tmp[2];  maxy = tmp[3];
  minz = tmp[4];  maxz = tmp[5];

  //TODO: Andy - Why are we not resetting the given robot index's BB?

  for(size_t i=1; i<m_activeBodies.size(); i++) {
    if(i != _robotIndex) {
      m_activeBodies[i]->FindBoundingBox();
      tmp = m_activeBodies[i]->GetBoundingBox();
      minx = min(minx, tmp[0]);  maxx = max(maxx, tmp[1]);
      miny = min(miny, tmp[2]);  maxy = max(maxy, tmp[3]);
      minz = min(minz, tmp[4]);  maxz = max(maxz, tmp[5]);
    }
  }

  for(size_t i=0; i<m_otherMultiBodies.size(); i++) {
      m_otherMultiBodies[i]->FindBoundingBox();
      tmp = m_otherMultiBodies[i]->GetBoundingBox();
      minx = min(minx, tmp[0]);  maxx = max(maxx, tmp[1]);
      miny = min(miny, tmp[2]);  maxy = max(maxy, tmp[3]);
      minz = min(minz, tmp[4]);  maxz = max(maxz, tmp[5]);
  }

  minx = min(minx-_d, origin_minx);
  maxx = max(maxx+_d, origin_maxx);
  miny = min(miny-_d, origin_miny);
  maxy = max(maxy+_d, origin_maxy);
  minz = min(minz-_d, origin_minz);
  maxz = max(maxz+_d, origin_maxz);

  vector<double> boundingBox;
  boundingBox.push_back(minx);
  boundingBox.push_back(maxx);
  boundingBox.push_back(miny);
  boundingBox.push_back(maxy);
  boundingBox.push_back(minz);
  boundingBox.push_back(maxz);

  m_boundaries->SetRange(boundingBox);
}

//===================================================================
//  Get_minmax_BodyAxisRange
//===================================================================
double 
Environment::
Getminmax_BodyAxisRange(){
  return minmax_BodyAxisRange;
}

shared_ptr<Boundary>
Environment::GetBoundary() const {
  return m_boundaries;
}

void Environment::SetBoundary(shared_ptr<Boundary> _b){
  m_boundaries = _b;
}

void 
Environment::Read(string _filename) {  
  VerifyFileExists(_filename);

  // open file and read first field
  ifstream ifs(_filename.c_str());

  int multibodyCount = ReadField<int>(ifs, "Number of Multibodies");
  //m_robotIndices.clear();

  for (int m=0; m<multibodyCount; m++) {    
    shared_ptr<MultiBody> mb(new MultiBody());
    mb->Read(ifs, false/*m_debug*/);

    //if( mb->GetFreeBodyCount() == 1) {
      //initial simple test to see if moveable robot
      //m_robotIndices.push_back(m);
    //}

    if( mb->IsActive() )
      m_activeBodies.push_back(mb);
    else if (!mb->IsSurface())
      m_otherMultiBodies.push_back(mb);
    else
      m_navigableSurfaces.push_back(mb);
  }

  ifs.close();
  BuildRobotStructure();
}

void
Environment::
BuildCDstructure(cd_predefined cdtype){
  for(vector<shared_ptr<MultiBody> >::iterator M = m_activeBodies.begin(); M != m_activeBodies.end(); ++M)
    (*M)->buildCDstructure(cdtype);

  for(vector<shared_ptr<MultiBody> >::iterator M = m_otherMultiBodies.begin(); M != m_otherMultiBodies.end(); ++M)
    (*M)->buildCDstructure(cdtype);
}

void
Environment::BuildRobotStructure() {
  if (m_activeBodies.empty()){
    cerr << "Error! No robots present in the environment!" << endl;
    exit(1);
  }

  size_t _robotIndex = 0; //assume homogenous flock
  shared_ptr<MultiBody> robot = m_activeBodies[_robotIndex];
  int fixedBodyCount = robot->GetFixedBodyCount();
  int freeBodyCount = robot->GetFreeBodyCount();
  for (int i = 0; i < fixedBodyCount; i++) {
    m_robotGraph.add_vertex(i);
  }
  for (int i = 0; i < freeBodyCount; i++) {
    m_robotGraph.add_vertex(i + fixedBodyCount); //Need to account for FixedBodies added above
  }
  //Total amount of bodies in environment: free + fixed
  for (int i = 0; i < freeBodyCount + fixedBodyCount; i++){
    shared_ptr<Body> body = robot -> GetBody(i);  
    //For each body, find forward connections and connect them 
    for (int j = 0; j < body->ForwardConnectionCount(); j++) {
      shared_ptr<Body> forward = body -> GetForwardConnection(j).GetNextBody();
      if (forward -> IsFixedBody()) {
        //Quick hack to avoid programming ability to determine subclass
        shared_ptr<FixedBody> castFixedBody = boost::dynamic_pointer_cast<FixedBody>(forward);
        int nextIndex = robot -> GetFixedBodyIndex(castFixedBody);
        m_robotGraph.add_edge(i, nextIndex);
      }
      else {
        shared_ptr<FreeBody> castFreeBody = boost::dynamic_pointer_cast<FreeBody>(forward);
        int nextIndex = robot -> GetFreeBodyIndex(castFreeBody);
        m_robotGraph.add_edge(i, nextIndex);
      }
    } 
  }

  //Robot ID typedef
  typedef RobotGraph::vertex_descriptor RID; 
  vector< pair<size_t,RID> > ccs;
  stapl::sequential::vector_property_map< RobotGraph,size_t > cmap;
  //Initialize CC information
  get_cc_stats(m_robotGraph,cmap,ccs);
  if(ccs.size()>1)
    robot->SetMultirobot(true);
  for (size_t i = 0; i < ccs.size(); i++) {
    cmap.reset();
    vector<RID> cc;
    //Find CCs, construct robot objects
    get_cc(m_robotGraph, cmap, ccs[i].second, cc);
    size_t baseIndx = -1;
    for(size_t j = 0; j<cc.size(); j++){
      size_t index = m_robotGraph.find_vertex(cc[j])->property();
      if(robot->GetFreeBody(index)->IsBase()){
        baseIndx = index;
        break;
      }
    }
    if(baseIndx == size_t(-1)){
      cerr << "Each robot must have at least one base. Please fix .env file." << endl;
      exit(1);
    }

    Robot::Base bt = robot->GetFreeBody(baseIndx)->GetBase();
    Robot::BaseMovement bm = robot->GetFreeBody(baseIndx)->GetBaseMovement();
    Robot::JointMap jm;
    for(size_t j = 0; j<cc.size(); j++){
      size_t index = m_robotGraph.find_vertex(cc[j])->property();
      typedef Robot::JointMap::iterator MIT;
      for(MIT mit = robot->GetJointMap().begin(); mit!=robot->GetJointMap().end(); mit++){
        if(mit->first.first == index){
          jm.push_back(*mit);
        }
      }
    }
    robotVec.push_back(Robot(bt, bm, jm, baseIndx));
  }
}

bool
Environment::
operator==(const Environment& _rhs) const
{
  if(m_activeBodies.size() != _rhs.m_activeBodies.size() ||
      m_otherMultiBodies.size() != _rhs.m_otherMultiBodies.size() ||
      m_usableMultiBodies.size() != _rhs.m_usableMultiBodies.size())
    return false;

  for(size_t i=0; i<m_activeBodies.size(); ++i)
    if(*(m_activeBodies[i]) != (*(_rhs.m_activeBodies[i])))
      return false;

  for(size_t i=0; i<m_otherMultiBodies.size(); ++i)
    if(*(m_otherMultiBodies[i]) != (*(_rhs.m_otherMultiBodies[i])))
      return false;

  for(size_t i=0; i<m_usableMultiBodies.size(); ++i)
    if(*(m_usableMultiBodies[i]) != (*(_rhs.m_usableMultiBodies[i])))
      return false;

  return (m_usableExternalbodyCount == _rhs.m_usableExternalbodyCount &&
    (*m_boundaries == *_rhs.m_boundaries) &&
    (positionRes == _rhs.positionRes) &&
    (orientationRes == _rhs.orientationRes) &&
    (minmax_BodyAxisRange == _rhs.minmax_BodyAxisRange));
}


//-------------------------------------------------------------------
///  GetRandomNavigableSurfaceIndex
///  Output: An index between -1 and m_navigableSurfaces.size()-1
///          -1 means base index 
//-------------------------------------------------------------------
size_t Environment::
GetRandomNavigableSurfaceIndex()  {
  size_t num_surfaces = GetNavigableSurfacesCount();
  size_t rindex = (LRand() % (num_surfaces+1)) - 1;
  return rindex;
}

shared_ptr<MultiBody>
Environment::GetRandomObstacle() const{
  //if there are k active bodies, then we'll assume that the first k entries of m_usableMultiBodies are the active
  //bodies, allowing us to choose a random index in (n-k, n)
  size_t activeCount = m_activeBodies.size();
  if (activeCount >= m_usableMultiBodies.size()){
    cerr << "Environment::GetRandomObstacle error - no usable obstacles." << endl;
    exit(1);
  }

  size_t rIndex = (LRand() % (m_usableMultiBodies.size() - activeCount)) + activeCount;
  return m_usableMultiBodies.at(rIndex);
}
