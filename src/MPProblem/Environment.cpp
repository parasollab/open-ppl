#include "Environment.h"
#include "MPProblem.h"
#include <fstream>
#include <iostream>
#include "boost/pointer_cast.hpp"

#define ENV_RES_DEFAULT                    0.05

//===================================================================
//  Constructors
//===================================================================


Environment::
Environment():
  pathVersion(PATHVER_20001125),
  usable_externalbody_count(0),
  robotIndex(0),
  positionRes(ENV_RES_DEFAULT),
  orientationRes(ENV_RES_DEFAULT),
  positionResFactor(ENV_RES_DEFAULT),
  orientationResFactor(ENV_RES_DEFAULT),
  minmax_BodyAxisRange(0),
  m_filename("")
{}

Environment::
Environment(shared_ptr<Boundary> _bb):
  pathVersion(PATHVER_20001125),
  usable_externalbody_count(0),
  robotIndex(0),
  m_boundaries(_bb),
  positionRes(ENV_RES_DEFAULT),
  orientationRes(ENV_RES_DEFAULT),
  positionResFactor(ENV_RES_DEFAULT),
  orientationResFactor(ENV_RES_DEFAULT),
  minmax_BodyAxisRange(0),
  m_filename("")
{}



/*
Environment::
Environment(int dofs, int pos_dofs, Input * _input) :
  pathVersion(PATHVER_20001125),
  usable_externalbody_count(0),
  robotIndex(0),
  positionRes(POSITION_RES_FACTOR),
  orientationRes(ORIENTATION_RES),
  minmax_BodyAxisRange(0),
  m_filename("")
{
  boundaries = new BoundingBox(dofs, pos_dofs);

  if (_input != NULL) {
    Read(_input->envFile.GetValue(), PMPL_EXIT,
	 _input->descDir.GetValue(), _input->cdtype);
    FindBoundingBox();
    
    //   if user supplied a bounding box, use it instead
    if (_input->bbox.IsActivated()) {
      std::stringstream i_bbox;
      i_bbox << _input->bbox.GetValue();
      boundaries->Parse(i_bbox);  
    }

    // if user supplied a positional resolution, use it instead
    if ( _input->posres.IsActivated() )
      positionRes = _input->posres.GetValue();
    
    // orientational resolution may be user supplied but at this time
    // is not calculated
    if ( _input->orires.IsActivated() )
      orientationRes = _input->orires.GetValue(); 
    
    // scale boundary
    if ( _input->bbox_scale.IsActivated() )
      boundaries->TranslationalScale(_input->bbox_scale.GetValue());  

    // activate objects inside the bounding box and deactivate other
    // objects
    SelectUsableMultibodies();  
  }
}
*/


/**
 * Copy Constructor
 */
Environment::
Environment(const Environment &from_env) :
  pathVersion(PATHVER_20001125),
  usable_externalbody_count(from_env.usable_externalbody_count),
  robotIndex(from_env.robotIndex),
  positionRes(from_env.positionRes),
  orientationRes(from_env.orientationRes),
  positionResFactor(from_env.positionResFactor),
  orientationResFactor(from_env.orientationResFactor),
  minmax_BodyAxisRange(from_env.minmax_BodyAxisRange),
  m_filename(from_env.m_filename)
{
  m_boundaries=from_env.m_boundaries;
  if(m_boundaries->GetDOFs() + m_boundaries->GetPosDOFs() == 0)
    cout << "FOUND EMPTY BBOX! (Environment copy constructor)\n";

  // only usable bodies in from_env will be copied
  for (int i = 0; i < from_env.GetMultiBodyCount(); i++) {
    multibody.push_back(from_env.GetMultiBody(i));
    //usable_multibody.push_back(from_env.GetMultiBody(i));
  }
  // copy of surfaces 
  for (int i = 0; i < from_env.GetNavigableSurfacesCount(); i++) {
    m_navigableSurfaces.push_back(from_env.GetNavigableSurface(i));
  }
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  rd_res=from_env.GetRdRes();
#endif
  SelectUsableMultibodies();
}


/**
 * Copy Constructor, copies from MPProblem's environemnt
 */
Environment::
Environment(MPProblem* in_pProblem) : 
  MPBaseObject(in_pProblem),
  pathVersion(PATHVER_20001125),
  usable_externalbody_count(in_pProblem->GetEnvironment()->usable_externalbody_count),
  robotIndex(in_pProblem->GetEnvironment()->robotIndex),
  positionRes(in_pProblem->GetEnvironment()->positionRes),
  orientationRes(in_pProblem->GetEnvironment()->orientationRes),
  positionResFactor(in_pProblem->GetEnvironment()->positionResFactor),
  orientationResFactor(in_pProblem->GetEnvironment()->orientationResFactor),
  minmax_BodyAxisRange(in_pProblem->GetEnvironment()->minmax_BodyAxisRange),
  m_filename(in_pProblem->GetEnvironment()->m_filename)
{
  Environment& from_env = *(in_pProblem->GetEnvironment());
  m_boundaries=from_env.m_boundaries; 
  if(m_boundaries->GetDOFs() + m_boundaries->GetPosDOFs() == 0)
    cout << "FOUND EMPTY BBOX! (MPProblem constructor) \n";

  for (int i = 0; i < from_env.GetMultiBodyCount(); i++) {
    multibody.push_back(from_env.GetMultiBody(i));
    //usable_multibody.push_back(from_env.GetMultiBody(i));
  }
  // copy of surfaces 
  for (int i = 0; i < from_env.GetNavigableSurfacesCount(); i++) {
    m_navigableSurfaces.push_back(from_env.GetNavigableSurface(i));
  }
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  rd_res=from_env.GetRdRes();
#endif
  SelectUsableMultibodies();
}


/**
 * Copy Constructor
 * receiving a bounding box (not necessarily the same as the original
 * environment
 */
Environment::
Environment(const Environment &from_env, const Boundary &i_boundaries) :
  pathVersion(PATHVER_20001125),
  robotIndex(from_env.robotIndex),
  positionRes(from_env.positionRes),
  orientationRes(from_env.orientationRes),
  positionResFactor(from_env.positionResFactor),
  orientationResFactor(from_env.orientationResFactor),
  minmax_BodyAxisRange(from_env.minmax_BodyAxisRange),
  m_filename(from_env.m_filename)
{
  m_boundaries=from_env.m_boundaries;
  for (int i = 0; i < from_env.GetMultiBodyCount(); i++) {
    multibody.push_back(from_env.GetMultiBody(i));
  }
  // copy of surfaces 
  for (int i = 0; i < from_env.GetNavigableSurfacesCount(); i++) {
    m_navigableSurfaces.push_back(from_env.GetNavigableSurface(i));
  }
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  rd_res=from_env.GetRdRes();
#endif
  SelectUsableMultibodies(); // select usable multibodies
}

///\brief Constructor taking in an XML node for parsing
///\todo Fix hack to input Env file ... Shouln't use Input Class.
///\todo Fix boundaries init

Environment::
Environment(XMLNodeReader& in_Node,  MPProblem* in_pProblem) : 
  MPBaseObject(in_Node, in_pProblem),
  pathVersion(PATHVER_20001125),
  usable_externalbody_count(0),
  robotIndex(0),
  positionRes(ENV_RES_DEFAULT),
  orientationRes(ENV_RES_DEFAULT),
  positionResFactor(ENV_RES_DEFAULT),
  orientationResFactor(ENV_RES_DEFAULT),
  minmax_BodyAxisRange(0),
  m_filename("")
{
  in_Node.verifyName(string("environment"));

  double pos_res = -1.0, ori_res=-1.0l;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  rd_res = 0.005;
#endif
 
  m_filename = in_Node.stringXMLParameter("input_env", true, "", "env filename");
  Read(m_filename);

  XMLNodeReader::childiterator citr;
  size_t num_joints = 0;
  for ( citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr ) {
    if ( citr->getName() == "robot") {
      string cfg_type = citr->stringXMLParameter("Cfg_name", true, "", "type of robot");
#ifdef PMPRigidMulti
      if(cfg_type == "Cfg_free_multi") {
        int num_robots = citr->numberXMLParameter("num_robots", true, 1, 1, MAX_INT, "number of robots");
        CfgType::setNumofRobots(num_robots);
      }
#endif

      num_joints = citr->numberXMLParameter(string("num_joints"),true,0,0,MAX_INT,string("num_joints"));
      CfgType tmp;
#ifndef PMPManifold
      vector<Robot> robots = tmp.GetRobots(num_joints);
#else
      vector<Robot> robots = robotVec;
#endif
      Cfg::InitRobots(robots);
      //in_pProblem->SetNumOfJoints(num_joints);

      XMLNodeReader::childiterator citr2;
      for ( citr2 = citr->children_begin(); citr2!= citr->children_end(); ++citr2 ) {
        if ( citr2->getName() == "boundary" ) {
          string type = citr2->stringXMLParameter("type",true,"","type");
          boundaryType = type;
          if(type == "bbox"){
            m_boundaries = shared_ptr<BoundingBox>(new BoundingBox(*citr2,in_pProblem));
          }
          else if(type == "bshpere") {
            m_boundaries = shared_ptr<BoundingSphere>(new BoundingSphere(*citr2,in_pProblem));
          }

          //@todo assumption of input bbox not strong. When no bbox provided call FindBoundingBox() 
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
  
  PrintOptions(cout);
    
  SelectUsableMultibodies();
}



///////////////////////////////

Environment::
Environment(const Environment& from_env, string filename) : 
  pathVersion(PATHVER_20001125),
  usable_externalbody_count(0),
  robotIndex(0),
  positionRes(from_env.positionRes),
  orientationRes(from_env.orientationRes),
  positionResFactor(from_env.positionResFactor),
  orientationResFactor(from_env.orientationResFactor),
  minmax_BodyAxisRange(0),
  m_filename(filename)
{
  m_boundaries=from_env.m_boundaries;
  if(m_boundaries->GetDOFs() + m_boundaries->GetPosDOFs() == 0)
    cout << "FOUND EMPTY BBOX! (Environment copy constructor, with filename)\n";
         
  Read(filename);
  FindBoundingBox();

  //compute RESOLUTION
  multibody[robotIndex]->FindBoundingBox();
  double bodies_min_span = multibody[robotIndex]->GetMaxAxisRange();
    
  bool first = true;
  for(size_t i = 0 ; i < multibody.size() ; i++){
    if((int)i != robotIndex){
      if(first){
        multibody[i]->FindBoundingBox();
        first = false;
        bodies_min_span = min(bodies_min_span,multibody[i]->GetMaxAxisRange());
      }
      else{
        multibody[i]->FindBoundingBox();
        bodies_min_span = min(bodies_min_span,multibody[i]->GetMaxAxisRange());
      } 
    }
  }
  minmax_BodyAxisRange = bodies_min_span;
  // END compute RESOLUTION

  SelectUsableMultibodies();
}


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
  multibody[robotIndex]->FindBoundingBox();
  double robot_span = multibody[robotIndex]->GetMaxAxisRange();
  double bodies_min_span = robot_span;
  for(size_t i = 0 ; i < multibody.size() ; i++){
    if((int)i != robotIndex){
      multibody[i]->FindBoundingBox();
      bodies_min_span = min(bodies_min_span,multibody[i]->GetMaxAxisRange());
    }
  }
 
  // Set to XML input resolution if specified, else compute resolution factor
  if ( _posRes > 0 ) positionRes = _posRes;
  else                   positionRes = bodies_min_span * _posResFactor;

  if ( _oriRes > 0 ) orientationRes = _oriRes;
  else                   orientationRes = 0.05;
  
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  //make sure to calculate the rdRes based upon the DOF of the robot
  rd_res = _numJoints * rd_res;
#endif
  
  minmax_BodyAxisRange = bodies_min_span;
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
  if(multibody.size() != 1) { //the robot is not the only object
    int i = 0;
    int j = multibody.size()-1;
    while (i < j) {
      //Quicksort
      while((i<(int)multibody.size()) && !multibody[i]->IsInternal()) 
        i++;
      while ((j>0) && (multibody[j]->IsInternal()))
        j--;
      if (i<j) {
        shared_ptr<MultiBody> pMidBody = multibody[j];//switch multibody[i] & multibody[j]
        multibody[j] = multibody[i];
        multibody[i] = pMidBody;
      }
    }
    if (i != j+1)
      cout << "Wrong sorting in void Environment::SortMultiBodies(){}"<<endl;
  }
}


//Get rid of obstacles outside the bounding box
void 
Environment::
SelectUsableMultibodies() {
  usable_multibody.clear();
  usable_externalbody_count = 0;

  //add robot to list of usable_multibodies
  int original_robotIndex = robotIndex;
  if(robotIndex < (int)multibody.size())
  {
    usable_multibody.push_back(multibody[robotIndex]);
    robotIndex = usable_multibody.size()-1;
    usable_externalbody_count++;
  }

  if((m_boundaries->GetPosDOFs() < (int)CfgType().PosDOF()) || (m_boundaries->GetDOFs() < (int)CfgType().DOF()))
    return;

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

  for (size_t i = 0; i < multibody.size(); i++) 
    if((int)i != original_robotIndex) { // @todo: need a test function in multibody to
      //see if bounding box of multibody overlaps BB
      multibody[i]->FindBoundingBox();
      const double *obb = multibody[i]->GetBoundingBox();
        if (((obb[0] <= maxx && obb[0] >= minx) || (obb[1] <= maxx && obb[1] >= minx)) &&
            ((obb[2] <= maxy && obb[2] >= miny) || (obb[3] <= maxy && obb[3] >= miny)) &&
            ((obb[4] <= maxz && obb[4] >= minz) || (obb[5] <= maxz && obb[5] >= minz))) {
          // any point in obstacle's bbox inside boundaries => obstacle is usable
          usable_multibody.push_back(multibody[i]);
          if (!(multibody[i]->IsInternal()))
            usable_externalbody_count++;
        } else { // bounding boxes cross each other 
          if (!(obb[0] > maxx || obb[1] < minx || 
                obb[2] > maxy || obb[3] < miny || 
                obb[4] > maxz || obb[5] < minz)) {
            usable_multibody.push_back(multibody[i]);
            if (!(multibody[i]->IsInternal()))
              usable_externalbody_count++;
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
    _os << usable_multibody.size() << endl;
    for (size_t i=0; i < usable_multibody.size(); i++)
        usable_multibody[i]->Write(_os);
}

void 
Environment::
FindBoundingBox(){
  double robot_span;
  multibody[robotIndex]->FindBoundingBox();
  robot_span = multibody[robotIndex]->GetMaxAxisRange();
  double bodies_min_span;
  bodies_min_span = robot_span;
    
  bool first = true;
  const double * tmp;
  double minx, miny, minz, maxx, maxy, maxz;
  minx = miny = minz = maxx = maxy = maxz = 0;
  //loop over multibody vec
  for(size_t i = 0 ; i < multibody.size() ; i++){
    if((int)i != robotIndex){
      if(first){
	multibody[i]->FindBoundingBox();
	tmp = multibody[i]->GetBoundingBox();
	minx = tmp[0]; maxx = tmp[1];
	miny = tmp[2]; maxy = tmp[3];
	minz = tmp[4]; maxz = tmp[5];
	first = false;
	bodies_min_span = min(bodies_min_span,multibody[i]->GetMaxAxisRange());
      }
      else{
	multibody[i]->FindBoundingBox();
	tmp = multibody[i]->GetBoundingBox();
	minx = min(minx,tmp[0]); maxx = max(maxx,tmp[1]);
	miny = min(miny,tmp[2]); maxy = max(maxy,tmp[3]);
	minz = min(minz,tmp[4]); maxz = max(maxz,tmp[5]);
	bodies_min_span = min(bodies_min_span,multibody[i]->GetMaxAxisRange());
      }
    }
  }
  //loop over nav surfaces
  for(size_t i = 0 ; i < m_navigableSurfaces.size() ; i++){
    m_navigableSurfaces[i]->FindBoundingBox();
    tmp = m_navigableSurfaces[i]->GetBoundingBox();
    minx = min(minx,tmp[0]); maxx = max(maxx,tmp[1]);
    miny = min(miny,tmp[2]); maxy = max(maxy,tmp[3]);
    minz = min(minz,tmp[4]); maxz = max(maxz,tmp[5]);
    bodies_min_span = min(bodies_min_span,m_navigableSurfaces[i]->GetMaxAxisRange());
  }
  
  double min_clearance = robot_span/3.0;
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

  positionRes = bodies_min_span * positionResFactor;
  minmax_BodyAxisRange = bodies_min_span;
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
  
  for (int m=0; m<multibodyCount; m++) {    
    shared_ptr<MultiBody> mb(new MultiBody());
    mb->Read(ifs, m_debug);
    if( !mb->IsSurface() )
      multibody.push_back(mb);
    else
      m_navigableSurfaces.push_back(mb);
  }
  
  ifs.close();

  BuildRobotStructure();
}

void
Environment::
buildCDstructure(cd_predefined cdtype)
{
  for(vector<shared_ptr<MultiBody> >::iterator M = multibody.begin(); M != multibody.end(); ++M)
    (*M)->buildCDstructure(cdtype);
}

void
Environment::BuildRobotStructure()
{
    shared_ptr<MultiBody> robot = multibody[robotIndex];
    int fixedBodyCount = robot -> GetFixedBodyCount();
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

  write_vertices_edges_graph(m_robotGraph,cout); 
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
operator==(const Environment& e) const
{
  if(multibody.size() != e.multibody.size())
    return false;
  for(size_t i=0; i<multibody.size(); ++i)
    if(*(multibody[i]) != (*(e.multibody[i])))
      return false;

  if(usable_multibody.size() != e.usable_multibody.size())
    return false;
  for(size_t i=0; i<usable_multibody.size(); ++i)
    if(*(usable_multibody[i]) != (*(e.usable_multibody[i])))
      return false;

  return (pathVersion == e.pathVersion) &&
         (usable_externalbody_count == e.usable_externalbody_count) &&
         (robotIndex == e.robotIndex) &&
         (*m_boundaries == *e.m_boundaries) &&
         (positionRes == e.positionRes) &&
         (orientationRes == e.orientationRes) &&
         (minmax_BodyAxisRange == e.minmax_BodyAxisRange);
}


//-------------------------------------------------------------------
///  GetRandomNavigableSurfaceIndex
///  Output: An index between -1 and m_navigableSurfaces.size()-1
///          -1 means base index 
//-------------------------------------------------------------------
int Environment::
GetRandomNavigableSurfaceIndex()  {
  int num_surfaces = GetNavigableSurfacesCount();
  int rindex = (LRand() % (num_surfaces+1)) - 1;
  return rindex;
}
