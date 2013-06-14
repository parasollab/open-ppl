#include "Environment.h"

#include "boost/pointer_cast.hpp"

#include "GraphAlgo.h"

#include "MPProblem/BoundingBox.h"
#include "MPProblem/BoundingSphere.h"

#include "Cfg/Cfg.h"

#define ENV_RES_DEFAULT                    0.05

Environment::Environment() :
  m_filename(""),
  m_positionRes(ENV_RES_DEFAULT),
  m_orientationRes(ENV_RES_DEFAULT) {
  }

Environment::Environment(XMLNodeReader& _node) {
  _node.verifyName("Environment");

  m_filename = _node.stringXMLParameter("filename", true, "", "env filename");
  m_positionRes = _node.numberXMLParameter("positionRes", false, -1.0, 0.0, MAX_DBL, "position resolution");
  double positionResFactor = _node.numberXMLParameter("positionResFactor", false, 0.05, 0.0, MAX_DBL, "position resolution factor");
  m_orientationRes = _node.numberXMLParameter("orientationRes", false, 0.05, 0.0, MAX_DBL, "orientation resolution");
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  m_rdRes = _node.numberXMLParameter("rdRes", false, .005, .00001, MAX_DBL, "reachable distance resolution");
#endif

  Read(m_filename);
  ComputeResolution(positionResFactor);
}

Environment::~Environment() {}

void 
Environment::Read(string _filename) {
  VerifyFileExists(_filename);
  m_filename = _filename;

  m_activeBodies.clear();
  m_obstacleBodies.clear();
  m_usableMultiBodies.clear();
  m_navigableSurfaces.clear();

  // open file
  ifstream ifs(_filename.c_str());

  //read boundary
  ReadBoundary(ifs);

  //read number of multibodies
  string mbds = ReadFieldString(ifs, "Multibodies tag.");
  if(mbds != "MULTIBODIES"){
    cerr << "Error reading tag for multibodies." << endl;
    exit(1);
  }
  int multibodyCount = ReadField<int>(ifs, "Number of Multibodies");

  //parse and construct each multibody
  for (int m=0; m<multibodyCount; m++) {    
    shared_ptr<MultiBody> mb(new MultiBody());
    mb->Read(ifs, false/*m_debug*/);

    if( mb->IsActive() )
      m_activeBodies.push_back(mb);
    else if (!mb->IsSurface())
      m_obstacleBodies.push_back(mb);
    else
      m_navigableSurfaces.push_back(mb);
  }

  m_usableMultiBodies = m_activeBodies;
  copy(m_obstacleBodies.begin(), m_obstacleBodies.end(), back_inserter(m_usableMultiBodies));

  ifs.close();
  BuildRobotStructure();
}

void
Environment::PrintOptions(ostream& _os) {
  _os << "Environment" << endl;
  _os << "\tpositionRes::" << m_positionRes << endl;
  _os << "\torientationRes::" << m_orientationRes << endl;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  _os << "\trdRes::" << m_rdRes << endl;
#endif
  _os << "\tboundary::" << *m_boundary << endl;
}

void 
Environment::Write(ostream & _os) {
  _os << m_usableMultiBodies.size() << endl;
  for (size_t i=0; i < m_usableMultiBodies.size(); i++)
    m_usableMultiBodies[i]->Write(_os);
}

//ComputeResolution, if _posRes is <0, auto compute
//the resolutions based on min_max body spans.
void 
Environment::ComputeResolution(double _positionResFactor){
  if(m_activeBodies.empty()){
    cerr << "Environment::ComputeResolution error - no active multibodies in the environment!" << endl;
    exit(-1);
  }

  double bodiesMinSpan = numeric_limits<double>::max();
  for(size_t i = 0 ; i < m_activeBodies.size() ; i++){
    m_activeBodies[i]->FindBoundingBox();
    bodiesMinSpan = min(bodiesMinSpan, m_activeBodies[i]->GetMaxAxisRange());
  }

  for(size_t i = 0 ; i < m_obstacleBodies.size() ; i++){
    m_obstacleBodies[i]->FindBoundingBox();
    bodiesMinSpan = min(bodiesMinSpan, m_obstacleBodies[i]->GetMaxAxisRange());
  }

  // Set to XML input resolution if specified, else compute resolution factor
  if(m_positionRes < 0)
    m_positionRes = bodiesMinSpan * _positionResFactor;

#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  //make sure to calculate the rdRes based upon the DOF of the robot
  m_rdRes *= Cfg::GetNumOfJoints();
#endif
}

//test whether input configuration satisfies joint constraints  (i.e., is
//inside of C-Space) and lies inside of the workspace boundary (i.e., the
//robot at that configuration is inside of the workspace).
bool
Environment::InBounds(const Cfg& _cfg, shared_ptr<Boundary> _b){
  if(InCSpace(_cfg, _b))
    if(InWSpace(_cfg, _b))
      return true;
  return false;
}

//access the possible range of values for the _i th DOF
pair<double, double>
Environment::GetRange(size_t _i, shared_ptr<Boundary> _b){
  size_t index = 0;
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots.begin(); rit != m_robots.end(); rit++) {
    if(rit->m_base != Robot::FIXED) {
      if(_i == index++) return _b->GetRange(0);
      if(_i == index++) return _b->GetRange(1);
      if(rit->m_base == Robot::VOLUMETRIC) {
        if(_i == index++) return _b->GetRange(2);
      }
      if(rit->m_baseMovement == Robot::ROTATIONAL) {
        if(rit->m_base == Robot::PLANAR) {
          if(_i == index++) return make_pair(-1, 1);
        }
        else {
          if(_i == index++) return make_pair(-1, 1);
          if(_i == index++) return make_pair(-1, 1);
          if(_i == index++) return make_pair(-1, 1);
        }
      }
    }
    typedef Robot::JointMap::iterator MIT;
    for(MIT mit = rit->m_joints.begin(); mit != rit->m_joints.end(); mit++) {
      if((*mit)->GetConnectionType() != Connection::NONACTUATED) {
        if(_i == index++) return (*mit)->GetJointLimits(0);
        if((*mit)->GetConnectionType() == Connection::SPHERICAL){
          if(_i == index++) return (*mit)->GetJointLimits(1);
        }
      } 
    }
  }
  return make_pair(0,0);
}

//reset the boundary to the minimum bounding box surrounding the obstacles
//increased by a margin of _d + robotRadius
void
Environment::ResetBoundary(double _d, size_t _robotIndex){

  double minx, miny, minz, maxx, maxy, maxz;
  minx = miny = minz = numeric_limits<double>::max();
  maxx = maxy = maxz = -numeric_limits<double>::max();

  double robotRadius = GetMultiBody(_robotIndex)->GetBoundingSphereRadius();
  _d += robotRadius;


  for(size_t i=0; i<m_obstacleBodies.size(); i++) {
    m_obstacleBodies[i]->FindBoundingBox();
    const double* tmp = m_obstacleBodies[i]->GetBoundingBox();
    minx = min(minx, tmp[0]);  maxx = max(maxx, tmp[1]);
    miny = min(miny, tmp[2]);  maxy = max(maxy, tmp[3]);
    minz = min(minz, tmp[4]);  maxz = max(maxz, tmp[5]);
  }

  vector<pair<double, double> > obstBBX(3);
  obstBBX[0].first = minx;
  obstBBX[0].second = maxx;
  obstBBX[1].first = miny;
  obstBBX[1].second = maxy;
  obstBBX[2].first = minz;
  obstBBX[2].second = maxz;

  m_boundary->ResetBoundary(obstBBX, _d);
}

shared_ptr<MultiBody>
Environment::GetRandomObstacle() const{
  if(!m_obstacleBodies.size()){
    cerr << "Environment::GetRandomObstacle error - no usable obstacles." << endl;
    exit(1);
  }

  size_t rIndex = LRand() % m_obstacleBodies.size();
  return m_obstacleBodies[rIndex];
}

//------------------------------------------------------------------
//  GetRandomNavigableSurfaceIndex
//  Output: An index between -1 and m_navigableSurfaces.size()-1
//          -1 means base index 
//------------------------------------------------------------------
size_t
Environment::GetRandomNavigableSurfaceIndex()  {
  size_t numSurfaces = GetNavigableSurfacesCount();
  size_t rindex = (LRand() % (numSurfaces+1)) - 1;
  return rindex;
}

int
Environment::AddObstacle(string _modelFileName, const vector<double>& _where, const vector<cd_predefined>& _cdTypes){
  shared_ptr<MultiBody> mb(new MultiBody());
  
  mb->Initialize(_modelFileName, _where);

  for(vector<cd_predefined>::const_iterator cdIter = _cdTypes.begin(); cdIter != _cdTypes.end(); ++cdIter)
    mb->buildCDstructure(*cdIter);

  m_obstacleBodies.push_back(mb);
  m_usableMultiBodies.push_back(mb);
  
  return m_obstacleBodies.size()-1;
}

void Environment::RemoveObstacleAt(size_t position){
  if (position < m_obstacleBodies.size()){
    shared_ptr<MultiBody> mb = m_obstacleBodies.at(position);

    m_obstacleBodies.erase(m_obstacleBodies.begin()+position);
    //try to find mb in usableMultiBodies
    vector<shared_ptr<MultiBody> >::iterator vecIter;
    for(vecIter = m_usableMultiBodies.end()-1; 
        vecIter != m_usableMultiBodies.begin()-1 && !(*vecIter==mb); --vecIter);

    if(*vecIter == mb)
      m_usableMultiBodies.erase(vecIter);
  }
  else {
    cerr << "Environment::RemoveObstacleAt Warning: unable to remove obst at position " << position << endl;
  }
}

void
Environment::BuildCDstructure(cd_predefined cdtype){
  for(vector<shared_ptr<MultiBody> >::iterator M = m_activeBodies.begin(); M != m_activeBodies.end(); ++M)
    (*M)->buildCDstructure(cdtype);

  for(vector<shared_ptr<MultiBody> >::iterator M = m_obstacleBodies.begin(); M != m_obstacleBodies.end(); ++M)
    (*M)->buildCDstructure(cdtype);
}

void 
Environment::ReadBoundary(istream& _is){
  string bndry = ReadFieldString(_is, "Boundary tag.");
  if(bndry != "BOUNDARY") {
    cerr << "Error reading environment. First item should be boundary." << endl;
    exit(1);
  }

  string btype = ReadFieldString(_is, "Boundary type.");
  if(btype == "BOX") {
    m_boundary = shared_ptr<BoundingBox>(new BoundingBox());

  }
  else if(btype == "SPHERE") {
    m_boundary = shared_ptr<BoundingSphere>(new BoundingSphere());
  }
  else {
    cerr << "Error::Unrecognized boundary type::" << btype << endl;
    exit(1);
  }

  m_boundary->Read(_is);

  cout << "Boundary::" << *m_boundary;
}

//BuildRobotStructure, builds a robot graph which determines DOFs for a given robot
//In an environment with multiple active bodies, for now this function will assume they all have the same DOFs
//until PMPL is changed later to support multiple roadmaps for heterogeneous systems. That is, this function assumes
//that if there is a multiagent sim going on, the agents are homogenous
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
  m_robotGraph = RobotGraph();
  for (int i = 0; i < fixedBodyCount; i++) {
    m_robotGraph.add_vertex(i);
  }
  for (int i = 0; i < freeBodyCount; i++) {
    m_robotGraph.add_vertex(i + fixedBodyCount); //Need to account for FixedBodies added above
  }
  //Total amount of bodies in environment: free + fixed
  for (int i = 0; i < freeBodyCount + fixedBodyCount; i++){
    shared_ptr<Body> body = robot->GetBody(i);  
    //For each body, find forward connections and connect them 
    for (int j = 0; j < body->ForwardConnectionCount(); j++) {
      shared_ptr<Body> forward = body->GetForwardConnection(j).GetNextBody();
      if (forward->IsFixedBody()) {
        //Quick hack to avoid programming ability to determine subclass
        shared_ptr<FixedBody> castFixedBody = boost::dynamic_pointer_cast<FixedBody>(forward);
        int nextIndex = robot->GetFixedBodyIndex(castFixedBody);
        m_robotGraph.add_edge(i, nextIndex);
      }
      else {
        shared_ptr<FreeBody> castFreeBody = boost::dynamic_pointer_cast<FreeBody>(forward);
        int nextIndex = robot->GetFreeBodyIndex(castFreeBody);
        m_robotGraph.add_edge(i, nextIndex);
      }
    } 
  }

  //Robot ID typedef
  typedef RobotGraph::vertex_descriptor RID; 
  vector<pair<size_t, RID> > ccs;
  stapl::sequential::vector_property_map<RobotGraph, size_t> cmap;
  //Initialize CC information
  get_cc_stats(m_robotGraph, cmap, ccs);
  if(ccs.size()>1)
    robot->SetMultirobot(true);
  m_robots.clear();
  for(size_t i = 0; i < ccs.size(); i++) {
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
        if((*mit)->GetPreviousBodyIndex() == index){
          jm.push_back(*mit);
        }
      }
    }
    m_robots.push_back(Robot(bt, bm, jm, baseIndx));
  }

  Cfg::InitRobots(m_robots);
}

bool 
Environment::InCSpace(const Cfg& _cfg, shared_ptr<Boundary> _b){
  size_t index = 0;
  typedef vector<Robot>::iterator RIT;
  for(RIT rit = m_robots.begin(); rit != m_robots.end(); rit++) {
    if(rit->m_base != Robot::FIXED) {
      Vector3D p;
      p[0] = _cfg[index];
      p[1] = _cfg[index+1];
      index+=2;
      if(rit->m_base == Robot::VOLUMETRIC) {
        p[2] = _cfg[index];
        index++;
      }
      if(!_b->InBoundary(p))
        return false;
      if(rit->m_baseMovement == Robot::ROTATIONAL) {
        if(rit->m_base == Robot::PLANAR) {
          if(fabs(_cfg[index]) > 1)
            return false;
          index++;
        }
        else {
          for(size_t i = 0; i<3; ++i){
            if(fabs(_cfg[index]) > 1)
              return false;
            index++;
          }
        }
      }
    }
    typedef Robot::JointMap::iterator MIT;
    for(MIT mit = rit->m_joints.begin(); mit != rit->m_joints.end(); mit++) {
      if((*mit)->GetConnectionType() != Connection::NONACTUATED) {
        if(_cfg[index] < (*mit)->GetJointLimits(0).first || _cfg[index] > (*mit)->GetJointLimits(0).second)
          return false;
        index++;
        if((*mit)->GetConnectionType() == Connection::SPHERICAL){
          if(_cfg[index] < (*mit)->GetJointLimits(1).first || _cfg[index] > (*mit)->GetJointLimits(1).second)
            return false;
          index++;
        }
      } 
    }
  }
  return true;
}

bool 
Environment::InWSpace(const Cfg& _cfg, shared_ptr<Boundary> _b){

  shared_ptr<MultiBody> robot = GetMultiBody(_cfg.GetRobotIndex());

  if(_b->GetClearance(_cfg.GetRobotCenterPosition()) < robot->GetBoundingSphereRadius()) { //faster, loose check
    // Robot is close to wall, have a strict check.
    _cfg.ConfigEnvironment(this); // Config the robot in the environment.

    //check each part of the robot multibody for being inside of the boundary
    for(int m=0; m<robot->GetFreeBodyCount(); ++m) {

      typedef vector<Vector3D>::const_iterator VIT;

      Transformation& worldTransformation = robot->GetFreeBody(m)->WorldTransformation();

      //first check just the boundary of the polyhedron
      GMSPolyhedron &bbPoly = robot->GetFreeBody(m)->GetBoundingBoxPolyhedron();
      bool bcheck = true;
      for(VIT v = bbPoly.m_vertexList.begin(); v != bbPoly.m_vertexList.end(); ++v){
        if(!_b->InBoundary(worldTransformation * (*v))) {
          bcheck = false;
          break;
        }
      }

      //boundary of polyhedron is inside the boundary thus the whole geometry is
      if(bcheck) 
        continue;

      //the boundary intersected. Now check the geometry itself.
      GMSPolyhedron &poly = robot->GetFreeBody(m)->GetPolyhedron();
      for(VIT v = poly.m_vertexList.begin(); v != poly.m_vertexList.end(); ++v)
        if(!_b->InBoundary(worldTransformation * (*v)))
          return false;      
    }
  }
  return true; 
}
