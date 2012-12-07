#ifndef MEDIALAXISUTILITY_H_
#define MEDIALAXISUTILITY_H_

#include "MPUtils.h"
#include "MetricUtils.h"

struct ClearanceStats{
  double m_avgClearance;
  double m_minClearance;
  double m_clearanceVariance;
  double m_pathLength;
};

//used to encapsulate all the fields and functions necessary for clearance and penetration calculations
template<class MPTraits>
class ClearanceUtility : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    
    ClearanceUtility(MPProblemType* _problem = NULL,
        string _vcLabel = "", string _dmLabel = "",
        bool _exactClearance = false, bool _exactPenetration = false,
        size_t _clearanceRays = 10, size_t _penetrationRays = 10,
        bool _useBBX = true, bool _positional = true, bool _debug = false);

    ClearanceUtility(MPProblemType* _problem, XMLNodeReader& _node);

    void ParseXML(XMLNodeReader& _node);

    virtual void PrintOptions(ostream& _os) const;

    //*********************************************************************//
    // Calculate Collision Information                                     //
    //   This is a wrapper function for getting the collision information  //
    // for the medial axis computation, calls either approx or exact       //
    //*********************************************************************//
    bool CollisionInfo(CfgType& _cfg, CfgType& _clrCfg, shared_ptr<Boundary> _bb, CDInfo& _cdInfo); 

    //*********************************************************************//
    // Get Exact Collision Information Function                            //
    //   Determines the exact collision information by taking the validity //
    // checker results against obstacles to the bounding box to get a      //
    // complete solution                                                   //
    //*********************************************************************//
    bool ExactCollisionInfo(CfgType& _cfg, shared_ptr<Boundary> _bb, CDInfo& _cdInfo);

    //*********************************************************************//
    // Get Approximate Collision Information Function                      //
    //   Calculate the approximate clearance using a series of rays. The   //
    // specified number of rays are sent out till they change in validity. //
    // The shortest ray is then considered the best calididate.            //
    //*********************************************************************//
    bool ApproxCollisionInfo(CfgType& _cfg, CfgType& _clrCfg, shared_ptr<Boundary> _bb, CDInfo& _cdInfo);

    //*********************************************************************//
    // Get Roadmap Clearance Statistics                                    //
    //   Calculates averages mins and maxes for clearance across roadmaps  //
    // paths and edges                                                     //
    //*********************************************************************//
    ClearanceStats RoadmapClearance();
    
    ClearanceStats PathClearance(VID _startVID, VID _goalVID);  

    double MinEdgeClearance(const CfgType& _c1, const CfgType& _c2, const WeightType& _weight);

  protected:
    string m_vcLabel;                //validity checker method label
    string m_dmLabel;                //distance metric method label
    bool m_exactClearance;           //use exact clearance calculations
    bool m_exactPenetration;         //use exact penetration calculations
    size_t m_clearanceRays;          //number of rays used to approximate clearance
    size_t m_penetrationRays;        //number of rays used to approximate penetration
    bool m_useBBX;                   //use bounding box as obstacle
    bool m_positional;               //use only positional dofs
};

template<class MPTraits>
class MedialAxisUtility : ClearanceUtility<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    
    MedialAxisUtility(MPProblemType* _problem = NULL,
        string _vcLabel = "", string _dmLabel = "",
        bool _exactClearance = false, bool _exactPenetration = false,
        size_t _clearanceRays = 10, size_t _penetrationRays = 10,
        bool _useBBX = true, bool _positional = true, bool _debug = false, 
        double _epsilon = 0.1, size_t _historyLength = 5);

    MedialAxisUtility(MPProblemType* _problem, XMLNodeReader& _node);

    void ParseXML(XMLNodeReader& _node);

    virtual void PrintOptions(ostream& _os) const;

    //***********************************//
    // Main Push To Medial Axis Function //
    //***********************************//
    bool PushToMedialAxis(CfgType& _cfg, shared_ptr<Boundary> _bb);

    //***************************************************************//
    // Push From Inside Obstacle                                     //
    //   In this function, a cfg is known to be inside an obstacle.  //
    // A direction is determined to move the cfg outside of the      //
    // obstacle and is pushed till outside.                          //
    //***************************************************************//
    bool PushFromInsideObstacle(CfgType& _cfg, shared_ptr<Boundary> _bb);

    //***************************************************************//
    // Push Cfg To Medial Axis                                       //
    //   This function is to perform a regular push to medial axis   //
    // algorithm stepping out at the resolution till the medial axis //
    // is found, determined by the clearance.                        //
    //***************************************************************//
    bool PushCfgToMedialAxis(CfgType& _cfg, shared_ptr<Boundary> _bb);

  private:
    double m_epsilon;
    size_t m_historyLength;
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Clearance Utility
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template<class MPTraits>
ClearanceUtility<MPTraits>::ClearanceUtility(MPProblemType* _problem,
    string _vcLabel, string _dmLabel,
    bool _exactClearance, bool _exactPenetration,
    size_t _clearanceRays, size_t _penetrationRays,
    bool _useBBX, bool _positional, bool _debug):
  m_vcLabel(_vcLabel), m_dmLabel(_dmLabel),
  m_exactClearance(_exactClearance), m_exactPenetration(_exactPenetration),
  m_clearanceRays(_clearanceRays), m_penetrationRays(_penetrationRays),
  m_useBBX(_useBBX), m_positional(_positional){
    this->m_name = "ClearanceUtility";
    this->SetMPProblem(_problem);
    this->m_debug = _debug;
  }

template<class MPTraits>
ClearanceUtility<MPTraits>::ClearanceUtility(MPProblemType* _problem, XMLNodeReader& _node):
  MPBaseObject<MPTraits>(_problem, _node){
    this->m_name = "ClearanceUtility";
    ParseXML(_node);
  }

template<class MPTraits>
void
ClearanceUtility<MPTraits>::ParseXML(XMLNodeReader& _node){
  m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");
  m_dmLabel = _node.stringXMLParameter("dmLabel", true, "", "Distance metric");
  
  //clearance and penetration types
  string clearanceType = _node.stringXMLParameter("clearanceType", true, "", "Clearance Computation (exact or approx)");
  m_exactClearance = clearanceType.compare("exact")==0;
  string penetrationType = _node.stringXMLParameter("penetrationType",true, "", "Penetration Computation (exact or approx)");
  m_exactPenetration = penetrationType.compare("exact")==0;
  
  //if approximate calculations require number of rays to be defined
  m_clearanceRays = _node.numberXMLParameter("clearanceRays", !m_exactClearance, 10, 1, 1000, "Number of Clearance Rays");
  m_penetrationRays = _node.numberXMLParameter("penetrationRays", !m_exactPenetration, 10, 1, 1000, "Number of Penetration Rays");
  
  m_useBBX = _node.boolXMLParameter("useBBX", false, true, "Use the Bounding Box as an Obstacle");
  m_positional = _node.boolXMLParameter("positional", false, true, "Use only positional DOFs");
}

template<class MPTraits>
void
ClearanceUtility<MPTraits>::PrintOptions(ostream& _os) const{
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tdmLabel = " << m_dmLabel << endl;
  _os << "\tuseBBX = " << m_useBBX << endl;
  _os << "\tclearance = ";
  _os << ((m_exactClearance) ? "exact, " : "approx, ");
  if(!m_exactClearance) _os << m_clearanceRays << " rays";
  _os << endl;
  _os << "\tpenetration = ";
  _os << ((m_exactPenetration) ? "exact, " : "approx, ");
  if(!m_exactPenetration) _os << m_penetrationRays << " rays";
  _os << endl;
}

//*********************************************************************//
// Calculate Collision Information                                     //
//   This is a wrapper function for getting the collision information  //
// for the medial axis computation, calls either approx or exact       //
//*********************************************************************//
template<class MPTraits>
bool
ClearanceUtility<MPTraits>::CollisionInfo(CfgType& _cfg, CfgType& _clrCfg, shared_ptr<Boundary> _bb, CDInfo& _cdInfo){ 
  if(m_exactClearance) 
    return ExactCollisionInfo(_cfg, _bb, _cdInfo);
  else
    return ApproxCollisionInfo(_cfg, _clrCfg, _bb, _cdInfo);
}

//*********************************************************************//
// Get Exact Collision Information Function                            //
//   Determines the exact collision information by taking the validity //
// checker results against obstacles to the bounding box to get a      //
// complete solution                                                   //
//*********************************************************************//
template<class MPTraits>
bool
ClearanceUtility<MPTraits>::ExactCollisionInfo(CfgType& _cfg, shared_ptr<Boundary> _bb, CDInfo& _cdInfo){
  // ClearanceUtility variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  // Setup Validity Checker
  string call = this->GetName() + "::ExactCollisionInfo";
  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  // If not in BBX or valid, return false (IsValid gets _cdInfo)
  if(!_cfg.InBoundary(env, _bb) || !(vcm->IsValid(_cfg, env, *stats, _cdInfo, &call))) 
    return false;

  // If not using the bbx, done
  if(!m_useBBX)
    return true;

  // CfgType is now know as good, get BBX and ROBOT info
  boost::shared_ptr<MultiBody> robot = env->GetMultiBody(env->GetRobotIndex());
  std::pair<double,double> bbxRange;

  // Find closest point between robot and bbx, set if less than min dist from obstacles
  for(int m=0; m < robot->GetFreeBodyCount(); ++m) {
    GMSPolyhedron &poly = robot->GetFreeBody(m)->GetWorldPolyhedron();
    for(size_t j = 0; j < poly.m_vertexList.size(); ++j){
      for(size_t k=0; k<_cfg.PosDOF(); ++k) { // For all positional DOFs
        bbxRange = _bb->GetRange(k);
        if((poly.m_vertexList[j][k] - bbxRange.first) < _cdInfo.m_minDist) {
          for(size_t l=0; l<_cfg.PosDOF(); ++l) {// Save new closest point
            _cdInfo.m_robotPoint[l]  = poly.m_vertexList[j][l];
            _cdInfo.m_objectPoint[l] = poly.m_vertexList[j][l];
          }
          _cdInfo.m_objectPoint[k] = bbxRange.first; // Lower Bound
          _cdInfo.m_minDist = poly.m_vertexList[j][k]-bbxRange.first;
          _cdInfo.m_nearestObstIndex = -(k*2);
        }
        if((bbxRange.second - poly.m_vertexList[j][k]) < _cdInfo.m_minDist) {
          for(size_t l=0; l<_cfg.PosDOF(); ++l) {// Save new closest point
            _cdInfo.m_robotPoint[l]  = poly.m_vertexList[j][l];
            _cdInfo.m_objectPoint[l] = poly.m_vertexList[j][l];
          }
          _cdInfo.m_objectPoint[k] = bbxRange.second; // Upper Bound
          _cdInfo.m_minDist = bbxRange.second-poly.m_vertexList[j][k];
          _cdInfo.m_nearestObstIndex = -(k*2+1);
        }
      }
    }
  }
  return (_cdInfo.m_minDist>=0) ? true : false;
}

//*********************************************************************//
// Get Approximate Collision Information Function                      //
//   Calculate the approximate clearance using a series of rays. The   //
// specified number of rays are sent out till they change in validity. //
// The shortest ray is then considered the best calididate.            //
//*********************************************************************//
template<class MPTraits>
bool
ClearanceUtility<MPTraits>::ApproxCollisionInfo(CfgType& _cfg, CfgType& _clrCfg, shared_ptr<Boundary> _bb, CDInfo& _cdInfo){
  // ClearanceUtility variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  // Initialization
  string call = this->GetName() + "::ApproxCollisionInfo";
  DistanceMetricPointer dm  = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(m_vcLabel);


  // Calculate MaxRange for dist calc
  double maxRange(0.0);
  for(size_t i=0; i< _cfg.PosDOF(); ++i) {
    std::pair<double,double> range = env->GetBoundary()->GetRange(i);
    double tmpRange = range.second-range.first;
    if(tmpRange > maxRange) 
      maxRange = tmpRange;
  }

  // If in BBX, check validity to get _cdInfo, return false if not valid
  if(!_cfg.InBoundary(env,_bb))
    return false;

  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;	

  bool initInside = vcm->IsInsideObstacle(_cfg, env, _cdInfo);             // Initially Inside Obst
  bool initValidity = vcm->IsValid(_cfg, env, *stats, _cdInfo, &call); // Initial Validity
  initValidity = initValidity && !initInside;

  // Setup Major Variables and Constants:
  CDInfo tmpInfo;
  Vector3D dif;
  size_t numRays;
  double posRes = env->GetPositionRes(), oriRes = env->GetOrientationRes();

  // Generate 'numRays' random directions at a distance 'dist' away from 'cfg'
  if(initValidity) 
    numRays = (initInside) ? m_penetrationRays : m_clearanceRays;
  else                 
    numRays = m_penetrationRays;    

  vector<CfgType> directions, candIn, tick, incr;
  typedef typename vector<CfgType>::iterator CIT;
  vector<size_t> candTick;
  int nTicks;
  candIn.clear();

  // Setup translation rays
  for(size_t i=0; i<numRays; ++i) {
    CfgType tmp1 = _cfg;
    tick.push_back(tmp1);
    CfgType tmp2;
    tmp2.GetRandomRay(posRes/maxRange, env, dm, false);
    if(m_positional) { // Use only positional dofs
      double factor=0.0;
      for(size_t j=0; j<tmp2.DOF(); j++) {
        if(j < tmp2.PosDOF())
          factor += pow(tmp2.GetSingleParam(j), 2);
        else 
          tmp2.SetSingleParam(j, 0.0);
      }
      tmp2.multiply(tmp2, posRes/sqrt(factor), false);
    }
    tmp2.add(tmp1, tmp2);
    CfgType tmp3;
    tmp3.FindIncrement(tmp1, tmp2, &nTicks, posRes, oriRes);
    for(size_t j = tmp3.PosDOF(); j < tmp3.DOF(); ++j) {
      if(tmp3.GetSingleParam(j) > 0.5)
        tmp3.SetSingleParam(j, tmp3.GetSingleParam(j) - 1.0, false);
    }
    incr.push_back(tmp3);
  }

  // Setup to step out along each direction:
  bool stateChangedFlag = false, currValidity, currInside, currInBBX;
  size_t lastLapIndex = -1;
  size_t iterations = 0, maxIterations=10000; // Arbitrary TODO: Smarter maxIter number

  // Shoot out each ray to determine the candidates
  while(!stateChangedFlag && iterations < maxIterations) {
    iterations++;
    // For Each Ray
    size_t i = 0;
    for(CIT incrIT = incr.begin(), tickIT = tick.begin(); 
        incrIT!=incr.end() || tickIT!=tick.end(); ++incrIT, ++tickIT) {
      tickIT->Increment(*incrIT);
      currInside = vcm->IsInsideObstacle(*tickIT, env, tmpInfo);
      currValidity = vcm->IsValid(*tickIT, env, *stats, tmpInfo, &call);
      currInBBX = tickIT->InBoundary(env, _bb);
      currValidity = currValidity && !currInside;

      if(m_useBBX) 
        currValidity = (currValidity && currInBBX);

      if(currValidity != initValidity) { // If Validity State has changed
        if(lastLapIndex != i) {
          CfgType candI;
          candI.negative(*incrIT);
          candI.add(*tickIT, candI);
          candIn.push_back(candI);
          candTick.push_back(i);
        } 
        if(lastLapIndex == (size_t)-1)  // Set lastLapIndex to first ray changing state
          lastLapIndex = (i==0)?(incr.size()-1):(i-1);
      }
      if(lastLapIndex == i){
        stateChangedFlag = true; // lastLapIndex == i, made full pass
        break; // Once validity changes, exit loop
      }
      i++;
    } // End for
  } // End while

  if(candIn.size() == 0)
    return false;

  double low=0.0, mid=0.5, high=1.0;
  CfgType middleCfg;
  bool midInside, midValidity, midInBBX, foundOne;
  vector<bool> remove;

  // Binary search on candidates to find the best result
  while(candIn.size() > 1) {
    mid=(low+high)/2.0;
    remove.clear();
    foundOne=false;
    for(size_t i=0; i<candIn.size(); ++i) {
      middleCfg.multiply(incr[candTick[i]], mid);
      middleCfg.add(candIn[i], middleCfg);
      midInside = vcm->IsInsideObstacle(middleCfg,env,tmpInfo);
      midValidity = vcm->IsValid(middleCfg, env, *stats, tmpInfo, &call);
      midInBBX = middleCfg.InBoundary(env,_bb);
      midValidity = midValidity && !midInside;
      if(m_useBBX) 
        midValidity = (midValidity && midInBBX);

      if(midValidity != initValidity) { // If Validity State has changed
        remove.push_back(false);
        foundOne = true;
      } 
      else {
        remove.push_back(true);
      }
    }

    // If at least one good candidate is found, remove all known bad ones
    if(foundOne) {
      size_t offset=0;
      for(size_t i=0; i<remove.size(); ++i) {
        if(remove[i]) {
          candIn.erase(candIn.begin() + (i-offset));
          candTick.erase(candTick.begin() + (i-offset));
          offset++;
        }
      }
      high=mid;
    } 
    else {
      low=mid;
    }
  }

  if(initValidity) { // Low may be initial cfg so keep looking
    while(low == 0.0) {
      mid=(low+high)/2.0;
      middleCfg.multiply(incr[candTick[0]],mid);
      middleCfg.add(candIn[0], middleCfg);

      midInside = vcm->IsInsideObstacle(middleCfg, env, tmpInfo);
      midValidity = vcm->IsValid(middleCfg, env, *stats, tmpInfo, &call);
      midInBBX = middleCfg.InBoundary(env,_bb);
      midValidity = midValidity && !midInside;
      if(m_useBBX) 
        midValidity = (midValidity && midInBBX);
      if(midValidity != initValidity) // If Validity State has changed
        high=mid;
      else
        low=mid;
    }
    mid=low;
  } 
  else { // Pushed out, high is all we need
    mid=high;
  }
  middleCfg.multiply(incr[candTick[0]], mid);
  middleCfg.add(candIn[0], middleCfg);
  _clrCfg = middleCfg;
  _cdInfo.m_minDist = dm->Distance(env, middleCfg, _cfg);

  return (_cdInfo.m_minDist < 0) ? false : true;
}

//*********************************************************************//
// Get Roadmap Clearance Statistics                                    //
//   Calculates averages mins and maxes for clearance across roadmaps  //
// paths and edges                                                     //
//*********************************************************************//
template<class MPTraits>
ClearanceStats
ClearanceUtility<MPTraits>::RoadmapClearance(){
  ClearanceStats output;
  ///temp until edge iterator & operator != in new stapl dynamic graph is fixed
  double minClearance = 1e6;
  double runningTotal = 0;
  GraphType* g = this->GetMPProblem()->GetRoadmap()->GetGraph();
  vector<double> clearanceVec;
  for(typename GraphType::edge_iterator it = g->edges_begin(); it != g->edges_end(); it++){
    double currentClearance = MinEdgeClearance(g->GetCfg((*it).source()), g->GetCfg((*it).target()), (*it).property());
    clearanceVec.push_back(currentClearance);//Save this value for variance computation later
    runningTotal+=currentClearance;
    if(currentClearance < minClearance){//Did we find a new minimum clearance value?
      minClearance = currentClearance;
    }
  }
  output.m_minClearance = minClearance;
  double average = runningTotal / g->get_num_edges();
  output.m_avgClearance = average;
  double varSum = 0;
  for(vector<double>::iterator it = clearanceVec.begin(); it != clearanceVec.end(); it++){
    varSum += pow(((*it) - average), 2);
  }
  output.m_clearanceVariance = varSum / clearanceVec.size();
  return output;
}

template<class MPTraits>
ClearanceStats 
ClearanceUtility<MPTraits>::PathClearance(VID _startVID, VID _goalVID){
  GraphType* g = this->GetMPProblem()->GetRoadmap()->GetGraph();
  vector<VID> path;
  find_path_dijkstra(*g, _startVID, _goalVID, path, WeightType::MaxWeight());
  ClearanceStats stats;
  typedef typename GraphType::EI EI;
  typedef typename GraphType::VI VI;
  typedef typename GraphType::EID EID;
  double runningTotal = 0;
  double minClearance = 1e6;
  double pathLength = 0;
  vector<double> clearanceVec;
  typedef typename vector<VID>::iterator VIT;
  for(VIT vit = path.begin(); (vit+1)!=path.end(); ++vit){
    EI ei;
    VI vi;
    EID ed(*vit, *(vit+1));
    g->find_edge(ed, vi, ei);
    WeightType weight = (*ei).property();
    pathLength += weight.Weight();
    double currentClearance = MinEdgeClearance(g->GetCfg((*ei).source()), g->GetCfg((*ei).target()), weight); 
    clearanceVec.push_back(currentClearance);
    runningTotal += currentClearance;
    if(currentClearance < minClearance){
      minClearance = currentClearance;    
    }
  }
  stats.m_minClearance = minClearance;
  double average = runningTotal / (path.size()/2);
  stats.m_avgClearance = average;
  double varSum = 0;
  for(vector<double>::iterator it = clearanceVec.begin(); it != clearanceVec.end(); it++){
    varSum+=pow(((*it) - average), 2);  
  }
  stats.m_clearanceVariance = varSum / clearanceVec.size();
  stats.m_pathLength = pathLength;
  return stats;
}

template<class MPTraits>
double
ClearanceUtility<MPTraits>::MinEdgeClearance(const CfgType& _c1, const CfgType& _c2, const WeightType& _weight){
  Environment* env = this->GetMPProblem()->GetEnvironment();
  double minClearance = 1e6;
  //Reconstruct the path given the two nodes
  vector<CfgType> intermediates = _weight.GetIntermediates();
  vector<CfgType> reconEdge = this->GetMPProblem()->GetLocalPlanner(_weight.GetLPLabel())->
    ReconstructPath(env, this->GetMPProblem()->GetDistanceMetric(m_dmLabel), _c1, _c2, intermediates, 
        env->GetPositionRes(), env->GetOrientationRes());
  typedef typename vector<CfgType>::iterator CIT;
  for(CIT it = reconEdge.begin(); it != reconEdge.end(); ++it){
    CDInfo collInfo;
    CfgType clrCfg;
    //Decide which collision info function to use
    CollisionInfo(*it, clrCfg, env->GetBoundary(), collInfo);
    double currentClearance = collInfo.m_minDist;
    //If newly computed clearance is less than the previous minimum, it becomes the minimum
    if(currentClearance < minClearance){
      minClearance = currentClearance;
    }
  }
  return minClearance;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Medial Axis Utility
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template<class MPTraits>
MedialAxisUtility<MPTraits>::MedialAxisUtility(MPProblemType* _problem,
    string _vcLabel, string _dmLabel,
    bool _exactClearance, bool _exactPenetration,
    size_t _clearanceRays, size_t _penetrationRays,
    bool _useBBX, bool _positional, bool _debug,
    double _epsilon, size_t _historyLength) :
  ClearanceUtility<MPTraits>(_problem, _vcLabel, _dmLabel, 
      _exactClearance, _exactPenetration, _clearanceRays, _penetrationRays, 
      _useBBX, _positional, _debug),
  m_epsilon(_epsilon), m_historyLength(_historyLength){
    this->m_name = "MedialAxisUtility";
  }

template<class MPTraits>
MedialAxisUtility<MPTraits>::MedialAxisUtility(MPProblemType* _problem, XMLNodeReader& _node):
  ClearanceUtility<MPTraits>(_problem, _node){
    this->m_name = "MedialAxisUtility";
    ParseXML(_node);
  }

template<class MPTraits>
void
MedialAxisUtility<MPTraits>::ParseXML(XMLNodeReader& _node){
  m_epsilon = _node.numberXMLParameter("epsilon", false, 0.1, 0.0, 1.0, "Epsilon-Close to the MA (fraction of the resolution)");
  m_historyLength = _node.numberXMLParameter("historyLength", false, 5, 3, 100,"History Length");
}

template<class MPTraits>
void
MedialAxisUtility<MPTraits>::PrintOptions(ostream& _os) const{
  ClearanceUtility<MPTraits>::PrintOptions(_os);
  _os << "\tepsilon::" << m_epsilon << endl;
  _os << "\thistory length::" << m_historyLength << endl;
}

//***********************************//
// Main Push To Medial Axis Function //
//***********************************//
template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::PushToMedialAxis(CfgType& _cfg, shared_ptr<Boundary> _bb){
  // Initialization
  string call = this->GetName() + "::PushToMedialAxis";
  
  if(this->m_debug)
    cout << endl << call << endl << "Being Pushed: " << _cfg;
 
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(this->m_vcLabel);
  bool inside, inCollision, found, pushed = true;
  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  // If invalid, push to the outside of the obstacle
  inside = vcm->IsInsideObstacle(_cfg, env, tmpInfo);
  inCollision = !(vcm->IsValid(_cfg, env, *stats, tmpInfo, &call));
  
  if(this->m_debug) cout << " Inside/In-Collision: " << inside << "/" << inCollision << endl;
  
  if(inside || inCollision)
    pushed = PushFromInsideObstacle(_cfg, _bb);
  
  if(!pushed) 
    return false;

  // Cfg is free, find medial axis
  found = PushCfgToMedialAxis(_cfg, _bb);
  if(!found) { 
    if(this->m_debug) cout << "Not found!! ERR in pushtomedialaxis" << endl; 
    return false; 
  }

  if(this->m_debug) cout << "FINAL CfgType: " << _cfg << endl << call << "::END" << endl << endl;
  
  return true;
}

//***************************************************************//
// Push From Inside Obstacle                                     //
//   In this function, a cfg is known to be inside an obstacle.  //
// A direction is determined to move the cfg outside of the      //
// obstacle and is pushed till outside.                          //
//***************************************************************//
template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::PushFromInsideObstacle(CfgType& _cfg, shared_ptr<Boundary> _bb){

  // Initialization
  string call = this->GetName() + "::PushFromInsideObstacle";
  
  if(this->m_debug) cout << call << endl << " CfgType: " << _cfg << endl;
  
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);
  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(this->m_vcLabel);

  // Variables
  CDInfo tmpInfo;
  Vector3D transDir, dif;
  bool inBBX;
  CfgType endCfg = _cfg, tmpCfg = _cfg, heldCfg = _cfg, transCfg = _cfg;
  double stepSize=1.0, factor;
  double posRes = env->GetPositionRes();
  double oriRes = env->GetOrientationRes(); 
  bool initValidity = vcm->IsValid(_cfg, env, *stats, tmpInfo, &call);
  bool tmpValidity=false, prevValidity=false;
  int nTicks;

  // If in collision (using the exact case), must use approx
  bool pExact = this->m_exactPenetration && initValidity; 

  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  //Determine direction to move
  if(this->CollisionInfo(_cfg, transCfg, _bb, tmpInfo)) {
    if(pExact) {
      Vector3D transDir = tmpInfo.m_objectPoint - tmpInfo.m_robotPoint;
      for(size_t i=0; i<transCfg.DOF(); i++) {
        if(i < transCfg.PosDOF()) {
          transCfg.SetSingleParam(i, transDir[i]);
          factor += pow(transDir[i],2);
        }
        else
          transCfg.SetSingleParam(i, 0.0);
      }
      transCfg.multiply(transCfg,posRes/sqrt(factor));
    }
    else {
      if(this->m_debug) cout << "CLEARANCE CFG: " << transCfg << endl;
      transCfg.FindIncrement(_cfg, transCfg, &nTicks, posRes, oriRes);
      for(size_t i=transCfg.PosDOF(); i<transCfg.DOF(); i++) {
        if(transCfg.GetSingleParam(i) > 0.5)
          transCfg.SetSingleParam(i, transCfg.GetSingleParam(i)-1.0, false);
      }
    }
    if(this->m_debug) cout << "TRANS CFG: " << transCfg << endl;
  } 
  else
    return false;

  // Check if valid and outside obstacle
  while(!tmpValidity) {
    tmpInfo.ResetVars();
    tmpInfo.m_retAllInfo = true;

    heldCfg = tmpCfg;
    tmpCfg.multiply(transCfg,stepSize);
    tmpCfg.add(_cfg, tmpCfg);

    tmpValidity = vcm->IsValid(tmpCfg, env, *stats, tmpInfo, &call);
    tmpValidity = tmpValidity && !vcm->IsInsideObstacle(tmpCfg, env, tmpInfo);
    if(tmpValidity) {
      tmpValidity = tmpValidity && prevValidity;
      if(!prevValidity) // Extra Step TODO: Test if necessary
        prevValidity = true;
    }
    inBBX = tmpCfg.InBoundary(env, _bb);
    if(!inBBX) { 
      if(this->m_debug) cout << "ERROR: Fell out of BBX, error out... " << endl;
      return false;
    }
    stepSize += 1.0;
  }
  _cfg = tmpCfg;
  if(this->m_debug) cout << "FINAL CfgType: " << _cfg << " steps: " << stepSize-1.0 << endl << call << "::END " << endl;
  return true;
}

//***************************************************************//
// Push Cfg To Medial Axis                                       //
//   This function is to perform a regular push to medial axis   //
// algorithm stepping out at the resolution till the medial axis //
// is found, determined by the clearance.                        //
//***************************************************************//
template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::PushCfgToMedialAxis(CfgType& _cfg, shared_ptr<Boundary> _bb){
  // Initialization
  string call = this->GetName() + "::PushCfgToMedialAxis";
  
  if(this->m_debug) cout << call << endl << "Cfg: " << _cfg << " eps: " << m_epsilon << endl;
  
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);
  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(this->m_vcLabel);
  shared_ptr<MultiBody> robot = env->GetMultiBody(env->GetRobotIndex());
  
  // Variables
  Vector3D transDir, dif;
  CDInfo tmpInfo, prevInfo;
  CfgType transCfg, tmpCfg, heldCfg, tmpTransCfg;
  double stepSize = 0.0, cbStepSize = 0.0, factor = 0.0, posRes = env->GetPositionRes(), oriRes = env->GetOrientationRes();
  bool inBBX = true, goodTmp = true, inside = vcm->IsInsideObstacle(_cfg, env, tmpInfo);
  int nTicks;

  // Should already be in free space
  if(inside) 
    return false;

  // tmpInfo and Origin Setup
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;
  tmpCfg = _cfg;

  // Determine positional direction to move
  if(this->CollisionInfo(_cfg, transCfg, _bb, tmpInfo)) {
    prevInfo = tmpInfo;
    if(this->m_exactClearance) { // Exact uses workspace witness point for transDir
      Vector3D transDir = tmpInfo.m_robotPoint - tmpInfo.m_objectPoint;
      for(size_t i=0; i<transCfg.DOF(); i++) {
        if(i < transCfg.PosDOF()) {
          transCfg.SetSingleParam(i, transDir[i]);
          factor += pow(transDir[i], 2);
        }
        else
          transCfg.SetSingleParam(i, 0.0);
      }
      transCfg.multiply(transCfg, posRes/sqrt(factor));
    }
    else { // Approx uses clearance CFG for transDir
      if(this->m_debug) cout << "CLEARANCE CFG: " << transCfg << endl;
      transCfg.FindIncrement(_cfg, transCfg, &nTicks, posRes, oriRes);
      transCfg.negative(transCfg);
      for(size_t i = transCfg.PosDOF(); i<transCfg.DOF(); i++) {
        if(transCfg.GetSingleParam(i) > 0.5)
          transCfg.SetSingleParam(i, transCfg.GetSingleParam(i) - 1.0, false);
      }
    }
    if(this->m_debug) cout << "TRANS CFG: " << transCfg << endl;
  }
  else
    return false;

  // Initialize temp Info
  size_t posCnt = 0, negCnt = 0, zroCnt = 0, stepsTaken, tcc = 0;
  bool peakFound = false, wpMoved = false, checkBack = false, fellOut = false;
  deque<double> segDists;
  deque<CfgType> segCfgs;
  double maxDist = 0.0, errEps = numeric_limits<double>::epsilon(); // Arbitrary Value for errEps

  // Determine gap for medial axis
  while(!peakFound && !wpMoved) {

    // Increment
    heldCfg = tmpCfg;
    tmpCfg.multiply(transCfg, checkBack ? cbStepSize : stepSize);
    tmpCfg.add(_cfg, tmpCfg);

    // Test for in BBX and inside obstacle
    inside = vcm->IsInsideObstacle(tmpCfg, env, tmpInfo);
    inBBX = tmpCfg.InBoundary(env, _bb);
    bool tmpVal = inside || !inBBX;
    if(this->m_debug) VDAddTempCfg(tmpCfg, tmpVal);
    if(this->m_debug) VDClearLastTemp();
    tcc++;

    // If inside obstacle or out of the bbx, step back
    if(inside || !inBBX) {
      if(!inBBX && !this->m_useBBX) 
        return false;
      if(segCfgs.size() > 0) {
        fellOut = true;
        break;
      }
      else
        return false;
    }
    else {
      tmpInfo.ResetVars();
      tmpInfo.m_retAllInfo = true;

      // If tmp is valid, move on to next step
      if(vcm->IsValid(tmpCfg, env, *stats, tmpInfo, &call)) {
        if(this->m_debug) cout << "TMP Cfg: " << tmpCfg;
        goodTmp = this->CollisionInfo(tmpCfg, tmpTransCfg, _bb, tmpInfo);
        if(goodTmp) {
          if(this->m_exactClearance) { // If exact, check witness points
            Vector3D transDir = tmpInfo.m_objectPoint - prevInfo.m_objectPoint;
            if(this->m_debug) cout << " obst: " << tmpInfo.m_nearestObstIndex;
            double tmpDist = 0.0;
            for(size_t i=0; i<transCfg.PosDOF(); i++)
              tmpDist += pow(transDir[i],2);
            tmpDist = sqrt(tmpDist);
            if(tmpDist > robot->GetBoundingSphereRadius()*2.0) { // TODO: might be better value
              if(this->m_debug) cout << " WP moved: " << tmpDist;
              peakFound = true; 
              wpMoved=true;
            }
          }
          prevInfo = tmpInfo;
          if(this->m_debug) cout << " clearance: " << tmpInfo.m_minDist;
          ++stepsTaken;

          // Update clearance history distances
          if(segDists.size() > m_historyLength) {
            segDists.erase(segDists.begin());
            segCfgs.erase(segCfgs.begin());
          }
          if(checkBack) {
            segDists.push_front(tmpInfo.m_minDist);
            segCfgs.push_front(tmpCfg);
          }
          else {
            segDists.push_back(tmpInfo.m_minDist);
            segCfgs.push_back(tmpCfg);
            checkBack=false;
          }
        }
        else { // Couldn't calculate clearance info
          if(this->m_debug) cout << "BROKE!" << endl;
          return false;
        }

        if(!peakFound) { // If no witness point success or approx, enumerate deltas to find peak
          maxDist = segDists[0]; posCnt=0; negCnt=0; zroCnt=0;
          for(size_t i=0; i<size_t(segDists.size())-1; i++) {
            double tmp = segDists[i+1] - segDists[i];
            if(tmp > errEps)
              ++posCnt;
            else if(tmp < -errEps)
              ++negCnt;
            else
              ++zroCnt;
            maxDist = (maxDist>segDists[i+1]) ? maxDist : segDists[i+1];
          }
          if(this->m_debug) cout << "  Pos/Zero/Neg counts: " << posCnt << "/" << zroCnt << "/" << negCnt << endl;
          if((negCnt > 0 && negCnt > posCnt) || (zroCnt > 0 && zroCnt > posCnt)) {
            if((posCnt < 1 && zroCnt < 1) || (posCnt < 1 && negCnt < 1)){ // Only see negatives or zeros, check back
              --cbStepSize;
              --stepSize;
              checkBack = true;
            }
            else {
              peakFound = true;
              if(this->m_debug) {
                cout << "Found peak!  Dists: ";
                for(size_t i=0; i<size_t(segDists.size()); i++)
                  cout << segDists[i] << " ";
                cout << endl;
              }
            }
          }
          else { // No peak found
            checkBack = false;
          }
        }
        // Increment step size
        if(!wpMoved && !peakFound)
          stepSize += 1.0;
      }
      else { // Else reduce step size or fall out of the loop
        return false;
      }
    }
  }

  // Check if there are enough cfgs
  if(segCfgs.size() < 2)
    return false;

  // Variables for modified binary search
  CfgType startCfg, midSCfg, midMCfg, midECfg, endingCfg;
  double gapDist, lBound, uBound, middle, middleS, middleE;
  size_t attempts = 0, maxAttempts = 20, badPeaks = 0, maxBadPeaks = 10, peak;
  bool passed = false, peaked = false;
  vector<double> dists(5, 0), deltas(4, 0), distsFromMax(5, 0);

  // Setup start, middle and end CfgTypes  
  startCfg = wpMoved ? heldCfg : segCfgs[0];
  endingCfg = wpMoved ? tmpCfg : segCfgs[segCfgs.size()-1];
  if(this->m_debug) {
    VDComment("Binary Cfgs: ");
    VDAddTempCfg(startCfg, true);
    VDAddTempCfg(endingCfg, true);
  }

  if(this->m_debug) cout << "Calculating Mid... stepSize/cbStepSize: " << stepSize << "/" << cbStepSize << endl;
  uBound = fellOut ? stepSize-1.0 : stepSize;
  if(wpMoved || fellOut) {
    lBound = uBound-1.0;
  } 
  else {
    if(segCfgs.size() < (stepSize - cbStepSize))
      lBound = stepSize - segCfgs.size();
    else
      lBound = cbStepSize;
  }
  middle = (lBound+uBound)/2.0;
  if(this->m_debug) cout << "Lower and upper bounds: " << lBound << "/" << middle << "/" << uBound  << endl;
  midMCfg.multiply(transCfg, middle);
  midMCfg.add(_cfg, midMCfg);

  if(this->m_debug) VDClearComments();
  if(this->m_debug) cout << "start/mid/end: " << endl << startCfg << endl << midMCfg << endl << endingCfg << endl;

  passed = this->CollisionInfo(startCfg, tmpTransCfg, _bb, tmpInfo);
  if(!passed) return false;
  dists[0] = tmpInfo.m_minDist;

  passed = this->CollisionInfo(midMCfg, tmpTransCfg, _bb, tmpInfo);
  if(!passed) return false;
  dists[2] = tmpInfo.m_minDist;

  passed = this->CollisionInfo(endingCfg, tmpTransCfg, _bb, tmpInfo);
  if(!passed) return false;
  dists[4] = tmpInfo.m_minDist;

  middleS = (lBound+middle)/2.0;
  midSCfg.multiply(transCfg, middleS);
  midSCfg.add(_cfg, midSCfg);

  middleE = (middle+uBound)/2.0;
  midECfg.multiply(transCfg, middleE);
  midECfg.add(_cfg, midECfg);  

  if(this->m_debug) {
    cout << "dists: ";
    for(size_t i=0; i<size_t(dists.size()); i++)
      cout << dists[i] << " ";
    cout << endl;
  }

  gapDist = dm->Distance(env, startCfg, endingCfg);

  if(this->m_debug) cout << "start/mids/end: " << endl << startCfg << endl << midSCfg 
    << endl << midMCfg << endl << midECfg << endl << endingCfg << endl;

  do{ // Modified Binomial search to find peaks

    if(this->m_debug) {
      VDAddTempCfg(midMCfg, true);
      VDClearLastTemp();
    }

    // Update Cfgs
    attempts++;
    middle = (lBound + uBound)/2.0;
    middleS = (lBound + middle)/2.0;
    middleE = (middle + uBound)/2.0;
    if(this->m_debug) cout << "Bounds: " << lBound << "/" << middleS << "/" << middle << "/" << middleE << "/" << uBound << endl;
    midSCfg.multiply(transCfg, middleS);
    midSCfg.add(_cfg, midSCfg);
    midECfg.multiply(transCfg, middleE);
    midECfg.add(_cfg, midECfg);  

    passed = this->CollisionInfo(midSCfg, tmpTransCfg, _bb, tmpInfo);
    if(!passed) return false;
    dists[1] = tmpInfo.m_minDist;

    passed = this->CollisionInfo(midECfg, tmpTransCfg, _bb, tmpInfo); 
    if(!passed) return false;
    dists[3] = tmpInfo.m_minDist;

    // Compute Deltas and Max Distance
    deltas.clear();
    distsFromMax.clear();
    maxDist = (dists[0]);
    if(this->m_debug) cout << "Deltas: ";
    for(size_t i=0; i<size_t(dists.size())-1; i++) {
      double tmp = dists[i+1] - dists[i];
      if(this->m_debug) cout << " " << tmp;
      deltas.push_back(tmp);
      maxDist = (maxDist>dists[i+1])?maxDist:dists[i+1];
    } 
    for(size_t i=0; i<size_t(dists.size()); i++)
      distsFromMax.push_back(maxDist-dists[i]);

    if(this->m_debug) cout << endl;

    // Determine Peak
    if(deltas[0] > errEps && deltas[1] <= errEps && distsFromMax[1] < errEps) {
      peak = 1;
      endingCfg = midMCfg;
      midMCfg = midSCfg;
      dists[4] = dists[2];
      dists[2] = dists[1];
      uBound = middle;
    } 
    else if(deltas[1] > errEps && deltas[2] <= errEps && distsFromMax[2] < errEps) {
      peak = 2;
      startCfg = midSCfg;
      endingCfg = midECfg;
      dists[0] = dists[1];
      dists[4] = dists[3];
      lBound = middleS;
      uBound = middleE;
    } 
    else if(deltas[2] > errEps && deltas[3] <= errEps && distsFromMax[3] < errEps) {
      peak = 3;
      startCfg = midMCfg;
      midMCfg = midECfg;
      dists[0] = dists[2];
      dists[2] = dists[3];
      lBound = middle;
    } 
    else {
      if(deltas[0] > errEps && deltas[1] > errEps && deltas[2] > errEps && deltas[3] > errEps) {
        peak = 3;
        startCfg = midMCfg;
        midMCfg = midECfg;
        dists[0] = dists[2];
        dists[2] = dists[3];
        lBound = middle;
      } 
      else if(deltas[0] < -errEps && deltas[1] < -errEps && deltas[2] < -errEps && deltas[3] < -errEps) {
        peak = 1;
        endingCfg = midMCfg;
        midMCfg = midSCfg;
        dists[4] = dists[2];
        dists[2] = dists[1];
        uBound = middle;
      } 
      else {
        // No peak found, recalculate old, mid and new clearance
        badPeaks++;
        peak = -1;

        if(this->CollisionInfo(startCfg, tmpTransCfg, _bb, tmpInfo) 
            && dists[0] >= tmpInfo.m_minDist)
          dists[0] = tmpInfo.m_minDist;

        if(this->CollisionInfo(midMCfg, tmpTransCfg, _bb, tmpInfo) 
            && dists[2] >= tmpInfo.m_minDist)
          dists[2] = tmpInfo.m_minDist;

        if(this->CollisionInfo(endingCfg, tmpTransCfg, _bb, tmpInfo) 
            && dists[4] >= tmpInfo.m_minDist)
          dists[4] = tmpInfo.m_minDist;
      }
    }
    if(this->m_debug) cout << " peak: " << peak << "  cfg: " << midMCfg;
    gapDist = dm->Distance(env, startCfg, endingCfg);
    if(this->m_debug) cout << " gap: " << gapDist << endl;
    if(m_epsilon >= gapDist)
      peaked = true;

  } while(!peaked && attempts < maxAttempts && badPeaks < maxBadPeaks);

  if(this->m_debug) {
    VDComment("Final CFG");
    VDAddTempCfg(midMCfg, true);
    VDClearLastTemp();
    VDClearComments();
    VDClearLastTemp();
    VDClearLastTemp();
  }

  _cfg = midMCfg;
  
  if(this->m_debug) cout << "FINAL Cfg: " << _cfg << " steps: " << stepSize << endl;
  
  return true;
}

#endif
