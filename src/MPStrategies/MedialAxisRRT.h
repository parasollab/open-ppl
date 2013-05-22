/**
 * The Medial Axis RRT builds a tree such that all nodes lie upon the medial
 * axis of the free space. For the edges to lie on the medial axis, one must use
 * MALP for validation of the edges.
 */

#ifndef MEDIALAXISRRT_H_
#define MEDIALAXISRRT_H_

#include "BasicRRTStrategy.h"

template<class MPTraits>
class MedialAxisRRT : public BasicRRTStrategy<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;

    MedialAxisRRT(const MedialAxisUtility<MPTraits>& _medialAxisUtility = MedialAxisUtility<MPTraits>(),
        double _extendDist = 0.5, size_t _maxIntermediates = 10);
    MedialAxisRRT(MPProblemType* _problem, XMLNodeReader& _node);
    
    virtual void ParseXML(XMLNodeReader& _node);

  private:
    virtual VID ExpandTree(CfgType& _dir);
    
    //***************************************************************//
    // MARRTExpand: Expands an RRT with a node that is pushed to the //
    // Medial Axis.                                                  //
    //***************************************************************//
    bool MedialAxisExtend(CfgType _start, CfgType _goal, vector<CfgType>& _innerNodes);

    MedialAxisUtility<MPTraits> m_medialAxisUtility;
    double m_extendDist;
    size_t m_maxIntermediates;
};

template<class MPTraits>
MedialAxisRRT<MPTraits>::MedialAxisRRT(
    const MedialAxisUtility<MPTraits>& _medialAxisUtility, 
    double _extendDist, size_t _maxIntermediates) : 
  m_medialAxisUtility(_medialAxisUtility), m_extendDist(_extendDist), 
  m_maxIntermediates(_maxIntermediates) {
    this->SetName("MedialAxisRRT");
  }

template<class MPTraits>
MedialAxisRRT<MPTraits>::MedialAxisRRT(MPProblemType* _problem, XMLNodeReader& _node) : 
  BasicRRTStrategy<MPTraits>(_problem, _node, false), m_medialAxisUtility(_problem, _node){
    this->SetName("MedialAxisRRT");
    ParseXML(_node);
    _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
void
MedialAxisRRT<MPTraits>::ParseXML(XMLNodeReader& _node) {
  m_extendDist = _node.numberXMLParameter("extendDist", true, 0.5, 0.0, MAX_DBL, "Step size for regular RRT extend");
  m_maxIntermediates = _node.numberXMLParameter("maxIntermediates", true, 5, 1, MAX_INT, "Maximum Number Of Intermediates");
}

template<class MPTraits>
typename MedialAxisRRT<MPTraits>::VID 
MedialAxisRRT<MPTraits>::ExpandTree(CfgType& _dir){
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dm);
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nf);
  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();  
  
  // Find closest Cfg in map
  vector<VID> kClosest;
  nf->KClosest(rdmp, _dir, 1, back_inserter(kClosest));     
  CfgType nearest = rdmp->GetGraph()->GetVertex(kClosest[0]);
 
  //Medial Axis Extend from nearest to _dir
  vector<CfgType> intermediateNodes;
  if(!MedialAxisExtend(nearest, _dir, intermediateNodes)) {
    if(this->m_debug) cout << "MARRT could not expand!" << endl; 
    return INVALID_VID;
  }

  //successful  extend. Update the roadmap
  if(this->m_debug) cout << "MARRT expanded." << endl;

  VID recentVID = INVALID_VID;
  CfgType newCfg = intermediateNodes.back(); 
  intermediateNodes.pop_back();
  double dist = dm->Distance(env, newCfg, nearest);
  // If good to go, add to roadmap
  if(dist >= this->m_minDist) {
    recentVID = rdmp->GetGraph()->AddVertex(newCfg);
    //TODO fix weight
    pair<WeightType, WeightType> weights = make_pair(WeightType(this->m_lp, dist, intermediateNodes), WeightType(this->m_lp, dist, intermediateNodes));
    rdmp->GetGraph()->AddEdge(kClosest[0], recentVID, weights);
    rdmp->GetGraph()->GetVertex(recentVID).SetStat("Parent", kClosest[0]);
  } 
  
  if(this->m_debug)
    cout << "new node added to roadmap: " << recentVID << endl;
  
  return recentVID;
}

//***************************************************************//
// MARRTExpand: Expands an RRT with a node that is pushed to the //
// Medial Axis.                                                  //
//***************************************************************//
template<class MPTraits>
bool
MedialAxisRRT<MPTraits>::MedialAxisExtend(CfgType _start, CfgType _goal, vector<CfgType>& _innerNodes){
  //Setup
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dm);
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(this->m_lp);
  
  LPOutput<MPTraits> lpOutput;

  string callee("MPUtility::MARRTExpand");

  CfgType tick, curr = _start, origin;
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  double pathLength = 0;
  
  while(_innerNodes.size() <= m_maxIntermediates) {
    tick = curr;

    //take a step at distance _extendDist
    CfgType incr = _goal - curr;
    dm->ScaleCfg(env, m_extendDist, origin, incr);
    tick = curr + incr;
    
    //Push tick to the MA
    if(!m_medialAxisUtility.PushToMedialAxis(tick, env->GetBoundary())) {
      if(_innerNodes.size() > 0){
        if(this->m_debug) cout << "PushToMedialAxis failed...returning intermediate nodes" << endl;
        return true;
      }
      else{
        if(this->m_debug) cout << "PushToMedialAxis failed...MARRTExpand failed" << endl;
        return false;    
      }
    }

    //check to see if tick moved
    if(curr == tick){
      if(this->m_debug) cout << "curr == tick; MARRT couldn't expand any further" << endl;
      if(_innerNodes.size() > 0){
        if(this->m_debug) cout << "returning intermediate nodes" << endl;
        return true;
      }
      else{
        if(this->m_debug) cout << "MARRTExpand failed" << endl;
        return false;    
      }
    }

    //We pushed to the MA and went somewhere, check visibility and update
    //structures
    CfgType col;
    if(lp->IsConnected(env, *stats, dm, curr, tick, col, &lpOutput, positionRes, orientationRes)){
      pathLength += lpOutput.edge.first.GetWeight();
      if(pathLength >= this->m_delta){
        if(this->m_debug) cout << "expanded past delta." << endl;
        if(_innerNodes.size() > 0){
          if(this->m_debug) cout << "Returning intermediates." << endl;
          return true;
        }
        else{
          if(this->m_debug) cout << "MedialAxisExtend failed." << endl;
          return false;
        }
      }
      _innerNodes.push_back(tick);
    }
    else{
      if(this->m_debug) cout << "Couldn't connect to previous node in MARRTExpand." << endl;
      if(_innerNodes.size() > 0){
        if(this->m_debug) cout << "Returning intermediates." << endl;
        return true;
      }
      else{
        if(this->m_debug) cout << "MedialAxisExtend failed." << endl;
        return false;
      }
    }
    curr = tick;
  }
  if(this->m_debug) cout << "max intermediates reached.  Returning..." << endl;
  return true;
}

#endif
