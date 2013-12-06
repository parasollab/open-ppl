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
        double _extendDist = 0.5, bool _addIntermediates = false, size_t _maxIntermediates = 10);
    MedialAxisRRT(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize();

  private:
    virtual VID ExpandTree(CfgType& _dir);

    //***************************************************************//
    // MARRTExpand: Expands an RRT with a node that is pushed to the //
    // Medial Axis.                                                  //
    //***************************************************************//
    bool MedialAxisExtend(const CfgType& _start, const CfgType& _goal, vector<CfgType>& _innerNodes, double& _length);

    MedialAxisUtility<MPTraits> m_medialAxisUtility;
    double m_extendDist;
    bool m_addIntermediates;
    size_t m_maxIntermediates;
};

template<class MPTraits>
MedialAxisRRT<MPTraits>::MedialAxisRRT(
    const MedialAxisUtility<MPTraits>& _medialAxisUtility,
    double _extendDist, bool _addIntermediates, size_t _maxIntermediates) :
  m_medialAxisUtility(_medialAxisUtility), m_extendDist(_extendDist),
  m_addIntermediates(_addIntermediates), m_maxIntermediates(_maxIntermediates) {
    this->SetName("MedialAxisRRT");
  }

template<class MPTraits>
MedialAxisRRT<MPTraits>::MedialAxisRRT(MPProblemType* _problem, XMLNodeReader& _node) :
  BasicRRTStrategy<MPTraits>(_problem, _node, false, true), m_medialAxisUtility(_problem, _node){
    this->SetName("MedialAxisRRT");
    ParseXML(_node);
    _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
void
MedialAxisRRT<MPTraits>::ParseXML(XMLNodeReader& _node) {
  m_extendDist = _node.numberXMLParameter("extendDist", true, 0.5, 0.0, MAX_DBL, "Step size for regular RRT extend");
  m_addIntermediates = _node.boolXMLParameter("addIntermediates", false, false, "Add all intermediates to nodes of tree");
  m_maxIntermediates = _node.numberXMLParameter("maxIntermediates", false, 10, 1, MAX_INT, "Maximum number of intermediates on an edge");
}

template<class MPTraits>
void
MedialAxisRRT<MPTraits>::Initialize() {
  vector<CfgType>& queryCfgs = this->m_query->GetQuery();
  typedef typename vector<CfgType>::iterator CIT;
  for(CIT cit = queryCfgs.begin(); cit!=queryCfgs.end(); ++cit){
    m_medialAxisUtility.PushToMedialAxis(*cit,
        this->GetMPProblem()->GetEnvironment()->GetBoundary());
  }
  BasicRRTStrategy<MPTraits>::Initialize();
}

template<class MPTraits>
typename MedialAxisRRT<MPTraits>::VID
MedialAxisRRT<MPTraits>::ExpandTree(CfgType& _dir){
  // Setup MP Variables
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dm);
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nf);
  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();

  // Find closest Cfg in map
  vector<pair<VID, double> > kClosest;
  nf->FindNeighbors(rdmp, _dir, back_inserter(kClosest));
  CfgType nearest = rdmp->GetGraph()->GetVertex(kClosest[0].first);

  //Medial Axis Extend from nearest to _dir
  vector<CfgType> intermediateNodes;
  double dist;
  if(!MedialAxisExtend(nearest, _dir, intermediateNodes, dist)) {
    if(this->m_debug) cout << "MARRT could not expand!" << endl;
    return INVALID_VID;
  }

  //successful  extend. Update the roadmap
  if(this->m_debug) cout << "MARRT expanded." << endl;

  VID recentVID = INVALID_VID;
  CfgType newCfg = intermediateNodes.back();
  intermediateNodes.pop_back();
  // If good to go, add to roadmap
  if(m_addIntermediates){
    typedef typename vector<CfgType>::iterator CIT;
    VID previousVID = kClosest[0].first;
    CfgType prevCfg = nearest;
    for(CIT cit = intermediateNodes.begin(); cit!=intermediateNodes.end(); cit++){
      dist = dm->Distance(prevCfg, *cit);
      recentVID = rdmp->GetGraph()->AddVertex(*cit);
      //TODO fix weight
      pair<WeightType, WeightType> weights = make_pair(WeightType(this->m_lp, dist), WeightType(this->m_lp, dist));
      rdmp->GetGraph()->AddEdge(previousVID, recentVID, weights);
      rdmp->GetGraph()->GetVertex(recentVID).SetStat("Parent", previousVID);
      previousVID = recentVID;
      prevCfg = *cit;
    }
  }
  else{
    recentVID = rdmp->GetGraph()->AddVertex(intermediateNodes.back());
    intermediateNodes.pop_back();
    //TODO fix weight
    pair<WeightType, WeightType> weights = make_pair(WeightType(this->m_lp, dist, intermediateNodes), WeightType(this->m_lp, dist, intermediateNodes));
    rdmp->GetGraph()->AddEdge(kClosest[0].first, recentVID, weights);
    rdmp->GetGraph()->GetVertex(recentVID).SetStat("Parent", kClosest[0].first);
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
MedialAxisRRT<MPTraits>::MedialAxisExtend(const CfgType& _start, const CfgType& _goal, vector<CfgType>& _innerNodes, double &_length){
  //Setup
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dm);
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(this->m_lp);

  LPOutput<MPTraits> lpOutput;

  string callee("MPUtility::MARRTExpand");

  CfgType tick, curr = _start;
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  double dist = 0;
  _length = 0;

  do {
    curr = tick;
    _length += dist;
    _innerNodes.push_back(curr);

    if(_innerNodes.size() > m_maxIntermediates)
      break;
    //take a step at distance _extendDist
    CfgType incr = _goal - curr;
    dm->ScaleCfg(m_extendDist, incr);
    tick = curr + incr;

    //Push tick to the MA
    if(!m_medialAxisUtility.PushToMedialAxis(tick, env->GetBoundary())) {
      if(this->m_debug) cout << "PushToMedialAxis failed...MARRTExpand failed" << endl;
      break;
    }

    dist = dm->Distance(curr, tick);

  } while(
      dist > this->m_minDist
      && lp->IsConnected(curr, tick, &lpOutput, positionRes, orientationRes)
      && _length+dist <= this->m_delta
      );

  _innerNodes.erase(_innerNodes.begin());
  if(_innerNodes.empty())
    return false;
  else
    return true;

  /*while(_innerNodes.size() <= m_maxIntermediates) {
    tick = curr;

    //take a step at distance _extendDist
    CfgType incr = _goal - curr;
    dm->ScaleCfg(m_extendDist, incr);
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
    if(lp->IsConnected(curr, tick, col, &lpOutput, positionRes, orientationRes)){
      pathLength += lpOutput.m_edge.first.GetWeight();
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
  return true;*/
}

#endif
