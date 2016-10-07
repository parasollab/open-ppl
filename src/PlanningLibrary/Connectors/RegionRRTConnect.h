#ifndef REGION_RRT_CONNECT_H_
#define REGION_RRT_CONNECT_H_

#include "ConnectorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class RegionRRTConnect: public ConnectorMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::ExtenderPointer ExtenderPointer;
    typedef typename stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType,WeightType> SequentialGraphType;
    typedef typename vector<VID>::iterator VIDIT;

    RegionRRTConnect(string _nf = "", string _lp = "", string _eLabel = "",
        size_t _iterations = 100, double _minDist = 0);
    RegionRRTConnect(MPProblemType* _problem, XMLNode& _node);

    virtual void Print(ostream& _os) const;

    template<typename InputIterator1, typename InputIterator2,
      typename OutputIterator>
        bool Connect(RoadmapType* _rm,
            InputIterator1 _itr1First, InputIterator1 _itr1Last,
            InputIterator2 _itr2First, InputIterator2 _itr2Last,
            OutputIterator _collision);

  protected:

    //////////////////////
    // Utility Method
    bool ExpandTree(CfgType& _dir, vector<VID>* _targetTree, bool _isLocal, VID& _newVID, CfgType& _newCfg);
    bool ExpandTree(CfgType& _dir, const VID& _dirVID, vector<VID>* _targetTree, bool _isLocal, VID& _newVID, CfgType& _newCfg, bool _interTree);

    CfgType SelectDirection();
    void UpdateTrees();

  private:
    string m_eLabel;
    size_t m_iterations;
    double m_minDist;

    vector<pair<VID, CfgType> > m_localPendingVIDs;
    vector<pair<VID, CfgType> > m_remotePendingVIDs;
    vector<pair<VID, VID> > m_localPendingEdges;
    vector<pair<VID, VID> > m_remotePendingEdges;

};

template<class MPTraits>
RegionRRTConnect<MPTraits>::
RegionRRTConnect(string _nf, string _lp, string _eLabel,
    size_t _iterations, double _minDist) :
  ConnectorMethod<MPTraits>(_nf, _lp), m_eLabel(_eLabel),
  m_iterations(_iterations), m_minDist(_minDist) {
    this->SetName("RegionRRTConnect");
  }

template<class MPTraits>
RegionRRTConnect<MPTraits>::
RegionRRTConnect(MPProblemType* _problem, XMLNode& _node) :
  ConnectorMethod<MPTraits>(_problem, _node) {
    this->SetName("RegionRRTConnect");
    m_iterations = _node.Read("iterations", true, 0, 0, MAX_INT,
        "Number of iterations that RRT Connect will perform");
    m_eLabel = _node.Read("eLabel", true, "", "Extender Method");
    m_minDist = _node.Read("minDist", false, 0.0, 0.0, MAX_DBL,
        "Minimum Distance");
  }

template<class MPTraits>
void
RegionRRTConnect<MPTraits>::
Print(ostream& _os) const {
  ConnectorMethod<MPTraits>::Print(_os);
}

/*
 *  INPUT: Two Trees (graph), Ta, Tb given by itr1 and itr2.
 *  ACTION: Attempt to connect these trees:
 *    getRandCfg
 *    Expand Ta to randCfg
 *    Expand Tb to newCfg
 *    switch Ta,Tb
 *    repeat
 * */
template<class MPTraits>
template<typename InputIterator1, typename InputIterator2, typename OutputIterator>
bool
RegionRRTConnect<MPTraits>::
Connect(RoadmapType* _rm,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    OutputIterator _collision) {

  // Ta = itr1, Tb = itr2
  map<VID, CfgType> existingNodes;
  vector<VID>* treeA = new vector<VID>();
  vector<VID>* treeB = new vector<VID>();
  for(InputIterator1 it = _itr1First; it != _itr1Last; ++it) {
    treeA->push_back(*it);
  }
  for(InputIterator2 it = _itr2First; it != _itr2Last; ++it) {
    treeB->push_back(*it);
  }

  size_t iter = 0;
  bool connected = false;
  bool isTreeALocal = true;
  while( iter < m_iterations && !connected) {

    CfgType dir = this->SelectDirection();
    // Expand in direction of Ta
    VID newVID, interTreeVID;
    CfgType newCfg, interTreeCfg;
    ExpandTree(dir, treeA, isTreeALocal, newVID, newCfg);

    if(newVID != INVALID_VID) {

      //treeA->push_back(newVID);  we add VID to the tree in Expand

      // Since expand goes until collision or goal is detected, we shouldnt iterate.
      connected = ExpandTree(newCfg, newVID, treeB, !isTreeALocal, interTreeVID, interTreeCfg, true);

    }

    // Switching trees
    swap(treeA, treeB);
    isTreeALocal = !isTreeALocal;
    iter++;
  }

  return connected;
}

template<class MPTraits>
bool
RegionRRTConnect<MPTraits>::
ExpandTree(CfgType& _dir, vector<VID>* _targetTree, bool _isLocal, VID& _newVID, CfgType& _newCfg) {
  return ExpandTree(_dir, INVALID_VID, _targetTree, _isLocal, _newVID, _newCfg, false);
}

template<class MPTraits>
bool
RegionRRTConnect<MPTraits>::
ExpandTree(CfgType& _dir, const VID& _dirVID, vector<VID>* _targetTree, bool _isLocal,
    VID& _newVID, CfgType& _newCfg, bool _interTree){
  // Setup MP Variables
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfLabel)->GetDMMethod();
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfLabel);

  // TODO Cesar fix cast
  //shared_ptr<FamilyLine> ptr(dynamic_pointer_cast<FamilyLine>(*i));
  shared_ptr<BruteForceNF<MPTraits> > bruteForceNF(dynamic_pointer_cast<BruteForceNF<MPTraits> >(nf));

  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  CDInfo  cdInfo;
  // Find closest Cfg in map
  vector<pair<VID, double>> kClosest;
  vector<CfgType> cfgs;

  SequentialGraphType* targetGraph = _isLocal ? this->m_localGraph : this->m_remoteGraph;
  // Choose the closest node from the three
  // TODO Use targetGraph instead of overall map
  //bruteForceNF->FindNeighbors(targetGraph, _targetTree->begin(), _targetTree->end(), _dir, back_inserter(kClosest));
  bruteForceNF->FindNeighbors(rdmp, _targetTree->begin(), _targetTree->end(), _dir, back_inserter(kClosest));

  _newVID = INVALID_VID;
  CfgType nearest  =  (*(targetGraph->find_vertex(kClosest[0].first))).property();

  LPOutput<MPTraits> lpOut;
  bool expanded = this->GetExtender(m_eLabel)->Extend(nearest, _dir, _newCfg, lpOut);

  if(!expanded) {
    return false;
  }

  if(dm->Distance(_newCfg, nearest) >= m_minDist) {
    // if _newCfg = Dir, we reached goal
    if (_newCfg == _dir && _interTree)  {  // this expansion is between trees
      _newVID = _dirVID;
    }
    else {

      #ifndef _PARALLEL
      _newVID = rdmp->GetGraph()->AddVertex(_newCfg);
      #else
      _newVID = rdmp->GetGraph()->add_vertex(_newCfg);

      targetGraph->add_vertex(_newVID, _newCfg);
      /*
      if(_isLocal)
        m_localPendingVIDs.push_back(make_pair(_newVID, _newCfg));
      else
        m_remotePendingVIDs.push_back(make_pair(_newVID, _newCfg));
      */
      if(this->m_debug) VDAddNode(_newCfg);
      #endif
    }

    #ifndef _PARALLEL
    rdmp->GetGraph()->AddEdge(kClosest[0].first, _newVID, lpOut.m_edge);
    #else
    GraphType* globalTree = rdmp->GetGraph();
    globalTree->add_edge_async(kClosest[0].first, _newVID, lpOut.m_edge.first);
    globalTree->add_edge_async(_newVID, kClosest[0].first, lpOut.m_edge.second);

    if(_newVID != _dirVID) {
      targetGraph->add_edge(kClosest[0].first,_newVID);
      targetGraph->add_edge(_newVID, kClosest[0].first);
    }
    /*
    if(_isLocal)
      m_localPendingEdges.push_back(make_pair(_newVID, kClosest[0]));
    else
      m_remotePendingEdges.push_back(make_pair(_newVID, kClosest[0]));
    */
    if(this->m_debug) VDAddEdge(nearest, _newCfg);
    #endif
    _targetTree->push_back(_newVID);
  }

  return true;
}


template<class MPTraits>
void
RegionRRTConnect<MPTraits>::
UpdateTrees() {
  for(int i=0; i<m_localPendingVIDs.size(); i++) {
      this->m_localGraph->add_vertex(m_localPendingVIDs[i].first,m_localPendingVIDs[i].second);
  }
  for(int i=0; i<m_remotePendingVIDs.size(); i++) {
      this->m_remoteGraph->add_vertex(m_remotePendingVIDs[i].first,m_remotePendingVIDs[i].second);
  }
  for(int i=0; i<m_localPendingEdges.size(); i++) {
      this->m_localGraph->add_edge(m_localPendingEdges[i].first,m_localPendingEdges[i].second);
      this->m_localGraph->add_edge(m_localPendingEdges[i].second,m_localPendingEdges[i].first);
  }
  for(int i=0; i<m_remotePendingEdges.size(); i++) {
      this->m_remoteGraph->add_edge(m_remotePendingEdges[i].first,m_remotePendingEdges[i].second);
      this->m_remoteGraph->add_edge(m_remotePendingEdges[i].second,m_remotePendingEdges[i].first);
  }
}

template<class MPTraits>
typename MPTraits::CfgType
RegionRRTConnect<MPTraits>::
SelectDirection() {
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CfgType dir;
  dir.GetRandomCfg(env);
  return dir;
}

#endif

