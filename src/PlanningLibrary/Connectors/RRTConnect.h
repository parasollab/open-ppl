#ifndef RRT_CONNECT_H
#define RRT_CONNECT_H

#include "ConnectorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class RRTConnect: public ConnectorMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::ExtenderPointer ExtenderPointer;
    typedef typename vector<VID>::iterator VIDIT;

    RRTConnect(string _nfLabel = "", string _lpLabel = "",
        size_t _iterations = 100, double _minDist = 0.0);
    RRTConnect(MPProblemType* _problem, XMLNode& _node);

    virtual void Print(ostream& _os);

    template<typename InputIterator1, typename InputIterator2,
      typename OutputIterator>
        void Connect(RoadmapType* _rm,
            InputIterator1 _itr1First, InputIterator1 _itr1Last,
            InputIterator2 _itr2First, InputIterator2 _itr2Last,
            bool _fromFullRoadmap,
            OutputIterator _collision);

  protected:

    bool ExpandTree(CfgType& _dir, vector<VID>* _targetTree, VID& _newVID, CfgType& _newCfg);
    bool ExpandTree(CfgType& _dir, const VID& _dirVID, vector<VID>* _targetTree, VID& _newVID, CfgType& _newCfg, bool _interTree);

    CfgType SelectDirection();

  private:
    size_t m_iterations;
    double m_minDist;
    string m_eLabel;
};

template<class MPTraits>
RRTConnect<MPTraits>::
RRTConnect(string _nfLabel, string _lpLabel, size_t _iterations, double _minDist) :
  ConnectorMethod<MPTraits>(_nfLabel, _lpLabel),
  m_iterations(_iterations), m_minDist(_minDist) {
    this->SetName("RRTConnect");
  }

template<class MPTraits>
RRTConnect<MPTraits>::
RRTConnect(MPProblemType* _problem, XMLNode& _node)
  : ConnectorMethod<MPTraits>(_problem, _node) {
    this->SetName("RRTConnect");

    //parse xml
    m_iterations = _node.Read("iterations", true, 0, 0, MAX_INT,
        "Number of iterations that RRT Connect will perform");
    m_minDist = _node.Read("minDist", false, 0.0, 0.0, MAX_DBL,
        "Minimum Distance");
    m_eLabel = _node.Read("eLabel", true, "", "Expander Method");
  }

template<class MPTraits>
void
RRTConnect<MPTraits>::
Print(ostream& _os) {
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
void
RRTConnect<MPTraits>::
Connect(RoadmapType* _rm,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision) {

  // Ta = itr1, Tb = itr2
  GraphType* g = this->GetRoadmap()->GetGraph();
  map<VID, CfgType> existingNodes;
  vector<VID>* treeA = new vector<VID>();
  vector<VID>* treeB = new vector<VID>();
  treeA->reserve(_itr1Last - _itr1First);
  treeB->reserve(_itr2Last - _itr2First);

  for(InputIterator1 it = _itr1First; it != _itr1Last; ++it) {
    treeA->push_back(g->GetVID(it));
  }
  for(InputIterator2 it = _itr2First; it != _itr2Last; ++it) {
    treeB->push_back(g->GetVID(it));
  }

  size_t iter = 0;
  bool connected = false;
  while( iter < m_iterations && !connected) {

    CfgType dir = this->SelectDirection();
    // Expand in direction of Ta
    VID newVID, interTreeVID;
    CfgType newCfg, interTreeCfg;
    ExpandTree(dir, treeA, newVID, newCfg);

    if(newVID != INVALID_VID) {

      //treeA->push_back(newVID);  we add VID to the tree in Expand

      // Since expand goes until collision or goal is detected, we shouldnt iterate.
      connected = ExpandTree(newCfg, newVID, treeB, interTreeVID, interTreeCfg, true);
    }

    // Switching trees
    swap(treeA, treeB);
    iter++;
  }

}

template<class MPTraits>
bool
RRTConnect<MPTraits>::
ExpandTree(CfgType& _dir, vector<VID>* _targetTree,
    VID& _newVID, CfgType& _newCfg) {
  return ExpandTree(_dir, INVALID_VID, _targetTree, _newVID, _newCfg, false);
}

template<class MPTraits>
bool
RRTConnect<MPTraits>::
ExpandTree(CfgType& _dir, const VID& _dirVID,
    vector<VID>* _targetTree, VID& _newVID, CfgType& _newCfg,
    bool _interTree) {
  // Setup MP Variables
  NeighborhoodFinderPointer nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  DistanceMetricPointer dm = nf->GetDMMethod();
  ExtenderPointer e = this->GetExtender(m_eLabel);
  RoadmapType* rdmp = this->GetRoadmap();

  // Find closest Cfg in map
  vector<pair<VID, double>> kClosest;
  nf->FindNeighbors(rdmp, _targetTree->begin(), _targetTree->end(), false,
      _dir, back_inserter(kClosest));

  bool connected = false;

  _newVID = INVALID_VID;
  const CfgType& nearest = rdmp->GetGraph()->GetVertex(kClosest[0].first);

  LPOutput<MPTraits> lpOut;
  bool expanded = e->Extend(nearest, _dir, _newCfg, lpOut);

  if(!expanded) {
    return connected;
  }

  if(dm->Distance(_newCfg, nearest) >= m_minDist) {
    // if _newCfg = Dir, we reached goal
    if (_newCfg == _dir && _interTree)  {  // this expansion is between trees
      _newVID = _dirVID;
      connected = true;
    }
    else {
      _newVID = rdmp->GetGraph()->AddVertex(_newCfg);
//#ifdef _PARALLEL
      this->m_localGraph->add_vertex(_newVID, _newCfg);
      if(this->m_debug) VDAddNode(_newCfg);
//#endif
    }

#ifndef _PARALLEL
    rdmp->GetGraph()->AddEdge(kClosest[0].first, _newVID, lpOut.m_edge);
#else
    GraphType* globalTree = rdmp->GetGraph();
    globalTree->add_edge_async(kClosest[0].first, _newVID, lpOut.m_edge.first);
    globalTree->add_edge_async(_newVID, kClosest[0].first, lpOut.m_edge.second);

#endif
    this->m_localGraph->add_edge(kClosest[0].first, _newVID);
    this->m_localGraph->add_edge(_newVID, kClosest[0].first);
    if(this->m_debug) VDAddEdge(nearest, _newCfg);
    _targetTree->push_back(_newVID);
  }

  return connected;
}

template<class MPTraits>
typename MPTraits::CfgType
RRTConnect<MPTraits>::
SelectDirection(){
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CfgType dir;
  dir.GetRandomCfg(env);
  return dir;
}

#endif
