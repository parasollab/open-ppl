#ifndef REGIONRRTCONNECT_H
#define REGIONRRTCONNECT_H

#include "ConnectorMethod.h"

/* TODO Cesar Add Description
 *
 */

#define KCLOSEST 5
#define MFAILURE 5

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
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType,WeightType> SequentialGraphType;
    typedef typename vector<VID>::iterator VIDIT;

    //////////////////////
    // Constructors and Destructor
    RegionRRTConnect(string _lp = "", string _nf = "",
        int _k = KCLOSEST, int _m = MFAILURE,
        bool _countFailures = false, bool _unconnected = false,
        bool _random = false);
    RegionRRTConnect(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~RegionRRTConnect();

    //////////////////////
    // Used in new MPProblem framework.
    virtual void PrintOptions(ostream& _os);
    virtual void ParseXML(XMLNodeReader& _node);

    //////////////////////
    // Core: Connection method
    template<typename ColorMap, typename InputIterator>
      bool IsConnect(RoadmapType* _rm, StatClass& _stats,
          ColorMap& _cmap, InputIterator _itr1First, InputIterator _itr1Last,
          InputIterator _itr2First, InputIterator _itr2Last) ;

  protected:

    //////////////////////
    // Utility Method
    bool ExpandTree(CfgType& _dir, vector<VID>* _targetTree, bool _isLocal, double _delta, VID& _newVID, CfgType& _newCfg);
    bool ExpandTree(CfgType& _dir, const VID& _dirVID, vector<VID>* _targetTree, bool _isLocal, double _delta, VID& _newVID, CfgType& _newCfg, bool _interTree);

    CfgType SelectDirection();
    void UpdateTrees();

  private:
    //////////////////////
    // Data
    int m_totalSuccess;
    int m_totalFailure;
    int m_iterSuccess;
    int m_iterFailure;
    int m_iterations;
    int m_fail;
    bool m_countFailures;
    double m_delta;
    double m_minDist;
    string m_vcLabel;

    vector<pair<VID, CfgType> > m_localPendingVIDs;
    vector<pair<VID, CfgType> > m_remotePendingVIDs;
    vector<pair<VID, VID> > m_localPendingEdges;
    vector<pair<VID, VID> > m_remotePendingEdges;

};

  template<class MPTraits>
RegionRRTConnect<MPTraits>::RegionRRTConnect(string _lp, string _nf, int _iterations, int _fail,
    bool _countFailures, bool _delta, bool _minDist)
  : ConnectorMethod<MPTraits>(), m_iterations(_iterations), m_fail(_fail),
  m_countFailures(_countFailures), m_delta(_delta),
  m_minDist(_minDist){
    this->SetName("RegionRRTConnect");
    this->m_lpMethod = _lp;
    this->m_nfMethod = _nf;
  }

  template<class MPTraits>
RegionRRTConnect<MPTraits>::RegionRRTConnect(MPProblemType* _problem, XMLNodeReader& _node)
  : ConnectorMethod<MPTraits>(_problem, _node) {
    ParseXML(_node);
  }

template<class MPTraits>
RegionRRTConnect<MPTraits>::~RegionRRTConnect(){
}

template<class MPTraits>
void
RegionRRTConnect<MPTraits>::ParseXML(XMLNodeReader& _node){
  this->SetName("RegionRRTConnect");
  m_countFailures = _node.boolXMLParameter("count_failures", false, false, "if false, ignore failure count and just attempt k; if true, attempt k neighbors until too many failures detected");
  m_iterations = _node.numberXMLParameter("iterations", true, 0, 0, MAX_INT, "Number of iterations that RRT Connect will perform");
  m_fail = _node.numberXMLParameter("fail", false, m_iterations, 0, MAX_INT, "amount of failed connections allowed before operation terminates");
  m_delta = _node.numberXMLParameter("delta", false, 1.0, 0.0, MAX_DBL, "Delta Distance");
  m_minDist = _node.numberXMLParameter("minDist", false, 0.0, 0.0, MAX_DBL, "Minimum Distance");
  m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Checker Method");
  _node.warnUnrequestedAttributes();


}

template<class MPTraits>
void
RegionRRTConnect<MPTraits>::PrintOptions(ostream& _os){
  ConnectorMethod<MPTraits>::PrintOptions(_os);
  /*
     _os << "    " << this->GetName() << "::  k = ";
     _os << m_k << "  fail = " << m_fail ;
     _os << "  count_failures = " << this->m_countFailures;
     _os << "  unconnected = " << m_unconnected;
     _os << "  random = " << m_random;
     _os << endl;
     */
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
template<typename ColorMap, typename InputIterator>
bool
RegionRRTConnect<MPTraits>::IsConnect(RoadmapType* _rm, StatClass& _stats,
    ColorMap& _cmap, InputIterator _itr1First, InputIterator _itr1Last,
    InputIterator _itr2First, InputIterator _itr2Last){

  if(this->m_debug){
    //cout << endl;
    // PrintOptions(cout);
  }


  // Ta = itr1, Tb = itr2
  map<VID, CfgType> existingNodes;
  vector<VID>* treeA = new vector<VID>();
  vector<VID>* treeB = new vector<VID>();
  for(InputIterator it = _itr1First; it != _itr1Last; ++it) {
    treeA->push_back(*it);
  }
  for(InputIterator it = _itr2First; it != _itr2Last; ++it) {
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
    ExpandTree(dir, treeA, isTreeALocal, m_delta, newVID, newCfg);

    if(newVID != INVALID_VID) {

      m_totalSuccess++;
      //treeA->push_back(newVID);  we add VID to the tree in Expand

      // Since expand goes until collision or goal is detected, we shouldnt iterate.
      connected = ExpandTree(newCfg, newVID, treeB, !isTreeALocal, MAX_DBL, interTreeVID, interTreeCfg, true);

    } else {
      m_totalFailure++;
    }

    // Switching trees
    swap(treeA, treeB);
    isTreeALocal = !isTreeALocal;
    iter++;
  }

}

template<class MPTraits>
bool
RegionRRTConnect<MPTraits>::ExpandTree(CfgType& _dir, vector<VID>* _targetTree, bool _isLocal, double _delta, VID& _newVID, CfgType& _newCfg){

  return ExpandTree(_dir, INVALID_VID, _targetTree, _isLocal, _delta, _newVID, _newCfg, false);
}

template<class MPTraits>
bool
RegionRRTConnect<MPTraits>::ExpandTree(CfgType& _dir, const VID& _dirVID, vector<VID>* _targetTree, bool _isLocal,
    double _delta, VID& _newVID, CfgType& _newCfg, bool _interTree){
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod);

  // TODO Cesar fix cast
  //shared_ptr<FamilyLine> ptr(dynamic_pointer_cast<FamilyLine>(*i));
  boost::shared_ptr<BruteForceNF<MPTraits> > bruteForceNF (boost::dynamic_pointer_cast<BruteForceNF<MPTraits> >(nf));

  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  CDInfo  cdInfo;
  // Find closest Cfg in map
  vector<VID> kClosest;
  vector<CfgType> cfgs;

  StatClass* kcloseStatClass = this->GetMPProblem()->GetStatClass();
  string kcloseClockName = "kclosest time ";
  SequentialGraphType* targetGraph = _isLocal ? this->m_localGraph : this->m_remoteGraph;

  kcloseStatClass->StartClock(kcloseClockName);
  // Choose the closest node from the three
  bruteForceNF->KClosest(targetGraph, _targetTree->begin(), _targetTree->end(), _dir, 1, back_inserter(kClosest));

  kcloseStatClass->StopClock(kcloseClockName);

  bool connected = false;

  _newVID = INVALID_VID;
  CfgType nearest  =   (*(targetGraph->find_vertex(kClosest[0]))).property();
//  const CfgType& nearest = rdmp->GetGraph()->GetCfg(kClosest[0]);
  int weight;

  StatClass* expandStatClass = this->GetMPProblem()->GetStatClass();
  string expandClockName = "RegionRRTConnect time ";
  expandStatClass->StartClock(expandClockName);

  string dmLabel = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod()->GetLabel();

  bool expanded = RRTExpand<MPTraits>(this->GetMPProblem(), m_vcLabel, dmLabel, nearest, _dir, _newCfg,
      _delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes());

  if(!expanded) {
    //if(this->m_debug) cout << "RRT could not expand!" << endl;
    return connected;
  }
  //if (this->m_debug) cout<<"RRT expanded"<<endl;

  expandStatClass->StopClock(expandClockName);
  if(dm->Distance(env, _newCfg, nearest) >= m_minDist) {
    // if _newCfg = Dir, we reached goal
    if (_newCfg == _dir && _interTree)  {  // this expansion is between trees
      _newVID = _dirVID;
      connected = true;
    } else {

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



    pair<WeightType, WeightType> weights = make_pair(WeightType("RegionRRTConnect", weight), WeightType("RegionRRTConnect", weight));

    #ifndef _PARALLEL
    rdmp->GetGraph()->AddEdge(kClosest[0], _newVID, weights);
    #else
    WeightType weightT("RRTExpand", weight);
    GraphType* globalTree = rdmp->GetGraph();
    globalTree->add_edge_async(kClosest[0], _newVID, weightT);
    globalTree->add_edge_async(_newVID, kClosest[0], weightT);

    if(_newVID != _dirVID) {
      targetGraph->add_edge(kClosest[0],_newVID);
      targetGraph->add_edge(_newVID, kClosest[0]);
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

  return connected;
}


template<class MPTraits>
void
RegionRRTConnect<MPTraits>::UpdateTrees(){

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
RegionRRTConnect<MPTraits>::SelectDirection(){
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CfgType dir;
  dir.GetRandomCfg(env);
  return dir;
}

///////////////////////////////////////////////////////////////////////////////
#endif

