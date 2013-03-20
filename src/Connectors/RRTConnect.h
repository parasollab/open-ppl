#ifndef RRTCONNECT_H
#define RRTCONNECT_H

#include "ConnectorMethod.h"

/* TODO Cesar Add Description
 *
 */

#define KCLOSEST 5 
#define MFAILURE 5 

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
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename vector<VID>::iterator VIDIT;

    //////////////////////
    // Constructors and Destructor
    RRTConnect(string _lp = "", string _nf = "", 
        int _k = KCLOSEST, int _m = MFAILURE, 
        bool _countFailures = false, bool _unconnected = false, 
        bool _random = false);
    RRTConnect(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~RRTConnect();

    //////////////////////
    // Used in new MPProblem framework.
    virtual void PrintOptions(ostream& _os);  
    virtual void ParseXML(XMLNodeReader& _node);

    //////////////////////
    // Core: Connection method
    template<typename ColorMap, typename InputIterator, typename OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats,
          ColorMap& _cmap, InputIterator _itr1First, InputIterator _itr1Last,
          InputIterator _itr2First, InputIterator _itr2Last, OutputIterator _collision) ;

  protected:

    //////////////////////
    // Utility Method
    bool ExpandTree(CfgType& _dir, vector<VID>* _targetTree, double _delta, VID& _newVID, CfgType& _newCfg);
    bool ExpandTree(CfgType& _dir, const VID& _dirVID, vector<VID>* _targetTree, double _delta, VID& _newVID, CfgType& _newCfg, bool _interTree);

    CfgType SelectDirection();

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
};

  template<class MPTraits>
RRTConnect<MPTraits>::RRTConnect(string _lp, string _nf, int _iterations, int _fail, 
    bool _countFailures, bool _delta, bool _minDist) 
  : ConnectorMethod<MPTraits>(), m_iterations(_iterations), m_fail(_fail), 
  m_countFailures(_countFailures), m_delta(_delta),
  m_minDist(_minDist){
    this->SetName("RRTConnect"); 
    this->m_lpMethod = _lp;
    this->m_nfMethod = _nf;
  }

  template<class MPTraits>
RRTConnect<MPTraits>::RRTConnect(MPProblemType* _problem, XMLNodeReader& _node) 
  : ConnectorMethod<MPTraits>(_problem, _node) {
    ParseXML(_node);
  }

template<class MPTraits>
RRTConnect<MPTraits>::~RRTConnect(){ 
}

template<class MPTraits>
void
RRTConnect<MPTraits>::ParseXML(XMLNodeReader& _node){
  this->SetName("RRTConnect"); 
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
RRTConnect<MPTraits>::PrintOptions(ostream& _os){
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
template<typename ColorMap, typename InputIterator, typename OutputIterator>
void
RRTConnect<MPTraits>::Connect(RoadmapType* _rm, StatClass& _stats, 
    ColorMap& _cmap, InputIterator _itr1First, InputIterator _itr1Last,
    InputIterator _itr2First, InputIterator _itr2Last, OutputIterator _collision){

  if(this->m_debug){
    //cout << endl; 
    // PrintOptions(cout);
  }


  // Ta = itr1, Tb = itr2
  map<VID, CfgType> existingNodes;
  vector<VID>* treeA = new vector<VID>();
  vector<VID>* treeB = new vector<VID>();
  cout << endl << "TreeA:" << endl;
  for(InputIterator it = _itr1First; it != _itr1Last; ++it) {
    treeA->push_back(*it);
    cout << *it << " ";
  }
  cout << endl;
  cout << "TreeB:" << endl;
  for(InputIterator it = _itr2First; it != _itr2Last; ++it) {
    treeB->push_back(*it);
    cout << *it << " ";
  }
  cout << endl;

  // TODO All good till here
  size_t iter = 0;
  bool connected = false;
  while( iter < m_iterations && !connected) {

    CfgType dir = this->SelectDirection();
    // Expand in direction of Ta
    VID newVID, interTreeVID;
    CfgType newCfg, interTreeCfg;
    ExpandTree(dir, treeA, m_delta, newVID, newCfg);

    if(newVID != INVALID_VID) {

      m_totalSuccess++;
      //treeA->push_back(newVID);  we add VID to the tree in Expand

      // Since expand goes until collision or goal is detected, we shouldnt iterate.
      connected = ExpandTree(newCfg, newVID, treeB, MAX_DBL, interTreeVID, interTreeCfg, true);

    } else {
      m_totalFailure++;
    }

    // Switching trees
    swap(treeA, treeB);
    iter++;
  }

}

template<class MPTraits>
bool
RRTConnect<MPTraits>::ExpandTree(CfgType& _dir, vector<VID>* _targetTree, double _delta, VID& _newVID, CfgType& _newCfg){

  return ExpandTree(_dir, INVALID_VID, _targetTree, _delta, _newVID, _newCfg, false);
}

template<class MPTraits>
bool
RRTConnect<MPTraits>::ExpandTree(CfgType& _dir, const VID& _dirVID, vector<VID>* _targetTree, double _delta, VID& _newVID, CfgType& _newCfg, bool _interTree){
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod);
  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  CDInfo  cdInfo;
  // Find closest Cfg in map
  vector<VID> kClosest;
  vector<CfgType> cfgs;

  StatClass* kcloseStatClass = this->GetMPProblem()->GetStatClass();
  string kcloseClockName = "kclosest time ";
  kcloseStatClass->StartClock(kcloseClockName);
  // Choose the closest node from the three    
  nf->KClosest(rdmp, _targetTree->begin(), _targetTree->end(), _dir, 1, back_inserter(kClosest));
  kcloseStatClass->StopClock(kcloseClockName);
  
  cout << endl << "Direction: " << _dir << endl;
  cout << "Direction VID: " << _dirVID << endl;
  cout << "Target Tree:" << endl;
  for(int i=0; i<_targetTree->size(); i++) {
    cout << (*_targetTree)[i] << " ";
  
  }
  cout << endl;

  bool connected = false;

  _newVID = INVALID_VID;
  const CfgType& nearest = rdmp->GetGraph()->GetCfg(kClosest[0]);
  int weight;
  cout << "qnear: " << kClosest[0] << "\t" << nearest << endl; 

  StatClass* expandStatClass = this->GetMPProblem()->GetStatClass();
  string expandClockName = "RRTConnect time ";
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
      
      this->m_localGraph->add_vertex(_newVID, _newCfg);
      if(this->m_debug) VDAddNode(_newCfg);
      #endif
    } 

    cout << "qnew: " << _newVID << "\t" << _newCfg << endl;
    
    
    pair<WeightType, WeightType> weights = make_pair(WeightType("RRTConnect", weight), WeightType("RRTConnect", weight));
    
    #ifndef _PARALLEL
    rdmp->GetGraph()->AddEdge(kClosest[0], _newVID, weights);
    #else 
    WeightType weightT("RRTExpand", weight);
    GraphType* globalTree = rdmp->GetGraph();
    globalTree->add_edge_async(kClosest[0], _newVID, weightT);
    globalTree->add_edge_async(_newVID, kClosest[0], weightT);
    
    this->m_localGraph->add_edge(kClosest[0],_newVID);
    this->m_localGraph->add_edge(_newVID, kClosest[0]);
    if(this->m_debug) VDAddEdge(nearest, _newCfg);
    #endif
    _targetTree->push_back(_newVID); 
  } 

  return connected;
}




template<class MPTraits>
typename MPTraits::CfgType 
RRTConnect<MPTraits>::SelectDirection(){
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CfgType dir;
  dir.GetRandomCfg(env);
  return dir;
}

///////////////////////////////////////////////////////////////////////////////
#endif

