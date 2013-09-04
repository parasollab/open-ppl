#ifndef SRTSTRATEGY_H_
#define SRTSTRATEGY_H_

#include "MPStrategyMethod.h"
#include "Utilities/RRTExpand.h"

template<class MPTraits>
class SRTStrategy : public MPStrategyMethod<MPTraits> {
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

    //Non-XML constructor sets all private variables
    SRTStrategy();
    SRTStrategy(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

    virtual void PrintOptions(ostream& _os) const;

  protected:
    //Helper functions

    //general SRT functions
    void GenerateTrees();
    void ExpandTrees();
    void FindCandidateConnections(vector<pair<VID, VID> >& _candPairs);
    void ConnectTrees(vector<pair<VID, VID> >& _candPairs);

    //connect trees functions
    bool Connect(VID _t1, VID _t2);
    bool RRTConnect(VID _t1, VID _t2);

    //RRT helpers
    CfgType SelectDirection();
    virtual VID ExpandTree(VID _tree, CfgType& _dir);

    vector<string> m_evaluators;
    string m_lpLabel;
    string m_dmLabel;
    string m_nfLabel;
    string m_vcLabel;
    double m_delta, m_minDist;
    size_t m_numSamples; //"k" random trees per iteration
    size_t m_numExpansions; //"m" expansion iterations per tree
    size_t m_numCloseCent; //"n_c" closest centroids
    size_t m_numRandCent; //"n_r" random centroids
    size_t m_numClosePairs; //"n_p" closest pairs
    size_t m_numConnIter; //"n_i" tree connection iterations

    typedef pair<CfgType, vector<VID> > Tree; //centroid, tree pair
    typedef map<VID, Tree> Trees; //node representative, tree mapping
    Trees m_trees;

    Query<MPTraits>* m_query; //temporary for loading q_s and q_g to spark trees
};

template<class MPTraits>
SRTStrategy<MPTraits>::SRTStrategy() {
  this->SetName("SRTStrategy");
}

template<class MPTraits>
SRTStrategy<MPTraits>::SRTStrategy(MPProblemType* _problem, XMLNodeReader& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node), m_query(NULL) {
    this->SetName("SRTStrategy");
    ParseXML(_node);
    _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
void
SRTStrategy<MPTraits>::ParseXML(XMLNodeReader& _node) {
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr){
    if(citr->getName() == "Evaluator"){
      string evalMethod = citr->stringXMLParameter("label", true, "", "Evaluation Method");
      m_evaluators.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    }
    else
      citr->warnUnknownNode();
  }

  m_delta = _node.numberXMLParameter("delta", false, 1.0, 0.0, MAX_DBL, "Delta Distance");
  m_minDist = _node.numberXMLParameter("minDist", false, 0.0, 0.0, m_delta, "Minimum Distance");
  m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");
  m_nfLabel = _node.stringXMLParameter("nfLabel", true, "", "Neighborhood Finder");
  m_dmLabel = _node.stringXMLParameter("dmLabel",true,"","Distance Metric");
  m_lpLabel = _node.stringXMLParameter("lpLabel", true, "", "Local Planning Method");

  m_numSamples = _node.numberXMLParameter("samples", true, 100, 0, MAX_INT, "k random trees per iteration");
  m_numExpansions = _node.numberXMLParameter("expansions", true, 10, 0, MAX_INT, "m expansions per random tree");
  m_numCloseCent = _node.numberXMLParameter("closeCent", true, 10, 0, MAX_INT, "n_c close centroids for edge attempts");
  m_numRandCent = _node.numberXMLParameter("randCent", true, 10, 0, MAX_INT, "n_r close centroids for edge attempts");
  m_numClosePairs = _node.numberXMLParameter("closePairs", true, 10, 0, MAX_INT, "n_p close pairs for connections");
  m_numConnIter = _node.numberXMLParameter("connIter", true, 10, 0, MAX_INT, "n_i iterations during tree connection");

  //optionally read in a query and create a Query object.
  string query = _node.stringXMLParameter("query", false, "", "Query Filename");
  if(!query.empty()) {
    cout << "New query is created" << endl;
    m_query = new Query<MPTraits>(query);
  }
}

template<class MPTraits>
void
SRTStrategy<MPTraits>::PrintOptions(ostream& _os) const {
  _os << "SRTStrategy::PrintOptions" << endl;
  _os << "\tNeighborhood Finder:: " << m_nfLabel << endl;
  _os << "\tDistance Metric:: " << m_dmLabel << endl;
  _os << "\tValidity Checker:: " << m_vcLabel << endl;
  _os << "\tLocal Planner:: " << m_lpLabel << endl;
  _os << "\tEvaluators:: " << endl;
  typedef vector<string>::const_iterator SIT;
  for(SIT sit = m_evaluators.begin(); sit!=m_evaluators.end(); sit++)
    _os << "\t\t" << *sit << endl;
  _os << "\tdelta:: " << m_delta << endl;
  _os << "\tminimum distance:: " << m_minDist << endl;
}

//////////////////////
//Initialization Phase
/////////////////////
template<class MPTraits>
void
SRTStrategy<MPTraits>::Initialize(){
  if(this->m_debug) cout<<"\nInitializing SRTStrategy"<<endl;

  // Setup SRT Variables
  if(m_query) {
    vector<CfgType>& queryCfgs = m_query->GetQuery();
    typedef typename vector<CfgType>::iterator CIT;
    cout << queryCfgs.size() << endl;
    for(CIT cit = queryCfgs.begin(); cit != queryCfgs.end(); cit++){
      VID v = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(*cit);
      m_trees[v] = Tree(*cit, vector<VID>(1, v));
      if(this->m_debug) cout << "Adding Cfg::" << *cit << " from query." << endl;
    }
    delete m_query;
  }

  if(this->m_debug) cout<<"\nEnding Initializing SRTStrategy"<<endl;
}

////////////////
//Run/Start Phase
////////////////
template<class MPTraits>
void
SRTStrategy<MPTraits>::Run() {
  if(this->m_debug) cout << "\nRunning SRTStrategy" << endl;

  // Setup MP Variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  if(this->m_recordKeep) stats->StartClock("SRT Generation");

  do {
    //grow "k" randomly placed trees for "m" iterations
    GenerateTrees();
    ExpandTrees();

    //determine edge candidates
    vector<pair<VID, VID> > connectionCandidates;
    FindCandidateConnections(connectionCandidates);

    //attempt edge candidates
    ConnectTrees(connectionCandidates);
  } while(!this->EvaluateMap(m_evaluators));

  if(this->m_recordKeep) stats->StopClock("SRT Generation");
  if(this->m_debug) {
    stats->PrintClock("SRT Generation", cout);
    cout<<"\nEnd Running SRTStrategy::" << endl;
  }
}

/////////////////////
//Finalization phase
////////////////////
template<class MPTraits>
void
SRTStrategy<MPTraits>::Finalize() {

  if(this->m_debug) cout<<"\nFinalizing SRTStrategy::"<<endl;

  //setup variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  string str;

  //output final map
  str = this->GetBaseFilename() + ".map";
  ofstream osMap(str.c_str());
  this->GetMPProblem()->GetRoadmap()->Write(osMap, this->GetMPProblem()->GetEnvironment());
  osMap.close();

  //output stats
  str = this->GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  osStat << "NodeGen+Connection Stats" << endl;
  stats->PrintAllStats(osStat, this->GetMPProblem()->GetRoadmap());
  stats->PrintClock("SRT Generation", osStat);
  osStat.close();

  if(this->m_debug) cout<<"\nEnd Finalizing SRTStrategy"<<endl;
}

template<class MPTraits>
void
SRTStrategy<MPTraits>::GenerateTrees() {
  if(this->m_debug) cout << "\nBegin GenerateTrees" << endl;

  //set up MP variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  ValidityCheckerPointer vcp = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  CDInfo cdInfo;

  string callee = this->GetNameAndLabel() + "::GenerateTrees";

  for(size_t i = 0; i < m_numSamples; ++i) {
    //generate random cfg in C-free
    CfgType tmp;
    do {
      tmp.GetRandomCfg(env);
    } while(!(env->InBounds(tmp) &&
          vcp->IsValid(tmp, env, *this->GetMPProblem()->GetStatClass(),
            cdInfo, &callee)));

    //create a new tree rooted at cfg
    VID v = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(tmp);
    m_trees[v] = Tree(tmp, vector<VID>(1, v));
  }

  if(this->m_debug) cout << "\nEnd GenerateTrees" << endl;
}

template<class MPTraits>
void
SRTStrategy<MPTraits>::ExpandTrees() {
  if(this->m_debug) cout << "\nBegin ExpandTrees" << endl;

  CfgType dir;

  typedef typename Trees::iterator TIT;
  for(TIT tit = m_trees.begin(); tit!=m_trees.end(); ++tit) {

    for(size_t i = 0; i < m_numExpansions; ++i) {
      CfgType dir = this->SelectDirection();
      VID recent = this->ExpandTree(tit->first, dir);
    }
  }

  if(this->m_debug) cout << "\nEnd ExpandTrees" << endl;
}

template<class MPTraits>
void
SRTStrategy<MPTraits>::FindCandidateConnections(vector<pair<VID, VID> >& _candPairs) {
  if(this->m_debug) cout << "\nBegin FindCandidateConnections" << endl;

  _candPairs.clear();

  NeighborhoodFinderPointer nfp = this->GetMPProblem()->GetNeighborhoodFinder(m_nfLabel);
  typedef typename Trees::iterator TIT;

  vector<VID> reps;
  RoadmapType centRdmp;
  for(TIT tit = m_trees.begin(); tit!=m_trees.end(); ++tit) {
    reps.push_back(tit->first);
    centRdmp.GetGraph()->add_vertex(tit->first, tit->second.first);
  }

  for(TIT tit = m_trees.begin(); tit!=m_trees.end(); ++tit) {
    set<VID> cands;

    //find n_c closest centroids
    vector<pair<VID, double> > closest;
    size_t oldK = nfp->GetK();
    nfp->GetK() = m_numCloseCent;
    nfp->FindNeighbors(&centRdmp, tit->second.first, back_inserter(closest));
    nfp->GetK() = oldK;
    typedef typename vector<pair<VID, double> >::iterator CIT;
    for(CIT cit = closest.begin(); cit!=closest.end(); ++cit)
      cands.insert(cit->first);

    //find n_r random centroids
    random_shuffle(reps.begin(), reps.end());
    cands.insert(reps.begin(), (m_numRandCent < m_trees.size()) ? reps.begin() + m_numRandCent : reps.end());

    //generate pairs for the tree
    typedef typename set<VID>::iterator VIT;
    for(VIT vit = cands.begin(); vit!=cands.end(); ++vit)
      _candPairs.push_back(make_pair(tit->first, *vit));
  }

  if(this->m_debug) cout << "\nEnd FindCandidateConnections" << endl;
}

template<class MPTraits>
void
SRTStrategy<MPTraits>::ConnectTrees(vector<pair<VID, VID> >& _candPairs) {
  if(this->m_debug) cout << "\nBegin ConnectTrees" << endl;

  stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> cmap;
  GraphType* g = this->GetMPProblem()->GetRoadmap()->GetGraph();
  vector<pair<VID, VID> > succPair;

  typedef typename vector<pair<VID, VID> >::iterator PIT;
  for(PIT pit = _candPairs.begin(); pit!=_candPairs.end(); ++pit) {
    if(this->m_debug) cout << "Connecting trees " << pit->first << "::" << pit->second << endl;

    //check if in same cc first
    cmap.reset();
    if(!stapl::sequential::is_same_cc(*g, cmap, pit->first, pit->second)) {
      //attempt k-closest pairs connection
      //then attempt RRT-connect upon failure
      if(Connect(pit->first, pit->second) || RRTConnect(pit->first, pit->second)) {
        if(this->m_debug) cout << "Successful connection." << endl;
        succPair.push_back(*pit);
      }
    }
  }

  //merge all the trees and update centroids
  for(PIT pit = succPair.begin(); pit!=succPair.end(); ++pit) {
    VID minv = min(pit->first, pit->second);
    VID maxv = max(pit->first, pit->second);

    //merge the trees
    size_t s1 = m_trees[minv].second.size();
    size_t s2 = m_trees[maxv].second.size();
    m_trees[minv].second.insert(m_trees[minv].second.end(), m_trees[maxv].second.begin(), m_trees[maxv].second.end());
    m_trees[minv].first = (m_trees[minv].first * s1 + m_trees[maxv].first * s2)/(s1+s2);

    //redo succPair vals for rest of pairs
    for(PIT pit2 = pit; pit2 != succPair.end(); ++pit2) {
      if(pit2->first == maxv)
        pit2->first = minv;
      if(pit2->second == maxv)
        pit2->second = minv;
    }

    //remove tree 2
    m_trees.erase(maxv);
  }

  if(this->m_debug) cout << "\nEnd ConnectTrees" << endl;
}

template<class MPTraits>
bool
SRTStrategy<MPTraits>::Connect(VID _t1, VID _t2) {

  Environment* env = this->GetMPProblem()->GetEnvironment();
  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  DistanceMetricPointer dmp = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
  NeighborhoodFinderPointer nfp = this->GetMPProblem()->GetNeighborhoodFinder(m_nfLabel);
  LocalPlannerPointer lpp = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);
  LPOutput<MPTraits> lpOutput;

  Tree& t1 = m_trees[_t1];
  Tree& t2 = m_trees[_t2];

  //find n_p closest pairs
  vector<pair<pair<VID, VID>, double> > closest;
  size_t oldK = nfp->GetK();
  nfp->GetK() = m_numClosePairs;
  nfp->FindNeighborPairs(rdmp,
      t1.second.begin(), t1.second.end(),
      t2.second.begin(), t2.second.end(),
      back_inserter(closest));
  nfp->GetK() = oldK;

  //attempt local plan
  typedef typename vector<pair<pair<VID, VID>, double> >::iterator CIT;
  for(CIT cit = closest.begin(); cit!=closest.end(); ++cit) {
    CfgType& c1 = rdmp->GetGraph()->GetVertex(cit->first.first);
    CfgType& c2 = rdmp->GetGraph()->GetVertex(cit->first.second);
    CfgType col;

    if(lpp->IsConnected(env, *this->GetMPProblem()->GetStatClass(), dmp,
          c1, c2, col,
          &lpOutput, env->GetPositionRes(), env->GetOrientationRes())) {
      //successful connection add graph edge
      rdmp->GetGraph()->AddEdge(cit->first.first, cit->first.second, lpOutput.edge);
      return true;
    }
  }

  return false;
}

template<class MPTraits>
bool
SRTStrategy<MPTraits>::RRTConnect(VID _t1, VID _t2) {

  for(size_t i = 0; i < m_numConnIter; ++i) {
    CfgType dir = SelectDirection();

    //if not trapped (RRTExtend is successful)
    VID v = ExpandTree(_t1, dir);
    if(v != INVALID_VID) {
      //greedily extend t2 towards t1
      CfgType qnew = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVertex(v);
      VID v2;
      do
        v2 = ExpandTree(_t2, qnew);
      while(v2 != INVALID_VID && v2 != v);
      if(v2 == v)
        return true;
    }
    swap(_t1, _t2);
  }

  return false;
}

template<class MPTraits>
typename MPTraits::CfgType
SRTStrategy<MPTraits>::SelectDirection(){
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CfgType dir;
  dir.GetRandomCfg(env);
  return dir;
}

template<class MPTraits>
typename SRTStrategy<MPTraits>::VID
SRTStrategy<MPTraits>::ExpandTree(VID _tree, CfgType& _dir) {

  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nfLabel);
  CDInfo cdInfo;

  VID recentVID = INVALID_VID;
  // Find closest Cfg in map
  vector<pair<VID, double> > kClosest;
  vector<CfgType> cfgs;

  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  GraphType* g = rdmp->GetGraph();
  Tree& currentTree = m_trees[_tree];

  nf->FindNeighbors(rdmp,
      currentTree.second.begin(), currentTree.second.end(),
      _dir, back_inserter(kClosest));

  CfgType& nearest = g->GetVertex(kClosest[0].first);

  CfgType newCfg;
  int weight;

  if(!RRTExpand<MPTraits>(this->GetMPProblem(), m_vcLabel, m_dmLabel,
        nearest, _dir, newCfg, m_delta,
        weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
    if(this->m_debug) cout << "RRT could not expand!" << endl;
    return recentVID;
  }

  if(this->m_debug) cout << "RRT expanded to " << newCfg << endl;

  // If good to go, add to roadmap
  if(dm->Distance(env, newCfg, nearest) >= m_minDist ) {
    recentVID = g->AddVertex(newCfg);
    currentTree.second.push_back(recentVID);

    //update centroid
    size_t s = currentTree.second.size();
    currentTree.first = (currentTree.first * (s-1) + newCfg) / s;

    //add edge
    pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", weight), WeightType("RRTExpand", weight));
    g->AddEdge(kClosest[0].first, recentVID, weights);
    g->GetVertex(recentVID).SetStat("Parent", kClosest[0].first);
  }

  return recentVID;
}

#endif
