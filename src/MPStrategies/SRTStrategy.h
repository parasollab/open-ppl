#ifndef SRT_STRATEGY_H_
#define SRT_STRATEGY_H_

#include "MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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
    SRTStrategy(MPProblemType* _problem, XMLNode& _node);

    virtual void ParseXML(XMLNode& _node);

    virtual void Initialize();
    virtual void Iterate();
    virtual void Finalize();

    virtual void Print(ostream& _os) const;

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

    string m_lpLabel;
    string m_dmLabel;
    string m_nfLabel;
    string m_vcLabel;
    string m_eLabel;
    double m_delta, m_minDist;
    size_t m_numSamples; //"k" random trees per iteration
    size_t m_numExpansions; //"m" expansion iterations per tree
    size_t m_numCloseCent; //"n_c" closest centroids
    size_t m_numRandCent; //"n_r" random centroids
    size_t m_numClosePairs; //"n_p" closest pairs
    size_t m_numConnIter; //"n_i" tree connection iterations
    size_t m_iteration;   ///< Number of iterations so far.

    typedef pair<CfgType, vector<VID> > Tree; //centroid, tree pair
    typedef map<VID, Tree> Trees; //node representative, tree mapping
    Trees m_trees;

    Query<MPTraits>* m_query; //temporary for loading q_s and q_g to spark trees
};

template<class MPTraits>
SRTStrategy<MPTraits>::
SRTStrategy() : m_iteration(0) {
  this->SetName("SRTStrategy");
}

template<class MPTraits>
SRTStrategy<MPTraits>::
SRTStrategy(MPProblemType* _problem, XMLNode& _node) :
    MPStrategyMethod<MPTraits>(_problem, _node), m_iteration(0), m_query(NULL) {
  this->SetName("SRTStrategy");
  ParseXML(_node);
}

template<class MPTraits>
void
SRTStrategy<MPTraits>::
ParseXML(XMLNode& _node) {
  for(auto& child : _node)
    if(child.Name() == "Evaluator")
      this->m_meLabels.push_back(
          child.Read("label", true, "", "Evaluation Method"));

  m_delta = _node.Read("delta", false, 1.0, 0.0, MAX_DBL, "Delta Distance");
  m_minDist = _node.Read("minDist", false, 0.0, 0.0, m_delta,
      "Minimum Distance");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");
  m_dmLabel = _node.Read("dmLabel",true,"", "Distance Metric");
  m_lpLabel = _node.Read("lpLabel", true, "", "Local Planning Method");
  m_eLabel = _node.Read("eLabel", true, "", "Extender Method");

  m_numSamples = _node.Read("samples", true, 100, 0, MAX_INT,
      "k random trees per iteration");
  m_numExpansions = _node.Read("expansions", true, 10, 0, MAX_INT,
      "m expansions per random tree");
  m_numCloseCent = _node.Read("closeCent", true, 10, 0, MAX_INT,
      "n_c close centroids for edge attempts");
  m_numRandCent = _node.Read("randCent", true, 10, 0, MAX_INT,
      "n_r close centroids for edge attempts");
  m_numClosePairs = _node.Read("closePairs", true, 10, 0, MAX_INT,
      "n_p close pairs for connections");
  m_numConnIter = _node.Read("connIter", true, 10, 0, MAX_INT,
      "n_i iterations during tree connection");

  //optionally read in a query and create a Query object.
  string query = _node.Read("query", false, "", "Query Filename");
  if(!query.empty())
    m_query = new Query<MPTraits>(query);
}

template<class MPTraits>
void
SRTStrategy<MPTraits>::
Print(ostream& _os) const {
  _os << "SRTStrategy::Print" << endl;
  _os << "\tNeighborhood Finder:: " << m_nfLabel << endl;
  _os << "\tDistance Metric:: " << m_dmLabel << endl;
  _os << "\tValidity Checker:: " << m_vcLabel << endl;
  _os << "\tLocal Planner:: " << m_lpLabel << endl;
  _os << "\tExtender:: " << m_eLabel << endl;
  _os << "\tEvaluators:: " << endl;
  for(const auto& label: this->m_meLabels)
    _os << "\t\t" << label << endl;
  _os << "\tdelta:: " << m_delta << endl;
  _os << "\tminimum distance:: " << m_minDist << endl;
}


template<class MPTraits>
void
SRTStrategy<MPTraits>::
Initialize() {
  if(this->m_debug)
    cout<<"\nInitializing SRTStrategy"<<endl;

  // Setup SRT Variables
  if(m_query) {
    vector<CfgType>& queryCfgs = m_query->GetQuery();
    typedef typename vector<CfgType>::iterator CIT;
    for(CIT cit = queryCfgs.begin(); cit != queryCfgs.end(); cit++){
      VID v = this->GetRoadmap()->GetGraph()->AddVertex(*cit);
      m_trees[v] = Tree(*cit, vector<VID>(1, v));
      if(this->m_debug)
        cout << "Adding Cfg::" << *cit << " from query." << endl;
    }
    delete m_query;
  }

  if(this->m_debug)
    cout<<"\nEnding Initializing SRTStrategy"<<endl;
}


template<class MPTraits>
void
SRTStrategy<MPTraits>::
Iterate() {
  cout << "Starting iteration " << ++m_iteration << endl;
  //grow "k" randomly placed trees for "m" iterations
  GenerateTrees();
  ExpandTrees();

  //determine edge candidates
  vector<pair<VID, VID> > connectionCandidates;
  FindCandidateConnections(connectionCandidates);

  //attempt edge candidates
  ConnectTrees(connectionCandidates);
}


template<class MPTraits>
void
SRTStrategy<MPTraits>::
Finalize() {
  if(this->m_debug)
    cout<<"\nFinalizing SRTStrategy::"<<endl;

  //output final map
  this->GetRoadmap()->Write(this->GetBaseFilename() + ".map", this->GetEnvironment());

  //output stats
  StatClass* stats = this->GetStatClass();
  string str = this->GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  osStat << "NodeGen+Connection Stats" << endl;
  stats->PrintAllStats(osStat, this->GetRoadmap());
  stats->PrintClock("SRT Generation", osStat);
  osStat.close();

  if(this->m_debug)
    cout<<"\nEnd Finalizing SRTStrategy"<<endl;
}

template<class MPTraits>
void
SRTStrategy<MPTraits>::
GenerateTrees() {
  if(this->m_debug)
    cout << "\nBegin GenerateTrees" << endl;

  //set up MP variables
  Environment* env = this->GetEnvironment();
  ValidityCheckerPointer vcp = this->GetValidityChecker(m_vcLabel);
  CDInfo cdInfo;

  string callee = this->GetNameAndLabel() + "::GenerateTrees";

  for(size_t i = 0; i < m_numSamples; ++i) {
    //generate random cfg in C-free
    CfgType tmp;
    do {
      tmp.GetRandomCfg(env);
    } while(!(env->InBounds(tmp) && vcp->IsValid(tmp, cdInfo, callee)));

    //create a new tree rooted at cfg
    VID v = this->GetRoadmap()->GetGraph()->AddVertex(tmp);
    m_trees[v] = Tree(tmp, vector<VID>(1, v));
  }

  if(this->m_debug)
    cout << "\nEnd GenerateTrees" << endl;
}

template<class MPTraits>
void
SRTStrategy<MPTraits>::
ExpandTrees() {
  if(this->m_debug)
    cout << "\nBegin ExpandTrees" << endl;

  CfgType dir;

  typedef typename Trees::iterator TIT;
  for(TIT tit = m_trees.begin(); tit!=m_trees.end(); ++tit) {
    while(tit->second.second.size() < m_numExpansions) {
      CfgType dir = this->SelectDirection();
      VID recent = this->ExpandTree(tit->first, dir);
    }
  }

  if(this->m_debug)
    cout << "\nEnd ExpandTrees" << endl;
}

template<class MPTraits>
void
SRTStrategy<MPTraits>::
FindCandidateConnections(vector<pair<VID, VID> >& _candPairs) {
  if(this->m_debug)
    cout << "\nBegin FindCandidateConnections" << endl;

  _candPairs.clear();

  NeighborhoodFinderPointer nf = this->GetNeighborhoodFinder(m_nfLabel);
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
    size_t oldK = nf->GetK();
    nf->GetK() = m_numCloseCent;
    nf->FindNeighbors(&centRdmp, tit->second.first, back_inserter(closest));
    nf->GetK() = oldK;
    typedef typename vector<pair<VID, double> >::iterator CIT;
    for(CIT cit = closest.begin(); cit!=closest.end(); ++cit)
      cands.insert(cit->first);

    //find n_r random centroids
    random_shuffle(reps.begin(), reps.end());
    cands.insert(reps.begin(), (m_numRandCent < m_trees.size()) ? reps.begin() +
        m_numRandCent : reps.end());

    //generate pairs for the tree
    typedef typename set<VID>::iterator VIT;
    for(VIT vit = cands.begin(); vit!=cands.end(); ++vit)
      _candPairs.push_back(make_pair(tit->first, *vit));
  }

  if(this->m_debug)
    cout << "\nEnd FindCandidateConnections" << endl;
}

template<class MPTraits>
void
SRTStrategy<MPTraits>::
ConnectTrees(vector<pair<VID, VID> >& _candPairs) {
  if(this->m_debug)
    cout << "\nBegin ConnectTrees" << endl;

  stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> cMap;
  GraphType* g = this->GetRoadmap()->GetGraph();
  vector<pair<VID, VID> > succPair;

  typedef typename vector<pair<VID, VID> >::iterator PIT;
  for(PIT pit = _candPairs.begin(); pit!=_candPairs.end(); ++pit) {
    if(this->m_debug)
      cout << "Connecting trees " << pit->first << "::" << pit->second << endl;

    //check if in same cc first
    cMap.reset();
    if(!stapl::sequential::is_same_cc(*g, cMap, pit->first, pit->second)) {
      //attempt k-closest pairs connection
      //then attempt RRT-connect upon failure
      if(Connect(pit->first, pit->second) || RRTConnect(pit->first,
          pit->second)) {
        if(this->m_debug)
          cout << "Successful connection." << endl;
        succPair.push_back(*pit);
      }
    }
  }

  //merge all the trees and update centroids
  for(PIT pit = succPair.begin(); pit!=succPair.end(); ++pit) {
    VID minV = min(pit->first, pit->second);
    VID maxV = max(pit->first, pit->second);

    //merge the trees
    size_t s1 = m_trees[minV].second.size();
    size_t s2 = m_trees[maxV].second.size();
    m_trees[minV].second.insert(m_trees[minV].second.end(),
        m_trees[maxV].second.begin(), m_trees[maxV].second.end());
    m_trees[minV].first = (m_trees[minV].first * s1 + m_trees[maxV].first * s2)/
        (s1+s2);

    //redo succPair vals for rest of pairs
    for(PIT pit2 = pit; pit2 != succPair.end(); ++pit2) {
      if(pit2->first == maxV)
        pit2->first = minV;
      if(pit2->second == maxV)
        pit2->second = minV;
    }

    //remove tree 2
    m_trees.erase(maxV);
  }

  if(this->m_debug)
    cout << "\nEnd ConnectTrees" << endl;
}

template<class MPTraits>
bool
SRTStrategy<MPTraits>::
Connect(VID _t1, VID _t2) {

  Environment* env = this->GetEnvironment();
  RoadmapType* rdmp = this->GetRoadmap();
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dmLabel);
  NeighborhoodFinderPointer nf = this->GetNeighborhoodFinder(m_nfLabel);
  LocalPlannerPointer lpp = this->GetLocalPlanner(m_lpLabel);
  LPOutput<MPTraits> lpOutput;

  Tree& t1 = m_trees[_t1];
  Tree& t2 = m_trees[_t2];

  //find n_p closest pairs
  vector<pair<pair<VID, VID>, double> > closest;
  size_t oldK = nf->GetK();
  nf->GetK() = m_numClosePairs;
  nf->FindNeighborPairs(rdmp,
      t1.second.begin(), t1.second.end(),
      t2.second.begin(), t2.second.end(),
      back_inserter(closest));
  nf->GetK() = oldK;

  //attempt local plan
  typedef typename vector<pair<pair<VID, VID>, double> >::iterator CIT;
  for(CIT cit = closest.begin(); cit!=closest.end(); ++cit) {
    CfgType& c1 = rdmp->GetGraph()->GetVertex(cit->first.first);
    CfgType& c2 = rdmp->GetGraph()->GetVertex(cit->first.second);
    CfgType col;

    if(lpp->IsConnected(c1, c2, col,
          &lpOutput, env->GetPositionRes(), env->GetOrientationRes())) {
      //successful connection add graph edge
      rdmp->GetGraph()->AddEdge(cit->first.first, cit->first.second,
          lpOutput.m_edge);
      return true;
    }
  }

  return false;
}

template<class MPTraits>
bool
SRTStrategy<MPTraits>::
RRTConnect(VID _t1, VID _t2) {

  for(size_t i = 0; i < m_numConnIter; ++i) {
    CfgType dir = SelectDirection();

    //if not trapped (RRTExtend is successful)
    VID v = ExpandTree(_t1, dir);
    if(v != INVALID_VID) {
      //greedily extend t2 towards t1
      CfgType qNew = this->GetRoadmap()->GetGraph()->GetVertex(v);
      VID v2 = v, v2repeated;
      do {
        v2repeated = v2;
        v2 = ExpandTree(_t2, qNew);
      } while(v2 != INVALID_VID && v2 != v && v2 != v2repeated);
      if(v2 == v)
        return true;
    }
    swap(_t1, _t2);
  }

  return false;
}

template<class MPTraits>
typename MPTraits::CfgType
SRTStrategy<MPTraits>::
SelectDirection(){
  Environment* env = this->GetEnvironment();
  CfgType dir;
  dir.GetRandomCfg(env);
  return dir;
}

template<class MPTraits>
typename SRTStrategy<MPTraits>::VID
SRTStrategy<MPTraits>::
ExpandTree(VID _tree, CfgType& _dir) {

  // Setup MP Variables
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dmLabel);
  NeighborhoodFinderPointer nf = this->GetNeighborhoodFinder(m_nfLabel);
  CDInfo cdInfo;

  VID recentVID = INVALID_VID;
  // Find closest Cfg in map
  vector<pair<VID, double> > kClosest;
  vector<CfgType> cfgs;

  RoadmapType* rdmp = this->GetRoadmap();
  GraphType* g = rdmp->GetGraph();
  Tree& currentTree = m_trees[_tree];

  nf->FindNeighbors(rdmp,
      currentTree.second.begin(), currentTree.second.end(), false,
      _dir, back_inserter(kClosest));

  CfgType& nearest = g->GetVertex(kClosest[0].first);

  CfgType newCfg;
  int weight = 0;

  typename MPProblemType::ExtenderPointer e = this->GetExtender(m_eLabel);
  LPOutput<MPTraits> lpOutput;
  if(!e->Extend(nearest, _dir, newCfg, lpOutput)) {
    if(this->m_debug)
      cout << "RRT could not expand!" << endl;
    return recentVID;
  }

  if(this->m_debug)
    cout << "RRT expanded to " << newCfg << endl;

  // If good to go, add to roadmap
  if(dm->Distance(newCfg, nearest) >= m_minDist ) {
    recentVID = g->AddVertex(newCfg);
    currentTree.second.push_back(recentVID);

    //update centroid
    size_t s = currentTree.second.size();
    currentTree.first = (currentTree.first * (s-1) + newCfg) / s;

    //add edge
    pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand",
        weight), WeightType("RRTExpand", weight));
    g->AddEdge(kClosest[0].first, recentVID, weights);
    g->GetVertex(recentVID).SetStat("Parent", kClosest[0].first);
  }

  return recentVID;
}

#endif
