#ifndef SRT_STRATEGY_H_
#define SRT_STRATEGY_H_

#include "MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief This method is the 'Sampling-based Roadmap of Trees', which grows
///        several RRT trees and connects them together to build the roadmap.
///
/// The earliest reference to this work is:
///   Bekris, Chen, Ladd, Plaku, and Kavraki. "Multiple query probabilistic
///     roadmap planning using single query planning primitives". Proc. of the
///     International Conference on Intelligent Robots and Systems (IROS), 2003.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class SRTStrategy : public MPStrategyMethod<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///\name Local Types
    ///@{

    typedef std::pair<CfgType, std::vector<VID> > Tree; ///< Centroid, tree pair.
    typedef std::map<VID, Tree> Trees;             ///< Node->tree mapping.

    ///@}
    ///\name Construction
    ///@{

    SRTStrategy();
    SRTStrategy(XMLNode& _node);

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///\name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Iterate() override;
    virtual void Finalize() override;

    ///@}

  protected:

    ///\name Helpers
    ///@{

    //general SRT functions
    void GenerateTrees();
    void ExpandTrees();
    void FindCandidateConnections(std::vector<std::pair<VID, VID> >& _candPairs);
    void ConnectTrees(std::vector<std::pair<VID, VID> >& _candPairs);

    //connect trees functions
    bool Connect(VID _t1, VID _t2);
    bool RRTConnect(VID _t1, VID _t2);

    //RRT helpers
    virtual CfgType SelectTarget();
    virtual VID ExpandTree(const VID _tree, const CfgType& _dir);

    ///@}
    ///\name MP Object Labels
    ///@{

    std::string m_lpLabel{"sl"};
    std::string m_dmLabel{"euclidean"};
    std::string m_nfLabel{"Nearest"};
    std::string m_vcLabel{"pqp_solid"};
    std::string m_exLabel{"BERO"};

    ///@}
    ///\name SRT Parameters
    ///@{

    size_t m_numSamples;    ///< "k" random trees per iteration
    size_t m_numExpansions; ///< "m" expansion iterations per tree
    size_t m_numCloseCent;  ///< "n_c" closest centroids
    size_t m_numRandCent;   ///< "n_r" random centroids
    size_t m_numClosePairs; ///< "n_p" closest pairs
    size_t m_numConnIter;   ///< "n_i" tree connection iterations

    ///@}
    ///\name
    ///@{

    size_t m_iteration{0};  ///< Number of iterations so far.
    Trees m_trees;          ///< The trees in the roadmap.

    ///@}
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
SRTStrategy<MPTraits>::
SRTStrategy() : MPStrategyMethod<MPTraits>() {
  this->SetName("SRTStrategy");
}


template <typename MPTraits>
SRTStrategy<MPTraits>::
SRTStrategy(XMLNode& _node) :
    MPStrategyMethod<MPTraits>(_node) {
  this->SetName("SRTStrategy");
  ParseXML(_node);
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
SRTStrategy<MPTraits>::
ParseXML(XMLNode& _node) {
  for(auto& child : _node)
    if(child.Name() == "Evaluator")
      this->m_meLabels.push_back(
          child.Read("label", true, "", "Evaluation Method"));

  m_vcLabel = _node.Read("vcLabel", false, m_vcLabel, "Validity Test Method");
  m_nfLabel = _node.Read("nfLabel", false, m_nfLabel, "Neighborhood Finder");
  m_dmLabel = _node.Read("dmLabel", false, m_dmLabel, "Distance Metric");
  m_lpLabel = _node.Read("lpLabel", false, m_lpLabel, "Local Planning Method");
  m_exLabel = _node.Read("exLabel", false, m_exLabel, "Extender Method");

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
}


template <typename MPTraits>
void
SRTStrategy<MPTraits>::
Print(std::ostream& _os) const {
  _os << "SRTStrategy::Print" << std::endl;
  _os << "\tNeighborhood Finder:: " << m_nfLabel << std::endl;
  _os << "\tDistance Metric:: " << m_dmLabel << std::endl;
  _os << "\tValidity Checker:: " << m_vcLabel << std::endl;
  _os << "\tLocal Planner:: " << m_lpLabel << std::endl;
  _os << "\tExtender:: " << m_exLabel << std::endl;
  _os << "\tEvaluators:: " << std::endl;
  for(const auto& label: this->m_meLabels)
    _os << "\t\t" << label << std::endl;
}

/*------------------------ MPStrategyMethod Overrides ------------------------*/

template <typename MPTraits>
void
SRTStrategy<MPTraits>::
Initialize() {
  if(this->m_debug)
    std::cout<<"\nInitializing SRTStrategy"<<std::endl;

  // Setup SRT Variables
  if(this->UsingQuery()) {
    auto g = this->GetRoadmap()->GetGraph();

    PRMQuery<MPTraits>* query = static_cast<PRMQuery<MPTraits>*>(
        this->GetMapEvaluator("PRMQuery").get());

    for(const auto& cfg : query->GetQuery()) {
      VID v = g->AddVertex(cfg);
      m_trees[v] = Tree(cfg, std::vector<VID>{v});
      if(this->m_debug)
        std::cout << "Adding Cfg::" << cfg << " from query." << std::endl;
    }
  }

  if(this->m_debug)
    std::cout<<"\nEnding Initializing SRTStrategy"<<std::endl;
}


template <typename MPTraits>
void
SRTStrategy<MPTraits>::
Iterate() {
  std::cout << "Starting iteration " << ++m_iteration << std::endl;
  //grow "k" randomly placed trees for "m" iterations
  GenerateTrees();
  ExpandTrees();

  //determine edge candidates
  std::vector<std::pair<VID, VID> > connectionCandidates;
  FindCandidateConnections(connectionCandidates);

  //attempt edge candidates
  ConnectTrees(connectionCandidates);
}


template <typename MPTraits>
void
SRTStrategy<MPTraits>::
Finalize() {
  if(this->m_debug)
    std::cout<<"\nFinalizing SRTStrategy::"<<std::endl;

  //output final map
  this->GetRoadmap()->Write(this->GetBaseFilename() + ".map",
      this->GetEnvironment());

  //output stats
  StatClass* stats = this->GetStatClass();
  std::string str = this->GetBaseFilename() + ".stat";
  std::ofstream  osStat(str.c_str());
  osStat << "NodeGen+Connection Stats" << std::endl;
  stats->PrintAllStats(osStat, this->GetRoadmap());
  stats->PrintClock("SRT Generation", osStat);
  osStat.close();

  if(this->m_debug)
    std::cout<<"\nEnd Finalizing SRTStrategy"<<std::endl;
}

/*------------------------------- Helpers ------------------------------------*/

template <typename MPTraits>
void
SRTStrategy<MPTraits>::
GenerateTrees() {
  if(this->m_debug)
    std::cout << "\nBegin GenerateTrees" << std::endl;

  Environment* env = this->GetEnvironment();
  auto vc = this->GetValidityChecker(m_vcLabel);

  for(size_t i = 0; i < m_numSamples; ++i) {
    // Generate random cfg in C-free.
    CfgType cfg(this->GetTask()->GetRobot());
    do {
      cfg.GetRandomCfg(env);
    } while(!cfg.InBounds(env) ||
        !vc->IsValid(cfg, "SRTStrategy::GenerateTrees"));

    // Create a new tree rooted at cfg.
    VID v = this->GetRoadmap()->GetGraph()->AddVertex(cfg);
    m_trees[v] = Tree(cfg, std::vector<VID>{v});
  }

  if(this->m_debug)
    std::cout << "\nEnd GenerateTrees" << std::endl;
}


template <typename MPTraits>
void
SRTStrategy<MPTraits>::
ExpandTrees() {
  if(this->m_debug)
    std::cout << "\nBegin ExpandTrees" << std::endl;

  for(auto& tree : m_trees)
    while(tree.second.second.size() < m_numExpansions)
      this->ExpandTree(tree.first, this->SelectTarget());

  if(this->m_debug)
    std::cout << "\nEnd ExpandTrees" << std::endl;
}


template <typename MPTraits>
void
SRTStrategy<MPTraits>::
FindCandidateConnections(std::vector<std::pair<VID, VID> >& _candPairs) {
  if(this->m_debug)
    std::cout << "\nBegin FindCandidateConnections" << std::endl;

  _candPairs.clear();

  std::vector<VID> reps;
  RoadmapType centRdmp(this->GetTask()->GetRobot());
  for(auto& tree : m_trees) {
    reps.push_back(tree.first);
    centRdmp.GetGraph()->AddVertex(tree.first, tree.second.first);
  }

  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  for(auto& tree : m_trees) {
    std::set<VID> cands;

    // Find n_c closest centroids.
    std::vector<Neighbor> closest;
    size_t oldK = nf->GetK();
    nf->GetK() = m_numCloseCent;
    nf->FindNeighbors(&centRdmp, tree.second.first, std::back_inserter(closest));
    nf->GetK() = oldK;
    for(auto& neighbor : closest)
      cands.insert(neighbor.target);

    // Find n_r random centroids.
    random_shuffle(reps.begin(), reps.end());
    cands.insert(reps.begin(), (m_numRandCent < m_trees.size()) ?
        reps.begin() + m_numRandCent : reps.end());

    // Generate pairs for the tree.
    for(auto& vid : cands)
      _candPairs.push_back(std::make_pair(tree.first, vid));
  }

  if(this->m_debug)
    std::cout << "\nEnd FindCandidateConnections" << std::endl;
}


template <typename MPTraits>
void
SRTStrategy<MPTraits>::
ConnectTrees(std::vector<std::pair<VID, VID>>& _candPairs) {
  if(this->m_debug)
    std::cout << "\nBegin ConnectTrees" << std::endl;

  stapl::sequential::vector_property_map<typename GraphType::STAPLGraph, size_t>
      cMap;
  GraphType* g = this->GetRoadmap()->GetGraph();
  std::vector<std::pair<VID, VID> > succPair;

  for(auto& pair : _candPairs) {
    if(this->m_debug)
      std::cout << "Connecting trees " << pair.first << "::" << pair.second << std::endl;

    //check if in same cc first
    cMap.reset();
    if(!stapl::sequential::is_same_cc(*g, cMap, pair.first, pair.second)) {
      //attempt k-closest pairs connection
      //then attempt RRT-connect upon failure
      if(Connect(pair.first, pair.second) || RRTConnect(pair.first,
            pair.second)) {
        if(this->m_debug)
          std::cout << "Successful connection." << std::endl;
        succPair.push_back(pair);
      }
    }
  }

  //merge all the trees and update centroids
  for(auto pit = succPair.begin(); pit != succPair.end(); ++pit) {
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
    for(auto pit2 = pit; pit2 != succPair.end(); ++pit2) {
      if(pit2->first == maxV)
        pit2->first = minV;
      if(pit2->second == maxV)
        pit2->second = minV;
    }

    //remove tree 2
    m_trees.erase(maxV);
  }

  if(this->m_debug)
    std::cout << "\nEnd ConnectTrees" << std::endl;
}


template <typename MPTraits>
bool
SRTStrategy<MPTraits>::
Connect(VID _t1, VID _t2) {
  Environment* env = this->GetEnvironment();
  RoadmapType* rdmp = this->GetRoadmap();
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  auto lp = this->GetLocalPlanner(m_lpLabel);
  LPOutput<MPTraits> lpOutput;

  Tree& t1 = m_trees[_t1];
  Tree& t2 = m_trees[_t2];

  //find n_p closest pairs
  std::vector<Neighbor> closest;
  size_t oldK = nf->GetK();
  nf->GetK() = m_numClosePairs;
  nf->FindNeighborPairs(rdmp,
      t1.second.begin(), t1.second.end(),
      t2.second.begin(), t2.second.end(),
      std::back_inserter(closest));
  nf->GetK() = oldK;

  //attempt local plan
  for(auto cit = closest.begin(); cit != closest.end(); ++cit) {
    CfgType& c1 = rdmp->GetGraph()->GetVertex(cit->source);
    CfgType& c2 = rdmp->GetGraph()->GetVertex(cit->target);
    CfgType col(this->GetTask()->GetRobot());

    if(lp->IsConnected(c1, c2, col,
          &lpOutput, env->GetPositionRes(), env->GetOrientationRes())) {
      //successful connection add graph edge
      rdmp->GetGraph()->AddEdge(cit->source, cit->target,
          lpOutput.m_edge);
      return true;
    }
  }

  return false;
}


template <typename MPTraits>
bool
SRTStrategy<MPTraits>::
RRTConnect(VID _t1, VID _t2) {
  for(size_t i = 0; i < m_numConnIter; ++i) {
    CfgType dir = this->SelectTarget();

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


template <typename MPTraits>
typename MPTraits::CfgType
SRTStrategy<MPTraits>::
SelectTarget() {
  CfgType target(this->GetTask()->GetRobot());
  target.GetRandomCfg(this->GetEnvironment());

  if(this->m_debug)
    std::cout << "Random growth target selected: " << target << std::endl;
  return target;
}


template <typename MPTraits>
typename SRTStrategy<MPTraits>::VID
SRTStrategy<MPTraits>::
ExpandTree(const VID _tree, const CfgType& _dir) {
  // Setup MP Variables
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  CDInfo cdInfo;

  VID recentVID = INVALID_VID;
  // Find closest Cfg in map
  std::vector<Neighbor> kClosest;
  std::vector<CfgType> cfgs;

  RoadmapType* rdmp = this->GetRoadmap();
  GraphType* g = rdmp->GetGraph();
  Tree& currentTree = m_trees[_tree];

  nf->FindNeighbors(rdmp,
      currentTree.second.begin(), currentTree.second.end(), false,
      _dir, std::back_inserter(kClosest));

  CfgType& nearest = g->GetVertex(kClosest[0].target);

  CfgType newCfg(this->GetTask()->GetRobot());
  int weight = 0;

  auto e = this->GetExtender(m_exLabel);
  LPOutput<MPTraits> lpOutput;
  if(!e->Extend(nearest, _dir, newCfg, lpOutput)) {
    if(this->m_debug)
      std::cout << "RRT could not expand!" << std::endl;
    return recentVID;
  }

  if(this->m_debug)
    std::cout << "RRT expanded to " << newCfg << std::endl;

  // If good to go, add to roadmap
  if(dm->Distance(newCfg, nearest) >= e->GetMinDistance()) {
    recentVID = g->AddVertex(newCfg);
    currentTree.second.push_back(recentVID);

    //update centroid
    size_t s = currentTree.second.size();
    currentTree.first = (currentTree.first * (s-1) + newCfg) / s;

    //add edge
    std::pair<WeightType, WeightType> weights = std::make_pair(WeightType("RRTExpand",
        weight), WeightType("RRTExpand", weight));
    g->AddEdge(kClosest[0].target, recentVID, weights);
    g->GetVertex(recentVID).SetStat("Parent", kClosest[0].target);
  }

  return recentVID;
}

/*----------------------------------------------------------------------------*/

#endif
