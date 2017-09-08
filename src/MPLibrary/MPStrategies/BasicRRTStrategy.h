#ifndef BASIC_RRT_STRATEGY_H_
#define BASIC_RRT_STRATEGY_H_

#include <iomanip>
#include "MPStrategyMethod.h"
#include "MPLibrary/MapEvaluators/RRTQuery.h"


////////////////////////////////////////////////////////////////////////////////
/// The RRT algorithm grows one or more trees from a set of root nodes to solve
/// a single-query planning problem.
///
/// Our not-so-basic RRT offers many variations by setting the appropriate
/// options:
/// @arg m_growGoals      Grow from goals?
/// @arg m_gt             Indicates DIRECTED vs. UNDIRECTED and GRAPH vs. TREE.
///
/// The original RRT reference: LaValle, Steven M. "Rapidly-Exploring Random
///                             Trees: A New Tool for Path Planning." TR 98-11,
///                             Computer Science Dept., Iowa State Univ., 1998.
///
/// @ingroup MotionPlanningStrategies
/// @internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class BasicRRTStrategy : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::vector<VID> TreeType;

    ///@}
    ///@name Construction
    ///@{

    BasicRRTStrategy(string _dm = "euclidean", string _nf = "bfnf",
        string _vc = "rapid", string _nc = "kClosest", string _ex = "BERO",
        vector<string> _evaluators = vector<string>(),
        string _gt = "UNDIRECTED_TREE",  bool _growGoals = false,
        double _growthFocus = .05, size_t _numRoots = 1,
        size_t _numDirections = 1, size_t _maxTrial = 3);

    BasicRRTStrategy(XMLNode& _node);

    virtual ~BasicRRTStrategy() = default;

    ///@}
    ///@name MPBaseObject overrides
    ///@{

    virtual void Print(ostream& _os) const;

    ///@}
    ///@name MPStrategy Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Iterate() override;
    virtual void Finalize() override;

    ///@}

  protected:

    ///@name Direction Helpers
    ///@{

    /// Get a random configuration to grow towards.
    virtual CfgType SelectTarget();

    /// Sample a target configuration to grow towards from an existing
    ///        configuration. m_maxTrial samples are attempted.
    /// @param _v The VID of the existing configuration.
    /// @return The sample who's growth direction yields the greatest separation
    ///         from the existing configuration's neighbors.
    CfgType SelectDispersedTarget(VID _v);

    ///@}
    ///@name Neighbor Helpers
    ///@{

    /// Get the configurations that are adjacent to _v in the map.
    vector<CfgType> SelectNeighbors(VID _v);

    /// Find the nearest configuration to the target _cfg within _tree.
    virtual VID FindNearestNeighbor(const CfgType& _cfg, const TreeType& _tree);

    /// If the graph type is GRAPH, try to connect a configuration to its
    /// neighbors. No-op for TREE type graph.
    /// @param _newVID The VID of the configuration to connect.
    void ConnectNeighbors(VID _newVID);

    ///@}
    ///@name Growth Helpers
    ///@{

    /// Extend a new configuration from a nearby configuration towards a growth
    /// target.
    /// @param _nearVID The nearby configuration's VID.
    /// @param _target  The growth target.
    /// @param _lp An LPOutput for returning local planner info.
    /// @return The new node's VID.
    virtual VID Extend(const VID _nearVID, const CfgType& _target,
        LPOutput<MPTraits>& _lp);

    /// @overload
    VID Extend(const VID _nearVID, const CfgType& _target);

    /// Attempt to connect one roadmap configuration to another with the extender.
    /// @param _start The starting node.
    /// @param _target The desired ending node.
    /// @return True if the edge between _start and _target was created
    ///         successfully.
    virtual bool ExtendLP(const VID _start, const VID _target);

    /// Add a new configuration to the roadmap and current tree.
    /// @param _newCfg    The new configuration to add.
    /// @return A pair with the added VID and a bool indicating whether the new
    ///         node was already in the map.
    virtual pair<VID, bool> AddNode(const CfgType& _newCfg);

    /// Add a new edge to the roadmap.
    /// @param _source   The source node.
    /// @param _target   The target node.
    /// @param _lpOutput The extender output.
    void AddEdge(VID _source, VID _target, const LPOutput<MPTraits>& _lpOutput);

    ///@}
    ///@name Tree Helpers
    ///@{

    /// Attempt to expand the map by growing towards a target
    ///        configuration from the nearest existing node in the current tree.
    /// @param _target The target configuration.
    /// @return            The VID of a newly created Cfg if successful,
    ///                    INVALID_VID otherwise.
    virtual VID ExpandTree(CfgType& _target);

    /// Attempt to expand the map by growing towards a target
    ///        configuration from an arbitrary node.
    /// @param _nearestVID The VID to grow from.
    /// @param _target     The target configuration.
    /// @return                The VID of a newly created Cfg if successful,
    ///                        INVALID_VID otherwise.
    virtual VID ExpandTree(const VID _nearestVID, const CfgType& _target);

    /// If multiple trees exist, try to connect the current tree with the
    ///        one that is nearest to a recently grown configuration.
    /// @param _recentlyGrown The VID of the recently grown configuration.
    void ConnectTrees(VID _recentlyGrown);

    /// Check that each node in the graph is also in a tree, and that the
    ///        number of trees is equal to the number of connected components.
    ///        Calls RebuildTrees to correct if either check fails.
    void ValidateTrees();

    /// Reconstruct m_trees from the roadmap CC info.
    void RebuildTrees();

    ///@}
    ///@name MP Object Labels
    ///@{

    string m_dmLabel;       ///< The distance metric label.
    string m_samplerLabel;  ///< The sampler label.
    string m_nfLabel;       ///< The neighborhood finder label.
    string m_vcLabel;       ///< The validity checker label.
    string m_ncLabel;       ///< The connector label.
    string m_exLabel;       ///< The extender label.
    string m_gt;            ///< The graph type.

    ///@}
    ///@name RRT Properties
    ///@{

    bool   m_growGoals;     ///< Grow trees from goals.
    double m_growthFocus;   ///< The fraction of goal-biased expansions.
    size_t m_numRoots;      ///< The number of roots to use without a query.
    size_t m_numDirections; ///< The number of expansion directions per iteration.
    size_t m_maxTrial;      ///< The number of samples taken for disperse search.

    ///@}
    ///@name Tree Data
    ///@{

    vector<TreeType> m_trees;                          ///< The current tree set.
    typename vector<TreeType>::iterator m_currentTree; ///< The working tree.

    ///@}
    ///@name Extension Success Tracking
    ///@{

    size_t m_successes{0};  ///< The count of successful extensions.
    size_t m_trials{0};     ///< The count of attempted extensions.

    ///@}
    ///@name Query
    ///@{

    RRTQuery<MPTraits>* m_query{nullptr}; ///< The query object.

    ///@}
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
BasicRRTStrategy<MPTraits>::
BasicRRTStrategy(string _dm, string _nf, string _vc, string _nc,
    string _ex, vector<string> _evaluators, string _gt, bool _growGoals,
    double _growthFocus, size_t _numRoots, size_t _numDirections,
    size_t _maxTrial) :
    m_dmLabel(_dm), m_nfLabel(_nf), m_vcLabel(_vc), m_ncLabel(_nc),
    m_exLabel(_ex), m_gt(_gt), m_growGoals(_growGoals),
    m_growthFocus(_growthFocus), m_numRoots(_numRoots),
    m_numDirections(_numDirections), m_maxTrial(_maxTrial) {
  this->SetName("BasicRRTStrategy");
  this->m_meLabels = _evaluators;
}


template <typename MPTraits>
BasicRRTStrategy<MPTraits>::
BasicRRTStrategy(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("BasicRRTStrategy");

  // Parse RRT parameters
  m_gt = _node.Read("gtype", true, "", "Graph type dir/undirected tree/graph");
  m_numRoots = _node.Read("numRoots", false, 1, 0, MAX_INT, "Number of Roots");
  m_growthFocus = _node.Read("growthFocus", false, 0.0, 0.0, 1.0,
      "Fraction of goal-biased iterations");
  m_numDirections = _node.Read("m", false, 1, 1, 1000,
      "Number of directions to extend");
  m_growGoals = _node.Read("growGoals", false, false,
      "Determines whether or not we grow a tree from the goal");
  m_maxTrial = _node.Read("trial", false, 3, 1, 1000,
      "Number of trials to get a dispersed direction");

  // Parse MP object labels
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");
  m_dmLabel = _node.Read("dmLabel",true,"","Distance Metric");
  m_ncLabel = _node.Read("connectorLabel", false, "", "Node Connection Method");
  m_exLabel = _node.Read("extenderLabel", true, "", "Extender label");
  m_samplerLabel = _node.Read("samplerLabel", true, "", "Sampler Label");

  // Parse child nodes.
  for(auto& child : _node)
    if(child.Name() == "Evaluator")
      this->m_meLabels.push_back(child.Read("label", true, "",
          "Evaluation Method"));
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
Print(ostream& _os) const {
  _os << "BasicRRTStrategy::Print" << endl
      << "  MP objects:" << endl
      << "\tDistance Metric:: " << m_dmLabel << endl
      << "\tSampler:: " << m_samplerLabel << endl
      << "\tNeighborhood Finder:: " << m_nfLabel << endl
      << "\tValidity Checker:: " << m_vcLabel << endl
      << "\tConnection Method:: " << m_ncLabel << endl
      << "\tExtender:: " << m_exLabel << endl
      << "\tEvaluators:: " << endl;
  for(auto& s : this->m_meLabels)
    _os << "\t\t" << s << endl;

  _os << "  RRT properties:" << endl
      << "\tGraph Type:: " << m_gt << endl
      << "\tGrow Goals:: " << m_growGoals << endl
      << "\tnumber of roots:: " << m_numRoots << endl
      << "\tgrowth focus:: " << m_growthFocus << endl
      << "\tnumber of expansion directions:: " << m_numDirections << endl;
}

/*-------------------------- MPStrategy overrides ----------------------------*/

template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
Initialize() {
  if(this->m_debug)
    cout << "Initializing BasicRRTStrategy" << endl;

  // Clear all state variables to avoid problems when running multiple times.
  m_trees.clear();
  m_successes = 0;
  m_trials = 0;

  GraphType* g = this->GetRoadmap()->GetGraph();

  // Check for query info.
  m_query = nullptr;

  // If a query was loaded, process query cfgs
  if(this->UsingQuery()) {
    m_query = static_cast<RRTQuery<MPTraits>*>(this->GetMapEvaluator("RRTQuery").
        get());
    const vector<CfgType>& queryCfgs = m_query->GetQuery();

    // If growing goals, set each query cfg as its own tree
    if(m_growGoals) {
      for(auto& cfg : queryCfgs) {
        VID add = g->AddVertex(cfg);
        m_trees.push_back(vector<VID>(1, add));
      }
    }

    // If not growing goals, add only the start to map.
    else {
      VID start = g->AddVertex(queryCfgs.front());
      m_trees.push_back(vector<VID>(1, start));
    }
  }
  // If no query loaded, make m_numRoots random roots
  else {
    for(size_t i = 0; i < m_numRoots; ++i) {
      auto env = this->GetEnvironment();
      auto vc  = this->GetValidityChecker(m_vcLabel);

      CfgType root(this->GetTask()->GetRobot());
      do {
        root.GetRandomCfg(env);
      } while(!root.InBounds(env) || !vc->IsValid(root, "BasicRRTStrategy"));

      VID rootVID = g->AddVertex(root);
      m_trees.push_back(vector<VID>(1, rootVID));
    }
  }

  // Set initial tree to be grown
  m_currentTree = m_trees.begin();

  // Output debugging info if requested
  if(this->m_debug) {
    cout << "There are " << m_trees.size() << " trees"
         << (m_trees.empty() ? "." : ":") << endl;
    for(size_t i = 0; i < m_trees.size(); ++i) {
      cout << "\tTree " << i << " has " << m_trees[i].size() << " vertices.\n";
      if(!m_trees[i].empty())
        cout << "\t\tIts root is: " << g->GetVertex(m_trees[i].front()) << endl;
    }
  }
}


template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
Iterate() {
  ++m_trials;
  if(this->m_debug)
    cout << "*** Starting iteration " << m_trials << " "
         << "***************************************************" << endl
         << "Graph has " << this->GetRoadmap()->GetGraph()->get_num_vertices()
         << " vertices." << std::endl;

  // Find my growth direction.
  CfgType target = this->SelectTarget();

  // Randomize Current Tree
  m_currentTree = m_trees.begin() + LRand() % m_trees.size();
  if(this->m_debug)
    cout << "Randomizing current tree:" << endl
         << "\tm_trees.size() = " << m_trees.size() << ", currentTree = "
         << distance(m_trees.begin(), m_currentTree) << ", |currentTree| = "
         << m_currentTree->size() << endl;

  // Ensure that all nodes in the graph are also in the RRT trees, and that
  // numTrees == numCCs
  ValidateTrees();

  // Find the nearest configuration to target within the current tree
  VID nearestVID = FindNearestNeighbor(target, *m_currentTree);

  // Expand current tree
  VID newVID = this->ExpandTree(nearestVID, target);
  if(newVID != INVALID_VID)
    ++m_successes;
}


template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
Finalize() {
  // Output final map
  RoadmapType* map = this->GetRoadmap();
  string baseFilename = this->GetBaseFilename();
  map->Write(baseFilename + ".map", this->GetEnvironment());

  // Output stats
  ofstream osStat(baseFilename + ".stat");
  this->GetStatClass()->PrintAllStats(osStat, map);
}

/*--------------------------- Direction Helpers ------------------------------*/

template <typename MPTraits>
typename MPTraits::CfgType
BasicRRTStrategy<MPTraits>::
SelectTarget() {
  CfgType target(this->GetTask()->GetRobot());

  // Select goal growth with probability m_growthFocus.
  const bool unreachedGoals = m_query && !m_query->GetGoals().empty();
  if(unreachedGoals && DRand() < m_growthFocus) {
    target = m_query->GetRandomGoal();
    if(this->m_debug)
      std::cout << "Goal-biased growth target selected: " << target.PrettyPrint()
                << std::endl;
  }
  // Otherwise, use uniform random sampling.
  else {
    auto s = this->GetSampler(m_samplerLabel);

    std::vector<CfgType> samples;
    s->Sample(1, 1, this->GetEnvironment()->GetBoundary(),
        std::back_inserter(samples));
    target = samples.front();

    if(this->m_debug)
      std::cout << "Random growth target selected: " << target.PrettyPrint()
                << std::endl;
  }

  return target;
}


template <typename MPTraits>
typename MPTraits::CfgType
BasicRRTStrategy<MPTraits>::
SelectDispersedTarget(VID _v) {
  /// \warning Should be named something like SelectDispersionTarget as this does
  ///          not return a direction.
  StatClass* stats = this->GetStatClass();
  stats->StartClock("BasicRRT::DisperseSampling");

  // Get original cfg with vid _v and its neighbors
  CfgType originalCfg = this->GetRoadmap()->GetGraph()->GetVertex(_v);
  vector<CfgType> neighbors = SelectNeighbors(_v);

  // Look for the best extension directio, which is the direction with the
  // largest angular separation from any neighbor.
  CfgType bestCfg(this->GetTask()->GetRobot());
  double maxAngle = -MAX_DBL;
  for(size_t i = 0; i < m_maxTrial; ++i) {
    // Get a random configuration
    CfgType randCfg(this->GetTask()->GetRobot());
    randCfg.GetRandomCfg(this->GetEnvironment());

    // Get the unit direction toward randCfg
    CfgType randDir = randCfg - originalCfg;
    randDir /= randDir.Magnitude();

    // Calculate the minimum angular separation between randDir and the
    // unit directions to originalCfg's neighbors
    double minAngle = MAX_DBL;
    for(auto& neighbor : neighbors) {
      // Get the unit direction toward neighbor
      CfgType neighborDir = neighbor - originalCfg;
      neighborDir /= neighborDir.Magnitude();

      // Compute the angle between randDir and neighborDir
      double sum{0};
      for(size_t j = 0; j < originalCfg.DOF(); ++j)
        sum += randDir[j] * neighborDir[j];
      double angle = acos(sum);

      // Update minimum angle
      minAngle = min(minAngle, angle);
    }

    // Now minAngle is the smallest angle between randDir and any neighborDir.
    // Keep the randDir that produces the largest minAngle.
    if(maxAngle < minAngle) {
      maxAngle = minAngle;
      bestCfg = randCfg;
    }
  }

  stats->StopClock("BasicRRT::DisperseSampling");
  return bestCfg;
}

/*---------------------------- Neighbor Helpers ------------------------------*/

template <typename MPTraits>
vector<typename MPTraits::CfgType>
BasicRRTStrategy<MPTraits>::
SelectNeighbors(VID _v) {
  GraphType* g = this->GetRoadmap()->GetGraph();
  typename GraphType::vertex_iterator vi = g->find_vertex(_v);
  vector<CfgType> vec;
  for(const auto& e : *vi)
    vec.push_back(g->GetVertex(e.target()));
  return vec;
}


template <typename MPTraits>
typename BasicRRTStrategy<MPTraits>::VID
BasicRRTStrategy<MPTraits>::
FindNearestNeighbor(const CfgType& _cfg, const TreeType& _tree) {
  this->GetStatClass()->StartClock("BasicRRT::NeighborhoodFinding");

  vector<pair<VID, double>> neighbors;

  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  nf->FindNeighbors(this->GetRoadmap(),
      _tree.begin(), _tree.end(),
      _tree.size() == this->GetRoadmap()->GetGraph()->get_num_vertices(),
      _cfg, back_inserter(neighbors));

  VID nearestVID = INVALID_VID;

  if(!neighbors.empty())
    nearestVID = neighbors[0].first;

  this->GetStatClass()->StopClock("BasicRRT::NeighborhoodFinding");
  return nearestVID;
}


template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
ConnectNeighbors(VID _newVID) {
  // Make sure _newVID is valid and graph type includes GRAPH
  if(_newVID == INVALID_VID || m_gt.find("GRAPH") == std::string::npos)
    return;

  this->GetStatClass()->StartClock("BasicRRT::ConnectNeighbors");

  vector<VID> currentVID(1, _newVID);
  this->GetConnector(m_ncLabel)->Connect(this->GetRoadmap(),
      currentVID.begin(), currentVID.end(),
      m_currentTree->begin(), m_currentTree->end(),
      m_currentTree->size() ==
      this->GetRoadmap()->GetGraph()->get_num_vertices());

  this->GetStatClass()->StopClock("BasicRRT::ConnectNeighbors");
}

/*----------------------------- Growth Helpers -------------------------------*/

template <typename MPTraits>
typename BasicRRTStrategy<MPTraits>::VID
BasicRRTStrategy<MPTraits>::
Extend(const VID _nearVID, const CfgType& _target) {
  LPOutput<MPTraits> dummyLP;
  return this->Extend(_nearVID, _target, dummyLP);
}


template <typename MPTraits>
typename BasicRRTStrategy<MPTraits>::VID
BasicRRTStrategy<MPTraits>::
Extend(const VID _nearVID, const CfgType& _target, LPOutput<MPTraits>& _lp) {
  MethodTimer mt(this->GetStatClass(), "BasicRRT::Extend");
  this->GetStatClass()->IncStat("BasicRRTExtend");

  auto e = this->GetExtender(m_exLabel);
  const CfgType& qNear = this->GetRoadmap()->GetGraph()->GetVertex(_nearVID);
  CfgType qNew(this->GetTask()->GetRobot());

  const bool success = e->Extend(qNear, _target, qNew, _lp);
  if(this->m_debug)
    std::cout << "\tExtended "
              << std::setprecision(4) << _lp.m_edge.first.GetWeight()
              << " units."
              << std::endl;

  if(!success) {
    // The extension failed to exceed the minimum distance.
    if(this->m_debug)
      std::cout << "\tNode too close, not adding." << std::endl;
    return INVALID_VID;
  }

  // The extension succeeded. Try to add the node.
  const auto extension = AddNode(qNew);

  const bool nodeIsNew = extension.second;
  if(!nodeIsNew) {
    // The extension reproduced an existing node.
    if(this->m_debug)
      std::cout << "\tNode already exists, not adding." << std::endl;
    return INVALID_VID;
  }

  // The extension was ok.
  const VID& newVID = extension.first;

  AddEdge(_nearVID, newVID, _lp);
  ConnectNeighbors(newVID);

  return newVID;
}


template <typename MPTraits>
bool
BasicRRTStrategy<MPTraits>::
ExtendLP(const VID _start, const VID _target) {
  MethodTimer mt(this->GetStatClass(), "BasicRRT::ExtendLP");
  this->GetStatClass()->IncStat("BasicRRTExtendLP");

  auto e = this->GetExtender(m_exLabel);
  const CfgType& start  = this->GetRoadmap()->GetGraph()->GetVertex(_start);
  const CfgType& target = this->GetRoadmap()->GetGraph()->GetVertex(_target);
  CfgType qNew(this->GetTask()->GetRobot());
  LPOutput<MPTraits> lp;

  const bool success = e->Extend(start, target, qNew, lp);
  if(this->m_debug)
    std::cout << "\tExtended "
              << std::setprecision(4) << lp.m_edge.first.GetWeight()
              << " units."
              << std::endl;

  if(!success) {
    if(this->m_debug)
      cout << "\tLP extension failed." << endl;
    return false;
  }

  // If we arrived at the goal, this is a valid local plan.
  const bool validLP = qNew == target;

  if(validLP)
    AddEdge(_start, _target, lp);
  else {
    const auto add = AddNode(qNew);
    const bool nodeIsNew = add.second;
    if(nodeIsNew)
      AddEdge(_start, add.first, lp);
  }

  return validLP;
}


template <typename MPTraits>
pair<typename BasicRRTStrategy<MPTraits>::VID, bool>
BasicRRTStrategy<MPTraits>::
AddNode(const CfgType& _newCfg) {
  MethodTimer mt(this->GetStatClass(), "BasicRRT::AddNode");

  GraphType* g = this->GetRoadmap()->GetGraph();

  VID lastVID = g->GetLastVID();
  VID newVID = g->AddVertex(_newCfg);

  const bool nodeIsNew = lastVID != g->GetLastVID();
  if(nodeIsNew) {
    m_currentTree->push_back(newVID);
    if(this->m_debug)
      cout << "\tAdding VID " << newVID << " to tree "
           << distance(m_trees.begin(), m_currentTree) << "." << endl;
  }
  else if(this->m_debug)
    cout << "\tVID " << newVID << " already exists, not adding." << endl;

  return make_pair(newVID, nodeIsNew);
}


template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
AddEdge(VID _source, VID _target, const LPOutput<MPTraits>& _lpOutput) {
  GraphType* g = this->GetRoadmap()->GetGraph();
  if(m_growGoals || m_gt.find("UNDIRECTED") != std::string::npos)
    g->AddEdge(_source, _target, _lpOutput.m_edge);
  else
    g->AddEdge(_source, _target, _lpOutput.m_edge.first);
  g->GetVertex(_target).SetStat("Parent", _source);

  if(this->m_debug)
    cout << "\tAdding Edge (" << _source << ", " << _target << ")." << endl;
}

/*------------------------------ Tree Helpers --------------------------------*/

template <typename MPTraits>
typename BasicRRTStrategy<MPTraits>::VID
BasicRRTStrategy<MPTraits>::
ExpandTree(CfgType& _target) {
  VID nearestVID = FindNearestNeighbor(_target, *m_currentTree);

  if(nearestVID == INVALID_VID)
    return false;

  return this->ExpandTree(nearestVID, _target);
}


template <typename MPTraits>
typename BasicRRTStrategy<MPTraits>::VID
BasicRRTStrategy<MPTraits>::
ExpandTree(const VID _nearestVID, const CfgType& _target) {
  if(this->m_debug)
    cout << "Trying expansion from node " << _nearestVID << " "
         << this->GetRoadmap()->GetGraph()->GetVertex(_nearestVID).PrettyPrint()
         << "..." << endl;

  // Try to extend from the _nearestVID to _target
  VID newVID = this->Extend(_nearestVID, _target);
  if(newVID != INVALID_VID)
    ConnectTrees(newVID);
  else
    return INVALID_VID;

  // Expand to other directions
  for(size_t i = 1; i < m_numDirections; ++i) {
    if(this->m_debug)
      cout << "Expanding to other directions (" << i << "/"
           << m_numDirections - 1 << "):: ";
    CfgType randCfg = SelectDispersedTarget(_nearestVID);
    VID otherVID = this->Extend(_nearestVID, randCfg);
    if(otherVID != INVALID_VID)
      ConnectTrees(otherVID);
  }

  return newVID;
}


template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
ConnectTrees(VID _recentlyGrown) {
  // Return if only one tree
  if(m_trees.size() == 1)
    return;

  // Setup MP variables
  GraphType* g = this->GetRoadmap()->GetGraph();
  auto dm = this->GetDistanceMetric(m_dmLabel);

  // Get qNew from its VID
  const CfgType& qNew = g->GetVertex(_recentlyGrown);

  // Find the closest neighbor to qNew in all other trees
  double minDist = MAX_DBL;
  VID closestVID = INVALID_VID;
  auto closestTree = m_currentTree;
  for(auto trit = m_trees.begin(); trit != m_trees.end(); ++trit) {
    // Skip current tree and empty trees.
    if(trit == m_currentTree or trit->empty())
      continue;

    // Find nearest neighbor to qNew in other tree
    VID nearestVID = FindNearestNeighbor(qNew, *trit);
    CfgType nearestCfg = g->GetVertex(nearestVID);
    double dist = dm->Distance(qNew, nearestCfg);

    // If nearest is the closest to qNew, save it as closest
    if(dist < minDist) {
      minDist = dist;
      closestTree = trit;
      closestVID = nearestVID;
    }
  }

  // If the closest VID is still invalid, abort.
  if(closestVID == INVALID_VID) {
    if(this->m_debug)
      std::cout << "Connecting trees, all trees except current are empty!"
                << std::endl;
    return;
  }

  if(this->m_debug)
    cout << "Connecting trees: from (tree "
         << distance(m_trees.begin(), closestTree) << ", VID "
         << closestVID << ") to (tree "
         << distance(m_trees.begin(), m_currentTree) << ", VID "
         << _recentlyGrown << "), distance = " << setw(4) << minDist << endl;

  // Try to expand from closestVID (in closestTree) to qNew (in m_currentTree)
  // in order to connect the trees.
  swap(m_currentTree, closestTree);

  if(this->ExtendLP(closestVID, _recentlyGrown)) {
    // The extension reached all the way to qNew. Merge the closest and current
    // trees.
    if(this->m_debug)
      cout << "\tConnectTrees succeeded!" << endl;

    // Merge trees into the lower of the two indexes
    if(distance(m_trees.begin(), m_currentTree) >
        distance(m_trees.begin(), closestTree))
      swap(m_currentTree, closestTree);
    m_currentTree->insert(m_currentTree->end(), closestTree->begin(),
        closestTree->end());
    m_trees.erase(closestTree);
  }
  else {
    // The extension didn't connect the trees. Swap back to the original tree.
    if(this->m_debug)
      cout << "\tConnectTrees failed: could not expand all the way." << endl;
    swap(m_currentTree, closestTree);
  }
}


template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
ValidateTrees() {
  // Count nodes in trees
  size_t numNodesInTrees = 0;
  for(auto& tree : m_trees)
    numNodesInTrees += tree.size();

  // Rebuild if nodes are missing from trees
  GraphType* g = this->GetRoadmap()->GetGraph();
  if(numNodesInTrees > g->get_num_vertices()) {
    RebuildTrees();
    return;
  }

  // Rebuild if numTrees != numCCs
  vector<pair<size_t, VID>> ccs;
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  get_cc_stats(*g, cmap, ccs);
  if(ccs.size() != m_trees.size())
    RebuildTrees();
}


template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
RebuildTrees() {
  m_trees.clear();

  // Get cc info from roadmap
  GraphType* g = this->GetRoadmap()->GetGraph();
  vector<pair<size_t, VID>> ccs;
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  get_cc_stats(*g, cmap, ccs);

  // Rebuild tree list from cc info
  vector<VID> ccVIDs;
  for(auto& cc : ccs) {
    cmap.reset();
    ccVIDs.clear();
    get_cc(*g, cmap, cc.second, ccVIDs);
    m_trees.push_back(ccVIDs);
  }

  m_currentTree = m_trees.begin();
}

#endif
