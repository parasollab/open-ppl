#ifndef DISASSEMBLY_RRT_STRATEGY_H_
#define DISASSEMBLY_RRT_STRATEGY_H_

#include <iomanip>
#include "MPStrategyMethod.h"
#include "MPLibrary/Extenders/ActiveBodyExtender.h"
#include "MPLibrary/MapEvaluators/RRTQuery.h"
#include "MPLibrary/MapEvaluators/StrategyStateEvaluator.h"
#include "MPLibrary/Samplers/MaskedSamplerMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// The RRT algorithm grows one or more trees from a set of root nodes to solve
/// a single-query planning problem.
///
/// Our not-so-basic RRT offers many variations by setting the appropriate
/// options:
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
class DisassemblyRRTStrategy : public MPStrategyMethod<MPTraits> {

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

    DisassemblyRRTStrategy(string _dm = "euclidean", string _nf = "bfnf",
        string _vc = "rapid", string _nc = "kClosest", string _ex = "BERO",
        vector<string> _evaluators = vector<string>(),
        string _gt = "UNDIRECTED_TREE",  bool _growGoals = false,
        double _growthFocus = .05, size_t _numRoots = 1,
        size_t _numDirections = 1, size_t _maxTrial = 3);

    DisassemblyRRTStrategy(XMLNode& _node);

    virtual ~DisassemblyRRTStrategy() = default;

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

    /// We need the start Cfg just for initializing the tree for disassembly
    CfgType* m_startCfg{nullptr};
    //In relation to the Manhattan-Like RRT (for the Thanh Le implementation)
    // this is q_init in that paper.

    void SetActiveBodies(const std::vector<unsigned int>& _bodies) {
      m_activeBodies = _bodies;
    }

    void SetKeepBestPathOnFailure(const bool _set) {
      m_returnBestPathOnFailure = _set;
    }

  protected:

    ///@name Direction Helpers
    ///@{

    /// Get a random configuration to grow towards.
    virtual CfgType SelectDirection();

    /// Sample a target configuration to grow towards from an existing
    ///        configuration. m_maxTrial samples are attempted.
    /// @param _v The VID of the existing configuration.
    /// @return The sample who's growth direction yields the greatest separation
    ///         from the existing configuration's neighbors.
    CfgType SelectDispersedDirection(VID _v);

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
    /// @param  _nearVID The nearby configuration's VID.
    /// @param  _qRand   The growth target.
    /// @param  _lp      This is a local plan: _qRand is already in the map.
    /// @return The new node's VID.
    virtual VID Extend(const VID _nearVID, const CfgType& _qRand,
                       const bool _lp = false);

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

    ///< Only for Manhattan-Like RRT.
    void PerterbCollidingParts(VID& _qNear, bool& _expanded);

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
    string m_nfLabel;       ///< The neighborhood finder label.
    string m_vcLabel;       ///< The validity checker label.
    string m_ncLabel;       ///< The connector label.
    string m_exLabel;       ///< The extender label.
    string m_gt;            ///< The graph type.

    string m_samplerLabel; ///< Sampler label.
    //No number or attempts is recorded because we need single
    // disassembly samples.

    string m_strategyLabel;
    string m_stateMELabel;


    ///@}
    ///@name RRT Properties
    ///@{

    size_t m_maxTrial;      ///< The number of samples taken for disperse search.

    VID m_startCfgVID{0};
    std::vector<unsigned int> m_activeBodies; //For the extender
    unsigned int m_lastRotAboutBody; // a default makes no sense here.

    ///< Provides path to node with highest clearance if it fails:
    bool m_returnBestPathOnFailure{false};

    bool m_doManhattanLike{false};///< The flag for ML-RRT (part perturbations).
    std::vector<unsigned int> m_collidingParts;///< The list of colliding parts for ML-RRT
    std::vector<double> m_vidClearances;///< The clearance indexed by VID for ML-RRT

    size_t m_maxPerterbIterations{20};

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
DisassemblyRRTStrategy<MPTraits>::
DisassemblyRRTStrategy(string _dm, string _nf, string _vc, string _nc,
    string _ex, vector<string> _evaluators, string _gt, bool _growGoals,
    double _growthFocus, size_t _numRoots, size_t _numDirections,
    size_t _maxTrial) :
    m_dmLabel(_dm), m_nfLabel(_nf), m_vcLabel(_vc), m_ncLabel(_nc),
    m_exLabel(_ex), m_gt(_gt), m_maxTrial(_maxTrial) {
  this->SetName("DisassemblyRRTStrategy");
  this->m_meLabels = _evaluators;
}


template <typename MPTraits>
DisassemblyRRTStrategy<MPTraits>::
DisassemblyRRTStrategy(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("DisassemblyRRTStrategy");

  // Parse RRT parameters
  m_gt = _node.Read("gtype", true, "", "Graph type dir/undirected tree/graph");
  m_maxTrial = _node.Read("trial", false, 1, 1, 1000,
      "Number of trials to get a dispersed direction");

  // Parse MP object labels
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");
  m_dmLabel = _node.Read("dmLabel",true,"","Distance Metric");
  m_ncLabel = _node.Read("connectorLabel", false, "", "Node Connection Method");
  m_exLabel = _node.Read("extenderLabel", true, "", "Extender label");
  m_samplerLabel = _node.Read("samplerLabel", false, "", "Sampler label");

  m_strategyLabel = _node.Read("label", true, "", "Strategy label");
  m_stateMELabel = _node.Read("stateMELabel", false, "", "Label for ME "
                 "if using a StrategyStateEvaluator for proper initialization");

  m_doManhattanLike =
        _node.Read("manhattanLike", false, m_doManhattanLike, "Sampler label");
  m_returnBestPathOnFailure = _node.Read("returnBestPathOnFailure", false,
                                    m_returnBestPathOnFailure, "Sampler label");

  m_maxPerterbIterations = _node.Read("maxPerterbIterations", false,
          m_maxPerterbIterations,
          "Maximum number of times to move colliding parts");

  // Parse child nodes.
  for(auto& child : _node) {
    if(child.Name() == "Evaluator")
      this->m_meLabels.push_back(child.Read("label", true, "",
          "Evaluation Method"));
  }
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
DisassemblyRRTStrategy<MPTraits>::
Print(ostream& _os) const {
  if(this->m_debug) {
  _os << "DisassemblyRRTStrategy::Print (debug is true)" << endl
      << "  MP objects:" << endl
      << "\tDistance Metric: " << m_dmLabel << endl
      << "\tNeighborhood Finder: " << m_nfLabel << endl
      << "\tValidity Checker: " << m_vcLabel << endl
      << "\tConnection Method: " << m_ncLabel << endl
      << "\tExtender: " << m_exLabel << endl
      << "\tEvaluators: " << endl;
  for(auto& s : this->m_meLabels)
    _os << "\t\t" << s << endl;

  _os << "  RRT properties:" << endl
      << "\tGraph Type: " << m_gt << endl;
  }
  else
    _os << "DisassemblyRRTStrategy::Print (debug is false)" << std::endl;
}

/*-------------------------- MPStrategy overrides ----------------------------*/

template <typename MPTraits>
void
DisassemblyRRTStrategy<MPTraits>::
Initialize() {
  if(this->m_debug)
    cout << "Initializing DisassemblyRRTStrategy" << endl;

  // Clear all state variables to avoid problems when running multiple times.
  m_trees.clear();
  m_successes = 0;
  m_trials = 0;

  m_collidingParts.clear();
  m_vidClearances.clear();

  GraphType* g = this->GetRoadmap()->GetGraph();

  // Note that this->m_successful gets set to false in MPStrategyMethod.

  for(string& label : this->m_meLabels) {
    if(this->m_debug)
      std::cout << "Setting ME's label to " << m_strategyLabel << std::endl;
    auto me = this->GetMapEvaluator(label);
    me->Initialize();
    if(!m_stateMELabel.empty()) {
      auto stateME = dynamic_pointer_cast<StrategyStateEvaluator<MPTraits> > (
                     this->GetMapEvaluator(m_stateMELabel));
      if(this->m_debug)
        std::cout << "Setting " << m_stateMELabel << "'s label to "
                  << m_strategyLabel << std::endl;
      stateME->m_strategyLabel = m_strategyLabel;
    }
  }

  // Check for query info.
  m_query = nullptr;

  // If a query was loaded, process query cfgs
  if(this->UsingQuery()) {
    throw RunTimeException(WHERE, "Not set up to handle queries yet");
  }
  else {
    //If there is a sampler label, then we need to make the only root be
    // m_startCfg so that the masked sampler has its starting point.
    if(!m_samplerLabel.empty() && m_startCfg) {
      m_startCfgVID = g->AddVertex(*m_startCfg);
      m_trees.push_back(vector<VID>(1, m_startCfgVID));
    }
    else // If no sampler and start cfg, throw an exception
      throw RunTimeException(WHERE, "RRT strategy for disassembly not "
                                    "initialized correctly!");
  }

  //There is some state built into our ME's for RRT, so ensure it starts over.
  for(auto label : this->m_meLabels)
    this->GetMapEvaluator(label)->Initialize();

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
DisassemblyRRTStrategy<MPTraits>::
Iterate() {
  auto graph = this->GetRoadmap()->GetGraph();
  ++m_trials;
  if(this->m_debug)
    cout << "*** Starting iteration " << m_trials << " "
         << "***************************************************" << endl
         << "Graph has " << graph->get_num_vertices()
         << " vertices." << std::endl;

  // Find random growth direction.
  CfgType target = this->SelectDirection();
  if(this->m_debug)
    std::cout << "Random direction selected: " << target << std::endl;

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
  if(newVID != INVALID_VID) {
    ++m_successes;
    if(this->m_debug) {
      CfgType tempCfg = graph->GetVertex(newVID);
      std::cout << "Added cfg (VID = " << newVID << ") = " <<
                tempCfg << std::endl;
    }
  }
}


template <typename MPTraits>
void
DisassemblyRRTStrategy<MPTraits>::
Finalize() {
  RoadmapType* const map = this->GetRoadmap();
  GraphType* const g = map->GetGraph();
  const string baseFilename = this->GetBaseFilename();
  const unsigned int mapVerts = map->GetGraph()->get_num_vertices();
  std::shared_ptr<SpecificBodyCollisionValidity<MPTraits> > vc;
  vc = dynamic_pointer_cast<SpecificBodyCollisionValidity<MPTraits>>(
                                     this->GetValidityChecker(this->m_vcLabel));
  if(this->m_debug) {
    std::cout << "DisassemblyRRTStrategy::Finalize()" << std::endl
              << "Map size = " << mapVerts << ".   VALIDITY:" << std::endl;
    for(size_t i = 0; i < mapVerts; ++i) {
      std::cout << " Map Vertex " << i << " validity = "
                << vc->IsValid(g->GetVertex(i), "RRTDEBUG") << std::endl;
    }
    std::cout << std::endl;
  }
  auto me = this->GetMapEvaluator("DistanceEvalSpecific");
  this->m_successful = me->operator()();
  if(this->m_successful) {
    //Create a temp RRTQuery to make the path for the masked composite
    // c-space path.
    RRTQuery<MPTraits> tempQuery;

    tempQuery.SetMPLibrary(this->GetMPLibrary());

    //Updates the roadmap and goals:
    tempQuery.Reset(map);
    VID lastAddedVID = map->GetGraph()->get_num_vertices()-1;

    //Perform the query so that the path is populated:
    *this->GetPath() += tempQuery.GeneratePath(m_startCfgVID, lastAddedVID);
    if(this->m_debug)
      std::cout << "RRTExtension: Just found path (from startVID = "
                << m_startCfgVID << " to goalVID = " << lastAddedVID
                << ") with total number of cfgs = "
                << this->GetPath()->VIDs().size() << std::endl
                << "Total number of cfgs in roadmap = "
                << this->GetRoadmap()->GetGraph()->get_num_vertices()
                << std::endl << std::endl;
  }
  else if(m_returnBestPathOnFailure) {
    // Find the cfg of max clearance (computed and stored during execution).
    // Since we failed to disassemble the part, make the path be to this cfg.
    if(this->m_debug)
      std::cout << "Manhattan-Like Finalize (failed to remove)"
                << std::endl;
    double maxClearance = -1.;
    VID maxClearanceVid = 0;
    for(size_t vid = 0; vid < m_vidClearances.size(); ++vid) {
      double clearance = m_vidClearances[vid];
      if(clearance > maxClearance) {
        maxClearance = clearance;
        maxClearanceVid = vid;
      }
    }

    //If we actually found a path to a vid with some clearance:
    if(maxClearanceVid != 0 && maxClearanceVid != m_startCfgVID) {
      //Create a temp RRTQuery to make the path for the masked composite
      // c-space path.
      RRTQuery<MPTraits> tempQuery;
      tempQuery.SetMPLibrary(this->GetMPLibrary());

      //Updates the roadmap and goals:
      tempQuery.Reset(map);

      //Perform the query so that the path is populated:
      *this->GetPath() += tempQuery.GeneratePath(m_startCfgVID, maxClearanceVid);
      if(this->m_debug)
        std::cout << "RRTExtension: Just found NON-REMOVAL path "
                  << "(from startVID = " << m_startCfgVID
                  << " to maxClearanceVid = " << maxClearanceVid
                  << ") with clearance = " << maxClearance
                  << " and total number of cfgs = "
                  << this->GetPath()->VIDs().size() << std::endl
                  << "Total number of cfgs in roadmap = "
                  << this->GetRoadmap()->GetGraph()->get_num_vertices()
                  << std::endl << std::endl;
    }
  }
  else
    if(this->m_debug)
      std::cout << "RRTExtension: Found no path!" << std::endl
                << "Total number of cfgs in roadmap = "
                << this->GetRoadmap()->GetGraph()->get_num_vertices()
                << std::endl;
}

/*--------------------------- Direction Helpers ------------------------------*/

template <typename MPTraits>
typename MPTraits::CfgType
DisassemblyRRTStrategy<MPTraits>::
SelectDirection() {
  /// \warning Should be named something like SelectTarget or SelectQRand as
  ///          this does not return a direction.

  CfgType target(this->GetTask()->GetRobot());
  if(this->m_samplerLabel.empty()) {
    target.GetRandomCfg(this->GetEnvironment()->GetBoundary());
  }
  else {
    if(!m_startCfg)
      RunTimeException(WHERE, "Cannot use the masked sampler version of RRT "
                              "without setting the start cfg before solving!");
    std::shared_ptr<MaskedSamplerMethod<MPTraits> > sampler =
                          dynamic_pointer_cast<MaskedSamplerMethod<MPTraits> >(
                          this->GetSampler(this->m_samplerLabel));

    std::vector<CfgType> successfulSamples;
    // Get one sample only trying one time.
    sampler->Sample(1, 1, this->GetEnvironment()->GetBoundary(),
                    back_inserter(successfulSamples));
    //If we got a sample, add it to the startCfg to get the needed cfg.
    if(!successfulSamples.empty()) {
      if(m_startCfg->DOF() == 0)
        throw RunTimeException(WHERE, "m_startCfg has no data!");

      //Choose a random cfg (if size = 1, it will just take the 0 index)
      target = successfulSamples[LRand() % successfulSamples.size()];
      //Save body rotated about to give to the extender for intermediate plan.
      m_lastRotAboutBody = sampler->GetLastRotAboutBody();
    }
  }
  return target;
}


template <typename MPTraits>
typename MPTraits::CfgType
DisassemblyRRTStrategy<MPTraits>::
SelectDispersedDirection(VID _v) {
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
    CfgType randCfg = SelectDirection();

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
DisassemblyRRTStrategy<MPTraits>::
SelectNeighbors(VID _v) {
  GraphType* g = this->GetRoadmap()->GetGraph();
  typename GraphType::vertex_iterator vi = g->find_vertex(_v);
  vector<CfgType> vec;
  for(const auto& e : *vi)
    vec.push_back(g->GetVertex(e.target()));
  return vec;
}


template <typename MPTraits>
typename DisassemblyRRTStrategy<MPTraits>::VID
DisassemblyRRTStrategy<MPTraits>::
FindNearestNeighbor(const CfgType& _cfg, const TreeType& _tree) {
  this->GetStatClass()->StartClock("BasicRRT::NeighborhoodFinding");

  vector<pair<VID, double>> neighbors;
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  nf->FindNeighbors(this->GetRoadmap(),
      _tree.begin(), _tree.end(),
      _tree.size() == this->GetRoadmap()->GetGraph()->get_num_vertices(),
      _cfg, back_inserter(neighbors));
  VID nearestVID = neighbors[0].first;

  this->GetStatClass()->StopClock("BasicRRT::NeighborhoodFinding");
  return nearestVID;
}


template <typename MPTraits>
void
DisassemblyRRTStrategy<MPTraits>::
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
typename DisassemblyRRTStrategy<MPTraits>::VID
DisassemblyRRTStrategy<MPTraits>::
Extend(const VID _nearVID, const CfgType& _qRand, const bool _lp) {
  const string label = this->GetNameAndLabel() + "::Extend()";
  GraphType* const graph = this->GetRoadmap()->GetGraph();
  const CfgType& qNear = graph->GetVertex(_nearVID);
  CfgType qNew(this->GetTask()->GetRobot());
  LPOutput<MPTraits> lpOutput;
  pair<VID, bool> extension{INVALID_VID, false};
  bool extended;
  CDInfo cdInfo(true);

  //Handle the case of disassembly expansion (to handle subassemblies).
  std::shared_ptr<ActiveBodyExtender<MPTraits> > extender =
                          dynamic_pointer_cast<ActiveBodyExtender<MPTraits> >(
                                                this->GetExtender(m_exLabel));
  ///@TODO: When done with the extender, enforce this:
//  if(!extender)
//    throw RunTimeException(WHERE, "You should be using an active body extender "
//                                  "when doing disassembly planning");

  //m_lastRotAboutBody is retrieved from the sampler after generating the sample
  std::vector<unsigned int> orderedActiveBodies = {m_lastRotAboutBody};
  if(extender) {
    if(m_activeBodies.size() > 1) {
      // The body to rotate about is first in the vector:
      for(const unsigned int body : m_activeBodies)
        if(body != orderedActiveBodies[0])
          orderedActiveBodies.push_back(body);
    }
    extender->SetActiveBodies(orderedActiveBodies);
    extended = extender->Extend(qNear, _qRand, qNew, lpOutput, cdInfo);
  }
  else {
    orderedActiveBodies = m_activeBodies; // Just as a placeholder
    extended = this->GetExtender(m_exLabel)->Extend(qNear, _qRand, qNew,
                                                    lpOutput, cdInfo);
  }
  if(extended)
    extension = AddNode(qNew); // qNew gets added to the roadmap here.

  // Always add active bodies to the edge, whether or not extender uses them.
  lpOutput.SetActiveBodies(orderedActiveBodies);

  const VID newVID = extension.first;
  const bool nodeIsNew = extension.second;

  if(m_doManhattanLike) { // Populate colliding body info.
    // The Extender's CDInfo will be from the last ticked Cfg, not the free one.
    // You can see that this is the case by looking at BasicExtender.h:200.
    m_collidingParts.clear();
    //The index of m_bodyDists is the body number, the entry is the distance.
    for(unsigned int part = 0; part < cdInfo.m_selfClearance.size(); part++)
      if(cdInfo.m_selfClearance[part] <= std::numeric_limits<double>::epsilon())
        this->m_collidingParts.push_back(part);

    if(this->m_debug)
     std::cout << "Found colliding parts = " << m_collidingParts << std::endl
               << "CDInfo bodyDists = " << cdInfo.m_selfClearance << std::endl;
  }

  if(m_returnBestPathOnFailure) {
    //Update the clearance for this VID (note: resize preserves existing data)
    if(newVID != INVALID_VID) {
      auto vc = this->GetValidityChecker(m_vcLabel);
      vc->IsValid(qNew, cdInfo, label); // Need the actual cdInfo for new cfg.
      if(newVID > m_vidClearances.size())
        m_vidClearances.resize((newVID + 1), 0);

      // Obstacles are included in this distance for clearance:
      m_vidClearances[newVID] = cdInfo.m_minDist;
    }
    if(this->m_debug)
      std::cout << "Min part/obstacle distance = " << cdInfo.m_minDist
                << std::endl;
  }

  if(newVID == INVALID_VID) {
    // The extension failed to reach a valid node.
    if(this->m_debug)
      std::cout << "\tNot adding new node. extended = " << extended<< std::endl;
    return INVALID_VID;
  }
  else if(this->m_debug)
    std::cout << "\tExtended " << lpOutput.m_edge.first.GetWeight()
              << " units." << std::endl;

  // If we are local planning, reached the goal, and haven't added the edge,
  // this is a valid local plan.
  const bool validLP = _lp && (qNew == _qRand)
                           && !graph->IsEdge(_nearVID, newVID);

  if(nodeIsNew || validLP) {
    lpOutput.AddIntermediatesToWeights(true);
    AddEdge(_nearVID, newVID, lpOutput);
  }

  if(nodeIsNew && !_lp)
    ConnectNeighbors(newVID);

  if((_lp && !validLP) || !nodeIsNew)
    return INVALID_VID; // If failed lp or node isn't new, return invalid vid.

  return newVID;
}


template <typename MPTraits>
pair<typename DisassemblyRRTStrategy<MPTraits>::VID, bool>
DisassemblyRRTStrategy<MPTraits>::
AddNode(const CfgType& _newCfg) {
  GraphType* const g = this->GetRoadmap()->GetGraph();
  const VID newVID = g->AddVertex(_newCfg);
  const bool nodeIsNew = (newVID == (g->get_num_vertices() - 1));
  if(nodeIsNew) {
    if(this->m_debug)
      std::cout << "m_currentTree.size() = " << m_currentTree->size() <<
                " | m_currentTree.capacity() = " << m_currentTree->capacity()
                << std::endl;
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
DisassemblyRRTStrategy<MPTraits>::
AddEdge(VID _source, VID _target, const LPOutput<MPTraits>& _lpOutput) {
  GraphType* g = this->GetRoadmap()->GetGraph();
  if(m_gt.find("UNDIRECTED") != std::string::npos)
    g->AddEdge(_source, _target, _lpOutput.m_edge);
  else
    g->AddEdge(_source, _target, _lpOutput.m_edge.first);
  g->GetVertex(_target).SetStat("Parent", _source);

  if(this->m_debug)
    cout << "\tAdding Edge (" << _source << ", " << _target << ")." << endl;
}

/*------------------------------ Tree Helpers --------------------------------*/

template <typename MPTraits>
typename DisassemblyRRTStrategy<MPTraits>::VID
DisassemblyRRTStrategy<MPTraits>::
ExpandTree(CfgType& _target) {
  VID nearestVID = FindNearestNeighbor(_target, *m_currentTree);
  return this->ExpandTree(nearestVID, _target);
}


template <typename MPTraits>
typename DisassemblyRRTStrategy<MPTraits>::VID
DisassemblyRRTStrategy<MPTraits>::
ExpandTree(const VID _nearestVID, const CfgType& _target) {
  if(this->m_debug)
    cout << "Trying expansion from node " << _nearestVID
         << " at workspace point "
         << this->GetRoadmap()->GetGraph()->GetVertex(_nearestVID).GetPoint()
         << "..." << endl;

  // Try to extend from the _nearestVID to _target
  VID newVID = this->Extend(_nearestVID, _target);

  VID qNear;//For manhattan-like algorithm
  bool expanded = false;
  if(newVID != INVALID_VID) {
    expanded = true;
    ConnectTrees(newVID);
    qNear = newVID;
  }
  else
    qNear = _nearestVID;

  if(m_doManhattanLike && !m_collidingParts.empty())
    PerterbCollidingParts(qNear, expanded);

  if(!expanded)
    return INVALID_VID;

  return newVID;
}


template <typename MPTraits>
void
DisassemblyRRTStrategy<MPTraits>::
PerterbCollidingParts(VID& _qNear, bool& _expanded) {
  if(this->m_debug)
    std::cout << "PerterbCollidingParts (Manhattan Like). _qNear = " << _qNear
              << " | _expanded = " << _expanded << std::endl;

  size_t iterations = 0;
  while(!m_collidingParts.empty()) {
    if(this->m_debug)
      std::cout << "PerterbCollidingParts: m_collidingParts = "
                << m_collidingParts << std::endl;
    std::vector<CfgType> successfulSamples;

    auto sampler = dynamic_pointer_cast<MaskedSamplerMethod<MPTraits>>(
                                        this->GetSampler(this->m_samplerLabel));

    std::shared_ptr<SpecificBodyCollisionValidity<MPTraits> > vc;
    vc = dynamic_pointer_cast<SpecificBodyCollisionValidity<MPTraits>>(
                                     this->GetValidityChecker(this->m_vcLabel));

    //Save the current mask from the sampler
    auto prevMask = sampler->GetMask();

    auto prevParts = vc->GetBodyNumbers();

    //Replace mask on sampler with the parts in m_collidingParts
    //We just use a random strategy for which part to perterb
    CfgType perterbCfg(this->GetTask()->GetRobot());
    const bool posDofsOnly = sampler->GetUseOnlyPosDofs();

    std::vector<unsigned int> partToAdjust =
                        {m_collidingParts[LRand() % m_collidingParts.size()]};
    vc->SetBodyNumbers(partToAdjust);
    sampler->SetMaskByBodyList(partToAdjust, posDofsOnly);

    //Create perterbCfg from this
    sampler->Sample(1, 1, this->GetEnvironment()->GetBoundary(),
                    back_inserter(successfulSamples));
    if(!successfulSamples.empty()) {
      //Choose a random cfg (if size = 1, it will just take the 0 index)
      perterbCfg = successfulSamples[LRand() % successfulSamples.size()];
      if(this->m_debug)
        std::cout << "successful perterbCfg = " << perterbCfg << std::endl;
    }

    //Save previous parts, since Extend() will clobber the data in the member.
    std::vector<unsigned int> prevCollidingParts = m_collidingParts;

    //Extend towards perterbCfg and add to the tree if not too similar.
    //NOTE will clear and replace all elements in m_collidingParts!
    VID newVID = this->Extend(_qNear, perterbCfg);

    //Reset the sampler's mask to the active part:
    sampler->SetMask(prevMask);

    vc->SetBodyNumbers(prevParts);

    if(newVID != INVALID_VID) {
      _expanded = true;//It should be correct to keep updating this.
      ConnectTrees(newVID);
      _qNear = newVID;
    }

    if(this->m_debug)
      std::cout << "prevCollidingParts = " << prevCollidingParts << std::endl
                << "new colliding parts = " << m_collidingParts << std::endl;

    //Update the colliding parts not to have any of the ones just adjusted:
    for(unsigned int part : prevCollidingParts) {
      m_collidingParts.erase(
          remove(m_collidingParts.begin(), m_collidingParts.end(), part),
          m_collidingParts.end());
    }
    if(this->m_debug)
      std::cout << "Updated new colliding parts = " << m_collidingParts
                << std::endl;

    if(++iterations > m_maxPerterbIterations) {
      if(this->m_debug)
        std::cout << "Maxed out iterations of PerterbCollidingParts!"
                  << std::endl;
      break;
    }
  }
  if(this->m_debug)
    std::cout << "Leaving PerterbCollidingParts" << std::endl << std::endl;
}


template <typename MPTraits>
void
DisassemblyRRTStrategy<MPTraits>::
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
    // Skip current tree
    if(trit == m_currentTree)
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

  if(this->m_debug)
    cout << "Connecting trees: from (tree "
         << distance(m_trees.begin(), closestTree) << ", VID "
         << closestVID << ") to (tree "
         << distance(m_trees.begin(), m_currentTree) << ", VID "
         << _recentlyGrown << "), distance = " << setw(4) << minDist << endl;

  // Try to expand from closestVID (in closestTree) to qNew (in m_currentTree)
  // in order to connect the trees.
  swap(m_currentTree, closestTree);
  VID newVID = this->Extend(closestVID, qNew, true);

  if(newVID != INVALID_VID) {
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
DisassemblyRRTStrategy<MPTraits>::
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
DisassemblyRRTStrategy<MPTraits>::
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
