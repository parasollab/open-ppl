#ifndef PMPL_DISASSEMBLY_RRT_STRATEGY_H_
#define PMPL_DISASSEMBLY_RRT_STRATEGY_H_

#include "MPStrategyMethod.h"
#include "MPLibrary/MapEvaluators/GroupQuery.h"
#include "MPLibrary/MapEvaluators/StrategyStateEvaluator.h"
#include "MPLibrary/MPStrategies/DisassemblyMethod.h"
#include "MPLibrary/Samplers/MaskedSamplerMethodGroup.h"

#include <iomanip>


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
///
/// This is sub-method for disassembly methods. Sub-method because it is
/// not designed to be used on its own, but rather to be called from a
/// disassembly method in order to try an RRT removal.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DisassemblyRRTStrategy : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef typename MPTraits::GroupWeightType   WeightType;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename MPTraits::GroupPathType     GroupPathType;
    typedef typename GroupRoadmapType::VID       VID;
    typedef typename GroupCfgType::Formation     Formation;
    typedef typename MPBaseObject<MPTraits>::MapEvaluatorPointer MapEvaluatorPointer;

    typedef typename MPTraits::MPLibrary::ValidityCheckerPointer ValidityCheckerPointer;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::vector<VID> TreeType;

    ///@}
    ///@name Construction
    ///@{

    DisassemblyRRTStrategy(std::string _dm = "euclidean", std::string _nf = "bfnf",
        std::string _vc = "rapid", std::string _nc = "kClosest", std::string _ex = "BERO",
        std::vector<std::string> _evaluators = std::vector<std::string>(),
        std::string _gt = "UNDIRECTED_TREE",  bool _growGoals = false,
        double _growthFocus = .05, size_t _numRoots = 1,
        size_t _numDirections = 1, size_t _maxTrial = 3);

    DisassemblyRRTStrategy(XMLNode& _node);

    virtual ~DisassemblyRRTStrategy() = default;

    ///@}
    ///@name MPBaseObject overrides
    ///@{

    virtual void Print(std::ostream& _os) const;

    ///@}

    /// We need the start Cfg just for initializing the tree for disassembly
//    const CfgType* m_startCfg{nullptr};
    const GroupCfgType* m_startGroupCfg{nullptr};
    //In relation to the Manhattan-Like RRT (for the Thanh Le implementation)
    // this is q_init in that paper.

    void SetActiveRobots(const std::vector<size_t>& _bodies) {
      m_activeRobots = _bodies;
    }

    void SetKeepBestPathOnFailure(const bool _set) {
      m_returnBestPathOnFailure = _set;
    }

    bool IsSuccessful() const {return m_successful;}

  protected:

    ///@name MPStrategy Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Iterate() override;
    virtual void Finalize() override;

    ///@}
    ///@name Direction Helpers
    ///@{

    /// Get a random configuration to grow towards, also returns the configuration
    /// that was chosen to mask relative to for composite CSpace.
    virtual std::pair<GroupCfgType, std::size_t> SelectTarget();

    ///@}
    ///@name Neighbor Helpers
    ///@{

    /// Get the configurations that are adjacent to _v in the map.
    std::vector<GroupCfgType> SelectNeighbors(VID _v);

    /// Find the nearest configuration to the target _cfg within _tree.
    virtual VID FindNearestNeighbor(const GroupCfgType& _cfg, const TreeType& _tree);

    /// If the graph type is GRAPH, try to connect a configuration to its
    /// neighbors. No-op for TREE type graph.
    /// @param _newVID The VID of the configuration to connect.
//    void ConnectNeighbors(VID _newVID);

    ///@}
    ///@name Growth Helpers
    ///@{

    /// Extend a new configuration from a nearby configuration towards a growth
    /// target.
    /// @param  _nearVID The nearby configuration's VID.
    /// @param  _qRand   The growth target.
    /// @param  _lp      This is a local plan: _qRand is already in the map.
    /// @return The new node's VID.
    virtual VID Extend(const VID _nearVID, const GroupCfgType& _qRand,
                       const bool _lp = false);

    /// Add a new configuration to the roadmap and current tree.
    /// @param _newCfg    The new configuration to add.
    /// @return A pair with the added VID and a bool indicating whether the new
    ///         node was already in the map.
    virtual std::pair<VID, bool> AddNode(const GroupCfgType& _newCfg);

    /// Add a new edge to the roadmap.
    /// @param _source   The source node.
    /// @param _target   The target node.
    /// @param _lpOutput The extender output.
    void AddEdge(VID _source, VID _target, const GroupLPOutput<MPTraits>& _lpOutput);

    ///@}
    ///@name Tree Helpers
    ///@{

    /// Attempt to expand the map by growing towards a target
    ///        configuration from the nearest existing node in the current tree.
    /// @param _target The target configuration.
    /// @return            The VID of a newly created Cfg if successful,
    ///                    INVALID_VID otherwise.
    virtual VID ExpandTree(GroupCfgType& _target);

    /// Attempt to expand the map by growing towards a target
    ///        configuration from an arbitrary node.
    /// @param _nearestVID The VID to grow from.
    /// @param _target     The target configuration.
    /// @return                The VID of a newly created Cfg if successful,
    ///                        INVALID_VID otherwise.
    virtual VID ExpandTree(const VID _nearestVID, const GroupCfgType& _target);

    ///< Only for Manhattan-Like RRT.
    void PerterbCollidingParts(VID& _qNear, bool& _expanded);

    ///@}
    ///@name MP Object Labels
    ///@{

    std::string m_nfLabel;       ///< The neighborhood finder label.
    std::string m_vcLabel;       ///< The validity checker label.
    std::string m_exLabel;       ///< The extender label.
    std::string m_gt;            ///< The graph type.

    std::string m_samplerLabel; ///< Sampler label.
    //No number or attempts is recorded because we need single
    // disassembly samples.

    std::string m_stateMELabel; ///< The ME label for the StrategyStateEvaluator used

    /// The ME label that determines the successful flag of this strategy
    /// (should be minimum clearance of the part being removed).
    std::string m_successfulCheckME;

    ///@}
    ///@name RRT Properties
    ///@{

    VID m_startGroupCfgVID{0};
    Formation m_activeRobots; //For the extender and neighborhood finder
//    size_t m_lastSamplesLeaderBody; // a default makes no sense here.

    ///< Provides path to node with highest clearance if it fails:
    bool m_returnBestPathOnFailure{true};
    bool m_preserveMinClearance{false};

    bool m_doManhattanLike{false};///< The flag for ML-RRT (part perturbations).
    std::vector<size_t> m_collidingParts;///< The list of colliding parts for ML-RRT
    std::vector<double> m_vidClearances;///< The clearance indexed by VID for ML-RRT

    size_t m_maxPerturbIterations{20};

    bool m_timeEverything{true};

    size_t m_numCfgsGenerated{0};
    size_t m_numPerturbCfgsGenerated{0};

    //Appends some RRT stats to a file if true in Finalize()
    bool m_recordNodeCountGenerated{false};

    bool m_successful{false};

    ///@}
    ///@name Tree Data
    ///@{

    TreeType m_tree;                ///< The tree of VIDs of the roadmap.

    ///@}
    ///@name Extension Success Tracking
    ///@{

    size_t m_successes{0};  ///< The count of successful extensions.
    size_t m_trials{0};     ///< The count of attempted extensions.

    ///@}
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
DisassemblyRRTStrategy<MPTraits>::
DisassemblyRRTStrategy(std::string _dm, std::string _nf, std::string _vc, std::string _nc,
    std::string _ex, std::vector<std::string> _evaluators, std::string _gt, bool _growGoals,
    double _growthFocus, size_t _numRoots, size_t _numDirections,
    size_t _maxTrial) :
        m_nfLabel(_nf), m_vcLabel(_vc), m_exLabel(_ex), m_gt(_gt) {
  this->SetName("DisassemblyRRTStrategy");
  this->m_meLabels = _evaluators;
}


template <typename MPTraits>
DisassemblyRRTStrategy<MPTraits>::
DisassemblyRRTStrategy(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("DisassemblyRRTStrategy");

  // Parse RRT parameters
  m_gt = _node.Read("gtype", true, "", "Graph type dir/undirected tree/graph");

  // Parse MP object labels
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");
  m_exLabel = _node.Read("extenderLabel", true, "", "Extender label");
  m_samplerLabel = _node.Read("samplerLabel", true, "", "Sampler label (should "
                                                        "be a masked version)");

  m_successfulCheckME = _node.Read("successfulCheckME", true, "NOT SET",
      "ME that determines the setting of the strategy's successful flag (should"
      " be MinClearanceEval or DistanceEvalSpecific).");

  m_stateMELabel = _node.Read("stateMELabel", false, "", "Label for ME "
                 "if using a StrategyStateEvaluator for proper initialization");

  m_doManhattanLike = _node.Read("manhattanLike", false, m_doManhattanLike,
                            "Flag to determine use of PerturbCollidingParts()");
  m_returnBestPathOnFailure = _node.Read("returnBestPathOnFailure", false,
      m_returnBestPathOnFailure, "Flag to determine returning of partial RRT "
                                            "progress if no part was removed.");

  m_preserveMinClearance = _node.Read("preserveMinClearance", false,
      m_preserveMinClearance, "If returning best path on failure, determines "
          "whether to consider the clearance of the starting node of RRT.");

  m_maxPerturbIterations = _node.Read("maxPerterbIterations", false,
                            m_maxPerturbIterations,
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
Print(std::ostream& _os) const {
  if(this->m_debug) {
  _os << "DisassemblyRRTStrategy::Print (debug is true)" << std::endl
      << "  MP objects:" << std::endl
//      << "\tDistance Metric: " << m_dmLabel << std::endl
      << "\tNeighborhood Finder: " << m_nfLabel << std::endl
      << "\tValidity Checker: " << m_vcLabel << std::endl
//      << "\tConnection Method: " << m_ncLabel << std::endl
      << "\tExtender: " << m_exLabel << std::endl
      << "\tEvaluators: " << std::endl;
  for(auto& s : this->m_meLabels)
    _os << "\t\t" << s << std::endl;

  _os << "  RRT properties:" << std::endl
      << "\tGraph Type: " << m_gt << std::endl;
  }
}

/*-------------------------- MPStrategy overrides ----------------------------*/

template <typename MPTraits>
void
DisassemblyRRTStrategy<MPTraits>::
Initialize() {
  if(this->m_debug) {
    std::cout << "Initializing DisassemblyRRTStrategy" << std::endl;
    m_timeEverything = true; // ALWAYS time when debugging.
  }

  // Clear all state variables to avoid problems when running multiple times.
  m_successful = false;
  m_successes = 0;
  m_trials = 0;

  m_numPerturbCfgsGenerated = 0;
  m_numCfgsGenerated = 0;

  m_collidingParts.clear();
  m_vidClearances.clear();

  GroupRoadmapType* g = this->GetGroupRoadmap();

  // Note that this->m_successful gets set to false in MPStrategyMethod.

  /// @todo This shouldn't be needed, everything is initialized at the top of
  ///       the library's solve call.
  for(string& label : this->m_meLabels) {
    auto me = this->GetMapEvaluator(label);
    me->Initialize();
  }
  if(!m_stateMELabel.empty()) {
    auto stateME = dynamic_pointer_cast<StrategyStateEvaluator<MPTraits> > (
                   this->GetMapEvaluator(m_stateMELabel));
    if(this->m_debug)
      std::cout << "Setting " << m_stateMELabel << "'s label to "
                << this->GetLabel() << std::endl;
    stateME->m_strategyLabel = this->GetLabel();
  }

  MapEvaluatorPointer me = this->GetMapEvaluator(m_successfulCheckME);
  me->SetActiveRobots(m_activeRobots);

  if(!m_samplerLabel.empty() && m_startGroupCfg && m_startGroupCfg->DOF() != 0) {
    // the startCfg will be the root in this roadmap, and we only use one tree.
    m_startGroupCfgVID = g->AddVertex(*m_startGroupCfg);
    m_tree = TreeType(1, m_startGroupCfgVID);
  }
  else // If no sampler and start cfg, throw an exception
    throw RunTimeException(WHERE, "RRT strategy for disassembly not "
                                  "initialized correctly!");

  /// @todo This shouldn't be needed, everything is initialized at the top of
  ///       the library's solve call. It is also done above.
  //There is some state built into our ME's for RRT, so ensure it starts over.
  for(auto label : this->m_meLabels)
    this->GetMapEvaluator(label)->Initialize();
}


template <typename MPTraits>
void
DisassemblyRRTStrategy<MPTraits>::
Iterate() {
  GroupRoadmapType* const graph = this->GetGroupRoadmap();
  auto const stats = this->GetStatClass();
  ++m_trials;
//  if(this->m_debug)
//    std::cout << "*** Starting iteration " << m_trials << " "
//         << "***************************************************" << std::endl
//         << "Graph state: " << std::endl << graph->PrettyPrint() << std::endl;

  stats->StartClock("SelectDirectionClock");

  // Find random growth direction.
  std::pair<GroupCfgType, VID> target = this->SelectTarget();

  // Check the sample was actually successful:
  if(target.first == *m_startGroupCfg) {
    if(this->m_debug)
      std::cout << std::endl << std::endl << std::endl
                << "Sample failed! Re-attempting, but counting this iteration."
                << std::endl;
    stats->StopClock("SelectDirectionClock");
    return;
  }

  ++m_numCfgsGenerated;

  if(this->m_debug)
    std::cout << "Random direction selected: " << target.first.PrettyPrint()
              << std::endl;

  stats->StopClock("SelectDireDisassemblyRRTctionClock");
  stats->StartClock("FindNearestNeighborClock");

  // Find the nearest configuration to target within the current tree
  const VID nearestVID = FindNearestNeighbor(target.first, m_tree);

  stats->StopClock("FindNearestNeighborClock");
  stats->StartClock("ExpandTreeClock");

  // Expand current tree
  const VID newVID = this->ExpandTree(nearestVID, target.first);
  if(newVID != INVALID_VID) {
    ++m_successes;
    if(this->m_debug) {
      GroupCfgType tempCfg = graph->GetVertex(newVID);
      std::cout << "Added cfg (VID = " << newVID << ") = " <<
                tempCfg << std::endl;
    }
  }
  stats->StopClock("ExpandTreeClock");
}


template <typename MPTraits>
void
DisassemblyRRTStrategy<MPTraits>::
Finalize() {
  GroupRoadmapType* const map = this->GetGroupRoadmap();
  const std::string baseFilename = this->GetBaseFilename();
  const size_t mapVerts = map->GetGraph()->get_num_vertices();
  GroupPathType* const path = this->GetGroupPath();
  ValidityCheckerPointer vc = this->GetValidityChecker(this->m_vcLabel);

//  if(this->m_debug) {
    std::cout << "DisassemblyRRTStrategy::Finalize()" << std::endl
              << "Roadmap state:" << std::endl << map->PrettyPrint()
              << std::endl;
    std::vector<VID> invalidVids;
    for(size_t i = 0; i < mapVerts; ++i)
      if(!vc->IsValid(map->GetVertex(i), "RRTDEBUG", m_activeRobots))
        invalidVids.push_back(i);

    if(invalidVids.empty())
      std::cout << "All group roadmap vertices were valid!" << std::endl;
    else
      std::cout << "The following group roadmap vids had invalid cfgs: "
                << invalidVids << std::endl;
//  }

  if(m_recordNodeCountGenerated) {
    //Append the data for number of nodes generated:
    std::ofstream nodeGenFile;
    nodeGenFile.open("NodesGenerated.data", std::ios_base::app);
    nodeGenFile << m_numCfgsGenerated << " " << m_numPerturbCfgsGenerated << " "
                << mapVerts << std::endl;
  }

  // Active robots is set in Initialize() for this:
  MapEvaluatorPointer me = this->GetMapEvaluator(m_successfulCheckME);
  m_successful = me->operator()();
  if(m_successful) {
    //Create a temp query to make the path through the group's c-space.
    GroupQuery<MPTraits> tempQuery;

    tempQuery.SetMPLibrary(this->GetMPLibrary());

    //Updates the roadmap and goals:
    path->Clear();
    VID lastAddedVID = map->GetGraph()->get_num_vertices()-1;

    //Perform the query so that the path is populated:
    std::vector<VID> nextSubPath = tempQuery.GeneratePath(m_startGroupCfgVID, lastAddedVID);
    *path += nextSubPath;
//    if(this->m_debug)
      std::cout << "RRTExtension: Just found path (from startVID = "
                << m_startGroupCfgVID << " to goalVID = " << lastAddedVID
                << ") with total number of cfgs = "
                << path->VIDs().size() << std::endl
                << "Total number of cfgs in roadmap = "
                << this->GetGroupRoadmap()->get_num_vertices()
                << std::endl << std::endl;
  }
  else if(m_returnBestPathOnFailure && !m_vidClearances.empty()) {
    // Find the cfg of max clearance (computed and stored during execution).
    // Since we failed to disassemble the part, make the path be to this cfg.
//    if(this->m_debug)
      std::cout << "RRT failed to remove the subassembly, but a best partial "
                << "path will be generated." << std::endl;

    // Temp flag to determine whether we're okay "losing" clearance in RRT
    // partial paths. If true, the root's clearance will be considered.
    // Note the maximum clearance will always be taken, regardless of this.
//    const bool m_preserveMinClearance = false;

      /// TODO: uncomment when we want the root's clearance considered:
    if(m_preserveMinClearance) {
      CDInfo cdInfo(true); // For root's clearance info
      GroupCfgType startCfg = *m_startGroupCfg;
      vc->IsValid(startCfg, cdInfo, this->GetNameAndLabel(), m_activeRobots);
      m_vidClearances[m_startGroupCfgVID] = cdInfo.m_minDist;
    }
    else
      m_vidClearances[m_startGroupCfgVID] = 0; //Don't consider root's clearance

    // Start off with the root cfg clearance as the max.
    double maxClearance = m_vidClearances[m_startGroupCfgVID];
    VID maxClearanceVid = m_startGroupCfgVID;
    std::vector<double> clearances;
    for(size_t vid = 0; vid < map->Size(); ++vid) {
      if(m_vidClearances[vid] == 0.
          && vid != m_startGroupCfgVID // TODO: remove this condition if we don't want to lose clearance!
          ) {
        CDInfo tempInfo(true);
        GroupCfgType& cfg = map->GetVertex(vid);
        vc->IsValid(cfg, tempInfo, this->GetNameAndLabel(), m_activeRobots);
        m_vidClearances[vid] = tempInfo.m_minDist;
        if(m_vidClearances[vid] > 0.)
          std::cerr << "RRT Warning: Vid " << vid << " didn't have a correct "
                       "clearance in finalize!" << std::endl;
      }

      const double clearance = m_vidClearances[vid];
      clearances.push_back(clearance);
      if(clearance > maxClearance) {
        maxClearance = clearance;
        maxClearanceVid = vid;
      }
    }

//    if(this->m_debug)
      std::cout << "Node clearances (in order of vids): " << clearances
                << std::endl;

    //If we actually found a path to a vid with some clearance:
    if(maxClearanceVid != 0 && maxClearanceVid != m_startGroupCfgVID) {
      //Create a temp RRTQuery to make the path for the masked composite
      // c-space path.
      GroupQuery<MPTraits> tempQuery;
      tempQuery.SetMPLibrary(this->GetMPLibrary());

      //Updates the roadmap and goals:
      path->Clear();

      //Perform the query so that the path is populated:
      *path += tempQuery.GeneratePath(m_startGroupCfgVID, maxClearanceVid);
//      if(this->m_debug)
        std::cout << "RRTExtension: Just found NON-REMOVAL path "
                  << "(from startVID = " << m_startGroupCfgVID
                  << " to maxClearanceVid = " << maxClearanceVid
                  << ") with clearance = " << maxClearance
                  << " and total number of cfgs = "
                  << path->VIDs().size() << std::endl
                  << "Total number of cfgs in roadmap = "
                  << this->GetGroupRoadmap()->get_num_vertices()
                  << std::endl << std::endl;
    }
    else
//      if(this->m_debug)
        std::cout << "The root had maximum clearance!" << std::endl;
  }
  else
//    if(this->m_debug)
      std::cout << "RRTExtension: Found no path!" << std::endl
                << "Total number of cfgs in roadmap = "
                << this->GetGroupRoadmap()->get_num_vertices()
                << std::endl;
}

/*--------------------------- Direction Helpers ------------------------------*/

template <typename MPTraits>
std::pair<typename DisassemblyRRTStrategy<MPTraits>::GroupCfgType, std::size_t>
DisassemblyRRTStrategy<MPTraits>::
SelectTarget() {
  std::shared_ptr<MaskedSamplerMethodGroup<MPTraits> > const sampler =
                      dynamic_pointer_cast<MaskedSamplerMethodGroup<MPTraits> >(
                      this->GetSampler(this->m_samplerLabel));
  auto const graph = this->GetGroupRoadmap();
  const size_t numVerts = graph->get_num_vertices();
  const VID randomVID = LRand() % numVerts;

  // Set the sampler to mask relative to a random cfg in the tree.
  sampler->SetStartCfg(graph->GetVertex(randomVID));

  // NOTE: that active bodies will already be correctly set (outside RRT method)

  std::vector<GroupCfgType> successfulSamples;
  // Get one sample trying a few times:
  const size_t numAttempts = 3;
  sampler->Sample(1, numAttempts, this->GetEnvironment()->GetBoundary(),
                  std::back_inserter(successfulSamples));

  if(!successfulSamples.empty()) {
    //Choose a random cfg (if size = 1, it will just take the 0 index)
    return std::make_pair(successfulSamples[LRand() % successfulSamples.size()],
                          randomVID);
  }

  return std::make_pair(*m_startGroupCfg, randomVID); // Unsuccessful sample.
}

/*---------------------------- Neighbor Helpers ------------------------------*/

template <typename MPTraits>
std::vector<typename DisassemblyRRTStrategy<MPTraits>::GroupCfgType>
DisassemblyRRTStrategy<MPTraits>::
SelectNeighbors(VID _v) {
  GroupRoadmapType* g = this->GetGroupRoadmap();
  typename GroupRoadmapType::vertex_iterator vi = g->find_vertex(_v);
  std::vector<GroupCfgType> vec;
  for(const auto& e : *vi)
    vec.push_back(g->GetVertex(e.target()));
  return vec;
}


template <typename MPTraits>
typename DisassemblyRRTStrategy<MPTraits>::VID
DisassemblyRRTStrategy<MPTraits>::
FindNearestNeighbor(const GroupCfgType& _cfg, const TreeType& _tree) {
  auto roadmap = this->GetGroupRoadmap();
  std::vector<Neighbor> neighbors;
  auto const nf = this->GetNeighborhoodFinder(m_nfLabel);
  nf->FindNeighbors(roadmap, _tree.begin(), _tree.end(),
                    _tree.size() == roadmap->get_num_vertices(),
                    _cfg, std::back_inserter(neighbors));
  const VID nearestVID = neighbors[0].target;

  return nearestVID;
}

/*----------------------------- Growth Helpers -------------------------------*/

template <typename MPTraits>
typename DisassemblyRRTStrategy<MPTraits>::VID
DisassemblyRRTStrategy<MPTraits>::
Extend(const VID _nearVID, const GroupCfgType& _qRand, const bool _lp) {
  const std::string label = this->GetNameAndLabel() + "::Extend()";
  GroupRoadmapType* const graph = this->GetGroupRoadmap();

  GroupLPOutput<MPTraits> lpOutput(graph);
  std::pair<VID, bool> extension{INVALID_VID, false};
  bool extended;
  CDInfo cdInfo(true);
  const GroupCfgType& qNear = graph->GetVertex(_nearVID);
  GroupCfgType qNew(graph);

  auto const ex = this->GetExtender(m_exLabel);

  if(m_timeEverything)
    this->GetStatClass()->StartClock("Extend(internal)::ExtendClock");

  // Create shuffled version of the active robot list and extend with that.
  Formation orderedActiveRobots = m_activeRobots;
  if(orderedActiveRobots.size() > 1) {
    // Since only leader (first entry) matters, just swap a random position:
    const size_t ind = LRand() % orderedActiveRobots.size();
    if(ind > 0)
      std::swap(orderedActiveRobots[0], orderedActiveRobots[ind]);
  }
  extended = ex->Extend(qNear, _qRand, qNew, lpOutput, cdInfo,
                        orderedActiveRobots);

  if(m_timeEverything) {
    this->GetStatClass()->StopClock("Extend(internal)::ExtendClock");
    this->GetStatClass()->StartClock("Extend(internal)::AfterExtendClock");
  }

  if(extended)
    extension = AddNode(qNew); // qNew gets added to the roadmap here.

  // Always add active bodies to the edge, as it's necessary edge info.
  lpOutput.SetActiveRobots(orderedActiveRobots);

  const VID newVID = extension.first;
  const bool nodeIsNew = extension.second;

  if(m_doManhattanLike) { // Populate colliding body info.
    // The Extender's CDInfo will be from the last ticked Cfg, not the free one.
    // You can see that this is the case by looking at BasicExtender.h:200.
    m_collidingParts.clear();
    //The index of m_bodyDists is the body number, the entry is the distance.
    for(size_t part = 0; part < cdInfo.m_selfClearance.size(); part++)
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
      vc->IsValid(qNew, cdInfo, label, m_activeRobots);
      if(newVID >= m_vidClearances.size())
        m_vidClearances.resize(newVID * 2, 0);

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
    if(m_timeEverything)
      this->GetStatClass()->StopClock("Extend(internal)::AfterExtendClock");
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

//  if(nodeIsNew && !_lp)
//    ConnectNeighbors(newVID);

  if(m_timeEverything)
    this->GetStatClass()->StopClock("Extend(internal)::AfterExtendClock");

  if((_lp && !validLP) || !nodeIsNew)
    return INVALID_VID; // If failed lp or node isn't new, return invalid vid.

  return newVID;
}


template <typename MPTraits>
std::pair<typename DisassemblyRRTStrategy<MPTraits>::VID, bool>
DisassemblyRRTStrategy<MPTraits>::
AddNode(const GroupCfgType& _newCfg) {
  GroupRoadmapType* const g = this->GetGroupRoadmap();
  const VID newVID = g->AddVertex(_newCfg);
  const bool nodeIsNew = (newVID == (g->get_num_vertices() - 1));
  if(nodeIsNew) {
    if(this->m_debug)
      std::cout << "m_tree.size() = " << m_tree.size()
                << " | m_tree.capacity() = " << m_tree.capacity()
                << std::endl;
    m_tree.push_back(newVID);
    if(this->m_debug)
      std::cout << "\tAdding VID " << newVID << " to tree." << std::endl;
  }
  else if(this->m_debug)
    std::cout << "\tVID " << newVID << " already exists, not adding." << std::endl;

  if(this->m_debug)
    std::cout << "RRT: roadmap state after adding vertex:" << std::endl
              << g->PrettyPrint();

  return std::make_pair(newVID, nodeIsNew);
}


template <typename MPTraits>
void
DisassemblyRRTStrategy<MPTraits>::
AddEdge(VID _source, VID _target, const GroupLPOutput<MPTraits>& _lpOutput) {
  GroupRoadmapType* g = this->GetGroupRoadmap();
  if(m_gt.find("UNDIRECTED") != std::string::npos)
    g->AddEdge(_source, _target, _lpOutput.m_edge);
  else
    g->AddEdge(_source, _target, _lpOutput.m_edge.first);
//  g->GetVertex(_target).SetStat("Parent", _source);

  if(this->m_debug)
    std::cout << "\tAdding Edge (" << _source << ", " << _target << ")." << std::endl;
}

/*------------------------------ Tree Helpers --------------------------------*/

template <typename MPTraits>
typename DisassemblyRRTStrategy<MPTraits>::VID
DisassemblyRRTStrategy<MPTraits>::
ExpandTree(GroupCfgType& _target) {
  VID nearestVID = FindNearestNeighbor(_target, m_tree);
  return this->ExpandTree(nearestVID, _target);
}


template <typename MPTraits>
typename DisassemblyRRTStrategy<MPTraits>::VID
DisassemblyRRTStrategy<MPTraits>::
ExpandTree(const VID _nearestVID, const GroupCfgType& _target) {
  if(m_timeEverything)
    this->GetStatClass()->StartClock("ExpandTree::ExtendClock");

  // Try to extend from the _nearestVID to _target
  const VID newVID = this->Extend(_nearestVID, _target);

  if(m_timeEverything) {
    this->GetStatClass()->StopClock("ExpandTree::ExtendClock");
    this->GetStatClass()->StartClock("ExpandTree::PerturbCollidingPartsClock");
  }

  bool expanded = (newVID != INVALID_VID);
  VID qNear;//For manhattan-like algorithm
  if(expanded)
    qNear = newVID;
  else
    qNear = _nearestVID;
  if(m_doManhattanLike && !m_collidingParts.empty())
    PerterbCollidingParts(qNear, expanded);

  if(m_timeEverything) {
    this->GetStatClass()->StopClock("ExpandTree::PerturbCollidingPartsClock");
  }

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
  auto const graph = this->GetGroupRoadmap();
  const GroupCfgType& qNear = graph->GetVertex(_qNear);

  auto const sampler = dynamic_pointer_cast<MaskedSamplerMethodGroup<MPTraits>>(
                                        this->GetSampler(this->m_samplerLabel));

  //Save the active bodies from the VC so it can be replaced at the end.
  const Formation prevActiveBodies = sampler->GetActiveRobots();

  if(prevActiveBodies != m_activeRobots)
    throw RunTimeException(WHERE, "Sampler's and VC's active bodies don't match!");

  //Set this, as we need to mask given the sample we are to perturb from.
  // This is because we might have already made progress on the main part this
  // RRT is for, so relative to _qNear we'll only be moving one other part in
  // here, but relative to the root, it might look like more than one part moving.
  const GroupCfgType samplersPrevStartCfg = sampler->GetStartCfg();
  sampler->SetStartCfg(qNear);

  if(this->m_debug)
    std::cout << "Saved previous active body/ies: " << prevActiveBodies
              << std::endl;

  for(size_t i = 0; i < m_maxPerturbIterations && !m_collidingParts.empty(); ++i) {
    if(this->m_debug)
      std::cout << "PerterbCollidingParts: m_collidingParts = "
                << m_collidingParts << std::endl;
    std::vector<GroupCfgType> successfulSamples;

    //Replace mask on sampler with the parts in m_collidingParts
    //We just use a random strategy for which part to perturb
    const Formation partToAdjust =
                          {m_collidingParts[LRand() % m_collidingParts.size()]};
//    vc->SetBodyNumbers(partToAdjust);
    sampler->SetActiveRobots(partToAdjust);
    m_activeRobots = partToAdjust;

    if(this->m_debug)
      std::cout << "Set active body before perturb sampling to: "
                << partToAdjust << std::endl;

    //Create perterbCfg from this
    sampler->Sample(1, 1, this->GetEnvironment()->GetBoundary(),
                    std::back_inserter(successfulSamples));

    //TODO: I'm removing this as the way samples are generated shouldn't matter,
    //      LPs/extenders should choose which leader to use.
//    m_lastSamplesLeaderBody = sampler->GetLastSamplesLeaderBody();
//    if(this->m_debug) {
//      std::cout << "After perturb sampling, last leader body = "
//                << m_lastSamplesLeaderBody << std::endl;
//      if(m_lastSamplesLeaderBody != partToAdjust[0])
//        throw RunTimeException(WHERE, "The part to adjust wasn't sampled for!");
//    }

    if(successfulSamples.empty())
      continue; // Call it a failed iteration if a perturbation wasn't possible.

    ++m_numPerturbCfgsGenerated;

    const GroupCfgType& perterbCfg =
                          successfulSamples[LRand() % successfulSamples.size()];

    //Save previous parts, since Extend() will clobber the data in the member.
    const std::vector<size_t> prevCollidingParts = m_collidingParts;

    if(m_timeEverything)
      this->GetStatClass()->StartClock("PerturbCollidingParts::ExtendClock");

    //Extend towards perterbCfg and add to the tree if not too similar.
    //NOTE will clear and replace all elements in m_collidingParts!
    const VID newVID = this->Extend(_qNear, perterbCfg);

    if(m_timeEverything)
      this->GetStatClass()->StopClock("PerturbCollidingParts::ExtendClock");

    if(newVID != INVALID_VID) {
      _expanded = true;//It should be correct to keep updating this.
      _qNear = newVID;
    }

    if(this->m_debug)
      std::cout << "prevCollidingParts = " << prevCollidingParts << std::endl
                << "new colliding parts = " << m_collidingParts << std::endl;

    //Update the colliding parts not to have any of the ones just considered:
    for(size_t part : prevCollidingParts)
      m_collidingParts.erase(
          remove(m_collidingParts.begin(), m_collidingParts.end(), part),
          m_collidingParts.end());

    if(++i > m_maxPerturbIterations) {
      if(this->m_debug)
        std::cout << "Maxed out iterations of PerterbCollidingParts!"
                  << std::endl;
      break;
    }

    if(this->m_debug)
      std::cout << "Updated new colliding parts = " << m_collidingParts
                << std::endl;
  }

  // Important! Reset the sampler's mask and VC's active bodies:
//  vc->Set(prevActiveBodies);
  sampler->SetActiveRobots(prevActiveBodies);
  sampler->SetStartCfg(samplersPrevStartCfg);
  m_activeRobots = prevActiveBodies;

  if(this->m_debug)
    std::cout << "Set sampler's and VC's active bodies back to: "
              << prevActiveBodies << std::endl
              << "Leaving PerterbCollidingParts" << std::endl;
}

#endif
