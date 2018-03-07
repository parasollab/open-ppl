#ifndef DISASSEMBLY_RRT_STRATEGY_H_
#define DISASSEMBLY_RRT_STRATEGY_H_

#include <iomanip>
#include "MPStrategyMethod.h"
#include "MPLibrary/Extenders/ActiveBodyExtender.h"
#include "MPLibrary/MapEvaluators/RRTQuery.h"
#include "MPLibrary/MapEvaluators/StrategyStateEvaluator.h"
#include "MPLibrary/Samplers/MaskedSamplerMethod.h"
#include "MPLibrary/ValidityCheckers/SpecificBodyCollisionValidity.h"


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
    typedef typename std::vector<unsigned int> Subassembly;
    typedef typename MPBaseObject<MPTraits>::MapEvaluatorPointer MapEvaluatorPointer;

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
    const CfgType* m_startCfg{nullptr};
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

    /// Get a random configuration to grow towards, also returns the configuration
    /// that was chosen to mask relative to for composite CSpace.
    virtual std::pair<CfgType, std::size_t> SelectTarget();

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

    string m_strategyLabel; ///< This strategy's label
    string m_stateMELabel; ///< The ME label for the StrategyStateEvaluator used
    string m_successfulCheckME; ///< The ME label that determines the successful
                                ///< flag of this strategy (should be minimum
                                ///< clearance).

    ///@}
    ///@name RRT Properties
    ///@{

    size_t m_maxTrial;      ///< The number of samples taken for disperse search.

    VID m_startCfgVID{0};
    Subassembly m_activeBodies; //For the extender
    unsigned int m_lastSamplesLeaderBody; // a default makes no sense here.

    ///< Provides path to node with highest clearance if it fails:
    bool m_returnBestPathOnFailure{false};

    bool m_doManhattanLike{false};///< The flag for ML-RRT (part perturbations).
    std::vector<unsigned int> m_collidingParts;///< The list of colliding parts for ML-RRT
    std::vector<double> m_vidClearances;///< The clearance indexed by VID for ML-RRT

    size_t m_maxPerturbIterations{20};

    bool m_timeEverything{true};

    unsigned int m_numCfgsGenerated{0};
    unsigned int m_numPerturbCfgsGenerated{0};

    //Appends some RRT stats to a file if true in Finalize()
    bool m_recordNodeCountGenerated{false};

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
  m_samplerLabel = _node.Read("samplerLabel", true, "", "Sampler label (should "
                                                        "be a masked version)");

  m_strategyLabel = _node.Read("label", true, "NOT SET", "Strategy's label");

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
Print(ostream& _os) const {
  if(this->m_debug) {
  _os << "DisassemblyRRTStrategy::Print (debug is true)" << std::endl
      << "  MP objects:" << std::endl
      << "\tDistance Metric: " << m_dmLabel << std::endl
      << "\tNeighborhood Finder: " << m_nfLabel << std::endl
      << "\tValidity Checker: " << m_vcLabel << std::endl
      << "\tConnection Method: " << m_ncLabel << std::endl
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
  m_successes = 0;
  m_trials = 0;

  m_numPerturbCfgsGenerated = 0;
  m_numCfgsGenerated = 0;

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

  if(this->UsingQuery()) { //Kept from original RRT to ensure a query isn't used
    throw RunTimeException(WHERE, "Not set up to handle queries!");
  }
  else if(!m_samplerLabel.empty() && m_startCfg && m_startCfg->DOF() != 0) {
    // the startCfg will be the root in this roadmap, and we only use one tree.
    m_startCfgVID = g->AddVertex(*m_startCfg);
    m_tree = TreeType(1, m_startCfgVID);
  }
  else // If no sampler and start cfg, throw an exception
    throw RunTimeException(WHERE, "RRT strategy for disassembly not "
                                  "initialized correctly!");

  //There is some state built into our ME's for RRT, so ensure it starts over.
  for(auto label : this->m_meLabels)
    this->GetMapEvaluator(label)->Initialize();
}


template <typename MPTraits>
void
DisassemblyRRTStrategy<MPTraits>::
Iterate() {
  auto graph = this->GetRoadmap()->GetGraph();
  auto const stats = this->GetStatClass();
  ++m_trials;
  if(this->m_debug)
    std::cout << "*** Starting iteration " << m_trials << " "
         << "***************************************************" << std::endl
         << "Graph has " << graph->get_num_vertices()
         << " vertices." << std::endl;

//  if(this->m_debug)
    stats->StartClock("SelectDirectionClock");

  // Find random growth direction.
  std::pair<CfgType, VID> target = this->SelectTarget();

  // Check the sample was actually successful:
  if(target.first == *m_startCfg) {
    if(this->m_debug) {
      std::cout << "Sample failed! Re-attempting, but counting this iteration."
                << std::endl;
    }
    stats->StopClock("SelectDirectionClock");
    return;
  }

  ++m_numCfgsGenerated;

  if(this->m_debug) {
    std::cout << "Random direction selected: " << target.first.PrettyPrint()
              << std::endl;
  }

  stats->StopClock("SelectDireDisassemblyRRTctionClock");
  stats->StartClock("FindNearestNeighborClock");

//  VID nearestVID;
//  if(m_activeBodies.size() > 1)
//    nearestVID = target.second; // Only allow subs to extend from its sample's start
//  else
//    nearestVID = FindNearestNeighbor(target.first, m_tree);

  // Find the nearest configuration to target within the current tree
  const VID nearestVID = FindNearestNeighbor(target.first, m_tree);


//  if(this->m_debug) {
    stats->StopClock("FindNearestNeighborClock");
    stats->StartClock("ExpandTreeClock");
//  }

  // Expand current tree
  const VID newVID = this->ExpandTree(nearestVID, target.first);
  if(newVID != INVALID_VID) {
    ++m_successes;
    if(this->m_debug) {
      CfgType tempCfg = graph->GetVertex(newVID);
      std::cout << "Added cfg (VID = " << newVID << ") = " <<
                tempCfg << std::endl;
    }
  }
//  if(this->m_debug)
    stats->StopClock("ExpandTreeClock");
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

  if(m_recordNodeCountGenerated) {
    //Append the data for number of nodes generated:
    std::ofstream nodeGenFile;
    nodeGenFile.open("NodesGenerated.data", std::ios_base::app);
    nodeGenFile << m_numCfgsGenerated << " " << m_numPerturbCfgsGenerated << " "
                << mapVerts << std::endl;
  }

  MapEvaluatorPointer me = this->GetMapEvaluator(m_successfulCheckME);
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

    CDInfo cdInfo(true); // For clearance info
    CfgType startCfg = *m_startCfg;
    vc->IsValid(startCfg, cdInfo, this->GetNameAndLabel());

    double maxClearance = cdInfo.m_minDist; //Start off with the start cfg.
    VID maxClearanceVid = m_startCfgVID;
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
std::pair<typename MPTraits::CfgType, std::size_t>
DisassemblyRRTStrategy<MPTraits>::
SelectTarget() {
  std::shared_ptr<MaskedSamplerMethod<MPTraits> > const sampler =
                        dynamic_pointer_cast<MaskedSamplerMethod<MPTraits> >(
                        this->GetSampler(this->m_samplerLabel));
  auto const graph = this->GetRoadmap()->GetGraph();
  const unsigned int numVerts = graph->get_num_vertices();
  const VID randomVID = LRand() % numVerts;

  //Set the sampler to mask relative to a random cfg in the tree:
  sampler->SetStartCfg(graph->GetVertex(randomVID));

  std::vector<CfgType> successfulSamples;
  // Get one sample only trying one time.
  sampler->Sample(1, 1, this->GetEnvironment()->GetBoundary(),
                  back_inserter(successfulSamples));
  if(!successfulSamples.empty()) {
    //Save body rotated about to give to the extender for intermediate planning.
    m_lastSamplesLeaderBody = sampler->GetLastSamplesLeaderBody();
    //Choose a random cfg (if size = 1, it will just take the 0 index)
    return std::make_pair(successfulSamples[LRand() % successfulSamples.size()],
                          randomVID);
  }

  return std::make_pair(*m_startCfg, randomVID); // Unsuccessful sample.
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
  auto const graph = this->GetRoadmap()->GetGraph();

  vector<pair<VID, double>> neighbors;
  auto const nf = this->GetNeighborhoodFinder(m_nfLabel);
  nf->FindNeighbors(this->GetRoadmap(),
      _tree.begin(), _tree.end(),
      _tree.size() == graph->get_num_vertices(),
      _cfg, back_inserter(neighbors));
  const VID nearestVID = neighbors[0].first;

  return nearestVID;
}


template <typename MPTraits>
void
DisassemblyRRTStrategy<MPTraits>::
ConnectNeighbors(VID _newVID) {
  // Make sure _newVID is valid and graph type includes GRAPH
  if(_newVID == INVALID_VID || m_gt.find("GRAPH") == std::string::npos)
    return;

  this->GetStatClass()->StartClock("ConnectNeighborsClock");

  vector<VID> currentVID(1, _newVID);
  this->GetConnector(m_ncLabel)->Connect(this->GetRoadmap(),
      currentVID.begin(), currentVID.end(),
      m_tree.begin(), m_tree.end(),
      m_tree.size() == this->GetRoadmap()->GetGraph()->get_num_vertices());

  this->GetStatClass()->StopClock("ConnectNeighborsClock");
}

/*----------------------------- Growth Helpers -------------------------------*/

template <typename MPTraits>
typename DisassemblyRRTStrategy<MPTraits>::VID
DisassemblyRRTStrategy<MPTraits>::
Extend(const VID _nearVID, const CfgType& _qRand, const bool _lp) {
  const string label = this->GetNameAndLabel() + "::Extend()";
  GraphType* const graph = this->GetRoadmap()->GetGraph();

  LPOutput<MPTraits> lpOutput;
  pair<VID, bool> extension{INVALID_VID, false};
  bool extended;
  CDInfo cdInfo(true);
  const CfgType& qNear = graph->GetVertex(_nearVID);
  CfgType qNew(this->GetTask()->GetRobot());

  // Some (hefty) validation that is important to do if debugging is needed:
  if(this->m_debug) {
    auto const sampler = dynamic_pointer_cast<MaskedSamplerMethod<MPTraits> >
                                         (this->GetSampler(this->m_samplerLabel));
    const CfgType& samplerStartCfg = sampler->GetStartCfg();
    const unsigned int numBodies = qNew.GetMultiBody()->GetNumBodies();
    const unsigned int posDofsPerBody = qNew.PosDOF();
    const unsigned int dofsPerBody = posDofsPerBody + qNew.OriDOF();
    const CfgType offsetCfg = _qRand - qNear;

    if(find(m_activeBodies.begin(), m_activeBodies.end(), m_lastSamplesLeaderBody)
        == m_activeBodies.end())
      throw RunTimeException(WHERE, "m_lastRotAboutBody not in m_activeBodies!");

    std::vector<double> uniformTranslationDofs(posDofsPerBody, 0.0);
    const bool translationOnly = posDofsPerBody == dofsPerBody;
    if(translationOnly) {
      //We know that in the case of no rotations, a subassembly will have
      // identical relative translation among all parts (if correct).
      for(unsigned int i = 0; i < dofsPerBody; ++i) {
        const unsigned int ind = m_activeBodies[0]*dofsPerBody + i;
        uniformTranslationDofs[i] = offsetCfg[ind];
      }
    }

    for(unsigned int bodyNum = 0; bodyNum < numBodies; ++bodyNum) {
      //If the body is found in m_activeBodies, it's validly moving.
      const bool isMovingBody = find(m_activeBodies.begin(), m_activeBodies.end(),
                                                 bodyNum) != m_activeBodies.end();
      //Note that we only look at translation since angle comparison is more involved.
      for(unsigned int dofNum = 0; dofNum < dofsPerBody; ++dofNum) {
        const unsigned int ind = bodyNum*dofsPerBody + dofNum;
        if(isMovingBody && translationOnly &&
           fabs(uniformTranslationDofs.at(dofNum) - offsetCfg[ind]) > 1e-10) {
          throw RunTimeException(WHERE, "The extension would have messed up the "
                                        "subassembly!");
        }

        if(!isMovingBody && fabs(offsetCfg[ind]) > 1e-10) {
          std::cout << "Error reached, the start cfg in the sampler was:"
                    << std::endl << samplerStartCfg.PrettyPrint() << std::endl
                    << "Offset cfg = " << offsetCfg.PrettyPrint() << std::endl;
          throw RunTimeException(WHERE, "The Extension would have moved bodies "
                                        "not in the m_activeBodies list!");
        }
      }
    }
  }

  auto const ex = this->GetExtender(m_exLabel);
  //Handle the case of disassembly expansion (to handle subassemblies).
  std::shared_ptr<ActiveBodyExtender<MPTraits> > activeBodyEx =
                        dynamic_pointer_cast<ActiveBodyExtender<MPTraits> >(ex);
  ///@TODO: When done with the extender, enforce this:
//  if(!activeBodyEx)
//    throw RunTimeException(WHERE, "You should be using an active body extender "
//                                  "when doing disassembly planning");

  if(m_timeEverything)
    this->GetStatClass()->StartClock("Extend(internal)::ExtendClock");

  //m_lastRotAboutBody is retrieved from the sampler after generating the sample
  std::vector<unsigned int> orderedActiveBodies = {m_lastSamplesLeaderBody};
  if(activeBodyEx) {
    if(m_activeBodies.size() > 1) {
      // The body to rotate about is first in the vector:
      for(const unsigned int body : m_activeBodies)
        if(body != orderedActiveBodies[0])
          orderedActiveBodies.push_back(body);
    }
    activeBodyEx->SetActiveBodies(orderedActiveBodies);
    extended = activeBodyEx->Extend(qNear, _qRand, qNew, lpOutput, cdInfo);
  }
  else {
    orderedActiveBodies = m_activeBodies; // Just as a placeholder
    extended = ex->Extend(qNear, _qRand, qNew, lpOutput, cdInfo);
  }

  if(m_timeEverything) {
    this->GetStatClass()->StopClock("Extend(internal)::ExtendClock");
    this->GetStatClass()->StartClock("Extend(internal)::AfterExtendClock");
  }

  if(extended)
    extension = AddNode(qNew); // qNew gets added to the roadmap here.

  // Always add active bodies to the edge, as it's necessary edge info.
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

  if(nodeIsNew && !_lp)
    ConnectNeighbors(newVID);

  if(m_timeEverything)
    this->GetStatClass()->StopClock("Extend(internal)::AfterExtendClock");

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
      std::cout << "m_tree.size() = " << m_tree.size()
                << " | m_tree.capacity() = " << m_tree.capacity()
                << std::endl;
    m_tree.push_back(newVID);
    if(this->m_debug)
      std::cout << "\tAdding VID " << newVID << " to tree." << std::endl;
  }
  else if(this->m_debug)
    std::cout << "\tVID " << newVID << " already exists, not adding." << std::endl;

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
    std::cout << "\tAdding Edge (" << _source << ", " << _target << ")." << std::endl;
}

/*------------------------------ Tree Helpers --------------------------------*/

template <typename MPTraits>
typename DisassemblyRRTStrategy<MPTraits>::VID
DisassemblyRRTStrategy<MPTraits>::
ExpandTree(CfgType& _target) {
  VID nearestVID = FindNearestNeighbor(_target, m_tree);
  return this->ExpandTree(nearestVID, _target);
}


template <typename MPTraits>
typename DisassemblyRRTStrategy<MPTraits>::VID
DisassemblyRRTStrategy<MPTraits>::
ExpandTree(const VID _nearestVID, const CfgType& _target) {
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
  auto const graph = this->GetRoadmap()->GetGraph();
  const CfgType& qNear = graph->GetVertex(_qNear);

  auto const sampler = dynamic_pointer_cast<MaskedSamplerMethod<MPTraits>>(
                                        this->GetSampler(this->m_samplerLabel));
  auto const vc = dynamic_pointer_cast<SpecificBodyCollisionValidity<MPTraits>>(
                                     this->GetValidityChecker(this->m_vcLabel));

  //Save the active bodies from the VC so it can be replaced at the end.
  const Subassembly prevActiveBodies = vc->GetBodyNumbers();

  //TODO: move into debug guard:
  if(prevActiveBodies != sampler->GetActiveBodies() || prevActiveBodies != m_activeBodies)
    throw RunTimeException(WHERE, "Sampler's and VC's active bodies don't match!");

  //Set this, as we need to mask given the sample we are to perturb from.
  // This is because we might have already made progress on the main part this
  // RRT is for, so relative to _qNear we'll only be moving one other part in
  // here, but relative to the root, it might look like more than one part moving.
  const CfgType prevSamplerStartCfg = sampler->GetStartCfg();
  sampler->SetStartCfg(qNear);

  if(this->m_debug)
    std::cout << "Saved previous active body/ies: " << prevActiveBodies
              << std::endl;

  for(size_t i = 0; i < m_maxPerturbIterations && !m_collidingParts.empty(); ++i) {
    if(this->m_debug)
      std::cout << "PerterbCollidingParts: m_collidingParts = "
                << m_collidingParts << std::endl;
    std::vector<CfgType> successfulSamples;

    //Replace mask on sampler with the parts in m_collidingParts
    //We just use a random strategy for which part to perturb
    const Subassembly partToAdjust =
                          {m_collidingParts[LRand() % m_collidingParts.size()]};
    vc->SetBodyNumbers(partToAdjust);
    sampler->SetMaskByBodyList(partToAdjust);
    m_activeBodies = partToAdjust;

    if(this->m_debug)
      std::cout << "Set active body before perturb sampling to: "
                << partToAdjust << std::endl;

    //Create perterbCfg from this
    sampler->Sample(1, 1, this->GetEnvironment()->GetBoundary(),
                    back_inserter(successfulSamples));
    m_lastSamplesLeaderBody = sampler->GetLastSamplesLeaderBody();

    if(this->m_debug) {
      std::cout << "After perturb sampling, last leader body = "
                << m_lastSamplesLeaderBody << std::endl;
      if(m_lastSamplesLeaderBody != partToAdjust[0])
        throw RunTimeException(WHERE, "The part to adjust wasn't sampled for!");
    }

    if(successfulSamples.empty())
      continue; // Call it a failed iteration if a perturbation wasn't possible.

    ++m_numPerturbCfgsGenerated;

    const CfgType& perterbCfg =
                          successfulSamples[LRand() % successfulSamples.size()];

    //Save previous parts, since Extend() will clobber the data in the member.
    const std::vector<unsigned int> prevCollidingParts = m_collidingParts;

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
    for(unsigned int part : prevCollidingParts)
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
  vc->SetBodyNumbers(prevActiveBodies);
  sampler->SetMaskByBodyList(prevActiveBodies);
  sampler->SetStartCfg(prevSamplerStartCfg);
  m_activeBodies = prevActiveBodies;

  if(this->m_debug)
    std::cout << "Set sampler's and VC's active bodies back to: "
              << prevActiveBodies << std::endl
              << "Leaving PerterbCollidingParts" << std::endl;
}

#endif
