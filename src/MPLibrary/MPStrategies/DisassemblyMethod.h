#ifndef DISASSEMBLY_METHOD_H_
#define DISASSEMBLY_METHOD_H_

#include "MPStrategyMethod.h"
#include "DisassemblyRRTStrategy.h"
#include "MPLibrary/Samplers/MaskedSamplerMethodGroup.h"
#include "MPLibrary/MapEvaluators/GroupQuery.h"
#include "MPLibrary/MapEvaluators/StrategyStateEvaluator.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/RobotGroup/GroupUtils.h"
#include <algorithm> //for std::find

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Interface for disassembly planning
///
/// It holds the main parameters and contains the 3 main functions:
/// SelectExpansionCfg, SelectSubassembly and Expand
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DisassemblyMethod : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef typename MPTraits::MPSolution        MPSolution;
    typedef typename GroupCfgType::Formation     Formation;

    // Group Types:
    typedef typename MPTraits::GroupPathType     GroupPath;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename GroupRoadmapType::VID       VID;
    typedef typename MPTraits::GroupWeightType   GroupWeightType;
    typedef std::vector<GroupCfgType>            CfgPath;
    typedef std::vector<VID>                     VIDPath;


    enum class Approach { mating, rrt, invalid };
    enum class State { singlePart, multiPart, done };

    struct DisassemblyNode {
      //The vid for the cfg that is to be expanded from in this node.
      VID vid;
      size_t depth = 0;
      size_t branch = 0;

      //Determine whether deterministic strategies have been exhausted for this
      // node. This is node-specific, but can change given progress made.
      bool determinismExhausted = false;

      //Two weight members: (best) local weight and best cumulative weight.
      // The local weight shall correspond to the weight for parents[0]
      double localWeight = 0;
      double bestCumulativeWeight = 0;

      // List of removed, but not yet disassembled, subassemblies.
      std::vector<Formation> usedSubassemblies;

      //The remaining parts of the assembly at initial position for this state.
      Formation initialParts;

      //In a node, the removalPaths either matches up with the parent (for
      // graph methods, where each edge represents only a single part
      // or subassembly removal) or in preemptive, the removedParts and
      // removalPaths matches up, since we can remove many parts in one edge.
      std::vector<size_t> removedParts;
      std::vector<VIDPath> removalPaths;

      //Parent/child relationships for the tree structure:
      //Note: for the A*-like search, it will be assumed/maintained that the
      // best path's parent node (what bestCumulativeWeight corresponds to)
      // will be in parents[0]
      std::vector<DisassemblyNode*> parents;

      //The edge weight is paired with children so we don't have to maintain
      // the order in this vector, but could also be easily sorted.
      std::vector<std::pair<double, DisassemblyNode*> > children;

      std::vector<size_t> GetCompletePartList() const {
        if(m_completePartList.empty()) {
          std::vector<size_t> parts = initialParts;
          for (auto &usedSub : usedSubassemblies)
            for (auto &id : usedSub)
              parts.push_back(id);
          return parts;
        }
        else
          return m_completePartList;
      }

      //This version will also update the totalRemainingParts member:
      std::vector<size_t> GetCompletePartList() {
        if(m_completePartList.empty()) {
          m_completePartList = initialParts;
          for (auto &usedSub : usedSubassemblies)
            for (auto &id : usedSub)
              m_completePartList.push_back(id);
          m_totalRemainingParts = m_completePartList.size();
        }
        return m_completePartList;
      }

      size_t NumberOfRemainingParts() const {
        if(m_totalRemainingParts == 0) {
          size_t num = 0;
          for (auto &usedSub : usedSubassemblies)
            num += usedSub.size();
          return (size_t)(num + initialParts.size());
        }
        return m_totalRemainingParts;
      }

      size_t NumberOfRemainingParts() {
        size_t num = 0;
        if(m_totalRemainingParts == 0) {
          for (auto &usedSub : usedSubassemblies)
            num += usedSub.size();
          m_totalRemainingParts = (num + initialParts.size());
        }
        return m_totalRemainingParts;
      }

    private:

      // Some cached data so we only have to calculate it once per node at most.
      size_t m_totalRemainingParts = 0;
      std::vector<size_t> m_completePartList;
    };

    DisassemblyMethod(
        const map<string, pair<size_t, size_t> >& _matingSamplerLabels =
                                          map<string, pair<size_t, size_t> >(),
        const map<string, pair<size_t, size_t> >& _rrtSamplerLabels =
                                          map<string, pair<size_t, size_t> >(),
        const string _vc = "", const string _singleVc = "",
        const string _lp = "", const string _ex = "", const string _dm = "",
        const std::vector<string>& _evaluatorLabels = std::vector<string>(),
        const double _contactDist = 0.1, const double _neighborDist = 1,
        const double _removePos = 8, const double _subassemblyPos = 5,
        const double _subassemblyOffset = 2, const double _minMatingDist = 0.0);
    DisassemblyMethod(XMLNode& _node);
    virtual ~DisassemblyMethod() {}

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    virtual void Initialize() override;
    virtual void Finalize();

    list<DisassemblyNode>& GetDisassemblyNodes() {return m_disNodes;}

    void PrintDisassemblyTree();

  protected:
    ///@}
    ///@name virtual disassembly methods
    ///@{
    virtual DisassemblyNode* SelectExpansionNode() = 0;
    virtual Formation SelectSubassembly(DisassemblyNode* _q) = 0;
    virtual pair<bool, VIDPath> Expand(DisassemblyNode* _q,
                                           const Formation& _subassembly) = 0;

    ///@}
    ///@name Expanding methods
    ///@{

    // This function will check if the clearance of a mating removal actually
    // passes the equivalent of its map evaluator check for minimum clearance.
    // This is also to check that if parts are embedded in an assembly, as a full
    // mating extension might complete, but not actually remove the part.
    bool EnsureMinimumClearance(GroupCfgType& _cfg, const Formation& _formation = Formation());

    // Create samples for the mating approach and return them as std::vector of cfgs
    std::vector<GroupCfgType> GetMatingSamples(const Formation& _subassembly);
    VIDPath ExpandMatingApproach(const VID _q,
                                               const Formation& _formation,
                                               VID& _newVid);
//    GroupCfg FindClosestRemovalCfg(const GroupCfg& _startCfg,
//                                   const GroupCfg& _endCfg,
//                                   GroupLPOutput<MPTraits>& _lpOutput);

    //Creates a new MPSolution and invokes the RRT strategy on a single piece of
    // the assembly. Then adds the path (if successful) to the roadmap.
    VIDPath ExpandRRTApproach(const VID _q,
                                            const Formation& _formation,
                                            VID& _newVid);

    // Adds a path of Cfgs (nodes and edges) to the roadmap. Aassumed that
    // _startVID is connected to _path[0], and _path[n] is connected to
    // _path[n+1] for all cfgs in _path.
    // NOTE: Make sure to put the original solution back into the MPLibrary
    // before calling this, as it uses the library to get the correct roadmap.
    VIDPath AddDissassemblyPathToRoadmap(const std::vector<VID>& _pathVids,
                                      GroupRoadmapType* const _rrtGraph,
                                      VID _roadmapStartVID,
                                      VID& _lastAddedVID);

    // Loops through m_rrtClockStrings and accumulates the times over runs:
    void AccumulateClockTimes(
        const std::map<std::string, ClockClass>& _rrtClocks);

    /// GenerateNode creates a new disassembly node and adds it to the graph.
    /// It requires the parent node, the list of parts removed,
    /// and whether that list is a subassembly or if it's
    /// a list of parts removed in parallel (_isMultiPart).
    /// The final parameter is optional and only used for A* search: the edge
    /// weight from _parent -> this new node.
    DisassemblyNode* GenerateNode(DisassemblyNode* const _parent,
                                  const std::vector<size_t>& _removedParts,
                                  const std::vector<VIDPath>& _removingPaths,
                                  const bool _isMultiPartSubassembly,
                                  const double _localWeight = 0);

    ///@}
    ///@name Subassembly Identification
    ///@{
    void AnalysePartContacts(const VID _q, const Formation& _partIndices);
    void AnalysePartContacts(const VID _q, const size_t _partIndex);

    /// @warning This function does not obey _parts for the generation of the
    /// subassembly candidates if the flag m_generateSubassembliesOnce is true.
    /// In some cases this is slower.
    std::vector<Formation> GenerateSubassemblies(const VID _currentVid,
                                                 const Formation& _parts);


    //The point of this helper function is to be given all of the valid/pruned
    // (non-obstacle) directions, as well as the 3d matrix/tensor that describes
    // for each body (the first index), which direction (the second index) causes
    // a collision with which other body (the last index). This means we can do:
    // m_bodyContacts[whichBody][whichDirection] to get all of the colliding Parts
    // for the piece at index whichBody and direction.
    std::vector<Formation> GenerateDirectionalCollisionSubassemblies(
                                const VID _currentVid, const Formation& _parts);
    std::vector<Formation> GeneratePredefinedSubassemblies(
                                                       const Formation& _parts);

    void PrintDirectionalCollisionInfo(const Formation& _parts);


    //This helper function will recursively check for parts that co-collide given
    // a certain direction. For each new part that is added to the subassembly,
    // we ensure that that part and all its collisions for this direction are
    // valid and also checked.
    void GetCollidingPartsRecursive(const size_t _partInd,
                                    const size_t _dirInd,
                                    Formation& _currentSubassembly);

    //Check if _subassemblies already has _subassembly as a member.
    bool IsDuplicateSubassembly(const Formation& _subassembly,
                                const std::vector<Formation>& _subassemblies);

    //Check if subassembly _a is identical to subassembly _b.
    bool SameSubassembly(const Formation& _a, const Formation& _b) const;

    ///@}
    ///@name adjacency identification
    ///@{

    /// Generates the adjacency information (currently used for subassembly
    /// identification.
    void GenerateAdjacencyMats();

    /// These functions are currently unused, but serve as an example for how
    /// to interface with the adjacency matrix, since it's not a straightforward
    /// structure.
    std::vector<size_t> GetContactParts(const size_t _partId);
    bool CheckContact(const size_t _firstPart, const size_t _secondPart);
    bool CheckNeighbor(const size_t _firstPart, const size_t _secondPart);

    ///@}
    ///@name Output file(s) generation function and helpers
    ///@{

    /// Given the vids from the Disassembly Graph nodes, this will get the
    /// complete roadmap path that includes vids and trajectories not directly
    /// represented by a node in the DG.
    GroupPath GenerateFullDisassemblyPath(const VIDPath& _path);

    void GetSequenceStats();
    void RecursivePathStats(DisassemblyNode* _node, double _curDist,
                            VIDPath& _curPath);
    void VerifyNodes();

    void GenerateDotFile();
    void GenerateDotNodeStruct(ofstream& _dotFile,
                               const DisassemblyNode* const _node) const;
    void GenerateDotEdges(ofstream& _dotFile) const;

    map<string, pair<size_t, size_t> > m_matingSamplerLabels;
    map<string, pair<size_t, size_t> > m_rrtSamplerLabels;
    std::string m_rrtDMLabel;

    string m_vcLabel;          ///< The validity checker label.
    string m_singleVcLabel;
    string m_lpLabel;          ///< The local planner label.
    string m_exLabel;          ///< The extender label.
    string m_dmLabel;          ///< The distance metric label.
    string m_rrtStrategyLabel; ///< The RRT label for extensions.

    string m_strategyLabel;    ///< The strategy label (needed for state ME)
    string m_stateMELabel;

    size_t m_numParts = 0;       ///< number of free Parts of the robot
    VID    m_rootVid = 0;     ///< root VID of the graph

    VID    m_lastAddedVID = 0; ///< Keep track so we can connect all CC's

    //m_disNodes is the master list of all the disassembly nodes in play during
    // execution. Everything else that uses a disassembly node should use a
    // pointer to the nodes in this list.
    list<DisassemblyNode> m_disNodes;
    DisassemblyNode* m_rootNode{nullptr};

    // connectivity graph variables
    std::vector<std::vector<bool>> m_contactMat;  ///< adjacency map for body contacts
    std::vector<std::vector<bool>> m_neighborMat; ///< adjacency map for body neighbors
    double m_contactDist = 0.1;
    double m_neighborDist = 1;

    /// This is basically a matrix of std::vectors, where each entry corresponds to
    /// a set of other parts found to collide when the indexed part is moved in
    /// the indexed direction.
    std::vector< std::vector< std::vector< size_t > > > m_partContacts;
    ///< body distances of the last mating approach
    ///< the first index is the body
    ///< the second index is the index of the normal
    ///< the std::vector at [bodyInd][normalInd] is the list of bodies
    ///< that bodyInd collides with when moving along normalInd.

    //Each std::vector here is a list of direction indices that are valid to be
    // considered in the subassembly identification.
    //Each std::vector is for one body of the assembly.
    //This means that m_nonObstacleDirs[i] is the list of direction indices that
    // can be used for body i.
    std::vector<std::vector<size_t>> m_nonObstacleDirs;

    // positions and offset for subassembly positioning
    double m_removePos;
    double m_subassemblyPos;
    double m_subassemblyOffset;
    double m_minMatingDist{0.0};
    double m_matingPosResMultiplier{1.0};


    bool m_graphMethod{false};
    bool m_aStarSearch{false};

    ///< A std::vector of predefined subassemblies. Using these as the candidates
    ///< will override the generation using directional collisions.
    std::vector<Formation> m_predefinedSubassemblies;

    /// list of path stats, each pair corresponds to a path, the first is the
    /// number of nodes in the path, the second is the path's distance.
    std::vector<std::pair<size_t, double>> m_seqList;

    //The index-aligned paths (vid sequences) that correspond to the leaf of
    // the path (tree only):
    std::vector<std::pair<size_t, VIDPath > > m_pathVids;

    bool m_printFullPath = true;

    /// For RRT failures, keep the node with highest clearance (and the path to
    /// it) or not.
    bool m_keepBestRRTPathOnFailure{true};

    /// If true, all of the mating directions will be attempted and a node for
    /// each will be added to the roadmap as an initial step for RRT.
    bool m_matingExtensionBeforeRRT = false;


    /// If true, only generate subassemblies once for the root node with all
    /// parts, and then do the matching scheme used for predefined subassemblies
    /// NOTE: This will be superseded by predefined subassemblies!
    bool m_generateSubassembliesOnce{true};
};

template <typename MPTraits>
DisassemblyMethod<MPTraits>::
DisassemblyMethod(
    const map<string, pair<size_t, size_t> >& _matingSamplerLabels,
    const map<string, pair<size_t, size_t> >& _rrtSamplerLabels,
    const string _vc, const string _singleVc, const string _lp,
    const string _ex,
    const string _dm, const std::vector<string>& _evaluatorLabels,
    const double _contactDist,
    const double _neighborDist, const double _removePos,
    const double _subassemblyPos, const double _subassemblyOffset,
    const double _minMatingDist) :
    m_matingSamplerLabels(_matingSamplerLabels),
    m_rrtSamplerLabels(_rrtSamplerLabels), m_vcLabel(_vc),
    m_singleVcLabel(_singleVc),
    m_lpLabel(_lp), m_exLabel(_ex), m_dmLabel(_dm), m_contactDist(_contactDist),
    m_neighborDist(_neighborDist), m_removePos(_removePos),
    m_subassemblyPos(_subassemblyPos), m_subassemblyOffset(_subassemblyOffset),
    m_minMatingDist(_minMatingDist) {
  this->m_meLabels = _evaluatorLabels;
}

template <typename MPTraits>
DisassemblyMethod<MPTraits>::
DisassemblyMethod(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  ParseXML(_node);
}

template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
ParseXML(XMLNode& _node) {
  const double doubleLim = numeric_limits<double>::max();
  m_vcLabel = _node.Read("vcLabel", true, m_vcLabel, "Validity Test Method");
  m_singleVcLabel = _node.Read("singleVcLabel", true, m_singleVcLabel,
      "Single Validity Test Method");
  m_lpLabel = _node.Read("lpLabel", true, m_lpLabel, "Local Planning Method");
  m_exLabel = _node.Read("exLabel", true, m_exLabel, "Extender Method");
  m_dmLabel = _node.Read("dmLabel", true, m_dmLabel, "Distance Metric");

  m_contactDist = _node.Read("contactDist", false, 0.001, 0., doubleLim,
                             "Contact distance");
  m_neighborDist = _node.Read("neighborDist", false, 0.1, 0., doubleLim,
                              "Neighbor distance");

  m_strategyLabel = _node.Read("label", true, "", "Strategy label");

  //This is needed because otherwise we cannot set the strategyLabel that is
  // needed for proper evaluation using this ME.
  m_stateMELabel = _node.Read("stateMELabel", true, "", "Label for ME "
                 "if using a StrategyStateEvaluator for proper initialization");

  m_removePos = _node.Read("removePos", false, 8., -doubleLim, doubleLim,
                           "Removed part position");
  m_subassemblyPos = _node.Read("subassemblyPos", false, 5., -doubleLim,
                                doubleLim, "Subassembly position");
  m_subassemblyOffset = _node.Read("subassemblyOffset", false, 2., -doubleLim,
                                   doubleLim, "Offset of subassembly position");
  m_minMatingDist = _node.Read("minMatingDist", false, 0., -doubleLim,
                               doubleLim, "Minimum mating distance");

  m_printFullPath = _node.Read("printFullPath", false, m_printFullPath, "Print"
                               " full path (if one was found)");

  m_keepBestRRTPathOnFailure = _node.Read("keepBestRRTPathOnFailure", false,
                          m_keepBestRRTPathOnFailure,
                          "Keep the best node (and path to it) when RRT fails");

  m_rrtDMLabel = _node.Read("rrtDMLabel", true, m_rrtDMLabel, "Label for the "
      "RRT's Nearest Neighbor's Distance Metric. Needed for active body setting.");

  m_generateSubassembliesOnce = _node.Read("generateSubassembliesOnce", false,
      m_generateSubassembliesOnce, "If true, will only calculate subassemblies "
      "once for everything, then match based on remaining parts");


  m_matingPosResMultiplier = _node.Read("matingPosResMultiplier", false,
                   m_matingPosResMultiplier, 1e-10, 1e10,
                   "Multiplies the position resolution for mating extensions.");

  std::vector<std::string> subassemblyStrings;
  for(auto& child : _node) {
    if(child.Name() == "MatingSampler") {
      string s = child.Read("method", true, "", "Sampler Label");
      size_t num = child.Read("number", true,
          1, 0, MAX_INT, "Number of samples");
      size_t attempts = child.Read("attempts", false,
          1, 0, MAX_INT, "Number of attempts per sample");
      m_matingSamplerLabels[s] = make_pair(num, attempts);
    }
    if(child.Name() == "RRTSampler") {
      string s = child.Read("method", true, "", "Sampler Label");
      size_t num = child.Read("number", true,
          1, 0, MAX_INT, "Number of samples");
      size_t attempts = child.Read("attempts", false,
          1, 0, MAX_INT, "Number of attempts per sample");
      m_rrtSamplerLabels[s] = make_pair(num, attempts);
    }
    if(child.Name() == "Evaluator")
      this->m_meLabels.push_back(child.Read("label", true, "",
          "Evaluation Method"));
    if(child.Name() == "ExpandRRTStrategy")
      m_rrtStrategyLabel = child.Read("method", true, "", "RRT strategy to use"
          "in the expansion step");

    if(child.Name() == "Subassembly") {
      const std::string subassemblyString = child.Read("parts", true, "",
                                     "The parts of the predefined subassembly");
      subassemblyStrings.push_back(subassemblyString);
    }
  }

  if (_node.Read("useRotation", false, false, "DEPRACATED: Rotation with RRT")) {
    std::cout << std::endl << "Warning: the useRotation flag is no longer "
        "obeyed by assembly planning methods, instead set up the environment "
        "and robot files correctly." << std::endl << std::endl;
  }

  //Parse out each subassembly found from nodes:
  for(const std::string& saStr : subassemblyStrings) {
    std::stringstream ss(saStr);
    Formation sub;
    size_t part;
    //Parse the subassembly string:
    while(ss >> part)
      sub.push_back(part);

    //sort it for easier matching later on:
    std::sort(sub.begin(), sub.end());
    m_predefinedSubassemblies.push_back(sub);
  }
}

template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
Print(ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);

  _os << "DisassemblyMethod::Print" << std::endl
      << "  MP objects:" << std::endl
      << "\tValidityChecker:: " << m_vcLabel << std::endl
      << "\tLocalPlanner:: " << m_lpLabel << std::endl
      << "\tExtender:: " << m_exLabel << std::endl
      << "\tMatingSamplers" << std::endl;
  for(const auto& label : m_matingSamplerLabels)
    _os << "\t\t" << label.first
        << "\tNumber:"   << label.second.first
        << "\tAttempts:" << label.second.second << std::endl;
  _os << "\tRRTSamplers" << std::endl;
  for(const auto& label : m_rrtSamplerLabels)
    _os << "\t\t" << label.first
        << "\tNumber:"   << label.second.first
        << "\tAttempts:" << label.second.second << std::endl;

  _os<<"\tMapEvaluators" << std::endl;
  for(const auto& label : this->m_meLabels)
    _os << "\t\t" << label << std::endl;
}

template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
Initialize() {
  // TODO: remove once this is no longer true:
  #ifndef VIZMO_MAP
    throw RunTimeException(WHERE, "Cannot use this method without the vizmo map"
                                  " option enabled in the Makefile!");
  #endif

  const string label = this->GetNameAndLabel() + "::Initialize()";

  //We shouldn't reset anything that gets set from the xml.
  m_disNodes.clear();
  m_rootNode = nullptr;

  // connectivity graph variables
  m_contactMat.clear();
  m_neighborMat.clear();
  m_partContacts.clear();

  m_nonObstacleDirs.clear();

  this->m_successful = false;

  for(string& label : this->m_meLabels) {
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

  //Initialize everything:
  GroupTask* const groupTask = this->GetGroupTask();
  GroupCfgType root(this->GetGroupRoadmap());
  groupTask->GetStartConstraintCenter(root);
  m_rootVid = this->GetGroupRoadmap()->AddVertex(root);

  m_numParts = root.GetNumRobots();

  GenerateAdjacencyMats();

  if(this->m_debug) {
    std::cout << label << std::endl;
    if(!m_predefinedSubassemblies.empty()) {
      std::cout << "Predefined subassemblies to use: " << std::endl;
      for(const Formation& sub : m_predefinedSubassemblies)
        std::cout << sub << std::endl;
    }
    else
      std::cout << "No predefined subassemblies, will generate subassemblies "
                   "manually" << std::endl;

    const size_t dofsPerBody = root.PosDOF() + root.OriDOF();
    //Print various other info:
    std::cout << "Number of free Parts: " << m_numParts << std::endl
              << "Number of DOFs: "
              << this->GetGroupTask()->GetRobotGroup()->TotalDofs()
              << std::endl << "DOFs per part: " << dofsPerBody << std::endl
              << "DissassemblyMethod::Initialize(): root VID = " << m_rootVid
              << std::endl << "Root cfg = " << root.PrettyPrint() << std::endl;
  }

  auto vc = this->GetValidityChecker(this->m_vcLabel);
  CDInfo cdInfo(true);
  // Note: No formation provided here intentionally; want a full validity check.
  if(!vc->IsValid(root, cdInfo, label)) {
    this->GetGroupRoadmap()->Write(this->GetBaseFilename() + ".map",
                              this->GetEnvironment());
    throw RunTimeException(WHERE, "Error: Root cfg not valid! "
                                  "(Roadmap with root was written)");
  }

  //Note: This can be useful in centering models (getting the proper root cfg
  //      formatted in an automated way) but I don't want it to always happen.
  // write translation between origin and body centroid
//  ofstream queryFile;
//  queryFile.open(this->GetBaseFilename() + ".query");
//  const mathtool::Vector3d zero(0,0,0);
//  MultiBody* const multiBody = m_robot->GetMultiBody();
//  for (size_t i = 0; i < m_numParts; ++i) {
//    mathtool::Vector3d posA = multiBody->GetBody(i)->
//        GetWorldPolyhedron().GetCentroid();
//    queryFile << posA << zero;
//  }
//  queryFile.close();
}

template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
Finalize() {
  //Always print out the overall success:
  const string statusString = this->m_successful ? "" : "not ";
  std::cout << std::endl << std::endl << "Disassembling " << statusString
            << " completed!!!" << std::endl << std::endl << std::endl;

  // Output final map.
  this->GetGroupRoadmap()->Write(this->GetBaseFilename() + ".map",
                                 this->GetEnvironment());
  //Update the roadmap and goals:
  GroupRoadmapType* const map = this->GetGroupRoadmap();
  const size_t numVertices = map->get_num_vertices();

  // Output stats.
  ofstream osStat(this->GetBaseFilename() + ".stat");
  StatClass* stats = this->GetStatClass();

  if(this->m_debug) {
    std::cout << "Checking state validity of all nodes in graph." << std::endl;
    VerifyNodes();
    std::cout << "All nodes were valid!" << std::endl;
  }

  //Add on the disassembly sequence stats:
  GetSequenceStats();//adds some stats to the stat class, gets all path stats.
  stats->PrintAllStats(osStat, map);

  std::vector<size_t> pathIndices; // indices of desired paths from m_pathVids

  const bool generatePath = true;

  if(m_printFullPath) {
    if(this->m_aStarSearch || !this->m_graphMethod) {
      //Both A* and Preemptive needs to setup the query here:
      pathIndices.clear();
      pathIndices.push_back(0);//use the 0th (only) path

      if(generatePath) {
        m_pathVids.clear();//clear both paths

        //Create the pair of weights, then the path will be between the root and
        // the last added VID.
        m_pathVids.push_back(make_pair(0, VIDPath()));
        m_pathVids[0].second.push_back(m_rootVid);
        m_pathVids[0].second.push_back(numVertices-1);
        //query will grab all needed vids along path (and create intermediates).
      }
    }
    else {
      //We want to use min/median/max paths. Note that stats and paths need to
      // be sorted based on number of nodes in path (done in GetSequenceStats()).
      pathIndices.push_back(0);
      pathIndices.push_back(m_pathVids.size() / 2);
      pathIndices.push_back(m_pathVids.size() - 1);
    }

    if(this->m_debug)
      std::cout << "Graph size: " << numVertices << std::endl;

    // For each path to print, generate the intermediates and then write it.
    for(size_t ind : pathIndices) {
      GroupPath path(map);
      if(generatePath)
        path = GenerateFullDisassemblyPath(m_pathVids.at(ind).second);
      else
        path += m_pathVids.at(ind).second; // m_pathVids has all needed vids.

      if(path.VIDs().size() > 1) {
        if(this->m_debug)
          std::cout << "Writing path with index " << ind << ", with vids: "
                    << path.VIDs() << std::endl;
        //Generate the intermediates, and output the cfgs to a path file:
        const string filename = this->GetBaseFilename() + "." +
                                std::to_string(ind) + ".path";
        const std::vector<GroupCfgType> fullCfgs =
                                        path.FullCfgs(this->GetMPLibrary(), "");
        WritePath(filename, fullCfgs);
      }
      else
        std::cout << std::endl << "GroupPath " << ind << " failed to write!"
                  << std::endl << std::endl;
    }
  }

  GenerateDotFile();
}

template <typename MPTraits>
typename DisassemblyMethod<MPTraits>::GroupPath
DisassemblyMethod<MPTraits>::
GenerateFullDisassemblyPath(const VIDPath& _path) {
  GroupRoadmapType* const map = this->GetGroupRoadmap();

  GroupQuery<MPTraits> tempQuery;
  tempQuery.SetMPLibrary(this->GetMPLibrary());
  GroupPath fullPath(map); // Note this is a group path.

  GroupPath* storedPath = this->GetGroupPath();

  //Add the root vid since I remove the first vid from each sub-path in the loop
  fullPath += VIDPath({m_rootVid});

  if(_path.size() < 2 || _path[1] == m_rootVid) {
    std::cout << "Warning! _path had too few VIDs, or the root was second!"
              << std::endl;
    fullPath.Clear();
    return fullPath;
  }
  if(this->m_debug)
    std::cout << "Generating each sub path between vids " << _path << std::endl;

  //Perform the query so that the path is populated:
  for(size_t i = 0; i < _path.size() - 1; ++i) {
    storedPath->Clear(); // Reset the sub-path

    //NOTE: this is very important! Since there are at least two edges for
    // EVERY removal, we must get the path from
    // [initial -> removed -> set aside cfg] (which the roadmap is set up for).
    //Accumulate the VID path:
//    throw RunTimeException(WHERE, "Ensure this is fixed! GroupQuery and "
//                   "QueryMethod need updating for all functions used in here!");

//    *this->GetPath() += tempQuery.GeneratePath(_path[j], _path[j+1]);
    *storedPath += tempQuery.GeneratePath(_path[i], _path[i+1]);

//    if(storedPath->VIDs().empty()) {
////      throw RunTimeException(WHERE, "Could not find path between vids " +
////          std::to_string(_path[j]) + " and " + std::to_string(_path[j+1]));
//      std::cerr << "Warning: Part of disassembly path not found. Output will be"
//                   " invalid!" << std::endl << std::endl;
//      continue;
//    }
//    else
    if(this->m_debug)
      std::cout << "Generating sub path between vids " << _path[i] << " and "
                << _path[i+1] << std::endl;

    //Extract the sub path, omitting the first entry (it would be a duplicate)
    const VIDPath& pathVids = storedPath->VIDs();
    //Note that subPath must still have at least 2 vids: one for the removed
    // position, and one for the set aside position.
    if(pathVids.size() > 1) {
      VIDPath subPath(pathVids.begin()+1, pathVids.end());
      fullPath += subPath;
    }
    else {
      std::cout << "Error! Sub-path not found between vids " << _path[i]
                << " and " << _path[i+1] << std::endl;
      fullPath.Clear();
      return fullPath;
    }
  }

  return fullPath;
}

template <typename MPTraits>
std::vector<typename DisassemblyMethod<MPTraits>::GroupCfgType>
DisassemblyMethod<MPTraits>::
GetMatingSamples(const Formation& _subassembly) {
  std::vector<GroupCfgType> samples;

  auto const sampler = m_matingSamplerLabels.begin();
  auto const s = dynamic_pointer_cast<MaskedSamplerMethodGroup<MPTraits> >(
            this->GetSampler(sampler->first));

  s->SetActiveRobots(_subassembly);

  s->Sample(sampler->second.first, sampler->second.second,
            this->GetEnvironment()->GetBoundary(), back_inserter(samples));
  return samples;
}


//template <typename MPTraits>
//typename DisassemblyMethod<MPTraits>::GroupCfg
//DisassemblyMethod<MPTraits>::
//FindClosestRemovalCfg(const GroupCfg& _startCfg, const GroupCfg& _endCfg,
//                      GroupLPOutput<MPTraits>& _lpOutput) {
//  if(this->m_debug)
//    std::cout << "Finding nearest cfg for successful removal (post "
//              << "processing step)" << std::endl;
//  //Finds the closest cfg to m_minMatingDist for the part being moved. Note this
//  // does not update any body settings, so it's to be used in the context of
//  // having the VC's body numbers already set and the given cfgs should match.
//  auto lp = this->GetLocalPlanner(m_lpLabel);
//  Environment* env = this->GetEnvironment();
//  auto vc = dynamic_pointer_cast<SpecificBodyCollisionValidity<MPTraits>>(
//                  this->GetValidityChecker(m_vcLabel));
//  // go the trajectory backwards to get the minimum distance
//  // get path from local planner.
//  GroupCfg col;
//  lp->IsConnected(_startCfg, _endCfg, col, &_lpOutput,
//    env->GetPositionRes(), env->GetOrientationRes(), true, true);
//  // get last valid cfg to the defined mating dist
//  string callee("DisassemblyMethod::ExpandMatingApproach");
//  CDInfo cdInfo(true);
//  GroupCfg lastCfg = _lpOutput.m_path.back();
//
//  typedef typename std::vector<GroupCfg>::reverse_iterator rIterator;
//  for (rIterator i = _lpOutput.m_path.rbegin();
//      i != _lpOutput.m_path.rend(); ++i ) {
//    vc->IsValid(*i, cdInfo, callee);
//    if (cdInfo.m_minDist < m_minMatingDist)
//      break;
//    else
//      lastCfg = *i;
//  }
//  // recompute the line to get GroupLPOutput
//  lp->IsConnected(_startCfg, lastCfg, col, &_lpOutput,
//    env->GetPositionRes(), env->GetOrientationRes(), true);
//  return lastCfg;
//}


template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
PrintDisassemblyTree() {
  /// This function loops through the member m_disNodes and prints key
  /// components of each node.

  std::cout << std::endl << std::endl << "Printing all " << m_disNodes.size()
            << " disassembly nodes:" << std::endl;
  size_t nodeNum = 0;
  for(const DisassemblyNode& node : m_disNodes) {
    std::cout << "Node " << nodeNum << ": " << std::endl
              << "{ VID: " << node.vid << "; Depth: " << node.depth << "; "
              << "Number of parents: " << node.parents.size() << "; "
              << "Number of children: " << node.children.size() << std::endl
              << "Parts in initial position: " << node.initialParts << "; "
              << std::endl
              << "Set aside subassemblies: " << node.usedSubassemblies << "; }"
              << std::endl;
    ++nodeNum;
  }
  std::cout << std::endl << std::endl;
}


template <typename MPTraits>
bool
DisassemblyMethod<MPTraits>::
EnsureMinimumClearance(GroupCfgType& _cfg, const Formation& _formation) {
  /// TODO Remove hardcoding of this.
  auto const minDistME = std::dynamic_pointer_cast<MinimumClearanceEvaluator<
                         MPTraits> >(this->GetMapEvaluator("MinClearanceEval"));
  auto const vc = this->GetValidityChecker(this->m_vcLabel);
  if(!minDistME)
    throw RunTimeException(WHERE, "Couldn't find and/or cast MinClearanceEval");
  CDInfo cdInfo(true); // Need full clearance info.
  if(!vc->IsValid(_cfg, cdInfo, this->GetNameAndLabel(), _formation))
    return false;

  // Note that minDist == clearance.
  if(cdInfo.m_minDist < minDistME->GetMinDist())
    return false;

  return true;
}

template <typename MPTraits>
typename DisassemblyMethod<MPTraits>::VIDPath
DisassemblyMethod<MPTraits>::
ExpandMatingApproach(const VID _q, const Formation& _formation, VID& _newVid) {
  this->GetStatClass()->StartClock("ExpandMatingApproachClock");
  auto const graph = this->GetGroupRoadmap();
  auto const env = this->GetEnvironment();
  auto const lp = this->GetLocalPlanner(this->m_lpLabel);
  auto const vc = this->GetValidityChecker(m_vcLabel);

  if(this->m_debug)
    std::cout << "Mating approach expanding from VID " << _q
              << " with subassembly " << _formation << std::endl;

  auto startCfg = graph->GetVertex(_q);

  const string samplerLabel = m_matingSamplerLabels.begin()->first;
  auto const s = dynamic_pointer_cast<MaskedSamplerMethodGroup<MPTraits> >(
                                              this->GetSampler(samplerLabel));
  s->SetStartCfg(startCfg);//Masked entries are set to the values in startCfg
  s->SetActiveRobots(_formation);

  const std::vector<GroupCfgType> samples = GetMatingSamples(_formation);

  bool pathFound = false;
  GroupCfgType newCfg(this->GetGroupRoadmap()), col(this->GetGroupRoadmap());
  GroupLPOutput<MPTraits> lpOutput(this->GetGroupRoadmap());
  size_t sampleNum = 0;
  const bool savePath = true;
  const bool checkCollision = true; // Just for clarity.

  if(this->m_debug)
    std::cout << "Attempting mating over " << samples.size() <<
                 " mating samples" << std::endl;

  const double posRes = m_matingPosResMultiplier * env->GetPositionRes();

  for (const GroupCfgType& sample : samples) {
    // Check if the local plan is successful:
    if(lp->IsConnected(startCfg, sample, col, &lpOutput, posRes,
                       env->GetOrientationRes(), checkCollision, savePath,
                       _formation)) {
//      if (m_minMatingDist > 0) // only used if mating distance is set (slower!)
//        newCfg = FindClosestRemovalCfg(startCfg, sample, lpOutput);
//      else
        newCfg = sample;

      if(!EnsureMinimumClearance(newCfg, _formation)) {
        if(this->m_debug)
          std::cout << "Warning: full mating extension completed, but the "
                       "clearance check failed!" << std::endl;
        continue; // Must make sure we hit a sufficient clearance for success.
      }

      pathFound = true;
      if(this->m_debug)
        std::cout << "Valid path found on direction " << sampleNum << std::endl;
      break; // break on the first path found
    }
    ++sampleNum;
  }
  if(!pathFound) {
    if(this->m_debug) {
      std::cout << "Mating failed over all directions."
                << std::endl << std::endl;
    }
    this->GetStatClass()->StopClock("ExpandMatingApproachClock");
    return VIDPath();
  }

  _newVid = graph->AddVertex(newCfg);
  lpOutput.SetActiveRobots(_formation);

  graph->AddEdge(_q, _newVid, lpOutput.m_edge);
  m_lastAddedVID = _newVid;
  VIDPath path = {_q, _newVid};

  if(this->m_debug) {
    if(!graph->IsEdge(_q, _newVid))
      std::cerr << "Edge (" << _q << ", " << _newVid
                << ") not added after RRT!" << std::endl << std::endl;
    else
      std::cout << "Removal Complete! Created edge between vid " << _q <<" and "
                << _newVid << std::endl << std::endl;
//    PrintDisassemblyTree(); // Prints a lot of output each time.
    for(size_t i = 0; i < startCfg.GetNumRobots(); ++i)
      if(startCfg.GetRobot(i) == nullptr || newCfg.GetRobot(i) == nullptr)
        throw RunTimeException(WHERE, "One of the endpoint cfgs has a null "
                                      "robot after LP!");
  }

  this->GetStatClass()->StopClock("ExpandMatingApproachClock");
  return path;
}

template <typename MPTraits>
typename DisassemblyMethod<MPTraits>::VIDPath
DisassemblyMethod<MPTraits>::
ExpandRRTApproach(const VID _q, const Formation& _formation, VID& _newVID) {
  this->GetStatClass()->StartClock("ExpandRRTApproachClock");
  auto const graph = this->GetGroupRoadmap();
  const string callee = "DiassemblyMethod::ExpandRRTApproach";

  auto sampler = dynamic_pointer_cast<MaskedSamplerMethodGroup<MPTraits> >(
                 this->GetSampler(m_rrtSamplerLabels.begin()->first));
  auto dm = this->GetDistanceMetric(m_rrtDMLabel);

  if(!sampler)
    throw RunTimeException(WHERE, "Error! dynamic pointer cast failed for "
                                  "sampler!");

  auto rrtStrategy = dynamic_pointer_cast<DisassemblyRRTStrategy<MPTraits> >(
                       this->GetMPStrategy(m_rrtStrategyLabel));

  //1. Make a whole new MPSolution and save the one currently there.
  MPSolution* const originalSolution = this->GetMPLibrary()->GetMPSolution();
  MPSolution* const tempSolution = new MPSolution(originalSolution->GetRobotGroup());

  //2. Swap the pointer in MPLibrary for the new MPSolution
  this->GetMPLibrary()->SetMPSolution(tempSolution);

  // Group cfgs take into account the roadmap used, so we must make a new group
  // cfg that uses the RRT's roadmap.
  const GroupCfgType& startCfgDisassemblyRoadmap = graph->GetVertex(_q);

  // NOTE: in order for this to be correct, the MPSolution change should have
  //       already happened!
  const GroupCfgType startCfgRRTRoadmap = startCfgDisassemblyRoadmap.
                                  ChangeRoadmap(rrtStrategy->GetGroupRoadmap());

  dm->SetActiveRobots(_formation);
  sampler->SetActiveRobots(_formation);
  sampler->SetStartCfg(startCfgRRTRoadmap);

  if(this->m_debug)
    std::cout << callee << ": Start VID in roadmap = " << _q << std::endl
              << std::endl << "Main group roadmap pointer = "
              << originalSolution->GetGroupRoadmap() << std::endl
              << "State: " << originalSolution->GetGroupRoadmap()->PrettyPrint()
              << std::endl << std::endl;

  // Set the starting cfg in the strategy:
  rrtStrategy->m_startGroupCfg = &startCfgRRTRoadmap;
  rrtStrategy->SetActiveRobots(_formation);// RRT needs this for its extender.

  //Make sure if we are/are not expecting a path on failure, that RRT matches:
  rrtStrategy->SetKeepBestPathOnFailure(m_keepBestRRTPathOnFailure);

  //3. Call the operator() on the MPStrategy, which will solve it and place
  //    all needed data in the new MPSolution.
  (*rrtStrategy)();
  const bool successfulRemoval = rrtStrategy->IsSuccessful();

  //Make a copy of the clocks from the RRT:
  const std::map<std::string, ClockClass> rrtClocks =
                                        rrtStrategy->GetStatClass()->m_clockMap;

  if(this->m_debug)
    std::cout << "After RRT: successfulRemoval = " << successfulRemoval
              << std::endl;

  bool pathFound = false;
  VIDPath disassemblyRoadmapPath;
  auto const rrtRoadmapPath = tempSolution->GetGroupPath();
  //Check that a solution was actually found:
  if(rrtRoadmapPath) {
    // We have all needed path data, so put the original solution back:
    this->GetMPLibrary()->SetMPSolution(originalSolution);
    if(this->m_debug)
      AccumulateClockTimes(rrtClocks);

    if(rrtRoadmapPath->VIDs().size() > 1) {
      //Append the solution path to the roadmap we have, starting from the
      // initial cfg that was passed here.
      //Make sure to put the original solution back into the MPLibrary
      // before calling this, as it uses the library to get the correct roadmap.
      disassemblyRoadmapPath = AddDissassemblyPathToRoadmap(
                                  rrtRoadmapPath->VIDs(),
                                  tempSolution->GetGroupRoadmap(), _q, _newVID);
      pathFound = disassemblyRoadmapPath.size() > 1;
      if(this->m_debug)
        std::cout << "RRT path length in disassembly roadmap = "
                  << disassemblyRoadmapPath.size() << std::endl;
    }
  }

  // After finishing, reset everything and return the path:
  if(!pathFound) { // Means we didn't set the solution already (see above)
    this->GetMPLibrary()->SetMPSolution(originalSolution);
    if(this->m_debug)
      AccumulateClockTimes(rrtClocks);
  }
  rrtStrategy->m_startGroupCfg = nullptr;
  delete tempSolution;

  if(m_keepBestRRTPathOnFailure) {
    if(!successfulRemoval) {
      if(pathFound) {
        if(this->m_debug)
          std::cout << "Failed to remove part " << _formation <<
                    " but a path ending at VID " << _newVID <<
                    " of highest clearance was kept." << std::endl;
      }
      else {
        if(this->m_debug)
          std::cout << "RRT failed and didn't find any path" << std::endl;
        if(disassemblyRoadmapPath.size() > 1)//Error, means LP is off from Extender.
          throw RunTimeException(WHERE, "RRT produced a path that the LP"
                                        " couldn't reproduce!");
        _newVID = 0;
      }
      this->GetStatClass()->StopClock("ExpandRRTApproachClock");
      return VIDPath();
    }
    else if(this->m_debug)
      std::cout << "ML-RRT removed part " << _formation << std::endl;
  }

  m_lastAddedVID = _newVID;

  this->GetStatClass()->StopClock("ExpandRRTApproachClock");

  return disassemblyRoadmapPath;
}

template <typename MPTraits>
typename DisassemblyMethod<MPTraits>::VIDPath
DisassemblyMethod<MPTraits>::
AddDissassemblyPathToRoadmap(const VIDPath& _pathVids,
                             GroupRoadmapType* const _rrtGraph,
                             VID _roadmapStartVID,
                             VID& _lastAddedVID) {
  GroupRoadmapType* const graph = this->GetGroupRoadmap();

  if(this->m_debug)
    std::cout << "Adding disassembly path of length " << _pathVids.size()
              << " starting from VID " << _roadmapStartVID << std::endl;

  VIDPath rrtPath;
  rrtPath.push_back(_roadmapStartVID);

  //Set up the first VID/Cfg for the loop:
  VID lastAddedCfgVID = _roadmapStartVID;
  VID newAddedCfgVID;

  for(auto it = _pathVids.begin(); it + 1 < _pathVids.end(); ++it) {
    // Get the next edge.
    bool validEdge = false;
    typename GroupRoadmapType::adj_edge_iterator ei;
    {
      typename GroupRoadmapType::edge_descriptor ed(*it, *(it+1));
      typename GroupRoadmapType::vertex_iterator vi;
      validEdge = _rrtGraph->find_edge(ed, vi, ei);
    }

    if(!validEdge)
      throw RunTimeException(WHERE, "Edge doesn't exist in roadmap!");

    if(ei->property().GetActiveRobots().empty())
      throw RunTimeException(WHERE, "RRT didn't set the edge's active bodies!");

    // Get cfg from rrt roadmap, and add it to the disassembly roadmap:
    GroupCfgType newCfg = _rrtGraph->GetVertex(*(it + 1));

    // Group cfgs take into account the roadmap used, so we must change out the
    // roadmap for this one and force all cfg info to be local to the cfg.
    newCfg = newCfg.ChangeRoadmap(graph);

    newAddedCfgVID = graph->AddVertex(newCfg);
    rrtPath.push_back(newAddedCfgVID);

    // Handle the individual edges (for individual roadmaps):
    GroupLPOutput<MPTraits> lpOutput(graph, ei->property());
    lpOutput.SetIndividualEdges(lpOutput.GetActiveRobots());

    //The active bodies will already be built into the edge we extracted.
    graph->AddEdge(lastAddedCfgVID, newAddedCfgVID, lpOutput.m_edge);

    if(this->m_debug) {
      if(!graph->IsEdge(lastAddedCfgVID, newAddedCfgVID))
        std::cerr << "Edge (" << lastAddedCfgVID << ", " << newAddedCfgVID
                  << ") not added after RRT!" << std::endl << std::endl;
      else
        std::cout << "RRT Created edge between vids " << lastAddedCfgVID
                  << " and " << newAddedCfgVID << std::endl;
    }
    //Update the VID:
    lastAddedCfgVID = newAddedCfgVID;
  }

  _lastAddedVID = lastAddedCfgVID;
  return rrtPath;
}

template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
AccumulateClockTimes(
    const std::map<std::string, ClockClass>& _rrtClocks) {
  StatClass* const currentStats = this->GetStatClass();
  const static string rrtString = "DisassemblyRRT::";

  for(const std::pair<std::string, ClockClass>& clockPair : _rrtClocks)
    currentStats->IncStat(
        rrtString + clockPair.first, clockPair.second.GetSeconds());
}


template <typename MPTraits>
typename DisassemblyMethod<MPTraits>::DisassemblyNode*
DisassemblyMethod<MPTraits>::
GenerateNode(DisassemblyNode* const _parent,
             const std::vector<size_t>& _removedParts,
             const std::vector<VIDPath>& _removingPaths,
             const bool _isMultiPartSubassembly,
             const double _localWeight) {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::GenerateNode()" << std::endl;

  this->GetStatClass()->StartClock("GenerateNodeClock");
  GroupRoadmapType* const graph = this->GetGroupRoadmap();

  DisassemblyNode node;
  node.parents.push_back(_parent);
  node.depth = _parent->depth + 1;
  node.removedParts = _removedParts;
  node.removalPaths = _removingPaths;

  if(!node.parents[0])
    throw RunTimeException(WHERE, "Null parent when creating new node!");

  // Update weights in the disassembly graph.
  const double parentPathWeight = node.parents[0]->bestCumulativeWeight;
  node.bestCumulativeWeight = parentPathWeight + _localWeight;
  node.localWeight = _localWeight;

  // Copy the initial parts of the parent, removing the part(s) removed.
  node.initialParts = _parent->initialParts;
  for (const size_t part : _removedParts)
    node.initialParts.erase(remove(node.initialParts.begin(),
                       node.initialParts.end(), part), node.initialParts.end());

  // Copy then remove the part(s) from any subassembly they were in for _parent:
  node.usedSubassemblies = _parent->usedSubassemblies;
  for (Formation& sub : node.usedSubassemblies)
    for (const size_t part : _removedParts)
      sub.erase(remove(sub.begin(), sub.end(), part), sub.end());

  // Remove any empty entries in usedSubassemblies.
  node.usedSubassemblies.erase(
    remove_if(node.usedSubassemblies.begin(), node.usedSubassemblies.end(),
        [](Formation const& s) { return s.empty(); }),
        node.usedSubassemblies.end());

  // Start with parent cfg to keep formation (removed part dofs get overwritten)
  GroupCfgType expansionCfg = graph->GetVertex(_parent->vid);

  // For all removed parts, we want to offset wrt their root positions:
  expansionCfg.OverwriteDofsForRobots(graph->GetVertex(m_rootVid),
                                      _removedParts);

  //Note: I cannot use the DOF() function here as it's not "per body"
  const size_t posDofsPerBody = expansionCfg.PosDOF();
  const size_t dofsPerBody = posDofsPerBody + expansionCfg.OriDOF();

  //dofOffsets represents the 'offset' from the root cfg that fully removed
  // parts are placed at. This means that the root's cfg is kept in-tact
  // it simply is translated, preserving each part's positions/rotations
  // relative to the other parts'.
  std::vector<double> dofOffsets(dofsPerBody, 0.0); // So default val is 0.
  if (_isMultiPartSubassembly) {
    node.usedSubassemblies.push_back(_removedParts); // Record this in the node.
    const double subassemblyOffset =
        m_subassemblyPos + (m_subassemblyOffset * node.depth);
    for(size_t i = 0; i < posDofsPerBody; ++i)
      dofOffsets[i] = subassemblyOffset;
  }
  else {
    // Either a single part or multiple parts were removed not as a subassembly,
    // offset the root cfg to place things to the side in formation:
    for(size_t i = 0; i < posDofsPerBody; ++i)
      dofOffsets[i] = m_removePos;
  }
  // Add the dofOffsets into expansionCfg for _removedParts.
  expansionCfg.AddDofsForRobots(dofOffsets, _removedParts);

  if(this->m_debug)
    std::cout << "Added dofs " << dofOffsets << " into root cfg for part(s) "
              << _removedParts << ". Final cfg = " << std::endl
              << expansionCfg.PrettyPrint() << std::endl;

  node.vid = graph->AddVertex(expansionCfg);

  if(m_printFullPath) {
    //If needing a path file, we must create edges here.
    if(_removingPaths.size() > 1) {
      //This is to handle the PreemptiveDFS case where many parts were removed,
      // but all individually/"in-parallel".
      if(this->m_graphMethod)
        throw RunTimeException(WHERE, "Removing paths should only have one path"
                                      " for exhaustive BFS!");
      GroupCfgType lastStateOffset(this->GetGroupRoadmap(), true);

      for(size_t i = 0; i < _removingPaths.size(); ++i) {
        //This could be optimized, but that's not currently worth the time.
        const std::vector<GroupCfgType>& cfgPair = {
                                       graph->GetVertex(_removingPaths[i][0]),
                                       graph->GetVertex(_removingPaths[i][1]) };

        //Ensure that we have a mating path with just two nodes:
        if(cfgPair.size() != 2)
          throw RunTimeException(WHERE, "There was a simultaneous removal with"
                " a path of size " + std::to_string('0' + cfgPair.size()));

        //We need to accumulate the removals, then fix them offset from the
        // assembly, until we remove all the ones for the node, then put off to
        // the side. This is purely for path generation.
        const GroupCfgType initialCfg = cfgPair[0] + lastStateOffset;
        const GroupCfgType removedCfg = cfgPair[1] + lastStateOffset;
        const VID initialVid = graph->AddVertex(initialCfg);
        const VID removalVid = graph->AddVertex(removedCfg);

        //Accumulate the next part's offset:
        lastStateOffset += (cfgPair[1] - cfgPair[0]);
        const std::vector<GroupCfgType> fakePath({initialCfg, removedCfg});

        GroupLPOutput<MPTraits> tempLPOutput(graph, fakePath);
        const Formation singlePart{_removedParts[i]};
        tempLPOutput.SetActiveRobots(singlePart);// Set the active body for LP.
        tempLPOutput.SetIndividualEdges(singlePart); // Create individual edges.
        tempLPOutput.SetEdgeWeights(1); // Give a nonzero weight.
        graph->AddEdge(initialVid, removalVid, tempLPOutput.m_edge);
        if(this->m_debug) {
          if(!graph->IsEdge(initialVid, removalVid))
            std::cerr << "Edge (" << initialVid << ", " << removalVid
                      << ") not added after RRT!" << std::endl << std::endl;
          else
            std::cout << "Created edge between vids " << initialVid << " and "
                      << removalVid << std::endl;
        }
        m_lastAddedVID = removalVid;
      }
    }

    // Create the edge where it will "teleport" to the disassembled position:
    const GroupCfgType& lastAddedCfg = graph->GetVertex(m_lastAddedVID);
    const std::vector<GroupCfgType> fakePath({lastAddedCfg, expansionCfg});

    // Dummy edge (group local plan)
    GroupLPOutput<MPTraits> skipLPOutput(graph, fakePath);
    skipLPOutput.SetIndividualEdges(_removedParts);
    skipLPOutput.SetSkipEdge(); // So FullCfgs() doesn't create intermediates.
    //NOTE: this won't be used (and is not necessarily valid!):
    skipLPOutput.SetActiveRobots(_removedParts);
    skipLPOutput.SetEdgeWeights(1); // give a nonzero weight
    graph->AddEdge(m_lastAddedVID, node.vid, skipLPOutput.m_edge);

    if(this->m_debug) {
      if(!graph->IsEdge(m_lastAddedVID, node.vid))
        std::cerr << "Edge (" << m_lastAddedVID << ", " << node.vid
                  << ") not added after RRT!" << std::endl << std::endl;
      else
        std::cout << "Created edge between vids " << m_lastAddedVID << " and "
                  << node.vid << std::endl;
    }
  }

  //Finally, we need to update the last added vid, so the next iteration starts
  // with all the removed parts in the offset position.
  m_lastAddedVID = node.vid;

  m_disNodes.push_back(node);
  //Update the parent to have this new node as a child.
  _parent->children.push_back(make_pair(_localWeight, &m_disNodes.back()));

  this->GetStatClass()->StopClock("GenerateNodeClock");

  return &m_disNodes.back();
}

template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
AnalysePartContacts(const VID _q, const Formation& _partIndices) {
  std::vector<size_t> fullIndexList;
  auto dirs = GetMatingSamples(_partIndices);
  for (size_t i = 0; i < dirs.size(); ++i)
    fullIndexList.push_back(i);
  m_nonObstacleDirs.clear();
  m_nonObstacleDirs.resize(m_numParts, fullIndexList);

  // initialize body contacts
  m_partContacts.clear();
  m_partContacts.resize(m_numParts, std::vector<std::vector<size_t>>());
  for (size_t i = 0; i < m_numParts; ++i)
    m_partContacts[i].resize(fullIndexList.size(), std::vector<size_t>());

  // loop through the remaining parts
  for (auto &partIndex : _partIndices)
    AnalysePartContacts(_q, partIndex);
}

template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
AnalysePartContacts(const VID _q, const size_t _partIndex) {
  auto graph = this->GetGroupRoadmap();
  auto env = this->GetEnvironment();
  auto lp = this->GetLocalPlanner(this->m_lpLabel);
  auto const vc = this->GetValidityChecker(m_vcLabel);
  GroupLPOutput<MPTraits> lpOutput(this->GetGroupRoadmap());

  GroupCfgType newCfg, col, startCfg = graph->GetVertex(_q);
  // loop through the remaining parts
  Formation activeRobots = {_partIndex};
  auto samples = GetMatingSamples(activeRobots);

  // loop through all direction of the mating approach
  auto& nonObstacleDirs = m_nonObstacleDirs[_partIndex];
  for (size_t dirIndex = 0; dirIndex < samples.size(); ++dirIndex) {
    newCfg = samples[dirIndex] + startCfg;
    col = startCfg;

    CDInfo cdInfo(true);
    cdInfo.m_selfClearance.resize(m_numParts, numeric_limits<double>::max());
    if (lp->IsConnected(startCfg, newCfg, col, &lpOutput,
                        env->GetPositionRes(), env->GetOrientationRes(), true,
                        false, activeRobots))
      return;
    else
      vc->IsValid(col, cdInfo, "AnalyzePartContacts", activeRobots);

    // append the collision Parts to the list
    for (size_t bodyInd = 0; bodyInd < m_numParts; ++bodyInd)
      if (cdInfo.m_selfClearance[bodyInd] < m_contactDist)
        m_partContacts[_partIndex][dirIndex].push_back(bodyInd);

    // erase the direction, if no collision with an another body
    if (m_partContacts[_partIndex][dirIndex].empty()) {
      nonObstacleDirs.erase(remove(nonObstacleDirs.begin(),
          nonObstacleDirs.end(), dirIndex), nonObstacleDirs.end());
    }
  }
}


template <typename MPTraits>
std::vector<typename DisassemblyMethod<MPTraits>::Formation>
DisassemblyMethod<MPTraits>::
GenerateSubassemblies(const VID _currentVid, const Formation& _parts) {
  if(this->m_debug)
      std::cout << this->GetNameAndLabel() << "::GenerateSubassemblies()"
                << std::endl;

  //Generate subassemblies on a node-to-node basis, versus using a master list:
  if(!m_generateSubassembliesOnce && m_predefinedSubassemblies.empty())
    return GenerateDirectionalCollisionSubassemblies(_currentVid, _parts);

  // Generate master list once and cache it. Then match subassemblies to
  // candidates based on the remaining _parts.
  if(m_predefinedSubassemblies.empty())
    m_predefinedSubassemblies = GenerateDirectionalCollisionSubassemblies(
                                     m_rootNode->vid, m_rootNode->initialParts);

  this->GetStatClass()->SetStat("Subassemblies in m_predefinedSubassemblies",
                                m_predefinedSubassemblies.size());

  // Match m_predefinedSubassemblies to _parts and only return subs that are
  // subsets of the set _parts.
  return GeneratePredefinedSubassemblies(_parts);
}

template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
PrintDirectionalCollisionInfo(const Formation& _parts) {
  for (auto &partInd : _parts) {
    std::cout << " Body " << partInd << " : ";
    for (auto &dir : m_nonObstacleDirs[partInd]) {
      std::cout << "dir " << dir << ": ";
      for (auto &id : m_partContacts[partInd][dir]) {
        std::cout << id << ", ";
      }
      std::cout << "  |  ";
    }
    std::cout << std::endl;
  }
}


template <typename MPTraits>
std::vector<typename DisassemblyMethod<MPTraits>::Formation>
DisassemblyMethod<MPTraits>::
GenerateDirectionalCollisionSubassemblies(const VID _currentVid,
                                          const Formation& _parts) {
  std::vector<Formation> subassemblies;
  if(_parts.size() < 2)
    return subassemblies;

  this->GetStatClass()->StartClock("GenerateDirectionalCollisionSubassembliesClock");

  if(this->m_debug)
    std::cout << this->GetNameAndLabel()
         << "::GenerateDirectionalCollisionSubassemblies()" << std::endl;

  //Set up all of the member variables I'll be using:
  AnalysePartContacts(_currentVid, _parts);

  if(this->m_debug) {
//    PrintDirectionalCollisionInfo(_parts);
    std::cout << "Generating subassemblies for parts " << _parts << std::endl;
  }

  for(size_t partInd : _parts) {
    // For each non-obstacle direction in the body:
    for(size_t dirInd : m_nonObstacleDirs[partInd]) {
      Formation subassembly; // Initially empty std::vector for recursion.
      GetCollidingPartsRecursive(partInd, dirInd, subassembly);

      sort(subassembly.begin(), subassembly.end()); // Sort for comparisons

      if(subassembly.size() < 2
          || IsDuplicateSubassembly(subassembly, subassemblies))
        continue;//try the next direction

      subassemblies.push_back(subassembly);
    } // End for all valid directions
  } // End for all Parts in the entire remaining assembly

  // Post-processing: remove bodies that have already been disassembled.
  for (Formation& subassembly : subassemblies) {
    auto subBody = begin(subassembly);
    while (subBody != end(subassembly)) {
      bool contained = false;
      for (auto id : _parts) {
        if (id == *subBody) {
          contained = true;
          break;
        }
      }
      if (!contained)
        subBody = subassembly.erase(subBody);
      else
        ++subBody;
    }
  }
  // second test for subassembly size, because of the removing of bodies
  subassemblies.erase(remove_if(subassemblies.begin(), subassemblies.end(),
                      [](Formation const& s) { return s.size() < 2; }),
                      subassemblies.end());

  // Sort subassemblies by size
  struct SortSize {
    bool operator() (const Formation& i, const Formation& j)
                      { return (i.size() < j.size());}
  } sortSize;
  sort(subassemblies.begin(), subassemblies.end(), sortSize);

  if (this->m_debug) {
    std::cout << "Generated subassemblies: ";
    for (auto tmp : subassemblies)
      std::cout << tmp << ";  ";
    std::cout << std::endl;
  }

  this->GetStatClass()->StopClock("GenerateDirectionalCollisionSubassembliesClock");

  return subassemblies;
}

template <typename MPTraits>
std::vector<typename DisassemblyMethod<MPTraits>::Formation>
DisassemblyMethod<MPTraits>::
GeneratePredefinedSubassemblies(const Formation& _parts) {
  //Given a list _parts, find the subassemblies in m_predefinedSubassemblies
  // that are valid candidates to attempt (all of the parts in a valid
  // subassembly will be inside _parts).
  std::vector<Formation> subs;

  for(Formation& cand : m_predefinedSubassemblies) {
    bool invalid = false;
    for(size_t part : cand)
      if(std::find(_parts.begin(), _parts.end(), part) == _parts.end())
        invalid = true;
    //If we make it through all candidate parts and they are all in the
    // remaining parts, we know we can use this subassembly:
    if(!invalid)
      subs.push_back(cand);
  }
  return subs;
}



template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
GetCollidingPartsRecursive(const size_t _partInd, const size_t _dirInd,
                           Formation& _currentSubassembly) {
  // If a body is in _currentSubassembly, it implies it has already been
  // recursively checked.
  //Check that the indices are good:
  if(_partInd >= m_partContacts.size() || _dirInd >= m_partContacts[0].size())
    throw RunTimeException(WHERE,"Out of bounds indices given");

  //Add this body to the list.
  _currentSubassembly.push_back(_partInd);

  //Get the initial contact bodies for this body (copying not needed).
  const std::vector<size_t>& parts = m_partContacts[_partInd][_dirInd];
  for(size_t part : parts) {
    //Check if this body is valid for the direction we are checking.
    const std::vector<size_t>& bodyValidDirs = m_nonObstacleDirs[part];
    if(find(bodyValidDirs.begin(), bodyValidDirs.end(), _dirInd)
            == bodyValidDirs.end()) {
      //Then the direction is not in the list of valid directions for this body,
      // this subassembly is now impossible for this direction.
      _currentSubassembly.clear();
      return;
    }

    //Check if the currentSubassembly already has this body (if so, it's already
    // been checked recursively and is done/valid). Note the direction is fixed
    // as the one given to the function.
    if(find(_currentSubassembly.begin(), _currentSubassembly.end(), part)
            == _currentSubassembly.end()) {
      GetCollidingPartsRecursive(part, _dirInd, _currentSubassembly);

      //If _currentSubassembly is cleared during a call, we know that this
      // direction for one of the bodies won't work, so we quit trying this
      // direction for this subassembly:
      if(_currentSubassembly.empty())
        return;
    }
  }
}


template <typename MPTraits>
bool
DisassemblyMethod<MPTraits>::
IsDuplicateSubassembly(const Formation& _subassembly,
                       const std::vector<Formation>& _subassemblies) {
  // check if subassembly is a duplicate against all current subassemblies.
  bool duplicate = false;
  for (auto &sub : _subassemblies) {
    if(sub.size() != _subassembly.size())
      continue; //If sizes are different, they can't be duplicates.
    bool isDuplicate = true; // temporary flag to check if current sub is a dupe
    for (size_t i = 0; i < sub.size(); ++i)
      if (sub[i] != _subassembly[i])
        isDuplicate = false;

    if (isDuplicate) {
      duplicate = true;
      break; //Don't need to check any more if one duplicate is found.
    }
  }
  return duplicate;
}


template <typename MPTraits>
bool
DisassemblyMethod<MPTraits>::
SameSubassembly(const Formation& _a, const Formation& _b) const {
  if(_a.size() != _b.size())
    return false;

  for(size_t i = 0; i < _a.size(); i++)
    if(_a[i] != _b[i])
      return false;

  //If we never returned false, then we checked all elements without a mismatch.
  return true;
}


template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
GenerateAdjacencyMats() {
  auto vc = dynamic_pointer_cast<CollisionDetectionValidity<MPTraits>>(
                                           this->GetValidityChecker(m_vcLabel));

  // clear and initialize adjacency mats
  m_contactMat.clear();
  m_neighborMat.clear();
  m_contactMat.resize(m_numParts, std::vector<bool>(m_numParts, false));
  m_neighborMat = m_contactMat;

  CDInfo cdInfo(true); // So we get all info from all CD calls.
  const GroupCfgType rootCfg = this->GetGroupRoadmap()->GetVertex(0);
  for(size_t i = 0; i < m_numParts - 1; ++i) {
    MultiBody* const body1 = rootCfg.GetRobot(i)->GetMultiBody();
    for (size_t j = i + 1; j < m_numParts; ++j) {
      MultiBody* const body2 = rootCfg.GetRobot(j)->GetMultiBody();
      vc->IsMultiBodyCollision(cdInfo, body1, body2, "generateAdjacencyMats");
      bool contact = (cdInfo.m_minDist < m_contactDist) ? true : false;
      bool neighbor = (cdInfo.m_minDist < m_neighborDist) ? true : false;
      m_contactMat[i][j] = contact;
      m_contactMat[j][i] = contact;
      m_neighborMat[i][j] = neighbor;
      m_neighborMat[j][i] = neighbor;
    }
  }
  if (this->m_debug) {
    std::cout << "Contact Matrix: " << std::endl;
    for (const std::vector<bool>& row : m_contactMat) {
      for (const bool contact : row) {
        std::cout << contact << " ";
      }
      std::cout << std::endl;
    }
  }
}

template <typename MPTraits>
std::vector<size_t>
DisassemblyMethod<MPTraits>::
GetContactParts(const size_t _bodyId) {
  std::vector<size_t> parts;
  std::vector<bool> row = m_contactMat[_bodyId];
  for (size_t i = 0; i < row.size(); ++i) {
    if (row[i])
      parts.push_back((size_t)i);
  }

  return parts;
}

template <typename MPTraits>
bool
DisassemblyMethod<MPTraits>::
CheckContact(const size_t _firstPart, const size_t _secondPart) {
  if (m_contactMat[_firstPart][_secondPart])
    return true;
  else
    return false;
}

template <typename MPTraits>
bool
DisassemblyMethod<MPTraits>::
CheckNeighbor(const size_t _firstPart, const size_t _secondPart) {
  if (m_neighborMat[_firstPart][_secondPart])
    return true;
  else
    return false;
}

template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
GetSequenceStats() {
  //This will create a .allpaths file to give stats about all DSP paths found
  // and also will add stats to the stat file
  ofstream osStat(this->GetBaseFilename() + ".allpaths");
  StatClass* stats = this->GetStatClass();

  // print sequence stats
  osStat << std::endl << "Sequences:" << std::endl << std::endl;
  m_seqList.clear();
  VIDPath pathPlaceHolder;

  // computation of all sequence stats
  for (auto &node : m_disNodes)
    if (node.initialParts.empty() && node.usedSubassemblies.empty()) {
      RecursivePathStats(&node, 0.0, pathPlaceHolder);
      if(!pathPlaceHolder.empty())
        throw RunTimeException(WHERE, "After recursion, vid path not empty! "
                                  "Probably did something wrong in recursion!");
    }

  //Sort m_seqList and m_pathVids, as we want that output sorted and we can
  // pull out paths based on stats much easier.
  //Should do nothing when there's one path (like for preemptive DFS).
  sort(m_seqList.begin(), m_seqList.end());
  sort(m_pathVids.begin(), m_pathVids.end());

  //Want this count for both files:
  osStat << "Successful Sequence Count: " << m_seqList.size() << std::endl;
  stats->SetStat("DSPSequenceCount", m_seqList.size());

  if(m_seqList.empty()) {
    osStat << "No sequence was found!!!" << std::endl;
    return;//don't continue, no stats to find.
  }

  // compute min and max and write all seq stats
  size_t count = 0;
  size_t minChanges = numeric_limits<size_t>::max();
  double minDist = numeric_limits<double>::max();
  size_t maxChanges = numeric_limits<size_t>::min();
  double maxDist = numeric_limits<double>::min();
  for (auto &seq : m_seqList) {
    osStat << "  sequence" << count << ": "
           << "changes: " << seq.first << " | "
           << "distance: " << seq.second << std::endl;
    ++count;

    // min and max
    if (seq.first < minChanges)
      minChanges = seq.first;
    if (seq.second < minDist)
      minDist = seq.second;
    if (seq.first > maxChanges)
      maxChanges = seq.first;
    if (seq.second > maxDist)
      maxDist = seq.second;
  }

  osStat << std::endl
         << "Minimum Node Count over all sequences: " << minChanges << std::endl
         << "Minimum Path Length over all sequences: " << minDist << std::endl
         << "Maximum Node Count over all sequences: " << maxChanges << std::endl
         << "Maximum Path Length over all sequences: " << maxDist << std::endl;
  stats->SetStat("MinimumNodeCount", minChanges);
  stats->SetStat("MinimumPathLength", minDist);
  stats->SetStat("MaximumNodeCount", maxChanges);
  stats->SetStat("MaximumPathLength", maxDist);
}


template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
VerifyNodes() {
  if(!m_disNodes.front().parents.empty())
    throw RunTimeException(WHERE, "Root's parents std::vector wasn't empty!");

  if(m_graphMethod) {
    for(auto& node : m_disNodes) {
      if(node.removalPaths.size() != node.parents.size())
        throw RunTimeException(WHERE,"A node's removalPaths don't match with"
                                     " its parents std::vector for a graph method!");
    }
  }
  else {
    for(auto& node : m_disNodes) {
      if(node.removalPaths.size() != node.removedParts.size()) {
        //If the removedParts is equal to a subassembly, we know to expect one
        // path for this subassembly, and so it's still valid if so.
        bool valid = false;
        if(node.removalPaths.size() == 1) {
          for(const auto& subs : node.usedSubassemblies)
            if(subs == node.removedParts)
              valid = true;
        }
        if(!valid)
          throw RunTimeException(WHERE,"A node's removalPaths don't match with "
                                   "its removedParts for a non-graph method!");
      }
    }
  }
}


template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
RecursivePathStats(DisassemblyNode* _node, const double _curDist,
                   VIDPath& _curPath) {
  GroupRoadmapType* const r = this->GetGroupRoadmap();
  _curPath.push_back(_node->vid);
  size_t numPushes = 1; // TODO: the addition done later could be optimized out

  if (_node->parents.empty()) { // Base case: quit at root node
    const size_t numNodes = _curPath.size();
    m_seqList.push_back(make_pair(numNodes, _curDist));

    //Add the path to the list, reverse it so the root is the first vid.
    m_pathVids.push_back(make_pair(numNodes, _curPath));
    std::reverse(m_pathVids.back().second.begin(),
                 m_pathVids.back().second.end());
  }
  else {
    // _node is not the root node, go deeper in recursion.
    auto dm = this->GetDistanceMetric(m_dmLabel);

    // Set the parts that the distance metric should be in regards to.
    dm->SetActiveRobots(_node->removedParts);

    for (size_t parentIndex = 0; parentIndex < _node->parents.size(); ++parentIndex) {
      double dist = _curDist;
      const size_t numRemovedParts = _node->removedParts.size();

      if (numRemovedParts > 1) {
        // If removedParts is greater than 1, then we either had a subassembly
        // or a multiple mating vector removal for preemptive dfs.
        // If subassembly, we are guaranteed only 1 path, so the path loop works
        // If not, then we view all the removals as "one" removal and so the
        // loop must accumulate all of it.
        for (auto &path : _node->removalPaths) {
          // Note: It's intentional to skip the 0-index element of the path, as
          // that would be a duplicate of the next node's vid.
          for (size_t i = path.size() - 1; i > 0; --i) {
            dist += dm->Distance(r->GetVertex(path[i]), r->GetVertex(path[i-1]));
            _curPath.push_back(path[i]);
            ++numPushes;
          }
        }
      }
      else if (numRemovedParts == 1) {
        //In this case, we know that the parents are indexed with the ,
        // removalPaths since in the graph search that's how it's done, and in
        // preemptive there will only be one parent and one removal path due
        // to one removed part.
        auto &path = _node->removalPaths[parentIndex];
        for (size_t i = path.size() - 1; i > 0; --i) {
          dist += dm->Distance(r->GetVertex(path[i]), r->GetVertex(path[i-1]));
          _curPath.push_back(path[i]);
          ++numPushes;
        }
      }
      else
        throw RunTimeException(WHERE, "Invalid removedParts size " +
                                      std::to_string(numRemovedParts));

      //Recurse deeper (or really, shallower towards the root):
      RecursivePathStats(_node->parents[parentIndex], dist, _curPath);
    }
  }

  //When returning, go ahead and remove the added nodes.
  for(size_t i = 0; i < numPushes; ++i)
    _curPath.pop_back();
}

template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
GenerateDotFile() {
  // initialize dot file
  ofstream dotFile;
  dotFile.open (this->GetBaseFilename() + ".dot");
  dotFile << "digraph structs {" << std::endl
         << "node [shape=record];" << std::endl;

  int count = 0;
  // Loop to generate all nodes of the tree. Note we assume that this is a
  // single and connected (i.e. valid) tree.
  for(auto& currNode : m_disNodes) {
    if(this->m_debug)
      std::cout << "Making dot node " << count++ << std::endl;
    GenerateDotNodeStruct(dotFile, &currNode);
  }

  GenerateDotEdges(dotFile);

  dotFile << "}" << std::endl;
  dotFile.close();
}

template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
GenerateDotNodeStruct(ofstream& _dotFile,
                      const DisassemblyNode* const _node) const {
  // If this is the root node, handle it accordingly:
  if (_node == m_rootNode || _node->parents.size() == 0) {
    _dotFile << "struct" << _node << "[shape=record,label=" << '"';
    for (size_t i = 0; i < _node->initialParts.size(); ++i) {
      _dotFile << "<f" << i << "> " << _node->initialParts[i] << "|";
    }
    _dotFile << '"' << "];" << std::endl;
    return;
  }

   _dotFile << "struct" << _node << "[shape=record,label=" << '"';
   _dotFile << "<f1>";
   // remove usedSubassemblies from removedParts and write them
  auto removedParts = _node->removedParts;
  for (auto &usedSub : _node->usedSubassemblies)
    for (auto &id : usedSub)
      removedParts.erase(remove(removedParts.begin(), removedParts.end(), id),
                         removedParts.end());
  for (auto &part : removedParts) {
    _dotFile << part << ",";
  }

  if (!_node->GetCompletePartList().empty())
    _dotFile << "|Re:|<f2>";

  // generate the list with initialParts without usedSubassemblies
  for (auto part : _node->initialParts)
    _dotFile << part << ",";

  // write usedSubassemblies
  for (auto &sub : _node->usedSubassemblies) {
    _dotFile << "(";
    for (auto &id : sub)
      _dotFile << id << ",";
    _dotFile << "),";
  }
  _dotFile << '"' << "];" << std::endl;
}


template <typename MPTraits>
void
DisassemblyMethod<MPTraits>::
GenerateDotEdges(ofstream& _dotFile) const {
  // create the edges of the graph
  for (const DisassemblyNode& node : m_disNodes) {
    if (node.parents.size() == 0)
      continue;

    for(auto parent : node.parents)
      _dotFile << "struct" << parent << " -> struct" << &node << ";"
               << std::endl;
  }
}

#endif
