#ifndef PPL_COMPOSITE_DYNAMIC_REGION_RRT_H_
#define PPL_COMPOSITE_DYNAMIC_REGION_RRT_H_

#include "GroupRRTStrategy.h"

#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "MPLibrary/MPTools/MeanCurvatureSkeleton3D.h"
#include "MPLibrary/MPTools/ReebGraphConstruction.h"
#include "Utilities/MedialAxis2D.h"
#include "Utilities/XMLNode.h"
#include "Utilities/MPUtils.h"
#include "Utilities/SSSP.h"
#include "Utilities/CBS.h"
#include "Workspace/WorkspaceSkeleton.h"
#include "Workspace/CompositeWorkspaceSkeleton.h"
#include "Workspace/PropertyMap.h"

////////////////////////////////////////////////////////////////////////////////
/// Composite Dynamic Region-biased RRT algorithm.
///
/// A composite-space RRT guided by a composite workspace skeleton.
///
/// Reference:
///   Scalable Multi-robot Motion Planning for Congested Environments Using 
///   Topological Guidance, Courtney McBeth, James Motes, Diane Uwacu, Marco 
///   Morales, Nancy M. Amato, ArXiv Preprint, Oct 2022.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////

template <typename MPTraits>
class CompositeDynamicRegionRRT : virtual public GroupRRTStrategy<MPTraits> {
  public:
    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::GroupCfgType     GroupCfgType;
    typedef typename MPTraits::GroupWeightType  GroupWeightType;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename GroupRoadmapType::VID      VID;
    
    typedef typename std::map<Robot*, VID>      VIDMap;
    typedef typename std::vector<VID>           VIDSet;
    typedef typename std::pair<VID, size_t>     VIDTime;

    typedef std::vector<Point3d>                PointSet;
    typedef std::map<Robot*, const Boundary*>   BoundaryMap;
    typedef std::map<Robot*, Vector3d>          VectorMap;
    typedef std::map<Robot*, std::vector<Robot*>> RobotMap;

    ///@}
    ///@name WorkspaceSkeleton Types
    ///@{

    typedef typename MPTraits::CompositeSkeletonVertex        CompositeSkeletonVertex;
    typedef typename MPTraits::CompositeSkeletonEdge          CompositeSkeletonEdge;
    typedef typename MPTraits::CompositeSkeletonType          CompositeSkeletonType;
    typedef typename CompositeSkeletonType::ED                SkeletonEdgeDescriptor;
    typedef typename CompositeSkeletonType::adj_edge_iterator SkeletonEdgeIterator;
    typedef typename CompositeSkeletonType::vertex_descriptor SkeletonVertexDescriptor;
    typedef typename CompositeSkeletonType::vertex_iterator   SkeletonVertexIterator;

    ///@}
    ///@name MAPF Types
    ///@{
    
    typedef GenericStateGraph<std::pair<size_t,size_t>,double> HeuristicSearch;
    typedef std::vector<size_t>                                CBSSolution;

    // A constraint is formulated as an (edge, time) represented by ((vid, vid=MAX), time)
    typedef std::pair<std::pair<SkeletonVertexDescriptor, SkeletonVertexDescriptor>, size_t> CBSConstraint;
    typedef CBSNode<Robot,CBSConstraint,CBSSolution>           CBSNodeType;

    typedef Robot* OrderingConstraint;
    typedef CBSNode<Robot, OrderingConstraint, CBSSolution>    PBSNodeType;

    ///@}
    ///@name Local Types
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// Representation of a sampling region.
    ////////////////////////////////////////////////////////////////////////////
    struct SamplingRegion {

      ///@name Internal State
      ///@{

      SkeletonEdgeIterator edgeIterator; ///< Iterator to region's edge.

      size_t edgeIndex{0};   ///< Which edge point are we at?
      double attempts{1};    ///< Number of attempts to extend into this region.
      double successes{1};   ///< Number of successful attempts.

      std::unordered_set<Robot*> activeRobots; ///< The robots that move on the edge.

      ///@}

      SamplingRegion() {}

      SamplingRegion(const SkeletonEdgeIterator& _eit) : edgeIterator(_eit) {
        activeRobots = edgeIterator->property().GetActiveRobots();
      }

      /// Track the success rate of extending into this region.
      void TrackSuccess(const size_t _success, const size_t _attempts) {
        successes *= .9;
        attempts  *= .9;
        successes += _success;
        attempts  += _attempts;
      }

      void IncrementSuccess() {
        successes += 1;
      }

      void IncrementAttempts() {
        attempts += 1;
      }

      /// Compute the weight for this region (i.e. success rate).
      double GetWeight() const noexcept {
        return successes / attempts;
      }

      /// Get the center of this region.
      const VectorMap GetCenter() const noexcept {
        VectorMap center;
        auto compState = (*edgeIterator).property().GetIntermediates()[edgeIndex];

        for(auto r : compState.GetRobots()) {
          center.insert(std::pair<Robot*, Vector3d>(r, compState.GetRobotCfg(r)));
        }

        return center;
      }

      /// Check if this region is at the last point on its skeleton edge.
      bool LastPoint() const noexcept {
        return edgeIndex == (*edgeIterator).property().GetNumIntermediates() - 1;
      }

      /// Advance this region to the next skeleton edge point.
      void Advance() noexcept {
        ++edgeIndex;
      }

      /// Assignment operator
      SamplingRegion& operator=(const SamplingRegion& _region) {
        if(this != &_region) {
          edgeIterator = _region.edgeIterator;
          edgeIndex = _region.edgeIndex;
          attempts = _region.attempts;
          successes = _region.successes;
          activeRobots = _region.activeRobots;
        }
        return *this;
      }

      /// Equality operator
      bool operator==(const SamplingRegion& _region) const {
        return edgeIterator == _region.edgeIterator;
      }

    };

    ///@}
    ///@ Construction
    ///@{

    CompositeDynamicRegionRRT();

    CompositeDynamicRegionRRT(XMLNode& _node);

    virtual ~CompositeDynamicRegionRRT() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;

    virtual void Finalize() override;

    ///@}
    ///@name GroupRRTStrategy Overrides
    ///@{

    /// Get a random configuration to grow towards.
    virtual GroupCfgType SelectTarget() override;

    /// Add a new configuration to the roadmap and current tree.
    /// @param _newCfg The new configuration to add.
    /// @return A pair with the added VID and a bool indicating whether the new
    ///         node was already in the map.
    virtual std::pair<VID, bool> AddNode(GroupCfgType& _newCfg) override;

    ///@}
    ///@name Helpers
    ///@{

    /// Computes the cost to go from every vertex in the workspace skeleton
    /// to the vertex closest to the goal for each robot.
    void InitializeCostToGo();

    /// Computes MAPF paths for each robot over the workspace skeleton and 
    /// translates these paths into a path over the composite skeleton.
    /// @param _vid The composite skeleton VID to start the MAPF solution from.
    void ComputeMAPFPaths(const VID _vid);

    /// Evaluates the cost of a CBS solution over the workspace skeleton.
    /// @param _node The CBS node to evaluate the cost of.
    double CostFunction(CBSNodeType& _node);

    /// Forms new CBS nodes from new constraints.
    std::vector<CBSNodeType> SplitNodeFunction(CBSNodeType& _node,
        std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
        CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution>& _lowLevel,
        CBSCostFunction<Robot,CBSConstraint,CBSSolution>& _cost);

    /// Finds a low level solution for a robot over the workspace skeleton for CBS.
    bool LowLevelPlanner(CBSNodeType& _node, Robot* _robot);

    /// Finds conflicts in a CBS solution.
    std::vector<std::pair<Robot*,CBSConstraint>> 
    ValidationFunction(CBSNodeType& _node);

    /// Evaluates the cost of a PBS solution over the workspace skeleton.
    /// @param _node The PBS node to evaluate the cost of.
    double CostFunction(PBSNodeType& _node);

    /// Forms new PBS nodes from new constraints.
    std::vector<PBSNodeType> SplitNodeFunction(PBSNodeType& _node,
        std::vector<std::pair<Robot*,OrderingConstraint>> _constraints,
        CBSLowLevelPlanner<Robot,OrderingConstraint,CBSSolution>& _lowLevel,
        CBSCostFunction<Robot,OrderingConstraint,CBSSolution>& _cost);

    /// Evaluate whether a set of priorities has a circular dependency.
    /// @param _robot The robot which the new constraint applies to.
    /// @param _newConstraint The new constraint which will be added.
    /// @param _constraints The existing priority constraints.
    /// @return Whether the new constraint adds a circular dependency.
    bool CheckCircularDependency(Robot* _robot, const OrderingConstraint& _newConstraint, 
                const std::map<Robot*,std::set<OrderingConstraint>>& _constraints);

    /// Finds a low level solution for a robot over the workspace skeleton for PBS.
    bool LowLevelPlanner(PBSNodeType& _node, Robot* _robot);

    /// Specify that robots that depend on _robot must also replan.
    void AddDependencies(std::set<Robot*>& _needsToReplan, Robot* _robot,
                const PBSNodeType& _node);

    /// Finds conflicts in a PBS solution.
    std::vector<std::pair<Robot*,OrderingConstraint>> 
    ValidationFunction(PBSNodeType& _node);

    /// Choose the next best region to sample from.
    /// @param replan Whether to replan the MAPF solution.
    void NextBestRegion(const bool replan=false);

    /// Sample a configuration from within a sampling region using the sampler
    /// given in m_samplerLabel.
    /// @param _region The region to sample from.
    /// @return Whether a configuration was successfully sampled and the 
    /// configuration within the sampling region.
    std::pair<bool, GroupCfgType> Sample(SamplingRegion* _region);

    /// Sample a configuration from within a given boundary using the sampler
    /// given in _samplerLabel.
    /// @param _region The region to sample from.
    /// @return A configuration within the boundary.
    GroupCfgType Sample(const Boundary* const _boundary);

    /// Determine if a region is touching a configuration.
    /// @param _cfg The configuration.
    /// @param _region The sampling region.
    bool IsTouching(const GroupCfgType& _cfg, SamplingRegion& _region);

    /// Calculate the boundary around a sampling region.
    /// @param _robot The robot to make the region for.
    /// @param _v The center of the sampling region.
    /// @param _deflate Deflate the boundary radius to account for penetration?
    /// @return The boundary with center _v and radius m_regionRadius.
    CSpaceBoundingSphere MakeBoundary(Robot* _robot, const VectorMap _v, 
                const bool _deflate=false);

    ///@}
    ///@name Skeleton and Workspace
    ///@{

    /// Build topological skeleton
    void BuildSkeleton();

    /// Select a region based on weighted success probabilities
    /// @return The sampling region to be expanded
    const size_t SelectSamplingRegion();

    /// Compute probabilities for selecting each sampling region.
    /// @return Probabilities based on extension success
    std::vector<double> ComputeProbabilities();

    /// Advance the sampling region until it is no longer touching the
    /// newly added configuration.
    /// @param _cfg The newly added configuration, q_new.
    void AdvanceRegion(const GroupCfgType& _cfg);

    /// Advance a region until it is either not longer touching a configuration
    /// or until it reaches the end of its respective skeleton edge.
    /// @param _cfg A configuration possibly touching the region.
    /// @param _region The region to advance along its skeleton edge.
    bool AdvanceRegionToCompletion(const GroupCfgType& _cfg, SamplingRegion* _region);

    ///@}
    ///@name Internal State
    ///@{

    /// Stores the distance from each vertex in the composite skeleton
    /// to the goal for each robot.
    std::map<Robot*, std::unordered_map<size_t,double>> m_distanceMap;

    std::unique_ptr<CompositeSkeletonType> m_skeleton; ///< The composite skeleton.

    WorkspaceSkeleton m_individualSkeleton; ///< The original workspace skeleton.

    /// The individual skeleton VIDs closest to the start and goal for each robot
    std::pair<std::vector<SkeletonVertexDescriptor>, 
              std::vector<SkeletonVertexDescriptor>> m_skeletonQuery;

    /// The workspace skeleton VID to start the MAPF query from for each robot.
    std::unordered_map<Robot*, SkeletonVertexDescriptor> m_MAPFStarts;

    /// Skeleton clearance annotations.
    std::map<Robot*, PropertyMap<std::vector<double>,double>*> m_annotationMap;

    std::string m_skeletonFilename;  ///< The output file for the skeleton graph
    std::string m_skeletonIO;        ///< Option to read or write the skeleton

    std::string m_skeletonType{"reeb"}; ///< Type of skeleton to build.
    std::string m_decompositionLabel;   ///< The workspace decomposition label.
    std::string m_scuLabel;             ///< The skeleton clearance utility label.

    SamplingRegion m_region; ///< The active dyamic sampling region.

    /// The next composite skeleton edges to explore.
    std::queue<SkeletonEdgeDescriptor> m_nextEdges;

    /// The set of composite skeleton edges that have already been explored.
    std::unordered_set<size_t> m_traversedEdges;

    /// The predecessor for each skeleton vertex in the MAPF solution.
    std::unordered_map<VIDTime, VIDTime> m_predecessorVIDs;

    /// The MAPF solution timestep at which the failed vertex occurs.
    size_t m_timestep{0};

    /// The set of composite skeleton edges that could not be traversed.
    std::set<std::pair<VID, VID>> m_failedEDs;

    /// The order in which MAPF solutions are replanned (back or front).
    std::string m_replanOrder{"back"};

    /// The number of attempts to replan each failed vertex in the MAPF solution.
    std::unordered_map<VID, size_t> m_replanAttempts;

    /// Whether the region was chosen to be sampled from this iteration.
    bool m_choseRegion{false};

    size_t m_maxEdges{10};       ///< The maximum replan attempts per vertex.
    size_t m_maxSampleFails{20}; ///< The maximum fails before an edge is abandoned.

    /// The dynamic sampling regions will have radius equal to this times the
    /// robot's bounding sphere radius.
    double m_regionFactor{2};

    std::map<Robot*, const double> m_regionRadius; ///< The region radius for each robot.

    /// Weight of explore vs. exploit in region selection probabilities.
    /// Exploring is a uniform chance to select each region, while exploit
    /// favors successful regions.
    double m_explore{.5};

    /// A configuration is considered to be touching a region when this fraction
    /// of its bounding sphere penetrates into the region.
    double m_penetrationFactor{1};

    bool m_abortOnFail{true}; ///< Stop planning attempt if all MAPF solutions fail?
    int m_mapfCount{0};       ///< Count the number of calls to MAPF.
    std::string m_mapf{"CBS"};///< The MAPF method to use.

    /// Split skeleton edges into pieces of maximum length m_split * robot radius,
    // value of 0 indicates not to split the edges.
    double m_split{0.};

    /// Shrink region size during sampling to account for penetration distance?
    bool m_deflate{false};

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
CompositeDynamicRegionRRT<MPTraits>::
CompositeDynamicRegionRRT() : GroupRRTStrategy<MPTraits>() {
    this->SetName("CompositeDynamicRegionRRT");
}

template <typename MPTraits>
CompositeDynamicRegionRRT<MPTraits>::
CompositeDynamicRegionRRT(XMLNode& _node) : GroupRRTStrategy<MPTraits>(_node) {
  this->SetName("CompositeDynamicRegionRRT");

  m_skeletonType = _node.Read("skeletonType", true, "",
      "the type of skeleton to use, Available options are reeb and mcs "
      "for 3d, ma for 2d");

  m_skeletonIO = _node.Read("skeletonIO", false, "", "read of write the "
      "skeleton file");

  m_skeletonFilename = _node.Read("skeletonFile", m_skeletonIO != "", "",
      "the skeleton file to read from or write to");

  // If using a reeb skeleton, we need a decomposition to build it.
  m_decompositionLabel = _node.Read("decompositionLabel",
      m_skeletonType == "reeb", "",
      "The workspace decomposition to use.");

  m_scuLabel = _node.Read("scuLabel", false, "", "The skeleton clearance utility "
      "to use. If not specified, we use the hack-fix from wafr16.");

  m_explore = _node.Read("explore", true, m_explore, 0., 1.,
      "Weight of explore vs. exploit in region selection probabilities");

  m_maxEdges = _node.Read("maxEdges", false, m_maxEdges, (size_t)1, SIZE_MAX, 
      "Maximum number of edges to expand from each vertex");

  m_maxSampleFails = _node.Read("maxSampleFails", false, m_maxSampleFails, 
      (size_t)1, SIZE_MAX, 
      "Maximum number of failures before abandoning a region");

  m_regionFactor = _node.Read("regionFactor", true,
      m_regionFactor, 1., std::numeric_limits<double>::max(),
      "Regions are this * robot's bounding sphere radius");

  m_penetrationFactor = _node.Read("penetration", true,
      m_penetrationFactor, std::numeric_limits<double>::min(), 1.,
      "Fraction of bounding sphere penetration that is considered touching");
  
  m_replanOrder = _node.Read("replanOrder", false, "back", "Replan from the front or back.");

  m_abortOnFail = _node.Read("abortOnFail", false, true, "Abort when all edges fail?");

  m_mapf = _node.Read("mapf", false, "CBS", "MAPF method to use (CBS or PBS)");

  m_split = _node.Read("split", false, 0., 0., std::numeric_limits<double>::max(), 
      "Split the skeleton into equal pieces?");

  m_deflate = _node.Read("deflate", false, false, "Deflate region radius during sampling?");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
CompositeDynamicRegionRRT<MPTraits>::Print(std::ostream& _os) const {
  GroupRRTStrategy<MPTraits>::Print(_os);

  _os << "\tSkeleton Type:" << m_skeletonType << std::endl;

  if(!m_decompositionLabel.empty())
    _os << "\tWorkspace Decomposition Label:" << m_decompositionLabel << std::endl;

  if(!m_scuLabel.empty())
    _os << "\tSkeleton Clearance Utility:" << m_scuLabel << std::endl;

  _os << "\tMaximum Number of Edges per Vertex: " << m_maxEdges << std::endl;
  _os << "\tMaximum Number of Region Failures: " << m_maxSampleFails << std::endl;
  _os << "\tRegion Factor: " << m_regionFactor << std::endl;
  _os << "\tPenetration Factor: " << m_penetrationFactor << std::endl;
  _os << "\tExploration Factor: " << m_explore << std::endl;
  _os << "\tReplan Order: " << m_replanOrder << std::endl;
  _os << "\tAbort on Edge Failure: " << m_abortOnFail << std::endl;

  _os << "\tMAPF Method: " << m_mapf << std::endl;
  _os << "\tSplit Skeleton Edges: " << m_split << std::endl;
}

/*---------------------------- MPStrategy Overrides --------------------------*/

template <typename MPTraits>
void
CompositeDynamicRegionRRT<MPTraits>::Initialize(){
  GroupRRTStrategy<MPTraits>::Initialize();
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::InitializeRoadmap");

  this->GetStatClass()->SetStat(this->GetNameAndLabel() + "::MAPFHeuristicCalls", 0);

  // Check that only one direction is being extended.
  if(this->m_numDirections > 1)
    throw RunTimeException(WHERE) << "Extending more than one direction "
                                  << "is not supported.";
  if(this->m_growGoals)
    throw RunTimeException(WHERE) << "Bidirectional growth is not supported.";

  if(this->m_goalDmLabel.empty())
    throw RunTimeException(WHERE) << "Goal distance metric label is required.";

  if(m_mapf != "CBS" and m_mapf != "PBS")
    throw RunTimeException(WHERE) << "Unknown MAPF method provided (options are CBS or PBS)";

  auto groupTask = this->GetGroupTask();

  // Disable velocity biasing if a robot is holonomic.
  for(auto& task : *groupTask) {
    auto robot = task.GetRobot();
    const double robotRadius = robot->GetMultiBody()->
                               GetBoundingSphereRadius() * m_regionFactor;
    m_regionRadius.insert(std::pair<Robot*, const double>(robot, robotRadius));
  }

  // Initialize the skeleton and regions.
  BuildSkeleton();

  // Get the individual skeleton vids closest to the start and goal
  std::vector<SkeletonVertexDescriptor> goalSkelVIDs;
  std::vector<SkeletonVertexDescriptor> startSkelVIDs;

  // Get the Points that correspond to the individual cfg goals.
  PointSet targets;
  int idx = 0;
  for(auto iter = groupTask->begin(); iter != groupTask->end(); iter++) {
    if(iter->GetGoalConstraints().size() != 1) 
      throw RunTimeException(WHERE) << "Exactly one goal is required.";

    const auto center = iter->GetGoalConstraints()[0]->GetBoundary()->GetCenter();
    const bool threeD = iter->GetRobot()->GetMultiBody()->GetBaseType()
                      == Body::Type::Volumetric;

    double centerVec[3];
    centerVec[0] = center[0];
    centerVec[1] = center[1];
    if(threeD)
      centerVec[2] = center[2];
    else
      centerVec[2] = 0.;

    targets.push_back(Vector<double, 3>(centerVec));
    auto skelIter = m_skeleton->GetIndividualGraph(idx)->FindNearestVertex(targets[targets.size()-1]);
    goalSkelVIDs.push_back(skelIter->descriptor());
    ++idx;
  }

  // Get the point values of all of the start configurations.
  PointSet start;
  auto& startVIDs = this->GetGoalTracker()->GetStartVIDs();
  if(startVIDs.size() == 1) {
    const auto vi = this->GetGroupRoadmap()->find_vertex(*startVIDs.begin());
    auto gcfg = vi->property();
    for(auto r : gcfg.GetRobots())
      start.push_back(gcfg.GetRobotCfg(r).GetPoint());
  } else
    throw RunTimeException(WHERE) << "Exactly one start VID is required.";

  if(this->m_debug)
    std::cout << "Initializing regions at start skeleton vertex."
              << std::endl;

  // Add the composite state corresponding to the composite start point.
  CompositeSkeletonVertex vertex(m_skeleton.get());
  for(size_t i = 0; i < vertex.GetNumRobots(); ++i) {
    const auto iter = m_individualSkeleton.FindNearestVertex(start.at(i));
    const auto vid = iter->descriptor();
    startSkelVIDs.push_back(vid);
    vertex.SetRobotCfg(i, vid);
  }

  // Initialize the closeness metric for region selection
  m_skeletonQuery = std::make_pair(startSkelVIDs, goalSkelVIDs);
  InitializeCostToGo();

  const auto compVID = m_skeleton->AddVertex(vertex);
  auto vidTime = std::make_pair(compVID, m_timestep);
  m_predecessorVIDs[vidTime] = std::make_pair(INVALID_VID, 0);
  m_replanAttempts[compVID] = 1;

  // Find the initial MAPF solution and make the first region.
  ComputeMAPFPaths(compVID);
  NextBestRegion();
}


template <typename MPTraits>
void
CompositeDynamicRegionRRT<MPTraits>::
Finalize() {
  this->GetStatClass()->SetStat(this->GetNameAndLabel() + "::MAPFHeuristicCalls", 
                                m_mapfCount);
  GroupRRTStrategy<MPTraits>::Finalize();
}

/*------------------------ GroupRRTStrategy Overrides ------------------------*/

template <typename MPTraits>
typename MPTraits::GroupCfgType
CompositeDynamicRegionRRT<MPTraits>::
SelectTarget() {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::SelectTarget");

  auto grm = this->GetGroupRoadmap();
  GroupCfgType gcfg(grm);

  // See if max sample fails has been exceeded.
  auto eit = m_region.edgeIterator;
  auto fails = m_region.attempts - m_region.successes;

  if(fails > m_maxSampleFails) {
    if(this->m_debug)
      std::cout << "Exceeded maximum number of failures. Deleting region on edge from "
                << eit->source() << " to "
                << eit->target() << std::endl;

    m_failedEDs.insert(std::make_pair(eit->source(), eit->target()));
    NextBestRegion(true);
  }

  size_t regionIdx = SIZE_MAX;
  bool notFound = true;
  while(notFound) {
    notFound = false;
    // Select a region for sample generation
    regionIdx = SelectSamplingRegion();

    if(regionIdx > 0) {
      gcfg = Sample(this->GetEnvironment()->GetBoundary());
    } else {
      // Check if a valid cfg was found
      auto result = Sample(&m_region);
      m_region.IncrementAttempts();

      notFound = not result.first;
      if(notFound) {
        if((m_region.attempts - m_region.successes) > m_maxSampleFails) {
          if(this->m_debug)
            std::cout << "Replacing region for target VID " 
                      << eit->target() << std::endl;

          NextBestRegion(true);
        }
        continue;
      }
        
      gcfg = result.second;
    }
  }

  m_choseRegion = (regionIdx == 0);

  return gcfg;
}

template <typename MPTraits>
std::pair<typename CompositeDynamicRegionRRT<MPTraits>::VID, bool>
CompositeDynamicRegionRRT<MPTraits>::
AddNode(GroupCfgType& _newCfg) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::AddNode");

  auto g = this->GetGroupRoadmap();

  const VID lastVID = g->GetLastVID();
  const VID newVID  = g->AddVertex(_newCfg);

  const bool nodeIsNew = lastVID != g->GetLastVID();

  if(nodeIsNew) {
    if(this->m_debug)
      std::cout << "\tAdding VID " << newVID << "."
                << std::endl;

    if(m_choseRegion == 0)
      m_region.IncrementSuccess();

    // On each new sample, check if we need to advance our region and generate
    // new ones.
    auto vi = g->find_vertex(newVID);

    PointSet p;
    auto compState = vi->property();
    for(auto r : compState.GetRobots()) {
      p.push_back(compState.GetRobotCfg(r).GetPoint());
    }

    AdvanceRegion(compState);
  }

  return {newVID, nodeIsNew};
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
std::pair<bool, typename MPTraits::GroupCfgType>
CompositeDynamicRegionRRT<MPTraits>::
Sample(SamplingRegion* _region) {
  if (this->m_debug) {
    std::cout << "\tSampling from region with center at on edge "
              << _region->edgeIterator->id() << " (intermediate "
              << _region->edgeIndex << "/"
              << _region->edgeIterator->property().GetNumIntermediates()
              << "), success rate so far "
              << _region->successes << " / " << _region->attempts
              << ", using sampler '" << this->m_samplerLabel << "'." << std::endl;

    for(auto r : _region->activeRobots) {
      std::cout << "\t Region centered at " << _region->GetCenter().at(r)
                << "for robot " << r->GetLabel() << "." << std::endl;
    }
  }

  // Get a boundary map for the composite region.
  auto robots = this->GetGroupTask()->GetRobotGroup()->GetRobots();
  const auto center = _region->GetCenter();

  std::vector<CSpaceBoundingSphere> bounds;
  for(auto robot : robots) {
    bounds.push_back(MakeBoundary(robot, center, m_deflate));
  }

  BoundaryMap boundMap;
  for(size_t i = 0; i < robots.size(); i++) {
    boundMap.emplace(std::make_pair(robots.at(i), &bounds.at(i)));
  }

  // Get the sampler.
  auto s = this->GetSampler(this->m_samplerLabel);

  // Try to sample 10 times before we consider it a failure
  std::vector<GroupCfgType> samples, collision;
  int tries = 0;

  while(tries < 10 and samples.empty()) {
    ++tries;
    s->Sample(1, 100, boundMap, std::back_inserter(samples),
      std::back_inserter(collision));
  }

  if(samples.empty()) {
    return std::make_pair<bool, GroupCfgType>(false, GroupCfgType());
  }

  auto& target = samples.front();

  if(this->m_debug)
    std::cout << "\t" << target.PrettyPrint() << std::endl;

  return std::make_pair<bool,GroupCfgType>(true, std::move(target));
}


template <typename MPTraits>
typename MPTraits::GroupCfgType
CompositeDynamicRegionRRT<MPTraits>::
Sample(const Boundary* const _boundary) {
  // Get the sampler.
  auto s = this->GetSampler(this->m_samplerLabel);

  std::vector<GroupCfgType> samples, collision;
  while(samples.empty())
    s->Sample(1, 100, _boundary, std::back_inserter(samples),
      std::back_inserter(collision));

  auto target = samples.front();
  if(this->m_debug)
    std::cout << "\t" << target.PrettyPrint() << std::endl;

  return target;
}


template <typename MPTraits>
bool
CompositeDynamicRegionRRT<MPTraits>::
IsTouching(const GroupCfgType& _groupCfg, SamplingRegion& _region) {
  const auto center = _region.GetCenter();

  bool touching = true;
  for (auto r : _region.activeRobots) {
    auto cfg = _groupCfg.GetRobotCfg(r);

    // Compute the penetration distance required. We want the robot's bounding
    // sphere to penetrate the region by the fraction m_penetrationThreshold.
    const double robotRadius  = cfg.GetMultiBody()->GetBoundingSphereRadius(),
                 threshold    = 2 * robotRadius * m_penetrationFactor;

    // Get the region boundary.
    auto boundary = MakeBoundary(r, center);

    // Compute the penetration distance (maximally enclosed bounding diameter).
    const Point3d robotCenter = cfg.GetPoint();
    const double penetration = boundary.GetClearance(robotCenter) + robotRadius;

    // The configuration is touching if the penetration exceeds the threshold.
    if(penetration < threshold) {
      touching = false;
      break;
    }
  }

  if(this->m_debug)
    std::cout << "\t Touch test: " << (touching ? "passed" : "failed") << std::endl;

  return touching;
}


template <typename MPTraits>
CSpaceBoundingSphere
CompositeDynamicRegionRRT<MPTraits>::
MakeBoundary(Robot* _robot, const VectorMap _v, const bool _deflate) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::MakeBoundary");

  const bool threeD = _robot->GetMultiBody()->GetBaseType()
                   == Body::Type::Volumetric;
  auto indV = _v.at(_robot);

  double radius = m_regionRadius.at(_robot);
  if(_deflate)
    radius -= m_penetrationFactor * _robot->GetMultiBody()->GetBoundingSphereRadius();

  // I'm not sure what the boundary code might do with a negative radius. Bound
  // it below at zero just in case.
  radius = std::max(0., radius);

  if (threeD)
    return CSpaceBoundingSphere({indV[0], indV[1], indV[2]}, radius);
  else
    return CSpaceBoundingSphere({indV[0], indV[1]}, radius);
}

/*--------------------------- Skeleton and Workspace -------------------------*/

template <typename MPTraits>
void
CompositeDynamicRegionRRT<MPTraits>::
BuildSkeleton() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::BuildSkeleton");

  // Get robots.
  auto robots = this->GetGroupTask()->GetRobotGroup()->GetRobots();

  // Determine if we need a 2d or 3d skeleton.
  auto env = this->GetEnvironment();
  const bool threeD = robots[0]->GetMultiBody()->GetBaseType() ==
      Body::Type::Volumetric;


  if(m_skeletonIO == "read") {
    m_individualSkeleton.Read(m_skeletonFilename);
    m_individualSkeleton.DoubleEdges();
  }
  else {
    if(threeD) {
      if(m_skeletonType == "mcs") {
        if(this->m_debug)
          std::cout << "Building a Mean Curvature skeleton." << std::endl;
        MeanCurvatureSkeleton3D mcs;
        mcs.SetEnvironment(this->GetEnvironment());
        mcs.BuildSkeleton();

        // Create the workspace skeleton.
        auto sk = mcs.GetSkeleton();
        m_individualSkeleton = sk.first;
      }
      else if(m_skeletonType == "reeb") {
        // Create a workspace skeleton using a reeb graph.
        if(this->m_debug)
          std::cout << "Building a Reeb Graph skeleton." << std::endl;
        auto decomposition = this->GetMPTools()->GetDecomposition(
            m_decompositionLabel);
        ReebGraphConstruction reeb;
        reeb.Construct(decomposition);

        // Create the workspace skeleton.
        m_individualSkeleton = reeb.GetSkeleton();
      }
      else
        throw ParseException(WHERE) << "Unrecognized skeleton type '"
          << m_skeletonType << "', options for 3d "
          << "problems are {mcs, reeb}.";
    }
    else {
      // Collect the obstacles we want to consider (all in this case).
      std::vector<GMSPolyhedron> polyhedra;
      for(size_t i = 0; i < env->NumObstacles(); ++i) {
        MultiBody* const obstacle = env->GetObstacle(i);
        for(size_t j = 0; j < obstacle->GetNumBodies(); ++j)
          polyhedra.emplace_back(obstacle->GetBody(j)->GetWorldPolyhedron());
      }

      // Build a skeleton from a 2D medial axis.
      if(this->m_debug)
        std::cout << "Build a skeleton from a 2D medial axis." << endl;
      MedialAxis2D ma(polyhedra, env->GetBoundary());
      ma.BuildMedialAxis();
      m_individualSkeleton = get<0>(ma.GetSkeleton(1)); // 1 for free space.
    }
  }

  if(m_split > 0) {
    double maxLength = m_split * robots[0]->GetMultiBody()->GetBoundingSphereRadius();
    m_individualSkeleton.RefineEdges(maxLength);
    // m_individualSkeleton.Write(this->GetBaseFilename() + ".split.graph");
  }
  m_individualSkeleton.DoubleEdges();

  // Get the clearance annotations
  auto annot = ClearanceAnnotatedSkeleton(this, &m_individualSkeleton, true);

  // Add self edges for waiting
  for(auto vid : m_individualSkeleton.GetAllVIDs()) {
    std::vector<Point3d> p = {};
    m_individualSkeleton.AddEdge(vid, vid, p);
  }

  std::vector<WorkspaceSkeleton*> ws;
  for (size_t i = 0; i < robots.size(); ++i) {
    ws.push_back(&m_individualSkeleton);
    m_annotationMap.insert(std::make_pair(robots[i], annot));
    if(this->m_debug)
      std::cout << "Added workspace skeleton for " << robots.at(i)->GetLabel() << "." << endl;
  }
  m_skeleton = std::unique_ptr<CompositeSkeletonType>(
    new CompositeSkeletonType(this->GetGroupTask()->GetRobotGroup(), ws));

  if(m_skeletonIO == "write")
    m_individualSkeleton.Write(m_skeletonFilename);
}


template<typename MPTraits>
const size_t
CompositeDynamicRegionRRT<MPTraits>::
SelectSamplingRegion() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::SelectSamplingRegion");

  // Update all region probabilities.
  const std::vector<double> probabilities = ComputeProbabilities();

  // Select a region to sample from. The last region is the whole environment.
  double rand = DRand();
  double lowerBound = 0.0;
  int index;

  for(index = 0; index < (int)probabilities.size(); index++) {
    if((lowerBound < rand) and (rand < lowerBound + probabilities[index]))
      break;

    lowerBound += probabilities[index];
  }

  if(this->m_debug) {
    std::cout << "Computed region selection probabilities ("
              << "last is whole env):\n\t";

    for(auto p : probabilities)
      std::cout << std::setprecision(4) << p << " ";

    std::cout << "\n\tSelected index " << index
              << (index == 0 ? "." : " (whole env).")
              << std::endl;
  }

  return index;
}


template <typename MPTraits>
std::vector<double>
CompositeDynamicRegionRRT<MPTraits>::
ComputeProbabilities() {

  // If there is no active region, sample from the environment with probability 1
  if(m_region.activeRobots.size() < 1)
    return {0.0, 1.0};

  return {1.0 - m_explore, m_explore};
}


template <typename MPTraits>
void
CompositeDynamicRegionRRT<MPTraits>::
InitializeCostToGo() {
  for(size_t i = 0; i < m_skeleton->GetNumRobots(); ++i) {
    auto ws = m_skeleton->GetIndividualGraph(i);

    SSSPTerminationCriterion<WorkspaceSkeleton> termination(
        [](typename WorkspaceSkeleton::vertex_iterator& _vi,
          const SSSPOutput<WorkspaceSkeleton>& _sssp) {
          return SSSPTermination::Continue;
        }
    );

    SSSPPathWeightFunction<WorkspaceSkeleton> weight = [&](
        typename WorkspaceSkeleton::adj_edge_iterator& _ei,
        const double _sourceDistance,
        const double _targetDistance) {

      const auto source = _ei->source();
      const auto target = _ei->target();

      const auto start = ws->find_vertex(source);
      const auto goal = ws->find_vertex(target);

      return _sourceDistance + (goal->property() - start->property()).norm();
    };

    std::vector<WorkspaceSkeleton::vertex_descriptor> vds = {m_skeletonQuery.second.at(i)};
    auto robot = m_skeleton->GetGroup()->GetRobot(i);
    auto output = DijkstraSSSP(ws, vds, weight, termination);
    m_distanceMap.insert(std::pair<Robot*, std::unordered_map<size_t, double>>
                                              (robot, output.distance));
  }
}


template <typename MPTraits>
std::vector<std::pair<Robot*, typename CompositeDynamicRegionRRT<MPTraits>::CBSConstraint>>
CompositeDynamicRegionRRT<MPTraits>::
ValidationFunction(CBSNodeType& _node) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ValidationFunction");

  size_t maxTimestep = 0;
  for(auto kv : _node.solutionMap) {
    maxTimestep = std::max(maxTimestep,kv.second->size());
  }

  std::unordered_map<SkeletonVertexDescriptor, std::unordered_map<size_t, double>> vertexCapacity;
  std::unordered_map<std::pair<SkeletonVertexDescriptor, SkeletonVertexDescriptor>, 
      std::unordered_map<size_t, double>> edgeCapacity;

  // new constraints of form (robot, ((vid, vid), time))
  std::vector<std::pair<Robot*, CBSConstraint>> constraints;

  // iterate through robots first to get the remaining capacity
  for(size_t i = 0; i < maxTimestep; i++) {
    for(auto iter = _node.solutionMap.begin(); iter != _node.solutionMap.end(); iter++) {
      auto robot = iter->first;
      auto path = *(iter->second);

      // source vertex (current timestep)
      auto s = std::min(i, path.size()-1);
      auto source = path[s];

      // target vertex (next timestep)
      auto t = std::min(i+1,path.size()-1);
      auto target = path[t];

      // Get the edge in the individual skeleton
      if(source != target) {
        // order such that source < target
        SkeletonVertexDescriptor sVID;
        SkeletonVertexDescriptor tVID;
        if(source < target) {
          sVID = source;
          tVID = target;
        } else {
          sVID = target;
          tVID = source;
        }

        auto robotIdx = m_skeleton->GetGroup()->GetGroupIndex(robot);
        WorkspaceSkeleton::adj_edge_iterator ei;
        m_skeleton->GetIndividualGraph(robotIdx)->GetEdge(sVID, tVID, ei);
        auto eid = ei->descriptor();
        auto edgePair = std::make_pair(sVID, tVID);

        // Check if this edge has been added
        if(edgeCapacity.find(edgePair) != edgeCapacity.end()) {
          // Check if this timestep has been added
          if(edgeCapacity.at(edgePair).find(i) != edgeCapacity.at(edgePair).end()) {
            // This edge and time are in the map, decrease the capacity as needed
            edgeCapacity[edgePair][i] -= robot->GetMultiBody()->GetBoundingSphereRadius();
          } else {
            // This time is not in the map, add it with the remaining capacity
            auto ds = m_annotationMap.at(robot)->GetEdgeProperty(eid);
            auto d = *std::min_element(ds.begin(), ds.end());
            d -= robot->GetMultiBody()->GetBoundingSphereRadius();
            edgeCapacity[edgePair].emplace(std::make_pair(i, d));
          }
        } else {
          // This edge needs to be added
          auto ds = m_annotationMap.at(robot)->GetEdgeProperty(eid);
          auto d = *std::min_element(ds.begin(), ds.end());
          d -= robot->GetMultiBody()->GetBoundingSphereRadius();

          std::unordered_map<size_t, double> eMap;
          eMap.emplace(i, d);
          edgeCapacity.emplace(std::make_pair(edgePair, eMap));
        }
      }
      
      // Check if this vid has been added yet
      if(vertexCapacity.find(source) != vertexCapacity.end()) {
        // Check if this timestep has been added
        if(vertexCapacity.at(source).find(i) != vertexCapacity.at(source).end()) {
          // This vid and time are in the map, decrease the capacity as needed
          vertexCapacity[source][i] -= robot->GetMultiBody()->GetBoundingSphereRadius();
        } else {
          // This time is not in the map, add it with the remaining capacity
          auto d = m_annotationMap.at(robot)->GetVertexProperty(source);
          d -= robot->GetMultiBody()->GetBoundingSphereRadius();
          vertexCapacity[source].emplace(std::make_pair(i, d));
        }
      } else {
        // This vid needs to be added
        auto d = m_annotationMap.at(robot)->GetVertexProperty(source);
        d -= robot->GetMultiBody()->GetBoundingSphereRadius();

        std::unordered_map<size_t, double> vMap;
        vMap.emplace(i, d);
        vertexCapacity.emplace(std::make_pair(source, vMap));
      }
    }
  }

  // Form a constraint for every robot, vid, timestep that has below 0 capacity remaining
  for(size_t i = 0; i < maxTimestep; i++) {
    std::unordered_set<Robot*> addedEdgeConstraint;
    std::unordered_set<Robot*> addedVertexConstraint;

    std::unordered_map<Robot*, VID> starts;
    std::unordered_map<Robot*, VID> targets;
    for(auto iter = _node.solutionMap.begin(); iter != _node.solutionMap.end(); iter++) {

      // Find the VID of this robot at time i
      auto robot = iter->first;
      auto path = *(iter->second);

      auto s = std::min(i, path.size()-1);
      auto source = path[s];
      auto t = std::min(i+1, path.size()-1);
      auto target = path[t];

      starts[robot] = source;
      targets[robot] = target;

      // order such that source < target
      auto sVID = std::min(source, target);
      auto tVID = std::max(source, target);
      auto edgePair = std::make_pair(sVID, tVID);

      // Check the capacity of the source vertex
      if(vertexCapacity.at(source).at(i) < 0) {
        auto vPair = std::make_pair(source, SIZE_MAX);
        constraints.push_back(std::make_pair(robot, std::make_pair(vPair, i)));
        addedVertexConstraint.insert(robot);
      }

      // Check for an edge
      if(source != target) {
        // Check the capacity of the vertex
        if(edgeCapacity.at(edgePair).at(i) < 0) {
          constraints.push_back(std::make_pair(robot, std::make_pair(edgePair, i)));
          addedEdgeConstraint.insert(robot);
        }
      }
    }

    for(auto ed : m_failedEDs) {
      auto startV = m_skeleton->GetVertex(ed.first);
      auto targetV = m_skeleton->GetVertex(ed.second);

      size_t matchedConstraint = 0;

      auto robots = this->GetGroupTask()->GetRobotGroup()->GetRobots();
      for(auto robot : robots) {
        auto start = starts.at(robot);
        auto target = targets.at(robot);

        if(startV.GetVID(robot) != start)
          continue;
        if(targetV.GetVID(robot) != target)
          continue;

        matchedConstraint++;
      }

      if(matchedConstraint == robots.size()) {
        for(auto robot : robots) {
          auto start = starts.at(robot);
          auto target = targets.at(robot);

          if(start == target and !addedVertexConstraint.count(robot)) {
            auto vPair = std::make_pair(start, SIZE_MAX);
            constraints.push_back(std::make_pair(robot, std::make_pair(vPair, i)));
          }

          if(start != target and !addedEdgeConstraint.count(robot)) {
            auto edgePair = std::make_pair(start, target);
            constraints.push_back(std::make_pair(robot, std::make_pair(edgePair, i)));
          }
        }
      }
    }
  }

  return constraints;
}


template <typename MPTraits>
std::vector<std::pair<Robot*, typename CompositeDynamicRegionRRT<MPTraits>::OrderingConstraint>>
CompositeDynamicRegionRRT<MPTraits>::
ValidationFunction(PBSNodeType& _node) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ValidationFunction");

  size_t maxTimestep = 0;
  for(auto kv : _node.solutionMap) {
    maxTimestep = std::max(maxTimestep,kv.second->size());
  }

  std::unordered_map<SkeletonVertexDescriptor, std::unordered_map<size_t, double>> vertexCapacity;
  std::unordered_map<std::pair<SkeletonVertexDescriptor, SkeletonVertexDescriptor>, 
      std::unordered_map<size_t, double>> edgeCapacity;

  // new constraints of form (robot, robot)
  std::vector<std::pair<Robot*, OrderingConstraint>> constraints;

  // iterate through robots first to get the remaining capacity
  for(size_t i = 0; i < maxTimestep; i++) {
    for(auto iter = _node.solutionMap.begin(); iter != _node.solutionMap.end(); iter++) {
      auto robot = iter->first;
      auto path = *(iter->second);

      // source vertex (current timestep)
      auto s = std::min(i, path.size()-1);
      auto source = path[s];

      // target vertex (next timestep)
      auto t = std::min(i+1,path.size()-1);
      auto target = path[t];

      // Get the edge in the individual skeleton
      if(source != target) {
        // order such that source < target
        auto sVID = std::min(source, target);
        auto tVID = std::max(source, target);

        auto robotIdx = m_skeleton->GetGroup()->GetGroupIndex(robot);
        WorkspaceSkeleton::adj_edge_iterator ei;
        m_skeleton->GetIndividualGraph(robotIdx)->GetEdge(sVID, tVID, ei);
        auto eid = ei->descriptor();
        auto edgePair = std::make_pair(sVID, tVID);

        // Check if this edge has been added
        if(edgeCapacity.find(edgePair) != edgeCapacity.end()) {
          // Check if this timestep has been added
          if(edgeCapacity.at(edgePair).find(i) != edgeCapacity.at(edgePair).end()) {
            // This edge and time are in the map, decrease the capacity as needed
            edgeCapacity[edgePair][i] -= robot->GetMultiBody()->GetBoundingSphereRadius();
          } else {
            // This time is not in the map, add it with the remaining capacity
            auto ds = m_annotationMap.at(robot)->GetEdgeProperty(eid);
            auto d = *std::min_element(ds.begin(), ds.end());
            d -= robot->GetMultiBody()->GetBoundingSphereRadius();
            edgeCapacity[edgePair].emplace(std::make_pair(i, d));
          }
        } else {
          // This edge needs to be added
          auto ds = m_annotationMap.at(robot)->GetEdgeProperty(eid);
          auto d = *std::min_element(ds.begin(), ds.end());
          d -= robot->GetMultiBody()->GetBoundingSphereRadius();

          std::unordered_map<size_t, double> eMap;
          eMap.emplace(i, d);
          edgeCapacity.emplace(std::make_pair(edgePair, eMap));
        }
      }
      
      // Check if this source vid has been added yet
      if(vertexCapacity.find(source) != vertexCapacity.end()) {
        // Check if this timestep has been added
        if(vertexCapacity.at(source).find(i) != vertexCapacity.at(source).end()) {
          // This vid and time are in the map, decrease the capacity as needed
          vertexCapacity[source][i] -= robot->GetMultiBody()->GetBoundingSphereRadius();
        } else {
          // This time is not in the map, add it with the remaining capacity
          auto d = m_annotationMap.at(robot)->GetVertexProperty(source);
          d -= robot->GetMultiBody()->GetBoundingSphereRadius();
          vertexCapacity[source].emplace(std::make_pair(i, d));
        }
      } else {
        // This vid needs to be added
        auto d = m_annotationMap.at(robot)->GetVertexProperty(source);
        d -= robot->GetMultiBody()->GetBoundingSphereRadius();

        std::unordered_map<size_t, double> vMap;
        vMap.emplace(i, d);
        vertexCapacity.emplace(std::make_pair(source, vMap));
      }
    }
  }

  // Form a constraint for every robot, vid that has below 0 capacity remaining
  for(size_t i = 0; i < maxTimestep; i++) {
    std::unordered_map<Robot*, VID> starts;
    std::unordered_map<Robot*, VID> targets;

    VID conflictSource = INVALID_VID;
    VID conflictTarget = INVALID_VID;
    for(auto iter = _node.solutionMap.begin(); iter != _node.solutionMap.end(); iter++) {

      // Find the VID of this robot at time i
      auto robot = iter->first;
      auto path = *(iter->second);

      auto s = std::min(i, path.size()-1);
      auto source = path[s];
      auto t = std::min(i+1, path.size()-1);
      auto target = path[t];

      starts[robot] = source;
      targets[robot] = target;

      // order such that source < target
      auto sVID = std::min(source, target);
      auto tVID = std::max(source, target);
      auto edgePair = std::make_pair(sVID, tVID);

      // Check the capacity of the source vertex
      if(vertexCapacity.at(source).at(i) < 0) {
        conflictSource = source;
        break;
      }

      // Check for an edge
      else if(source != target) {
        // Check the capacity of the vertex
        if(edgeCapacity.at(edgePair).at(i) < 0) {
          conflictSource = sVID;
          conflictTarget = tVID;
          break;
        }
      }
    }

    std::vector<Robot*> robots;
    if(conflictSource != INVALID_VID and conflictTarget == INVALID_VID) {
      for(auto iter = _node.solutionMap.begin(); iter != _node.solutionMap.end(); iter++) {

        // Find the VID of this robot at time i
        auto robot = iter->first;
        auto path = *(iter->second);

        auto s = std::min(i, path.size()-1);
        auto source = path[s];

        if(source == conflictSource)
          robots.push_back(robot);
      }
    }

    else if(conflictSource != INVALID_VID and conflictTarget != INVALID_VID) {
      for(auto iter = _node.solutionMap.begin(); iter != _node.solutionMap.end(); iter++) {

        // Find the VID of this robot at time i
        auto robot = iter->first;
        auto path = *(iter->second);

        auto s = std::min(i, path.size()-1);
        auto source = path[s];
        auto t = std::min(i+1, path.size()-1);
        auto target = path[t];

        // order such that source < target
        auto sVID = std::min(source, target);
        auto tVID = std::max(source, target);

        if(sVID == conflictSource and tVID == conflictTarget)
          robots.push_back(robot);
      }
    }

    if(robots.size()) {
      for(size_t k = 0; k < robots.size(); k++) {
        for(size_t l = k+1; l < robots.size(); l++) {
          constraints.push_back(std::make_pair(robots.at(k), robots.at(l)));
          constraints.push_back(std::make_pair(robots.at(l), robots.at(k)));
        }
      }
      break;
    }
  }

  return constraints;
}


template <typename MPTraits>
std::vector<typename CompositeDynamicRegionRRT<MPTraits>::CBSNodeType> 
CompositeDynamicRegionRRT<MPTraits>::
SplitNodeFunction(CBSNodeType& _node,
        std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
        CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution>& _lowLevel,
        CBSCostFunction<Robot,CBSConstraint,CBSSolution>& _cost) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SplitNodeFunction");

  std::vector<CBSNodeType> newNodes;

  for(auto pair : _constraints) {
    // Unpack constraint info
    auto robot = pair.first;
    auto constraint = pair.second;

    // Copy parent node
    CBSNodeType child = _node;
  
    // Add new constraint
    child.constraintMap[robot].insert(constraint);

    // Replan tasks affected by constraint. Skip if no valid replanned path is found
    if(!_lowLevel(child,robot)) 
      continue;

    // Update the cost and add to set of new nodes
    double cost = _cost(child);
    child.cost = cost;
    newNodes.push_back(child);
  }

  return newNodes;
}


template <typename MPTraits>
std::vector<typename CompositeDynamicRegionRRT<MPTraits>::PBSNodeType> 
CompositeDynamicRegionRRT<MPTraits>::
SplitNodeFunction(PBSNodeType& _node,
        std::vector<std::pair<Robot*,OrderingConstraint>> _constraints,
        CBSLowLevelPlanner<Robot,OrderingConstraint,CBSSolution>& _lowLevel,
        CBSCostFunction<Robot,OrderingConstraint,CBSSolution>& _cost) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SplitNodeFunction");

  std::vector<PBSNodeType> children;

  for(auto constraintPair : _constraints) {
    auto robot = constraintPair.first;
    auto constraint = constraintPair.second;
    auto child = _node;

    // Double check that constraints don't make a circular dependency
    if(CheckCircularDependency(robot,constraint,child.constraintMap))
      continue;
    
    child.constraintMap[robot].insert(constraint);

    if(!_lowLevel(child, robot))
      continue;

    child.cost = _cost(child);
    children.push_back(child);
  }

  return children;
}


template <typename MPTraits>
bool
CompositeDynamicRegionRRT<MPTraits>::
CheckCircularDependency(Robot* _robot, const OrderingConstraint& _newConstraint, 
                const std::map<Robot*,std::set<OrderingConstraint>>& _constraints) {

  // Check for duplicate constraints
  if(_constraints.at(_robot).count(_newConstraint))
    return true;

  // Grow tree from new constraint and see if see if a cycle is formed
  std::queue<OrderingConstraint> queue;
  queue.push(_newConstraint);

  while(!queue.empty()) {
    auto r = queue.front();
    queue.pop();
    auto constraints = _constraints.at(r);

    for(auto c : constraints) {
      if(c == _robot)
        return true;

      queue.push(c);
    }
  }

  return false;
}


template <typename MPTraits>
bool
CompositeDynamicRegionRRT<MPTraits>::
LowLevelPlanner(CBSNodeType& _node, Robot* _robot) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::LowLevelPlanner");

  auto constraints = _node.constraintMap[_robot];
  auto h = std::shared_ptr<HeuristicSearch>(new HeuristicSearch(_robot));

  auto robotIdx = m_skeleton->GetGroup()->GetGroupIndex(_robot);

  // Get the start and goal vids in the individual skeleton
  auto start = m_MAPFStarts.at(_robot);
  auto goal = m_skeletonQuery.second.at(robotIdx);

  // Check that start does not violate a constraint and get min end time
  size_t minEndTimestep = 0;
  for(auto constraint : constraints) {

    minEndTimestep = std::max(minEndTimestep,constraint.second);

    if(constraint.first.second != SIZE_MAX or start != constraint.first.first)
      continue;

    if(constraint.second <= 0)
      return false;
  }

  // Distance from each skeleton vertex to the goal
  auto dist2go = m_distanceMap.at(_robot);
  auto g = m_skeleton->GetIndividualGraph(robotIdx);

  auto startVertex = std::make_pair(start,0);
  auto startVID = h->AddVertex(startVertex);

  SSSPTerminationCriterion<HeuristicSearch> termination(
    [goal,minEndTimestep](typename HeuristicSearch::vertex_iterator& _vi,
           const SSSPOutput<HeuristicSearch>& _sssp) {
      
      auto vertex = _vi->property();

      if(goal == vertex.first and minEndTimestep <= vertex.second)
        return SSSPTermination::EndSearch;

      return SSSPTermination::Continue;
    }
  );

  SSSPPathWeightFunction<HeuristicSearch> weight(
    [constraints,h](typename HeuristicSearch::adj_edge_iterator& _ei,
       const double _sourceDistance,
       const double _targetDistance) {
     
      auto source = h->GetVertex(_ei->source()).first;
      auto target = h->GetVertex(_ei->target()).first;
      auto timestep = h->GetVertex(_ei->source()).second;

      auto sVID = std::min(source, target);
      auto tVID = std::max(source, target);

      for(auto constraint : constraints) {
        // Check if the edge violates a constraint
        if(sVID == constraint.first.first and tVID == constraint.first.second) {
          if(timestep == constraint.second) {
            return std::numeric_limits<double>::infinity();
          }
        }

        if(source == constraint.first.first and target == constraint.first.second) {
          if(timestep == constraint.second) {
            return std::numeric_limits<double>::infinity();
          }
        }

        // Ignore any other edges
        if(constraint.first.second != SIZE_MAX)
          continue;

        // Check for a conflict at the source
        if(source == constraint.first.first and timestep == constraint.second) {
          return std::numeric_limits<double>::infinity();
        }

        // Check for a conflict at the target
        if(target == constraint.first.first and timestep+1 == constraint.second) {
          return std::numeric_limits<double>::infinity();
        }
      }

      return _sourceDistance + _ei->property();
    }
  );

  SSSPHeuristicFunction<HeuristicSearch> heuristic(
    [dist2go](const HeuristicSearch* _h, 
       typename HeuristicSearch::vertex_descriptor _source,
       typename HeuristicSearch::vertex_descriptor _target) {

      // Distance to go heuristic
      auto vertex = _h->GetVertex(_target);
      double toGo = dist2go.at(vertex.first);

      return std::max(0.0,toGo);
    }
  );

  SSSPNeighborsFunction<HeuristicSearch> neighbors(
    [g](HeuristicSearch* _h, typename HeuristicSearch::vertex_descriptor _vid) {
      auto vertex = _h->GetVertex(_vid);
      auto gvid = vertex.first;
      auto timestep = vertex.second;
      
      auto vit = g->find_vertex(gvid);

      for(auto eit = vit->begin(); eit != vit->end(); eit++) {
        auto target = eit->target();
        auto neighbor = std::make_pair(target,timestep+1);

        // Use atomic edges
        auto nvid = _h->AddVertex(neighbor);
        _h->AddEdge(_vid,nvid,1.0);
      }
    }
  );

  std::vector<size_t> starts = {startVID};
  std::vector<size_t> goals = {goal};

  auto sssp = AStarSSSP(h.get(),starts,goals,weight,heuristic,neighbors,termination);

  // Check that a path was found
  const size_t last = sssp.ordering.back();
  if(h->GetVertex(last).first != goal or h->GetVertex(last).second < minEndTimestep) {
    if(this->m_debug) {
      std::cout << "Failed to find a path for " << _robot->GetLabel() << std::endl;
    }
    return false;
  }

  // Reconstruct the path
  std::vector<size_t> path = {h->GetVertex(last).first};
  auto current = last;
  do {
    current = sssp.parent.at(current);
    path.push_back(h->GetVertex(current).first);
  } while(current != startVID);
  std::reverse(path.begin(),path.end());

  // Save path in solution
  _node.solutionMap[_robot] = new vector<size_t>();
  *(_node.solutionMap[_robot]) = path;

  return true;
}


template <typename MPTraits>
bool
CompositeDynamicRegionRRT<MPTraits>::
LowLevelPlanner(PBSNodeType& _node, Robot* _robot) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::LowLevelPlanner");

  std::set<Robot*> needsToReplan,hasReplanned;
  needsToReplan.insert(_robot);
  std::vector<VID>* pathCopy = nullptr;

  AddDependencies(needsToReplan,_robot,_node);

  Robot* robot = nullptr;
  while(hasReplanned.size() != needsToReplan.size()) {

    for(auto r : needsToReplan) {
      if(hasReplanned.count(r))
        continue;

      bool ready = true;
      for(auto c : _node.constraintMap.at(r)) {
        if(!needsToReplan.count(c) or hasReplanned.count(c))
          continue;

        ready = false;
        break;
      }

      if(!ready)
        continue;

      robot = r;
      break;
    }


    auto h = std::shared_ptr<HeuristicSearch>(new HeuristicSearch(robot));
    auto robotIdx = m_skeleton->GetGroup()->GetGroupIndex(robot);

    // Get the start and goal vids in the individual skeleton
    auto start = m_MAPFStarts.at(robot);
    auto goal = m_skeletonQuery.second.at(robotIdx);

    // Check that start does not violate a consfalsetraint and get min end time
    size_t minEndTimestep = 0;
    for(auto kv : _node.solutionMap) {
      auto path = kv.second;
      if(path)
        minEndTimestep = std::max(minEndTimestep, path->size() - 1);
    }

    // Distance from each skeleton vertex to the goal
    auto dist2go = m_distanceMap.at(robot);
    auto g = m_skeleton->GetIndividualGraph(robotIdx);
    auto annot = m_annotationMap.at(robot);

    auto startVertex = std::make_pair(start,0);
    auto startVID = h->AddVertex(startVertex);

    auto f = m_failedEDs;
    auto s = m_skeleton.get();

    SSSPTerminationCriterion<HeuristicSearch> termination(
      [goal,minEndTimestep](typename HeuristicSearch::vertex_iterator& _vi,
            const SSSPOutput<HeuristicSearch>& _sssp) {
        
        auto vertex = _vi->property();

        if(goal == vertex.first and minEndTimestep <= vertex.second)
          return SSSPTermination::EndSearch;

        return SSSPTermination::Continue;
      }
    );

    SSSPPathWeightFunction<HeuristicSearch> weight(
      [_node,robot,annot,f,s,g,h](typename HeuristicSearch::adj_edge_iterator& _ei,
        const double _sourceDistance,
        const double _targetDistance) {
      
        auto source = h->GetVertex(_ei->source()).first;
        auto target = h->GetVertex(_ei->target()).first;
        auto timestep = h->GetVertex(_ei->source()).second;

        double edgeUsage = robot->GetMultiBody()->GetBoundingSphereRadius();
        double sourceUsage = robot->GetMultiBody()->GetBoundingSphereRadius();
        double targetUsage = robot->GetMultiBody()->GetBoundingSphereRadius();

        // Preserve transitivity by adding dependencies of dependencies
        auto constraints = _node.constraintMap.at(robot);
        std::queue<OrderingConstraint> addConst;
        for(auto c : constraints)
          addConst.push(c);

        while(!addConst.empty()) {
          auto c = addConst.front();
          for(auto d : _node.constraintMap.at(c)) {
            if(!constraints.count(d)) {
              constraints.insert(d);
              addConst.push(d);
            }
          }
          addConst.pop();
        }

        for(auto r : constraints) {
          auto path = *_node.solutionMap.at(r);

          auto time = timestep >= path.size() ? path.size() - 1 : timestep;
          auto nextTime = timestep + 1 >= path.size() ? path.size() - 1 : timestep + 1;
          auto ps = path.at(time);
          auto pt = path.at(nextTime);

          // Check the capacity at the start vertex and target vertex
          if(ps == source)
            sourceUsage += r->GetMultiBody()->GetBoundingSphereRadius();

          if(pt == target)
            targetUsage += r->GetMultiBody()->GetBoundingSphereRadius();

          // Check the capacity on the edge
          if(ps == pt or source == target)
            continue;

          if((source == ps and target == pt) or (source == pt and target == ps))
            edgeUsage += r->GetMultiBody()->GetBoundingSphereRadius();
        }

        if(sourceUsage > annot->GetVertexProperty(source))
          return std::numeric_limits<double>::infinity();
        
        if(targetUsage > annot->GetVertexProperty(target))
          return std::numeric_limits<double>::infinity();
        
        if(source != target) {
          WorkspaceSkeleton::adj_edge_iterator ei;
          g->GetEdge(source, target, ei);
          auto eid = ei->descriptor();

          auto dists = annot->GetEdgeProperty(eid);
          auto minDist = *std::min_element(dists.begin(), dists.end());
          
          if(edgeUsage > minDist)
            return std::numeric_limits<double>::infinity();
        }

        // Check that not all of the robots are trying to traverse a failed edge
        for(auto ed : f) {
          auto startV = s->GetVertex(ed.first);
          auto targetV = s->GetVertex(ed.second);

          if(startV.GetVID(robot) != source)
              continue;
          if(targetV.GetVID(robot) != target)
              continue;

          size_t matchedConstraint = 1;

          for(auto kv : _node.solutionMap) {
            auto r = kv.first;

            if(r == robot or kv.second == nullptr)
              continue;

            auto path = *kv.second;
            auto s = std::min(timestep, path.size()-1);
            auto rsource = path[s];
            auto t = std::min(timestep+1, path.size()-1);
            auto rtarget = path[t];

            if(startV.GetVID(r) != rsource)
              continue;
            if(targetV.GetVID(r) != rtarget)
              continue;

            matchedConstraint++;
          }

          if(matchedConstraint == _node.solutionMap.size())
            return std::numeric_limits<double>::infinity();
        }

        return _sourceDistance + _ei->property();
      }
    );

    SSSPHeuristicFunction<HeuristicSearch> heuristic(
      [dist2go](const HeuristicSearch* _h, 
        typename HeuristicSearch::vertex_descriptor _source,
        typename HeuristicSearch::vertex_descriptor _target) {

        // Distance to go heuristic
        auto vertex = _h->GetVertex(_target);
        double toGo = dist2go.at(vertex.first);

        return std::max(0.0,toGo);
      }
    );

    SSSPNeighborsFunction<HeuristicSearch> neighbors(
      [g](HeuristicSearch* _h, typename HeuristicSearch::vertex_descriptor _vid) {
        auto vertex = _h->GetVertex(_vid);
        auto gvid = vertex.first;
        auto timestep = vertex.second;
        
        auto vit = g->find_vertex(gvid);
        auto sourceProp = vit->property();

        for(auto eit = vit->begin(); eit != vit->end(); eit++) {
          auto target = eit->target();
          auto targetProp = g->find_vertex(target)->property();
          auto neighbor = std::make_pair(target,timestep+1);

          // Add a small cost for waiting so no edge has 0 weight
          auto nvid = _h->AddVertex(neighbor);
          auto dist = (targetProp - sourceProp).norm();
          _h->AddEdge(_vid,nvid,std::max(dist, 0.0000001));
        }
      }
    );

    std::vector<size_t> starts = {startVID};
    std::vector<size_t> goals = {goal};

    auto sssp = AStarSSSP(h.get(),starts,goals,weight,heuristic,neighbors,termination);

    // Check that a path was found
    const size_t last = sssp.ordering.back();
    if(h->GetVertex(last).first != goal or h->GetVertex(last).second < minEndTimestep) {
      if(this->m_debug) {
        std::cout << "Failed to find a path for " << robot->GetLabel() << std::endl;
      }
      pathCopy = nullptr;
      break;
    }

    // Reconstruct the path
    std::vector<size_t> path = {h->GetVertex(last).first};
    auto current = last;
    do {
      current = sssp.parent.at(current);
      path.push_back(h->GetVertex(current).first);
    } while(current != startVID);
    std::reverse(path.begin(),path.end());

    // Save path in solution
    _node.solutionMap[robot] = new vector<size_t>();
    *(_node.solutionMap[robot]) = path;
    pathCopy = _node.solutionMap[robot];

    hasReplanned.insert(robot);
  }

  return pathCopy != nullptr and pathCopy->size();
}


template <typename MPTraits>
void
CompositeDynamicRegionRRT<MPTraits>::
AddDependencies(std::set<Robot*>& _needsToReplan, Robot* _robot, const PBSNodeType& _node) {
  for(auto iter : _node.constraintMap) {
     auto r = iter.first;
     auto dep = iter.second;
     if(dep.count(_robot)) {
      if(!_needsToReplan.count(r)) {
        _needsToReplan.insert(r);
        AddDependencies(_needsToReplan, r, _node);
      }
     }
  }
}


template <typename MPTraits>
double
CompositeDynamicRegionRRT<MPTraits>::
CostFunction(CBSNodeType& _node) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CostFunction");

  // Treat an edge as having cost 1, use makespan
  double cost = 0;
  for(auto kv : _node.solutionMap) {
    cost = std::max(cost,double(kv.second->size()));
  }

  return cost;
}


template <typename MPTraits>
double
CompositeDynamicRegionRRT<MPTraits>::
CostFunction(PBSNodeType& _node) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CostFunction");

  // Treat an edge as having cost 1, use makespan
  double cost = 0;
  for(auto kv : _node.solutionMap) {
    cost = std::max(cost,double(kv.second->size()));
  }

  return cost;
}


template <typename MPTraits>
void
CompositeDynamicRegionRRT<MPTraits>::
ComputeMAPFPaths(const VID _vid) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::ComputeMAPFHPaths");
  m_mapfCount++;

  auto robots = this->GetGroupTask()->GetRobotGroup()->GetRobots();
  auto vertex = m_skeleton->GetVertex(_vid);
  for(auto robot : robots) {
    auto indVID = vertex.GetVID(robot);
    m_MAPFStarts[robot] = indVID;
  }

  // Configure CBS Functions
  std::unordered_map<Robot*, CBSSolution*> solution;

  if(m_mapf == "CBS") {
    CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution> lowLevel(
      [this](CBSNodeType& _node, Robot* _task) {
        return this->LowLevelPlanner(_node,_task);
      }
    );

    CBSValidationFunction<Robot,CBSConstraint,CBSSolution> validation(
      [this](CBSNodeType& _node) {
        return this->ValidationFunction(_node);
      }
    );

    CBSSplitNodeFunction<Robot,CBSConstraint,CBSSolution> splitNode(
      [this](CBSNodeType& _node, std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
            CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution>& _lowLevel,
            CBSCostFunction<Robot,CBSConstraint,CBSSolution>& _cost) {
        return this->SplitNodeFunction(_node,_constraints,_lowLevel,_cost);
      }
    );

    CBSCostFunction<Robot,CBSConstraint,CBSSolution> cost(
      [this](CBSNodeType& _node) {
        return this->CostFunction(_node);
      }
    );

    auto sol = CBS(robots,validation,splitNode,lowLevel,cost);
    solution = sol.solutionMap;
  } 
  else {
    CBSLowLevelPlanner<Robot,OrderingConstraint,CBSSolution> lowLevel(
      [this](PBSNodeType& _node, Robot* _task) {
        return this->LowLevelPlanner(_node,_task);
      }
    );

    CBSValidationFunction<Robot,OrderingConstraint,CBSSolution> validation(
      [this](PBSNodeType& _node) {
        return this->ValidationFunction(_node);
      }
    );

    CBSSplitNodeFunction<Robot,OrderingConstraint,CBSSolution> splitNode(
      [this](PBSNodeType& _node, std::vector<std::pair<Robot*,OrderingConstraint>> _constraints,
            CBSLowLevelPlanner<Robot,OrderingConstraint,CBSSolution>& _lowLevel,
            CBSCostFunction<Robot,OrderingConstraint,CBSSolution>& _cost) {
        return this->SplitNodeFunction(_node,_constraints,_lowLevel,_cost);
      }
    );

    CBSCostFunction<Robot,OrderingConstraint,CBSSolution> cost(
      [this](PBSNodeType& _node) {
        return this->CostFunction(_node);
      }
    );

    auto sol = CBS(robots,validation,splitNode,lowLevel,cost);
    solution = sol.solutionMap;
  }

  size_t maxTimestep = 0;
  for(auto kv : solution) {
    maxTimestep = std::max(maxTimestep,kv.second->size());
  }

  // Construct path through the composite skeleton
  auto lastVID = _vid;
  auto time = m_timestep;

  std::queue<SkeletonEdgeDescriptor> empty;
  std::swap(m_nextEdges, empty);

  for(size_t i = 1; i < maxTimestep; i++) {
    auto cv = CompositeSkeletonVertex(m_skeleton.get());
    auto edge = CompositeSkeletonEdge(m_skeleton.get(), 1.);

    auto lastVertex = m_skeleton->GetVertex(lastVID);

    for(auto robot : robots) {
      auto path = *solution[robot];
      auto tVID = i < path.size() ? path.at(i) : path.at(path.size() - 1);
      cv.SetRobotCfg(robot, tVID);

      auto sVID = lastVertex.GetVID(robot);

      WorkspaceSkeleton::CEI ei;
      WorkspaceSkeleton::CVI vi;
      auto eid = WorkspaceSkeleton::EID(sVID, tVID);
      m_individualSkeleton.find_edge(eid, vi, ei);
      eid = ei->descriptor();
      edge.SetEdge(robot, eid);
    }

    auto vid = m_skeleton->AddVertex(cv);
    if(lastVID == vid) {
      continue;
    }

    auto ed = m_skeleton->AddEdge(lastVID, vid, edge);
    auto vidTime = std::make_pair(vid, time + 1);
    m_predecessorVIDs[vidTime] = std::make_pair(lastVID, time);

    if(ed.id() == std::numeric_limits<size_t>::max()) {
      typename CompositeSkeletonType::CEI ei;
      typename CompositeSkeletonType::CVI vi;
      auto eid = typename CompositeSkeletonType::EID(lastVID, vid);
      m_skeleton->find_edge(eid, vi, ei);

      m_nextEdges.push(ei->descriptor());
    } else {
      m_nextEdges.push(ed);
    }

    lastVID = vid;
    time++;
  }

  if(this->m_debug) {
    std::cout << "Heuristic Paths" << std::endl;
    for(auto kv : solution) {
      auto robot = kv.first;
      auto path = *kv.second;
      std::cout << "\t" << robot->GetLabel() << ": ";
      for(auto vid : path) {
        std::cout << vid << ", ";
      }
      std::cout << std::endl;
    }
    std::cout << "Next edges size: " << m_nextEdges.size() << std::endl;
  }
}


template <typename MPTraits>
void
CompositeDynamicRegionRRT<MPTraits>::
NextBestRegion(const bool replan) {
  if(replan and m_replanOrder == "front") {
    if(this->m_debug)
      std::cout << "Replanning path from the front..." << std::endl;

    ComputeMAPFPaths(0);
  }

  if(replan and m_replanOrder == "back") {

    if(this->m_debug)
      std::cout << "Replanning with iterative backtracking" << std::endl;

    bool success = false;
    auto replanVID = m_region.edgeIterator->source();
    while(!success) {

      // Backtrack to the last viable vertex
      while(replanVID != INVALID_VID and m_replanAttempts[replanVID] >= m_maxEdges) {
        if(this->m_debug)
          std::cout << "VID " << replanVID 
                    << " exceeded max replan attempts, backtracking"
                    << std::endl;

        auto vidTime = std::make_pair(replanVID, m_timestep);
        vidTime = m_predecessorVIDs.at(vidTime);
        replanVID = vidTime.first;
        m_timestep = vidTime.second;
      }

      if(m_abortOnFail and replanVID == INVALID_VID)
        throw RunTimeException(WHERE) << "Ran out of new regions to create. Aborting...";

      // If failed at the start, stay here
      if(this->m_debug)
        std::cout << "Replanning path from VID "
                  << (replanVID == INVALID_VID ? 0 : replanVID)
                  << "..." << std::endl;

      if(replanVID == INVALID_VID)
        ComputeMAPFPaths(0);
      else
        ComputeMAPFPaths(replanVID);

      m_replanAttempts[replanVID]++;
      success = m_nextEdges.size() > 0;
      if(!success)
        m_replanAttempts[replanVID] = m_maxEdges;
    }
  }

  // Get the next edge from the queue
  auto ed = m_nextEdges.front();
  while(!m_nextEdges.empty() and m_traversedEdges.count(ed.id())) {
    m_nextEdges.pop();
    ed = m_nextEdges.front();
    m_timestep++;
  }

  if(m_nextEdges.empty()) {
    if(this->m_debug)
      std::cout << "Traversed all edges in the computed MAPF solution. Reverting to Composite RRT"
                << std::endl;
    return;
  }

  SkeletonVertexIterator vi;
  SkeletonEdgeIterator eit;
  m_skeleton->find_edge(ed, vi, eit);

  m_region = SamplingRegion(eit);

  // Delete this edge so it can't be chosen again
  m_nextEdges.pop();

  if(this->m_debug)
    std::cout << "Added new region on next edge from " << eit->source()
              << " to " << eit->target() << std::endl;
}


template <typename MPTraits>
void
CompositeDynamicRegionRRT<MPTraits>::
AdvanceRegion(const GroupCfgType& _cfg) {
  if(this->m_debug)
    std::cout << "Checking region for contact with new configuration "
              << _cfg.PrettyPrint() << "."
              << std::endl;

  // Advance this region until the robot at _cfg is no longer touching it.
  if(!AdvanceRegionToCompletion(_cfg, &m_region)) {
    return;
  }

  // We have reached the end of this region's edge. Delete it and save the
  // target vertex. to spawn a new region.
  auto eit = m_region.edgeIterator;
  m_traversedEdges.insert(eit->id());
  m_timestep++;
  NextBestRegion();
}


template <typename MPTraits>
bool
CompositeDynamicRegionRRT<MPTraits>::
AdvanceRegionToCompletion(const GroupCfgType& _cfg, SamplingRegion* _region) {
  // Find the edge path this region is traversing.
  auto eit = _region->edgeIterator;
  const auto path = eit->property().GetIntermediates();
  size_t& i = _region->edgeIndex;

  if(this->m_debug)
    std::cout << "\tChecking region at "
              << _region->GetCenter() << "."
              << "\n\t Region is at index " << i << " / " << path.size() - 1
              << std::endl;

  while(IsTouching(_cfg, *_region)) {

    // If there are no more points left on this edge, this region is completed.
    if(_region->LastPoint()) {
      if(this->m_debug)
        std::cout << "\t Region has reached the end of its "
                  << "path, erasing it now."
                  << std::endl;

      return true;
    }

    // Otherwise there are still points left; advance the region and index.
    _region->Advance();

    if(this->m_debug)
      std::cout << "\t Advancing region from index "
                << i - 1 << " to " << i << " / " << path.size() - 1 << "."
                << std::endl;
  }

  if(this->m_debug)
    std::cout << "\t Region is still traversing this edge." << std::endl;

  return false;
}

#endif
