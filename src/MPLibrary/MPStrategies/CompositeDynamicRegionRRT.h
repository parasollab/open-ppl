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
/// An RRT guided by a composite workspace skeleton.
///
/// Reference:
///   Coming soon
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////

template <typename MPTraits>
class CompositeDynamicRegionRRT : virtual public GroupRRTStrategy<MPTraits> {
  public:
    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;

    // Add robot group types
    typedef typename MPTraits::GroupCfgType     GroupCfgType;
    typedef typename MPTraits::GroupWeightType  GroupWeightType;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename GroupRoadmapType::VID      VID;
    
    typedef typename std::map<Robot*, VID>      VIDMap;

    typedef std::vector<Point3d>                PointSet;
    // typedef std::vector<Vector3d>               VectorSet;
    typedef std::map<Robot*, const Boundary*>   BoundaryMap;
    typedef std::map<Robot*, Vector3d>          VectorMap;
    // typedef std::map<Robot*, Point3d>           PointMap;
    typedef std::map<Robot*, std::vector<Robot*>> RobotMap;

    ///@}
    ///@name WorkspaceSkeleton Types
    ///@{

    typedef typename MPTraits::CompositeSkeletonVertex              CompositeSkeletonVertex;
    typedef typename MPTraits::CompositeSkeletonEdge                CompositeSkeletonEdge;
    typedef typename MPTraits::CompositeSkeletonType                CompositeSkeletonType;
    typedef typename CompositeSkeletonType::ED                      SkeletonEdgeDescriptor;
    typedef typename CompositeSkeletonType::adj_edge_iterator       SkeletonEdgeIterator;
    typedef typename CompositeSkeletonType::vertex_descriptor       SkeletonVertexDescriptor;
    typedef typename CompositeSkeletonType::vertex_iterator         SkeletonVertexIterator;

    ///@}
    ///@name CBS Types
    ///@{
    
    typedef GenericStateGraph<std::pair<size_t,size_t>,double> HeuristicSearch;
    typedef std::vector<size_t>                                CBSSolution;

    // ((vid, vid=MAX), time)
    typedef std::pair<std::pair<SkeletonVertexDescriptor, SkeletonVertexDescriptor>, size_t> CBSConstraint;
    typedef CBSNode<Robot,CBSConstraint,CBSSolution>           CBSNodeType;

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

      double cost{1.0};      ///< Cost based on the MAPF heuristic

      std::unordered_set<Robot*> activeRobots; ///< The robots that move on the edge.

      ///@}

      SamplingRegion(const SkeletonEdgeIterator& _eit, const double _cost) : 
            edgeIterator(_eit), cost(_cost) {
        auto ei = _eit;
        activeRobots = ei->property().GetActiveRobots();
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
        // const bool eit = edgeIterator == _region.edgeIterator;
        // const bool idx = edgeIndex == _region.edgeIndex;
        // const bool att = attempts == _region.attempts;
        // const bool succ = successes == _region.successes;
        // return eit and idx and att and succ;
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

    ///@}
    ///@name GroupRRTStrategy Overrides
    ///@{

    /// Get a random configuration to grow towards.
    // virtual CfgType SelectIndividualTarget(SamplingRegion* _region);
    virtual GroupCfgType SelectTarget() override;

    /// Add a new configuration to the roadmap and current tree.
    /// @param _newCfg The new configuration to add.
    /// @return A pair with the added VID and a bool indicating whether the new
    ///         node was already in the map.
    virtual std::pair<VID, bool> AddNode(const GroupCfgType& _newCfg) override;
    // virtual std::pair<VID, bool> AddNode(const CfgType& _newCfg) override;

    ///@}
    ///@name Helpers
    ///@{

    void InitializeCostToGo();

    /// Computes the sum of costs from a composite skeleton vertex to the 
    /// composite vertex closest to q_goal.
    double CompositeCostToGo(const CompositeSkeletonVertex _v);

    double ComputeMAPFHeuristic(const CompositeSkeletonVertex _vertex);

    double CostFunction(CBSNodeType& _node);

    std::vector<CBSNodeType> SplitNodeFunction(CBSNodeType& _node,
        std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
        CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution>& _lowLevel,
        CBSCostFunction<Robot,CBSConstraint,CBSSolution>& _cost);

    bool LowLevelPlanner(CBSNodeType& _node, Robot* _robot);

    std::vector<std::pair<Robot*,CBSConstraint>> ValidationFunction(CBSNodeType& _node);

    // Add the next best region to m_regions
    void NextBestRegion();

    /// Sample a configuration from within a sampling region using the sampler
    /// given in m_samplerLabel.
    /// @param _region The region to sample from.
    /// @return A configuration with the sampling region.
    std::pair<bool, GroupCfgType> Sample(SamplingRegion* _region);

    /// Sample a configuration from within a given boundary using the sampler
    /// given in _samplerLabel.
    /// @param _region The region to sample from.
    /// @return A configuration with the boundary.
    GroupCfgType Sample(const Boundary* const _boundary);

    /// Calculate the velocity bias along a region's skeleton edge.
    /// @param _region The region whose skeleton edge to bias the velocity along.
    /// @return The velocity bias.
    // const Vector3d GetVelocityBias(SamplingRegion* _region);

    /// Determine if a region is touching a configuration.
    /// @param _cfg The configuration.
    /// @param _region The sampling region.
    bool IsTouching(const GroupCfgType& _cfg, SamplingRegion& _region);

    /// Calculate the boundary around a sampling region.
    /// @param _v The center of the sampling region.
    /// @return The boundary with center _v and radius m_regionRadius.
    CSpaceBoundingSphere MakeBoundary(Robot* _robot, const VectorMap _v);

    ///@}
    ///@name Skeleton and Workspace
    ///@{

    /// Build topological skeleton
    void BuildSkeleton();

    /// Heuristic function for composite region selection.
    /// Calls Dijkstra's from goal vertex to find shortest path to all
    /// vertices in individual skeletons.
    double RegionSelectionHeuristic(const CompositeSkeletonType* _g,
                                    SkeletonVertexDescriptor _source,
                                    SkeletonVertexDescriptor _target);

    /// Select a region based on weighted success probabilities
    /// @return The sampling region to be expanded
    const size_t SelectSamplingRegion();

    /// Compute probabilities for selecting each sampling region.
    /// @return Probabilities based on extension success
    std::vector<double> ComputeProbabilities();

    // TODO:: (MUCH LATER - Make a group cfg version)
    /// Bias the velocity of a sample along a direction perscribed by
    /// the region.
    /// @param _cfg The sample to bias.
    /// @param _region The region from which _cfg was sampled.
    // void BiasVelocity(CfgType& _cfg, SamplingRegion* _region);

    /// Check if q_new is close enough to an unvisited skeleton vertex to create
    /// new regions on the outgoing edges of that vertex. If so, create those
    /// new regions.
    /// @param _p The new configuration added to the roadmap.
    void CheckRegionProximity(const PointSet& _p);

    /// Create new regions on the outgoing edges of the skeleton vertex.
    /// @param _iter The skeleton vertex iterator.
    /// @return The newly created sampling regions.
    std::vector<SamplingRegion>
    CreateRegions(const SkeletonVertexIterator _iter, const size_t _maxRegions);

    // TODO::Decide if we can instead pass a vid, otherwise make a group version
    /// Advance all sampling regions until they are no longer touching the
    /// newly added configuration.
    /// @param _cfg The newly added configuration, q_new.
    void AdvanceRegions(const GroupCfgType& _cfg);

    // TODO::Decide if we can instead pass a vid, otherwise make a group version
    /// Advance a region until it is either not longer touching a configuration
    /// or until it reaches the end of its respective skeleton edge.
    /// @param _cfg A configuration possibly touching the region.
    /// @param _region The region to advance along its skeleton edge.
    bool AdvanceRegionToCompletion(const GroupCfgType& _cfg, SamplingRegion* _region);

    RobotMap GetAdjacentRobots(const VectorMap _centers);

    ///@}
    ///@name Internal State
    ///@{

    // TODO::Add variables to store heuristic values
    std::map<Robot*, std::unordered_map<size_t,double>> m_distanceMap;

    // WorkspaceSkeleton m_originalSkeleton; ///< The original workspace skeleton.
    // WorkspaceSkeleton m_skeleton;         ///< The directed/pruned workspace skeleton.

    //TODO::Should make this a unique_ptr to avoid memory leaks
    // std::unique_ptr<CompositeSkeletonType> m_skeleton;
    CompositeSkeletonType* m_skeleton{nullptr};

    // Assume all individual skeletons are the same for now.
    WorkspaceSkeleton m_individualSkeleton;

    // The individual skeleton VIDs closest to the start and goal for each robot
    std::pair<std::vector<SkeletonVertexDescriptor>, 
              std::vector<SkeletonVertexDescriptor>> m_skeletonQuery;

    std::unordered_map<Robot*, SkeletonVertexDescriptor> m_MAPFStarts;

    // Skeleton clearance annotations
    std::map<Robot*, PropertyMap<std::vector<double>,double>*> m_annotationMap;

    std::string m_skeletonFilename;  ///< The output file for the skeleton graph
    std::string m_skeletonIO;        ///< Option to read or write the skeleton

    // TODO::Do we need to support different types for different robots? I'm leaving this for now.
    std::string m_skeletonType{"reeb"}; ///< Type of skeleton to build.
    std::string m_decompositionLabel; ///< The workspace decomposition label.
    std::string m_scuLabel;           ///< The skeleton clearance utility label.

    // bool m_velocityBiasing{false};    ///< Use velocity biasing?
    // double m_velocityAlignment{.1};   ///< Strength of velocity biasing.

    bool m_initialized{false};    ///< Have auxiliary structures been initialized?

    /// Pair of points we use to direct the skeleton.
    // std::pair<Point3d, Point3d> m_queryPair;
    // std::pair<PointSet, PointSet> m_queryPair;

    /// The set of active dynamic sampling regions and associated metadata.
    std::vector<SamplingRegion> m_regions;

    /// The current VIDs we are exploring
    std::unordered_map<size_t, size_t> m_backtrace; // map current to predecessor vid
    // SkeletonVertexDescriptor m_currentVID{SIZE_MAX};
    // std::vector<SkeletonVertexDescriptor> m_backtrace;

    // Stores the next best edges to add regions on
    std::multimap<double, SkeletonEdgeDescriptor> m_edgeQueue;

    /// The region that was last sampled from (we only advance sibling regions).
    int m_lastRegionIdx{-1};

    /// Check if regions should be expanded (not for goal extension).
    bool m_checkRegions{false};

    /// Keep track of which skeleton vertices we've visited.
    std::unordered_map<SkeletonVertexDescriptor, bool> m_visited;
    std::unordered_map<SkeletonVertexDescriptor, bool> m_explored;

    /// Cache heuristic costs for each edge
    std::map<VIDMap, double> m_costs; // TODO - for speedup, make this unordered and add a hash function

    std::unordered_map<SkeletonVertexDescriptor, std::multimap<double, SkeletonEdgeDescriptor>> m_bestEdges;
    size_t m_maxEdges{10};
    size_t m_maxSampleFails{20};
    size_t m_maxRegions{1};

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

    bool m_shortcutCD{false};

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

  // m_velocityBiasing = _node.Read("velocityBiasing", false, m_velocityBiasing,
  //     "Bias nonholonomic samples along the skeleton?");

  // m_velocityAlignment = _node.Read("velocityAlignment", false,
  //     m_velocityAlignment, -1., .99,
  //     "Minimum dot product for sampled velocity and biasing direction.");

  m_explore = _node.Read("explore", true, m_explore, 0., 1.,
      "Weight of explore vs. exploit in region selection probabilities");

  m_maxRegions = _node.Read("maxRegions", true, m_maxRegions, (size_t)1, 
      SIZE_MAX, "Maximum number of active regions at a time");

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

  m_shortcutCD = _node.Read("shortcutCD", false, true, "Shortcut collision detection?");
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

  // _os << "\tVelocity Biasing: " << m_velocityBiasing << std::endl;
  // _os << "\tVelocity Alignment: " << m_velocityAlignment << std::endl;

  _os << "\tMaximum Number of Regions: " << m_maxRegions << std::endl;
  _os << "\tMaximum Number of Edges per Vertex: " << m_maxEdges << std::endl;
  _os << "\tMaximum Number of Region Failures: " << m_maxSampleFails << std::endl;
  _os << "\tRegion Factor: " << m_regionFactor << std::endl;
  _os << "\tPenetration Factor: " << m_penetrationFactor << std::endl;
  _os << "\tExploration Factor: " << m_explore << std::endl;
  _os << "\tShortcut Collision Detection: " << m_shortcutCD << std::endl;
}

/*---------------------------- MPStrategy Overrides --------------------------*/

template <typename MPTraits>
void
CompositeDynamicRegionRRT<MPTraits>::Initialize(){
  GroupRRTStrategy<MPTraits>::Initialize();

  MethodTimer mt(this->GetStatClass(),
        this->GetNameAndLabel() + "::InitializeRoadmap");

  // Check that only one direction is being extended.
  if(this->m_numDirections > 1)
    throw RunTimeException(WHERE) << "Extending more than one direction "
                                  << "is not supported.";
  if(this->m_growGoals)
    throw RunTimeException(WHERE) << "Bidirectional growth is not supported.";

  if(this->m_goalDmLabel.empty())
    throw RunTimeException(WHERE) << "Goal distance metric label is required.";

  // Disable velocity biasing if the robot is holonomic.
  // m_velocityBiasing &= this->GetTask()->GetRobot()->IsNonholonomic();

  auto groupTask = this->GetGroupTask();

  for(auto& task : *groupTask) {
    auto robot = task.GetRobot();
    const double robotRadius = robot->GetMultiBody()->
                               GetBoundingSphereRadius() * m_regionFactor;
    m_regionRadius.insert(std::pair<Robot*, const double>(robot, robotRadius));
  }

  // Initialize the skeleton and regions.
  if(!m_initialized) {
    m_initialized = true;
    BuildSkeleton();

    // Get the individual skeleton vids closest to the start and goal
    std::vector<SkeletonVertexDescriptor> goalSkelVIDs;
    std::vector<SkeletonVertexDescriptor> startSkelVIDs;

    // Get the Points that correspond to the individual cfg goals.
    PointSet targets;
    auto groupTask = this->GetGroupTask();
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
    auto& startVIDs  = this->GetGoalTracker()->GetStartVIDs();
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
    CompositeSkeletonVertex vertex(m_skeleton);
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
    const auto compIter = m_skeleton->find_vertex(compVID);

    // Mark all nodes unvisited.
    m_visited.clear();
    for(auto vit = m_skeleton->begin(); vit != m_skeleton->end(); ++vit)
      m_visited[vit->descriptor()] = false;

    // Find the vertex nearest to start and create regions for each outgoing
    // edge.
    CreateRegions(compIter, m_maxRegions);
  }
}

/*------------------------ GroupRRTStrategy Overrides ------------------------*/

template <typename MPTraits>
typename MPTraits::GroupCfgType
CompositeDynamicRegionRRT<MPTraits>::
SelectTarget() {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::SelectTarget");

  auto grm = this->GetGroupRoadmap();
  GroupCfgType gcfg(grm);

  // Iterate through all regions to see if max sample fails has been exceeded.
  std::vector<SkeletonVertexDescriptor> replaceVertices;
  for(auto iter = m_regions.begin(); iter != m_regions.end(); ) {
    auto eit = iter->edgeIterator;
    auto fails = iter->attempts - iter->successes;

    if(fails > m_maxSampleFails) {
      if(this->m_debug)
        std::cout << "Exceeded maximum number of failures. Deleting region on edge from "
                  << eit->source() << " to "
                  << eit->target() << std::endl;

      iter = m_regions.erase(iter);
      replaceVertices.push_back(eit->source());
    } else 
      ++iter;
  }

  // Construct replacement regions for any that failed too many times.
  for(auto v : replaceVertices) {
    if(this->m_debug)
      std::cout << "Replacing region for target VID " << v << std::endl;
    NextBestRegion();
  }

  bool notFound = true;
  while(notFound) {
    notFound = false;
    // Select a region for sample generation (null indicates sample from whole env.)
    size_t regionIdx = SelectSamplingRegion();

    if(regionIdx > m_regions.size() - 1) {
      m_lastRegionIdx = -1;
      gcfg = Sample(this->GetEnvironment()->GetBoundary());
    } else {
      // Check if a valid cfg was found
      m_lastRegionIdx = regionIdx;
      auto result = Sample(&m_regions[regionIdx]);
      notFound = not result.first;
      if(notFound)
        continue;
      gcfg = result.second;
    }

    if(m_lastRegionIdx > -1)
      m_regions[m_lastRegionIdx].IncrementAttempts();
  }

  m_checkRegions = true;

  return gcfg;
}

template <typename MPTraits>
std::pair<typename CompositeDynamicRegionRRT<MPTraits>::VID, bool>
CompositeDynamicRegionRRT<MPTraits>::
AddNode(const GroupCfgType& _newCfg) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::AddNode");

  auto g = this->GetGroupRoadmap();

  const VID lastVID = g->GetLastVID();
  const VID newVID  = g->AddVertex(_newCfg);

  const bool nodeIsNew = lastVID != g->GetLastVID();
  if(nodeIsNew) {
    if(this->m_debug)
      std::cout << "\tAdding VID " << newVID << "."
                << std::endl;
  }

  if(nodeIsNew and m_checkRegions) {
    // Already checking for this node, don't need to do it again
    m_checkRegions = false;

    // Increment the success rate if we were able to successfully generate a node.
    if(m_lastRegionIdx > -1)
      m_regions.at(m_lastRegionIdx).IncrementSuccess();

    // On each new sample, check if we need to advance our region and generate
    // new ones.
    auto vi = g->find_vertex(newVID);

    PointSet p;
    auto compState = vi->property();
    for(auto r : compState.GetRobots()) {
      p.push_back(compState.GetRobotCfg(r).GetPoint());
    }

    // CheckRegionProximity(p); // don't do this if we only want one region

    if(m_lastRegionIdx > -1)
      AdvanceRegions(compState);
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
    bounds.push_back(MakeBoundary(robot, center));
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

  if(m_shortcutCD) {
    // Get the map of adjacent robots
    const auto adjMap = GetAdjacentRobots(center);
    while(tries < 10 and samples.empty()) {
      ++tries;
      s->Sample(1, 100, boundMap, std::back_inserter(samples),
        std::back_inserter(collision), adjMap);
    }
  } else {
    while(tries < 10 and samples.empty()) {
      ++tries;
      s->Sample(1, 100, boundMap, std::back_inserter(samples),
        std::back_inserter(collision));
    }
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

  // if(this->m_debug)
  //   std::cout << "\t Touch test: " << (touching ? "passed" : "failed")
  //             << "\n\t  Bounding sphere: " << robotCenter << " ; " << robotRadius
  //             << "\n\t  Region:          " << _region.GetCenter() << " ; "
  //             << m_regionRadius
  //             << "\n\t  Bounding sphere penetrates by "
  //             << std::setprecision(4)
  //             << penetration << (touching ? " >= " : " < ") << threshold
  //             << " units."
  //             << std::endl;

  return touching;
}


template <typename MPTraits>
CSpaceBoundingSphere
CompositeDynamicRegionRRT<MPTraits>::
MakeBoundary(Robot* _robot, const VectorMap _v) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::MakeBoundary");

  const bool threeD = _robot->GetMultiBody()->GetBaseType()
                   == Body::Type::Volumetric;

  auto indV = _v.at(_robot);

  // I'm not sure what the boundary code might do with a negative radius. Bound
  // it below at zero just in case.
  const double radius = std::max(0., m_regionRadius.at(_robot));

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
        m_individualSkeleton.DoubleEdges();
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
        m_individualSkeleton.DoubleEdges();
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
      m_individualSkeleton.DoubleEdges();
    }
  }

  // Get the clearance annotations
  auto annot = ClearanceAnnotatedSkeleton(this, &m_individualSkeleton, false);

  std::vector<WorkspaceSkeleton*> ws;
  for (size_t i = 0; i < robots.size(); ++i) {
    ws.push_back(&m_individualSkeleton);
    m_annotationMap.insert(std::make_pair(robots[i], annot));
    if(this->m_debug)
      std::cout << "Added workspace skeleton for " << robots.at(i)->GetLabel() << "." << endl;
  }
  m_skeleton = new CompositeSkeletonType(this->GetGroupTask()->GetRobotGroup(), ws);

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
              << (index != (int)m_regions.size() ? "." : " (whole env).")
              << std::endl;
  }

  return index;
}


template <typename MPTraits>
std::vector<double>
CompositeDynamicRegionRRT<MPTraits>::
ComputeProbabilities() {

  // sum of costs

  // std::vector<double> sumOfCosts;
  // double max_c = 0.0;
  // for(auto r : m_regions) {
  //   const auto target = r.edgeIterator->target();
  //   const auto vi = m_skeleton->find_vertex(target);
  //   const auto compState = vi->property();

  //   double c = 0.0;
  //   for(auto robot : m_skeleton->GetGroup()->GetRobots())
  //     c += m_distanceMap.at(robot).at(compState.GetVID(robot));
  //   sumOfCosts.push_back(c);

  //   if(c > max_c)
  //     max_c = c;
  // }

  // std::vector<double> closeness;
  // for(size_t i = 0; i < m_regions.size(); ++i) {
  //   closeness.push_back(max_c - sumOfCosts.at(i) + 0.001); // avoid 0
  // }

  // Get the closeness of all current regions and the makespans.
  // std::vector<double> makespans;
  // double maxMakespan = 0.;
  // for(auto r : m_regions) {
  //   const auto target = r.edgeIterator->target();
  //   const auto vi = m_skeleton->find_vertex(target);
  //   const auto compState = vi->property();

  //   double maxDist = 0.;
  //   for(auto robot : m_skeleton->GetGroup()->GetRobots()) {
  //     double dist = m_distanceMap.at(robot).at(compState.GetVID(robot));
  //     if(dist > maxDist)
  //       maxDist = dist;
  //   }

  //   if(maxDist > maxMakespan)
  //     maxMakespan = maxDist;
  //   makespans.push_back(maxDist);
  // }

  // Find the maximum cost
  double maxMakespan = 0.;
  for(auto r : m_regions) {
    if(r.cost > maxMakespan)
      maxMakespan = r.cost;
  }

  // Get the exponenetial inverse makespan (closeness) for each region
  std::vector<double> closeness;
  double totalWeight = 0.;
  for(size_t i = 0; i < m_regions.size(); ++i) {
    closeness.push_back(exp(maxMakespan - m_regions.at(i).cost));
    totalWeight += m_regions.at(i).GetWeight() * closeness.at(i);
  }

  // Compute the probabilities for the current regions.
  std::vector<double> probabilities;
  probabilities.reserve(m_regions.size() + 1);

  // const double explore = m_explore / (m_regions.size() + 1);
  const double explore = m_explore;

  for(size_t i = 0; i < m_regions.size(); ++i) {
    const double exploit = (1 - m_explore) * m_regions.at(i).GetWeight() * closeness.at(i) / totalWeight;

    // probabilities.emplace_back(exploit + explore);
    probabilities.emplace_back(exploit);
  }

  // Get the probability for the whole environment.
  probabilities.emplace_back(explore);

  return probabilities;
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

    // const auto vi = ws->FindNearestVertex(_targets.at(i));

    std::vector<WorkspaceSkeleton::vertex_descriptor> vds = {m_skeletonQuery.second.at(i)};
    auto robot = m_skeleton->GetGroup()->GetRobot(i);
    auto output = DijkstraSSSP(ws, vds, weight, termination);
    m_distanceMap.insert(std::pair<Robot*, std::unordered_map<size_t, double>>(robot, output.distance));
  }

  // Find the max dist and compute (max - each dist) for closeness score
  // double maxDist = 0;
  // for(auto r : m_skeleton->GetGroup()->GetRobots()) {
  //   const auto distMap = m_distanceMap.at(r);

  //   for(auto d: distMap) {
  //     if(d.second > maxDist)
  //       maxDist = d.second;
  //   }
  // }

  // for(auto r : m_skeleton->GetGroup()->GetRobots()) {
  //   auto distMap = m_distanceMap.at(r);

  //   for(auto d: distMap) {
  //     // Add small value so we never get zero probability of selecting a region
  //     distMap[d.first] = maxDist - d.second + 1e-7;
  //   }
  // }
}


template <typename MPTraits>
double
CompositeDynamicRegionRRT<MPTraits>::
CompositeCostToGo(const CompositeSkeletonVertex _v) {
  double cost = 0.0;
  for(size_t i = 0; i < _v.GetNumRobots(); ++i) {
    auto vid = _v.GetVID(i);
    cost += m_distanceMap.at(_v.GetRobot(i)).at(vid);
  }
  return cost;
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
    for(auto iter = _node.solutionMap.begin(); iter != _node.solutionMap.end(); iter++) {

      // Find the VID of this robot at time i
      auto robot = iter->first;
      auto path = *(iter->second);

      auto s = std::min(i, path.size()-1);
      auto source = path[s];
      auto t = std::min(i+1,path.size()-1);
      auto target = path[t];

      // order such that source < target
      auto sVID = std::min(source, target);
      auto tVID = std::max(source, target);
      auto edgePair = std::make_pair(sVID, tVID);

      // Check for an edge
      if(source != target) {
        // Check the capacity of the vertex
        if(edgeCapacity.at(edgePair).at(i) < 0) {
          constraints.push_back(std::make_pair(robot, std::make_pair(edgePair, i)));
        }
      }

      // Check the capacity of the source vertex
      if(vertexCapacity.at(source).at(i) < 0) {
        auto vPair = std::make_pair(source, SIZE_MAX);
        constraints.push_back(std::make_pair(robot, std::make_pair(vPair, i)));
      }
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
          if (timestep == constraint.second) {
            return std::numeric_limits<double>::infinity();
          }
        }

        // Ignore any other edges
        if(constraint.first.second != SIZE_MAX)
          continue;

        // Check for vertex constraint
        if(source != constraint.first.first)
          continue;
    
        if(timestep == constraint.second)
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

      for(auto eit = vit->begin(); eit != vit->end(); eit++) {
        auto target = eit->target();
        auto neighbor = std::make_pair(target,timestep+1);
        // auto edge = eit->property();

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
double
CompositeDynamicRegionRRT<MPTraits>::
CostFunction(CBSNodeType& _node) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CostFunction");

  // For now, treat an edge as having cost 1, use makespan

  double cost = 0;
  for(auto kv : _node.solutionMap) {
    cost = std::max(cost,double(kv.second->size()));
    // double pathLength = 0;
    // const auto& path = *kv.second;
    // for(size_t i = 1; i < path.size(); i++) {
    //   auto source = path[i-1];
    //   auto target = path[i];
    //   auto edge = g->GetEdge(source,target);
    //   pathLength += edge;
    // }
    // //cost += pathLength;
    // cost = std::max(cost,pathLength);
  }

  return cost;
}


template <typename MPTraits>
double
CompositeDynamicRegionRRT<MPTraits>::
ComputeMAPFHeuristic(const CompositeSkeletonVertex _vertex) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::ComputeMAPFHeuristic");

  for(auto robot : m_skeleton->GetGroup()->GetRobots()) {
    auto indVID = _vertex.GetVID(robot);
    m_MAPFStarts[robot] = indVID;
  }

  // Configure CBS Functions
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

  CBSCostFunction<Robot,CBSConstraint,CBSSolution> cost(
    [this](CBSNodeType& _node) {
      return this->CostFunction(_node);
    }
  );

  CBSSplitNodeFunction<Robot,CBSConstraint,CBSSolution> splitNode(
    [this](CBSNodeType& _node, std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
           CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution>& _lowLevel,
           CBSCostFunction<Robot,CBSConstraint,CBSSolution>& _cost) {
      return this->SplitNodeFunction(_node,_constraints,_lowLevel,_cost);
    }
  );

  std::vector<Robot*> robots = m_skeleton->GetGroup()->GetRobots();
  CBSNodeType solution = CBS(robots,validation,splitNode,lowLevel,cost);

  // Cache results to reduce calls to CBS.
  std::vector<VIDMap> vertices;
  vertices.resize(int(solution.cost));
  for(auto kv : solution.solutionMap) {
    auto robot = kv.first;
    auto path = *kv.second;
    for(size_t i = 0; i < path.size(); i++)
      vertices.at(i).emplace(std::make_pair(robot, path.at(i)));

    for(size_t i = path.size(); i < (size_t) solution.cost; i++)
      vertices.at(i).emplace(std::make_pair(robot, path.at(path.size() - 1)));
  }

  double subcost = solution.cost;
  for(auto vset : vertices) {
    // Stop caching if we've reached a previously discovered path.
    if(m_costs.find(vset) != m_costs.end())
      break;

    m_costs.emplace(std::make_pair(vset, subcost));
    subcost = subcost - 1.0;
  }

  if(this->m_debug) {
    std::cout << "Heuristic Paths" << std::endl;
    for(auto kv : solution.solutionMap) {
      auto robot = kv.first;
      auto path = *kv.second;
      std::cout << "\t" << robot->GetLabel() << ": ";
      for(auto vid : path) {
        std::cout << vid << ", ";
      }
      std::cout << std::endl;
    }
    std::cout << "Heuristic Cost: " << solution.cost << std::endl;
  }

  return solution.cost;
}


template <typename MPTraits>
void
CompositeDynamicRegionRRT<MPTraits>::
CheckRegionProximity(const PointSet& _p) {
  // TODO this isn't used. Should it be?

  // Check each skeleton node to see if a new region should be created.
  const auto vids = m_skeleton->GetAllVIDs();
  for(auto v : vids) {
    const auto iter = m_skeleton->find_vertex(v);

    // Create regions if an individual point is within all individual regions.
    bool reached = true;
    const auto robots = m_skeleton->GetGroup()->GetRobots();
    for(size_t i = 0; i < robots.size(); ++i) {
      const double indDist = (iter->property().GetRobotCfg(robots[i]) - _p[i]).norm();

      reached = reached and (indDist < m_regionRadius.at(robots[i]));

      if(!reached)
        break;
    }

    if(!reached)
      continue;

    CreateRegions(iter, m_maxRegions);
  }
}


template <typename MPTraits>
std::vector<typename CompositeDynamicRegionRRT<MPTraits>::SamplingRegion>
CompositeDynamicRegionRRT<MPTraits>::
CreateRegions(const SkeletonVertexIterator _iter, const size_t _maxRegions) {
  MethodTimer mt(this->GetStatClass(),
        this->GetNameAndLabel() + "::CreateRegions");

  const auto descriptor = _iter->descriptor();
  // Skip skeleton nodes that are already visited.
  if(m_visited.find(descriptor) != m_visited.end() and m_visited.at(descriptor))
    return {};
  m_visited[descriptor] = true;

  // Add new vertices and edges to the composite skeleton
  const auto cwr = _iter->property();
  // const auto origCost = CompositeCostToGo(_iter->property());

  // Collect neighbor sets
  std::vector<std::vector<size_t>> neighborSets;
  neighborSets.push_back({});

  for(size_t i = 0; i < m_skeleton->GetNumRobots(); i++) {
    std::vector<std::vector<size_t>> newNeighbors;

    auto region = cwr.GetVID(i);
    auto vit = m_skeleton->GetIndividualGraph(i)->find_vertex(region);

    // Add edge self edge at this individual vertex -- get rid of this, just add vid to set
    std::vector<Point3d> p = {};
    if(!m_skeleton->GetIndividualGraph(i)->IsEdge(region, region)) {
      m_skeleton->GetIndividualGraph(i)->AddEdge(region, region, p);
    }

    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      auto target = eit->target();

      for(auto set : neighborSets) {
        set.push_back(target);
        newNeighbors.push_back(set);
      }
    }

    neighborSets = newNeighbors;
  }

  // Create composite vertices from neighbor sets and add to skeleton.
  for(const auto& set : neighborSets) {
    // Create composite vertex.
    CompositeSkeletonVertex vertex(m_skeleton);

    for(size_t i = 0; i < m_skeleton->GetNumRobots(); i++) {
      vertex.SetRobotCfg(i,set[i]);
    }

    // Build edge.
    // TODO::Develop actual weight function.
    const double weight = 1.;
    CompositeSkeletonEdge edge(m_skeleton,weight);

    // Map target vertices and edges to remaining clearance (VID1 < VID2)
    std::map<std::pair<VID, VID>, double> edgeClearance;
    std::map<VID, double> vertexClearance;
    bool noRoom = false;

    // Set the individual edge descriptors.
    for(size_t i = 0; i < m_skeleton->GetNumRobots(); i++) {
      auto sVID = cwr.GetVID(i);
      auto tVID = set[i];

      WorkspaceSkeleton::CEI ei;
      WorkspaceSkeleton::CVI vi;
      auto eid = WorkspaceSkeleton::EID(sVID, tVID);
      m_skeleton->GetIndividualGraph(i)->find_edge(eid, vi, ei);
      eid = ei->descriptor();
      edge.SetEdge(m_skeleton->GetGroup()->GetRobot(i), eid);

      // Check the vertex clearance at the target
      auto viter = vertexClearance.find(tVID);
      if(viter == vertexClearance.end()) {
        auto d = m_annotationMap.at(m_skeleton->GetGroup()->GetRobot(i))
                 ->GetVertexProperty(tVID);
        d -= m_skeleton->GetGroup()->GetRobot(i)->GetMultiBody()
             ->GetBoundingSphereRadius();
        vertexClearance.insert(std::make_pair(tVID, d));

        if(d < 0) {
          noRoom = true;
          break;
        }
      } else {
        viter->second -= m_skeleton->GetGroup()->GetRobot(i)->GetMultiBody()
                         ->GetBoundingSphereRadius();

        if(viter->second < 0) {
          noRoom = true;
          break;
        }
      }

      // Skip checking edge if the robot is waiting
      if(sVID == tVID) {
        continue;
      }

      // Check the skeleton edge annotations for clearance
      if(sVID > tVID) {
        auto temp = sVID;
        sVID = tVID;
        tVID = temp;
      }

      auto iter = edgeClearance.find(std::make_pair(sVID, tVID));
      if(iter == edgeClearance.end()) {
        // Find the minimum clearance along the edge

        auto dists = m_annotationMap.at(m_skeleton->GetGroup()->GetRobot(i))
                     ->GetEdgeProperty(eid);
        double minDist = std::numeric_limits<double>::max();
        for(auto d : dists) {
          if(d < minDist)
            minDist = d;
        }

        // The remaining clearance is the min clearance minus robot size
        minDist -= m_skeleton->GetGroup()->GetRobot(i)->GetMultiBody()->
                   GetBoundingSphereRadius();
        edgeClearance.insert(std::make_pair(std::make_pair(sVID, tVID), minDist));

        if(minDist < 0) {
          noRoom = true;
          break;
        }
      } else {
        iter->second -= m_skeleton->GetGroup()->GetRobot(i)->GetMultiBody()->
                        GetBoundingSphereRadius() * 2;

        if(iter->second < 0) {
          noRoom = true;
          break;
        }
      }
    }

    // Skip edges that are too small or don't have any active robots.
    if(noRoom or edge.GetActiveRobots().size() < 1)
      continue;

    // Add composite vertex to the skeleton.
    auto vid = m_skeleton->AddVertex(vertex);

    auto eid = m_skeleton->AddEdge(descriptor,vid,edge);

    // Check if the heuristic cost has been cached, otherwise calculate it.
    VIDMap vertices;
    for(auto r : vertex.GetRobots()) {
      vertices.emplace(std::make_pair(r, vertex.GetVID(r)));
    }

    double cost;
    if(m_costs.find(vertices) == m_costs.end()) {
      cost = ComputeMAPFHeuristic(vertex);
      
      if(this->m_debug)
        std::cout << "Cache miss for " << vertex.GetVIDs() 
                  << ". Computed cost " << cost << std::endl;
    }
    else {
      cost = m_costs.at(vertices);
      
      if(this->m_debug)
        std::cout << "Cache hit for " << vertex.GetVIDs() 
                  << " with cost " << cost << std::endl;
    }

    m_bestEdges[descriptor].emplace(cost, eid);
  }

  // Keep only the best edges
  size_t idx = 0;
  for(auto mIter = m_bestEdges.at(descriptor).cbegin(); mIter != m_bestEdges[descriptor].cend(); ) {
    ++idx;
    if(idx > m_maxEdges) {
      // Delete edges that are not the lowest cost
      mIter = m_bestEdges.at(descriptor).erase(mIter);
    } else {
      // Add best edges to the next region queue
      auto eid = mIter->second;
      auto edgeCost = mIter->first;
      m_edgeQueue.emplace(std::make_pair(edgeCost, eid));
      ++mIter;
    }
  }

  // Save the set of created regions to return.
  std::vector<SamplingRegion> newRegions;

  // Indicate which is the start vid.
  if(m_backtrace.size() < 1)
    m_backtrace.insert(std::make_pair(descriptor, SIZE_MAX));

  // Add the predecessor of each target vid.
  for(auto mIter = m_bestEdges.at(descriptor).cbegin(); mIter != m_bestEdges.at(descriptor).cend(); ++mIter) {
    m_backtrace.insert(std::make_pair(mIter->second.target(), descriptor));
  }

  // Create as many regions as possible without exceeding _maxRegions
  for(auto mIter = m_bestEdges.at(descriptor).begin(); mIter != m_bestEdges.at(descriptor).end(); ) {

    if(newRegions.size() >= _maxRegions)
      break;

     // Create a new region for this skeleton node on the best edge.
    SkeletonVertexIterator vi;
    SkeletonEdgeIterator eit;
    m_skeleton->find_edge(mIter->second, vi, eit);

    m_regions.push_back(SamplingRegion(eit, mIter->first));
    newRegions.push_back(m_regions.at(m_regions.size() - 1));

    if(this->m_debug)
      std::cout << "Created new region"
                << " on edge (" << eit->source() << ", "
                << eit->target() << ", " << eit->id() << ") "
                << "with cost " << mIter->first
                << "."
                << std::endl;

    mIter = m_bestEdges.at(descriptor).erase(mIter);
  }

  return newRegions;
}


template <typename MPTraits>
void
CompositeDynamicRegionRRT<MPTraits>::
NextBestRegion() {
  // If there are no remaining edges, then throw exception
  if(m_edgeQueue.size() < 1)
    throw RunTimeException(WHERE) << "Ran out of new regions to create. Aborting";

  // Get the next lowest cost edge from the queue
  auto eidIter = m_edgeQueue.begin();
  SkeletonVertexIterator vi;
  SkeletonEdgeIterator eit;
  m_skeleton->find_edge(eidIter->second, vi, eit);

  auto cost = eidIter->first;
  m_regions.push_back(SamplingRegion(eit, eidIter->first));

  // Delete this edge in the map so it can't be chosen again
  m_edgeQueue.erase(eidIter);

  if(this->m_debug)
    std::cout << "Added new region on next best edge from " << eit->source()
              << " to " << eit->target() << " with cost " << cost << std::endl;
}


template <typename MPTraits>
void
CompositeDynamicRegionRRT<MPTraits>::
AdvanceRegions(const GroupCfgType& _cfg) {
  if(this->m_debug)
    std::cout << "Checking sibling regions of " << m_lastRegionIdx
              << " for contact with new configuration "
              << _cfg.PrettyPrint() << "."
              << std::endl;

  // Keep track of any newly reached vertices to spawn regions on their outbound
  // edges.
  std::queue<SkeletonVertexDescriptor> newlyReachedVertices;

  // Keep track of edge iterators that need to be replaced
  std::queue<SkeletonVertexDescriptor> replaceVertices;

  // Only check sibling regions to the one we just advanced
  SkeletonVertexDescriptor sourceVID = m_regions[m_lastRegionIdx].edgeIterator->source();

  // Iterate through all existing regions to see which should be advanced.
  for(auto iter = m_regions.begin(); iter != m_regions.end(); ) {

    if(iter->edgeIterator->source() != sourceVID) {
      ++iter;
      continue;
    }

    // Advance this region until the robot at _cfg is no longer touching it.
    if(!AdvanceRegionToCompletion(_cfg, &(*iter))) {
      ++iter;
      continue;
    }

    // We have reached the end of this region's edge. Delete it and save the
    // target vertex. to spawn new regions.
    auto eit = iter->edgeIterator;
    const auto target = eit->target();
    iter = m_regions.erase(iter);

    if(m_visited.find(target) == m_visited.end() or !m_visited.at(target))
      newlyReachedVertices.push(target);
    else
      replaceVertices.push(eit->source());
  }

  while(!replaceVertices.empty()) {
    replaceVertices.pop();
    NextBestRegion();
  }

  // TODO advance new regions
  while(!newlyReachedVertices.empty()) {
    // Pop the next vertex off the queue.
    SkeletonVertexDescriptor targetVD = newlyReachedVertices.front();
    SkeletonVertexIterator target = m_skeleton->find_vertex(targetVD);
    newlyReachedVertices.pop();

    // Create regions at this vertex.
    const size_t numRegions = (size_t)std::max((int)m_maxRegions - (int)m_regions.size(), 0);
    std::vector<SamplingRegion> newRegions = CreateRegions(target, numRegions);

    // If there weren't any new regions created, replace it with the next best
    if(newRegions.size() < 1 and numRegions > 0)
      NextBestRegion();
  }

  // TODO come back to this later
  // Create new regions for each newly reached vertex.
  // while(!newlyReachedVertices.empty()) {
  //   // Pop the next vertex off the queue.
  //   SkeletonVertexDescriptor targetVD = newlyReachedVertices.front();
  //   SkeletonVertexIterator target = m_skeleton->find_vertex(targetVD);
  //   newlyReachedVertices.pop();

  //   // Create regions at this vertex.
  //   std::vector<SamplingRegion> newRegions = CreateRegions(target);

  //   if(newRegions.size() < 1)
  //     NextBestRegion(eit); // TODO this only works in special cases

  //   // Advance each new region.
  //   for(auto region : newRegions) {

  //     // Advance this region until the robot at _cfg is no longer touching it.
  //     if(!AdvanceRegionToCompletion(_cfg, &region))
  //       continue;

  //     // We have reached the end of this region's edge. Delete it and save the
  //     // target vertex. to spawn new regions.
  //     auto iter = find(m_regions.begin(), m_regions.end(), region);
  //     auto eit = region.edgeIterator;
  //     newlyReachedVertices.push(eit->target());
  //     m_regions.erase(iter);
  //   }
  // }
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
                  << "path, erasing it now. " << m_regions.size() - 1
                  << " regions remain."
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


template <typename MPTraits>
typename CompositeDynamicRegionRRT<MPTraits>::RobotMap
CompositeDynamicRegionRRT<MPTraits>::
GetAdjacentRobots(const VectorMap _centers) {
  auto robots = this->GetGroupTask()->GetRobotGroup()->GetRobots();

  RobotMap robotMap;

  for(auto robot : robots) {
    robotMap.emplace(std::make_pair(robot, std::vector<Robot*>()));
    for(auto otherRobot : robots) {
      if(robot == otherRobot)
        continue;

      // Get the distances between individual region centers
      double dist = (_centers.at(robot) - _centers.at(otherRobot)).norm();

      if(dist < (m_regionRadius.at(robot) + m_regionRadius.at(otherRobot))) {
        robotMap.at(robot).push_back(otherRobot);
      }
    }
  }

  return robotMap;
}

#endif
