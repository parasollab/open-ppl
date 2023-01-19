#ifndef PPL_WORKSPACE_DASH_H_
#define PPL_WORKSPACE_DASH_H_

#include "TMPStrategyMethod.h"

#include "TMPLibrary/ActionSpace/Condition.h"

#include "Traits/CfgTraits.h"

#include "Workspace/WorkspaceSkeleton.h"
#include "Workspace/HypergraphWorkspaceSkeleton.h"

class Action;
class Interaction;

class WoDaSH : public TMPStrategyMethod {
  public:
    ///@name Local Types
    ///@{

    typedef Cfg                                       CfgType;
    typedef typename TMPBaseObject::GroupCfgType      GroupCfgType;
    typedef typename TMPBaseObject::GroupRoadmapType  GroupRoadmapType;
    typedef typename MPTraits<CfgType>::GroupPathType GroupPathType;
    typedef size_t                                    VID;
    
    typedef typename std::map<Robot*, VID>      VIDMap;
    typedef std::vector<Point3d>                PointSet;
    typedef std::map<Robot*, const Boundary*>   BoundaryMap;
    typedef std::map<Robot*, Vector3d>          VectorMap;

    typedef Condition::State                 State;
    typedef std::set<std::string>            RoleSet;

    ///@}
    ///@name Skeleton Types
    ///@{

    typedef typename WorkspaceSkeleton::EI                SkeletonEdgeIterator;
    typedef typename WorkspaceSkeleton::ED                SkeletonEdgeDescriptor;

    typedef CompositeState<WorkspaceSkeleton>  CompositeSkeletonVertex;
    typedef CompositeEdge<WorkspaceSkeleton>   CompositeSkeletonEdge;
    typedef HypergraphWorkspaceSkeleton<CompositeSkeletonVertex, 
            CompositeSkeletonEdge> HypergraphSkeletonType;
    typedef size_t HID;

    ///@}
    ///@name CBS Types
    ///@{

    typedef GenericStateGraph<std::pair<size_t,size_t>,double> HeuristicSearch;
    typedef std::vector<size_t>                                CBSSolution;

    // ((vid, vid=INVAILD_VID), time)
    typedef std::pair<std::pair<VID, VID>, size_t>             CBSConstraint;
    typedef CBSNode<Robot,CBSConstraint,CBSSolution>           CBSNodeType;


    ///@} 
    ///@name Construction
    ///@{

    WoDaSH();

    WoDaSH(XMLNode& _node);

    ~WoDaSH();

    ///@}
    ///@name Overrides
    ///@{

    virtual void Initialize() override;
	
    virtual void PlanTasks() override; 

    ///@}

  private: 
    ///@name Helper Functions
    ///@{

    /// Build the individual topological skeleton
    void BuildSkeleton();

    void InitializeCostToGo();

    std::vector<std::pair<Robot*, CBSConstraint>> ValidationFunction(CBSNodeType& _node);

    std::vector<CBSNodeType> SplitNodeFunction(CBSNodeType& _node, 
            std::vector<std::pair<Robot*, CBSConstraint>> _constraints,
            CBSLowLevelPlanner<Robot, CBSConstraint, CBSSolution>& _lowLevel,
            CBSCostFunction<Robot, CBSConstraint, CBSSolution>& _cost);
    
    bool LowLevelPlanner(CBSNodeType& _node, Robot* _robot);

    double CostFunction(CBSNodeType& _node);

    /// Generate a set of paths through the implicit skeleton using CBS
    std::unordered_map<Robot*, CBSSolution*> MAPFSolution();

    // Convert MAPF solution to hyperskeleton hyperpath
    void ConstructHyperpath(std::unordered_map<Robot*, CBSSolution*> _mapfSolution);

    bool GroundHyperskeleton();

    bool SampleTrajectories();

    VID SpawnVertex(GroupRoadmapType* _grm, CompositeSkeletonEdge _edge);

    VID SpawnVertex(GroupRoadmapType* _grm, CompositeSkeletonVertex _vertex);

    CSpaceBoundingSphere MakeBoundary(Robot* _robot, const Point3d _indV);

    bool FinishedEdge(CompositeSkeletonEdge _edge, const GroupCfgType& _groupCfg);

    std::vector<CompositeSkeletonVertex>
    ComputeIntermediates(const CompositeSkeletonVertex _source, 
                        const CompositeSkeletonVertex _target,
                        CompositeSkeletonEdge _edge,
                        const bool pushStart=false,
                        const bool pushTarget=false);

    void ConnectToSkeleton();

    RobotGroup* AddGroup(std::vector<Robot*> _robots);

    HID AddTransitionToGroundedHypergraph(std::set<VID> _tail, std::set<VID> _head, 
      GroupPathType* _path, std::shared_ptr<GroupTask> _task);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_drStrategy; // Dynamic regions strategy to ground edges
    std::string m_trajStrategy; // Strategy to get trajectories between edges (comp. rrt)

    std::string m_sampler; // Sampler to generate spawn vertices

    std::string m_replanMethod{"global"}; // How to deal with constraints when replanning MAPF solution

    std::unordered_map<Robot*, MPTask*> m_taskMap;

    WorkspaceSkeleton m_indSkeleton;
    std::string m_skeletonFilename;        ///< The output file for the skeleton graph
    std::string m_skeletonIO;              ///< Option to read or write the skeleton
    std::string m_skeletonType{"reeb"};    ///< Type of skeleton to build.
    std::string m_decompositionLabel;      ///< The workspace decomposition label.
    std::string m_scuLabel;                ///< The skeleton clearance utility label.
    std::string m_groundedHypergraphLabel; ///< The grounded hypergraph label
    std::string m_queryLabel;              ///< The hypergraph query label

    /// Skeleton clearance annotations
    std::map<Robot*, PropertyMap<std::vector<double>,double>*> m_annotationMap; 

    std::unique_ptr<HypergraphSkeletonType> m_skeleton;
    std::unordered_map<size_t, std::unordered_map<Robot*, HID>> m_hidPaths;
    std::unordered_map<Robot*, size_t> m_pathLengths;

    std::unordered_set<HID> m_groundHIDs;
    std::unordered_set<HID> m_mergeTraj;
    std::unordered_set<HID> m_splitTraj;

    std::unordered_set<HID> m_origStart;
    std::unordered_set<HID> m_origTarget;

    // TODO::Think about if we will ever have more than one grounded instance of a WHS vertex
    // Start and end VIDs for each hyperskeleton hyperarc
    std::unordered_map<HID, std::pair<GroupRoadmapType*, VID>> m_startReps;
    std::unordered_map<HID, std::pair<GroupRoadmapType*, VID>> m_endReps;

    /// The dynamic sampling regions will have radius equal to this times the
    /// robot's bounding sphere radius.
    double m_regionFactor{2};

    std::map<Robot*, const double> m_regionRadius; ///< The region radius for each robot.

    /// A configuration is considered to be touching a region when this fraction
    /// of its bounding sphere penetrates into the region.
    double m_penetrationFactor{1};

    double m_intermediateFactor{3.};

    // Cost to go in the individual skeletons
    std::unordered_map<Robot*, std::unordered_map<VID, double>> m_distanceMap;

    std::unordered_map<Robot*, VID> m_skeletonStarts;
    std::unordered_map<Robot*, VID> m_skeletonGoals;

    // Constraints from hyperarcs that failed to ground
    std::unordered_map<std::pair<VID, VID>, RobotGroup*> m_failedEdges;

    // Map from workspace hyper-skeleton VID to all grounded instances of itself
    std::unordered_map<VID,std::unordered_set<VID>> m_vertexGroundingMap;

    ///@}
};

#endif
