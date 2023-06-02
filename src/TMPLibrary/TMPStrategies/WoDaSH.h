#ifndef PPL_WORKSPACE_DASH_H_
#define PPL_WORKSPACE_DASH_H_

#include "TMPStrategyMethod.h"

#include "TMPLibrary/ActionSpace/Condition.h"
#include "Geometry/Boundaries/Boundary.h"

#include "Traits/CfgTraits.h"

#include "Workspace/WorkspaceSkeleton.h"
#include "Workspace/HypergraphWorkspaceSkeleton.h"


class Action;
class Interaction;
class Boundary;

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

    typedef std::pair<GroupRoadmapType*, VID> RepresentativeVertex;
    typedef std::pair<bool, size_t>           BoolHID;

    typedef Condition::State                 State;
    typedef std::set<std::string>            RoleSet;

    ///@}
    ///@name Skeleton Types
    ///@{

    typedef typename WorkspaceSkeleton::EI                SkeletonEdgeIterator;
    typedef typename WorkspaceSkeleton::ED                SkeletonEdgeDescriptor;

    typedef CompositeState<WorkspaceSkeleton>  CompositeSkeletonVertex;
    typedef CompositeEdge<WorkspaceSkeleton>   CompositeSkeletonEdge;
    typedef size_t HID;

    ///@}
    ///@name MAPF Types
    ///@{

    typedef GenericStateGraph<std::pair<size_t,size_t>,double> HeuristicSearch;
    typedef std::vector<size_t>                                CBSSolution;

    // ((vid, vid=INVAILD_VID), time)
    typedef std::pair<std::pair<VID, VID>, size_t>             CBSConstraint;
    typedef CBSNode<Robot,CBSConstraint,CBSSolution>           CBSNodeType;

    typedef Robot* OrderingConstraint;
    typedef CBSNode<Robot, OrderingConstraint, CBSSolution>    PBSNodeType;

    ///@}
    ///@name Hyperskeleton Types
    ///@{

    enum class HyperskeletonArcType {
      Movement, ///< Move along a skeleton edge
      Couple,   ///< Merge two groups going in the opposite direction on same edge
      Decouple, ///< Split two groups of robots moving in opposite directions
      Merge,    ///< Merge together groups coming into the same vertex
      Split     ///< Split groups going out from a merge onto different edges
    };

    struct HyperskeletonArc {
      CompositeSkeletonEdge edge;
      bool pushStart{false};
      bool pushTarget{false};
      HyperskeletonArcType type;

      RepresentativeVertex startRep;
      RepresentativeVertex endRep;

      HyperskeletonArc() {}

      HyperskeletonArc(CompositeSkeletonEdge _edge, bool _pushStart, bool _pushTarget,
                       HyperskeletonArcType _type, RepresentativeVertex _start,
                       RepresentativeVertex _end) : edge(_edge), pushStart(_pushStart),
                       pushTarget(_pushTarget), type(_type), startRep(_start), endRep(_end) {}
      
      HyperskeletonArc(CompositeSkeletonEdge _edge, HyperskeletonArcType _type) :
          edge(_edge), type(_type) {}

      bool operator==(const HyperskeletonArc& _arc) {
        return !(*this != _arc);
      }

      bool operator!=(const HyperskeletonArc& _arc) {
        if(edge != _arc.edge)
          return true;
        
        if(pushStart != _arc.pushStart or pushTarget != _arc.pushTarget)
          return true;
        
        if(type != _arc.type)
          return true;
        
        if(startRep != _arc.startRep or endRep != _arc.endRep)
          return true;
        
        return false;
      }

      friend std::ostream& operator<<(std::ostream& _os, const HyperskeletonArc& _arc) {
        _os << "Composite Edge: " << _arc.edge
            << ", pushStart: " << _arc.pushStart
            << ", pushTarget: " << _arc.pushTarget
            << ", type: ";

        switch(_arc.type) {
          case HyperskeletonArcType::Movement: _os << "Movement";
          case HyperskeletonArcType::Couple: _os << "Couple";
          case HyperskeletonArcType::Decouple: _os << "Decouple";
          case HyperskeletonArcType::Merge: _os << "Merge";
          case HyperskeletonArcType::Split: _os << "Split";
        }

        _os << ", startRep: " << _arc.startRep
            << ", endRep: " << _arc.endRep;

        return _os;
      }
    };

    struct HyperskeletonPath {
      std::set<HID> movementHyperarcs; // Want these to ground in order
      std::unordered_set<HID> coupleHyperarcs;
      std::unordered_set<HID> decoupleHyperarcs;
      std::unordered_set<HID> mergeHyperarcs;
      std::unordered_set<HID> splitHyperarcs;

      std::unordered_map<HID, std::unordered_map<Robot*, BoolHID>> predecessors;
      std::unordered_map<HID, std::unordered_map<Robot*, BoolHID>> successors;

      HyperskeletonPath() {}

      void Reset() {
        movementHyperarcs.clear();
        coupleHyperarcs.clear();
        decoupleHyperarcs.clear();
        mergeHyperarcs.clear();
        splitHyperarcs.clear();
        predecessors.clear();
        successors.clear();
      }
    };

    typedef Hypergraph<CompositeSkeletonVertex,
                       HyperskeletonArc> HypergraphSkeletonType;

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

    template <typename NodeType>
    double CostFunction(NodeType& _node);

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

    bool ConnectToSkeleton();

    void GroundStartAndGoal();

    RobotGroup* AddGroup(std::vector<Robot*> _robots);

    HID AddTransitionToGroundedHypergraph(std::set<VID> _tail, std::set<VID> _head, 
      GroupPathType* _path, std::shared_ptr<GroupTask> _task);

    HID AddTransitionToGroundedHypergraph(std::set<RepresentativeVertex> _tail, 
      std::set<RepresentativeVertex> _head, GroupPathType* _path, 
      std::shared_ptr<GroupTask> _task);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_drStrategy; // Dynamic regions strategy to ground edges
    std::string m_trajStrategy; // Strategy to get trajectories between edges (comp. rrt)

    std::string m_sampler; // Sampler to generate spawn vertices

    std::string m_mapf{"cbs"};
    std::string m_replanMethod{"global"}; // How to deal with constraints when replanning MAPF solution

    std::unordered_map<Robot*, MPTask*> m_taskMap;
    std::unordered_map<RobotGroup*, GroupRoadmapType> m_roadmaps;

    RobotGroup* m_wholeGroup{nullptr};
    GroupTask* m_wholeTask{nullptr};

    WorkspaceSkeleton m_indSkeleton;
    std::string m_skeletonFilename;        ///< The output file for the skeleton graph
    std::string m_skeletonIO;              ///< Option to read or write the skeleton
    std::string m_skeletonType{"reeb"};    ///< Type of skeleton to build.
    std::string m_decompositionLabel;      ///< The workspace decomposition label.
    std::string m_scuLabel;                ///< The skeleton clearance utility label.
    std::string m_edgeQueryLabel;          ///< The query method to extract paths along grounded edges.
    std::string m_groundedHypergraphLabel; ///< The grounded hypergraph label
    std::string m_motionEvaluator;         ///< The motion evaluator label (Scheduled CBS)

    /// Skeleton clearance annotations
    std::map<Robot*, PropertyMap<std::vector<double>,double>*> m_annotationMap; 

    std::unique_ptr<HypergraphSkeletonType> m_skeleton;

    // TODO update everything to account for this change (false, vid for waiting)
    std::unordered_map<size_t, std::unordered_map<Robot*, BoolHID>> m_hidPaths;

    // Map hyperskeleton vertex to a represenative group configuration
    std::unordered_map<VID, RepresentativeVertex> m_waitingReps;
    std::unordered_map<Robot*, size_t> m_pathLengths;

    HyperskeletonPath m_path;

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

    VID m_groundedStartVID{INVALID_VID};
    VID m_groundedGoalVID{INVALID_VID};

    // Constraints from hyperarcs that failed to ground
    std::vector<std::unordered_map<Robot*, std::pair<VID, VID>>> m_failedEdges;
    // std::unordered_map<std::pair<VID, VID>, RobotGroup*> m_failedEdges;

    // Map from workspace hyper-skeleton VID to all grounded instances of itself
    std::unordered_map<VID,std::unordered_set<VID>> m_vertexGroundingMap;

    ///@}
};

#endif
