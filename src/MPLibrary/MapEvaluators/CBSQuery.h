#ifndef PPL_CBS_QUERY_H_
#define PPL_CBS_QUERY_H_

#include "MapEvaluatorMethod.h"
#include "SIPPMethod.h"
#include "Utilities/CBS.h"

////////////////////////////////////////////////////////////////////////////////
/// Generates paths for each robot individually and then finds and resolves
/// conflicts by setting constraints on each robot's path and replanning.
///
/// Reference:
///   Guni Sharon, Roni Stern, Ariel Felner, and Nathan Sturtevant. "Conflict-
///   Based Search For Optimal Multi-Agent Path Finding". AAAI 2012.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
class CBSQuery : public MapEvaluatorMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType   RoadmapType;
    typedef typename RoadmapType::VID        VID;
    typedef typename RoadmapType::EdgeID     EdgeID;

    ///}

  private:

    ///@name Internal Types
    ///@{

    // Edge <Source, Target>, Time Interval <Start, End> 
    typedef std::pair<std::pair<size_t,size_t>,Range<size_t>> Constraint;
    typedef CBSNode<Robot,Constraint,Path>    Node;

    typedef std::map<size_t,std::vector<Range<size_t>>> VertexIntervals;
    typedef std::map<std::pair<size_t,size_t>,std::vector<Range<size_t>>> EdgeIntervals;

    ///@}

  public:

    ///@name Construction
    ///@{

    CBSQuery();

    CBSQuery(XMLNode& _node);

    virtual ~CBSQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name MapEvaluator Overrides
    ///@{

    virtual bool operator()() override;

    ///@}

  protected:

    ///@name CBS Functors
    ///@{

    /// Generate a path for a robot given a set of constraints.
    /// @param _robot The robot to solve the task for.
    /// @param _node The CBS node to store the new path in.
    /// @return Path pointer to the resulting path. A nullptr indicates failure.
    Path* SolveIndividualTask(Robot* const _robot, Node& _node);

    /// Validate that the set of paths in the node do not contain conflicts.
    /// @param _node The node to validate
    /// @return The set of new robot-constraint pairs to add to the node generated
    ///         from discovered conflicts.
    std::vector<std::pair<Robot*,Constraint>> ValidationFunction(Node& _node);

    /// Create child nodes corresponding to the additional constraints.
    /// @param _node The parent CBS node to spawn children for.
    /// @param _constraints The set of new constraints to spawn new child nodes for.
    /// @param _lowLevel The functor to plan new paths for robots with new constraints.
    /// @param _cost The functor to compute the cost of new child nodes.
    /// @return The set of new child nodes.
    std::vector<Node> SplitNodeFunction(Node& _node, 
          std::vector<std::pair<Robot*, Constraint>> _constraints,
          CBSLowLevelPlanner<Robot, Constraint, Path>& _lowlevel,
          CBSCostFunction<Robot, Constraint, Path>& _cost);

    /// Compute the cost of a CBS node. Can either be makespace or sum-of-cost.
    /// @param _node Node to compute cost for.
    /// @return The computed cost.
    double CostFunction(Node& _node);

    ///@}
    ///@name Helpers
    ///@{

    /// Compute the safe intervals for a robot given its constraint set in the node.
    /// @param _robot The robot to compute safe intervals for.
    /// @param _node The node containing the constraint set.
    void ComputeIntervals(Robot* _robot, const Node& _node);

    /// Construct the safe intervals that complement the unsafe intervals.
    /// @param _unsafeIntervals The unsafe intervals that define the complement.
    /// @return The set of safe intervals.
    std::vector<Range<size_t>> ConstructSafeIntervals(std::vector<Range<size_t>>& _unsafeIntervals);

    /// Check if two intervals are overlapping.
    /// @param _interval1 First interval to check.
    /// @param _interval2 Second interval to check.
    /// @return Boolean indicating if they are overlapping.
    bool OverlappingIntervals(Range<size_t> _interval1, Range<size_t> _interval2);

    /// Merge a pair of intervals into a new interval.
    /// @param _interval1 First interval to check.
    /// @param _interval2 Second interval to check.
    /// @return New merged interval.
    Range<double> MergeIntervals(Range<double> _interval1, Range<double> _interval2);

    ///@}
    ///@name Internal State
    ///@{

    std::vector<Robot*> m_robots; ///< The robots in the group

    std::string m_queryLabel;  ///< Query method for making individual plans.
    std::string m_vcLabel;     ///< Validity checker for conflict detection.
    std::string m_costLabel = "SOC"; ///< The label of the cost function

    size_t m_nodeLimit{std::numeric_limits<size_t>::max()}; ///< The maximum number of nodes

    std::unordered_map<Robot*, MPTask*> m_taskMap; ///< The task for each robot

    VertexIntervals m_vertexIntervals;  ///< The current set of safe vertex intervals.
    EdgeIntervals m_edgeIntervals;      ///< The current set of safe edge intervals.

    ///@}

};

#endif
