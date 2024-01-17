#ifndef PMPL_QUERY_METHOD_H_
#define PMPL_QUERY_METHOD_H_

#include "MapEvaluatorMethod.h"

#include "ConfigurationSpace/Path.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/SSSP.h"


////////////////////////////////////////////////////////////////////////////////
/// Base class for all query methods. These objects evaluate a roadmap under
/// construction to see if a planning task has been satisfied.
///
/// The planning task is defined by the MPTask's Constraint objects. The query
/// searches the current roadmap and aims to find a connecting path between
/// configurations satisfying the task's start and goal constraints.
///
/// @note The query will consider multiple nodes satisfying the goal constraints
///       and quit after finding the first. This is an incomplete algorithm if
///       your problem has a sequence of non-point goals.
///
/// @todo Implement A* by accepting a distance metric label for the heuristic
///       function, and expanding the SSSP functions to accept a binary functor
///       heuristic function.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
class QueryMethod : virtual public MapEvaluatorMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType      RoadmapType;
    typedef typename RoadmapType::VID               VID;
    typedef typename RoadmapType::VertexSet         VertexSet;
    typedef std::unordered_set<size_t>              VIDSet;

    ///@}
    ///@name Local Types
    ///@{

    enum GraphSearchAlg {DIJKSTRAS, ASTAR}; ///< The supported sssp algorithms.

    ///@}
    ///@name Construction
    ///@{

    QueryMethod();

    QueryMethod(XMLNode& _node);

    virtual ~QueryMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    virtual void Initialize() override;

    ///@}
    ///@name MapEvaluator Interface
    ///@{

    virtual bool operator()() override;

    ///@}
    ///@name Query Interface
    ///@{

    /// Generate a path through the roadmap from a start node to an end node.
    /// @param _start The start node.
    /// @param _end The set of allowed end nodes.
    /// @return A path of VIDs which transition from _start to the nearest node
    ///         in _end.
    std::vector<VID> GeneratePath(const VID _start, const VIDSet& _end);

    /// Set an alternate distance metric to use when searching the roadmap
    /// (instead of the saved edge weights).
    /// @param _label The Distance Metric label to use. Set to empty string to
    ///               use the saved edge weights.
    void SetDMLabel(const std::string& _dmLabel);

    /// Set an alternate path weight function to use when searching the roadmap.
    /// @param The path weight function object to use.
    virtual void SetPathWeightFunction(SSSPPathWeightFunction<RoadmapType> _f);

    /// Set the time of the last constraint.
    /// @param _last The time to use.
		void SetLastConstraintTime(double _last);

    /// Set the time of the last goal constraint.
    /// @param _time The time to use.
		void SetLastGoalConstraintTime(double _time);

    /// Set the start time of the query.
    /// @param _start The time to use.
		void SetStartTime(double _start);

    /// Set the end time of the query.
    /// @param _end The time to use.
		void SetEndTime(double _end);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Reset the path and list of undiscovered goals.
    /// @param _r The roadmap to use.
    virtual void Reset(RoadmapType* const _r);

    /// Set the search algorithm choice from a string.
    /// @param _alg The search algorithm to use ('astar' or 'dijkstras').
    /// @param _where Error location info in case _alg isn't recognized.
    void SetSearchAlgViaString(std::string _alg, const std::string& _where);

    /// Check whether a path connecting a start to one of several goals exists
    /// in the roadmap.
    /// @param _start The start VID to use.
    /// @param _goals The goal VIDs to use.
    /// @return True if a path from _start to one of _goals was generated.
    virtual bool PerformSubQuery(const VID _start, const VIDSet& _goal);

    /// Define a function for computing a path weight for a specific edge,
    /// ignoring dynamic obstacles.
    /// @param _ei An iterator to the edge we are checking.
    /// @param _sourceDistance The shortest distance to the source node.
    /// @param _targetDistance The best known distance to the target node.
    /// @return The distance to the target node via this edge, or infinity if
    ///         the edge isn't used due to lazy invalidation.
    virtual double StaticPathWeight(
        typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    /// Define a function for computing path weights w.r.t. dynamic obstacles.
    /// Here the metric is the number of time steps, and we return infinity if
    /// taking an edge would result in a collision with a dynamic obstacle.
    /// @param _ei An iterator to the edge we are checking.
    /// @param _sourceDistance The shortest time to the source node.
    /// @param _targetDistance The best known time to the target node.
    /// @return The time to the target node via this edge, or infinity if taking
    ///         this edge would result in a collision with dynamic obstacles.
    virtual double DynamicPathWeight(
        typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    /// Perform a query between start and goal vertices in the roadmap
    /// @param _start The start vertex.
    /// @param _goal The goal vertex.
    std::vector<typename QueryMethod::VID>
        TwoVariableQuery(const VID _start, const VIDSet& _goals);

    ///@}
    ///@name Internal State
    ///@{

    RoadmapType* m_roadmap{nullptr};   ///< Last roadmap queried.
    MPTask* m_task{nullptr};           ///< Last task we looked at.

    size_t m_goalIndex{0};             ///< Index of next unreached goal.

    GraphSearchAlg m_searchAlg{DIJKSTRAS};  ///< The sssp algorithm to use.

    std::string m_safeIntervalLabel; ///< The SafeIntervalTool label.
    std::string m_dmLabel;           ///< The DistanceMetric label.
		bool m_twoVariable{false};       ///< Temporary flag to use two varibale state search.

    /// The function for computing total path weight.
    SSSPPathWeightFunction<RoadmapType> m_weightFunction;

		double m_lastConstraint{0};     ///< The time of the last constraint
		double m_lastGoalConstraint{0}; ///< The time of the last goal constraint
		double m_startTime{0};          ///< The start time of the query
		double m_endTime{0};            ///< The end time of the query

    ///@}

};

#endif
