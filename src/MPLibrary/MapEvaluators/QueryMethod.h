#ifndef PMPL_QUERY_METHOD_H_
#define PMPL_QUERY_METHOD_H_

#include "MapEvaluatorMethod.h"

#include "ConfigurationSpace/Path.h"
#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/SSSP.h"

#include "nonstd/io.h"


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
template <typename MPTraits>
class QueryMethod : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType          RoadmapType;
    typedef typename MPTraits::CfgType              CfgType;
    typedef typename RoadmapType::VID               VID;
    typedef typename RoadmapType::EdgeID            EdgeID;
    typedef typename MPTraits::GoalTracker          GoalTracker;
    typedef typename GoalTracker::VIDSet            VIDSet;

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

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Reset the path and list of undiscovered goals.
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
    double StaticPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    /// Define a function for computing path weights w.r.t. dynamic obstacles.
    /// Here the metric is the number of time steps, and we return infinity if
    /// taking an edge would result in a collision with a dynamic obstacle.
    /// @param _ei An iterator to the edge we are checking.
    /// @param _sourceDistance The shortest time to the source node.
    /// @param _targetDistance The best known time to the target node.
    /// @return The time to the target node via this edge, or infinity if taking
    ///         this edge would result in a collision with dynamic obstacles.
    double DynamicPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    /// Define a function for computing path weights w.r.t. multi-robot
    /// problems. Here the metric is the number of time steps, and we return
    /// infinity if taking an edge would result in a inter-robot collision.
    /// @param _ei An iterator to the edge we are checking.
    /// @param _sourceDistance The shortest time to the source node.
    /// @param _targetDistance The best known time to the target node.
    /// @return The time to the target node via this edge, or infinity if taking
    ///         this edge would result in a collision with dynamic obstacles.
    double MultiRobotPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    ///@}
    ///@name Internal State
    ///@{

    RoadmapType* m_roadmap{nullptr};   ///< Last roadmap queried.

    size_t m_goalIndex{0};             ///< Index of next unreached goal.

    GraphSearchAlg m_searchAlg{DIJKSTRAS};  ///< The sssp algorithm to use.

    std::string m_safeIntervalLabel; ///< The SafeIntervalTool label.
    std::string m_dmLabel;           ///< The DistanceMetric label.

    ///@}

};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
QueryMethod<MPTraits>::
QueryMethod() : MapEvaluatorMethod<MPTraits>() {
  this->SetName("QueryMethod");
}


template <typename MPTraits>
QueryMethod<MPTraits>::
QueryMethod(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("QueryMethod");

  // Parse the search algorithm type.
  const std::string searchAlg = _node.Read("graphSearchAlg", false, "dijkstras",
      "Graph search algorithm.");
  SetSearchAlgViaString(searchAlg, _node.Where());

  // Parse the other options.
  m_safeIntervalLabel = _node.Read("safeIntervalToolLabel", false, "",
      "Label of the SafeIntervalTool");
  m_dmLabel = _node.Read("dmLabel", false, "",
      "Alternate distance metric to replace edge weight during search.");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
QueryMethod<MPTraits>::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel() << "::"
      << "\n\tSearch Alg: " << m_searchAlg
      << "\n\tSI Tool Label: " << m_safeIntervalLabel
      << "\n\tAlternate DM: " << m_dmLabel
      << std::endl;
}


template <typename MPTraits>
void
QueryMethod<MPTraits>::
Initialize() {
  // Clear previous state.
  m_dmLabel.clear();
  Reset(nullptr);

  this->GetStatClass()->SetStat(
      "QueryMethod::" + this->GetLabel() + "::FoundPath", 0);
}

/*-------------------------- MapEvaluator Interface --------------------------*/

template <typename MPTraits>
bool
QueryMethod<MPTraits>::
operator()() {
  auto goalTracker = this->GetGoalTracker();
  const std::vector<size_t> unreachedGoals = goalTracker->UnreachedGoalIndexes();
  auto task = this->GetTask();
  const size_t numGoals = task->GetNumGoals();

  if(goalTracker->GetStartVIDs().empty()){
    return false;
  }
  // If no goals remain, then this must be a refinement step (as in optimal
  // planning). In this case or the roadmap has changed, reinitialize and
  // rebuild the whole path.
  auto r = this->GetRoadmap();
  if(unreachedGoals.empty() or r != m_roadmap)
    Reset(r);

  if(this->m_debug)
    std::cout << "Querying roadmap for a path satisfying task '"
              << task->GetLabel()
              << "', " << unreachedGoals.size() << " / " << numGoals
              << " goals not reached."
              << "\n\tTrying to connect goal: " << m_goalIndex
              << "\n\tUnreached goals: " << unreachedGoals
              << std::endl;

  // Search for a sequential path through each task constraint in order.
  auto path = this->GetPath();
  for(; m_goalIndex < numGoals; ++m_goalIndex) {
    // If this goal constraint is unreached, the query cannot succeed.
    auto iter = std::find(unreachedGoals.begin(), unreachedGoals.end(),
        m_goalIndex);
    const bool unreached = iter != unreachedGoals.end();
    if(unreached) {
      if(this->m_debug)
        std::cout << "\tGoal " << m_goalIndex << " has no satisfying VIDs."
                  << std::endl;
      return false;
    }

    // Get the start VID for this subquery.
    const VID start = path->Empty() ? *goalTracker->GetStartVIDs().begin()
                                    : path->VIDs().back();

    // Get the goal VIDs for this subquery.
    const VIDSet& goals = goalTracker->GetGoalVIDs(m_goalIndex);
    if(goals.empty())
      throw RunTimeException(WHERE) << "No VIDs located for reached goal "
                                    << m_goalIndex << ".";

    if(this->m_debug)
      std::cout << "\tEvaluating sub-query from " << start << " to " << goals
                << "." << std::endl;

    // Warn users if multiple goals are found.
    if(goals.size() > 1 and numGoals > 1)
      std::cerr << "\tWarning: subquery has " << goals.size() << " possible VIDs "
                << "for goal " << m_goalIndex << "/" << numGoals
                << ". The algorithm will try its best but isn't complete for "
                << "this case." << std::endl;

    // Perform this subquery. If it fails, there is no path.
    const bool success = PerformSubQuery(start, goals);
    if(!success)
      return false;
  }

  // We generated a path successfully: track the path length history.
  this->GetStatClass()->AddToHistory("pathlength", this->GetPath()->Length());

  this->GetStatClass()->SetStat(
      "QueryMethod::" + this->GetLabel() + "::FoundPath", 1);

  if(this->m_debug)
    std::cout << "\tConnected all goals!" << std::endl;

  return true;
}

/*--------------------------- Query Interface --------------------------------*/

template <typename MPTraits>
std::vector<typename QueryMethod<MPTraits>::VID>
QueryMethod<MPTraits>::
GeneratePath(const VID _start, const VIDSet& _goals) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "QueryMethod::GeneratePath");

  if(this->m_debug)
    std::cout << "Generating path from " << _start << " to " << _goals << "."
              << std::endl;

  // Check for trivial path.
  if(_goals.count(_start))
    return {_start};

  stats->IncStat("Graph Search");

  // Set up the termination criterion to quit early if we find a goal node.
  SSSPTerminationCriterion<RoadmapType> termination(
      [_goals](typename RoadmapType::vertex_iterator& _vi,
             const SSSPOutput<RoadmapType>& _sssp) {
        return _goals.count(_vi->descriptor()) ? SSSPTermination::EndSearch
                                               : SSSPTermination::Continue;
      }
  );

  // Set up the path weight function depending on whether we have any dynamic
  // obstacles.
  SSSPPathWeightFunction<RoadmapType> weight;
  if(!this->GetMPProblem()->GetDynamicObstacles().empty()) {
    weight = [this](typename RoadmapType::adj_edge_iterator& _ei,
                    const double _sourceDistance,
                    const double _targetDistance) {
      return this->DynamicPathWeight(_ei, _sourceDistance, _targetDistance);
    };
  } else if(this->GetMPProblem()->NumRobots() >= 2) {
    weight = [this](typename RoadmapType::adj_edge_iterator& _ei,
                    const double _sourceDistance,
                    const double _targetDistance) {
      return this->MultiRobotPathWeight(_ei, _sourceDistance, _targetDistance);
    };
  } else {
    weight = [this](typename RoadmapType::adj_edge_iterator& _ei,
                    const double _sourceDistance,
                    const double _targetDistance) {
      return this->StaticPathWeight(_ei, _sourceDistance, _targetDistance);
    };
  }


  // Run dijkstra's algorithm to find the path, if it exists.
  auto g = this->GetRoadmap();
  const SSSPOutput<RoadmapType> sssp = DijkstraSSSP(g, {_start}, weight,
      termination);

  // Find the last discovered node, which should be a goal if there is a valid
  // path.
  const VID last = sssp.ordering.back();
  if(!_goals.count(last))
    return {};

  // Extract the path.
  std::vector<VID> path;
  path.push_back(last);

  VID current = last;
  do {
    current = sssp.parent.at(current);
    path.push_back(current);
  } while(current != _start);
  std::reverse(path.begin(), path.end());

  return path;
}


template <typename MPTraits>
void
QueryMethod<MPTraits>::
SetDMLabel(const std::string& _dmLabel) {
  m_dmLabel = _dmLabel;
}

/*------------------------------- Helpers ------------------------------------*/

template <typename MPTraits>
void
QueryMethod<MPTraits>::
Reset(RoadmapType* const _r) {
  // Set the roadmap.
  m_roadmap = _r;

  // Reset the goal index.
  m_goalIndex = 0;

  // Reset the path.
  if(this->GetPath())
    this->GetPath()->Clear();
}


template <typename MPTraits>
void
QueryMethod<MPTraits>::
SetSearchAlgViaString(std::string _alg, const std::string& _where) {
  std::transform(_alg.begin(), _alg.end(), _alg.begin(), ::tolower);

  if(_alg == "dijkstras")
    m_searchAlg = DIJKSTRAS;
  else if(_alg == "astar")
  {
    throw NotImplementedException(_where) << "We do not actually have A* "
                                          << "implemented since the STAPL "
                                          << "version does not use the "
                                          << "heuristic, and the new impl only "
                                          << "supports dijkstras right now.";
    m_searchAlg = ASTAR;
  }
  else
    throw ParseException(_where) << "Invalid graph search algorithm '" << _alg
                                 << "'. Choices are 'dijkstras'.";// or 'astar'.";
}


template <typename MPTraits>
bool
QueryMethod<MPTraits>::
PerformSubQuery(const VID _start, const VIDSet& _goals) {
  // Try to generate a path from _start to _goal.
  auto path = this->GeneratePath(_start, _goals);

  // If the path isn't empty, we succeeded.
  if(!path.empty()) {
    *this->GetPath() += path;
    const VID goalVID = path.back();

    if(this->m_debug)
      std::cout << "\tSuccess: reached goal node " << goalVID << "."
                << std::endl;

    return true;
  }
  else if(this->m_debug)
    std::cout << "\tFailed: could not find a path." << std::endl;

  return false;
}


template <typename MPTraits>
double
QueryMethod<MPTraits>::
StaticPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
  // First check if the edge is lazily invalidated. If so, the distance is
  // infinite.

  if(m_roadmap->IsEdgeInvalidated(_ei->id()))
    return std::numeric_limits<double>::infinity();

  // Check if Distance Metric has been defined. If so use the Distance Metric's
  // EdgeWeight function to compute the target distance.
  if(!m_dmLabel.empty()) {
    auto dm = this->GetDistanceMetric(m_dmLabel);
    return _sourceDistance + dm->EdgeWeight(_ei->source(), _ei->target());
  }

  // Otherwise use the existing edge weight to compute the distance.
  const double edgeWeight  = _ei->property().GetWeight(),
               newDistance = _sourceDistance + edgeWeight;
  return newDistance;
}


template <typename MPTraits>
double
QueryMethod<MPTraits>::
DynamicPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
  // First check if the edge is lazily invalidated. If so, the distance is
  // infinite.
  if(m_roadmap->IsEdgeInvalidated(_ei->id()))
    return std::numeric_limits<double>::infinity();

  // Compute the new 'distance', which is the number of timesteps at which
  // the robot would reach the target node.
  const double edgeWeight  = _ei->property().GetTimeSteps(),
               newDistance = _sourceDistance + edgeWeight;

  // If this edge isn't better than the previous, we won't use it regardless
  // and can return without checking the dynamic obstacles.
  // if(newDistance >= _targetDistance) {
  //   if(this->m_debug)
  //     std::cout << "Breaking because the path is not optimal."
  //               << std::endl;
  //   return newDistance;
  // }

  // Get the graph and safe interval tool.
  //auto g = this->GetRoadmap();
  SafeIntervalTool<MPTraits>* siTool = this->GetMPTools()->GetSafeIntervalTool(
      m_safeIntervalLabel);

  if(!siTool->IsEdgeDynamicallySafe(m_roadmap->GetVertex(_ei->source()),m_roadmap
            ->GetVertex(_ei->target()), _sourceDistance, _targetDistance)) {
    // if(this->m_debug) {
    //   std::cout << "Invalidating conflicting edge (" << _ei->source()
    //    << "," << _ei->target() << ")" << "at timestep "
    //    << conflictTimestep << std::endl;
    // }
    // If the edge gets invalidadted we add it to the EdgeInvalidated
    // List to avoid future collision checkings
    // m_roadmap->SetEdgeInvalidatedAt(_ei->source(), _ei->target(),
    //   conflictTimestep, true);
    return std::numeric_limits<double>::infinity();
  }

  // ------------------------------------------------------------------------
  // This part is commented beacause don't want to compute the safe intervals
  //------------------------------------------------------------------------
  // Ensure that the target vertex is contained within a SafeInterval when
  // arriving.
  // auto vertexIntervals = siTool->ComputeIntervals(g->GetVertex(_ei->target()));
  // if(!(siTool->ContainsTimestep(vertexIntervals, newDistance))) {
  //   if(this->m_debug)
  //     std::cout << "Breaking because the target vertex is dynamically invalid."
  //               << "\n\tvertexIntervals: " << vertexIntervals
  //               << std::endl;
  //   return std::numeric_limits<double>::infinity();
  // }

  // // Ensure that the edge is contained within a SafeInterval if leaving now.
  // auto edgeIntervals = siTool->ComputeIntervals(_ei->property(),_ei->source(),
  // _ei->target(),g->GetVertex(_ei->source()) );
  // if(!(siTool->ContainsTimestep(edgeIntervals, _sourceDistance))){
  //   if(this->m_debug)
  //     std::cout << "Breaking because the edge is dynamically invalid."
  //               << std::endl;
  //   return std::numeric_limits<double>::infinity();
  // }
  // --------------------------------------------------------------------------

  // If we're still here, the edge is OK.
  return newDistance;
}


template <typename MPTraits>
double
QueryMethod<MPTraits>::
MultiRobotPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
  // Checking if the edge is already invalidated
  auto dm = this->GetDistanceMetric(m_dmLabel);
  if(m_roadmap->IsEdgeInvalidatedAt(_ei->id(),_sourceDistance,_sourceDistance +
   dm->EdgeWeight(_ei->source(), _ei->target()))) {
   	if(this->m_debug)
    	std::cout << "EDGE INVALIDATED!!!" << std::endl;
    return std::numeric_limits<double>::infinity();
  } 
	// If the edge is not invalidated, we will go trough the ConflictCfg List
	// to obtain all the conflicting timesteps
  const std::vector<std::pair<CfgType,double>>& conflictCfgsAt = m_roadmap->
  	GetAllConflictsCfgAt();
  double conflictTimestep = 0;
  for(size_t i = 0 ; i <  conflictCfgsAt.size() ; ++i){
      conflictTimestep = conflictCfgsAt[i].second;
      // If the edge contains one of the conflicting timesteps we perform a
      // collision checking over the whole edge against the corresponding
      // conflicting cfg
      if(conflictTimestep > _sourceDistance && conflictTimestep < (
      		_sourceDistance + dm->EdgeWeight(_ei->source(), _ei->target()))
      	) {
        SafeIntervalTool<MPTraits>* siTool = this->GetMPTools()->
      		GetSafeIntervalTool("SI");
        if(!siTool->IsEdgeSafe(m_roadmap->GetVertex(_ei->source()),m_roadmap
        	->GetVertex(_ei->target()), conflictCfgsAt[i].first)) {
        	if(this->m_debug) {
          	std::cout << "Invalidating conflicting edge (" << _ei->source()
          	 << "," << _ei->target() << ")" << "at timestep "
          	 << conflictTimestep << std::endl;
        	}
        	// If the edge gets invalidadted we add it to the EdgeInvalidated
        	// List to avoid future collision checkings
          m_roadmap->SetEdgeInvalidatedAt(_ei->source(), _ei->target(),
          	conflictTimestep, true);
          return std::numeric_limits<double>::infinity();
        }
      }
  }
  
  return StaticPathWeight(_ei, _sourceDistance, _targetDistance);
} 


/*----------------------------------------------------------------------------*/

#endif
