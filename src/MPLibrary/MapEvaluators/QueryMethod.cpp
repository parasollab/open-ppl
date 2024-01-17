#include "QueryMethod.h"

#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "Utilities/MetricUtils.h"

#include "nonstd/io.h"

/*----------------------------- Construction ---------------------------------*/

QueryMethod::
QueryMethod() : MapEvaluatorMethod() {
  this->SetName("QueryMethod");
}


QueryMethod::
QueryMethod(XMLNode& _node) : MapEvaluatorMethod(_node) {
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
	m_twoVariable = _node.Read("twoVariableSS",false,m_twoVariable,
									"Flag to use two variable state search.");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

void
QueryMethod::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel() << "::"
      << "\n\tSearch Alg: " << m_searchAlg
      << "\n\tSI Tool Label: " << m_safeIntervalLabel
      << "\n\tAlternate DM: " << m_dmLabel
      << std::endl;
}


void
QueryMethod::
Initialize() {
  // Clear previous state.
  Reset(nullptr);

  this->GetStatClass()->SetStat(this->GetNameAndLabel() + "::FoundPath", 0);
}

/*-------------------------- MapEvaluator Interface --------------------------*/

bool
QueryMethod::
operator()() {
  auto goalTracker = this->GetMPLibrary()->GetGoalTracker();
  const std::vector<size_t> unreachedGoals = goalTracker->UnreachedGoalIndexes();
  auto task = this->GetTask();
  const size_t numGoals = task->GetNumGoals();

  // Record the path length history when this function exits.
  nonstd::call_on_destruct trackHistory([this](){
      this->GetStatClass()->AddToHistory("pathlength", this->GetPath()->Length());
  });

  if(goalTracker->GetStartVIDs().empty()) {
    if(this->m_debug)
      std::cout << "No start VIDs, query cannot succeed."
                << std::endl;
    return false;
  }

  // If no goals remain, then this must be a refinement step (as in optimal
  // planning). In this case or the roadmap has changed, reinitialize and
  // rebuild the whole path.
  auto r = this->GetRoadmap();
  if(unreachedGoals.empty() or r != m_roadmap or task != m_task)
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

  this->GetStatClass()->SetStat(this->GetNameAndLabel() + "::FoundPath", 1);

  if(this->m_debug)
    std::cout << "\tConnected all goals!" << std::endl;

  return true;
}

/*--------------------------- Query Interface --------------------------------*/

std::vector<typename QueryMethod::VID>
QueryMethod::
GeneratePath(const VID _start, const VIDSet& _goals) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "QueryMethod::GeneratePath");

  if(this->m_debug)
    std::cout << "Generating path from " << _start << " to " << _goals << "."
              << std::endl;

  // Check for trivial path.
  if(_goals.count(_start) and m_startTime >= m_endTime)
    return {_start};

  // Check for impossibility with CC information (if available).
  auto ccTracker = m_roadmap->GetCCTracker();
  if(ccTracker) {
    const bool impossible = std::none_of(_goals.begin(), _goals.end(),
        [ccTracker, _start](const VID _vid) {
          return ccTracker->InSameCC(_start, _vid);
        }
    );

    if(impossible) {
      if(this->m_debug)
        std::cout << "\tStart and goal in different CCs, no path exists"
                  << std::endl;
      return {};
    }
  }

  stats->IncStat("Graph Search");

  if(m_twoVariable)
    return TwoVariableQuery(_start,_goals);

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
  if(m_weightFunction) {
    weight = m_weightFunction;
  }
  else if(!this->GetMPProblem()->GetDynamicObstacles().empty()) {
    weight = [this](typename RoadmapType::adj_edge_iterator& _ei,
                    const double _sourceDistance,
                    const double _targetDistance) {
      return this->DynamicPathWeight(_ei, _sourceDistance, _targetDistance);
    };
  }
  else {
    weight = [this](typename RoadmapType::adj_edge_iterator& _ei,
                    const double _sourceDistance,
                    const double _targetDistance) {
      return this->StaticPathWeight(_ei, _sourceDistance, _targetDistance);
    };
  }

  // Run dijkstra's algorithm to find the path, if it exists.
  const SSSPOutput<RoadmapType> sssp = DijkstraSSSP(m_roadmap, {_start}, weight,
      termination);

  // Find the last discovered node, which should be a goal if there is a valid
  // path.
  const VID last = sssp.ordering.back();
  if(!_goals.count(last)) {
    if(this->m_debug)
      std::cout << "\tFailed: could not find a path." << std::endl;
    return {};
  }

  // Extract the path.
  std::vector<VID> path;
  path.push_back(last);

  VID current = last;
  do {
    current = sssp.parent.at(current);
    path.push_back(current);
  } while(current != _start);
  std::reverse(path.begin(), path.end());

  if(this->m_debug)
    std::cout << "\tSuccess: reached goal node " << last << " with path cost "
              << sssp.distance.at(last) << "."
              << std::endl;

  return path;
}


void
QueryMethod::
SetDMLabel(const std::string& _dmLabel) {
  m_dmLabel = _dmLabel;
}


void
QueryMethod::
SetPathWeightFunction(SSSPPathWeightFunction<RoadmapType> _f) {
  m_weightFunction = _f;
}

/*------------------------------- Helpers ------------------------------------*/

void
QueryMethod::
Reset(RoadmapType* const _r) {
  // Set the roadmap.
  m_roadmap = _r;
  m_task = this->GetTask();

  // Reset the goal index.
  m_goalIndex = 0;

  // Reset the path.
  if(this->GetPath())
    this->GetPath()->Clear();
}


void
QueryMethod::
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


bool
QueryMethod::
PerformSubQuery(const VID _start, const VIDSet& _goals) {
  // Try to generate a path from _start to _goal.
  auto path = this->GeneratePath(_start, _goals);

  // If the path is empty, we failed.
  if(path.empty())
    return false;

  // Otherwise, add this segment to the full solution path.
  *this->GetPath() += path;
  return true;
}


double
QueryMethod::
StaticPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
  // Check if Distance Metric has been defined. If so use the Distance Metric's
  // EdgeWeight function to compute the target distance.
  if(!m_dmLabel.empty()) {
    auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);
    return _sourceDistance + dm->EdgeWeight(m_roadmap, _ei);
  }

  // Otherwise use the existing edge weight to compute the distance.
  const double edgeWeight  = _ei->property().GetWeight(),
               newDistance = _sourceDistance + edgeWeight;
  return newDistance;
}


double
QueryMethod::
DynamicPathWeight(typename RoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
  // Compute the new 'distance', which is the number of timesteps at which
  // the robot would reach the target node.
  const double edgeWeight  = _ei->property().GetTimeSteps(),
               newDistance = _sourceDistance + edgeWeight;

  // If this edge isn't better than the previous, we won't use it regardless
  // and can return without checking the dynamic obstacles.
  if(newDistance >= _targetDistance) {
    if(this->m_debug)
      std::cout << "Breaking because the path is not optimal."
                << std::endl;
    return newDistance;
  }

  auto siTool = this->GetMPLibrary()->GetMPTools()->GetSafeIntervalTool(m_safeIntervalLabel);

  // Ensure that the target vertex is contained within a SafeInterval when
  // arriving.
  const auto& target = m_roadmap->GetVertex(_ei->target());
  auto vertexIntervals = siTool->ComputeIntervals(target);
  if(!(siTool->ContainsTimestep(vertexIntervals, newDistance))) {
    if(this->m_debug)
      std::cout << "Breaking because the target vertex is dynamically invalid."
                << "\n\tvertexIntervals: " << vertexIntervals
                << std::endl;
    return std::numeric_limits<double>::infinity();
  }

  // Ensure that the edge is contained within a SafeInterval if leaving now.
  auto edgeIntervals = siTool->ComputeIntervals(_ei->property(), _ei->source(),
      _ei->target(), m_roadmap);
  if(!(siTool->ContainsTimestep(edgeIntervals, _sourceDistance))){
    if(this->m_debug)
      std::cout << "Breaking because the edge is dynamically invalid."
                << std::endl;
    return std::numeric_limits<double>::infinity();
  }

  // If we're still here, the edge is OK.
  return newDistance;
}


std::vector<typename QueryMethod::VID>
QueryMethod::
TwoVariableQuery(const VID _start, const VIDSet& _goals) {
/*
	double minEnd = m_endTime;

  SSSPTerminationCriterion<RoadmapType> termination(
      [minEnd,_goals](typename RoadmapType::vertex_iterator& _vi,
             const SSSPOutput<RoadmapType>& _sssp) {
        return (_goals.count(_vi->descriptor()) or _sssp.distance.at(_vi->descriptor()) > minEnd)
																							? SSSPTermination::EndSearch
                                              : SSSPTermination::Continue;
      }
  );
*/
  SSSPPathWeightFunction<RoadmapType> weight;
  if(m_weightFunction) {
    weight = m_weightFunction;
  }
  else if(!this->GetMPProblem()->GetDynamicObstacles().empty()) {
    weight = [this](typename RoadmapType::adj_edge_iterator& _ei,
                    const double _sourceDistance,
                    const double _targetDistance) {
      return this->DynamicPathWeight(_ei, _sourceDistance, _targetDistance);
    };
  }
  else {
    weight = [this](typename RoadmapType::adj_edge_iterator& _ei,
                    const double _sourceDistance,
                    const double _targetDistance) {
      return this->StaticPathWeight(_ei, _sourceDistance, _targetDistance);
    };
  }


  auto g = this->GetRoadmap();
/*
  // Run dijkstra's algorithm to find the path, if it exists.
  const SSSPOutput<RoadmapType> sssp = DijkstraSSSP(g, {_start}, weight,
      termination);

  // Find the last discovered node, which should be a goal if there is a valid
  // path.
  const VID last = sssp.ordering.back();
  if(_goals.count(last)) {
  	// Extract the path.
  	std::vector<VID> path;
  	path.push_back(last);

  	VID current = last;
  	do {
    	current = sssp.parent.at(current);
    	path.push_back(current);
  	} 	while(current != _start);
  	std::reverse(path.begin(), path.end());

  	if(this->m_debug)
    	std::cout << "\tSuccess: reached goal node " << last << " with path cost "
      	        << sssp.distance.at(last) << "."
      	        << std::endl;

  	return path;
  }
*/
	//If there is not a valid path satisfying both the end time and the goal vid use two variable search

	/*double lastConflict = 0;
	for(auto conflict : g->GetAllConflictsCfgAt()) {
		if(conflict.second > lastConflict) {
			lastConflict = conflict.second;
		}
	}*/

	//double minStep = this->GetMPProblem()->GetEnvironment()->GetTimeRes();
  //double minStep = g->GetEdge(*(_goals.begin()), *(_goals.begin())).GetTimeSteps();

  auto node = TwoVariableDijkstraSSSP(g, {_start}, _goals, m_startTime,
      m_endTime, m_lastConstraint, m_lastGoalConstraint, weight);

  if(!node)
    return {};

  this->GetPath()->SetFinalWaitTimeSteps(node->m_waitTimeSteps);

  std::vector<typename QueryMethod::VID> path;
  while(node->m_parent) {
    path.push_back(node->m_vid);
    node = node->m_parent;
    if(!node)
      return {};
  }

  path.push_back(node->m_vid);
  std::reverse(path.begin(),path.end());

  return path;
}


void
QueryMethod::
SetLastConstraintTime(double _last) {
	m_lastConstraint = _last;
}


void
QueryMethod::
SetLastGoalConstraintTime(double _time) {
	m_lastGoalConstraint = _time;
}


void
QueryMethod::
SetStartTime(double _start) {
	m_startTime = _start;
}


void
QueryMethod::
SetEndTime(double _end) {
	m_endTime = _end;
}
