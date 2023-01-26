#ifndef PPL_SIPP_METHOD_H_
#define PPL_SIPP_METHOD_H_

#include "MapEvaluatorMethod.h"

#include "ConfigurationSpace/Path.h"
#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"

#include "Geometry/Boundaries/Range.h"

#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "MPLibrary/MPTools/SafeIntervalTool.h"

#include "MPProblem/GroupTask.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include "Utilities/MetricUtils.h"
#include "Utilities/SSSP.h"

#include "nonstd/io.h"
#include <unordered_map>
#include <vector>
#include <iostream>
#include <unordered_set>

////////////////////////////////////////////////////////////////////////////////
/// Map evaluator for Safe Interval Path Planning (SIPP)
/// Utilizes MPTools/SafeIntervalTools to help calculate Safe intervals
/// for vertices and edges
////////////////////////////////////////////////////////////////////////////////
struct SIPPVertex {
  size_t vid;
  Range<size_t> interval;

  bool operator==(const SIPPVertex& _other) const {
    return vid      == _other.vid &&
    interval == _other.interval;
  }
};

struct SIPPEdge {
  size_t source;
  size_t target;
  Range<double> interval;
};

template <typename MPTraits>
class SIPPMethod : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{
    typedef typename MPTraits::GoalTracker      GoalTracker;
    typedef typename GoalTracker::VIDSet        VIDSet;
    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType     GroupCfgType;
    typedef typename MPTraits::GroupWeightType  GroupWeightType;

    typedef std::map<std::pair<size_t,size_t>,
                    std::vector<Range<size_t>>> EdgeIntervalMap;

    typedef std::map<size_t,std::vector<Range<size_t>>> VertexIntervalMap;

    typedef GenericStateGraph<SIPPVertex, SIPPEdge> SIPPGraph;

    ///@}
    ///@name Construction
    ///@{

    SIPPMethod();

    SIPPMethod(XMLNode& _node);

    virtual ~SIPPMethod() = default;

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

    /// Generate a path from a start point to a goal point
    /// @param _start The start coordinate.
    /// @param _goal The goal coordinate.
    std::pair<std::vector<size_t>,std::vector<size_t>> 
          GeneratePath(const size_t _start, const std::vector<size_t> _goals);

    /// Set the distance metric to use
    void SetDMLabel(const std::string& _dmLabel);

    /// Set the start time of the query
    void SetStartTime(size_t _start);

    /// Set the end time of the query
    void SetMinEndTime(size_t _end);

    /// Set the edge intervals to use to generate a path
    void SetEdgeIntervals(EdgeIntervalMap _edgeIntervals);

    /// Set the edge intervals to use to generate a path
    void SetVertexIntervals(VertexIntervalMap _vertexIntervals);

    /// Check if the path satisfies all constraints
    bool SatisfyConstraints(Range<size_t> _interval);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Reset the path and list of undiscovered goals.
    /// Also resets wait times and cached safe intervals.
    virtual void Reset();

    /// Check whether a path connecting a start to one of several goals exists
    /// in the roadmap using safe intervals.
    /// @param _start The start VID to use.
    /// @param _goals The goal VIDs to use.
    /// @return True if a path from _start to one of _goals was generated.
    virtual bool PerformSubQuery(const size_t _start, const std::vector<size_t> _goals);

    /// Define a function for computing path weights w.r.t. dynamic obstacles.
    /// Here the metric is the number of time steps, and we return the distance
    /// with a wait time if taking an edge would result in a collision
    /// with a dynamic obstacle. If waiting cannot fix, return infinity.
    /// @param _ei An iterator to the edge we are checking.
    /// @param _sourceDistance The shortest time to the source node.
    /// @param _targetDistance The best known time to the target node.
    /// @return The time to the target node via this edge including waiting,
    ///         or infinity if taking this edge would result in a collision
    ///         with dynamic obstacles.
    virtual double PathWeight(typename SIPPGraph::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance);

    /// Heuristic function for Safe interval path planning.
    /// Calls Dijkstra's from goal node to find shortest path to all
    /// nodes in roadmap.
    /// @param _g
    /// @param _source
    /// @param _target
    /// @return
    double SIPPHeuristic(const SIPPGraph* _g,
                         typename SIPPGraph::vertex_descriptor _source,
                         typename SIPPGraph::vertex_descriptor _target);

    /// Neighbors function for Safe interval path planning.
    void SIPPNeighbors(SIPPGraph* _g, typename SIPPGraph::vertex_descriptor _vid);
 
    template <typename AbstractRoadmap>
    void SIPPNeighbors(SIPPGraph* _g, typename SIPPGraph::vertex_descriptor _vid,
                       AbstractRoadmap* _rm);

    /// Construct the SIPP neighbors for roadmap edge
    template <typename AbstractRoadmap>
    void BuildNeighbors(typename SIPPGraph::vertex_descriptor _sippSource,
                        size_t _rmTarget, AbstractRoadmap* _rm);

    /// Initialize the cost to go from the start point to the goal
    void InitializeCostToGo(const std::vector<size_t> _goal);

    ///@}
    ///@name Internal State
    ///@{

    /// The graph representing the time-interval extended state space.
    SIPPGraph* m_sippGraph{nullptr}; 

    size_t m_goalIndex{0};      ///< Index of the current goal to extend the path to.
    size_t m_startVID{SIZE_MAX}; ///< The start vid in the sipp graph.

    /// Cost-to-go map used for heuristic values.
    std::unordered_map<size_t,double> m_costToGoMap;

    EdgeIntervalMap m_edgeIntervals; ///< Set of safe edge intervals.
    VertexIntervalMap m_vertexIntervals; ///< Set of safe vertex intervals.

    /// Compute wait timesteps at each vertex during the search process.
    std::unordered_map<size_t,std::unordered_map<size_t,size_t>> m_waitTimesteps;

    size_t m_startTime{0}; ///< The start time of the path.
    size_t m_minEndTime{0}; ///< The minimum end time of the path.

    bool m_initialized{false}; ///< Flag indicating if the object has been initialized.
    bool m_minTime{true}; ///< Flag indiciating if search is minimizing time or distance metric

    // James: We no longer use the SI Tool to compute intervals.
    // More accurately, we no longer compute intervals within this class and instead 
    // set them externally. That encapsulation is fine, but we should remove this.
    std::string m_safeIntervalLabel; ///< Label of the SI Tool to use.
    std::string m_dmLabel;           ///< Distance metric label.

    ///@}
};

/*----------------------- Construction -----------------------*/

template <typename MPTraits>
SIPPMethod<MPTraits>::
SIPPMethod() : MapEvaluatorMethod<MPTraits>() {
  this->SetName("SIPPMethod");
}

template <typename MPTraits>
SIPPMethod<MPTraits>::
SIPPMethod(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("SIPPMethod");

  // Parse options
  m_safeIntervalLabel = _node.Read("safeIntervalToolLabel", true, "",
      "Label of the SafeIntervalTool");
  m_dmLabel = _node.Read("dmLabel", false, "",
      "Alternate distance metric to replace edge weight during search.");
  m_minTime = _node.Read("minTime",false,m_minTime,
      "Flag indicating if search is minimizing time or distance metric.");
}


/*------------------------ Interface -------------------------*/

template <typename MPTraits>
void
SIPPMethod<MPTraits>::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel() << "::"
      << "\n\tSearch Alg: astar"
      << "\n\tSI Tool Label: " << m_safeIntervalLabel
      << "\n\tAlternative DM: " << m_dmLabel
      << std::endl;
}

template <typename MPTraits>
void
SIPPMethod<MPTraits>::
Initialize() {
  if(m_initialized)
    return;
  // Clear previous state
  Reset();

  this->GetStatClass()->SetStat(
      "SIPPMethod::" + this->GetLabel() + "::FoundPath", 0);

  m_initialized = true;
}



template <typename MPTraits>
bool
SIPPMethod<MPTraits>::
operator()() {

  Reset();

  // Initialize task and goals
  auto goalTracker = this->GetGoalTracker();
  const std::vector<size_t> unreachedGoals = goalTracker->UnreachedGoalIndexes();
  const size_t numGoals = this->GetGroupTask() ? this->GetGroupTask()->GetNumGoals()
                                               : this->GetTask()->GetNumGoals();

  if(goalTracker->GetStartVIDs().empty()) {
    std::cout << "No start VIDs, query cannot succeed."
              << std::endl;
    return false;
  }

  // Search for a sequential path through each task constraint in order.
  for(; m_goalIndex < numGoals; ++m_goalIndex) {
    // If this goal constraint is unreached, the query cannot succeed.
    auto iter = std::find(unreachedGoals.begin(), unreachedGoals.end(),
        m_goalIndex);
    const bool unreached = iter != unreachedGoals.end();
    if(unreached) {
      std::cout << "\tGoal " << m_goalIndex << " has no satisfying VIDs."
                  << std::endl;
      return false;
    }

    // Get the start VID for this subquery.
    //const size_t start = m_startVID == MAX_INT ? *goalTracker->GetStartVIDs().begin()
    //                                           : m_sippGraph->GetVertex(m_startVID).vid;

    // Get the start VID for this subquery.
    size_t start = SIZE_MAX;
    if(m_startVID != SIZE_MAX) {
      m_sippGraph->GetVertex(m_startVID).vid;
    }
    else if(this->GetTask()) {
      start = *goalTracker->GetStartVIDs().begin();
    }
    else {
      //auto r = this->GetGroupRoadmap();

      //if(r->GetActiveFormations().empty()) {
      //  start = *goalTracker->GetStartVIDs().begin();
      //}

      for(auto vid : goalTracker->GetStartVIDs()) {
        //if(SameFormations(r->GetVertex(vid).GetFormations(),r->GetActiveFormations())) {
          start = vid;
          break;
        //}
      }
    }

    if(start == SIZE_MAX)
      throw RunTimeException(WHERE) << "No VIDs located for start.";

    // Get the goal VIDs for this subquery.
    const VIDSet& goals = goalTracker->GetGoalVIDs(m_goalIndex);
    if(goals.empty())
      throw RunTimeException(WHERE) << "No VIDs located for reached goal "
                                    << m_goalIndex << ".";

    std::cout << "\tEvaluating sub-query from " << start << " to " << goals
                << "." << std::endl;

    // Warn users if multiple goals are found.
    if(goals.size() > 1 and numGoals > 1)
      std::cerr << "\tWarning: subquery has " << goals.size() << " possible VIDs "
                << "for goal " << m_goalIndex << "/" << numGoals
                << ". The algorithm will try its best but isn't complete for "
                << "this case." << std::endl;

    // Check if this is the first query or not
    if(m_startVID == SIZE_MAX) {

      // Create start state for query
      SIPPVertex vertex;
      vertex.vid = start;
      
      std::vector<Range<size_t>> startIntervals;
      if(!m_vertexIntervals.empty()) {
        startIntervals = m_vertexIntervals[start];
      }
      else {
        startIntervals.push_back(Range<size_t>(0,SIZE_MAX)); 
      }

      bool found = false;
      for(auto iv : startIntervals) {
        if(iv.Contains(m_startTime)) {
          vertex.interval = iv;
          found = true;
          break;
        }
      }
    
      if(!found) {
        std::cout << "Failed to find valid start interval for vid: "
                  << start
                  << " at time :"
                  << m_startTime;

        return false;
      }

      m_sippGraph = new SIPPGraph(nullptr);
      m_startVID = m_sippGraph->AddVertex(vertex);
    }

    // Perform this subquery. If it fails, there is no path.
    std::vector<size_t> g(goals.begin(),goals.end());
    const bool success = PerformSubQuery(m_startVID, g);
    if(!success)
      return false;

  }


  // We generated a path successfully: track the path length history.
  this->GetStatClass()->AddToHistory("pathlength", 
    this->GetGroupTask() ? this->GetGroupPath()->Length()
                         : this->GetPath()->Length());
  this->GetStatClass()->SetStat(
      "QueryMethod::" + this->GetLabel() + "::FoundPath", 1);

  std::cout << "\tConnected all goals!" << std::endl;

  return true;
}

template <typename MPTraits>
std::pair<std::vector<size_t>,std::vector<size_t>>
SIPPMethod<MPTraits>::
GeneratePath(const size_t _start, const std::vector<size_t> _goals) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "SIPPMethod::GeneratePath");
  stats->IncStat("SIPP Graph Search");

  // Set up termination criterion to exit early if we find goal node.
  SSSPTerminationCriterion<SIPPGraph> termination(
      [_goals,this](typename SIPPGraph::vertex_iterator& _vi, const SSSPOutput<SIPPGraph>& _sssp) {

      // Check if vertex has reached the goal roadmap vid and satisfies other constraints
      auto vertex = _vi->property();
      
      bool isGoal = false;
      for(auto vid : _goals) {
        if(vid == vertex.vid) {
          isGoal = true;
          break;
        }
      }

      if(isGoal&& this->SatisfyConstraints(vertex.interval))
        return SSSPTermination::EndSearch;

      return SSSPTermination::Continue;
    }
  );

  SSSPPathWeightFunction<SIPPGraph> weight = [this](
      typename SIPPGraph::adj_edge_iterator& _ei,
      const double _sourceDistance,
      const double _targetDistance) {
    return this->PathWeight(_ei, _sourceDistance, _targetDistance);
  };

  SSSPHeuristicFunction<SIPPGraph> heuristic = [this](
      const SIPPGraph* _g,
      typename SIPPGraph::vertex_descriptor _source,
      typename SIPPGraph::vertex_descriptor _target) {
    return this->SIPPHeuristic(_g, _source, _target);
  };

  SSSPNeighborsFunction<SIPPGraph> neighbors = [this](
      SIPPGraph* _g, typename SIPPGraph::vertex_descriptor _vid) {
    this->SIPPNeighbors(_g, _vid);
  };

  SSSPOutput<SIPPGraph> output;
  std::vector<size_t> starts = {_start};
  std::vector<size_t> goals = {};


  output = AStarSSSP(m_sippGraph,
                          starts,
                          goals,
                          weight,
                          heuristic,
                          neighbors,
                          termination,
                          m_startTime);

  auto current = output.ordering.back();
  auto lastState = m_sippGraph->GetVertex(current);

  bool isGoal = false;
  for(auto vid : _goals) {
    if(vid == lastState.vid) {
      isGoal = true; 
      break;
    }
  }

  if(!isGoal or !SatisfyConstraints(lastState.interval)) {
    std::cout << "Failed to find a path." <<std::endl;
    return std::make_pair(std::vector<size_t>(),std::vector<size_t>());
  }

  auto vertex = m_sippGraph->GetVertex(current);
  //auto prev = vertex;
  std::vector<size_t> path = {vertex.vid};
  std::vector<size_t> waitTimesteps;
  if(output.distance[current] >= m_minEndTime) {
    waitTimesteps.push_back(0);
  }
  else {
    //const double timeRes = this->GetEnvironment()->GetTimeRes();
    //waitTimesteps.push_back(std::ceil((m_minEndTime - output.distance[current])/timeRes));
    waitTimesteps.push_back(std::ceil((m_minEndTime - output.distance[current])));
  }
  auto previous = current;

  while(current != _start) {
    previous = current;
    current = output.parent[current];
    vertex = m_sippGraph->GetVertex(current);
    //m_wait_timesteps.push_back(ceil(m_transitionWait[current][previous]));
    //prev = state;
    path.push_back(vertex.vid);
    waitTimesteps.push_back(m_waitTimesteps[current][previous]);
  }

  std::reverse(path.begin(), path.end());
  std::reverse(waitTimesteps.begin(), waitTimesteps.end());

  return std::make_pair(path,waitTimesteps);
}

template <typename MPTraits>
void
SIPPMethod<MPTraits>::
SetDMLabel(const std::string& _dmLabel) {
  m_dmLabel = _dmLabel;
}

template <typename MPTraits>
void
SIPPMethod<MPTraits>::
SetStartTime(size_t _start) {
  m_startTime = _start;
}

template <typename MPTraits>
void
SIPPMethod<MPTraits>::
SetMinEndTime(size_t _end) {
  m_minEndTime = _end;
}

template <typename MPTraits>
void
SIPPMethod<MPTraits>::
Reset() {
  m_waitTimesteps.clear();
  m_goalIndex = 0;
  m_startVID = SIZE_MAX;
  delete m_sippGraph;
  m_sippGraph = nullptr;

  if(this->GetPath())
    this->GetPath()->Clear();
}

/*--------------------- Helper Functions ---------------------*/

template <typename MPTraits>
bool
SIPPMethod<MPTraits>::
PerformSubQuery(const size_t _start, const std::vector<size_t> _goals) {
  InitializeCostToGo(_goals);
  auto pair = this->GeneratePath(_start, _goals);
  auto path = pair.first;
  auto waitTimesteps = pair.second;

  if(this->m_debug) {
    std::cout << "Found path with vids and wait times:" << std::endl;
    std::cout << path << std::endl << waitTimesteps << std::endl;
  }

  // Check if path is empty
  if(path.empty())
    return false;

  // Save last vid as start for next subquery
  m_startVID = path.back();

  // Add segment of path to full solution path
  if(this->GetGroupTask()) {
    *(this->GetGroupPath()) += path;
    auto oldWaitTimes = this->GetGroupPath()->GetWaitTimes();
    for(auto w : waitTimesteps) {
      oldWaitTimes.push_back(w);
    }
    this->GetGroupPath()->SetWaitTimes(oldWaitTimes);
  }
  else  {
    *(this->GetPath()) += path;
    auto oldWaitTimes = this->GetPath()->GetWaitTimes();
    for(auto w : waitTimesteps) {
      oldWaitTimes.push_back(w);
    }
    this->GetPath()->SetWaitTimes(oldWaitTimes);
  }
  return true;
}

template <typename MPTraits>
double
SIPPMethod<MPTraits>::
PathWeight(typename SIPPGraph::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) {
  //const double timeRes = this->GetEnvironment()->GetTimeRes();
  const size_t buffer = 2;
  const size_t zero = 0;

  size_t sourceDistance = size_t(_sourceDistance);

  auto source = m_sippGraph->GetVertex(_ei->source());
  auto target = m_sippGraph->GetVertex(_ei->target());

  std::vector<Range<size_t>> intervals;
  if(!m_edgeIntervals.empty()) {
    intervals = m_edgeIntervals[std::make_pair(source.vid,target.vid)];
  }
  else {
    intervals.push_back(Range<size_t>(zero,SIZE_MAX));
  }

  //bool reachedFirstInterval = false;
  for(auto iter = intervals.begin(); iter != intervals.end(); iter++) {
    //if(!reachedFirstInterval and *iter != edge.interval) {
    //  continue;
    //}
    //reachedFirstInterval = true;

    auto interval = *iter;

    // Check if transition is valid w.r.t. intervals
    size_t minWaitTime = std::max(zero,std::min(interval.min - sourceDistance,interval.min));
    if(minWaitTime > 0)
      minWaitTime += buffer;
    const size_t minStartTime = sourceDistance + minWaitTime;

    // Check source interval for min start time
    if(!source.interval.Contains(std::max(std::min(minStartTime-buffer,minStartTime),m_startTime)) 
    or !source.interval.Contains(std::max(minStartTime,minStartTime+buffer))) {
      continue;
    }

    // Check edge interval for min start time
    if(!interval.Contains(std::max(std::min(minStartTime-buffer,minStartTime),m_startTime)) 
    or !interval.Contains(std::max(minStartTime,minStartTime+buffer))) {
      continue;
    }

    // Check target interval for overlap
    const size_t duration = this->GetGroupTask()  
          ? this->GetGroupRoadmap()->GetEdge(source.vid,target.vid).GetTimeSteps()
          : this->GetRoadmap()->GetEdge(source.vid,target.vid).GetTimeSteps();
    const size_t minEndTime = minStartTime + duration;

    size_t waitTime = minWaitTime;
    if(!target.interval.Contains(std::max(std::min(minEndTime-buffer,minEndTime),m_startTime)) 
    or !target.interval.Contains(std::max(minEndTime+buffer,minEndTime))) {
      // Check that min end time is not greater than the interval range
      if(target.interval.max < std::max(minEndTime+buffer,minEndTime)) {
        continue;
      }

      waitTime = target.interval.min - minEndTime;
      if(waitTime > 0)
        waitTime += buffer;

      const size_t startTime = minStartTime + waitTime;

      // Check source interval for start time
      if(!source.interval.Contains(std::max(std::min(startTime-buffer,startTime),m_startTime)) 
      or !source.interval.Contains(std::max(startTime+buffer,startTime))) {
        continue;
      }

      // Check edge interval for start time
      if(!interval.Contains(std::max(std::min(startTime-buffer,startTime),m_startTime)) 
      or !interval.Contains(std::max(startTime+buffer,startTime))) {
        continue;
      }

      if(!target.interval.Contains(std::max(std::min(startTime+duration-buffer,startTime+duration),m_startTime)) 
      or !target.interval.Contains(std::max(startTime+duration+buffer,startTime+duration)))
        continue;

      waitTime = startTime - sourceDistance;
    }
    m_waitTimesteps[_ei->source()][_ei->target()] = waitTime; //size_t(std::ceil(waitTime/timeRes));

    double edgeCost;

    if(m_minTime) {
      edgeCost = double(waitTime + duration);
    }
    else {
      throw RunTimeException(WHERE) << "Arbitrary distance metric based eval not supported.";
      auto source = m_sippGraph->GetVertex(_ei->source()).vid;
      auto target = m_sippGraph->GetVertex(_ei->target()).vid;
      edgeCost = this->GetGroupTask() ? this->GetGroupRoadmap()->GetEdge(source,target).GetWeight()
                                      : this->GetRoadmap()->GetEdge(source,target).GetWeight();
    }

    return  sourceDistance + edgeCost;
  }

  return std::numeric_limits<double>::infinity();
}

template <typename MPTraits>
double
SIPPMethod<MPTraits>::
SIPPHeuristic(const  SIPPGraph* _g,
              typename SIPPGraph::vertex_descriptor _source,
              typename SIPPGraph::vertex_descriptor _target) {
  SIPPVertex vertex = _g->GetVertex(_target);

  if(m_costToGoMap.find(vertex.vid) != m_costToGoMap.end())
    return m_costToGoMap.at(vertex.vid);

  return std::numeric_limits<double>::infinity();
}

template <typename MPTraits>
void
SIPPMethod<MPTraits>::
SIPPNeighbors(SIPPGraph* _g,
    typename SIPPGraph::vertex_descriptor _vid) {

  // Switch on single robot or robot group
  if(this->GetGroupTask()) {
    SIPPNeighbors(_g,_vid,this->GetGroupRoadmap());
  }
  else {
    SIPPNeighbors(_g,_vid,this->GetRoadmap());
  }
}

template <typename MPTraits>
template <typename AbstractRoadmap>
void
SIPPMethod<MPTraits>::
SIPPNeighbors(SIPPGraph* _g,
    typename SIPPGraph::vertex_descriptor _vid,
    AbstractRoadmap* _rm) {

  auto vertex = _g->GetVertex(_vid);
  auto vid = vertex.vid;

  // Get vertex iterator for vid
  auto vit = _rm->find_vertex(vid);

  // Check that vertex is contained in roadmap
  if(vit == _rm->end())
    throw RunTimeException(WHERE) << vid << " does not existing in roadmap.";

  // Build SIPP Neighbors for each roadmap neighbor
  for(auto eit = vit->begin(); eit != vit->end(); eit++) {
    BuildNeighbors(_vid,eit->target(),_rm);
  }
}

template<typename MPTraits>
template <typename AbstractRoadmap>
void
SIPPMethod<MPTraits>::
BuildNeighbors(typename SIPPGraph::vertex_descriptor _sippSource, size_t _rmTarget,
               AbstractRoadmap* _rm) {

  const auto vertex = m_sippGraph->GetVertex(_sippSource);
  const size_t rmSource = vertex.vid;

  // Find candidate intervals for target vertex
  std::vector<Range<size_t>> endIntervals;
  if(!m_vertexIntervals.empty()) { 
    endIntervals = m_vertexIntervals[_rmTarget];
  }
  else {
    endIntervals.push_back(Range<size_t>(0,SIZE_MAX));
  }

  for(auto& inter : endIntervals) {

    // Add new sipp vertex and edge to graph
    SIPPVertex targetVertex;
    targetVertex.vid = _rmTarget;
    targetVertex.interval = inter;

    auto targetVID = m_sippGraph->AddVertex(targetVertex);
    
    SIPPEdge newEdge;
    newEdge.source = rmSource;
    newEdge.target = _rmTarget;

    m_sippGraph->AddEdge(_sippSource,targetVID,newEdge);
  }

}

template<typename MPTraits>
void
SIPPMethod<MPTraits>::
InitializeCostToGo(const vector<size_t> _goals) {

  std::vector<size_t> starts = _goals;

  if(this->GetGroupTask()) {

    SSSPTerminationCriterion<GroupRoadmapType> termination(
        [](typename GroupRoadmapType::vertex_iterator& _vi,
          const SSSPOutput<GroupRoadmapType>& _sssp) {
          return SSSPTermination::Continue;
        }
    );

    SSSPPathWeightFunction<GroupRoadmapType> weight = [this](
        typename GroupRoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance,
        const double _targetDistance) {

     return _sourceDistance + double(_ei->property().GetTimeSteps());
    };

    auto output = DijkstraSSSP(this->GetGroupRoadmap(), starts, weight, termination);
    m_costToGoMap = output.distance;
  }
  else {

    SSSPTerminationCriterion<RoadmapType> termination(
        [](typename RoadmapType::vertex_iterator& _vi,
          const SSSPOutput<RoadmapType>& _sssp) {
          return SSSPTermination::Continue;
        }
    );

    SSSPPathWeightFunction<RoadmapType> weight = [this](
        typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance,
        const double _targetDistance) {
     return _sourceDistance + _ei->property().GetTimeSteps();
    };

    auto output = DijkstraSSSP(this->GetRoadmap(), starts, weight, termination);
    m_costToGoMap = output.distance;
  }
}

template <typename MPTraits>
void
SIPPMethod<MPTraits>::
SetEdgeIntervals(EdgeIntervalMap _edgeIntervals) {
  m_edgeIntervals = _edgeIntervals;
}

template <typename MPTraits>
void
SIPPMethod<MPTraits>::
SetVertexIntervals(VertexIntervalMap _vertexIntervals) {
  m_vertexIntervals = _vertexIntervals;
}

template <typename MPTraits>
bool
SIPPMethod<MPTraits>::
SatisfyConstraints(Range<size_t> _interval) {
  return _interval.max >= m_minEndTime;
}

/*template <typename MPTraits>
bool
SIPPMethod<MPTraits>::
SameFormations(std::unordered_set<Formation*> _set1, std::unordered_set<Formation*> _set2) {
  
  bool matched = true;

  for(auto f1 : _set1) {
    matched = false;
    for(auto f2 : _set2) {
      if(*f1 == *f2) {
        matched = true;
        break;
      }
    }

    if(!matched)
      break;
  }

  if(!matched)
    return std::numeric_limits<double>::infinity();

  for(auto f2 : _set2) {
    matched = false;
    for(auto f1 : _set1) {
      if(*f2 == *f1) {
        matched = true;
        break;
      }
    }

    if(!matched)
      break;
  }

  if(!matched)
    return false;

  return true;
}
*/

/*------------------------------------------------------------*/

std::istream& operator>>(std::istream& _is, SIPPVertex& _vertex);

std::ostream& operator<<(std::ostream& _os, const SIPPVertex& _vertex);

std::istream& operator>>(std::istream& _is, SIPPEdge& _edge);

std::ostream& operator<<(std::ostream& _os, const SIPPEdge& _edge);

#endif
