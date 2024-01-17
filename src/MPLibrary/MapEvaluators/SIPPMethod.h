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

class SIPPMethod : public MapEvaluatorMethod {

  public:

    ///@name Local Types
    ///@{

    typedef std::unordered_set<size_t>              VIDSet;
    typedef typename MPBaseObject::RoadmapType      RoadmapType;
    typedef typename MPBaseObject::GroupRoadmapType GroupRoadmapType;
    typedef typename MPBaseObject::GroupCfgType     GroupCfgType;
    typedef typename MPBaseObject::GroupWeightType  GroupWeightType;

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

template <typename AbstractRoadmap>
void
SIPPMethod::
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


template <typename AbstractRoadmap>
void
SIPPMethod::
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

std::istream& operator>>(std::istream& _is, SIPPVertex& _vertex);

std::ostream& operator<<(std::ostream& _os, const SIPPVertex& _vertex);

std::istream& operator>>(std::istream& _is, SIPPEdge& _edge);

std::ostream& operator<<(std::ostream& _os, const SIPPEdge& _edge);

#endif
