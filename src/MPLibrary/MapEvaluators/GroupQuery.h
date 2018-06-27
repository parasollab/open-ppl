#ifndef GROUP_QUERY_H_
#define GROUP_QUERY_H_

#include "QueryMethod.h"

#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/SSSP.h"

#include "containers/sequential/graph/algorithms/astar.h"

////////////////////////////////////////////////////////////////////////////////
/// Evaluate a group roadmap under construction to see if a query has been
/// satisfied.
///
/// This query is based off of GroupQuery, modified to work for GroupRoadmaps.
/// Only GeneratePath has been implemented for this, and it is not a valid
/// query evaluation as it stands. This is intended for path-building
/// functionality alone.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GroupQuery : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::GroupCfgType       CfgType;
    typedef typename MPTraits::GroupRoadmapType   GroupRoadmapType;
    typedef typename MPTraits::GroupWeightType    WeightType;

    typedef typename GroupRoadmapType::VID       VID;

    typedef stapl::sequential::map_property_map<GroupRoadmapType, size_t> ColorMap;

    ///@}
    ///@name Construction
    ///@{

    GroupQuery();
    GroupQuery(XMLNode& _node);
    virtual ~GroupQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    /// Interface function for MapEvaluatorMethod. The function of GroupQuery
    /// is not to actually evaluate a group roadmap for now, but rather for
    /// building paths with group cfgs/vids.
    virtual bool operator()() override {
      throw RunTimeException(WHERE, "Not to be used as a map evaluator in its "
                                    "current state!");
    }

    ///@}
    ///@name Query Interface
    ///@{

    /// Perform the query, overloaded to handle the Group case.
//    virtual bool PerformQuery(GroupRoadmapType* const _r) override;

    /// Get a random unconnected goal.
//    const CfgType& GetRandomGoal() const;

//    virtual bool PerformSubQuery(const CfgType& _start, const CfgType& _goal)
//        override;

//    virtual void Reset(GroupRoadmapType* const _r);

    std::vector<VID> GeneratePath(const VID _start, const VID _end);

    double StaticPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    double DynamicPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Find the nearest node to a goal configuration, assuming that the goal is
    /// not already in the map.
    /// @param[in] _goal The goal node of interest.
    /// @return The descriptor of the map node that is nearest to _goal, or
    ///         INVALID_VID if we have already checked every node in the map. The
    ///         second item is the distance from said node to _goal.
//    pair<VID, double> FindNearestNeighbor(const CfgType& _goal);

    /// Find the nearest node to a goal configuration that is already connected
    /// to the start node. The start is assumed to be in the map, but the goal
    /// may or may not be.
    /// @param[in] _start The descriptor of the start node in the map.
    /// @param[in] _goal  The goal configuration, possibly in the map.
    /// @return If the goal isn't in the map, return the descriptor of the map
    ///         node that is nearest to it and their separation distance. If
    ///         the goal is already in the map, return its VID if it is connected
    ///         to the start and INVALID_VID otherwise.
//    pair<VID, double> FindNearestConnectedNeighbor(const VID _start,
//        const CfgType& _goal);

    /// Extend towards a goal from the nearest node in the map.
    /// @param[in] _nearest The descriptor of the nearest node and its distance
    ///                     to _goal.
    /// @param[in] _goal The goal configuration to extend towards.
    /// @return The descriptor of the newly extended node and its distance to
    ///         _goal, or INVALID_VID if the extension failed.
//    pair<VID,double> ExtendToGoal(const pair<VID, double>& _nearest,
//        const CfgType& _goal) const;


//    void GenerateQuery();

    /// Generate a color map for CC operations which marks the unused vertices
    /// as 'black' (already completed), which causes them to be skipped in graph
    /// operations.
    ColorMap GetColorMap() const;


    ///@}
    ///@name Query State
    ///@{

//    double m_goalDist{0.};      ///< Getting at least this close = success.
//    VID m_highestCheckedVID;    ///< The highest VID we have tried to connect to
                                ///< the goal.

    ///@}
    ///@name MP Object Labels
    ///@{

    std::string m_dmLabel;    ///< Distance metric label

//    std::string m_nfLabel{"Nearest"};       ///< Neighborhood finder label.
//    std::string m_exLabel{"BERO"};          ///< Extender label.

    ///@}
    ///@name Unhide QueryMethod names.
    ///@{

//    using QueryMethod<MPTraits>::m_goals;

    ///@}

//    enum GraphSearchAlg {DIJKSTRAS, ASTAR}; ///< The supported sssp algorithms.
//    GraphSearchAlg m_searchAlg{DIJKSTRAS};  ///< The sssp algorithm to use.
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
GroupQuery<MPTraits>::
GroupQuery() : MapEvaluatorMethod<MPTraits>() {
  this->SetName("GroupQuery");
}


template <typename MPTraits>
GroupQuery<MPTraits>::
GroupQuery(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("GroupQuery");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
GroupQuery<MPTraits>::
Print(ostream& _os) const {
  MapEvaluatorMethod<MPTraits>::Print(_os);
}

/*--------------------------- Query Interface --------------------------------*/

template <typename MPTraits>
typename GroupQuery<MPTraits>::ColorMap
GroupQuery<MPTraits>::
GetColorMap() const {
  auto g = this->GetGroupRoadmap();

  // Define a constant for the color marking.
  static const auto white = stapl::graph_color<size_t>::white();

  // Mark all vertices white (none are marked unused).
  ColorMap colorMap;
  for(auto vi = g->begin(); vi != g->end(); ++vi)
    colorMap.put(*vi, white);

  return colorMap;
}

//template <typename MPTraits>
//std::vector<typename GroupQuery<MPTraits>::VID>
//GroupQuery<MPTraits>::
//GeneratePath(const VID _start, const VID _end) {
//
//  auto stats = this->GetStatClass();
//  MethodTimer mt(stats, "GroupQuery::GraphSearch");
//  stats->IncStat("Graph Search");
//
//  // Define types we will need for the graph search.
//  using namespace stapl::sequential;
//  using namespace stapl;
//
//  typedef typename GroupRoadmapType::vertex_reference     VR;
//  typedef map_property_map<GroupRoadmapType, VID>         ParentMap;
//  //typedef map_property_map<GroupRoadmapType, WeightType>  DistanceMap;
//  typedef DynamicWeightMap<GroupRoadmapType, WeightType>  DistanceMap;
//  typedef std::less<WeightType>                    Comparator;
//  typedef std::plus<WeightType>                    Combiner;
//  typedef d_ary_heap_indirect<VR, 4, ParentMap, DistanceMap, Comparator> Queue;
//
//  GroupRoadmapType* const g = this->GetGroupRoadmap();
//
////  if(this->m_debug) {
////    // Hardcoded tests for pentomino roadmap connectedness:
////    const bool valid = g->IsEdge(0, 1) and g->IsEdge(1, 12) and g->IsEdge(12, 13)
////        and g->IsEdge(13, 14) and g->IsEdge(14, 15) and g->IsEdge(15, 16) and
////        g->IsEdge(16, 17) and g->IsEdge(17, 18) and g->IsEdge(18, 19) and
////        g->IsEdge(19,20) and g->IsEdge(20, 11) and g->IsEdge(11, 21) and
////        g->IsEdge(21, 24);
////    std::cout << "GroupQuery::GeneratePath: " << std::endl;
////    if(!valid)
////      std::cerr << "Error! Path won't be found!" << std::endl << std::endl;
////    else
////      std::cout << "All edges are present in roadmap" << std::endl << std::endl;
////  }
//
//  // Build structures for STAPL graph search.
//  static const WeightType weightMax(g, "", std::numeric_limits<double>::max()),
//                          weightZero(g, "", 0.);
//  static const Combiner combiner;
//  static const Comparator comparator;
//  ParentMap parentMap, indexMap, colorMap = GetColorMap();
//  DistanceMap weightMap, distanceMap;
//
//  // Initialize the various maps.
//  for(auto vi = g->begin(); vi != g->end(); ++vi) {
//    parentMap.put(*vi, vi->descriptor());
//
////    const bool usingThis = IsVertexUsed(vi->descriptor());
////    distanceMap.put(*vi, usingThis ? weightMax : weightZero);
//    distanceMap.put(*vi, weightMax);
//
//    // Initialize the edge weight map for each adjacent edge.
//    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
//      std::cout << "Edge weight: " << ei->property().GetWeight() << std::endl;
//      weightMap.put(*ei, ei->property());
//    }
////      weightMap.put(*ei, this->IsEdgeUsed(ei->id()) ? ei->property() : weightMax);
//  }
//
//  // Make sure the start vertex has zero distance.
//  distanceMap.put(_start, weightZero);
//
//  Queue queue(distanceMap, indexMap, comparator);
//
//
////  if(this->m_debug) {
////    std::cout << std::endl << std::endl << std::endl;
////    std::cout << "Printing out weight map:" << std::endl;
////    for(auto e : weightMap.m_map)
////      std::cout << "first: " << e.first << "; second: " << e.second << std::endl;
////
////    std::cout << std::endl << std::endl << std::endl;
////    std::cout << "Printing out distance map:" << std::endl;
////    for(auto e : distanceMap.m_map)
////      std::cout << "first: " << e.first << "; second: " << e.second << std::endl;
////
////    std::cout << std::endl << std::endl << std::endl;
////    std::cout << "Printing out parent map:" << std::endl;
////    for(auto e : parentMap.m_map)
////      std::cout << "first: " << e.first << "; second: " << e.second << std::endl;
////  }
//
//
//
//  // Run the graph search.
////  switch(m_searchAlg) {
////    case DIJKSTRAS:
//  dijkstra_sssp(*g, parentMap, distanceMap, weightMap,
//                _start, _end, comparator, weightMax,
//                combiner, colorMap, queue);
////      break;
////    case ASTAR:
////      Heuristic<MPTraits> heuristic(g->GetVertex(_end),
////          this->GetEnvironment()->GetPositionRes(),
////          this->GetEnvironment()->GetOrientationRes());
////      DistanceMap dummyMap;
////      astar(*g, parentMap, distanceMap, dummyMap, weightMap,
////            _start, _end, heuristic, comparator,
////            combiner, colorMap, queue);
////      break;
////  }
//
//  // Extract path from sssp results.
//  vector<VID> path;
//  path.push_back(_end);
//  VID temp = _end;
//  while(parentMap.get(temp) != temp)
//  {
//    path.push_back(parentMap.get(temp));
//    temp = parentMap.get(temp);
//  }
//
//  // A path was found if temp is the start node. In that case reverse it so that
//  // it goes from start to goal.
//  if(temp == _start)
//    std::reverse(path.begin(), path.end());
//  else
//    path.clear();
//
//  return path;
//}



template <typename MPTraits>
std::vector<typename GroupQuery<MPTraits>::VID>
GroupQuery<MPTraits>::
GeneratePath(const VID _start, const VID _end) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "GroupQuery::GeneratePath");
  stats->IncStat("Graph Search");

  // Set up the termination criterion to quit early if we find the _end node.
  SSSPTerminationCriterion<GroupRoadmapType> termination(
      [_end](typename GroupRoadmapType::vertex_iterator& _vi,
             const SSSPOutput<GroupRoadmapType>& _sssp) {
        return _vi->descriptor() == _end ? SSSPTermination::EndSearch
                                         : SSSPTermination::Continue;
      }
  );

  // Set up the path weight function depending on whether we have any dynamic
  // obstacles.
  SSSPPathWeightFunction<GroupRoadmapType> weight;
  if(!this->GetMPProblem()->GetDynamicObstacles().empty()) {
    weight = [this](typename GroupRoadmapType::adj_edge_iterator& _ei,
                    const double _sourceDistance,
                    const double _targetDistance) {
      return this->DynamicPathWeight(_ei, _sourceDistance, _targetDistance);
    };
  }
  else {
    weight = [this](typename GroupRoadmapType::adj_edge_iterator& _ei,
                    const double _sourceDistance,
                    const double _targetDistance) {
      return this->StaticPathWeight(_ei, _sourceDistance, _targetDistance);
    };
  }


  // Run dijkstra's algorithm to find the path, if it exists.
  auto g = this->GetGroupRoadmap();
  const SSSPOutput<GroupRoadmapType> sssp = DijkstraSSSP(g, {_start}, weight);

  // If the end node has no parent, there is no path.
  if(!sssp.parent.count(_end))
    return {};

  // Extract the path.
  std::vector<VID> path;
  path.push_back(_end);

  VID current = _end;
  do {
    current = sssp.parent.at(current);
    path.push_back(current);
  } while(current != _start);
  std::reverse(path.begin(), path.end());

  return path;
}



template <typename MPTraits>
double
GroupQuery<MPTraits>::
StaticPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
  // Check if Distance Metric has been defined. If so use the Distance Metric's
  // Edge Weight Function
  if(!(m_dmLabel.empty())) {
    auto dm = this->GetDistanceMetric(m_dmLabel);
    const double edgeWeight = dm->EdgeWeight(_ei->source(), _ei->target());
    return edgeWeight;
  }

  // Compute the new 'distance', which is the number of timesteps at which
  // the robot would reach the target node.
  const double edgeWeight  = _ei->property().GetWeight(),
               newDistance = _sourceDistance + edgeWeight;
  return newDistance;
}


template <typename MPTraits>
double
GroupQuery<MPTraits>::
DynamicPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
  throw RunTimeException(WHERE, "Not implemented!");
}

/*----------------------------------------------------------------------------*/

#endif
