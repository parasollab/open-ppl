#ifndef PMPL_GROUP_QUERY_H_
#define PMPL_GROUP_QUERY_H_

#include "QueryMethod.h"

#include "Utilities/MetricUtils.h"
#include "Utilities/SSSP.h"


////////////////////////////////////////////////////////////////////////////////
/// Unlike Query Method, GroupQuery is not yet set up to evaluate a group
/// roadmap under construction to see if a query has been satisfied. All it can
/// do currently is generate a path calling SSSP utilities for the roadmap.
///
/// This query is based off of Query Method, modified to work for GroupRoadmaps.
/// Only GeneratePath has been implemented for this, and this is not a valid
/// query evaluation as it stands. This is intended for path-building
/// functionality alone.
///
/// @todo Fix this so that it works like the other queries, or homogenize
///       the two.
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
    typedef typename GroupRoadmapType::VID        VID;

    ///@}
    ///@name Construction
    ///@{

    GroupQuery();
    GroupQuery(XMLNode& _node);
    virtual ~GroupQuery() = default;

    ///@}
    ///@name MapEvaluator Overrides
    ///@{

    /// Interface function for MapEvaluatorMethod. The function of GroupQuery
    /// is not to actually evaluate a group roadmap for now, but rather for
    /// building paths with group cfgs/vids.
    virtual bool operator()() override;

    ///@}
    ///@name Query Interface
    ///@{

    std::vector<VID> GeneratePath(const VID _start, const VID _end);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    double StaticPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    double DynamicPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    ///@}

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

/*-------------------------- MapEvaluator Overrides --------------------------*/

template <typename MPTraits>
bool
GroupQuery<MPTraits>::
operator()() {
  throw RunTimeException(WHERE) << "Not to be used as a map evaluator in its "
                                << "current state!";
  return false;
}

/*--------------------------- Query Interface --------------------------------*/

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

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
double
GroupQuery<MPTraits>::
StaticPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
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
