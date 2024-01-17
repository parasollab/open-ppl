#include "DistanceMetricMethod.h"

/*------------------------------- Construction -------------------------------*/


DistanceMetricMethod::
DistanceMetricMethod(XMLNode& _node) : MPBaseObject(_node) {
}

/*----------------------------- Distance Interface ---------------------------*/

double
DistanceMetricMethod::
Distance(const GroupCfgType& _c1, const GroupCfgType& _c2) {
  double sum = 0;
  for(size_t i = 0; i < _c1.GetNumRobots(); ++i)
    sum += Distance(_c1.GetRobotCfg(i), _c2.GetRobotCfg(i));

  return sum;
}


double
DistanceMetricMethod::
EdgeWeight(const RoadmapType* const _r, const typename RoadmapType::CEI& _edge)
    noexcept {
  // Get the intermediates. If there aren't any, the weight is just from source
  // to target.
  const auto& edge = *_edge;
  const auto& intermediates = edge.property().GetIntermediates();
  const typename RoadmapType::VP& source = _r->GetVertex(edge.source()),
                                & target = _r->GetVertex(edge.target());
  if(intermediates.empty())
    return Distance(source, target);

  // If we're still here, there are intermediates. Add up the distance through
  // the intermediates.
  double totalDistance = Distance(source, intermediates.front())
                       + Distance(intermediates.back(), target);

  for(size_t i = 0; i < intermediates.size() - 1; ++i)
    totalDistance += Distance(intermediates[i], intermediates[i + 1]);

  return totalDistance;
}


double
DistanceMetricMethod::
EdgeWeight(const RoadmapType* const _r, const VID _source,
    const VID _target) noexcept {
  // Find the edge and ensure it exists.
  typename RoadmapType::CEI edge;
  if(!_r->GetEdge(_source, _target, edge))
    throw RunTimeException(WHERE) << "Requested non-existent edge ("
                                  << _source << ", " << _target
                                  << ") in roadmap " << _r << "."
                                  << std::endl;
  return EdgeWeight(_r, edge);
}

/*---------------------------- Configuration Scaling -------------------------*/

void
DistanceMetricMethod::
ScaleCfg(double _length, Cfg& _c, const Cfg& _o) {
  /// @todo This is a very expensive way to scale a configuration. We should
  ///       probably remove it and require derived classes to implement a more
  ///       efficient function (this is the best we can do for a base class that
  ///       does not know anything about the properties of the metric space).

  _length = fabs(_length); //a distance must be positive
  Cfg origin = _o;
  Cfg outsideCfg = _c;

  // first find an outsite configuration with sufficient size
  while(Distance(origin, outsideCfg) < 2 * _length)
    for(size_t i = 0; i < outsideCfg.DOF(); ++i)
      outsideCfg[i] *= 2.0;
  // now, using binary search find a configuration with the approximate length
  Cfg aboveCfg = outsideCfg;
  Cfg belowCfg = origin;
  Cfg currentCfg = _c;
  while (1) {
    for(size_t i=0; i<currentCfg.DOF(); ++i)
      currentCfg[i] = (aboveCfg[i] + belowCfg[i]) / 2.0;
    double magnitude = Distance(origin, currentCfg);
    if((magnitude >= _length*0.9) && (magnitude <= _length*1.1))
      break;
    if(magnitude>_length)
      aboveCfg = currentCfg;
    else
      belowCfg = currentCfg;
  }

  _c = currentCfg;
}


void
DistanceMetricMethod::
ScaleCfg(double _length, Cfg& _c) {
  ScaleCfg(_length, _c, Cfg(_c.GetRobot()));
}


void
DistanceMetricMethod::
ScaleCfg(double _length, GroupCfgType& _c, const GroupCfgType& _o) {
  _length = fabs(_length); //a distance must be positive
  GroupCfgType origin = _o;
  GroupCfgType outsideCfg = _c;

  // first find an outsite configuration with sufficient size
  while(Distance(origin, outsideCfg) < 2 * _length) {
    for(size_t i = 0; i < _c.GetNumRobots(); ++i) {
      for(size_t j = 0; j < _c.GetRobotCfg(i).DOF(); ++j) {
          outsideCfg.GetRobotCfg(i)[j] *= 2.0; 
      }
    }
  }
  // now, using binary search find a configuration with the approximate length
  GroupCfgType aboveCfg = outsideCfg;
  GroupCfgType belowCfg = origin;
  GroupCfgType currentCfg = _c;

  while (1) {
    for(size_t i = 0; i < _c.GetNumRobots(); ++i) {
      for(size_t j=0; j<currentCfg.GetRobotCfg(i).DOF(); ++j)
          currentCfg.GetRobotCfg(i)[j] = (aboveCfg.GetRobotCfg(i)[j] + belowCfg.GetRobotCfg(i)[j]) / 2.0;
    }
    double magnitude = Distance(origin, currentCfg); 
    if((magnitude >= _length*0.9) && (magnitude <= _length*1.1))
      break;
    if(magnitude >_length)
      aboveCfg = currentCfg;
    else
      belowCfg = currentCfg;
  }

  _c = currentCfg;
}


void
DistanceMetricMethod::
ScaleCfg(double _length, GroupCfgType& _c) {
  const GroupCfgType origin(_c.GetGroupRoadmap());
  ScaleCfg(_length, _c, origin);
}

/*----------------------------------------------------------------------------*/
