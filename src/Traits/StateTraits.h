#ifndef STATE_TRAITS_H_
#define STATE_TRAITS_H_

#include "Cfg/State.h"
#include "Edges/StateEdge.h"

//distance metric includes
#include "DistanceMetrics/ExperimentalDistance.h"
#include "DistanceMetrics/WeightedEuclideanDistance.h"
#include "DistanceMetrics/ReachabilityDistance.h"

//validity checker includes
#include "ValidityCheckers/AlwaysTrueValidity.h"
#include "ValidityCheckers/CollisionDetectionValidity.h"

//neighborhood finder includes
#include "NeighborhoodFinders/BruteForceNF.h"
#include "NeighborhoodFinders/RadiusNF.h"

//sampler includes
#include "Samplers/UniformRandomSampler.h"

//extenders includes
#include "Extenders/KinodynamicExtender.h"
#include "Extenders/MixExtender.h"

//metric includes
#include "Metrics/NumNodesMetric.h"

//map evaluator includes
#include "MapEvaluators/ComposeEvaluator.h"
#include "MapEvaluators/ConditionalEvaluator.h"
#include "MapEvaluators/RRTQuery.h"
#include "MapEvaluators/TimeEvaluator.h"

//mp strategies includes
#include "MPStrategies/BasicRRTStrategy.h"
#include "MPStrategies/DynamicDomainRRT.h"
#include "MPStrategies/DynamicRegionRRT.h"
#include "MPStrategies/Syclop.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief Defines available methods in the Motion Planning Universe for State
///
/// StateTraits is a type class which defines the motion planning universe. We
/// construct our methods through a factory design pattern, and thus this states
/// all available classes within an abstraction that you can use in the system.
/// Essentially the important types are, the CfgType or the @cspace abstraction
/// class, the WeightType or the edge type of the graph, and a type list for
/// each algorithm abstraction --- here you only need to define what you need,
/// as extraneous methods in the type class imply longer compile times.
////////////////////////////////////////////////////////////////////////////////
struct StateTraits {

  typedef State CfgType;
  typedef StateEdge<State> WeightType;
  typedef State& CfgRef;

  typedef MPProblem<StateTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    ExperimentalDistance<StateTraits>,
    WeightedEuclideanDistance<StateTraits>,
    ReachabilityDistance<StateTraits>
      > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    AlwaysTrueValidity<StateTraits>,
    CollisionDetectionValidity<StateTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BruteForceNF<StateTraits>,
    RadiusNF<StateTraits>
    > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    UniformRandomSampler<StateTraits>
          > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
      > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
    KinodynamicExtender<StateTraits>,
    MixExtender<StateTraits>
      > ExtenderMethodList;

  //types of path smoothing available in our world
  typedef boost::mpl::list<
      > PathModifierMethodList;


  //types of connectors available in our world
  typedef boost::mpl::list<
          > ConnectorMethodList;

  //types of metrics available in our world
  typedef boost::mpl::list<
    NumNodesMetric<StateTraits>
      > MetricMethodList;


  //types of map evaluators available in our world
  typedef boost::mpl::list<
    ComposeEvaluator<StateTraits>,
    ConditionalEvaluator<StateTraits>,
    RRTQuery<StateTraits>,
    TimeEvaluator<StateTraits>
      > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    BasicRRTStrategy<StateTraits>,
    DynamicDomainRRT<StateTraits>,
    DynamicRegionRRT<StateTraits>,
    Syclop<StateTraits>
      > MPStrategyMethodList;
};

#endif
