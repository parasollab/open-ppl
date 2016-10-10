#ifndef STATE_TRAITS_H_
#define STATE_TRAITS_H_

#include "MPProblem/ConfigurationSpace/State.h"
#include "MPProblem/ConfigurationSpace/StateEdge.h"

//distance metric includes
#include "PlanningLibrary/DistanceMetrics/ExperimentalDistance.h"
#include "PlanningLibrary/DistanceMetrics/WeightedEuclideanDistance.h"

//validity checker includes
#include "PlanningLibrary/ValidityCheckers/CollisionDetectionValidity.h"

//neighborhood finder includes
#include "PlanningLibrary/NeighborhoodFinders/BruteForceNF.h"
#include "PlanningLibrary/NeighborhoodFinders/RadiusNF.h"

//extenders includes
#include "PlanningLibrary/Extenders/KinodynamicExtender.h"
#include "PlanningLibrary/Extenders/MixExtender.h"

//metric includes
#include "PlanningLibrary/Metrics/NumNodesMetric.h"

//map evaluator includes
#include "PlanningLibrary/MapEvaluators/ComposeEvaluator.h"
#include "PlanningLibrary/MapEvaluators/ConditionalEvaluator.h"

#include "PlanningLibrary/MapEvaluators/RRTQuery.h"

//mp strategies includes
#include "PlanningLibrary/MPStrategies/BasicRRTStrategy.h"
#include "PlanningLibrary/MPStrategies/DynamicDomainRRT.h"
#include "PlanningLibrary/MPStrategies/DynamicRegionRRT.h"

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
    WeightedEuclideanDistance<StateTraits>
      > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    CollisionDetectionValidity<StateTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BruteForceNF<StateTraits>,
    RadiusNF<StateTraits>
    > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
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
    RRTQuery<StateTraits>
      > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    BasicRRTStrategy<StateTraits>,
    DynamicDomainRRT<StateTraits>,
    DynamicRegionRRT<StateTraits>
      > MPStrategyMethodList;
};

#endif
