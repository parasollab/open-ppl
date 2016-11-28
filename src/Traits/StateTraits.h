#ifndef STATE_TRAITS_H_
#define STATE_TRAITS_H_

#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/MPSolution.h"
#include "MPLibrary/MPTask.h"

#include "ConfigurationSpace/State.h"
#include "ConfigurationSpace/StateEdge.h"

//distance metric includes
#include "MPLibrary/DistanceMetrics/ExperimentalDistance.h"
#include "MPLibrary/DistanceMetrics/WeightedEuclideanDistance.h"

//validity checker includes
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"

//neighborhood finder includes
#include "MPLibrary/NeighborhoodFinders/BruteForceNF.h"
#include "MPLibrary/NeighborhoodFinders/RadiusNF.h"

//extenders includes
#include "MPLibrary/Extenders/KinodynamicExtender.h"
#include "MPLibrary/Extenders/MixExtender.h"

//metric includes
#include "MPLibrary/Metrics/NumNodesMetric.h"

//map evaluator includes
#include "MPLibrary/MapEvaluators/ComposeEvaluator.h"
#include "MPLibrary/MapEvaluators/ConditionalEvaluator.h"

#include "MPLibrary/MapEvaluators/RRTQuery.h"

//mp strategies includes
#include "MPLibrary/MPStrategies/BasicRRTStrategy.h"
#include "MPLibrary/MPStrategies/DynamicDomainRRT.h"
#include "MPLibrary/MPStrategies/DynamicRegionRRT.h"

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

  typedef MPLibraryType<StateTraits> MPLibrary;
  typedef MPSolution<StateTraits>    MPSolution;
  typedef MPTaskType<StateTraits>    MPTask;

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
