#ifndef CFG_TRAITS_H_
#define CFG_TRAITS_H_

#include "Cfg/State.h"
#include "Edges/StateEdge.h"

//distance metric includes
#include "DistanceMetrics/EuclideanDistance.h"

//validity checker includes
#include "ValidityCheckers/AlwaysTrueValidity.h"
#include "ValidityCheckers/CollisionDetectionValidity.h"

//neighborhood finder includes
#include "NeighborhoodFinders/BruteForceNF.h"
#include "NeighborhoodFinders/OptimalNF.h"
#include "NeighborhoodFinders/RadiusNF.h"
#include "NeighborhoodFinders/RandomNF.h"

//sampler includes
#include "Samplers/BridgeTestSampler.h"
#include "Samplers/GaussianSampler.h"
#include "Samplers/GridSampler.h"
#include "Samplers/MedialAxisSampler.h"
#include "Samplers/MixSampler.h"
#include "Samplers/ObstacleBasedSampler.h"
#include "Samplers/UniformMedialAxisSampler.h"
#include "Samplers/UniformObstacleBasedSampler.h"
#include "Samplers/UniformRandomSampler.h"

//local planner includes
#include "LocalPlanners/StraightLine.h"

//extenders includes
#include "Extenders/KinodynamicExtender.h"

//path smoothing includes
#include "PathModifiers/ShortcuttingPathModifier.h"

//connector includes
#include "Connectors/CCsConnector.h"
#include "Connectors/NeighborhoodConnector.h"
#include "Connectors/RewireConnector.h"

//metric includes
#include "Metrics/NumEdgesMetric.h"
#include "Metrics/NumNodesMetric.h"
#include "Metrics/TimeMetric.h"

//map evaluator includes
#include "MapEvaluators/ComposeEvaluator.h"
#include "MapEvaluators/ConditionalEvaluator.h"
#include "MapEvaluators/PrintMapEvaluation.h"

//mp strategies includes
#include "MPStrategies/KinodynamicRRTStrategy.h"

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
  typedef StateEdge WeightType;
  typedef State& CfgRef;

  typedef MPProblem<StateTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    EuclideanDistance<StateTraits>
      > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    AlwaysTrueValidity<StateTraits>,
    CollisionDetectionValidity<StateTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BruteForceNF<StateTraits>,
    OptimalNF<StateTraits>,
    RadiusNF<StateTraits>,
    RandomNF<StateTraits>
    > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    BridgeTestSampler<StateTraits>,
    GaussianSampler<StateTraits>,
    GridSampler<StateTraits>,
    MedialAxisSampler<StateTraits>,
    MixSampler<StateTraits>,
    ObstacleBasedSampler<StateTraits>,
    UniformMedialAxisSampler<StateTraits>,
    UniformObstacleBasedSampler<StateTraits>,
    UniformRandomSampler<StateTraits>
      > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
    StraightLine<StateTraits>
      > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
    KinodynamicExtender<StateTraits>
      > ExtenderMethodList;

  //types of path smoothing available in our world
  typedef boost::mpl::list<
    ShortcuttingPathModifier<StateTraits>
      > PathModifierMethodList;


  //types of connectors available in our world
  typedef boost::mpl::list<
    CCsConnector<StateTraits>,
    NeighborhoodConnector<StateTraits>,
    RewireConnector<StateTraits>
      > ConnectorMethodList;

  //types of metrics available in our world
  typedef boost::mpl::list<
    NumEdgesMetric<StateTraits>,
    NumNodesMetric<StateTraits>,
    TimeMetric<StateTraits>
      > MetricMethodList;


  //types of map evaluators available in our world
  typedef boost::mpl::list<
    ComposeEvaluator<StateTraits>,
    ConditionalEvaluator<StateTraits>,
    PrintMapEvaluation<StateTraits>
      > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    KinodynamicRRTStrategy<StateTraits>
      > MPStrategyMethodList;
};

#endif
