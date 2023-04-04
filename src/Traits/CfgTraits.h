#ifndef PMPL_CFG_TRAITS_H_
#define PMPL_CFG_TRAITS_H_

#include "MPLibrary/GoalTracker.h"
#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/MPSolution.h"
#include "MPLibrary/MPTools/MPTools.h"

#include "ConfigurationSpace/LocalObstacleMap.h"
#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupPath.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "ConfigurationSpace/Path.h"
#include "ConfigurationSpace/Weight.h"

//distance metric includes
#include "MPLibrary/DistanceMetrics/EuclideanDistance.h"
#include "MPLibrary/DistanceMetrics/ManhattanDistance.h"
#include "MPLibrary/DistanceMetrics/MinkowskiDistance.h"
#include "MPLibrary/DistanceMetrics/WorkspaceTranslationDistance.h"
#include "MPLibrary/DistanceMetrics/RMSDDistance.h"
#include "MPLibrary/DistanceMetrics/WeightedEuclideanDistance.h"
#include "MPLibrary/DistanceMetrics/ScaledEuclideanDistance.h"
#include "MPLibrary/DistanceMetrics/LPSweptDistance.h"
#include "MPLibrary/DistanceMetrics/BinaryLPSweptDistance.h"


//validity checker includes
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "MPLibrary/ValidityCheckers/AlwaysTrueValidity.h"
#include "MPLibrary/ValidityCheckers/ComposeValidity.h"
#include "MPLibrary/ValidityCheckers/ComposeCollision.h"
#include "MPLibrary/ValidityCheckers/TerrainValidityChecker.h"

//neighborhood finder includes
#include "MPLibrary/NeighborhoodFinders/BruteForceNF.h"

//sampler includes
#include "MPLibrary/Samplers/BridgeTestSampler.h"
#include "MPLibrary/Samplers/MixSampler.h"
#include "MPLibrary/Samplers/ObstacleBasedSampler.h"
#include "MPLibrary/Samplers/UniformRandomSampler.h"
#include "MPLibrary/Samplers/GaussianSampler.h"

//local planner includes
#include "MPLibrary/LocalPlanners/StraightLine.h"

//extenders includes
#include "MPLibrary/Extenders/BasicExtender.h"

//path smoothing includes

//connector includes
#include "MPLibrary/Connectors/NeighborhoodConnector.h"
#include "MPLibrary/Connectors/CCsConnector.h"
#include "MPLibrary/Connectors/RewireConnector.h"

//metric includes
#include "MPLibrary/Metrics/NumNodesMetric.h"
#include "MPLibrary/Metrics/NumEdgesMetric.h"
#include "MPLibrary/Metrics/TimeMetric.h"

//map evaluator includes
#include "MPLibrary/MapEvaluators/CBSQuery.h"
#include "MPLibrary/MapEvaluators/GroupQuery.h"
#include "MPLibrary/MapEvaluators/ComposeEvaluator.h"
#include "MPLibrary/MapEvaluators/ConditionalEvaluator.h"
#include "MPLibrary/MapEvaluators/LazyQuery.h"
#include "MPLibrary/MapEvaluators/PrintMapEvaluation.h"
#include "MPLibrary/MapEvaluators/QueryMethod.h"
#include "MPLibrary/MapEvaluators/SIPPMethod.h"
#include "MPLibrary/MapEvaluators/TimeEvaluator.h"

//mp strategies includes
#include "MPLibrary/MPStrategies/AdaptiveRRT.h"
#include "MPLibrary/MPStrategies/GroupPRM.h"
#include "MPLibrary/MPStrategies/EET.h"
#include "MPLibrary/MPStrategies/BasicPRM.h"
#include "MPLibrary/MPStrategies/BasicRRTStrategy.h"
#include "MPLibrary/MPStrategies/DynamicRegionRRT.h"
#include "MPLibrary/MPStrategies/DynamicRegionsPRM.h"
#include "MPLibrary/MPStrategies/GroupDecoupledStrategy.h"
#include "MPLibrary/MPStrategies/GroupStrategyMethod.h"
#include "MPLibrary/MPStrategies/TogglePRMStrategy.h"
#include "MPLibrary/MPStrategies/ValidationStrategy.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief Defines available methods in the Motion Planning Universe for Cfg
/// @tparam C Cfg type
/// @tparam W Weight type
///
/// MPTraits is a type class which defines the motion planning universe. We
/// construct our methods through a factory design pattern, and thus this states
/// all available classes within an abstraction that you can use in the system.
/// Essentially, the important types are the CfgType or the @cspace abstraction
/// class, the WeightType or the edge type of the graph, and a type list for
/// each algorithm abstraction --- here you only need to define what you need,
/// as extraneous methods in the type class imply longer compile times.
////////////////////////////////////////////////////////////////////////////////
template <typename C, typename W = DefaultWeight<C>>
struct MPTraits {

  typedef C                               CfgType;
  typedef W                               WeightType;
  typedef GenericStateGraph<C, W>         RoadmapType;
  typedef PathType<MPTraits>              Path;
  typedef MPLibraryType<MPTraits>         MPLibrary;
  typedef MPSolutionType<MPTraits>        MPSolution;
  typedef MPToolsType<MPTraits>           MPTools;
  typedef LocalObstacleMapType<MPTraits>  LocalObstacleMap;
  typedef GoalTrackerType<MPTraits>       GoalTracker;

  typedef GroupCfg<RoadmapType>                          GroupCfgType;
  typedef GroupLocalPlan<RoadmapType>                    GroupWeightType;
  typedef GroupRoadmap<GroupCfgType, GroupWeightType>    GroupRoadmapType;
  typedef GroupPath<MPTraits>                            GroupPathType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    EuclideanDistance<MPTraits>,
    ManhattanDistance<MPTraits>,
    MinkowskiDistance<MPTraits>,
    WorkspaceTranslationDistance<MPTraits>,
    RMSDDistance<MPTraits>,
    WeightedEuclideanDistance<MPTraits>,
    LPSweptDistance<MPTraits>,
    BinaryLPSweptDistance<MPTraits>,
    ScaledEuclideanDistance<MPTraits>
      > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    CollisionDetectionValidity<MPTraits>,
    AlwaysTrueValidity<MPTraits>,
    ComposeValidity<MPTraits>,
    ComposeCollision<MPTraits>,
    TerrainValidityChecker<MPTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BruteForceNF<MPTraits>
      > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    BridgeTestSampler<MPTraits>,
    MixSampler<MPTraits>,
    ObstacleBasedSampler<MPTraits>,
    UniformRandomSampler<MPTraits>,
    GaussianSampler<MPTraits>
      > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
    StraightLine<MPTraits>
      > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
    BasicExtender<MPTraits>
      > ExtenderMethodList;

  //types of path smoothing available in our world
  typedef boost::mpl::list<
      > PathModifierMethodList;


  //types of connectors available in our world
  typedef boost::mpl::list<
    NeighborhoodConnector<MPTraits>,
    CCsConnector<MPTraits>,
    RewireConnector<MPTraits>
      > ConnectorMethodList;

  //types of metrics available in our world
  typedef boost::mpl::list<
    NumNodesMetric<MPTraits>,
    NumEdgesMetric<MPTraits>,
    TimeMetric<MPTraits>
      > MetricMethodList;


  //types of map evaluators available in our world
  typedef boost::mpl::list<
    CBSQuery<MPTraits>,
    GroupQuery<MPTraits>,
    ComposeEvaluator<MPTraits>,
    ConditionalEvaluator<MPTraits>,
    LazyQuery<MPTraits>,
    PrintMapEvaluation<MPTraits>,
    QueryMethod<MPTraits>,
    SIPPMethod<MPTraits>,
    TimeEvaluator<MPTraits>
      > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    AdaptiveRRT<MPTraits>,
    GroupPRM<MPTraits>,
    BasicPRM<MPTraits>,
    BasicRRTStrategy<MPTraits>,
    DynamicRegionRRT<MPTraits>,
    DynamicRegionsPRM<MPTraits>,
    EET<MPTraits>,
    GroupDecoupledStrategy<MPTraits>,
    GroupStrategyMethod<MPTraits>,
    TogglePRMStrategy<MPTraits>,
    ValidationStrategy<MPTraits>
      > MPStrategyMethodList;


};

#endif
