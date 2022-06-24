#ifndef PMPL_CFG_TRAITS_H_
#define PMPL_CFG_TRAITS_H_

#include "MPLibrary/GoalTracker.h"
#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/MPSolution.h"
#include "MPLibrary/MPTools/MPTools.h"

#include "ConfigurationSpace/LocalObstacleMap.h"
#include "ConfigurationSpace/CompositeGraph.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupPath.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "ConfigurationSpace/Path.h"
#include "ConfigurationSpace/Weight.h"
#include "ConfigurationSpace/GenericStateGraph.h"

//distance metric includes
#include "MPLibrary/DistanceMetrics/EuclideanDistance.h"
#include "MPLibrary/DistanceMetrics/ManhattanDistance.h"
#include "MPLibrary/DistanceMetrics/MinkowskiDistance.h"

//validity checker includes
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "MPLibrary/ValidityCheckers/AlwaysTrueValidity.h"
#include "MPLibrary/ValidityCheckers/ComposeValidity.h"
#include "MPLibrary/ValidityCheckers/ComposeCollision.h"

//neighborhood finder includes
#include "MPLibrary/NeighborhoodFinders/BruteForceNF.h"
#include "MPLibrary/NeighborhoodFinders/OptimalNF.h"
#include "MPLibrary/NeighborhoodFinders/RadiusNF.h"

//sampler includes
#include "MPLibrary/Samplers/ObstacleBasedSampler.h"
#include "MPLibrary/Samplers/UniformRandomSampler.h"

//local planner includes
#include "MPLibrary/LocalPlanners/StraightLine.h"

//extenders includes
#include "MPLibrary/Extenders/BasicExtender.h"
#include "MPLibrary/Extenders/DiscreteExtender.h"

//path smoothing includes

//connector includes
#include "MPLibrary/Connectors/NeighborhoodConnector.h"
#include "MPLibrary/Connectors/RewireConnector.h"
#include "MPLibrary/Connectors/CCsConnector.h"

//metric includes
#include "MPLibrary/Metrics/NumNodesMetric.h"

//map evaluator includes
#include "MPLibrary/MapEvaluators/CBSQuery.h"
#include "MPLibrary/MapEvaluators/ComposeEvaluator.h"
#include "MPLibrary/MapEvaluators/ConditionalEvaluator.h"
#include "MPLibrary/MapEvaluators/DRRT.h"
#include "MPLibrary/MapEvaluators/GroupDecoupledQuery.h"
#include "MPLibrary/MapEvaluators/GroupQuery.h"
#include "MPLibrary/MapEvaluators/LazyQuery.h"
#include "MPLibrary/MapEvaluators/SIPPMethod.h"
#include "MPLibrary/MapEvaluators/QueryMethod.h"
#include "MPLibrary/MapEvaluators/TimeEvaluator.h"

//mp strategies includes
#include "MPLibrary/MPStrategies/AdaptiveRRT.h"
#include "MPLibrary/MPStrategies/BasicPRM.h"
#include "MPLibrary/MPStrategies/BasicRRTStrategy.h"
#include "MPLibrary/MPStrategies/DynamicRegionRRT.h"
#include "MPLibrary/MPStrategies/DynamicRegionsPRM.h"
#include "MPLibrary/MPStrategies/GroupDecoupledStrategy.h"
#include "MPLibrary/MPStrategies/GroupPRM.h"
#include "MPLibrary/MPStrategies/GroupRRTStrategy.h"
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
    MinkowskiDistance<MPTraits>
      > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    CollisionDetectionValidity<MPTraits>,
    AlwaysTrueValidity<MPTraits>,
    ComposeValidity<MPTraits>,
    ComposeCollision<MPTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BruteForceNF<MPTraits>,
    OptimalNF<MPTraits>,
    RadiusNF<MPTraits>
      > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    ObstacleBasedSampler<MPTraits>,
    UniformRandomSampler<MPTraits>
      > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
    StraightLine<MPTraits>
      > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
    BasicExtender<MPTraits>,
    DiscreteExtender<MPTraits>
      > ExtenderMethodList;

  //types of path smoothing available in our world
  typedef boost::mpl::list<
      > PathModifierMethodList;


  //types of connectors available in our world
  typedef boost::mpl::list<
    NeighborhoodConnector<MPTraits>,
    RewireConnector<MPTraits>,
    CCsConnector<MPTraits>
      > ConnectorMethodList;

  //types of metrics available in our world
  typedef boost::mpl::list<
    NumNodesMetric<MPTraits>
      > MetricMethodList;


  //types of map evaluators available in our world
  typedef boost::mpl::list<
    CBSQuery<MPTraits>,
    ComposeEvaluator<MPTraits>,
    ConditionalEvaluator<MPTraits>,
    DRRT<MPTraits>,
    LazyQuery<MPTraits>,
    GroupDecoupledQuery<MPTraits>,
    GroupQuery<MPTraits>,
    QueryMethod<MPTraits>,
    SIPPMethod<MPTraits>,
    TimeEvaluator<MPTraits>
      > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    AdaptiveRRT<MPTraits>,
    BasicPRM<MPTraits>,
    BasicRRTStrategy<MPTraits>,
    DynamicRegionRRT<MPTraits>,
    DynamicRegionsPRM<MPTraits>,
    GroupDecoupledStrategy<MPTraits>,
    GroupPRM<MPTraits>,
    GroupRRTStrategy<MPTraits>,
    GroupStrategyMethod<MPTraits>,
    TogglePRMStrategy<MPTraits>,
    ValidationStrategy<MPTraits>
      > MPStrategyMethodList;


};

#endif
