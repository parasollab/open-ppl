#ifndef PMPL_CFG_TRAITS_H_
#define PMPL_CFG_TRAITS_H_

#include "MPLibrary/GoalTracker.h"
#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/MPSolution.h"
#include "MPLibrary/MPTools/MPTools.h"

#include "ConfigurationSpace/LocalObstacleMap.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupPath.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "ConfigurationSpace/Path.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "ConfigurationSpace/Weight.h"

//distance metric includes
#include "MPLibrary/DistanceMetrics/MinkowskiDistance.h"

//validity checker includes
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "MPLibrary/ValidityCheckers/AlwaysTrueValidity.h"

//neighborhood finder includes
#include "MPLibrary/NeighborhoodFinders/BruteForceNF.h"

//sampler includes
#include "MPLibrary/Samplers/UniformRandomSampler.h"

//local planner includes

//extenders includes
#include "MPLibrary/Extenders/BasicExtender.h"

//path smoothing includes

//connector includes

//metric includes

//map evaluator includes

//mp strategies includes
#include "MPLibrary/MPStrategies/BasicRRTStrategy.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief Defines available methods in the Motion Planning Universe for Cfg
/// @tparam C Cfg type
/// @tparam W Weight type
///
/// MPTraits is a type class which defines the motion planning universe. We
/// construct our methods through a factory design pattern, and thus this states
/// all available classes within an abstraction that you can use in the system.
/// Essentially the important types are, the CfgType or the @cspace abstraction
/// class, the WeightType or the edge type of the graph, and a type list for
/// each algorithm abstraction --- here you only need to define what you need,
/// as extraneous methods in the type class imply longer compile times.
////////////////////////////////////////////////////////////////////////////////
template <typename C, typename W = DefaultWeight<C>>
struct MPTraits {

  typedef C                               CfgType;
  typedef W                               WeightType;
  typedef RoadmapGraph<C, W>              RoadmapType;
  typedef PathType<MPTraits>              Path;
  typedef MPLibraryType<MPTraits>         MPLibrary;
  typedef MPSolutionType<MPTraits>        MPSolution;
  typedef MPToolsType<MPTraits>           MPTools;
  typedef LocalObstacleMapType<MPTraits>  LocalObstacleMap;
  typedef GoalTrackerType<MPTraits>       GoalTracker;

  typedef GroupLocalPlan<CfgType>                    GroupWeightType;
  typedef GroupRoadmap<GroupCfg, GroupWeightType>    GroupRoadmapType;
  typedef GroupPath<MPTraits>                        GroupPathType;
  typedef GroupCfg                                   GroupCfgType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    MinkowskiDistance<MPTraits>
      > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    CollisionDetectionValidity<MPTraits>,
    AlwaysTrueValidity<MPTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BruteForceNF<MPTraits>
      > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    UniformRandomSampler<MPTraits>
      > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
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
      > ConnectorMethodList;

  //types of metrics available in our world
  typedef boost::mpl::list<
      > MetricMethodList;


  //types of map evaluators available in our world
  typedef boost::mpl::list<
      > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    BasicRRTStrategy<MPTraits>
      > MPStrategyMethodList;
};

#endif
