#ifndef PMPL_AVAILABLE_METHODS_H_
#define PMPL_AVAILABLE_METHODS_H_

#include <boost/preprocessor/seq.hpp>
#define BOOST_PP_SEQ_ENUM_0

// Connectors
#define CCS_CONNECTOR_AVAILABLE 1
#define NEIGHBOR_CONNECTOR_AVAILABLE 1
#define REWIRE_CONNECTOR_AVAILABLE 1

// DistanceMetrics
#define BIN_LP_SWEPT_AVAILABLE 1
#define LP_SWEPT_AVAILABLE 1
#define EUCLIDEAN_AVAILABLE 1
#define SCALED_EUCLIDEAN_AVAILABLE 0
#define WEIGHTED_EUCLIDEAN_AVAILABLE 1
#define MINKOWSKI_AVAILABLE 1
#define MANHATTAN_AVAILABLE 1
#define RMSD_AVAILABLE 1
#define TOPOLOGICAL_DISTANCE_AVAILABLE 1
#define WORKSPACE_TRANSLATION_AVAILABLE 1

// EdgeValidityCheckers
#define INTERMEDIATES_EVC_AVAILABLE 1

// Extenders
#define BASIC_EXTENDER_AVAILABLE 1
#define ROTATION_THEN_TRANSLATION_AVAILABLE 1

// LocalPlanners
#define STRAIGHT_LINE_AVAILABLE 1
#define HIERARCHICAL_LP_AVAILABLE 1

// MapEvaluators
#define CBS_QUERY_AVAILABLE 1
#define CLEARANCE_QUERY_AVAILABLE 1
#define COLLISION_EVAL_AVAILABLE 1
#define COMPOSE_EVAL_AVAILABLE 1
#define CONDITIONAL_EVAL_AVAILABLE 1
#define GROUP_QUERY_AVAILABLE 1
#define LAZY_QUERY_AVAILABLE 1
#define MIN_DIST_EVAL_AVAILABLE 1
#define NEGATE_EVAL_AVAILABLE 1
#define PATH_EVAL_AVAILABLE 1
#define PRINT_MAP_AVAILABLE 1
#define QUERY_METHOD_AVAILABLE 1
#define SIPP_METHOD_AVAILABLE 1
#define TIME_EVAL_AVAILABLE 1

// Metrics
#define NUM_NODES_AVAILABLE 1
#define NUM_EDGES_AVAILABLE 1
#define TIME_METRIC_AVAILABLE 1

// MPStrategies
#define ADAPTIVE_RRT_AVAILABLE 1
#define BASIC_PRM_AVAILABLE 1
#define BASIC_RRT_AVAILABLE 1
#define DYNAMIC_DOMAIN_RRT_AVAILABLE 1
#define DYNAMIC_REGION_RRT_AVAILABLE 1
#define DYNAMIC_REGION_PRM_AVAILABLE 1
#define EET_AVAILABLE 1
#define GROUP_DECOUPLED_STRAT_AVAILABLE 1
#define GROUP_PRM_AVAILABLE 1
#define MODIFY_PATH_AVAILABLE 1
#define PATH_STRATEGY_AVAILABLE 0
#define SYCLOP_AVAILABLE 1
#define TOGGLE_PRM_AVAILABLE 1
#define WRENCH_ACCESS_STRAT_AVAILABLE 1

// NeighborhoodFinders
#define BRUTE_FORCE_NF_AVAILABLE 1
#define RADIUS_NF_AVAILABLE 1
#define RANDOM_NF_AVAILABLE 1

// PathModifiers
#define SHORTCUTTING_AVAILABLE 1

// Samplers
#define BRIDGE_TEST_AVAILABLE 1
#define GAUSSIAN_AVAILABLE 1
#define MIX_SAMPLER_AVAILABLE 1
#define OBSTACLE_BASED_SAMPLER_AVAILABLE 1
#define UNIFORM_RANDOM_AVAILABLE 1
#define UNIFORM_OBSTACLE_AVAILABLE 0

// ValidityCheckers
#define ALWAYS_TRUE_AVAILABLE 1
#define COMPOSE_CD_AVAILABLE 1
#define COMPOSE_VC_AVAILABLE 1
#define NODE_CLEARANCE_VC_AVAILABLE 1
#define OBSTACLE_CLEARANCE_VC_AVAILABLE 1
#define TERRAIN_VC_AVAILABLE 1
#define TOPOLOGICAL_MAP_VC_AVAILABLE 1

/******************************* Connectors ***********************************/
#if CCS_CONNECTOR_AVAILABLE
#include "MPLibrary/Connectors/CCsConnector.h"
#endif
#if NEIGHBOR_CONNECTOR_AVAILABLE
#include "MPLibrary/Connectors/NeighborhoodConnector.h"
#endif
#if REWIRE_CONNECTOR_AVAILABLE
#include "MPLibrary/Connectors/RewireConnector.h"
#endif

#define CONNECTOR_CLASSES                                     \
  ((CCsConnector, CCS_CONNECTOR_AVAILABLE))(                  \
      (NeighborhoodConnector, NEIGHBOR_CONNECTOR_AVAILABLE))( \
      (RewireConnector, REWIRE_CONNECTOR_AVAILABLE))

/***************************** DistanceMetrics ********************************/
#if BIN_LP_SWEPT_AVAILABLE
#include "MPLibrary/DistanceMetrics/BinaryLPSweptDistance.h"
#endif
#if LP_SWEPT_AVAILABLE
#include "MPLibrary/DistanceMetrics/LPSweptDistance.h"
#endif
#if MINKOWSKI_AVAILABLE
#include "MPLibrary/DistanceMetrics/MinkowskiDistance.h"
#endif
#if EUCLIDEAN_AVAILABLE
#include "MPLibrary/DistanceMetrics/EuclideanDistance.h"
#endif
#if SCALED_EUCLIDEAN_AVAILABLE
#include "MPLibrary/DistanceMetrics/ScaledEuclideanDistance.h"
#endif
#if WEIGHTED_EUCLIDEAN_AVAILABLE
#include "MPLibrary/DistanceMetrics/WeightedEuclideanDistance.h"
#endif
#if MANHATTAN_AVAILABLE
#include "MPLibrary/DistanceMetrics/ManhattanDistance.h"
#endif
#if RMSD_AVAILABLE
#include "MPLibrary/DistanceMetrics/RMSDDistance.h"
#endif
#if TOPOLOGICAL_DISTANCE_AVAILABLE
#include "MPLibrary/DistanceMetrics/TopologicalDistance.h"
#endif
#if WORKSPACE_TRANSLATION_AVAILABLE
#include "MPLibrary/DistanceMetrics/WorkspaceTranslationDistance.h"
#endif

#define DM_CLASSES                                                \
  ((BinaryLPSweptDistance, BIN_LP_SWEPT_AVAILABLE))(              \
      (LPSweptDistance, LP_SWEPT_AVAILABLE))(                     \
      (MinkowskiDistance, MINKOWSKI_AVAILABLE))(                  \
      (EuclideanDistance, EUCLIDEAN_AVAILABLE))(                  \
      (ScaledEuclideanDistance, SCALED_EUCLIDEAN_AVAILABLE))(     \
      (WeightedEuclideanDistance, WEIGHTED_EUCLIDEAN_AVAILABLE))( \
      (ManhattanDistance, MANHATTAN_AVAILABLE))(                  \
      (RMSDDistance, RMSD_AVAILABLE))(                            \
      (TopologicalDistance, TOPOLOGICAL_DISTANCE_AVAILABLE))(     \
      (WorkspaceTranslationDistance, WORKSPACE_TRANSLATION_AVAILABLE))

/************************** EdgeValidityCheckers ******************************/
#if INTERMEDIATES_EVC_AVAILABLE
#include "MPLibrary/EdgeValidityCheckers/IntermediatesEdgeValidityChecker.h"
#endif

#define EVC_CLASSES \
  ((IntermediatesEdgeValidityChecker, INTERMEDIATES_EVC_AVAILABLE))

/******************************* Extenders ************************************/
#if BASIC_EXTENDER_AVAILABLE
#include "MPLibrary/Extenders/BasicExtender.h"
#endif
#if ROTATION_THEN_TRANSLATION_AVAILABLE
#include "MPLibrary/Extenders/RotationThenTranslation.h"
#endif

#define EXT_CLASSES                            \
  ((BasicExtender, BASIC_EXTENDER_AVAILABLE))( \
      (RotationThenTranslation, ROTATION_THEN_TRANSLATION_AVAILABLE))
// ... and so on ...

/***************************** LocalPlanners **********************************/
#if STRAIGHT_LINE_AVAILABLE
#include "MPLibrary/LocalPlanners/StraightLine.h"
#endif
#if HIERARCHICAL_LP_AVAILABLE
#include "MPLibrary/LocalPlanners/HierarchicalLP.h"
#endif

#define LP_CLASSES                           \
  ((StraightLine, STRAIGHT_LINE_AVAILABLE))( \
      (HierarchicalLP, HIERARCHICAL_LP_AVAILABLE))
// ... and so on ...

/***************************** MapEvaluators **********************************/
#if CBS_QUERY_AVAILABLE
#include "MPLibrary/MapEvaluators/CBSQuery.h"
#endif
#if CLEARANCE_QUERY_AVAILABLE
#include "MPLibrary/MapEvaluators/ClearanceQuery.h"
#endif
#if CONDITIONAL_EVAL_AVAILABLE
#include "MPLibrary/MapEvaluators/ConditionalEvaluator.h"
#endif
#if COLLISION_EVAL_AVAILABLE
#include "MPLibrary/MapEvaluators/CollisionEvaluator.h"
#endif
#if COMPOSE_EVAL_AVAILABLE
#include "MPLibrary/MapEvaluators/ComposeEvaluator.h"
#endif
#if GROUP_QUERY_AVAILABLE
#include "MPLibrary/MapEvaluators/GroupQuery.h"
#endif
#if LAZY_QUERY_AVAILABLE
#include "MPLibrary/MapEvaluators/LazyQuery.h"
#endif
#if MIN_DIST_EVAL_AVAILABLE
#include "MPLibrary/MapEvaluators/MinimumDistanceEvaluator.h"
#endif
#if NEGATE_EVAL_AVAILABLE
#include "MPLibrary/MapEvaluators/NegateEvaluator.h"
#endif
#if PATH_EVAL_AVAILABLE
#include "MPLibrary/MapEvaluators/PathEvaluator.h"
#endif
#if PRINT_MAP_AVAILABLE
#include "MPLibrary/MapEvaluators/PrintMapEvaluation.h"
#endif
#if QUERY_METHOD_AVAILABLE
#include "MPLibrary/MapEvaluators/QueryMethod.h"
#endif
#if SIPP_METHOD_AVAILABLE
#include "MPLibrary/MapEvaluators/SIPPMethod.h"
#endif
#if TIME_EVAL_AVAILABLE
#include "MPLibrary/MapEvaluators/TimeEvaluator.h"
#endif

#define ME_CLASSES                                                             \
  ((CBSQuery, CBS_QUERY_AVAILABLE))(                                           \
      (ClearanceQuery, CLEARANCE_QUERY_AVAILABLE))(                            \
      (CollisionEvaluator, COLLISION_EVAL_AVAILABLE))(                         \
      (ComposeEvaluator, COMPOSE_EVAL_AVAILABLE))(                             \
      (ConditionalEvaluator, CONDITIONAL_EVAL_AVAILABLE))(                     \
      (GroupQuery, GROUP_QUERY_AVAILABLE))((LazyQuery, LAZY_QUERY_AVAILABLE))( \
      (MinimumDistanceEvaluator, MIN_DIST_EVAL_AVAILABLE))(                    \
      (NegateEvaluator, NEGATE_EVAL_AVAILABLE))(                               \
      (PathEvaluator, PATH_EVAL_AVAILABLE))(                                   \
      (PrintMapEvaluation, PRINT_MAP_AVAILABLE))(                              \
      (QueryMethod, QUERY_METHOD_AVAILABLE))(                                  \
      (SIPPMethod, SIPP_METHOD_AVAILABLE))(                                    \
      (TimeEvaluator, TIME_EVAL_AVAILABLE))

/******************************** Metrics *************************************/
#if NUM_NODES_AVAILABLE
#include "MPLibrary/Metrics/NumNodesMetric.h"
#endif
#if NUM_EDGES_AVAILABLE
#include "MPLibrary/Metrics/NumEdgesMetric.h"
#endif
#if TIME_METRIC_AVAILABLE
#include "MPLibrary/Metrics/TimeMetric.h"
#endif

#define METRIC_CLASSES                        \
  ((NumNodesMetric, NUM_NODES_AVAILABLE))(    \
      (NumEdgesMetric, NUM_EDGES_AVAILABLE))( \
      (TimeMetric, TIME_METRIC_AVAILABLE))

/***************************** MPStrategies ***********************************/
#if ADAPTIVE_RRT_AVAILABLE
#include "MPLibrary/MPStrategies/AdaptiveRRT.h"
#endif
#if BASIC_PRM_AVAILABLE
#include "MPLibrary/MPStrategies/BasicPRM.h"
#endif
#if BASIC_RRT_AVAILABLE
#include "MPLibrary/MPStrategies/BasicRRTStrategy.h"
#endif
#if DYNAMIC_DOMAIN_RRT_AVAILABLE
#include "MPLibrary/MPStrategies/DynamicDomainRRT.h"
#endif
#if DYNAMIC_REGION_RRT_AVAILABLE
#include "MPLibrary/MPStrategies/DynamicRegionRRT.h"
#endif
#if DYNAMIC_REGION_PRM_AVAILABLE
#include "MPLibrary/MPStrategies/DynamicRegionsPRM.h"
#endif
#if EET_AVAILABLE
#include "MPLibrary/MPStrategies/EET.h"
#endif
#if GROUP_DECOUPLED_STRAT_AVAILABLE
#include "MPLibrary/MPStrategies/GroupDecoupledStrategy.h"
#endif
#if GROUP_PRM_AVAILABLE
#include "MPLibrary/MPStrategies/GroupPRM.h"
#endif
#if MODIFY_PATH_AVAILABLE
#include "MPLibrary/MPStrategies/ModifyPath.h"
#endif
#if PATH_STRATEGY_AVAILABLE
#include "MPLibrary/MPStrategies/PathStrategy.h"
#endif
#if SYCLOP_AVAILABLE
#include "MPLibrary/MPStrategies/Syclop.h"
#endif
#if TOGGLE_PRM_AVAILABLE
#include "MPLibrary/MPStrategies/TogglePRMStrategy.h"
#endif
#if WRENCH_ACCESS_STRAT_AVAILABLE
#include "MPLibrary/MPStrategies/WrenchAccessibilityStrategy.h"
#endif

#define MPSTRATEGY_CLASSES                                                     \
  ((AdaptiveRRT, ADAPTIVE_RRT_AVAILABLE))((BasicPRM, BASIC_PRM_AVAILABLE))(    \
      (BasicRRTStrategy, BASIC_RRT_AVAILABLE))(                                \
      (DynamicDomainRRT, DYNAMIC_DOMAIN_RRT_AVAILABLE))(                       \
      (DynamicRegionRRT, DYNAMIC_REGION_RRT_AVAILABLE))((                      \
      DynamicRegionsPRM, DYNAMIC_REGION_PRM_AVAILABLE))((EET, EET_AVAILABLE))( \
      (GroupDecoupledStrategy, GROUP_DECOUPLED_STRAT_AVAILABLE))(              \
      (GroupPRM, GROUP_PRM_AVAILABLE))((ModifyPath, MODIFY_PATH_AVAILABLE))(   \
      (PathStrategy, PATH_STRATEGY_AVAILABLE))((Syclop, SYCLOP_AVAILABLE))(    \
      (TogglePRMStrategy, TOGGLE_PRM_AVAILABLE))(                              \
      (WrenchAccessibilityStrategy, WRENCH_ACCESS_STRAT_AVAILABLE))

/************************** NeighborhoodFinders *******************************/
#if BRUTE_FORCE_NF_AVAILABLE
#include "MPLibrary/NeighborhoodFinders/BruteForceNF.h"
#endif
#if RADIUS_NF_AVAILABLE
#include "MPLibrary/NeighborhoodFinders/RadiusNF.h"
#endif
#if RANDOM_NF_AVAILABLE
#include "MPLibrary/NeighborhoodFinders/RandomNF.h"
#endif

#define NF_CLASSES                                                             \
  ((BruteForceNF, BRUTE_FORCE_NF_AVAILABLE))((RadiusNF, RADIUS_NF_AVAILABLE))( \
      (RandomNF, RANDOM_NF_AVAILABLE))

/***************************** PathModifiers **********************************/
#if SHORTCUTTING_AVAILABLE
#include "MPLibrary/PathModifiers/ShortcuttingPathModifier.h"
#endif

#define PM_CLASSES ((ShortcuttingPathModifier, SHORTCUTTING_AVAILABLE))

/******************************* Samplers *************************************/
#if BRIDGE_TEST_AVAILABLE
#include "MPLibrary/Samplers/BridgeTestSampler.h"
#endif
#if GAUSSIAN_AVAILABLE
#include "MPLibrary/Samplers/GaussianSampler.h"
#endif
#if MIX_SAMPLER_AVAILABLE
#include "MPLibrary/Samplers/MixSampler.h"
#endif
#if OBSTACLE_BASED_SAMPLER_AVAILABLE
#include "MPLibrary/Samplers/ObstacleBasedSampler.h"
#endif
#if UNIFORM_RANDOM_AVAILABLE
#include "MPLibrary/Samplers/UniformRandomSampler.h"
#endif
#if UNIFORM_OBSTACLE_AVAILABLE
#include "MPLibrary/Samplers/UniformObstacleBasedSampler.h"
#endif

#define SAMPLER_CLASSES                                          \
  ((BridgeTestSampler, BRIDGE_TEST_AVAILABLE))(                  \
      (GaussianSampler, GAUSSIAN_AVAILABLE))(                    \
      (MixSampler, MIX_SAMPLER_AVAILABLE))(                      \
      (ObstacleBasedSampler, OBSTACLE_BASED_SAMPLER_AVAILABLE))( \
      (UniformRandomSampler, UNIFORM_RANDOM_AVAILABLE))(         \
      (UniformObstacleBasedSampler, UNIFORM_OBSTACLE_AVAILABLE))

/**************************** ValidityCheckers ********************************/
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#if ALWAYS_TRUE_AVAILABLE
#include "MPLibrary/ValidityCheckers/AlwaysTrueValidity.h"
#endif
#if COMPOSE_CD_AVAILABLE
#include "MPLibrary/ValidityCheckers/ComposeCollision.h"
#endif
#if COMPOSE_VC_AVAILABLE
#include "MPLibrary/ValidityCheckers/ComposeValidity.h"
#endif
#if NODE_CLEARANCE_VC_AVAILABLE
#include "MPLibrary/ValidityCheckers/NodeClearanceValidity.h"
#endif
#if OBSTACLE_CLEARANCE_VC_AVAILABLE
#include "MPLibrary/ValidityCheckers/ObstacleClearanceValidity.h"
#endif
#if TERRAIN_VC_AVAILABLE
#include "MPLibrary/ValidityCheckers/TerrainValidityChecker.h"
#endif
#if TOPOLOGICAL_MAP_VC_AVAILABLE
#include "MPLibrary/ValidityCheckers/TopologicalMapValidity.h"
#endif

#define VC_CLASSES                                                   \
  ((CollisionDetectionValidity, 1))(                                 \
      (AlwaysTrueValidity, ALWAYS_TRUE_AVAILABLE))(                  \
      (ComposeCollision, COMPOSE_CD_AVAILABLE))(                     \
      (ComposeValidity, COMPOSE_VC_AVAILABLE))(                      \
      (NodeClearanceValidity, NODE_CLEARANCE_VC_AVAILABLE))(         \
      (ObstacleClearanceValidity, OBSTACLE_CLEARANCE_VC_AVAILABLE))( \
      (TerrainValidityChecker, TERRAIN_VC_AVAILABLE))(               \
      (TopologicalMapValidity, TOPOLOGICAL_MAP_VC_AVAILABLE))

/******************************************************************************/

#define ADD_CLASS_IF_AVAILABLE(r, data, elem)  \
  BOOST_PP_IF(BOOST_PP_TUPLE_ELEM(2, 1, elem), \
              (BOOST_PP_TUPLE_ELEM(2, 0, elem)), BOOST_PP_EMPTY())

#define CONN_SEQ \
  BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, CONNECTOR_CLASSES)
#define DM_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, DM_CLASSES)
#define EXT_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, EXT_CLASSES)
#define EVC_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, EVC_CLASSES)
#define LP_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, LP_CLASSES)
#define ME_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, ME_CLASSES)
#define METRIC_SEQ \
  BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, METRIC_CLASSES)
#define MPSTRATEGY_SEQ \
  BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, MPSTRATEGY_CLASSES)
#define NF_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, NF_CLASSES)
#define PM_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, PM_CLASSES)
#define SAMPLER_SEQ \
  BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, SAMPLER_CLASSES)
#define VC_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, VC_CLASSES)

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief Defines available methods in the Motion Planning Universe
////////////////////////////////////////////////////////////////////////////////
struct MPUniverse {
  // types of distance metrics available in our world
  typedef boost::mpl::list<BOOST_PP_SEQ_ENUM(DM_SEQ)> DistanceMetricMethodList;

  // types of validity checkers available in our world
  typedef boost::mpl::list<BOOST_PP_SEQ_ENUM(VC_SEQ)> ValidityCheckerMethodList;

  // types of neighborhood finders available in our world
  typedef boost::mpl::list<BOOST_PP_SEQ_ENUM(NF_SEQ)>
      NeighborhoodFinderMethodList;

  // types of samplers available in our world
  typedef boost::mpl::list<BOOST_PP_SEQ_ENUM(SAMPLER_SEQ)> SamplerMethodList;

  // types of local planners available in our world
  typedef boost::mpl::list<BOOST_PP_SEQ_ENUM(LP_SEQ)> LocalPlannerMethodList;

  // types of extenders avaible in our world
  typedef boost::mpl::list<BOOST_PP_SEQ_ENUM(EXT_SEQ)> ExtenderMethodList;

  // types of path smoothing available in our world
  typedef boost::mpl::list<BOOST_PP_SEQ_ENUM(PM_SEQ)> PathModifierMethodList;

  // types of edge validity checkers available in our world
  typedef boost::mpl::list<BOOST_PP_SEQ_ENUM(EVC_SEQ)>
      EdgeValidityCheckerMethodList;

  // types of connectors available in our world
  typedef boost::mpl::list<BOOST_PP_SEQ_ENUM(CONN_SEQ)> ConnectorMethodList;

  // types of metrics available in our world
  typedef boost::mpl::list<BOOST_PP_SEQ_ENUM(METRIC_SEQ)> MetricMethodList;

  // types of map evaluators available in our world
  typedef boost::mpl::list<BOOST_PP_SEQ_ENUM(ME_SEQ)> MapEvaluatorMethodList;

  // types of motion planning strategies available in our world
  typedef boost::mpl::list<BOOST_PP_SEQ_ENUM(MPSTRATEGY_SEQ)>
      MPStrategyMethodList;
};

#endif
