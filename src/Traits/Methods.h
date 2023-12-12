#ifndef PMPL_AVAILABLE_METHODS_H_
#define PMPL_AVAILABLE_METHODS_H_

#include <boost/preprocessor/seq.hpp>

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

// Extenders
#define BASIC_EXTENDER_AVAILABLE 1

// LocalPlanners

// MapEvaluators
#define CONDITIONAL_EVAL_AVAILABLE 1

// Metrics
#define NUM_NODES_AVAILABLE 1

// MPStrategies
#define BASIC_RRT_AVAILABLE 1

// NeighborhoodFinders
#define BRUTE_FORCE_NF_AVAILABLE 1

// Samplers
#define UNIFORM_RANDOM_AVAILABLE 1

// ValidityCheckers
#define ALWAYS_TRUE_AVAILABLE 1

/******************************* Connectors ***********************************/
#ifdef CCS_CONNECTOR_AVAILABLE
    #include "MPLibrary/Connectors/CCsConnector.h"
#endif
#ifdef NEIGHBOR_CONNECTOR_AVAILABLE
    #include "MPLibrary/Connectors/NeighborhoodConnector.h"
#endif
#ifdef REWIRE_CONNECTOR_AVAILABLE
    #include "MPLibrary/Connectors/RewireConnector.h"
#endif

#define CONNECTOR_CLASSES \
    ((CCsConnector, CCS_CONNECTOR_AVAILABLE)) \
    ((NeighborhoodConnector, NEIGHBOR_CONNECTOR_AVAILABLE)) \
    ((RewireConnector, REWIRE_CONNECTOR_AVAILABLE))

/***************************** DistanceMetrics ********************************/
#ifdef BIN_LP_SWEPT_AVAILABLE
    #include "MPLibrary/DistanceMetrics/BinaryLPSweptDistance.h"
#endif
#ifdef LP_SWEPT_AVAILABLE
    #include "MPLibrary/DistanceMetrics/LPSweptDistance.h"
#endif
#ifdef MINKOWSKI_AVAILABLE
    #include "MPLibrary/DistanceMetrics/MinkowskiDistance.h"
#endif
#ifdef EUCLIDEAN_AVAILABLE
    #include "MPLibrary/DistanceMetrics/EuclideanDistance.h"
#endif
#ifdef SCALED_EUCLIDEAN_AVAILABLE
    #include "MPLibrary/DistanceMetrics/ScaledEuclideanDistance.h"
#endif
#ifdef WEIGHTED_EUCLIDEAN_AVAILABLE
    #include "MPLibrary/DistanceMetrics/WeightedEuclideanDistance.h"
#endif
#ifdef MANHATTAN_AVAILABLE
    #include "MPLibrary/DistanceMetrics/ManhattanDistance.h"
#endif
#ifdef RMSD_AVAILABLE
    #include "MPLibrary/DistanceMetrics/RMSDDistance.h"
#endif
#ifdef TOPOLOGICAL_DISTANCE_AVAILABLE
    #include "MPLibrary/DistanceMetrics/TopologicalDistance.h"
#endif
#ifdef WORKSPACE_TRANSLATION_AVAILABLE
    #include "MPLibrary/DistanceMetrics/WorkspaceTranslationDistance.h"
#endif

#define DM_CLASSES \
    ((BinaryLPSweptDistance, BIN_LP_SWEPT_AVAILABLE)) \
    ((LPSweptDistance, LP_SWEPT_AVAILABLE)) \
    ((MinkowskiDistance, MINKOWSKI_AVAILABLE)) \
    ((EuclideanDistance, EUCLIDEAN_AVAILABLE)) \
    ((ScaledEuclideanDistance, SCALED_EUCLIDEAN_AVAILABLE)) \
    ((WeightedEuclideanDistance, WEIGHTED_EUCLIDEAN_AVAILABLE)) \
    ((ManhattanDistance, MANHATTAN_AVAILABLE)) \
    ((RMSDDistance, RMSD_AVAILABLE)) \
    ((TopologicalDistance, TOPOLOGICAL_DISTANCE_AVAILABLE)) \
    ((WorkspaceTranslationDistance, WORKSPACE_TRANSLATION_AVAILABLE))

/************************** EdgeValidityCheckers ******************************/

/******************************* Extenders ************************************/
#ifdef BASIC_EXTENDER_AVAILABLE
    #include "MPLibrary/Extenders/BasicExtender.h"
#endif

#define EXT_CLASSES \
    ((BasicExtender, BASIC_EXTENDER_AVAILABLE))
    // ... and so on ...

/***************************** LocalPlanners **********************************/

/***************************** MapEvaluators **********************************/
#ifdef CONDITIONAL_EVAL_AVAILABLE
    #include "MPLibrary/MapEvaluators/ConditionalEvaluator.h"
#endif

#define ME_CLASSES \
    ((ConditionalEvaluator, CONDITIONAL_EVAL_AVAILABLE))
    // ... and so on ...

/******************************** Metrics *************************************/
#ifdef NUM_NODES_AVAILABLE
    #include "MPLibrary/Metrics/NumNodesMetric.h"
#endif

#define METRIC_CLASSES \
    ((NumNodesMetric, NUM_NODES_AVAILABLE))
    // ... and so on ...

/***************************** MPStrategies ***********************************/
#ifdef BASIC_RRT_AVAILABLE
    #include "MPLibrary/MPStrategies/BasicRRTStrategy.h"
#endif

#define MPSTRATEGY_CLASSES \
    ((BasicRRTStrategy, BASIC_RRT_AVAILABLE))
    // ((B, CLASS_B_AVAILABLE)) \
    // ... and so on ...

/************************** NeighborhoodFinders *******************************/
#ifdef BRUTE_FORCE_NF_AVAILABLE
    #include "MPLibrary/NeighborhoodFinders/BruteForceNF.h"
#endif

#define NF_CLASSES \
    ((BruteForceNF, BRUTE_FORCE_NF_AVAILABLE))
    // ... and so on ...

/***************************** PathModifiers **********************************/

/******************************* Samplers *************************************/
#ifdef UNIFORM_RANDOM_AVAILABLE
    #include "MPLibrary/Samplers/UniformRandomSampler.h"
#endif

#define SAMPLER_CLASSES \
    ((UniformRandomSampler, UNIFORM_RANDOM_AVAILABLE))
    // ((B, CLASS_B_AVAILABLE)) \
    // ... and so on ...

/**************************** ValidityCheckers ********************************/
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#ifdef ALWAYS_TRUE_AVAILABLE
    #include "MPLibrary/ValidityCheckers/AlwaysTrueValidity.h"
#endif

#define VC_CLASSES \
    ((CollisionDetectionValidity, 1)) \
    ((AlwaysTrueValidity, ALWAYS_TRUE_AVAILABLE)) \
    // ... and so on ...


#define ADD_CLASS_IF_AVAILABLE(r, data, elem) \
    BOOST_PP_IF(BOOST_PP_TUPLE_ELEM(2, 1, elem), (BOOST_PP_TUPLE_ELEM(2, 0, elem)), BOOST_PP_EMPTY())

#define CONN_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, CONNECTOR_CLASSES)
#define DM_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, DM_CLASSES)
#define EXT_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, EXT_CLASSES)
#define ME_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, ME_CLASSES)
#define METRIC_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, METRIC_CLASSES)
#define MPSTRATEGY_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, MPSTRATEGY_CLASSES)
#define NF_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, NF_CLASSES)
#define SAMPLER_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, SAMPLER_CLASSES)
#define VC_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, VC_CLASSES)

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief Defines available methods in the Motion Planning Universe
////////////////////////////////////////////////////////////////////////////////
struct MPUniverse {

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    BOOST_PP_SEQ_ENUM(DM_SEQ)
      > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    BOOST_PP_SEQ_ENUM(VC_SEQ)
     > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BOOST_PP_SEQ_ENUM(NF_SEQ)
      > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    BOOST_PP_SEQ_ENUM(SAMPLER_SEQ)
      > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
    // StraightLine<MPTraits>
      > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
    BOOST_PP_SEQ_ENUM(EXT_SEQ)
      > ExtenderMethodList;

  //types of path smoothing available in our world
  typedef boost::mpl::list<
    // ShortcuttingPathModifier<MPTraits>
      > PathModifierMethodList;

  //types of edge validity checkers available in our world
  typedef boost::mpl::list<
    // IntermediatesEdgeValidityChecker<MPTraits>
      > EdgeValidityCheckerMethodList;

  //types of connectors available in our world
  typedef boost::mpl::list<
    BOOST_PP_SEQ_ENUM(CONN_SEQ)
      > ConnectorMethodList;

  //types of metrics available in our world
  typedef boost::mpl::list<
    BOOST_PP_SEQ_ENUM(METRIC_SEQ)
      > MetricMethodList;


  //types of map evaluators available in our world
  typedef boost::mpl::list<
    BOOST_PP_SEQ_ENUM(ME_SEQ)
      > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    BOOST_PP_SEQ_ENUM(MPSTRATEGY_SEQ)
    > MPStrategyMethodList;


};

#endif
