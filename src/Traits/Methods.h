#ifndef PMPL_AVAILABLE_METHODS_H_
#define PMPL_AVAILABLE_METHODS_H_

// Samplers
#define UNIFORM_RANDOM_AVAILABLE 1

// DistanceMetrics
#define MINKOWSKI_AVAILABLE 1
#define EUCLIDEAN_AVAILABLE 1

// NeighborhoodFinders
#define BRUTE_FORCE_NF_AVAILABLE 1

// Extenders
#define BASIC_EXTENDER_AVAILABLE 1

// MapEvaluators
#define CONDITIONAL_EVAL_AVAILABLE 1

// Metrics
#define NUM_NODES_AVAILABLE 1

// MPStrategies
#define BASIC_RRT_AVAILABLE 1

// ValidityCheckers
#define ALWAYS_TRUE_AVAILABLE 1

#include <boost/preprocessor/seq.hpp>

#define ADD_CLASS_IF_AVAILABLE(r, data, elem) \
    BOOST_PP_IF(BOOST_PP_TUPLE_ELEM(2, 1, elem), (BOOST_PP_TUPLE_ELEM(2, 0, elem)), BOOST_PP_EMPTY())

// Samplers
#ifdef UNIFORM_RANDOM_AVAILABLE
    #include "MPLibrary/Samplers/UniformRandomSampler.h"
#endif

#define SAMPLER_CLASSES \
    ((UniformRandomSampler, UNIFORM_RANDOM_AVAILABLE))
    // ((B, CLASS_B_AVAILABLE)) \
    // ... and so on ...

#define SAMPLER_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, SAMPLER_CLASSES)

// Distance Metrics
#ifdef MINKOWSKI_AVAILABLE
    #include "MPLibrary/DistanceMetrics/MinkowskiDistance.h"
#endif
#ifdef EUCLIDEAN_AVAILABLE
    #include "MPLibrary/DistanceMetrics/EuclideanDistance.h"
#endif

#define DM_CLASSES \
    ((MinkowskiDistance, MINKOWSKI_AVAILABLE)) \
    ((EuclideanDistance, EUCLIDEAN_AVAILABLE))
    // ... and so on ...

#define DM_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, DM_CLASSES)

// Neighborhood Finders
#ifdef BRUTE_FORCE_NF_AVAILABLE
    #include "MPLibrary/NeighborhoodFinders/BruteForceNF.h"
#endif

#define NF_CLASSES \
    ((BruteForceNF, BRUTE_FORCE_NF_AVAILABLE))
    // ... and so on ...

#define NF_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, NF_CLASSES)

// Extenders
#ifdef BASIC_EXTENDER_AVAILABLE
    #include "MPLibrary/Extenders/BasicExtender.h"
#endif

#define EXT_CLASSES \
    ((BasicExtender, BASIC_EXTENDER_AVAILABLE))
    // ... and so on ...

#define EXT_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, EXT_CLASSES)

// MapEvaluators
#ifdef CONDITIONAL_EVAL_AVAILABLE
    #include "MPLibrary/MapEvaluators/ConditionalEvaluator.h"
#endif

#define ME_CLASSES \
    ((ConditionalEvaluator, CONDITIONAL_EVAL_AVAILABLE))
    // ... and so on ...

#define ME_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, ME_CLASSES)

// Metrics
#ifdef NUM_NODES_AVAILABLE
    #include "MPLibrary/Metrics/NumNodesMetric.h"
#endif

#define METRIC_CLASSES \
    ((NumNodesMetric, NUM_NODES_AVAILABLE))
    // ... and so on ...

#define METRIC_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, METRIC_CLASSES)

// Validity Checkers - CollisionDetectionValidity has to be included, but
// others are parameterizable
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#ifdef ALWAYS_TRUE_AVAILABLE
    #include "MPLibrary/ValidityCheckers/AlwaysTrueValidity.h"
#endif

#define VC_CLASSES \
    ((CollisionDetectionValidity, 1)) \
    ((AlwaysTrueValidity, ALWAYS_TRUE_AVAILABLE)) \
    // ... and so on ...

#define VC_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, VC_CLASSES)

// MPStrategies
#ifdef BASIC_RRT_AVAILABLE
    #include "MPLibrary/MPStrategies/BasicRRTStrategy.h"
#endif

#define MPSTRATEGY_CLASSES \
    ((BasicRRTStrategy, BASIC_RRT_AVAILABLE))
    // ((B, CLASS_B_AVAILABLE)) \
    // ... and so on ...

#define MPSTRATEGY_SEQ BOOST_PP_SEQ_FOR_EACH(ADD_CLASS_IF_AVAILABLE, _, MPSTRATEGY_CLASSES)

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
    // NeighborhoodConnector<MPTraits>,
    // CCsConnector<MPTraits>,
    // RewireConnector<MPTraits>
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
