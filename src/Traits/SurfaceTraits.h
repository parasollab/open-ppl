#ifndef SURFACE_TRAITS_H_
#define SURFACE_TRAITS_H_

#include "CfgTraits.h"

#include "MPProblem/Weight.h"

//distance metric includes
#include "DistanceMetrics/EuclideanDistance.h"

//validity checker includes
#include "ValidityCheckers/CollisionDetectionValidity.h"
#include "ValidityCheckers/SurfaceValidity.h"
#include "ValidityCheckers/SSSurfaceValidity.h"

//neighborhood finder includes
#include "NeighborhoodFinders/BruteForceNF.h"

//sampler includes
#include "Samplers/SurfaceGridSampler.h"
#include "Samplers/SurfaceSampler.h"
#include "Samplers/UniformRandomSampler.h"

//local planner includes
#include "LocalPlanners/StraightLine.h"
#include "LocalPlanners/SurfaceLP.h"

//extenders includes
#include "Extenders/BasicExtender.h"
#include "Extenders/MixExtender.h"
#include "Extenders/RandomObstacleVector.h"
#include "Extenders/RotationThenTranslation.h"
#include "Extenders/TraceCSpaceObstacle.h"
#include "Extenders/TraceMAPush.h"
#include "Extenders/TraceObstacle.h"

//path smoothing includes
#include "PathModifiers/CombinedPathModifier.h"
#include "PathModifiers/MedialAxisPathModifier.h"
#include "PathModifiers/ResamplePathModifier.h"
#include "PathModifiers/ShortcuttingPathModifier.h"

//connector includes
#include "Connectors/ConnectNeighboringSurfaces.h"
#include "Connectors/NeighborhoodConnector.h"

//metric includes
#include "Metrics/NumNodesMetric.h"

//map evaluator includes
#include "MapEvaluators/ConditionalEvaluator.h"
#include "MapEvaluators/Query.h"

//mp strategies includes
#include "MPStrategies/BasicPRM.h"
#include "MPStrategies/LocalManeuveringStrategy.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief Defines available methods in the Motion Planning Universe for
/// SurfaceCfg template specialization
///
/// MPTraits is a type class which defines the motion planning universe. We
/// construct our methods through a factory design pattern, and thus this states
/// all available classes within an abstraction that you can use in the system.
/// Essentially the important types are, the CfgType or the @cspace abstraction
/// class, the WeightType or the edge type of the graph, and a type list for
/// each algorithm abstraction --- here you only need to define what you need,
/// as extraneous methods in the type class imply longer compile times.
////////////////////////////////////////////////////////////////////////////////
class CfgSurface;

#ifdef PMPCfgSurface
template<>
struct MPTraits<CfgSurface, DefaultWeight<CfgSurface> > {
  typedef CfgSurface CfgType;
  typedef DefaultWeight<CfgType> WeightType;
  typedef CfgSurface& CfgRef;

  typedef MPProblem<MPTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    EuclideanDistance<MPTraits>
    > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    CollisionDetectionValidity<MPTraits>,
    SurfaceValidity<MPTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BruteForceNF<MPTraits>
    > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    SurfaceSampler<MPTraits>,
    SurfaceGridSampler<MPTraits>
      > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
    SurfaceLP<MPTraits>
    > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
    BasicExtender<MPTraits>,
    MixExtender<MPTraits>,
    RandomObstacleVector<MPTraits>,
    RotationThenTranslation<MPTraits>,
    TraceCSpaceObstacle<MPTraits>,
    TraceMAPush<MPTraits>,
    TraceObstacle<MPTraits>
      > ExtenderMethodList;

  //types of path smoothing available in our world
  typedef boost::mpl::list<
    CombinedPathModifier<MPTraits>,
    MedialAxisPathModifier<MPTraits>,
    ResamplePathModifier<MPTraits>,
    ShortcuttingPathModifier<MPTraits>
      > PathModifierMethodList;

  //types of connectors available in our world
  typedef boost::mpl::list<
    ConnectNeighboringSurfaces<MPTraits>,
    NeighborhoodConnector<MPTraits>//,
    > ConnectorMethodList;

  //types of metrics available in our world
  typedef boost::mpl::list<
    NumNodesMetric<MPTraits>
    > MetricMethodList;

  //types of map evaluators available in our world
  typedef boost::mpl::list<
    ConditionalEvaluator<MPTraits>
    > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    BasicPRM<MPTraits>
    > MPStrategyMethodList;
};
#endif

#ifdef PMPSSSurfaceMult
class SSSurfaceMult;
template<>
struct CfgTraits<SSSurfaceMult, DefaultWeight<SSSurfaceMult> > {
  typedef SSSurfaceMult CfgType;
  typedef DefaultWeight<CfgType> WeightType;
  typedef SSSurfaceMult& CfgRef;

  typedef MPProblem<MPTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    EuclideanDistance<MPTraits>
    > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    CollisionDetectionValidity<MPTraits>,
    SSSurfaceValidity<MPTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BruteForceNF<MPTraits>
    > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
    > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
    BasicExtender<MPTraits>,
    MixExtender<MPTraits>,
    RandomObstacleVector<MPTraits>,
    RotationThenTranslation<MPTraits>,
    TraceCSpaceObstacle<MPTraits>,
    TraceMAPush<MPTraits>,
    TraceObstacle<MPTraits>
      > ExtenderMethodList;

  //types of path smoothing available in our world
  typedef boost::mpl::list<
    CombinedPathModifier<MPTraits>
    > PathModifierMethodList;

  //types of connectors available in our world
  typedef boost::mpl::list<
    > ConnectorMethodList;

  //types of metrics available in our world
  typedef boost::mpl::list<
    NumNodesMetric<MPTraits>
    > MetricMethodList;

  //types of map evaluators available in our world
  typedef boost::mpl::list<
    ConditionalEvaluator<MPTraits>
    > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    BasicPRM<MPTraits>,
    LocalManeuveringStrategy<MPTraits>
      > MPStrategyMethodList;
};
#endif

#endif
