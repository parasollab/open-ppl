#ifndef SURFACE_TRAITS_H_
#define SURFACE_TRAITS_H_

#include "Cfg/CfgSurface.h"
#include "Cfg/SSSurfaceMult.h"
#include "MPProblem/Weight.h"

//distance metric includes
#include "PlanningLibrary/DistanceMetrics/EuclideanDistance.h"

//validity checker includes
#include "PlanningLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "PlanningLibrary/ValidityCheckers/SurfaceValidity.h"
#include "PlanningLibrary/ValidityCheckers/SSSurfaceValidity.h"

//neighborhood finder includes
#include "PlanningLibrary/NeighborhoodFinders/BruteForceNF.h"
#include "PlanningLibrary/NeighborhoodFinders/MPNNNF.h"

//sampler includes
#include "PlanningLibrary/Samplers/SurfaceGridSampler.h"
#include "PlanningLibrary/Samplers/SurfaceSampler.h"
#include "PlanningLibrary/Samplers/UniformRandomSampler.h"

//local planner includes
#include "PlanningLibrary/LocalPlanners/StraightLine.h"
#include "PlanningLibrary/LocalPlanners/SurfaceLP.h"

//extenders includes
#include "PlanningLibrary/Extenders/BasicExtender.h"
#include "PlanningLibrary/Extenders/MixExtender.h"
#include "PlanningLibrary/Extenders/RandomObstacleVector.h"
#include "PlanningLibrary/Extenders/RotationThenTranslation.h"
#include "PlanningLibrary/Extenders/TraceCSpaceObstacle.h"
#include "PlanningLibrary/Extenders/TraceMAPush.h"
#include "PlanningLibrary/Extenders/TraceObstacle.h"

//path smoothing includes
#include "PlanningLibrary/PathModifiers/CombinedPathModifier.h"
#include "PlanningLibrary/PathModifiers/MedialAxisPathModifier.h"
#include "PlanningLibrary/PathModifiers/ResamplePathModifier.h"
#include "PlanningLibrary/PathModifiers/ShortcuttingPathModifier.h"

//connector includes
#include "PlanningLibrary/Connectors/CCsConnector.h"
#include "PlanningLibrary/Connectors/ConnectNeighboringSurfaces.h"
#include "PlanningLibrary/Connectors/NeighborhoodConnector.h"

//metric includes
#include "PlanningLibrary/Metrics/NumNodesMetric.h"

//map evaluator includes
#include "PlanningLibrary/MapEvaluators/ConditionalEvaluator.h"

//mp strategies includes
#include "PlanningLibrary/MPStrategies/BasicPRM.h"
#include "PlanningLibrary/MPStrategies/LocalManeuveringStrategy.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief Defines available methods in the Motion Planning Universe for
///        SurfaceCfg template specialization
///
/// SurfaceTraits is a type class which defines the motion planning universe. We
/// construct our methods through a factory design pattern, and thus this states
/// all available classes within an abstraction that you can use in the system.
/// Essentially the important types are, the CfgType or the @cspace abstraction
/// class, the WeightType or the edge type of the graph, and a type list for
/// each algorithm abstraction --- here you only need to define what you need,
/// as extraneous methods in the type class imply longer compile times.
////////////////////////////////////////////////////////////////////////////////

#ifdef PMPCfgSurface

struct SurfaceTraits {
  typedef CfgSurface CfgType;
  typedef DefaultWeight<CfgType> WeightType;
  typedef CfgSurface& CfgRef;

  typedef MPProblem<SurfaceTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    EuclideanDistance<SurfaceTraits>
    > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    CollisionDetectionValidity<SurfaceTraits>,
    SurfaceValidity<SurfaceTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BruteForceNF<SurfaceTraits>,
    MPNNNF<SurfaceTraits>
    > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    SurfaceSampler<SurfaceTraits>,
    SurfaceGridSampler<SurfaceTraits>
      > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
    SurfaceLP<SurfaceTraits>
    > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
    BasicExtender<SurfaceTraits>,
    MixExtender<SurfaceTraits>,
    RandomObstacleVector<SurfaceTraits>,
    RotationThenTranslation<SurfaceTraits>,
    TraceCSpaceObstacle<SurfaceTraits>,
    TraceMAPush<SurfaceTraits>,
    TraceObstacle<SurfaceTraits>
      > ExtenderMethodList;

  //types of path smoothing available in our world
  typedef boost::mpl::list<
    CombinedPathModifier<SurfaceTraits>,
    MedialAxisPathModifier<SurfaceTraits>,
    ResamplePathModifier<SurfaceTraits>,
    ShortcuttingPathModifier<SurfaceTraits>
      > PathModifierMethodList;

  //types of connectors available in our world
  typedef boost::mpl::list<
    ConnectNeighboringSurfaces<SurfaceTraits>,
    NeighborhoodConnector<SurfaceTraits>,
    CCsConnector<SurfaceTraits>
    > ConnectorMethodList;

  //types of metrics available in our world
  typedef boost::mpl::list<
    NumNodesMetric<SurfaceTraits>
    > MetricMethodList;

  //types of map evaluators available in our world
  typedef boost::mpl::list<
    ConditionalEvaluator<SurfaceTraits>
    > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    BasicPRM<SurfaceTraits>
    > MPStrategyMethodList;
};

#endif

#ifdef PMPSSSurfaceMult

struct SSSurfaceMultTraits {
  typedef SSSurfaceMult CfgType;
  typedef DefaultWeight<CfgType> WeightType;
  typedef SSSurfaceMult& CfgRef;
  //switching SSSurfaceMultTraits with CfgTraits
  typedef MPProblem<SSSurfaceMultTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    EuclideanDistance<SSSurfaceMultTraits>
    > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    CollisionDetectionValidity<SSSurfaceMultTraits>,
    SurfaceValidity<SSSurfaceMultTraits>,
    SSSurfaceValidity<SSSurfaceMultTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BruteForceNF<SSSurfaceMultTraits>
    > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
    > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
    BasicExtender<SSSurfaceMultTraits>,
    MixExtender<SSSurfaceMultTraits>,
    RandomObstacleVector<SSSurfaceMultTraits>,
    RotationThenTranslation<SSSurfaceMultTraits>,
    TraceCSpaceObstacle<SSSurfaceMultTraits>,
    TraceMAPush<SSSurfaceMultTraits>,
    TraceObstacle<SSSurfaceMultTraits>
      > ExtenderMethodList;

  //types of path smoothing available in our world
  typedef boost::mpl::list<
    CombinedPathModifier<SSSurfaceMultTraits>
    > PathModifierMethodList;

  //types of connectors available in our world
  typedef boost::mpl::list<
    > ConnectorMethodList;

  //types of metrics available in our world
  typedef boost::mpl::list<
    NumNodesMetric<SSSurfaceMultTraits>
    > MetricMethodList;

  //types of map evaluators available in our world
  typedef boost::mpl::list<
    ConditionalEvaluator<SSSurfaceMultTraits>
    > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    BasicPRM<SSSurfaceMultTraits>,
    LocalManeuveringStrategy<SSSurfaceMultTraits>
      > MPStrategyMethodList;
};

#endif

#endif
