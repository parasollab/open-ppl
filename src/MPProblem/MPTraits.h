/* Traits class for CfgType and WeightType.
 */

#ifndef MPTRAITS_H_
#define MPTRAITS_H_

#include "Weight.h"

//distance metric includes
#include "DistanceMetrics/BinaryLPSweptDistance.h"
#include "DistanceMetrics/CenterOfMassDistance.h"
#include "DistanceMetrics/EuclideanDistance.h"
#include "DistanceMetrics/KnotTheoryDistance.h"
#include "DistanceMetrics/LPSweptDistance.h"
#include "DistanceMetrics/RMSDDistance.h"
#include "DistanceMetrics/ManhattanDistance.h"
#include "DistanceMetrics/ReachableDistance.h"
#include "DistanceMetrics/ScaledEuclideanDistance.h"

//validity checker includes
#include "ValidityCheckers/AlwaysTrueValidity.h"
#include "ValidityCheckers/CollisionDetectionValidity.h"
#include "ValidityCheckers/ComposeValidity.h"
#include "ValidityCheckers/MedialAxisClearanceValidity.h"
#include "ValidityCheckers/NegateValidity.h"
#include "ValidityCheckers/NodeClearanceValidity.h"
#include "ValidityCheckers/ObstacleClearanceValidity.h"
#include "ValidityCheckers/SurfaceValidity.h"
#include "ValidityCheckers/SSSurfaceValidity.h"

//neighborhood finder includes
#include "NeighborhoodFinders/BandsNF.h"
#include "NeighborhoodFinders/BruteForceNF.h"
#include "NeighborhoodFinders/HierarchicalNF.h"
#include "NeighborhoodFinders/HopLimitNF.h"
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
#include "Samplers/SimilarStructureSampler.h"
#include "Samplers/SurfaceGridSampler.h"
#include "Samplers/SurfaceSampler.h"
#include "Samplers/UniformObstacleBasedSampler.h"
#include "Samplers/UniformRandomSampler.h"

//local planner includes
#include "LocalPlanners/AStar.h"
#include "LocalPlanners/HierarchicalLP.h"
#include "LocalPlanners/MedialAxisLP.h"
#include "LocalPlanners/RotateAtS.h"
#include "LocalPlanners/StraightLine.h"
#include "LocalPlanners/SurfaceLP.h"
#include "LocalPlanners/ToggleLP.h"
#include "LocalPlanners/TransformAtS.h"

//extenders includes
#include "Extenders/BasicExtender.h"
#include "Extenders/MedialAxisExtender.h"
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
#include "Connectors/AdaptiveConnector.h"
#include "Connectors/CCsConnector.h"
#include "Connectors/ConnectNeighboringSurfaces.h"
#include "Connectors/NeighborhoodConnector.h"
#include "Connectors/RewireConnector.h"
#include "Connectors/CCExpansion.h"

//metric includes
#include "Metrics/CCDistanceMetric.h"
#include "Metrics/ConnectivityMetric.h"
#include "Metrics/CoverageDistanceMetric.h"
#include "Metrics/CoverageMetric.h"
#include "Metrics/DiameterMetric.h"
#include "Metrics/NumEdgesMetric.h"
#include "Metrics/NumNodesMetric.h"
#ifndef _PARALLEL
#include "Metrics/RoadmapSet.h"
#endif
#include "Metrics/TimeMetric.h"
#include "Metrics/VectorSet.h"

//map evaluator includes
#include "MapEvaluators/ComposeEvaluator.h"
#include "MapEvaluators/ConditionalEvaluator.h"
#include "MapEvaluators/LazyQuery.h"
#include "MapEvaluators/LazyToggleQuery.h"
#include "MapEvaluators/NegateEvaluator.h"
#include "MapEvaluators/PrintMapEvaluation.h"
#include "MapEvaluators/Query.h"
#include "MapEvaluators/ReplanningEvaluation.h"
#include "MapEvaluators/TrueEvaluation.h"

//mp strategies includes
#include "MPStrategies/AdaptiveRRT.h"
#include "MPStrategies/BasicPRM.h"
#include "MPStrategies/BasicRRTStrategy.h"
#include "MPStrategies/EvaluateMapStrategy.h"
#include "MPStrategies/LocalManeuveringStrategy.h"
#include "MPStrategies/MultiStrategy.h"
#include "MPStrategies/PushQueryToMA.h"
#include "MPStrategies/TogglePRMStrategy.h"
#include "MPStrategies/UnitTest/ClearanceTestStrategy.h"
#include "MPStrategies/UnitTest/DMTestStrategy.h"
#include "MPStrategies/UtilityGuidedGenerator.h"
#include "MPStrategies/VisibilityBasedPRM.h"

#ifdef _PARALLEL
#include "ParallelMethods/BasicParallelPRM.h"
#include "ParallelMethods/RegularSubdivisionMethod.h"
#endif

template<class C, class W = DefaultWeight<C> >
struct MPTraits{
  typedef C CfgType;
  typedef W WeightType;
#ifdef _PARALLEL
  typedef C CfgRef;
#else
  typedef C& CfgRef;
#endif

  typedef MPProblem<MPTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    BinaryLPSweptDistance<MPTraits>,
    CenterOfMassDistance<MPTraits>,
    EuclideanDistance<MPTraits>,
    KnotTheoryDistance<MPTraits>,
    LPSweptDistance<MPTraits>,
    ManhattanDistance<MPTraits>,
    MinkowskiDistance<MPTraits>,
    #if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
    ReachableDistance<MPTraits>,
    #endif
    RMSDDistance<MPTraits>,
    ScaledEuclideanDistance<MPTraits>
    > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    AlwaysTrueValidity<MPTraits>,
    CollisionDetectionValidity<MPTraits>,
    ComposeValidity<MPTraits>,
    MedialAxisClearanceValidity<MPTraits>,
    NegateValidity<MPTraits>,
    NodeClearanceValidity<MPTraits>,
    ObstacleClearanceValidity<MPTraits>
    > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BandsNF<MPTraits>,
    BruteForceNF<MPTraits>,
    //CGALNF<MPTraits>,
    //DPESNF<MPTraits>,
    HierarchicalNF<MPTraits>,
    HopLimitNF<MPTraits>,
    //MetricTreeNF<MPTraits>,
    //MPNNNF<MPTraits>,
    OptimalNF<MPTraits>,
    RadiusNF<MPTraits>,
    RandomNF<MPTraits>//,
    //SpillTreeNF<MPTraits>
    > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    BridgeTestSampler<MPTraits>,
    GaussianSampler<MPTraits>,
    GridSampler<MPTraits>,
    MedialAxisSampler<MPTraits>,
    MixSampler<MPTraits>,
    ObstacleBasedSampler<MPTraits>,
    SimilarStructureSampler<MPTraits>,
    UniformObstacleBasedSampler<MPTraits>,
    UniformRandomSampler<MPTraits>
      > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
    AStarClearance<MPTraits>,
    AStarDistance<MPTraits>,
    HierarchicalLP<MPTraits>,
    MedialAxisLP<MPTraits>,
    RotateAtS<MPTraits>,
    StraightLine<MPTraits>,
    ToggleLP<MPTraits>,
    TransformAtS<MPTraits>
    > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
    BasicExtender<MPTraits>,
    MedialAxisExtender<MPTraits>,
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
    AdaptiveConnector<MPTraits>,
    CCExpansion<MPTraits>,
    CCsConnector<MPTraits>,
    NeighborhoodConnector<MPTraits>,
    //PreferentialAttachment<MPTraits>,
    RewireConnector<MPTraits>//,
    //ClosestVE<MPTraits>
      > ConnectorMethodList;

#ifndef _PARALLEL
  typedef ConnectivityMetric<MPTraits, RoadmapSet<MPTraits> > ConnectivityMetricRoadmapSet;
  typedef CoverageDistanceMetric<MPTraits, RoadmapSet<MPTraits> > CoverageDistanceMetricRoadmapSet;
  typedef CoverageMetric<MPTraits, RoadmapSet<MPTraits> > CoverageMetricRoadmapSet;
#endif
  typedef ConnectivityMetric<MPTraits, VectorSet<MPTraits> > ConnectivityMetricVectorSet;
  typedef CoverageDistanceMetric<MPTraits, VectorSet<MPTraits> > CoverageDistanceMetricVectorSet;
  typedef CoverageMetric<MPTraits, VectorSet<MPTraits> > CoverageMetricVectorSet;

  //types of metrics available in our world
  typedef boost::mpl::list<
    CCDistanceMetric<MPTraits>,
#ifndef _PARALLEL
    ConnectivityMetricRoadmapSet,
    CoverageDistanceMetricRoadmapSet,
    CoverageMetricRoadmapSet,
#endif
    ConnectivityMetricVectorSet,
    CoverageDistanceMetricVectorSet,
    CoverageMetricVectorSet,
    DiameterMetric<MPTraits>,
    NumEdgesMetric<MPTraits>,
    NumNodesMetric<MPTraits>,
    TimeMetric<MPTraits>
    > MetricMethodList;

  //types of map evaluators available in our world
  typedef boost::mpl::list<
    ComposeEvaluator<MPTraits>,
    ConditionalEvaluator<MPTraits>,
    LazyQuery<MPTraits>,
    LazyToggleQuery<MPTraits>,
    NegateEvaluator<MPTraits>,
    PrintMapEvaluation<MPTraits>,
    Query<MPTraits>,
    ReplanningEvaluation<MPTraits>,
    TrueEvaluation<MPTraits>
    > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    AdaptiveRRT<MPTraits>,
    BasicPRM<MPTraits>,
    BasicRRTStrategy<MPTraits>,
    ClearanceTestStrategy<MPTraits>,
    DMTestStrategy<MPTraits>,
    EvaluateMapStrategy<MPTraits>,
    MultiStrategy<MPTraits>,
    PushQueryToMA<MPTraits>,
    TogglePRMStrategy<MPTraits>,
    UtilityGuidedGenerator<MPTraits>,
    VisibilityBasedPRM<MPTraits>
    #ifdef _PARALLEL
    ,BasicParallelPRM<MPTraits>
    ,RegularSubdivisionMethod<MPTraits>
    #endif
    > MPStrategyMethodList;
};

#if(defined(PMPCfgSurface) || defined(PMPSSSurfaceMult))
class CfgSurface;

#ifdef PMPCfgSurface
template<>
//struct CfgSurfMPTraits<CfgSurface, DefaultWeight<CfgSurface> > {
struct MPTraits<CfgSurface, DefaultWeight<CfgSurface> > {
  typedef CfgSurface CfgType;
  typedef DefaultWeight<CfgType> WeightType;
#ifdef _PARALLEL
  typedef CfgSurface CfgRef;
#else
  typedef CfgSurface& CfgRef;
#endif

  typedef MPProblem<MPTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    EuclideanDistance<MPTraits>
    > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    CollisionDetectionValidity<MPTraits>,
#ifdef PMPCfgSurface
    SurfaceValidity<MPTraits>
#else
    SSSurfaceValidity<MPTraits>
#endif
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
#ifdef PMPCfgSurface
    ConnectNeighboringSurfaces<MPTraits>,
    NeighborhoodConnector<MPTraits>//,
#endif
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
    BasicPRM<MPTraits>//,
//#ifdef PMPSSSurfaceMult
//    ,LocalManeuveringStrategy<MPTraits>
//#endif
    #ifdef _PARALLEL
    //,BasicParallelPRM<GBMPTraits>
    //,RegularSubdivisionMethod<GBMPTraits>
    #endif
    > MPStrategyMethodList;
};
#endif


#ifdef PMPSSSurfaceMult
class SSSurfaceMult;
template<>
struct MPTraits<SSSurfaceMult, DefaultWeight<SSSurfaceMult> > {
  typedef SSSurfaceMult CfgType;
  typedef DefaultWeight<CfgType> WeightType;
#ifdef _PARALLEL
  typedef SSSurfaceMult CfgRef;
#else
  typedef SSSurfaceMult& CfgRef;
#endif


  typedef MPProblem<MPTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    EuclideanDistance<MPTraits>
    > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    CollisionDetectionValidity<MPTraits>,
//#ifdef PMPCfgSurface
//    SurfaceValidity<MPTraits>
//#else
    SSSurfaceValidity<MPTraits>
//#endif
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
//#ifdef PMPCfgSurface
//    ConnectNeighboringSurfaces<MPTraits>,
//    NeighborhoodConnector<MPTraits>//,
//#endif
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
    BasicPRM<MPTraits>//,
#ifdef PMPSSSurfaceMult
    ,LocalManeuveringStrategy<MPTraits>
#endif
    #ifdef _PARALLEL
    //,BasicParallelPRM<GBMPTraits>
    //,RegularSubdivisionMethod<GBMPTraits>
    #endif
    > MPStrategyMethodList;
};
#endif

/*
//template specialization for SurfaceCfgs
template<>
//struct MPTraits<CfgSurface, DefaultWeight<CfgSurface> > {
  //typedef CfgSurface CfgType;
  //typedef DefaultWeight<CfgSurface> WeightType;
#ifdef PMPSSSurfaceMult
struct MPTraits<SSSurfaceMult, DefaultWeight<SSSurfaceMult> > {
  typedef SSSurfaceMult CfgType;
#else
struct MPTraits<CfgSurface, DefaultWeight<CfgSurface> > {
  typedef CfgSurface CfgType;
#endif
  typedef DefaultWeight<CfgType> WeightType;
#ifdef _PARALLEL
  typedef CfgType CfgRef;
#else
  typedef CfgType& CfgRef;
#endif


  typedef MPProblem<MPTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    EuclideanDistance<MPTraits>
    > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    CollisionDetectionValidity<MPTraits>,
#ifdef PMPCfgSurface
    SurfaceValidity<MPTraits>
#else
    SSSurfaceValidity<MPTraits>
#endif
    > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BruteForceNF<MPTraits>
    > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    SurfaceSampler<MPTraits>
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
    CombinedPathModifier<MPTraits>
  > PathModifierMethodList;

  //types of connectors available in our world
  typedef boost::mpl::list<
#ifdef PMPCfgSurface
    ConnectNeighboringSurfaces<MPTraits>,
    NeighborhoodConnector<MPTraits>//,
#endif
      > ConnectorMethodList;

#ifndef _PARALLEL
  typedef ConnectivityMetric<MPTraits, RoadmapSet<MPTraits> > ConnectivityMetricRoadmapSet;
  typedef CoverageDistanceMetric<MPTraits, RoadmapSet<MPTraits> > CoverageDistanceMetricRoadmapSet;
  typedef CoverageMetric<MPTraits, RoadmapSet<MPTraits> > CoverageMetricRoadmapSet;
#endif
  typedef ConnectivityMetric<MPTraits, VectorSet<MPTraits> > ConnectivityMetricVectorSet;
  typedef CoverageDistanceMetric<MPTraits, VectorSet<MPTraits> > CoverageDistanceMetricVectorSet;
  typedef CoverageMetric<MPTraits, VectorSet<MPTraits> > CoverageMetricVectorSet;

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
    BasicPRM<MPTraits>//,
#ifdef PMPSSSurfaceMult
    ,LocalManeuveringStrategy<MPTraits>
#endif
    #ifdef _PARALLEL
    //,BasicParallelPRM<GBMPTraits>
    //,RegularSubdivisionMethod<GBMPTraits>
    #endif
    > MPStrategyMethodList;
};
*/

#endif
#endif
