/* Traits class for CfgType and WeightType.
 */

#ifndef MPTRAITS_H_
#define MPTRAITS_H_

#include "Weight.h"

//distance metric includes
#include "DistanceMetrics/CenterOfMassDistance.h"
#include "DistanceMetrics/EuclideanDistance.h"
#include "DistanceMetrics/KnotTheoryDistance.h"
#include "DistanceMetrics/LPSweptDistance.h"
#include "DistanceMetrics/RMSDDistance.h"

//validity checker includes
#include "ValidityCheckers/AlwaysTrueValidity.h"
#include "ValidityCheckers/CollisionDetectionValidity.h"
#include "ValidityCheckers/ComposeValidity.h"
#include "ValidityCheckers/MedialAxisClearanceValidity.h"
#include "ValidityCheckers/NegateValidity.h"
#include "ValidityCheckers/NodeClearanceValidity.h"
#include "ValidityCheckers/ObstacleClearanceValidity.h"
#ifdef PMPCfgSurface
#include "ValidityCheckers/SurfaceValidity.h"
#endif

//neighborhood finder includes
#include "NeighborhoodFinders/BruteForceNF.h"
#include "NeighborhoodFinders/HierarchicalNF.h"
#include "NeighborhoodFinders/RadiusNF.h"

//sampler includes
#include "Samplers/BridgeTestSampler.h"
#include "Samplers/GaussianSampler.h"
#include "Samplers/GridSampler.h"
#include "Samplers/MedialAxisSampler.h"
#include "Samplers/MixSampler.h"
#include "Samplers/ObstacleBasedSampler.h"
#include "Samplers/UniformObstacleBasedSampler.h"
#include "Samplers/UniformRandomSampler.h"
#ifdef PMPCfgSurface
#include "Samplers/SurfaceSampler.h"
#endif

//local planner includes
#include "LocalPlanners/StraightLine.h"
#include "LocalPlanners/TransformAtS.h"
#include "LocalPlanners/RotateAtS.h"
#ifdef PMPCfgSurface
#include "LocalPlanners/SurfaceLP.h"
#endif
#include "LocalPlanners/ToggleLP.h"

//connector includes
#include "Connectors/CCsConnector.h"
#include "Connectors/OptimalConnection.h"
#include "Connectors/OptimalRewire.h"
#ifdef PMPCfgSurface
#include "Connectors/ConnectNeighboringSurfaces.h"
#endif
#include "Connectors/NeighborhoodConnector.h"

//metric includes
#include "Metrics/CCDistanceMetric.h"
#include "Metrics/ConnectivityMetric.h"
#include "Metrics/CoverageDistanceMetric.h"
#include "Metrics/CoverageMetric.h"
#include "Metrics/DiameterMetric.h"
#include "Metrics/NumEdgesMetric.h"
#include "Metrics/NumNodesMetric.h"
#include "Metrics/TimeMetric.h"

//map evaluator includes
#include "MapEvaluators/ComposeEvaluator.h"
#include "MapEvaluators/ConditionalEvaluator.h"
#include "MapEvaluators/LazyQuery.h"
#include "MapEvaluators/LazyToggleQuery.h"
#include "MapEvaluators/NegateEvaluator.h"
#include "MapEvaluators/PrintMapEvaluation.h"
#include "MapEvaluators/Query.h"
#include "MapEvaluators/TrueEvaluation.h"

//mp strategies includes
#include "MPStrategies/BasicPRM.h"
#include "MPStrategies/BasicRRTStrategy.h"
#include "MPStrategies/EvaluateMapStrategy.h"
#include "MPStrategies/MedialAxisRRT.h"
#include "MPStrategies/ResamplePointStrategy.h"
#include "MPStrategies/TogglePRMStrategy.h"
#include "MPStrategies/UnitTest/DMTestStrategy.h"

#ifdef _PARALLEL
#include "ParallelMethods/BasicParallelPRM.h"
#include "ParallelMethods/RegularSubdivisionMethod.h"
#endif

template<class C, class W = DefaultWeight<C> >
struct MPTraits{
  typedef C CfgType;
  typedef W WeightType;

  typedef MPProblem<MPTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    //BinaryLPSweptDistance<MPTraits>,
    CenterOfMassDistance<MPTraits>,
    EuclideanDistance<MPTraits>,
    KnotTheoryDistance<MPTraits>,
    LPSweptDistance<MPTraits>,
    //ManhattanDistance<MPTraits>,
    MinkowskiDistance<MPTraits>,
    //#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
    //ReachableDistance<MPTraits>, 
    //#endif
    RMSDDistance<MPTraits>//,
    //ScaledEuclideanDistance<MPTraits>
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
    #ifdef PMPCfgSurface
    , SurfaceValidity<MPTraits>
    #endif
    > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    //BandsNF<MPTraits>,
    BruteForceNF<MPTraits>,
    //CGALNF<MPTraits>,
    //DPESNF<MPTraits>,
    HierarchicalNF<MPTraits>,
    //MetricTreeNF<MPTraits>,
    //MPNNNF<MPTraits>,
    RadiusNF<MPTraits>//,
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
#ifdef PMPCfgSurface
    SurfaceSampler<MPTraits>,
#endif
    UniformObstacleBasedSampler<MPTraits>,
    UniformRandomSampler<MPTraits>
      > SamplerMethodList;
  
  //types of local planners available in our world
  typedef boost::mpl::list<
    //AStarClearance<MPTraits>,
    //AStarDistance<MPTraits>,
    //MedialAxisLP<MPTraits>,
    TransformAtS<MPTraits>,
    RotateAtS<MPTraits>,
    StraightLine<MPTraits>,
    //TransformAtS<MPTraits>,
    #if defined(PMPCfgSurface)
    SurfaceLP<MPTraits>,
    #endif
    ToggleLP<MPTraits>
    > LocalPlannerMethodList;

  //types of connectors available in our world
  typedef boost::mpl::list<
    CCsConnector<MPTraits>,
#if defined(PMPCfgSurface)
    ConnectNeighboringSurfaces<MPTraits>,
#endif
    NeighborhoodConnector<MPTraits>,
    OptimalConnection<MPTraits>,
    OptimalRewire<MPTraits>/*, 
    PreferentialAttachment<MPTraits>, 
    ClosestVE<MPTraits>*/
      > ConnectorMethodList;
  
  //types of metrics available in our world
  typedef boost::mpl::list<
    CCDistanceMetric<MPTraits>,
    ConnectivityMetric<MPTraits>,
    CoverageDistanceMetric<MPTraits>,
    CoverageMetric<MPTraits>,
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
    TrueEvaluation<MPTraits>
    > MapEvaluatorMethodList;
  
  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    BasicPRM<MPTraits>,
    BasicRRTStrategy<MPTraits>,
    DMTestStrategy<MPTraits>,
    EvaluateMapStrategy<MPTraits>,
    MedialAxisRRT<MPTraits>,
    ResamplePointStrategy<MPTraits>,
    TogglePRMStrategy<MPTraits>
    #ifdef _PARALLEL
    ,BasicParallelPRM<MPTraits>
    ,RegularSubdivisionMethod<MPTraits>
    #endif
    > MPStrategyMethodList;
};

#endif
