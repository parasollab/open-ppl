/* Traits class for CfgType and WeightType.
 */

#ifndef MPTRAITS_H_
#define MPTRAITS_H_

#include "Weight.h"

//distance metric includes
#include "DistanceMetrics/EuclideanDistance.h"
#include "DistanceMetrics/RMSDDistance.h"

//validity checker includes
#include "ValidityCheckers/AlwaysTrueValidity.h"
#include "ValidityCheckers/CollisionDetectionValidity.h"
#include "ValidityCheckers/ObstacleClearanceValidity.h"
#ifdef PMPCfgSurface
#include "ValidityCheckers/SurfaceValidity.h"
#endif

//neighborhood finder includes
#include "NeighborhoodFinders/BruteForceNF.h"

//sampler includes
#include "Samplers/BridgeTestSampler.h"
#include "Samplers/GaussianSampler.h"
#include "Samplers/GridSampler.h"
#include "Samplers/MixSampler.h"
#include "Samplers/ObstacleBasedSampler.h"
#include "Samplers/UniformObstacleBasedSamplers.h"
#include "Samplers/UniformRandomSampler.h"

//local planner includes
#include "LocalPlanners/StraightLine.h"
#include "LocalPlanners/ToggleLP.h"
#ifdef PMPCfgSurface
#include "LocalPlanners/SurfaceLP.h"
#endif

//connector includes
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
#include "MapEvaluators/Query.h"
#include "MapEvaluators/LazyQuery.h"
#include "MapEvaluators/LazyToggleQuery.h"
#include "MapEvaluators/ConditionalEvaluator.h"
#include "MapEvaluators/PrintMapEvaluation.h"
#include "MapEvaluators/TrueEvaluation.h"
#include "MapEvaluators/ComposeEvaluator.h"

//map evaluator includes
#include "MPStrategies/BasicPRM.h"
#include "MPStrategies/BasicRRTStrategy.h"
#include "MPStrategies/ResamplePointStrategy.h"
#include "MPStrategies/TogglePRMStrategy.h"

template<class C, class W = DefaultWeight<C> >
struct MPTraits{
  typedef C CfgType;
  typedef W WeightType;

  typedef MPProblem<MPTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    MinkowskiDistance<MPTraits>,
    //ManhattanDistance,
    EuclideanDistance<MPTraits>,
    //ScaledEuclideanDistance<MPTraits>,
    //MPNNEuclideanDistance<MPTraits>,
    //CGALEuclideanDistance<MPTraits>,
    //CenterOfMassDistance<MPTraits>,
    RMSDDistance<MPTraits>//,
    //LPSweptDistance<MPTraits>,
    //BinaryLPSweptDistance<MPTraits>,
    //#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
    //ReachableDistance<MPTraits>, 
    //#endif
    //KnotTheoryDistance<MPTraits>
    > DistanceMetricMethodList;
  
  //types of validity checkers available in our world
  typedef boost::mpl::list<
    #ifdef PMPCfgSurface
    SurfaceValidity<MPTraits>,
    #endif
    AlwaysTrueValidity<MPTraits>,
    CollisionDetectionValidity<MPTraits>,
    ObstacleClearanceValidity<MPTraits>//,
    //NodeClearanceValidity<MPTraits>,
    //MedialAxisClearanceValidity<MPTraits>,
    //ComposeValidity<MPTraits>,
    //NegateValidity<MPTraits>,*/
    > ValidityCheckerMethodList;

  //typdes of neighborhood finders available in our world
  typedef boost::mpl::list<
    BruteForceNF<MPTraits>//,
    //HierarchicalNF<MPTraits>,
    //RadiusNF<MPTraits>,
    //DPESNF<MPTraits>,
    //MPNNNF<MPTraits>,
    //CGALNF<MPTraits>,
    //SpillTreeNF<MPTraits>, 
    //MetricTreeNF<MPTraits>,
    //BandsNF<MPTraits>
    > NeighborhoodFinderMethodList;
  
  //types of samplers available in our world
  typedef boost::mpl::list<
    BridgeTestSampler<MPTraits>,
    GaussianSampler<MPTraits>,
    GridSampler<MPTraits>,
    //MedialAxisSampler<MPTraits>,
    MixSampler<MPTraits>,
    ObstacleBasedSampler<MPTraits>,
    UniformObstacleBasedSampler<MPTraits>,
    UniformRandomSampler<MPTraits>
      > SamplerMethodList;
  
  //types of local planners available in our world
  typedef boost::mpl::list<
    StraightLine<MPTraits>,
    //RotateAtS<MPTraits>,
    //TransformAtS<MPTraits>,
    //MedialAxisLP<MPTraits>,
    #if defined(PMPCfgSurface)
    SurfaceLP<MPTraits>,
    #endif
    //AStarDistance<MPTraits>,
    //AStarClearance<MPTraits>,
    ToggleLP<MPTraits>
    > LocalPlannerMethodList;

  //types of connectors available in our world
  typedef boost::mpl::list<
    NeighborhoodConnector<MPTraits>/*, 
    ConnectCCs<MPTraits>, 
    PreferentialAttachment<MPTraits>, 
    OptimalConnection<MPTraits>,
    OptimalRewire<MPTraits>,
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
    ConditionalEvaluator<MPTraits>,
    TrueEvaluation<MPTraits>,
    PrintMapEvaluation<MPTraits>, 
    Query<MPTraits>,
    LazyQuery<MPTraits>,
    LazyToggleQuery<MPTraits>,
    ComposeEvaluator<MPTraits>//,
    //NegateEvaluation<MPTraits>
    > MapEvaluatorMethodList;
  
  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    BasicPRM<MPTraits>,
    BasicRRTStrategy<MPTraits>,
    ResamplePointStrategy<MPTraits>,
    TogglePRMStrategy<MPTraits>
    > MPStrategyMethodList;
};

#endif
