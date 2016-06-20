#ifndef RV_TRAITS_H_
#define RV_TRAITS_H_

#include "Cfg/CfgReachableVolume.h"
#include "MPProblem/Weight.h"

//distance metric includes
#include "DistanceMetrics/BinaryLPSweptDistance.h"
#include "DistanceMetrics/CenterOfMassDistance.h"
#include "DistanceMetrics/EuclideanDistance.h"
#include "DistanceMetrics/KnotTheoryDistance.h"
#include "DistanceMetrics/LPSweptDistance.h"
#include "DistanceMetrics/ManhattanDistance.h"
#include "DistanceMetrics/ReachableDistance.h"
#include "DistanceMetrics/RMSDDistance.h"
#include "DistanceMetrics/ScaledEuclideanDistance.h"
#include "DistanceMetrics/RVDistance.h"

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
#include "Samplers/UniformRandomSampler.h"

//local planner includes
#include "LocalPlanners/RotateAtS.h"
#include "LocalPlanners/StraightLine.h"
#include "LocalPlanners/TransformAtS.h"
#include "LocalPlanners/RVLocalPlanner.h"

//extenders includes
#include "Extenders/BasicExtender.h"

//path smoothing includes
#include "PathModifiers/CombinedPathModifier.h"
#include "PathModifiers/MedialAxisPathModifier.h"
#include "PathModifiers/ResamplePathModifier.h"
#include "PathModifiers/ShortcuttingPathModifier.h"

//connector includes
#include "Connectors/AdaptiveConnector.h"
#include "Connectors/CCExpansion.h"
#include "Connectors/CCsConnector.h"
#include "Connectors/ClosestVE.h"
#include "Connectors/NeighborhoodConnector.h"
#include "Connectors/RRTConnect.h"

//metric includes
#include "Metrics/CCDistanceMetric.h"
#include "Metrics/ConnectivityMetric.h"
#include "Metrics/CoverageDistanceMetric.h"
#include "Metrics/CoverageMetric.h"
#include "Metrics/DiameterMetric.h"
#include "Metrics/NumEdgesMetric.h"
#include "Metrics/NumNodesMetric.h"
#include "Metrics/RoadmapSet.h"
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
#include "MPStrategies/BasicPRM.h"
#include "MPStrategies/BasicRRTStrategy.h"
#include "MPStrategies/RVRRT.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief Defines available methods in the Motion Planning Universe for
///        ReachableVolumeCfg
///
/// CfgReachableVolumeTraits is a type class which defines the motion planning universe. We
/// construct our methods through a factory design pattern, and thus this states
/// all available classes within an abstraction that you can use in the system.
/// Essentially the important types are, the CfgType or the @cspace abstraction
/// class, the WeightType or the edge type of the graph, and a type list for
/// each algorithm abstraction --- here you only need to define what you need,
/// as extraneous methods in the type class imply longer compile times.
////////////////////////////////////////////////////////////////////////////////
struct CfgReachableVolumeTraits {

  typedef CfgReachableVolume CfgType;
  typedef DefaultWeight<CfgType> WeightType;
  typedef C& CfgRef;

  typedef MPProblem<CfgReachableVolumeTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    BinaryLPSweptDistance<CfgReachableVolumeTraits>,
    CenterOfMassDistance<CfgReachableVolumeTraits>,
    EuclideanDistance<CfgReachableVolumeTraits>,
    KnotTheoryDistance<CfgReachableVolumeTraits>,
    LPSweptDistance<CfgReachableVolumeTraits>,
    ManhattanDistance<CfgReachableVolumeTraits>,
    MinkowskiDistance<CfgReachableVolumeTraits>,
    RMSDDistance<CfgReachableVolumeTraits>,
    ScaledEuclideanDistance<CfgReachableVolumeTraits>,
    RVDistance<CfgReachableVolumeTraits>
      > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    AlwaysTrueValidity<CfgReachableVolumeTraits>,
    CollisionDetectionValidity<CfgReachableVolumeTraits>,
    ComposeValidity<CfgReachableVolumeTraits>,
    MedialAxisClearanceValidity<CfgReachableVolumeTraits>,
    NegateValidity<CfgReachableVolumeTraits>,
    NodeClearanceValidity<CfgReachableVolumeTraits>,
    ObstacleClearanceValidity<CfgReachableVolumeTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BandsNF<CfgReachableVolumeTraits>,
    BruteForceNF<CfgReachableVolumeTraits>,
    //CGALNF<CfgReachableVolumeTraits>,
    //DPESNF<CfgReachableVolumeTraits>,
    HierarchicalNF<CfgReachableVolumeTraits>,
    HopLimitNF<CfgReachableVolumeTraits>,
    //MetricTreeNF<CfgReachableVolumeTraits>,
    //MPNNNF<CfgReachableVolumeTraits>,
    OptimalNF<CfgReachableVolumeTraits>,
    RadiusNF<CfgReachableVolumeTraits>,
    RandomNF<CfgReachableVolumeTraits>//,
    //SpillTreeNF<CfgReachableVolumeTraits>
    > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    UniformRandomSampler<CfgReachableVolumeTraits>
      > SamplerMethodList;

  //types of local planners available in our world
  //All of these samplers are applicable to problems without constraints
  //The RV Local planner is the only method that is applicable to problems with constraints
  typedef boost::mpl::list<
    RotateAtS<CfgReachableVolumeTraits>,
    StraightLine<CfgReachableVolumeTraits>,
    TransformAtS<CfgReachableVolumeTraits>,
    RVLocalPlanner<CfgReachableVolumeTraits>
      > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
    BasicExtender<CfgReachableVolumeTraits>
      > ExtenderMethodList;

  //types of path smoothing available in our world
  //path smoothing can only be applied to problems without constraints
  typedef boost::mpl::list<
    CombinedPathModifier<CfgReachableVolumeTraits>,
    MedialAxisPathModifier<CfgReachableVolumeTraits>,
    ResamplePathModifier<CfgReachableVolumeTraits>,
    ShortcuttingPathModifier<CfgReachableVolumeTraits>
      > PathModifierMethodList;


  //types of connectors available in our world
  typedef boost::mpl::list<
    AdaptiveConnector<CfgReachableVolumeTraits>,
    CCExpansion<CfgReachableVolumeTraits>,
    CCsConnector<CfgReachableVolumeTraits>,
    ClosestVE<CfgReachableVolumeTraits>,
    NeighborhoodConnector<CfgReachableVolumeTraits>,
    //PreferentialAttachment<CfgReachableVolumeTraits>,
    RRTConnect<CfgReachableVolumeTraits>
      > ConnectorMethodList;

  typedef ConnectivityMetric<CfgReachableVolumeTraits, RoadmapSet<CfgReachableVolumeTraits> > ConnectivityMetricRoadmapSet;
  typedef CoverageDistanceMetric<CfgReachableVolumeTraits, RoadmapSet<CfgReachableVolumeTraits> > CoverageDistanceMetricRoadmapSet;
  typedef CoverageMetric<CfgReachableVolumeTraits, RoadmapSet<CfgReachableVolumeTraits> > CoverageMetricRoadmapSet;

  typedef ConnectivityMetric<CfgReachableVolumeTraits, VectorSet<CfgReachableVolumeTraits> > ConnectivityMetricVectorSet;
  typedef CoverageDistanceMetric<CfgReachableVolumeTraits, VectorSet<CfgReachableVolumeTraits> > CoverageDistanceMetricVectorSet;
  typedef CoverageMetric<CfgReachableVolumeTraits, VectorSet<CfgReachableVolumeTraits> > CoverageMetricVectorSet;

  //types of metrics available in our world
  typedef boost::mpl::list<
    CCDistanceMetric<CfgReachableVolumeTraits>,
    ConnectivityMetricRoadmapSet,
    CoverageDistanceMetricRoadmapSet,
    CoverageMetricRoadmapSet,
    ConnectivityMetricVectorSet,
    CoverageDistanceMetricVectorSet,
    CoverageMetricVectorSet,
    DiameterMetric<CfgReachableVolumeTraits>,
    NumEdgesMetric<CfgReachableVolumeTraits>,
    NumNodesMetric<CfgReachableVolumeTraits>,
    TimeMetric<CfgReachableVolumeTraits>
      > MetricMethodList;


  //types of map evaluators available in our world
  typedef boost::mpl::list<
    ComposeEvaluator<CfgReachableVolumeTraits>,
    ConditionalEvaluator<CfgReachableVolumeTraits>,
    LazyQuery<CfgReachableVolumeTraits>,
    LazyToggleQuery<CfgReachableVolumeTraits>,
    NegateEvaluator<CfgReachableVolumeTraits>,
    PrintMapEvaluation<CfgReachableVolumeTraits>,
    Query<CfgReachableVolumeTraits>,
    ReplanningEvaluation<CfgReachableVolumeTraits>,
    TrueEvaluation<CfgReachableVolumeTraits>
      > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    BasicPRM<CfgReachableVolumeTraits>,
    BasicRRTStrategy<CfgReachableVolumeTraits>,
    ReachableVolumeRRT<CfgReachableVolumeTraits>
      > MPStrategyMethodList;
};

#endif
