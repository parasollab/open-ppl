#ifndef CFG_TRAITS_H_
#define CFG_TRAITS_H_

#include "MPProblem/MPProblem.h"
#include "PlanningLibrary/PlanningLibrary.h"

#include "MPProblem/ConfigurationSpace/Weight.h"

//distance metric includes
#include "PlanningLibrary/DistanceMetrics/BinaryLPSweptDistance.h"
#include "PlanningLibrary/DistanceMetrics/CenterOfMassDistance.h"
#include "PlanningLibrary/DistanceMetrics/EuclideanDistance.h"
#include "PlanningLibrary/DistanceMetrics/KnotTheoryDistance.h"
#include "PlanningLibrary/DistanceMetrics/LPSweptDistance.h"
#include "PlanningLibrary/DistanceMetrics/ManhattanDistance.h"
#include "PlanningLibrary/DistanceMetrics/RMSDDistance.h"
#include "PlanningLibrary/DistanceMetrics/ScaledEuclideanDistance.h"

//validity checker includes
#include "PlanningLibrary/ValidityCheckers/AlwaysTrueValidity.h"
#include "PlanningLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "PlanningLibrary/ValidityCheckers/ComposeValidity.h"
#include "PlanningLibrary/ValidityCheckers/MedialAxisClearanceValidity.h"
#include "PlanningLibrary/ValidityCheckers/NegateValidity.h"
#include "PlanningLibrary/ValidityCheckers/NodeClearanceValidity.h"
#include "PlanningLibrary/ValidityCheckers/ObstacleClearanceValidity.h"

//neighborhood finder includes
#include "PlanningLibrary/NeighborhoodFinders/BandsNF.h"
#include "PlanningLibrary/NeighborhoodFinders/BruteForceNF.h"
#include "PlanningLibrary/NeighborhoodFinders/DPESNF.h"
#include "PlanningLibrary/NeighborhoodFinders/HierarchicalNF.h"
#include "PlanningLibrary/NeighborhoodFinders/HopLimitNF.h"
#include "PlanningLibrary/NeighborhoodFinders/OptimalNF.h"
#include "PlanningLibrary/NeighborhoodFinders/RadiusNF.h"
#include "PlanningLibrary/NeighborhoodFinders/RandomNF.h"

//sampler includes
#include "PlanningLibrary/Samplers/BridgeTestSampler.h"
#include "PlanningLibrary/Samplers/GaussianSampler.h"
#include "PlanningLibrary/Samplers/GridSampler.h"
#include "PlanningLibrary/Samplers/MedialAxisSampler.h"
#include "PlanningLibrary/Samplers/MixSampler.h"
#include "PlanningLibrary/Samplers/ObstacleBasedSampler.h"
#include "PlanningLibrary/Samplers/SimilarStructureSampler.h"
#include "PlanningLibrary/Samplers/UniformMedialAxisSampler.h"
#include "PlanningLibrary/Samplers/UniformObstacleBasedSampler.h"
#include "PlanningLibrary/Samplers/UniformRandomSampler.h"
#include "PlanningLibrary/Samplers/WorkspaceImportanceSampler.h"

//local planner includes
#include "PlanningLibrary/LocalPlanners/AStar.h"
#include "PlanningLibrary/LocalPlanners/HierarchicalLP.h"
#include "PlanningLibrary/LocalPlanners/MedialAxisLP.h"
#include "PlanningLibrary/LocalPlanners/RotateAtS.h"
#include "PlanningLibrary/LocalPlanners/StraightLine.h"
#include "PlanningLibrary/LocalPlanners/ToggleLP.h"
#include "PlanningLibrary/LocalPlanners/TransformAtS.h"
#include "PlanningLibrary/LocalPlanners/ApproxSpheres.h"

//extenders includes
#include "PlanningLibrary/Extenders/BasicExtender.h"
#include "PlanningLibrary/Extenders/MedialAxisExtender.h"
#include "PlanningLibrary/Extenders/MixExtender.h"
#include "PlanningLibrary/Extenders/RandomObstacleVector.h"
#include "PlanningLibrary/Extenders/RotationThenTranslation.h"
#include "PlanningLibrary/Extenders/TraceCSpaceObstacle.h"
#include "PlanningLibrary/Extenders/TraceMAPush.h"
#include "PlanningLibrary/Extenders/TraceObstacle.h"

//path smoothing includes
#include "PlanningLibrary/PathModifiers/CombinedPathModifier.h"
#include "PlanningLibrary/PathModifiers/CRetractionPathModifier.h"
#include "PlanningLibrary/PathModifiers/MedialAxisPathModifier.h"
#include "PlanningLibrary/PathModifiers/ResamplePathModifier.h"
#include "PlanningLibrary/PathModifiers/ShortcuttingPathModifier.h"

//connector includes
#include "PlanningLibrary/Connectors/AdaptiveConnector.h"
#include "PlanningLibrary/Connectors/CCExpansion.h"
#include "PlanningLibrary/Connectors/CCsConnector.h"
#include "PlanningLibrary/Connectors/ClosestVE.h"
#include "PlanningLibrary/Connectors/NeighborhoodConnector.h"
#include "PlanningLibrary/Connectors/RewireConnector.h"

//metric includes
#include "PlanningLibrary/Metrics/CCDistanceMetric.h"
#include "PlanningLibrary/Metrics/ConnectivityMetric.h"
#include "PlanningLibrary/Metrics/CoverageDistanceMetric.h"
#include "PlanningLibrary/Metrics/CoverageMetric.h"
#include "PlanningLibrary/Metrics/DiameterMetric.h"
#include "PlanningLibrary/Metrics/NumEdgesMetric.h"
#include "PlanningLibrary/Metrics/NumNodesMetric.h"
#include "PlanningLibrary/Metrics/RoadmapSet.h"
#include "PlanningLibrary/Metrics/TimeMetric.h"
#include "PlanningLibrary/Metrics/VectorSet.h"

//map evaluator includes
#include "PlanningLibrary/MapEvaluators/ComposeEvaluator.h"
#include "PlanningLibrary/MapEvaluators/ConditionalEvaluator.h"
#include "PlanningLibrary/MapEvaluators/LazyQuery.h"
#include "PlanningLibrary/MapEvaluators/LazyToggleQuery.h"
#include "PlanningLibrary/MapEvaluators/NegateEvaluator.h"
#include "PlanningLibrary/MapEvaluators/PrintMapEvaluation.h"
#include "PlanningLibrary/MapEvaluators/PRMQuery.h"
#include "PlanningLibrary/MapEvaluators/RRTQuery.h"
#include "PlanningLibrary/MapEvaluators/TimeEvaluator.h"
#include "PlanningLibrary/MapEvaluators/TrueEvaluation.h"

//mp strategies includes
#include "PlanningLibrary/MPStrategies/AdaptiveRRT.h"
#include "PlanningLibrary/MPStrategies/BasicPRM.h"
#include "PlanningLibrary/MPStrategies/BasicRRTStrategy.h"
#include "PlanningLibrary/MPStrategies/DynamicDomainRRT.h"
#include "PlanningLibrary/MPStrategies/DynamicRegionRRT.h"
#include "PlanningLibrary/MPStrategies/EvaluateMapStrategy.h"
#include "PlanningLibrary/MPStrategies/HybridPRM.h"
#include "PlanningLibrary/MPStrategies/LPCompare.h"
#include "PlanningLibrary/MPStrategies/ModifyPath.h"
#include "PlanningLibrary/MPStrategies/MultiStrategy.h"
#include "PlanningLibrary/MPStrategies/PushQueryToMA.h"
#include "PlanningLibrary/MPStrategies/SparkPRM.h"
#include "PlanningLibrary/MPStrategies/SRTStrategy.h"
#include "PlanningLibrary/MPStrategies/TogglePRMStrategy.h"
#include "PlanningLibrary/MPStrategies/UnitTest/ClearanceTestStrategy.h"
#include "PlanningLibrary/MPStrategies/UnitTest/DMTestStrategy.h"
#include "PlanningLibrary/MPStrategies/UtilityGuidedGenerator.h"
#include "PlanningLibrary/MPStrategies/VisibilityBasedPRM.h"

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
template<class C, class W = DefaultWeight<C> >
struct MPTraits {

  typedef C CfgType;
  typedef W WeightType;
  typedef C& CfgRef;

  typedef MPProblem<MPTraits> MPProblemType;
  typedef PlanningLibrary<MPTraits> PlanningLibraryType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    BinaryLPSweptDistance<MPTraits>,
    CenterOfMassDistance<MPTraits>,
    EuclideanDistance<MPTraits>,
    KnotTheoryDistance<MPTraits>,
    LPSweptDistance<MPTraits>,
    ManhattanDistance<MPTraits>,
    MinkowskiDistance<MPTraits>,
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
    DPESNF<MPTraits>,
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
    UniformMedialAxisSampler<MPTraits>,
    UniformObstacleBasedSampler<MPTraits>,
    UniformRandomSampler<MPTraits>,
    WorkspaceImportanceSampler<MPTraits>
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
    TransformAtS<MPTraits>,
    ApproxSpheres<MPTraits>
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
    CRetractionPathModifier<MPTraits>,
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
    ClosestVE<MPTraits>,
    NeighborhoodConnector<MPTraits>,
    RewireConnector<MPTraits>
      > ConnectorMethodList;

  typedef ConnectivityMetric<MPTraits, RoadmapSet<MPTraits>>
      ConnectivityMetricRoadmapSet;
  typedef CoverageDistanceMetric<MPTraits, RoadmapSet<MPTraits>>
      CoverageDistanceMetricRoadmapSet;
  typedef CoverageMetric<MPTraits, RoadmapSet<MPTraits>>
      CoverageMetricRoadmapSet;

  typedef ConnectivityMetric<MPTraits, VectorSet<MPTraits>>
      ConnectivityMetricVectorSet;
  typedef CoverageDistanceMetric<MPTraits, VectorSet<MPTraits>>
      CoverageDistanceMetricVectorSet;
  typedef CoverageMetric<MPTraits, VectorSet<MPTraits>>
      CoverageMetricVectorSet;

  //types of metrics available in our world
  typedef boost::mpl::list<
    CCDistanceMetric<MPTraits>,
    ConnectivityMetricRoadmapSet,
    CoverageDistanceMetricRoadmapSet,
    CoverageMetricRoadmapSet,
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
    PRMQuery<MPTraits>,
    RRTQuery<MPTraits>,
    TimeEvaluator<MPTraits>,
    TrueEvaluation<MPTraits>
      > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    AdaptiveRRT<MPTraits>,
    BasicPRM<MPTraits>,
    BasicRRTStrategy<MPTraits>,
    ClearanceTestStrategy<MPTraits>,
    DMTestStrategy<MPTraits>,
    DynamicDomainRRT<MPTraits>,
    DynamicRegionRRT<MPTraits>,
    EvaluateMapStrategy<MPTraits>,
    HybridPRM<MPTraits>,
    LPCompare<MPTraits>,
    ModifyPath<MPTraits>,
    MultiStrategy<MPTraits>,
    PushQueryToMA<MPTraits>,
    SparkPRM<MPTraits, BasicPRM>,
    SparkPRM<MPTraits, TogglePRMStrategy>,
    SRTStrategy<MPTraits>,
    TogglePRMStrategy<MPTraits>,
    UtilityGuidedGenerator<MPTraits>,
    VisibilityBasedPRM<MPTraits>
      > MPStrategyMethodList;
};

#endif
