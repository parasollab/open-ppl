#ifndef CFG_TRAITS_H_
#define CFG_TRAITS_H_

#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/MPSolution.h"
#include "MPLibrary/MPTask.h"

#include "ConfigurationSpace/Path.h"
#include "ConfigurationSpace/Roadmap.h"
#include "ConfigurationSpace/Weight.h"

//distance metric includes
#include "MPLibrary/DistanceMetrics/BinaryLPSweptDistance.h"
#include "MPLibrary/DistanceMetrics/CenterOfMassDistance.h"
#include "MPLibrary/DistanceMetrics/EuclideanDistance.h"
#include "MPLibrary/DistanceMetrics/KnotTheoryDistance.h"
#include "MPLibrary/DistanceMetrics/LPSweptDistance.h"
#include "MPLibrary/DistanceMetrics/ManhattanDistance.h"
#include "MPLibrary/DistanceMetrics/RMSDDistance.h"
#include "MPLibrary/DistanceMetrics/ScaledEuclideanDistance.h"

//validity checker includes
#include "MPLibrary/ValidityCheckers/AlwaysTrueValidity.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "MPLibrary/ValidityCheckers/ComposeValidity.h"
#include "MPLibrary/ValidityCheckers/MedialAxisClearanceValidity.h"
#include "MPLibrary/ValidityCheckers/NegateValidity.h"
#include "MPLibrary/ValidityCheckers/NodeClearanceValidity.h"
#include "MPLibrary/ValidityCheckers/ObstacleClearanceValidity.h"

//neighborhood finder includes
#include "MPLibrary/NeighborhoodFinders/BandsNF.h"
#include "MPLibrary/NeighborhoodFinders/BruteForceNF.h"
#include "MPLibrary/NeighborhoodFinders/DPESNF.h"
#include "MPLibrary/NeighborhoodFinders/HierarchicalNF.h"
#include "MPLibrary/NeighborhoodFinders/HopLimitNF.h"
#include "MPLibrary/NeighborhoodFinders/OptimalNF.h"
#include "MPLibrary/NeighborhoodFinders/RadiusNF.h"
#include "MPLibrary/NeighborhoodFinders/RandomNF.h"

//sampler includes
#include "MPLibrary/Samplers/BridgeTestSampler.h"
#include "MPLibrary/Samplers/GaussianSampler.h"
#include "MPLibrary/Samplers/GridSampler.h"
#include "MPLibrary/Samplers/MedialAxisSampler.h"
#include "MPLibrary/Samplers/MixSampler.h"
#include "MPLibrary/Samplers/ObstacleBasedSampler.h"
#include "MPLibrary/Samplers/SimilarStructureSampler.h"
#include "MPLibrary/Samplers/UniformMedialAxisSampler.h"
#include "MPLibrary/Samplers/UniformObstacleBasedSampler.h"
#include "MPLibrary/Samplers/UniformRandomSampler.h"
#include "MPLibrary/Samplers/WorkspaceImportanceSampler.h"

//local planner includes
#include "MPLibrary/LocalPlanners/AStar.h"
#include "MPLibrary/LocalPlanners/HierarchicalLP.h"
#include "MPLibrary/LocalPlanners/MedialAxisLP.h"
#include "MPLibrary/LocalPlanners/RotateAtS.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "MPLibrary/LocalPlanners/ToggleLP.h"
#include "MPLibrary/LocalPlanners/TransformAtS.h"
#include "MPLibrary/LocalPlanners/ApproxSpheres.h"

//extenders includes
#include "MPLibrary/Extenders/BasicExtender.h"
#include "MPLibrary/Extenders/MedialAxisExtender.h"
#include "MPLibrary/Extenders/MixExtender.h"
#include "MPLibrary/Extenders/RandomObstacleVector.h"
#include "MPLibrary/Extenders/RotationThenTranslation.h"
#include "MPLibrary/Extenders/TraceCSpaceObstacle.h"
#include "MPLibrary/Extenders/TraceMAPush.h"
#include "MPLibrary/Extenders/TraceObstacle.h"

//path smoothing includes
#include "MPLibrary/PathModifiers/CombinedPathModifier.h"
#include "MPLibrary/PathModifiers/CRetractionPathModifier.h"
#include "MPLibrary/PathModifiers/MedialAxisPathModifier.h"
#include "MPLibrary/PathModifiers/ResamplePathModifier.h"
#include "MPLibrary/PathModifiers/ShortcuttingPathModifier.h"

//connector includes
#include "MPLibrary/Connectors/AdaptiveConnector.h"
#include "MPLibrary/Connectors/CCExpansion.h"
#include "MPLibrary/Connectors/CCsConnector.h"
#include "MPLibrary/Connectors/ClosestVE.h"
#include "MPLibrary/Connectors/NeighborhoodConnector.h"
#include "MPLibrary/Connectors/RewireConnector.h"

//metric includes
#include "MPLibrary/Metrics/CCDistanceMetric.h"
#include "MPLibrary/Metrics/ConnectivityMetric.h"
#include "MPLibrary/Metrics/CoverageDistanceMetric.h"
#include "MPLibrary/Metrics/CoverageMetric.h"
#include "MPLibrary/Metrics/DiameterMetric.h"
#include "MPLibrary/Metrics/NumEdgesMetric.h"
#include "MPLibrary/Metrics/NumNodesMetric.h"
#include "MPLibrary/Metrics/RoadmapSet.h"
#include "MPLibrary/Metrics/TimeMetric.h"
#include "MPLibrary/Metrics/VectorSet.h"

//map evaluator includes
#include "MPLibrary/MapEvaluators/ComposeEvaluator.h"
#include "MPLibrary/MapEvaluators/ConditionalEvaluator.h"
#include "MPLibrary/MapEvaluators/LazyQuery.h"
#include "MPLibrary/MapEvaluators/LazyToggleQuery.h"
#include "MPLibrary/MapEvaluators/NegateEvaluator.h"
#include "MPLibrary/MapEvaluators/PrintMapEvaluation.h"
#include "MPLibrary/MapEvaluators/PRMQuery.h"
#include "MPLibrary/MapEvaluators/RRTQuery.h"
#include "MPLibrary/MapEvaluators/TimeEvaluator.h"
#include "MPLibrary/MapEvaluators/TrueEvaluation.h"

//mp strategies includes
#include "MPLibrary/MPStrategies/AdaptiveRRT.h"
#include "MPLibrary/MPStrategies/BasicPRM.h"
#include "MPLibrary/MPStrategies/BasicRRTStrategy.h"
#include "MPLibrary/MPStrategies/DynamicDomainRRT.h"
#include "MPLibrary/MPStrategies/DynamicRegionRRT.h"
#include "MPLibrary/MPStrategies/EvaluateMapStrategy.h"
#include "MPLibrary/MPStrategies/HybridPRM.h"
#include "MPLibrary/MPStrategies/LPCompare.h"
#include "MPLibrary/MPStrategies/ModifyPath.h"
#include "MPLibrary/MPStrategies/MultiStrategy.h"
#include "MPLibrary/MPStrategies/PushQueryToMA.h"
#include "MPLibrary/MPStrategies/SparkPRM.h"
#include "MPLibrary/MPStrategies/SRTStrategy.h"
#include "MPLibrary/MPStrategies/TogglePRMStrategy.h"
#include "MPLibrary/MPStrategies/UnitTest/ClearanceTestStrategy.h"
#include "MPLibrary/MPStrategies/UnitTest/DMTestStrategy.h"
#include "MPLibrary/MPStrategies/UtilityGuidedGenerator.h"
#include "MPLibrary/MPStrategies/VisibilityBasedPRM.h"

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
template <typename C, typename W = DefaultWeight<C>>
struct MPTraits {

  typedef C                        CfgType;
  //typedef C&                       CfgRef;
  typedef W                        WeightType;
  typedef Path<MPTraits>           PathType;
  typedef Roadmap<MPTraits>        RoadmapType;
  typedef MPTaskType<MPTraits>     MPTask;
  typedef MPLibraryType<MPTraits>  MPLibrary;
  typedef MPSolutionType<MPTraits> MPSolution;

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
