#ifndef PMPL_CFG_TRAITS_H_
#define PMPL_CFG_TRAITS_H_

#include "MPLibrary/GoalTracker.h"
#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/MPSolution.h"
#include "MPLibrary/MPTools/MPTools.h"

#include "ConfigurationSpace/LocalObstacleMap.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupPath.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "ConfigurationSpace/Path.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "ConfigurationSpace/Weight.h"

//distance metric includes
#include "MPLibrary/DistanceMetrics/BinaryLPSweptDistance.h"
#include "MPLibrary/DistanceMetrics/EuclideanDistance.h"
#include "MPLibrary/DistanceMetrics/KnotTheoryDistance.h"
#include "MPLibrary/DistanceMetrics/LPSweptDistance.h"
#include "MPLibrary/DistanceMetrics/ManhattanDistance.h"
#include "MPLibrary/DistanceMetrics/RMSDDistance.h"
#include "MPLibrary/DistanceMetrics/ScaledEuclideanDistance.h"
#include "MPLibrary/DistanceMetrics/TopologicalDistance.h"
#include "MPLibrary/DistanceMetrics/WeightedEuclideanDistance.h"
#include "MPLibrary/DistanceMetrics/WorkspaceTranslationDistance.h"

//validity checker includes
#include "MPLibrary/ValidityCheckers/AlwaysTrueValidity.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "MPLibrary/ValidityCheckers/ComposeValidity.h"
#include "MPLibrary/ValidityCheckers/MedialAxisClearanceValidity.h"
#include "MPLibrary/ValidityCheckers/NegateValidity.h"
#include "MPLibrary/ValidityCheckers/NodeClearanceValidity.h"
#include "MPLibrary/ValidityCheckers/ObstacleClearanceValidity.h"
#include "MPLibrary/ValidityCheckers/TerrainValidityChecker.h"
#include "MPLibrary/ValidityCheckers/TopologicalMapValidity.h"

//neighborhood finder includes
#include "MPLibrary/NeighborhoodFinders/BruteForceNF.h"
#include "MPLibrary/NeighborhoodFinders/DPESNF.h"
#include "MPLibrary/NeighborhoodFinders/HierarchicalNF.h"
#include "MPLibrary/NeighborhoodFinders/KdTreeNF.h"
#include "MPLibrary/NeighborhoodFinders/LSHNF.h"
#include "MPLibrary/NeighborhoodFinders/OptimalNF.h"
#include "MPLibrary/NeighborhoodFinders/RadiusNF.h"
#include "MPLibrary/NeighborhoodFinders/RandomNF.h"
#include "MPLibrary/NeighborhoodFinders/TopologicalFilter.h"

//sampler includes
#include "MPLibrary/Samplers/BridgeTestSampler.h"
#include "MPLibrary/Samplers/DynamicRegionSampler.h"
#include "MPLibrary/Samplers/GaussianSampler.h"
#include "MPLibrary/Samplers/GridSampler.h"
#include "MPLibrary/Samplers/MaskedProximitySamplerGroup.h"
#include "MPLibrary/Samplers/MaskedSamplerMethodGroup.h"
#include "MPLibrary/Samplers/MatingNormalSamplerGroup.h"
#include "MPLibrary/Samplers/MedialAxisSampler.h"
#include "MPLibrary/Samplers/MixSampler.h"
#include "MPLibrary/Samplers/ObstacleBasedSampler.h"
#include "MPLibrary/Samplers/ReachableVolumeSampler.h"
#include "MPLibrary/Samplers/UniformMedialAxisSampler.h"
#include "MPLibrary/Samplers/UniformObstacleBasedSampler.h"
#include "MPLibrary/Samplers/UniformRandomSampler.h"
#include "MPLibrary/Samplers/WorkspaceImportanceSampler.h"

//local planner includes
#include "MPLibrary/LocalPlanners/HierarchicalLP.h"
#include "MPLibrary/LocalPlanners/MedialAxisLP.h"
#include "MPLibrary/LocalPlanners/RotateAtS.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "MPLibrary/LocalPlanners/ToggleLP.h"
#include "MPLibrary/LocalPlanners/TransformAtS.h"

//extenders includes
#include "MPLibrary/Extenders/BasicExtender.h"
#include "MPLibrary/Extenders/KinodynamicExtender.h"
#include "MPLibrary/Extenders/LimitedDistanceExtender.h"
#ifdef PMPL_USE_MATLAB
#include "MPLibrary/Extenders/MatlabNeedleExtender.h"
#endif
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
#include "MPLibrary/Connectors/CCsConnector.h"
#include "MPLibrary/Connectors/NeighborhoodConnector.h"
#include "MPLibrary/Connectors/RewireConnector.h"

//metric includes
#include "MPLibrary/Metrics/NumEdgesMetric.h"
#include "MPLibrary/Metrics/NumNodesMetric.h"
#include "MPLibrary/Metrics/TimeMetric.h"

//map evaluator includes
#include "MPLibrary/MapEvaluators/ComposeEvaluator.h"
#include "MPLibrary/MapEvaluators/ConditionalEvaluator.h"
#include "MPLibrary/MapEvaluators/GroupCBSQuery.h"
#include "MPLibrary/MapEvaluators/GroupDecoupledQuery.h"
#include "MPLibrary/MapEvaluators/GroupQuery.h"
#include "MPLibrary/MapEvaluators/IterationCountEvaluator.h"
#include "MPLibrary/MapEvaluators/LazyQuery.h"
#include "MPLibrary/MapEvaluators/LazyToggleQuery.h"
#include "MPLibrary/MapEvaluators/MinimumClearanceEvaluator.h"
#include "MPLibrary/MapEvaluators/MinimumDistanceEvaluator.h"
#include "MPLibrary/MapEvaluators/NegateEvaluator.h"
#include "MPLibrary/MapEvaluators/PrintMapEvaluation.h"
#include "MPLibrary/MapEvaluators/QueryMethod.h"
#include "MPLibrary/MapEvaluators/StrategyStateEvaluator.h"
#include "MPLibrary/MapEvaluators/TimeEvaluator.h"
#include "MPLibrary/MapEvaluators/TrueEvaluation.h"

//mp strategies includes
#include "MPLibrary/MPStrategies/AdaptiveRRT.h"
#include "MPLibrary/MPStrategies/BasicPRM.h"
#include "MPLibrary/MPStrategies/BasicRRTStrategy.h"
#include "MPLibrary/MPStrategies/DisassemblyExhaustiveGraph.h"
#include "MPLibrary/MPStrategies/DisassemblyParallel.h"
#include "MPLibrary/MPStrategies/DisassemblyParallelizedSAs.h"
#include "MPLibrary/MPStrategies/DisassemblyPreemptiveDFSManipulator.h"
#include "MPLibrary/MPStrategies/DisassemblyRRTStrategy.h"
#include "MPLibrary/MPStrategies/DisassemblyIMLRRT.h"
#include "MPLibrary/MPStrategies/DynamicDomainRRT.h"
#include "MPLibrary/MPStrategies/EvaluateMapStrategy.h"
#include "MPLibrary/MPStrategies/GroupDecoupledStrategy.h"
#include "MPLibrary/MPStrategies/GroupPRM.h"
#include "MPLibrary/MPStrategies/GroupStrategyMethod.h"
#include "MPLibrary/MPStrategies/ModifyPath.h"
#include "MPLibrary/MPStrategies/MultiStrategy.h"
#include "MPLibrary/MPStrategies/PushCfgToMATest.h"
#include "MPLibrary/MPStrategies/ScratchStrategy.h"
#include "MPLibrary/MPStrategies/StableSparseRRT.h"
#include "MPLibrary/MPStrategies/Syclop.h"
#include "MPLibrary/MPStrategies/TogglePRMStrategy.h"
//#include "MPLibrary/MPStrategies/UnitTest/BoundaryTest.h"
//#include "MPLibrary/MPStrategies/UnitTest/DMTestStrategy.h"
//#include "MPLibrary/MPStrategies/UnitTest/SVMTest.h"
#include "MPLibrary/MPStrategies/UtilityGuidedGenerator.h"
#include "MPLibrary/MPStrategies/ValidationStrategy.h"
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

  typedef C                               CfgType;
  typedef W                               WeightType;
  typedef RoadmapGraph<C, W>              RoadmapType;
  typedef PathType<MPTraits>              Path;
  typedef MPLibraryType<MPTraits>         MPLibrary;
  typedef MPSolutionType<MPTraits>        MPSolution;
  typedef MPToolsType<MPTraits>           MPTools;
  typedef LocalObstacleMapType<MPTraits>  LocalObstacleMap;
  typedef GoalTrackerType<MPTraits>       GoalTracker;

  typedef GroupLocalPlan<CfgType>                    GroupWeightType;
  typedef GroupRoadmap<GroupCfg, GroupWeightType>    GroupRoadmapType;
  typedef GroupPath<MPTraits>                        GroupPathType;
  typedef GroupCfg                                   GroupCfgType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    BinaryLPSweptDistance<MPTraits>,
    EuclideanDistance<MPTraits>,
    KnotTheoryDistance<MPTraits>,
    LPSweptDistance<MPTraits>,
    ManhattanDistance<MPTraits>,
    MinkowskiDistance<MPTraits>,
    RMSDDistance<MPTraits>,
    ScaledEuclideanDistance<MPTraits>,
    TopologicalDistance<MPTraits>,
    WeightedEuclideanDistance<MPTraits>,
    WorkspaceTranslationDistance<MPTraits>
      > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    AlwaysTrueValidity<MPTraits>,
    CollisionDetectionValidity<MPTraits>,
    ComposeValidity<MPTraits>,
    MedialAxisClearanceValidity<MPTraits>,
    NegateValidity<MPTraits>,
    NodeClearanceValidity<MPTraits>,
    ObstacleClearanceValidity<MPTraits>,
    TerrainValidityChecker<MPTraits>,
    TopologicalMapValidity<MPTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    BruteForceNF<MPTraits>,
    DPESNF<MPTraits>,
    HierarchicalNF<MPTraits>,
    KdTreeNF<MPTraits>,
    LSHNF<MPTraits>,
    OptimalNF<MPTraits>,
    RadiusNF<MPTraits>,
    RandomNF<MPTraits>,
    TopologicalFilter<MPTraits>
      > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    BridgeTestSampler<MPTraits>,
    DynamicRegionSampler<MPTraits>,
    GaussianSampler<MPTraits>,
    GridSampler<MPTraits>,
    MaskedProximitySamplerGroup<MPTraits>,
    MatingNormalSamplerGroup<MPTraits>,
    MedialAxisSampler<MPTraits>,
    MixSampler<MPTraits>,
    ObstacleBasedSampler<MPTraits>,
    ReachableVolumeSampler<MPTraits>,
    UniformMedialAxisSampler<MPTraits>,
    UniformObstacleBasedSampler<MPTraits>,
    UniformRandomSampler<MPTraits>,
    WorkspaceImportanceSampler<MPTraits>
      > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
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
    KinodynamicExtender<MPTraits>,
    LimitedDistanceExtender<MPTraits>,
#ifdef PMPL_USE_MATLAB
    MatlabNeedleExtender<MPTraits>,
#endif
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
    CCsConnector<MPTraits>,
    NeighborhoodConnector<MPTraits>,
    RewireConnector<MPTraits>
      > ConnectorMethodList;

  //types of metrics available in our world
  typedef boost::mpl::list<
    NumEdgesMetric<MPTraits>,
    NumNodesMetric<MPTraits>,
    TimeMetric<MPTraits>
      > MetricMethodList;


  //types of map evaluators available in our world
  typedef boost::mpl::list<
    ComposeEvaluator<MPTraits>,
    GroupCBSQuery<MPTraits>,
    GroupDecoupledQuery<MPTraits>,
    GroupQuery<MPTraits>,
    ConditionalEvaluator<MPTraits>,
    IterationCountEvaluator<MPTraits>,
    LazyQuery<MPTraits>,
    LazyToggleQuery<MPTraits>,
    MinimumClearanceEvaluator<MPTraits>,
    MinimumDistanceEvaluator<MPTraits>,
    NegateEvaluator<MPTraits>,
    PrintMapEvaluation<MPTraits>,
    QueryMethod<MPTraits>,
    StrategyStateEvaluator<MPTraits>,
    TimeEvaluator<MPTraits>,
    TrueEvaluation<MPTraits>
      > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
#if 0
    AdaptiveRRT<MPTraits>, Fix after ICRA 18
#endif
    BasicPRM<MPTraits>,
    BasicRRTStrategy<MPTraits>,
    DisassemblyExhaustiveGraph<MPTraits>,
    DisassemblyParallel<MPTraits>,
    DisassemblyParallelizedSAs<MPTraits>,
    DisassemblyPreemptiveDFSManipulator<MPTraits>,
    DisassemblyRRTStrategy<MPTraits>,
    DisassemblyIMLRRT<MPTraits>,
    DynamicDomainRRT<MPTraits>,
    EvaluateMapStrategy<MPTraits>,
    GroupDecoupledStrategy<MPTraits>,
    GroupPRM<MPTraits>,
    ModifyPath<MPTraits>,
    MultiStrategy<MPTraits>,
    ScratchStrategy<MPTraits>,
    StableSparseRRT<MPTraits>,
    Syclop<MPTraits>,
    TogglePRMStrategy<MPTraits>,
    UtilityGuidedGenerator<MPTraits>,
    VisibilityBasedPRM<MPTraits>,
    ValidationStrategy<MPTraits>,

    //BoundaryTest<MPTraits>,
    //DMTestStrategy<MPTraits>,
    PushCfgToMATest<MPTraits>
    //SVMTest<MPTraits>
      > MPStrategyMethodList;
};

#endif
