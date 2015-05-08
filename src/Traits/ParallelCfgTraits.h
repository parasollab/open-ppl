#ifndef PARALLEL_CFG_TRAITS_H_
#define PARALLEL_CFG_TRAITS_H_

#include "MPProblem/Weight.h"

//distance metric includes
//#include "DistanceMetrics/BinaryLPSweptDistance.h"
//#include "DistanceMetrics/CenterOfMassDistance.h"
#include "DistanceMetrics/EuclideanDistance.h"
//#include "DistanceMetrics/KnotTheoryDistance.h"
//#include "DistanceMetrics/LPSweptDistance.h"
//#include "DistanceMetrics/ManhattanDistance.h"
//#include "DistanceMetrics/RMSDDistance.h"
#include "DistanceMetrics/ScaledEuclideanDistance.h"

//validity checker includes
#include "ValidityCheckers/AlwaysTrueValidity.h"
#include "ValidityCheckers/CollisionDetectionValidity.h"
//#include "ValidityCheckers/ComposeValidity.h"
//#include "ValidityCheckers/MedialAxisClearanceValidity.h"
//#include "ValidityCheckers/NegateValidity.h"
//#include "ValidityCheckers/NodeClearanceValidity.h"
//#include "ValidityCheckers/ObstacleClearanceValidity.h"

//neighborhood finder includes
//#include "NeighborhoodFinders/BandsNF.h"
#include "NeighborhoodFinders/BruteForceNF.h"
//#include "NeighborhoodFinders/HierarchicalNF.h"
//#include "NeighborhoodFinders/HopLimitNF.h"
//#include "NeighborhoodFinders/OptimalNF.h"
#include "NeighborhoodFinders/RadiusNF.h"
//#include "NeighborhoodFinders/RandomNF.h"

//sampler includes
#include "Samplers/BridgeTestSampler.h"
#include "Samplers/GaussianSampler.h"
//#include "Samplers/GridSampler.h"
//#include "Samplers/MedialAxisSampler.h"
//#include "Samplers/MixSampler.h"
#include "Samplers/ObstacleBasedSampler.h"
//#include "Samplers/SimilarStructureSampler.h"
//#include "Samplers/UniformMedialAxisSampler.h"
//#include "Samplers/UniformObstacleBasedSampler.h"
#include "Samplers/UniformRandomSampler.h"

//local planner includes
//#include "LocalPlanners/AStar.h"
//#include "LocalPlanners/HierarchicalLP.h"
//#include "LocalPlanners/MedialAxisLP.h"
//#include "LocalPlanners/RotateAtS.h"
#include "LocalPlanners/StraightLine.h"
//#include "LocalPlanners/ToggleLP.h"
//#include "LocalPlanners/TransformAtS.h"
//#include "LocalPlanners/ApproxSpheres.h"

//extenders includes
#include "Extenders/BasicExtender.h"
//#include "Extenders/MixExtender.h"
//#include "Extenders/RandomObstacleVector.h"
//#include "Extenders/RotationThenTranslation.h"
//#include "Extenders/TraceCSpaceObstacle.h"
//#include "Extenders/TraceMAPush.h"
//#include "Extenders/TraceObstacle.h"

//path smoothing includes
//#include "PathModifiers/CombinedPathModifier.h"
//#include "PathModifiers/MedialAxisPathModifier.h"
//#include "PathModifiers/ResamplePathModifier.h"
//#include "PathModifiers/ShortcuttingPathModifier.h"

//connector includes
//#include "Connectors/AdaptiveConnector.h"
#include "Connectors/CCsConnector.h"
#include "Connectors/NeighborhoodConnector.h"
#include "Connectors/RegionConnector.h"
#include "Connectors/RegionRRTConnect.h"
#include "Connectors/RewireConnector.h"
#include "Connectors/RRTConnect.h"
//#include "Connectors/CCExpansion.h"
//#include "Connectors/ClosestVE.h"

//metric includes
//#include "Metrics/CCDistanceMetric.h"
//#include "Metrics/ConnectivityMetric.h"
//#include "Metrics/CoverageDistanceMetric.h"
//#include "Metrics/CoverageMetric.h"
//#include "Metrics/DiameterMetric.h"
//#include "Metrics/NumEdgesMetric.h"
#include "Metrics/NumNodesMetric.h"
//#include "Metrics/RoadmapSet.h"
//#include "Metrics/TimeMetric.h"
//#include "Metrics/VectorSet.h"

//map evaluator includes
//#include "MapEvaluators/ComposeEvaluator.h"
#include "MapEvaluators/ConditionalEvaluator.h"
//#include "MapEvaluators/LazyQuery.h"
//#include "MapEvaluators/LazyToggleQuery.h"
//#include "MapEvaluators/NegateEvaluator.h"
//#include "MapEvaluators/PrintMapEvaluation.h"
#include "MapEvaluators/Query.h"
//#include "MapEvaluators/ReplanningEvaluation.h"
//#include "MapEvaluators/TrueEvaluation.h"


//mp strategies includes
//#include "MPStrategies/AdaptiveRRT.h"
//#include "MPStrategies/BasicPRM.h"
//#include "MPStrategies/BasicRRTStrategy.h"
//#include "MPStrategies/EvaluateMapStrategy.h"
//#include "MPStrategies/LocalManeuveringStrategy.h"
//#include "MPStrategies/MedialAxisRRT.h"
//#include "MPStrategies/MultiStrategy.h"
//#include "MPStrategies/SparkPRM.h"
//#include "MPStrategies/SRTStrategy.h"
//#include "MPStrategies/TogglePRMStrategy.h"
//#include "MPStrategies/UnitTest/DMTestStrategy.h"
//#include "MPStrategies/UtilityGuidedGenerator.h"
//#include "MPStrategies/VisibilityBasedPRM.h"
#include "MPStrategies/BlindRRT.h"
#include "ParallelMethods/BasicParallelPRM.h"
#include "ParallelMethods/BulkRRT.h"
#include "ParallelMethods/RadialBlindRRT.h"
#include "ParallelMethods/RadialSubdivisionRRT.h"
#include "ParallelMethods/RegularSubdivisionMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief Defines available methods in the Motion Planning Universe for Cfg
///        under parallel compile
/// @tparam C Cfg type
/// @tparam W Weight type
///
/// MPTraits is a type class which defines the motion planning
/// universe. We construct our methods through a factory design pattern, and
/// thus this states all available classes within an abstraction that you can
/// use in the system. Essentially the important types are, the CfgType or the
/// @cspace abstraction class, the WeightType or the edge type of the graph, and
/// a type list for each algorithm abstraction --- here you only need to define
/// what you need, as extraneous methods in the type class imply longer compile
/// times.
////////////////////////////////////////////////////////////////////////////////
template<class C, class W = DefaultWeight<C> >
struct MPTraits {
  typedef C CfgType;
  typedef W WeightType;
  typedef C CfgRef;

  typedef MPProblem<MPTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
    //BinaryLPSweptDistance<MPTraits>,
    //CenterOfMassDistance<MPTraits>,
    EuclideanDistance<MPTraits>,
    //KnotTheoryDistance<MPTraits>,
    //LPSweptDistance<MPTraits>,
    //ManhattanDistance<MPTraits>,
    //MinkowskiDistance<MPTraits>,
    //RMSDDistance<MPTraits>,
    ScaledEuclideanDistance<MPTraits>
      > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
    AlwaysTrueValidity<MPTraits>,
    CollisionDetectionValidity<MPTraits>//,
    //ComposeValidity<MPTraits>,
    //MedialAxisClearanceValidity<MPTraits>,
    //NegateValidity<MPTraits>,
    //NodeClearanceValidity<MPTraits>,
    //ObstacleClearanceValidity<MPTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
    //BandsNF<MPTraits>,
    BruteForceNF<MPTraits>,
    //CGALNF<MPTraits>,
    //DPESNF<MPTraits>,
    //HierarchicalNF<MPTraits>,
    //HopLimitNF<MPTraits>,
    //MetricTreeNF<MPTraits>,
    //MPNNNF<MPTraits>,
    //OptimalNF<MPTraits>,
    RadiusNF<MPTraits>//,
    //RandomNF<MPTraits>//,
    //SpillTreeNF<MPTraits>
    > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    BridgeTestSampler<MPTraits>,
    GaussianSampler<MPTraits>,
    //GridSampler<MPTraits>,
    //MedialAxisSampler<MPTraits>,
    //MixSampler<MPTraits>,
    ObstacleBasedSampler<MPTraits>,
    //SimilarStructureSampler<MPTraits>,
    //UniformMedialAxisSampler<MPTraits>,
    //UniformObstacleBasedSampler<MPTraits>,
    UniformRandomSampler<MPTraits>
      > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
    //AStarClearance<MPTraits>,
    //AStarDistance<MPTraits>,
    //HierarchicalLP<MPTraits>,
    //MedialAxisLP<MPTraits>,
    //RotateAtS<MPTraits>,
    StraightLine<MPTraits>//,
    //ToggleLP<MPTraits>,
    //TransformAtS<MPTraits>,
    //ApproxSpheres<MPTraits>
      > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
    BasicExtender<MPTraits>//,
    //MixExtender<MPTraits>,
    //RandomObstacleVector<MPTraits>,
    //RotationThenTranslation<MPTraits>,
    //TraceCSpaceObstacle<MPTraits>,
    //TraceMAPush<MPTraits>,
    //TraceObstacle<MPTraits>
      > ExtenderMethodList;

  //types of path smoothing available in our world

  typedef boost::mpl::list<
    //CombinedPathModifier<MPTraits>,
    //MedialAxisPathModifier<MPTraits>,
    //ResamplePathModifier<MPTraits>,
    //ShortcuttingPathModifier<MPTraits>
    > PathModifierMethodList;


  //types of connectors available in our world
  typedef boost::mpl::list<
    //AdaptiveConnector<MPTraits>,
    //CCExpansion<MPTraits>,
    CCsConnector<MPTraits>,
    //ClosestVE<MPTraits>,
    NeighborhoodConnector<MPTraits>,
    //PreferentialAttachment<MPTraits>,
    RegionConnector<MPTraits>,
    RegionRRTConnect<MPTraits>,
    RewireConnector<MPTraits>,
    RRTConnect<MPTraits>
      > ConnectorMethodList;

  //typedef ConnectivityMetric<MPTraits, RoadmapSet<MPTraits> > ConnectivityMetricRoadmapSet;
  //typedef CoverageDistanceMetric<MPTraits, RoadmapSet<MPTraits> > CoverageDistanceMetricRoadmapSet;
  //typedef CoverageMetric<MPTraits, RoadmapSet<MPTraits> > CoverageMetricRoadmapSet;

  //typedef ConnectivityMetric<MPTraits, VectorSet<MPTraits> > ConnectivityMetricVectorSet;
  //typedef CoverageDistanceMetric<MPTraits, VectorSet<MPTraits> > CoverageDistanceMetricVectorSet;
  //typedef CoverageMetric<MPTraits, VectorSet<MPTraits> > CoverageMetricVectorSet;

  //types of metrics available in our world
  typedef boost::mpl::list<
    //CCDistanceMetric<MPTraits>,
    //ConnectivityMetricRoadmapSet,
    //CoverageDistanceMetricRoadmapSet,
    //CoverageMetricRoadmapSet,
    //ConnectivityMetricVectorSet,
    //CoverageDistanceMetricVectorSet,
    //CoverageMetricVectorSet,
    //DiameterMetric<MPTraits>,
    //NumEdgesMetric<MPTraits>,
    NumNodesMetric<MPTraits>//,
    //TimeMetric<MPTraits>
    > MetricMethodList;


  //types of map evaluators available in our world
  typedef boost::mpl::list<
    //ComposeEvaluator<MPTraits>,
    ConditionalEvaluator<MPTraits>//,
    //LazyQuery<MPTraits>,
    //LazyToggleQuery<MPTraits>,
    //NegateEvaluator<MPTraits>,
    //PrintMapEvaluation<MPTraits>,
    //Query<MPTraits>,
    //ReplanningEvaluation<MPTraits>,
    //TrueEvaluation<MPTraits>
    > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    //AdaptiveRRT<MPTraits>,
    //BasicPRM<MPTraits>,
    //BasicRRTStrategy<MPTraits>,
    //DMTestStrategy<MPTraits>,
    //EvaluateMapStrategy<MPTraits>,
    //MedialAxisRRT<MPTraits>,
    //MultiStrategy<MPTraits>,
    //SparkPRM<MPTraits, BasicPRM>,
    //SparkPRM<MPTraits, TogglePRMStrategy>,
    //SRTStrategy<MPTraits>,
    //TogglePRMStrategy<MPTraits>,
    //UtilityGuidedGenerator<MPTraits>,
    //VisibilityBasedPRM<MPTraits>

    BasicParallelPRM<MPTraits>,
    BlindRRT<MPTraits>,
    BulkRRT<MPTraits>,
    RadialBlindRRT<MPTraits>,
    RadialSubdivisionRRT<MPTraits>,
    RegularSubdivisionMethod<MPTraits>
    > MPStrategyMethodList;
};

#endif
