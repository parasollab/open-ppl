#ifndef PPL_TEST_TRAITS_H_
#define PPL_TEST_TRAITS_H_

#include "MPLibrary/GoalTracker.h"
#include "MPLibrary/MPSolution.h"
#include "MPLibrary/MPTools/MPTools.h"

#include "ConfigurationSpace/LocalObstacleMap.h"
#include "ConfigurationSpace/CompositeGraph.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupPath.h"
#include "ConfigurationSpace/Path.h"
#include "ConfigurationSpace/Weight.h"

//distance metric includes
#include "Testing/MPLibrary/DistanceMetrics/ManhattanDistanceTest.h"
#include "Testing/MPLibrary/DistanceMetrics/MinkowskiDistanceTest.h"
#include "Testing/MPLibrary/DistanceMetrics/EuclideanDistanceTest.h"
#include "Testing/MPLibrary/DistanceMetrics/WorkspaceTranslationDistanceTest.h"
#include "Testing/MPLibrary/DistanceMetrics/RMSDDistanceTest.h"
#include "Testing/MPLibrary/DistanceMetrics/WeightedEuclideanDistanceTest.h"
#include "Testing/MPLibrary/DistanceMetrics/ScaledEuclideanDistanceTest.h"
#include "Testing/MPLibrary/DistanceMetrics/LPSweptDistanceTest.h"
#include "Testing/MPLibrary/DistanceMetrics/BinaryLPSweptDistanceTest.h"

//validity checker includes
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "Testing/MPLibrary/ValidityCheckers/AlwaysTrueValidityTest.h"
#include "Testing/MPLibrary/ValidityCheckers/CollisionDetection/BoundingSpheresCollisionDetectionTest.h"
#include "Testing/MPLibrary/ValidityCheckers/CollisionDetection/InsideSpheresCollisionDetectionTest.h"
#include "Testing/MPLibrary/ValidityCheckers/TerrainValidityCheckerTest.h"

//neighborhood finder includes
#include "Testing/MPLibrary/NeighborhoodFinders/BruteForceNFTest.h"

//sampler includes
#include "Testing/MPLibrary/Samplers/BridgeTestSamplerTest.h"
#include "Testing/MPLibrary/Samplers/MixSamplerTest.h"
#include "Testing/MPLibrary/Samplers/ObstacleBasedSamplerTest.h"
#include "Testing/MPLibrary/Samplers/UniformRandomSamplerTest.h"

//local planner includes
#include "Testing/MPLibrary/LocalPlanners/StraightLineTest.h"

//extenders includes
#include "Testing/MPLibrary/Extenders/BasicExtenderTest.h"

//path smoothing includes

//connector includes
#include "Testing/MPLibrary/Connectors/CCsConnectorTest.h"
#include "Testing/MPLibrary/Connectors/NeighborhoodConnectorTest.h"
#include "Testing/MPLibrary/Connectors/RewireConnectorTest.h"

//metric includes
#include "Testing/MPLibrary/Metrics/NumNodesMetricTest.h"
#include "Testing/MPLibrary/Metrics/NumEdgesMetricTest.h"
#include "Testing/MPLibrary/Metrics/TimeMetricTest.h"

//map evaluator includes
#include "Testing/MPLibrary/MapEvaluators/LazyQueryTest.h"
#include "Testing/MPLibrary/MapEvaluators/QueryMethodTest.h"

//mp strategies includes
#include "MPLibrary/MPStrategies/ValidationStrategy.h"

//geometry includes
#include "Testing/Geometry/Shapes/NBoxTest.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief Defines available methods in the Motion Planning Universe for Cfg
/// @tparam C Cfg type
/// @tparam W Weight type
///
///TODO::Update this description
/// MPTraits is a type class which defines the motion planning universe. We
/// construct our methods through a factory design pattern, and thus this states
/// all available classes within an abstraction that you can use in the system.
/// Essentially, the important types are the CfgType or the @cspace abstraction
/// class, the WeightType or the edge type of the graph, and a type list for
/// each algorithm abstraction --- here you only need to define what you need,
/// as extraneous methods in the type class imply longer compile times.
///
/// All methods should have "Test" at the end to specify that they are using the
/// test version, and the test version header file should be included at the
/// top of this file.
////////////////////////////////////////////////////////////////////////////////
template <typename C, typename W = DefaultWeight<C>>
struct MPTraits {

  typedef C                               CfgType;
  typedef W                               WeightType;
  typedef GenericStateGraph<C, W>              RoadmapType;
  typedef PathType<MPTraits>              Path;
  typedef MPLibraryTests<MPTraits>        MPLibrary;
  typedef MPSolutionType<MPTraits>        MPSolution;
  typedef MPToolsType<MPTraits>           MPTools;
  typedef LocalObstacleMapType<MPTraits>  LocalObstacleMap;
  typedef GoalTrackerType<MPTraits>       GoalTracker;

  typedef GroupCfg<RoadmapType>                          GroupCfgType;
  typedef GroupLocalPlan<RoadmapType>                    GroupWeightType;
  typedef GroupRoadmap<GroupCfgType, GroupWeightType>    GroupRoadmapType;
  typedef GroupPath<MPTraits>                            GroupPathType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
      ManhattanDistanceTest<MPTraits>,
      MinkowskiDistanceTest<MPTraits>,
      EuclideanDistanceTest<MPTraits>,
      WorkspaceTranslationDistanceTest<MPTraits>,
      RMSDDistanceTest<MPTraits>,
      WeightedEuclideanDistanceTest<MPTraits>,
      LPSweptDistanceTest<MPTraits>,
      BinaryLPSweptDistanceTest<MPTraits>,
      ScaledEuclideanDistanceTest<MPTraits>
      > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
      AlwaysTrueValidityTest<MPTraits>,
      TerrainValidityCheckerTest<MPTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
      BruteForceNFTest<MPTraits>
      > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
      BridgeTestSamplerTest<MPTraits>,
      MixSamplerTest<MPTraits>,
      ObstacleBasedSamplerTest<MPTraits>,
      UniformRandomSamplerTest<MPTraits>
      > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
    // StraightLine<MPTraits>
      > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
      BasicExtenderTest<MPTraits>
      > ExtenderMethodList;

  //types of path smoothing available in our world
  typedef boost::mpl::list<
      > PathModifierMethodList;

  //types of connectors available in our world
  typedef boost::mpl::list<
      CCsConnectorTest<MPTraits>,
      NeighborhoodConnectorTest<MPTraits>,
      RewireConnectorTest<MPTraits>
      > ConnectorMethodList;

  //types of metrics available in our world
  typedef boost::mpl::list<
      NumNodesMetricTest<MPTraits>,
      NumEdgesMetricTest<MPTraits>,
      TimeMetricTest<MPTraits>
      > MetricMethodList;


  //types of map evaluators available in our world
  typedef boost::mpl::list<
    LazyQueryTest<MPTraits>,
    QueryMethodTest<MPTraits>
      > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    //ValidationStrategy<MPTraits>
      > MPStrategyMethodList;

  //types of shapes available in our world
  typedef boost::mpl::list<
    NBox
      > ShapesList;
};

#endif
