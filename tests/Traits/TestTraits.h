#ifndef PPL_TEST_TRAITS_H_
#define PPL_TEST_TRAITS_H_

#include "MPLibrary/GoalTracker.h"      //src
#include "MPLibrary/MPSolution.h"       //src
#include "MPLibrary/MPTools/MPTools.h"  //src

#include "../MPLibrary/MPLibraryTests.h"

#include "ConfigurationSpace/LocalObstacleMap.h"    //src
#include "ConfigurationSpace/CompositeGraph.h"      //src
#include "ConfigurationSpace/GroupCfg.h"            //src
#include "ConfigurationSpace/GroupLocalPlan.h"      //src
#include "ConfigurationSpace/GroupPath.h"           //src
#include "ConfigurationSpace/Path.h"                //src
#include "ConfigurationSpace/Weight.h"              //src

//distance metric includes
#include "../MPLibrary/DistanceMetrics/ManhattanDistanceTest.h"
#include "../MPLibrary/DistanceMetrics/MinkowskiDistanceTest.h"
#include "../MPLibrary/DistanceMetrics/EuclideanDistanceTest.h"
#include "../MPLibrary/DistanceMetrics/WorkspaceTranslationDistanceTest.h"
#include "../MPLibrary/DistanceMetrics/RMSDDistanceTest.h"
#include "../MPLibrary/DistanceMetrics/WeightedEuclideanDistanceTest.h"
#include "../MPLibrary/DistanceMetrics/ScaledEuclideanDistanceTest.h"
#include "../MPLibrary/DistanceMetrics/LPSweptDistanceTest.h"
#include "../MPLibrary/DistanceMetrics/BinaryLPSweptDistanceTest.h"

//validity checker includes
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"  //src
#include "../MPLibrary/ValidityCheckers/AlwaysTrueValidityTest.h"
#include "../MPLibrary/ValidityCheckers/CollisionDetection/BoundingSpheresCollisionDetectionTest.h"
#include "../MPLibrary/ValidityCheckers/CollisionDetection/InsideSpheresCollisionDetectionTest.h"
#include "../MPLibrary/ValidityCheckers/TerrainValidityCheckerTest.h"

//neighborhood finder includes
#include "../MPLibrary/NeighborhoodFinders/BruteForceNFTest.h"

//sampler includes
#include "../MPLibrary/Samplers/BridgeTestSamplerTest.h"
#include "../MPLibrary/Samplers/MixSamplerTest.h"
#include "../MPLibrary/Samplers/ObstacleBasedSamplerTest.h"
#include "../MPLibrary/Samplers/UniformRandomSamplerTest.h"
#include "../MPLibrary/Samplers/UniformObstacleBasedSamplerTest.h"

//local planner includes
#include "../MPLibrary/LocalPlanners/StraightLineTest.h"

//extenders includes
#include "../MPLibrary/Extenders/BasicExtenderTest.h"

//path smoothing includes

//connector includes
#include "../MPLibrary/Connectors/CCsConnectorTest.h"
#include "../MPLibrary/Connectors/NeighborhoodConnectorTest.h"
#include "../MPLibrary/Connectors/RewireConnectorTest.h"

//metric includes
#include "../MPLibrary/Metrics/NumNodesMetricTest.h"
#include "../MPLibrary/Metrics/NumEdgesMetricTest.h"
#include "../MPLibrary/Metrics/TimeMetricTest.h"

//map evaluator includes
#include "../MPLibrary/MapEvaluators/LazyQueryTest.h"
#include "../MPLibrary/MapEvaluators/QueryMethodTest.h"

//mp strategies includes
#include "MPLibrary/MPStrategies/ValidationStrategy.h"  //src

//geometry includes
// #include "../Geometry/Shapes/NBoxTest.h"

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
      UniformRandomSamplerTest<MPTraits>,
      UniformObstacleBasedSamplerTest<MPTraits>
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
  typedef boost::mpl::list<NBox> ShapesList;
};

#endif
