#ifndef RV_TRAITS_H_
#define RV_TRAITS_H_

#include "Cfg/CfgReachableVolume.h"
#include "Edges/Weight.h"

//distance metric includes
#include "PlanningLibrary/DistanceMetrics/ScaledEuclideanDistance.h"
#include "PlanningLibrary/DistanceMetrics/RVDistance.h"

//validity checker includes
#include "PlanningLibrary/ValidityCheckers/AlwaysTrueValidity.h"
#include "PlanningLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "PlanningLibrary/ValidityCheckers/ComposeValidity.h"
#include "PlanningLibrary/ValidityCheckers/MedialAxisClearanceValidity.h"
#include "PlanningLibrary/ValidityCheckers/NegateValidity.h"
#include "PlanningLibrary/ValidityCheckers/NodeClearanceValidity.h"
#include "PlanningLibrary/ValidityCheckers/ObstacleClearanceValidity.h"
#include "PlanningLibrary/ValidityCheckers/SurfaceValidity.h"
#include "PlanningLibrary/ValidityCheckers/SSSurfaceValidity.h"

//neighborhood finder includes
#include "PlanningLibrary/NeighborhoodFinders/BruteForceNF.h"

//sampler includes
#include "PlanningLibrary/Samplers/UniformRandomSampler.h"

//local planner includes
#include "PlanningLibrary/LocalPlanners/RVLocalPlanner.h"

//extenders includes
#include "PlanningLibrary/Extenders/BasicExtender.h"

//path smoothing includes

//connector includes
#include "PlanningLibrary/Connectors/RRTConnect.h"

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
#include "PlanningLibrary/MapEvaluators/NegateEvaluator.h"
#include "PlanningLibrary/MapEvaluators/PrintMapEvaluation.h"
#include "PlanningLibrary/MapEvaluators/TrueEvaluation.h"

//mp strategies includes
#include "PlanningLibrary/MPStrategies/BasicPRM.h"
#include "PlanningLibrary/MPStrategies/BasicRRTStrategy.h"
#include "PlanningLibrary/MPStrategies/RVRRT.h"

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
template<class C, class W = DefaultWeight<C> >
struct CfgReachableVolumeTraits {

  typedef CfgReachableVolume CfgType;
  typedef DefaultWeight<CfgType> WeightType;
  typedef C& CfgRef;

  typedef MPProblem<CfgReachableVolumeTraits> MPProblemType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
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
    BruteForceNF<CfgReachableVolumeTraits>
    > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
    UniformRandomSampler<CfgReachableVolumeTraits>
      > SamplerMethodList;

  //types of local planners available in our world
  //All of these samplers are applicable to problems without constraints
  //The RV Local planner is the only method that is applicable to problems with constraints
  typedef boost::mpl::list<
    RVLocalPlanner<CfgReachableVolumeTraits>
      > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
    BasicExtender<CfgReachableVolumeTraits>
      > ExtenderMethodList;

  //types of path smoothing available in our world
  //path smoothing can only be applied to problems without constraints
  typedef boost::mpl::list<
    > PathModifierMethodList;


  //types of connectors available in our world
  typedef boost::mpl::list<
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
    NegateEvaluator<CfgReachableVolumeTraits>,
    PrintMapEvaluation<CfgReachableVolumeTraits>,
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
