cmake_minimum_required(VERSION 3.15)

# Connectors
option(COMPILE_CCS_CONNECTOR "Compile CCsConnector" ON)
option(COMPILE_NEIGHBORHOOD_CONNECTOR "Compile NeighborhoodConnector" ON)
option(COMPILE_REWIRE_CONNECTOR "Compile RewireConnector" ON)

# DistanceMetrics
option(COMPILE_BINARY_LP_SWEPT_DISTANCE "Compile BinaryLPSweptDistance" ON)
option(COMPILE_LP_SWEPT_DISTANCE "Compile LPSweptDistance" ON)
option(COMPILE_MANHATTAN_DISTANCE "Compile ManhattanDistance" ON)
option(COMPILE_MINKOWSKI_DISTANCE "Compile MinkowskiDistance" ON)
option(COMPILE_EUCLIDEAN_DISTANCE "Compile EuclideanDistance" ON)
option(COMPILED_SCALED_EUCLIDEAN_DISTANCE "Compile ScaledEuclideanDistance" ON)
option(COMPILE_RMSD_DISTANCE "Compile RMSDDistance" ON)
option(COMPILE_TOPOLOGICAL_DISTANCE "Compile TopologicalDistance" ON)
option(COMPILE_WEIGHTED_EUCLIDEAN_DISTANCE "Compile WeightedEuclideanDistance" ON)
option(COMPILE_WORKSPACE_TRANSLATION_DISTANCE "Compile WorkspaceTranslationDistance" ON)

# EdgeValidityCheckers
option(COMPILE_INTERMEDIATES_EVC "Compile IntermediatesEdgeValidityChecker" ON)

# Extenders
option(COMPILE_BASIC_EXTENDER "Compile BasicExtender" ON)
option(COMPILE_ROTATE_TRANSLATE_EXTENDER "Compile RotateThenTranslateExtender" ON)

# LocalPlanners
option(COMPILE_STRAIGHT_LINE_LP "Compile StraightLineLP" ON)
option(COMPILE_HIERARCHICAL_LP "Compile HierarchicalLP" ON)

# MapEvaluators
option(COMPILE_CBS_QUERY "Compile CBSQuery" ON)
option(COMPILE_CLEARANCE_QUERY "Compile ClearanceQuery" ON)
option(COMPILE_COLLISION_EVALUATOR "Compile CollisionEvaluator" ON)
option(COMPILE_COMPOSE_EVALUATOR "Compile ComposeEvaluator" ON)
option(COMPILE_CONDITIONAL_EVALUATOR "Compile ConditionalEvaluator" ON)
option(COMPILE_GROUP_QUERY "Compile GroupQuery" ON)
option(COMPILE_LAZY_QUERY "Compile LazyQuery" ON)
option(COMPILE_MINIMUM_DISTANCE_EVALUATOR "Compile MinimumDistanceEvaluator" ON)
option(COMPILE_NEGATE_EVALUATOR "Compile NegateEvaluator" ON)
option(COMPILE_PATH_EVALUATOR "Compile PathEvaluator" ON)
option(COMPILE_PRINT_MAP_EVALUATION "Compile PrintMapEvaluation" ON)
option(COMPILE_QUERY_METHOD "Compile QueryMethod" ON)
option(COMPILE_SIPP_METHOD "Compile SIPPMethod" ON)
option(COMPILE_TIME_EVALUATOR "Compile TimeEvaluator" ON)

# Metrics
option(COMPILE_NUM_NODES_METRIC "Compile NumNodesMetric" ON)
option(COMPILE_NUM_EDGES_METRIC "Compile NumEdgesMetric" ON)
option(COMPILE_TIME_METRIC "Compile TimeMetric" ON)

# MPStrategies
option(COMPILE_ADAPTIVE_RRT "Compile AdaptiveRRT" ON)
option(COMPILE_BASIC_PRM "Compile BasicPRM" ON)
option(COMPILE_BASIC_RRT "Compile BasicRRTStrategy" ON)
option(COMPILE_DYANMIC_DOMAIN_RRT "Compile DynamicDomainRRT" ON)
option(COMPILE_DYNAMIC_REGION_RRT "Compile DynamicRegionRRT" ON)
option(COMPILE_DYNAMIC_REGION_PRM "Compile DynamicRegionsPRM" ON)
option(COMPILE_EET "Compile EET" ON)
option(COMPILE_GROUP_DECOUPLED_STRATEGY "Compile GroupDecoupledStrategy" ON)
option(COMPILE_GROUP_PRM "Compile GroupPRM" ON)
option(COMPILE_MODIFY_PATH "Compile ModifyPath" ON)
option(COMPILE_PATH_STRATEGY "Compile PathStrategy" ON)
option(COMPILE_SYCLOP "Compile Syclop" ON)
option(COMPILE_TOGGLE_PRM " Compile TogglePRM " ON)
option(COMPILE_WRENCH_ACCESS_STRAT " Compile WrenchAccessibilityStrategy " ON)

# MPTools
option(COMPILE_CLEARANCE_UTILS " Compile ClearanceUtilities " ON)
option(COMPILE_MED_AXIS_UTILS " Compile MedialAxisUtilities " ON)
option(COMPILE_POINT_CONSTRUCTION " Compile PointConstruction " ON)
option(COMPILE_REACHABILITY_UTIL " Compile ReachabilityUtil " ON)
option(COMPILE_SAFE_INTERVAL_TOOL " Compile SafeIntervalTool " ON)
option(COMPILE_SKELETON_CLEARANCE " Compile SkeletonClearanceUtility " ON)
option(COMPILE_TOPOLOGICAL_MAP " Compile TopologicalMap " ON)
option(COMPILE_TET_GEN_DECOMP " Compile TetGenDecomposition " ON)
option(COMPILE_REEB_GRAPH " Compile ReebGraphConstruction " ON)
option(COMPILE_MEAN_CURVATURE_SKEL " Compile MeanCurvatureSkeleton3D " ON)
option(COMPILE_WRENCH_ACCESS_TOOL " Compile WrenchAccessibilityTool " ON)

# NeighborhoodFinders
option(COMPILE_BRUTE_FORCE_NF " Compile BruteForceNF " ON)
option(COMPILE_KDTREE_NF " Compile KdTreeNF " ON)
option(COMPILE_RADIUS_NF " Compile RadiusNF " ON)
option(COMPILE_RANDOM_NF " Compile RandomNF " ON)

# PathModifiers
option(COMPILE_SHORTCUTTING " Compile ShortcuttingPathModifier " ON)

# Samplers
option(COMPILE_BRIDGE_TEST_SAMPLER " Compile BridgeTestSampler " ON)
option(COMPILE_GAUSSIAN_SAMPLER " Compile GaussianSampler " ON)
option(COMPILE_MIX_SAMPLER " Compile MixSampler " ON)
option(COMPILE_OBSTACLE_BASED_SAMPLER " Compile ObstacleBasedSampler " ON)
option(COMPILE_UNIFORM_RANDOM_SAMPLER " Compile UniformRandomSampler " ON)
option(COMPILE_UNIFORM_OBSTACLE_SAMPLER " Compile UniformObstacleBasedSampler " ON)

# ValidityCheckers
option(COMPILE_ALWAYS_TRUE_VC " Compile AlwaysTrueValidity " ON)
option(COMPILE_COMPOSE_CD " Compile ComposeCollision " ON)
option(COMPILE_COMPOSE_VC " Compile ComposeValidity " ON)
option(COMPILE_NODE_CLEARANCE_VC " Compile NodeClearanceValidity " ON)
option(COMPILE_OBSTACLE_CLEARANCE_VC " Compile ObstacleClearanceValidity " ON)
option(COMPILE_TERRAIN_VC " Compile TerrainValidityChecker " ON)
option(COMPILE_TOPOLOGICAL_MAP_VC " Compile TopologicalMapValidity " ON)

set(COMM_SRCS
    PARENT_SCOPE
)

set(PLANNER_SRCS
    PARENT_SCOPE
)

set(CLIENT_SRCS
    PARENT_SCOPE
)

set(XML_SRCS
    $(CMAKE_CURRENT_SOURCE_DIR)/xml_main.cpp
    PARENT_SCOPE
)

set(MASTER_SRCS
    $(CMAKE_CURRENT_SOURCE_DIR)/master_node_main.cpp
    PARENT_SCOPE
)

set(PMPL_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/ConfigurationSpace/Cfg.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ConfigurationSpace/GroupPath.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ConfigurationSpace/LocalObstacleMap.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ConfigurationSpace/Path.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ConfigurationSpace/ReachableVolumes.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/GMSPolygon.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/GMSPolyhedron.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Bodies/Body.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Bodies/Chain.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Bodies/Connection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Bodies/DHParameters.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Bodies/MultiBody.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Boundaries/AbstractBoundingBox.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Boundaries/AbstractBoundingSphere.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Boundaries/AbstractBoundingSphericalShell.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Boundaries/Boundary.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Boundaries/BoundaryIntersection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Boundaries/CSpaceBoundingBox.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Boundaries/CSpaceBoundingSphere.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Boundaries/TetrahedralBoundary.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Boundaries/WorkspaceBoundingBox.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Boundaries/WorkspaceBoundingSphere.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Boundaries/WorkspaceBoundingSphericalShell.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Shapes/NBox.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Shapes/NSphere.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry/Shapes/NSphericalShell.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPBaseObject.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPSolution.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Connectors/ConnectorMethod.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/DistanceMetrics/DistanceMetricMethod.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/EdgeValidityCheckers/EdgeValidityCheckerMethod.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Extenders/ExtenderMethod.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/LocalPlanners/LocalPlannerMethod.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/LocalPlanners/LPOutput.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/LocalPlanners/GroupLPOutput.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/MapEvaluatorMethod.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Metrics/MetricMethod.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/MPStrategyMethod.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/GroupStrategyMethod.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/BasicRRTStrategy.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/MPTools.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/NeighborhoodFinders/NeighborhoodFinderMethod.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/PathModifiers/PathModifierMethod.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Samplers/SamplerMethod.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/CollisionDetectionValidity.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/CollisionDetection/CollisionDetectionMethod.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/CollisionDetection/RapidCollisionDetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/CollisionDetection/PQPCollisionDetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/CollisionDetection/SpheresCollisionDetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/ValidityCheckerMethod.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/GoalTracker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPLibrary.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/MPProblem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/MPTask.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/GroupTask.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/InteractionInformation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Constraints/Constraint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Constraints/BoundaryConstraint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Constraints/CSpaceConstraint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/DynamicObstacle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Environment/Environment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/Actuator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/Control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/Robot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/RobotGroup/RobotGroup.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/TaskHierarchy/Decomposition.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/TaskHierarchy/SemanticTask.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/TaskHierarchy/SubtaskFlow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Utilities/ClockClass.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Utilities/Conversions.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Utilities/CountingStreamBuffer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Utilities/IOUtils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Utilities/MedialAxis2D.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Utilities/MetricUtils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Utilities/MPUtils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Utilities/SSSP.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Utilities/XMLNode.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Workspace/PropertyMap.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Workspace/GridOverlay.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Workspace/WorkspaceDecomposition.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Workspace/WorkspacePortal.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Workspace/WorkspaceRegion.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Workspace/WorkspaceSkeleton.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/PMPL.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ObstacleSubset/obst_subset.cpp
)

# Connectors
if(COMPILE_CCS_CONNECTOR)
    set(CCS_CONNECTOR_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Connectors/CCsConnector.cpp)
else()
    set(CCS_CONNECTOR_AVAILABLE 0)
endif()

if(COMPILE_NEIGHBORHOOD_CONNECTOR)
    set(NEIGHBOR_CONNECTOR_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Connectors/NeighborhoodConnector.cpp)
else()
    set(NEIGHBOR_CONNECTOR_AVAILABLE 0)
endif()

if(COMPILE_REWIRE_CONNECTOR)
    set(REWIRE_CONNECTOR_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Connectors/RewireConnector.cpp)
else()
    set(REWIRE_CONNECTOR_AVAILABLE 0)
endif()

# DistanceMetrics
if(COMPILE_BINARY_LP_SWEPT_DISTANCE)
    set(BIN_LP_SWEPT_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/DistanceMetrics/BinaryLPSweptDistance.cpp)
else()
    set(BIN_LP_SWEPT_AVAILABLE 0)
endif()

if(COMPILE_LP_SWEPT_DISTANCE)
    set(LP_SWEPT_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/DistanceMetrics/LPSweptDistance.cpp)
else()
    set(LP_SWEPT_AVAILABLE 0)
endif()

if(COMPILE_EUCLIDEAN_DISTANCE)
    set(EUCLIDEAN_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/DistanceMetrics/EuclideanDistance.cpp)
else()
    set(EUCLIDEAN_AVAILABLE 0)
endif()

if(COMPILE_SCALED_EUCLIDEAN_DISTANCE)
    set(SCALED_EUCLIDEAN_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/DistanceMetrics/ScaledEuclideanDistance.cpp)
else()
    set(SCALED_EUCLIDEAN_AVAILABLE 0)
endif()

if(COMPILE_MANHATTAN_DISTANCE)
    set(MANHATTAN_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/DistanceMetrics/ManhattanDistance.cpp)
else()
    set(MANHATTAN_AVAILABLE 0)
endif()

if(COMPILE_MINKOWSKI_DISTANCE)
    set(MINKOWSKI_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/DistanceMetrics/MinkowskiDistance.cpp)
else()
    set(MINKOWSKI_AVAILABLE 0)
endif()

if(COMPILE_RMSD_DISTANCE)
    set(RMSD_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/DistanceMetrics/RMSDDistance.cpp)
else()
    set(RMSD_AVAILABLE 0)
endif()

if(COMPILE_TOPOLOGICAL_DISTANCE)
    set(TOPOLOGICAL_DISTANCE_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/DistanceMetrics/TopologicalDistance.cpp)
else()
    set(TOPOLOGICAL_DISTANCE_AVAILABLE 0)
endif()

if(COMPILE_WEIGHTED_EUCLIDEAN_DISTANCE)
    set(WEIGHTED_EUCLIDEAN_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/DistanceMetrics/WeightedEuclideanDistance.cpp)
else()
    set(WEIGHTED_EUCLIDEAN_AVAILABLE 0)
endif()

if(COMPILE_WORKSPACE_TRANSLATION_DISTANCE)
    set(WORKSPACE_TRANSLATION_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/DistanceMetrics/WorkspaceTranslationDistance.cpp)
else()
    set(WORKSPACE_TRANSLATION_AVAILABLE 0)
endif()

# EdgeValidityCheckers
if(COMPILE_INTERMEDIATES_EVC)
    set(INTERMEDIATES_EVC_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/EdgeValidityCheckers/IntermediatesEdgeValidityChecker.cpp)
else()
    set(INTERMEDIATES_EVC_AVAILABLE 0)
endif()

# Extenders
if(COMPILE_BASIC_EXTENDER)
    set(BASIC_EXTENDER_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Extenders/BasicExtender.cpp)
else()
    set(BASIC_EXTENDER_AVAILABLE 0)
endif()

if(COMPILE_ROTATE_TRANSLATE_EXTENDER)
    set(ROTATION_THEN_TRANSLATION_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Extenders/RotationThenTranslation.cpp)
else()
    set(ROTATION_THEN_TRANSLATION_AVAILABLE 0)
endif()

# LocalPlanners
if(COMPILE_STRAIGHT_LINE_LP)
    set(STRAIGHT_LINE_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/LocalPlanners/StraightLine.cpp)
else()
    set(STRAIGHT_LINE_AVAILABLE 0)
endif()

if(COMPILE_HIERARCHICAL_LP)
    set(HIERARCHICAL_LP_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/LocalPlanners/HierarchicalLP.cpp)
else()
    set(HIERARCHICAL_LP_AVAILABLE 0)
endif()

# MapEvaluators
if(COMPILE_CBS_QUERY)
    set(CBS_QUERY_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/CBSQuery.cpp)
else()
    set(CBS_QUERY_AVAILABLE 0)
endif()

if(COMPILE_CLEARANCE_QUERY)
    set(CLEARANCE_QUERY_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/ClearanceQuery.cpp)
else()
    set(CLEARANCE_QUERY_AVAILABLE 0)
endif()

if(COMPILE_COLLISION_EVALUATOR)
    set(COLLISION_EVAL_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/CollisionEvaluator.cpp)
else()
    set(COLLISION_EVAL_AVAILABLE 0)
endif()

if(COMPILE_COMPOSE_EVALUATOR)
    set(COMPOSE_EVAL_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/ComposeEvaluator.cpp)
else()
    set(COMPOSE_EVAL_AVAILABLE 0)
endif()

if(COMPILE_CONDITIONAL_EVALUATOR)
    set(CONDITIONAL_EVAL_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/ConditionalEvaluator.cpp)
else()
    set(CONDITIONAL_EVAL_AVAILABLE 0)
endif()

if(COMPILE_GROUP_QUERY)
    set(GROUP_QUERY_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/GroupQuery.cpp)
else()
    set(GROUP_QUERY_AVAILABLE 0)
endif()

if(COMPILE_LAZY_QUERY)
    set(LAZY_QUERY_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/LazyQuery.cpp)
else()
    set(LAZY_QUERY_AVAILABLE 0)
endif()

if(COMPILE_MINIMUM_DISTANCE_EVALUATOR)
    set(MIN_DIST_EVAL_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/MinimumDistanceEvaluator.cpp)
else()
    set(MIN_DIST_EVAL_AVAILABLE 0)
endif()

if(COMPILE_NEGATE_EVALUATOR)
    set(NEGATE_EVAL_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/NegateEvaluator.cpp)
else()
    set(NEGATE_EVAL_AVAILABLE 0)
endif()

if(COMPILE_PATH_EVALUATOR)
    set(PATH_EVAL_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/PathEvaluator.cpp)
else()
    set(PATH_EVAL_AVAILABLE 0)
endif()

if(COMPILE_PRINT_MAP_EVALUATION)
    set(PRINT_MAP_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/PrintMapEvaluation.cpp)
else()
    set(PRINT_MAP_AVAILABLE 0)
endif()

if(COMPILE_QUERY_METHOD)
    set(QUERY_METHOD_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/QueryMethod.cpp)
else()
    set(QUERY_METHOD_AVAILABLE 0)
endif()

if(COMPILE_SIPP_METHOD)
    set(SIPP_METHOD_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/SIPPMethod.cpp)
else()
    set(SIPP_METHOD_AVAILABLE 0)
endif()

if(COMPILE_TIME_EVALUATOR)
    set(TIME_EVAL_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/TimeEvaluator.cpp)
else()
    set(TIME_EVAL_AVAILABLE 0)
endif()

# Metrics
if(COMPILE_NUM_NODES_METRIC)
    set(NUM_NODES_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Metrics/NumNodesMetric.cpp)
else()
    set(NUM_NODES_AVAILABLE 0)
endif()

if(COMPILE_NUM_EDGES_METRIC)
    set(NUM_EDGES_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Metrics/NumEdgesMetric.cpp)
else()
    set(NUM_EDGES_AVAILABLE 0)
endif()

if(COMPILE_TIME_METRIC)
    set(TIME_METRIC_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Metrics/TimeMetric.cpp)
else()
    set(TIME_METRIC_AVAILABLE 0)
endif()

# MPStrategies
if(COMPILE_ADAPTIVE_RRT)
    set(ADAPTIVE_RRT_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/AdaptiveRRT.cpp)
else()
    set(ADAPTIVE_RRT_AVAILABLE 0)
endif()

if(COMPILE_BASIC_PRM)
    set(BASIC_PRM_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/BasicPRM.cpp)
else()
    set(BASIC_PRM_AVAILABLE 0)
endif()

if(COMPILE_BASIC_RRT)
    set(BASIC_RRT_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/BasicRRTStrategy.cpp)
else()
    set(BASIC_RRT_AVAILABLE 0)
endif()

if(COMPILE_DYANMIC_DOMAIN_RRT)
    set(DYNAMIC_DOMAIN_RRT_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/DynamicDomainRRT.cpp)
else()
    set(DYNAMIC_DOMAIN_RRT_AVAILABLE 0)
endif()

if(COMPILE_DYNAMIC_REGION_RRT)
    set(DYNAMIC_REGION_RRT_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/DynamicRegionRRT.cpp)
else()
    set(DYNAMIC_REGION_RRT_AVAILABLE 0)
endif()

if(COMPILE_DYNAMIC_REGION_PRM)
    set(DYNAMIC_REGION_PRM_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/DynamicRegionsPRM.cpp)
else()
    set(DYNAMIC_REGION_PRM_AVAILABLE 0)
endif()

if(COMPILE_EET)
    set(EET_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/EET.cpp)
else()
    set(EET_AVAILABLE 0)
endif()

if(COMPILE_GROUP_DECOUPLED_STRATEGY)
    set(GROUP_DECOUPLED_STRAT_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/GroupDecoupledStrategy.cpp)
else()
    set(GROUP_DECOUPLED_STRAT_AVAILABLE 0)
endif()

if(COMPILE_GROUP_PRM)
    set(GROUP_PRM_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/GroupPRM.cpp)
else()
    set(GROUP_PRM_AVAILABLE 0)
endif()

if(COMPILE_MODIFY_PATH)
    set(MODIFY_PATH_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/ModifyPath.cpp)
else()
    set(MODIFY_PATH_AVAILABLE 0)
endif()

if(COMPILE_PATH_STRATEGY)
    set(PATH_STRAT_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/PathStrategy.cpp)
else()
    set(PATH_STRAT_AVAILABLE 0)
endif()

if(COMPILE_SYCLOP)
    set(SYCLOP_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/Syclop.cpp)
else()
    set(SYCLOP_AVAILABLE 0)
endif()

if(COMPILE_TOGGLE_PRM)
    set(TOGGLE_PRM_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/TogglePRMStrategy.cpp)
else()
    set(TOGGLE_PRM_AVAILABLE 0)
endif()

if(COMPILE_WRENCH_ACCESS_STRAT)
    set(WRENCH_ACCESS_STRAT_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/WrenchAccessibilityStrategy.cpp)
else()
    set(WRENCH_ACCESS_STRAT_AVAILABLE 0)
endif()

# MPTools
if(COMPILE_CLEARANCE_UTILS)
    set(CLEARANCE_UTILS_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/ClearanceUtilities.cpp)
else()
    set(CLEARANCE_UTILS_AVAILABLE 0)
endif()

if(COMPILE_MED_AXIS_UTILS)
    set(MED_AXIS_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/MedialAxisUtilities.cpp)
else()
    set(MED_AXIS_AVAILABLE 0)
endif()

if(COMPILE_POINT_CONSTRUCTION)
    set(POINT_CONSTRUCTION_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/PointConstruction.cpp)
else()
    set(POINT_CONSTRUCTION_AVAILABLE 0)
endif()

if(COMPILE_REACHABILITY_UTIL)
    set(REACHABILITY_UTIL_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/ReachabilityUtil.cpp)
else()
    set(REACHABILITY_UTIL_AVAILABLE 0)
endif()

if(COMPILE_SAFE_INTERVAL_TOOL)
    set(SAFE_INTERVAL_TOOL_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/SafeIntervalTool.cpp)
else()
    set(SAFE_INTERVAL_TOOL_AVAILABLE 0)
endif()

if(COMPILE_SKELETON_CLEARANCE)
    set(SKELETON_CLEARANCE_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/SkeletonClearanceUtility.cpp)
else()
    set(SKELETON_CLEARANCE_AVAILABLE 0)
endif()

if(COMPILE_TOPOLOGICAL_MAP)
    set(TOPOLOGICAL_MAP_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/TopologicalMap.cpp)
else()
    set(TOPOLOGICAL_MAP_AVAILABLE 0)
endif()

if(COMPILE_TET_GEN_DECOMP)
    set(TET_GEN_DECOMP_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/TetGenDecomposition.cpp)
else()
    set(TET_GEN_DECOMP_AVAILABLE 0)
endif()

if(COMPILE_REEB_GRAPH)
    set(REEB_GRAPH_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/ReebGraphConstruction.cpp)
else()
    set(REEB_GRAPH_AVAILABLE 0)
endif()

if(COMPILE_MEAN_CURVATURE_SKEL)
    set(MEAN_CURVE_SKEL_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/MeanCurvatureSkeleton3D.cpp)
else()
    set(MEAN_CURVE_SKEL_AVAILABLE 0)
endif()

if(COMPILE_WRENCH_ACCESS_TOOL)
    set(WRENCH_ACCESS_TOOL_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/WrenchAccessibilityTool.cpp)
else()
    set(WRENCH_ACCESS_TOOL_AVAILABLE 0)
endif()

# NeighborhoodFinders
if(COMPILE_BRUTE_FORCE_NF)
    set(BRUTE_FORCE_NF_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/NeighborhoodFinders/BruteForceNF.cpp)
else()
    set(BRUTE_FORCE_NF_AVAILABLE 0)
endif()

if(COMPILE_KDTREE_NF)
    set(KDTREE_NF_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/NeighborhoodFinders/KdTreeNF.cpp)
else()
    set(KDTREE_NF_AVAILABLE 0)
endif()

if(COMPILE_RADIUS_NF)
    set(RADIUS_NF_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/NeighborhoodFinders/RadiusNF.cpp)
else()
    set(RADIUS_NF_AVAILABLE 0)
endif()

if(COMPILE_RANDOM_NF)
    set(RANDOM_NF_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/NeighborhoodFinders/RandomNF.cpp)
else()
    set(RANDOM_NF_AVAILABLE 0)
endif()

# PathModifiers
if(COMPILE_SHORTCUTTING)
    set(SHORTCUTTING_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/PathModifiers/ShortcuttingPathModifier.cpp)
else()
    set(SHORTCUTTING_AVAILABLE 0)
endif()

# Samplers
if(COMPILE_BRIDGE_TEST_SAMPLER)
    set(BRIDGE_TEST_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Samplers/BridgeTestSampler.cpp)
else()
    set(BRIDGE_TEST_AVAILABLE 0)
endif()

if(COMPILE_GAUSSIAN_SAMPLER)
    set(GAUSSIAN_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Samplers/GaussianSampler.cpp)
else()
    set(GAUSSIAN_AVAILABLE 0)
endif()

if(COMPILE_MIX_SAMPLER)
    set(MIX_SAMPLER_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Samplers/MixSampler.cpp)
else()
    set(MIX_SAMPLER_AVAILABLE 0)
endif()

if(COMPILE_OBSTACLE_BASED_SAMPLER)
    set(OBSTACLE_BASED_SAMPLER_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Samplers/ObstacleBasedSampler.cpp)
else()
    set(OBSTACLE_BASED_SAMPLER_AVAILABLE 0)
endif()

if(COMPILE_UNIFORM_RANDOM_SAMPLER)
    set(UNIFORM_RANDOM_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Samplers/UniformRandomSampler.cpp)
else()
    set(UNIFORM_RANDOM_AVAILABLE 0)
endif()

if(COMPILE_UNIFORM_OBSTACLE_SAMPLER)
    set(UNIFORM_RANDOM_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Samplers/UniformObstacleBasedSampler.cpp)
else()
    set(UNIFORM_OBSTACLE_AVAILABLE 0)
endif()

# ValidityCheckers
if(COMPILE_ALWAYS_TRUE_VC)
    set(ALWAYS_TRUE_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/AlwaysTrueValidity.cpp)
else()
    set(ALWAYS_TRUE_AVAILABLE 0)
endif()

if(COMPILE_COMPOSE_CD)
    set(COMPOSE_CD_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/ComposeCollision.cpp)
else()
    set(COMPOSE_CD_AVAILABLE 0)
endif()

if(COMPILE_COMPOSE_VC)
    set(COMPOSE_VC_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/ComposeValidity.cpp)
else()
    set(COMPOSE_VC_AVAILABLE 0)
endif()

if(COMPILE_NODE_CLEARANCE_VC)
    set(NODE_CLEARANCE_VC_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/NodeClearanceValidity.cpp)
else()
    set(NODE_CLEARANCE_VC_AVAILABLE 0)
endif()

if(COMPILE_OBSTACLE_CLEARANCE_VC)
    set(OBSTACLE_CLEARANCE_VC_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/ObstacleClearanceValidity.cpp)
else()
    set(OBSTACLE_CLEARANCE_VC_AVAILABLE 0)
endif()

if(COMPILE_TERRAIN_VC)
    set(TERRAIN_VC_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/TerrainValidityChecker.cpp)
else()
    set(TERRAIN_VC_AVAILABLE 0)
endif()

if(COMPILE_TOPOLOGICAL_MAP_VC)
    set(TOPOLOGICAL_MAP_VC_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/TopologicalMapValidity.cpp)
else()
    set(TOPOLOGICAL_MAP_VC_AVAILABLE 0)
endif()

set(PMPL_SRCS ${PMPL_SRCS} PARENT_SCOPE)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Traits/Methods.h.in ${CMAKE_CURRENT_SOURCE_DIR}/Traits/Methods.h)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/MPTools.h.in ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/MPTools.h)