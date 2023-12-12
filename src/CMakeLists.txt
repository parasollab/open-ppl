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

# LocalPlanners

# MapEvaluators
option(COMPILE_CONDITIONAL_EVALUATOR "Compile ConditionalEvaluator" ON)

# Metrics
option(COMPILE_NUM_NODES_METRIC "Compile NumNodesMetric" ON)

# MPStrategies
option(COMPILE_BASIC_RRT "Compile BasicRRTStrategy" ON)

# MPTools
option(COMPILE_CLEARANCE_UTILS "Compile ClearanceUtilities" ON)
option(COMPILE_MED_AXIS_UTILS "Compile MedialAxisUtilities" ON)
option(COMPILE_POINT_CONSTRUCTION "Compile PointConstruction" ON)
option(COMPILE_REACHABILITY_UTIL "Compile ReachabilityUtil" ON)
option(COMPILE_SAFE_INTERVAL_TOOL "Compile SafeIntervalTool" ON)
option(COMPILE_SKELETON_CLEARANCE "Compile SkeletonClearanceUtility" ON)
option(COMPILE_TOPOLOGICAL_MAP "Compile TopologicalMap" ON)
option(COMPILE_TET_GEN_DECOMP "Compile TetGenDecomposition" ON)
option(COMPILE_REEB_GRAPH "Compile ReebGraphConstruction" ON)
option(COMPILE_MEAN_CURVATURE_SKEL "Compile MeanCurvatureSkeleton3D" ON)

# NeighborhoodFinders
option(COMPILE_BRUTE_FORCE_NF "Compile BruteForceNF" ON)

# PathModifiers

# Samplers
option(COMPILE_UNIFORM_RANDOM_SAMPLER "Compile UniformRandomSampler" ON)

# ValidityCheckers
option(COMPILE_ALWAYS_TRUE_VC "Compile AlwaysTrueValidity" ON)

set(HARDWARE_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/ActuatorInterface.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/Battery.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/Commands.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/HardwareInterface.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/RobotCommandQueue.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/SensorInterface.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/StateEstimator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/AverageEstimator.cpp
        PARENT_SCOPE
        )

if(BUILD_ICREATE)
    list(APPEND HARDWARE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/ArucoDetectorInterface.cpp)
    list(APPEND HARDWARE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/ArucoMarkerMap.cpp)
endif()

if(BUILD_ARUCO)
    list(APPEND HARDWARE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/ICreateInterface.cpp)
endif()

# empty srcs
set(MATLAB_SRCS
        PARENT_SCOPE
        )

if(BUILD_MATLAB)
    list(APPEND MATLAB_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Controllers/MatlabNeedleController.cpp)
    list(APPEND MATLAB_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Simulator/MatlabMicroSimulator.cpp)
endif()

set(COMM_SRCS
        PARENT_SCOPE
        )

set(PLANNER_SRCS
        PARENT_SCOPE
        )

set(CLIENT_SRCS
        PARENT_SCOPE
        )

set(SIM_SRCS
        $(CMAKE_CURRENT_SOURCE_DIR)/sim_main.cpp
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

set(TASK_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/RobotCommandQueue_sim.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/DrawableBody.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/DrawableBoundary.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/DrawableCfg.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/DrawableMultiBody.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/DrawablePath.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/DrawablePolyhedron.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/DrawableRoadmap.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/DrawableWorkspaceSkeleton.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Gui/EditBodyDialog.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Gui/EditCfgDialog.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Gui/EditConnectionDialog.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Gui/EditMultiBodyDialog.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Gui/EditWidgets.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Gui/Setup.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Gui/SliderTextWidget.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Gui/TestWidget.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Simulator/Simulation.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/PoIPlacementMethods/PoIPlacementMethod.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/Solution/Plan.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/Solution/TaskSolution.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/StateGraphs/StateGraph.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/TaskAllocators/TaskAllocatorMethod.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/TaskDecomposers/TaskDecomposerMethod.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/TaskEvaluators/SimpleMotionEvaluator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/TMPBaseObject.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/TMPStrategies/SimpleMotionMethod.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/TMPStrategies/TMPStrategyMethod.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/TMPTools/TMPTools.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Agents/AgentFactory_sim.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Agents/ChildAgent.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Agents/Coordinator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Agents/PathFollowingAgent.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Agents/PlanningAgent.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Agents/StepFunctions/DefaultCoordinatorStepFunction.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Agents/StepFunctions/StepFunction.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Agents/StepFunctions/StepFunctionFactory.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TMPLibrary/TMPLibrary.cpp
        PARENT_SCOPE
        )

set(SIM_SRCS_EXCL
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/RobotCommandQueue_sim.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Agents/AgentFactory_sim.cpp
        PARENT_SCOPE
        )

set(PMPL_SRCS_EXCL
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Agents/AgentFactory_pmpl.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/RobotCommandQueue_pmpl.cpp
        PARENT_SCOPE
        )

set(PMPL_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Agents/Agent.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Agents/AgentFactory_pmpl.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Controllers/CarlikeNeedleController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Controllers/ControllerMethod.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Controllers/ControlSetGenerators.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Controllers/SimpleController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Controllers/ICreateController.cpp
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
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/Robot/HardwareInterfaces/RobotCommandQueue_pmpl.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/RobotGroup/RobotGroup.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/TaskHierarchy/Decomposition.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/TaskHierarchy/SemanticTask.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPProblem/TaskHierarchy/SubtaskFlow.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Simulator/BulletEngine.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Simulator/BulletModel.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Simulator/Conversions.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Simulator/MicroSimulator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Utilities/ClockClass.cpp
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

# LocalPlanners

# MapEvaluators
if(COMPILE_CONDITIONAL_EVALUATOR)
    set(CONDITIONAL_EVAL_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/ConditionalEvaluator.cpp)
else()
    set(CONDITIONAL_EVAL_AVAILABLE 0)
endif()

# Metrics
if(COMPILE_NUM_NODES_METRIC)
    set(NUM_NODES_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Metrics/NumNodesMetric.cpp)
else()
    set(NUM_NODES_AVAILABLE 0)
endif()

# MPStrategies
if(COMPILE_BASIC_RRT)
    set(BASIC_RRT_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPStrategies/BasicRRTStrategy.cpp)
else()
    set(BASIC_RRT_AVAILABLE 0)
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

# NeighborhoodFinders
if(COMPILE_BRUTE_FORCE_NF)
    set(BRUTE_FORCE_NF_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/NeighborhoodFinders/BruteForceNF.cpp)
else()
    set(BRUTE_FORCE_NF_AVAILABLE 0)
endif()

# PathModifiers

# Samplers
if(COMPILE_UNIFORM_RANDOM_SAMPLER)
    set(UNIFORM_RANDOM_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/Samplers/UniformRandomSampler.cpp)
else()
    set(UNIFORM_RANDOM_AVAILABLE 0)
endif()

# ValidityCheckers
if(COMPILE_ALWAYS_TRUE_VC)
    set(ALWAYS_TRUE_AVAILABLE 1)
    list(APPEND PMPL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/AlwaysTrueValidity.cpp)
else()
    set(ALWAYS_TRUE_AVAILABLE 0)
endif()

set(PMPL_SRCS ${PMPL_SRCS} PARENT_SCOPE)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Traits/Methods.h.in ${CMAKE_CURRENT_SOURCE_DIR}/Traits/Methods.h)
