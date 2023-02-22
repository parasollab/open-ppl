cmake_minimum_required(VERSION 3.15)

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

set(CLIENT_SRCS
        PARENT_SCOPE
        )

set(CLIENT_SRCS
        PARENT_SCOPE
        )

set(SIM_SRCS
        $(CMAKE_CURRENT_SOURCE_DIR)/sim_main.cpp
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
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Controllers/CarlikeNeedleController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Controllers/ControllerMethod.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Controllers/ControlSetGenerators.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Controllers/SimpleController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Behaviors/Controllers/ICreateController.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ConfigurationSpace/Cfg.cpp
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
        ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MapEvaluators/SIPPMethod.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/MeanCurvatureSkeleton3D.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/ReebGraphConstruction.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/MPTools/TetGenDecomposition.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/CollisionDetection/CollisionDetectionMethod.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/CollisionDetection/RapidCollisionDetection.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/CollisionDetection/PQPCollisionDetection.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/ValidityCheckers/CollisionDetection/SpheresCollisionDetection.cpp
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
        ${CMAKE_CURRENT_SOURCE_DIR}/Workspace/GridOverlay.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Workspace/WorkspaceDecomposition.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Workspace/WorkspacePortal.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Workspace/WorkspaceRegion.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Workspace/WorkspaceSkeleton.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MPLibrary/PMPL.cpp
        PARENT_SCOPE
)