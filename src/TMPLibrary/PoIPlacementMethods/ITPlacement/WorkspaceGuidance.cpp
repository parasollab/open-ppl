#include "WorkspaceGuidance.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/TaskPlan.h"

WorkspaceGuidance::
WorkspaceGuidance(XMLNode& _node) : ITPlacementMethod(_node) {
  m_dmLabel = _node.Read("dmLabel", false, "positionEuclidean",
                         "distance metric to track distance between interaction templates");
  m_distanceThreshold = _node.Read("distanceThreshold", true, nan(""), 0., 1000.,
                                   "Closest distance any two ITs can be to each other");

}

std::unique_ptr<ITPlacementMethod>
WorkspaceGuidance::
Clone(){
	return std::unique_ptr<WorkspaceGuidance>(new WorkspaceGuidance(*this));	
}

void
WorkspaceGuidance::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution){
  BuildSkeleton();

  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();
  auto g = m_skeleton.GetGraph();
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);

  for(auto vi = g.begin(); vi != g.end(); vi++){
    std::cout << vi->property() << std::endl;
    Cfg location(vi->property(), robot);
    bool check = true; //distance check
    for(auto cfg : _it->GetInformation()->GetTemplateLocations()){
      if(dm->Distance(location, cfg) < m_distanceThreshold){
        check = false;
        break;
      }
    }
    if(check){
      _it->GetInformation()->AddTemplateLocation(location);
      std::cout << location.PrettyPrint() << std::endl;
    }
  }

}

void
WorkspaceGuidance::
BuildSkeleton(){

  if(m_initialized)
    return;
  m_initialized = true;

  //MethodTimer mt(this->GetStatClass(), "BuildSkeleton");

  // Determine if we need a 2d or 3d skeleton.
  auto env = this->GetMPProblem()->GetEnvironment();
  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();
  //const bool threeD = robot->GetMultiBody()->GetBaseType() ==
  //    Body::Type::Volumetric;

  //Only considering 2D right now with the icreates
  //if(threeD) {
  if(false){
    // Create a workspace skeleton using a reeb graph.

    /*auto decomposition = this->GetMPLibrary()->GetMPTools()->GetDecomposition(m_decompositionLabel);
    ReebGraphConstruction reeb;
    reeb.Construct(decomposition);

    // Create the workspace skeleton.
    m_skeleton = reeb.GetSkeleton();
    */
  }
  else {
    // Collect the obstacles we want to consider (all in this case).
    std::vector<GMSPolyhedron> polyhedra;
    for(size_t i = 0; i < env->NumObstacles(); ++i) {
      MultiBody* const obstacle = env->GetObstacle(i);
      for(size_t j = 0; j < obstacle->GetNumBodies(); ++j)
        polyhedra.emplace_back(obstacle->GetBody(j)->GetWorldPolyhedron());
    }

    // Build a skeleton from a 2D medial axis.
    MedialAxis2D ma(polyhedra, env->GetBoundary());
    ma.BuildMedialAxis();
    m_skeleton = get<0>(ma.GetSkeleton(1)); // 1 for free space.
  }


  const auto& startConstraint = this->GetMPProblem()->GetTasks(robot)[0]->GetStartConstraint();
  auto startCenter = startConstraint->GetBoundary()->GetCenter();
  Point3d start(startCenter[0], startCenter[1], 0);

  // Only support single-goal tasks; this is inherent to the method. The problem
  // is solvable but hasn't been solved yet.
  const auto& goalConstraints = this->GetMPProblem()->GetTasks(robot)[0]->GetGoalConstraints();
  if(goalConstraints.size() != 1)
    throw RunTimeException(WHERE) << "Only supports single-goal tasks. "
                                  << "Multi-step tasks will need new skeletons "
                                  << "for each sub-component.";
  auto goalCenter = goalConstraints[0]->GetBoundary()->GetCenter();
  Point3d goal(goalCenter[0], goalCenter[1], 0);


  // Get the start and goal point from the goal tracker.
  // Try to prevent non-point tasks by requiring single VIDs for the start and
  // goal.
  /*auto goalTracker = this->GetMPLibrary()->GetGoalTracker();
  goalTracker.reset(new GoalTracker(this->GetMPLibrary()->GetRoadmap(robot),this->GetMPLibrary()->GetTask()));
  const auto& startVIDs = goalTracker->GetStartVIDs();
  const auto& goalVIDs  = goalTracker->GetGoalVIDs(0);
  if(startVIDs.size() != 1)
    throw RunTimeException(WHERE) << "Exactly one start VID is required, but "
                                  << startVIDs.size() << " were found.";
  if(goalVIDs.size() != 1)
    throw RunTimeException(WHERE) << "Exactly one goal VID is required, but "
                                  << goalVIDs.size() << " were found.";

  // Get the start and goal vertices.
  auto g = this->GetMPLibrary()->GetRoadmap(robot)->GetGraph();
  const size_t startVID = *startVIDs.begin(),
            goalVID  = *goalVIDs.begin();
  const Point3d start = g->GetVertex(startVID).GetPoint(),
                goal  = g->GetVertex(goalVID).GetPoint();
  */

  // Direct the workspace skeleton outward from the starting point.
  m_skeleton = m_skeleton.Direct(start);

  // Prune the workspace skeleton relative to the goal.
  m_skeleton.Prune(goal);

}
