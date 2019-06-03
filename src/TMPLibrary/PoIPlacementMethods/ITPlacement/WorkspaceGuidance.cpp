#include "WorkspaceGuidance.h"

#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"

WorkspaceGuidance::
WorkspaceGuidance(MPProblem* _problem) : PlacementMethod(_problem) {}


WorkspaceGuidance::
WorkspaceGuidance(MPProblem* _problem, XMLNode& _node) : PlacementMethod(_problem) {
  m_label = _node.Read("label", true, "", "label for a fixed base it placement method");
  m_dmLabel = _node.Read("dmLabel", false, "positionEuclidean",
                         "distance metric to track distance between interaction templates");
  m_distanceThreshold = _node.Read("distanceThreshold", true, nan(""), 0., 1000.,
                                   "Closest distance any two ITs can be to each other");

}

std::unique_ptr<PlacementMethod>
WorkspaceGuidance::
Clone(){
	return std::unique_ptr<WorkspaceGuidance>(new WorkspaceGuidance(*this));	
}

void
WorkspaceGuidance::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library, TMPStrategyMethod* _tmpMethod){
  BuildSkeleton(_library, _tmpMethod);

  auto robot = _tmpMethod->GetRobot();
  auto g = m_skeleton.GetGraph();
  auto dm = _library->GetDistanceMetric(m_dmLabel);

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
BuildSkeleton(MPLibrary* _library, TMPStrategyMethod* _tmpMethod){

  if(m_initialized)
    return;
  m_initialized = true;

  //MethodTimer mt(this->GetStatClass(), "BuildSkeleton");

  // Determine if we need a 2d or 3d skeleton.
  auto env = m_problem->GetEnvironment();
  auto robot = _tmpMethod->GetRobot();
  //const bool threeD = robot->GetMultiBody()->GetBaseType() ==
  //    Body::Type::Volumetric;

  //Only considering 2D right now with the icreates
  //if(threeD) {
  if(false){
    // Create a workspace skeleton using a reeb graph.

    /*auto decomposition = _library->GetMPTools()->GetDecomposition(m_decompositionLabel);
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


  const auto& startConstraint = m_problem->GetTasks(robot)[0]->GetStartConstraint();
  auto startCenter = startConstraint->GetBoundary()->GetCenter();
  Point3d start(startCenter[0], startCenter[1], 0);

  // Only support single-goal tasks; this is inherent to the method. The problem
  // is solvable but hasn't been solved yet.
  const auto& goalConstraints = m_problem->GetTasks(robot)[0]->GetGoalConstraints();
  if(goalConstraints.size() != 1)
    throw RunTimeException(WHERE) << "Only supports single-goal tasks. "
                                  << "Multi-step tasks will need new skeletons "
                                  << "for each sub-component.";
  auto goalCenter = goalConstraints[0]->GetBoundary()->GetCenter();
  Point3d goal(goalCenter[0], goalCenter[1], 0);


  // Get the start and goal point from the goal tracker.
  // Try to prevent non-point tasks by requiring single VIDs for the start and
  // goal.
  /*auto goalTracker = _library->GetGoalTracker();
  goalTracker.reset(new GoalTracker(_library->GetRoadmap(robot),_library->GetTask()));
  const auto& startVIDs = goalTracker->GetStartVIDs();
  const auto& goalVIDs  = goalTracker->GetGoalVIDs(0);
  if(startVIDs.size() != 1)
    throw RunTimeException(WHERE) << "Exactly one start VID is required, but "
                                  << startVIDs.size() << " were found.";
  if(goalVIDs.size() != 1)
    throw RunTimeException(WHERE) << "Exactly one goal VID is required, but "
                                  << goalVIDs.size() << " were found.";

  // Get the start and goal vertices.
  auto g = _library->GetRoadmap(robot)->GetGraph();
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
