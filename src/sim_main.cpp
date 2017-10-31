#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "Behaviors/Agents/PathFollowingAgent.h"
#include "Behaviors/Agents/RoadmapFollowingAgent.h"
#include "Behaviors/Agents/AgentGroup.h"
#include "MPLibrary/PMPL.h"
#include "Simulator/Simulation.h"
#include "sandbox/gui/main_window.h"


int
main(int _argc, char** _argv) {
  try {
    if(_argc < 3 || std::string(_argv[1]) != "-f")
      throw ParseException(WHERE, "Incorrect usage. Usage: -f options.xml");

    // Make problem object.
    MPProblem* problem = new MPProblem(_argv[2]);
    auto robot = problem->GetRobot(0);

    // If it's a nonholonomic robot, use the nonholonomic agent to correctly
    // use the roadmap data (control sets between each pair of cfgs in roadmap).
    if(!robot->IsNonholonomic())
      //robot->SetAgent(new PathFollowingAgent(robot));
      robot->SetAgent(new AgentGroup(robot));
      //robot->SetAgent(new PathFollowingChildAgent(robot));
    else
      robot->SetAgent(new PathFollowingAgent(robot));
      //robot->SetAgent(new AgentGroup(robot));
      //robot->SetAgent(new RoadmapFollowingAgent(robot));

    // Position the robot by sampling from the first task.
    /// @TODO Decide on a way to declare the starting configuration either
    ///       explicitly or from a specific task. For now we will assume that
    ///       the first task is a query and its start boundary is a single point.
    
    for(auto r: problem->GetRobots()) {
      auto startBoundary = problem->GetTasks(r).front()->GetStartBoundary();
      r->GetMultiBody()->Configure(startBoundary->GetCenter());
      for(size_t i=0; i< r->GetMultiBody()->GetNumBodies();i++) {
        float c1 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float c2 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float c3 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        glutils::color c = {c1, c2, c3, 1.};
        r->GetMultiBody()->GetBody(i)->SetBodyColor(c);
      }
    }

    // Make simulation object.
    Simulation simulation(problem);

    // Make visualizer object.
    QApplication app(_argc, _argv);
    main_window window;

    // Load the simulation into the visualizer and start it.
    window.visualization(&simulation);
    window.show();
    app.exec();

    // Clean up the simulation and problem when we are done.
    simulation.Uninitialize();
    delete problem;

    return 0;
  }
  catch(const std::runtime_error& e) {
    std::cerr << std::endl << e.what() << std::endl;
    return 1;
  }
}
