#include "Behaviors/Agents/PathFollowingAgent.h"
#include "Behaviors/Agents/RoadmapFollowingAgent.h"
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

    //If it's a nonholonomic robot, use the nonholonomic agent to correctly
    // use the roadmap data (control sets between each pair of cfgs in roadmap).
    if(robot->IsNonholonomic()) {
      robot->SetAgent(new RoadmapFollowingAgent(robot));
    }
    else {
      robot->SetAgent(new PathFollowingAgent(robot));
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
