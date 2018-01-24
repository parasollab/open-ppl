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

    // Position the robot by sampling from the first task and set colors.
    /// @TODO Decide on a way to declare the starting configuration either
    ///       explicitly or from a specific task. For now we will assume that
    ///       the first task is a query and its start boundary is a single point.
    for(const auto& robot : problem->GetRobots()) {
      Robot* const r = robot.get();
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
