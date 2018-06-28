#include "MPLibrary/PMPL.h"
#include "Simulator/Simulation.h"
#include "Visualization/Gui/Setup.h"

#include "sandbox/gui/main_window.h"
#include "sandbox/gui/gl_widget.h"


int
main(int _argc, char** _argv) {
  try {
    const std::string flag = std::string(_argv[1]);
    if(_argc < 3 || (flag != "-f" and flag != "-e"))
      throw ParseException(WHERE, "Incorrect usage. Usage: {-f|-e} options.xml");

    // Make problem object.
    std::shared_ptr<MPProblem> problem(new MPProblem(_argv[2]));

    // Position the robot by sampling from the first task and set colors.
    /// @TODO Decide on a way to declare the starting configuration either
    ///       explicitly or from a specific task. For now we will assume that
    ///       the first task is a query and its start boundary is a single point.
    for(const auto& robot : problem->GetRobots()) {
      Robot* const r = robot.get();
      if(r->IsVirtual())
        continue;

      // Position the robot at zero, or at the task center if one exists.
      std::vector<double> dofs(r->GetMultiBody()->DOF(), 0);
      if(problem->GetTasks(r).front()->GetStartConstraint())
        dofs = problem->GetTasks(r).front()->GetStartConstraint()->
               GetBoundary()->GetCenter();
      r->GetMultiBody()->Configure(dofs);

      // Randomize body colors for now to help see the robots.
      for(size_t i=0; i< r->GetMultiBody()->GetNumBodies();i++) {
        glutils::color c = {float(DRand() * .5 + .5),
                            float(DRand() * .5 + .5),
                            float(DRand() * .5 + .5), 1.};
        r->GetMultiBody()->GetBody(i)->SetBodyColor(c);
      }
    }

    // Make simulation object.
    const bool editMode = flag == "-e";
    Simulation::Create(problem, editMode);
    Simulation* simulation = Simulation::Get();

    // Make visualizer object.
    QApplication app(_argc, _argv);
    main_window window;
    theOneWindow = &window;

    // Set up the extra gui elements if we are in edit mode.
    if(editMode)
      SetupMainWindow(&window);

    // Load the simulation into the visualizer and start it.
    window.visualization(simulation);
    window.show();
    window.gl()->start();
    app.exec();

    // Clean up the simulation and problem when we are done.
    simulation->Uninitialize();

    return 0;
  }
  catch(const std::runtime_error& _e) {
    // Write exceptions to cout so that we still get them when piping stdout.
    std::cout << std::endl << _e.what() << std::endl;
    return 1;
  }
}
