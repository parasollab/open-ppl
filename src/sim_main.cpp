#include "MPLibrary/PMPL.h"

#include "Simulator/Simulation.h"
#include "sandbox/gui/main_window.h"

using namespace std;

int
main(int _argc, char** _argv) {
  try {
    if(_argc < 3 || string(_argv[1]) != "-f")
      throw ParseException(WHERE, "Incorrect usage. Usage: -f options.xml");

    // Make problem object.
    MPProblem* problem = new MPProblem(_argv[2]);

    // Make simulation object.
    Simulation<MPProblem> simulation(problem);

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
    cerr << endl << e.what() << endl;
    return 1;
  }
}

