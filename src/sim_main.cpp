#ifdef PMPCfg
#include "MPProblem/ConfigurationSpace/Cfg.h"
#include "Traits/CfgTraits.h"
typedef MPTraits<Cfg> PMPLTraits;

#elif (defined(PMPState))
#include "MPProblem/ConfigurationSpace/State.h"
#include "Traits/StateTraits.h"
typedef StateTraits PMPLTraits;

#else
#error "Error, must define a RobotType for PMPL application"
#endif
typedef PMPLTraits::MPProblemType MPProblemType;
typedef PMPLTraits::MPLibraryType MPLibraryType;

#include "Simulator/Simulation.h"
#include "sandbox/gui/main_window.h"

using namespace std;

int
main(int _argc, char** _argv) {
  try {
    if(_argc < 3 || string(_argv[1]) != "-f")
      throw ParseException(WHERE, "Incorrect usage. Usage: -f options.xml");

    // Make problem object.
    MPProblemType* problem = new MPProblemType(_argv[2]);

    // Make simulation object.
    Simulation<MPProblemType> simulation(problem);

    // Make visualizer object.
    QApplication app(_argc, _argv);
    main_window window;

    // Load the simulation into the visualizer and start it.
    window.visualization(&simulation);
    window.show();
    app.exec();

    // Clean up problem when we are done.
    delete problem;

    return 0;
  }
  catch(const std::runtime_error& e) {
    cerr << endl << e.what() << endl;
    return 1;
  }
}

