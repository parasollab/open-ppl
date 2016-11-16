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

#include "Simulator/Simulator.h"
#include "sandbox/gui/main_window.h"

using namespace std;

int
main(int _argc, char** _argv) {
  try {
    if(_argc < 3 || string(_argv[1]) != "-f")
      throw ParseException(WHERE, "Incorrect usage. Usage: -f options.xml");

    MPProblemType* problem = new MPProblemType(_argv[2]);

    Simulator<MPProblemType> simulator(problem);
    simulator.Initialize();

    for(int i = 0; i < 75; ++i)
      simulator.Step();

    QApplication app(_argc, _argv);
    main_window window;
    window.visualization(&simulator);

    window.show();
    app.exec();

    delete problem;

    return 0;
  }
  catch(const std::runtime_error& e) {
    cerr << endl << e.what() << endl;
    return 1;
  }
  catch(...) {
    cerr << "Unknown error." << endl;
    return 1;
  }
}

