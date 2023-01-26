void MPLibraryUseCase() {
  // The MPProblem holds the representation of the motion planning problem. It
  // includes things such as the environment, the tasks, and the robots. It is
  // typically defined and parsed from an XML file.
  MPProblem* mpProblem = this->GetMPProblem();

  // MPTask defines the motion planning task. If assigned to a specific robot,
  // the robot will perform the motion planning task. If not assigned (nullptr),
  // any robot can complete the task. An MPTask can be created by defining a
  // robot, start and goal constraints, and, if necessary, path constraints.
  // MPTasks can also be defined and parsed from an XML file.
  MPTask* task = new MPTask(<Specified Robot*>);

  // MPSolution stores the motion plan solved by the planning algorithm. It
  // contains information such as the roadmap and path for each robot. It can be
  // created by passing in the robot to which the motion plan applies to.
  MPSolution* solution = new MPSolution(<Specified Robot*>);

  // The solver label is a string that corresponds to the XML node label for the
  // motion planning algorithm you would like to use to solve the motion
  // planning problem.
  std::string solverLabel = "PRM";

  // The seed is used to control any random generators used within the solver.
  long seed = 1234;

  // The base filename is a string that is used to name all output files.
  // Output files can be set in the XML file and can contain information
  // pertaining to the performance of the solver.
  std::string baseFilename = "MPLibrary_Example";

  // The solve call within the MPLibrary will take in all of the information
  // provided from the MPProblem, MPTask and use the specified solver to store
  // the path solution in the provided MPSolution object.
  this->GetMPLibrary()->Solve(mpProblem, task, solution, solverLabel, seed,
      baseFilename);
}
