#include "RoadmapFollowingAgent.h"

#include <limits>

#include "Behaviors/Controllers/ControllerMethod.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"

#include "nonstd.h" // for print_container for debug output



/*------------------------------ Construction --------------------------------*/

RoadmapFollowingAgent::
RoadmapFollowingAgent(Robot* const _r) : Agent(_r) { }

RoadmapFollowingAgent::
~RoadmapFollowingAgent() {
  // Ensure agent is properly torn down.
  RoadmapFollowingAgent::Uninitialize();
}

/*------------------------------ Agent Interface -----------------------------*/

void
RoadmapFollowingAgent::
Initialize() {
  if(m_initialized)
    return;
  m_initialized = true;

  // Get problem info.
  auto problem = m_robot->GetMPProblem();
  const std::string& xmlFile = problem->GetXMLFilename();

  // Initialize the agent's planning library.
  m_library = new MPLibrary(xmlFile);

  /// @TODO Choose the task rather than just taking the first one.
  auto task = problem->GetTasks().front();

  // Create a new solution object to hold a plan for this agent.
//  auto solution = new MPSolution(task->GetRobot());
  m_solution = new MPSolution(task->GetRobot());

  // Use the planning library to find a path.
  m_library->Solve(problem, task, m_solution);

  // Save the roadmap.
  m_roadmap = m_solution->GetPath(); //I've verified this is the solution path
  //std::cout << "MPSolution found! Now simulating the solution path of length "
  //            << m_roadmap->VIDs().size() << std::endl;

  //initialize the path iterator:
  m_currentVID = m_roadmap->VIDs().begin();
  m_currentVertexPathIndex = 0;
}

void PrintRobotBaseForceVector(Robot* robot) {
  auto task = robot->GetMPProblem()->GetTasks().front();
  btMultiBody* mBody = task->GetRobot()->GetDynamicsModel()->Get();
  btVector3DoubleData printData;
  //since bullet prints all 0 vectors weirdly:
  mBody->getBaseForce().serializeDouble(printData);
  std::cout << "body base force = {" << printData.m_floats[0] << ", "
            << printData.m_floats[1] << ", "
            << printData.m_floats[2] << ", " << printData.m_floats[3] << "}"
            << std::endl;
}


void
RoadmapFollowingAgent::
Step(const double _dt) {

  //If it hasn't been done yet, get all problem info and run PMPL to
  // solve the problem, and make the solution/roadmap:
  Initialize();

  //Check that the _dt is constant and what we expect:
//  std::cout << "We expect a _dt of " << 1./30. << " and actual _dt = "
//            << _dt << std::endl;
//  std::cout << "This is a difference of " << (1./30. - _dt) << std::endl;

  //Execute the control that should be for m_delayStepCount more times:
  if(m_delayStepCount > 0) {
    if(!m_delayControl) {
      RunTimeException(WHERE, "Roadmap following agent was given a null control"
          " (note this is different from a null actuator in a control) with a "
          "deisred number of time steps. This sholdn't happen!");
    }

    m_delayStepCount--; //count down to 0, which is when we move to next control

    //std::cout << "RoadmapFollowingAgent::Step() continuing same control. "
    //          "Remaining delay count = " << m_delayStepCount << std::endl;

    //TODO Confirm this is correct, but I assume I must execute the control
    // each time this is "delayed"
    if(m_delayControl->actuator) // If actuator is null, we are coasting.
      m_delayControl->Execute();

    if(m_delayStepCount == 0) {
      delete m_delayControl; // Clean up the copied control
      m_delayControl = nullptr;
    }

//    PrintRobotBaseForceVector(m_robot);//should have forces here

    /// @TODO TODO Big note!!! This part is not ready for multiple controls on
    /// a single edge! Obviously it's not supported in general yet, but
    /// I'm not sure if the numTimeStep will need to be changed for it to work
    /// (since my belief is that the control must be executed m_numTimeSteps
    /// times over that many time steps, so the single time step that the
    /// weight has wouldn't logically cover multiple controls.)
    /// I believe all that is needed is to vectorize m_delayStepCount and m_delayControl

    return; // We must not get a new control, so return.
  }

//  PrintRobotBaseForceVector(m_robot);//should have a purely 0 vector here


  // Check if out of path.
  if(m_currentVID + 1 >= m_roadmap->VIDs().end()) {
    m_currentVID = m_roadmap->VIDs().end(); // Make it clear the path is done.
    /// @TODO Force the agent to stop the robot here, as right now the agent
    ///  just floats off forever after controls stop being applied.
    return;
  }

  // Now check if trying to move between the same Cfg (shouldn't happen).
  if(*m_currentVID == *(m_currentVID + 1)) {
    m_currentVID++; // Attempt the next one next time Step() is called.
    return;
  }

  //Using the Length() function as a basis (from Path.h):
  typename GraphType::edge_descriptor ed(*m_currentVID, *(m_currentVID + 1));
  typename GraphType::vertex_iterator vi;
  typename GraphType::adj_edge_iterator ei;

  WeightType weight;
  //If the edge exists, grab the weight from it. Throw exception otherwise.
  if(m_roadmap->GetRoadmap()->GetGraph()->find_edge(ed, vi, ei)) {
    //So by piecing together type info between RoadmapGraph.h (the type that
    // GetGraph() returns) and graph.h, I see that property() itself should
    // be the WeightType that I want. This is good.
    weight = (*ei).property();
  }
  else
    throw RunTimeException(WHERE, "Couldn't find next edge in roadmap for "
        "the roadmap following agent.");

  //Print some info:
  Cfg shouldBe = m_roadmap->Cfgs().at(m_currentVertexPathIndex);
  Cfg current = m_robot->GetDynamicsModel()->GetSimulatedState();
  //Extremely helpful debugging output:
//  std::cout << std::endl <<"--------------------------------------------------";
//  std::cout << std::endl << "Robot is currently at        "
//            << nonstd::print_container(current.GetData()) << ", "
//            << nonstd::print_container(current.GetVelocity()) << std::endl;
//  std::cout << "Robot SHOULD currently be at "
//            << nonstd::print_container(shouldBe.GetData()) << ", "
//            << nonstd::print_container(shouldBe.GetVelocity())
//            << std::endl
//            << "Desired Robot cfg minus current cfg = " << shouldBe - current
//            << std::endl
//            << "Euclidian distance = " << (shouldBe-current).Magnitude()
//            << std::endl << std::endl;

  if(shouldBe != current) {
    RunTimeException(WHERE, "The simulated configuration is off from the "
        "planned configuration! This means that a test for the test suite has "
        "failed if this is a test, and the roadmap will not be followed "
        "perfectly whether or not this is a test.");
  }

  //Get the control set (can be size 1) from the edge weight:
  ControlSet controls = weight.GetControlSet();

  if(controls.size() > 1)
    RunTimeException(WHERE, // See top of function for detailed explanation.
        "More than one control per edge is not yet supported!");

  //Set the number of delay steps that the control should be applied for:
  //TODO this needs to eventually be vectorized
  m_delayStepCount = weight.GetTimeSteps() - 1;
  //minus 1 because we are about to execute it THIS time step, then the
  // remaining executions will happen the next m_delayStepCount times Step()
  // is called.

  //std::cout << "About to apply " << controls.size() << " control(s)"
  //          << " from vertex " << *m_currentVID << std::endl;

  for(std::size_t i = 0; i < controls.size(); i++) {

    //Update the control that will be repeated m_delayStepCount times. This
    // needs to happen whether coast or normal control, so do it first:
    //TODO vectorize this part, but it's pointless now since I require one control
    m_delayControl = new Control(controls.at(i));


    //Very important! If the actuator is null, it's a coast control.
    if(!m_delayControl->actuator) {
      //std::cout << "Coast control applied. There are " << m_delayStepCount
      //          << " remaining coast controls to be applied"<< std::endl;
      continue;
    }

    //Non-null actuator, so execute it:
    m_delayControl->Execute(); // Execute the control (has robot/actuator info)

    //std::cout << "Applying control " << i+1 << "/" << controls.size()
    //          << " from MPSolution roadmap." << std::endl
    //          << "Control = " << controls.at(i) << " will be repeated "
    //          << m_delayStepCount << " more times." << std::endl;
  }

//  PrintRobotBaseForceVector(m_robot);//should have forces added here

  //Iterate to the next VID in the path, for the next time Step() is called.
  // Note we already have the controls needed for this edge, so it should be
  // fine to do this right now (might move elsewhere though).
  m_currentVID++;
  m_currentVertexPathIndex++;
}


void
RoadmapFollowingAgent::
Uninitialize() {
  if(!m_initialized)
    return;
  m_initialized = false;

  m_roadmap = nullptr; //Deleting solution should properly take care of roadmap
  delete m_solution;
  delete m_library;
}

/*----------------------------------------------------------------------------*/
