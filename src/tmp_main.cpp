#include <exception>
#include <limits>
#include <string>

#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Agents/Coordinator.h"
#include "Traits/CfgTraits.h"
#include "TMPLibrary/TMPLibrary.h"
#include "TMPLibrary/Solution/Plan.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/GroupTask.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"
#include "Utilities/PMPLExceptions.h"


#include "MPProblem/Robot/Kinematics/ur_kin.h"

int
main(int _argc, char** _argv) {
  // Assert that this platform supports an infinity for doubles.
  if(!std::numeric_limits<double>::has_infinity)
    throw RunTimeException(WHERE) << "This platform does not support infinity "
                                  << "for double-types, which is required for "
                                  << "pmpl to work properly.";

  if(_argc != 3 || std::string(_argv[1]) != "-f")
    throw ParseException(WHERE) << "Incorrect usage. Usage: -f options.xml";

  // Get the XML file name from the command line.
  std::string xmlFile = _argv[2];

  // Parse the Problem node into an MPProblem object.
  MPProblem* problem = new MPProblem(xmlFile);

  double* T = new double[16];
  double q_sols[8*6];
  printf("James's test\n");
  // Point
  T[3] = .7;
  T[7] = 0;
  T[11] = .2;

  // Orientation matrix
  T[0] = 0;
  T[1] = 0;
  T[2] = 1;

  T[4] = 0;
  T[5] = 1;
  T[6] = 0;

  T[8] = -1;
  T[9] = 0;
  T[10] = 0;

  T[12] = 0;
  T[13] = 0;
  T[14] = 0;
  T[15] = 1;
  for(int i=0;i<4;i++) {
    for(int j=i*4;j<(i+1)*4;j++)
      printf("%1.3f ", T[j]);
    printf("\n");
  }
  auto num_sols = ur_kinematics::inverse(T, q_sols);
  for(int i=0;i<num_sols;i++) 
    printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n", 
       q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);

  // Parse the Library node into an TMPLibrary object.
  TMPLibrary* ppl = new TMPLibrary(xmlFile);

    // Position the robot by sampling from the first task and set colors.
    /// @TODO Decide on a way to declare the starting configuration either
    ///       explicitly or from a specific task. For now we will assume that
    ///       the first task is a query and its start boundary is a single point.
    if(!problem->GetRobotGroups().empty()) {
      for(const auto& group : problem->GetRobotGroups()) {
        //TODO needs to be updated to track which robots have been given a
        //starting position and which ones have not when considering multiple
        //grouptasks
        auto groupTask = problem->GetTasks(group.get()).front();
        for(auto it = groupTask->begin(); it != groupTask->end(); it++){
          Robot* const r = it->GetRobot();
          if(r->IsVirtual())
            continue;

          // Position the robot at zero, or at the task center if one exists.
          std::vector<double> dofs(r->GetMultiBody()->DOF(), 0);
          if(it->GetStartConstraint())
            dofs = it->GetStartConstraint()->
              GetBoundary()->GetCenter();
          r->GetMultiBody()->Configure(dofs);

					// Store robot's initial position
					Cfg initial(r);
					initial.SetData(dofs);
					problem->SetInitialCfg(r,initial);

        }
      }
    }
    else {
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
				// Store robot's initial position
				Cfg initial(r);
				initial.SetData(dofs);
				problem->SetInitialCfg(r,initial);
      }
    }

	/*
  // Create storage for the solution and ask the library to solve our problem.
  std::vector<std::shared_ptr<MPTask>> tasks;
  for(auto& r : problem->GetRobots()) {
		// Collect the robot's tasks
	  auto robotTasks = problem->GetTasks(r.get());
 	  for(auto task : robotTasks)
      if(!task->GetStatus().is_complete())
				tasks.push_back(task);
	}
 	ppl->Solve(problem, tasks);

  // Also solve the group task(s).
	std::vector<std::shared_ptr<GroupTask>> groupTasks;
  if(!problem->GetRobotGroups().empty()) {
    for(auto& robotGroup : problem->GetRobotGroups()) 
	    for(auto groupTask : problem->GetTasks(robotGroup.get()))
				groupTasks.push_back(groupTask);
  	ppl->Solve(problem, groupTasks);
  }

  if(tasks.empty() and groupTasks.empty())
    throw RunTimeException(WHERE) << "No tasks were specified!";
	*/

	for(const auto& decomps : problem->GetDecompositions()) {
		auto a = decomps.first->GetAgent();
		auto c = dynamic_cast<Coordinator*>(a);
		std::vector<Robot*> team;
		for(auto label : c->GetMemberLabels()){
			team.push_back(problem->GetRobot(label));
		}

		for(const auto& decomp : decomps.second) {
			Plan* plan = new Plan();
			plan->SetCoordinator(c);
			plan->SetTeam(team);
			plan->SetDecomposition(decomp.get());
			ppl->Solve(problem, decomp.get(), plan, c, team);
      const std::string basename = problem->GetBaseFilename(),
                        fullname = problem->GetPath(basename + "-ppl.stat");

      // Print the stats to file.
      std::ofstream osStat(fullname);
      plan->GetStatClass()->PrintAllStats(osStat);
		}
	}
  // Release resources.
  delete problem;
  delete ppl;

  return 0;
}
