#include <catch2/catch_test_macros.hpp>

#include "TMPLibraryTests.h"
#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Agents/Coordinator.h"
#include "MPProblem/Robot/Robot.h"

#include "TMPLibrary/Solution/Plan.h"
#include "../MPProblem/MPProblemTests.h"
#include "../Traits/TMPTestTraits.h"

/*--------------------------- Construction ---------------------------*/

TMPLibraryTests::
TMPLibraryTests() {
  InitializeMethodSets();
}

TMPLibraryTests::
TMPLibraryTests(const std::string& _xmlFile) {
  InitializeMethodSets();
  ReadXMLFile(_xmlFile);
}

TMPLibraryTests::
~TMPLibraryTests() {}

/*----------------------------- Interface ----------------------------*/

/*-------------------------- Helper Functions ------------------------*/

void
TMPLibraryTests::
InitializeMethodSets() {
  m_taskAllocatorTests = new TaskAllocatorMethodTestSet(this,
      typename TMPTraits::TaskAllocatorMethodList(), "TaskAllocators");
  m_taskDecomposerTests = new TaskDecomposerMethodTestSet(this,
      typename TMPTraits::TaskDecomposerMethodList(), "TaskDecomposer");
  m_taskEvaluatorTests = new TaskEvaluatorMethodTestSet(this,
      typename TMPTraits::TaskEvaluatorMethodList(), "TaskEvaluators");
}
/*---------------------------- XML Helpers -----------------------------------*/

void
TMPLibraryTests::
ReadXMLFile(const std::string& _filename) {
  // Open the XML and get the root node.
  XMLNode tmpNode(_filename, "MotionPlanning");

  // Find the 'TMPLibrary' node.
  XMLNode* library = nullptr;
  for(auto& child : tmpNode)
    if(child.Name() == "TMPLibrary")
      library = &child;

  // Throw exception if we can't find it.
  if(!library)
    throw ParseException(WHERE) << "Cannot find TMPLibrary node in XML file '"
                                << _filename << "'.";

  // Parse the library node to set algorithms and parameters.
  for(auto& child : *library)
    ParseChild(child);

}


bool
TMPLibraryTests::
ParseChild(XMLNode& _node) {
  if(_node.Name() == "TaskAllocators") {
    m_taskAllocatorTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "TaskDecomposers") {
    m_taskDecomposerTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "TaskEvaluators") {
    m_taskEvaluatorTests->ParseXML(_node);
    return true;
  }
  else
    return false;
}
/*-------------------------------- Debugging ---------------------------------*/
/*--------------------------------------------------------------------*/

/*---------------------------------- TESTS ------------------------*/

extern std::string test_ufile;

TEST_CASE( "TMPLibrary TESTS", "[multi-file:5][!mayfail]" ) {
    // Parse the Problem node into an MPProblem object.
    MPProblemTests* problem = new MPProblemTests(test_ufile);

    // Parse the Library node into an TMPLibrary object.
    TMPLibraryTests* ppl = new TMPLibraryTests(test_ufile);

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
			//ppl->Solve(problem, decomp.get(), plan, c, team);
		}
	}

  // Task allocator tests
  auto pplTaskAllocator = *ppl->m_taskAllocatorTests;
  for(auto iter = pplTaskAllocator.begin(); iter != pplTaskAllocator.end(); iter++) {
      std::cout << "Running test for " << iter->first << "..." << std::endl;

      auto test = iter->second.get();
      auto result = test->RunTest();

      std::cout << result.second << std::endl;
      CHECK(result.first == true);
  }

  // Task decomposer tests
  auto pplTaskDecomposer = *ppl->m_taskDecomposerTests;
  for(auto iter = pplTaskDecomposer.begin(); iter != pplTaskDecomposer.end(); iter++) {
      std::cout << "Running test for " << iter->first << "..." << std::endl;

      auto test = iter->second.get();
      auto result = test->RunTest();

      std::cout << result.second << std::endl;
      CHECK(result.first == true);
  }

  // Task evaluator tests
  auto pplTaskEvaluator = *ppl->m_taskEvaluatorTests;
  for(auto iter = pplTaskEvaluator.begin(); iter != pplTaskEvaluator.end(); iter++) {
      std::cout << "Running test for " << iter->first << "..." << std::endl;

      auto test = iter->second.get();
      auto result = test->RunTest();

      std::cout << result.second << std::endl;
      CHECK(result.first == true);
  }

  std::cout << "\nCOMPLETED TMPLibrary TESTS\n" << std::endl;

  delete problem;
  delete ppl;
}
