#include "FixedBase.h"

#include "Geometry/Boundaries/CSpaceBoundingBox.h"

#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"

FixedBase::
FixedBase(MPProblem* _problem) : ITPlacementMethod(_problem) {}


FixedBase::
FixedBase(XMLNode& _node) : ITPlacementMethod(_node) {}

void
FixedBase::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library, TMPStrategyMethod* _tmpMethod){
  auto tasks = _it->GetInformation()->GetInteractionTasks();
  CSpaceBoundingBox* standard = new CSpaceBoundingBox(1);
  *standard = *(static_cast<const CSpaceBoundingBox*>(tasks[0]->GetStartConstraint()->GetBoundary()));

  auto& robots = m_problem->GetRobots();

  for(auto& robot : robots){
    auto mb = robot->GetMultiBody();
    /*if(mb->GetBaseType() != Body::Type::Fixed){
      continue;
    }*/
    if(robot->GetLabel() == "coordinator_m" || robot->GetLabel() == "coordinator"){
      continue;
    }
    const size_t numDOF = mb->PosDOF() + mb->OrientationDOF();
    auto basePos = robot->GetSimulationModel()->GetState();
    auto bbx = standard;//static_cast<const CSpaceBoundingBox*>(standard);

    for(size_t i = 0; i < numDOF; i++){
      bbx->SetRange(i, basePos[i], basePos[i]);
    }
    //TODO convert bbx into a cfg
    //probably just sample with the robot (change library task)

    auto oldRobot = _library->GetTask()->GetRobot();
    _library->GetTask()->SetRobot(robot.get());


    std::vector<Cfg> basePoints;
    auto sampler = _library->GetSampler("UniformRandomFree");
    size_t numNodes = 1, numAttempts = 100;
    sampler->Sample(numNodes, numAttempts, bbx,
        std::back_inserter(basePoints));

    if(basePoints.empty())
      throw RunTimeException(WHERE, "No valid fixed base position for interaction template placement.");

    // Add location Cfg to IT
    _it->GetInformation()->AddTemplateLocation(basePoints[0]);

    std::cout << "IT Location: " << basePoints[0].PrettyPrint() << std::endl;

    _library->GetTask()->SetRobot(oldRobot);
  }

  //_solution->AddInteractionTemplate(_it);
}
