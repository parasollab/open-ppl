#include "ProximityCondition.h"

#include "Geometry/Boundaries/Boundary.h"

#include "MPProblem/RobotGroup/RobotGroup.h"

#include "TMPLibrary/TMPLibrary.h"

#include "Utilities/MPUtils.h"

#include <math.h>

/*----------------------- Construction -----------------------*/

ProximityCondition::
ProximityCondition() {
  this->SetName("ProximityCondition");
}

ProximityCondition::
ProximityCondition(XMLNode& _node, TMPLibrary* _library) 
                  : Condition(_node,_library) {
  ParseXML(_node);
}

ProximityCondition::
~ProximityCondition() {}
/*------------------------ Interface -------------------------*/

RobotGroup*
ProximityCondition::
Satisfied(const State& _state) const {

  const auto center = ComputeCompositeCenter(_state);

  // Set the boundary center to the composite center.
  m_boundary->SetCenter(center);

  // Check if all cfgs fit inside the boundary.
  for(auto kv : _state) {
    auto group = kv.first;
    auto grm = kv.second.first;
    auto gvid = kv.second.second;
    auto gcfg = grm->GetVertex(gvid);

    for(auto robot : group->GetRobots()) {
      auto cfg = gcfg.GetRobotCfg(robot);
      if(!m_boundary->InBoundary(cfg))
        return nullptr;
    }
  }

  return _state.begin()->first;
}

/*------------------------ Accessors -------------------------*/

Boundary*
ProximityCondition::
GetBoundary(State _state){
  const auto center = ComputeCompositeCenter(_state);
  m_boundary->SetCenter(center);
  return m_boundary.get();
}

/*--------------------- Helper Functions ---------------------*/ 

void
ProximityCondition::
ParseXML(XMLNode& _node) {
  this->SetName("ProximityCondition");

  m_boundary = Boundary::Factory(_node);

  m_threshold = _node.Read("threshold", false, m_boundary->GetMaxDist(), 0.0, MAX_DBL,
                        "Proximity radius for condition.");
}

std::vector<double>
ProximityCondition::
ComputeCompositeCenter(const State& _state) const {

  // Collect the center points of all the robots in the input state.
  std::vector<double> center(3,0);
  double numRobots = 0;

  for(auto kv : _state) {
    auto group = kv.first;
    auto gcfg = kv.second.first->GetVertex(kv.second.second);

    for(auto robot : group->GetRobots()) {
      auto cfg = gcfg.GetRobotCfg(robot);
      numRobots += 1;

      auto mb = robot->GetMultiBody();
      mb->Configure(cfg);
      auto base = mb->GetBase();
      auto bbx = base->GetBoundingBox();
      auto transformation = base->GetWorldTransformation();
      bbx = bbx.operator*=(transformation);

      auto centroid = bbx.GetCentroid();
      for(size_t i = 0; i < center.size(); i++) {
        center[i] += centroid[i];
      }
    }
  }

  // Average the center values to get the composite center.
  for(size_t i = 0; i < center.size(); i++) {
    center[i] = center[i]/numRobots;
  }

  return center;
}
/*------------------------------------------------------------*/
