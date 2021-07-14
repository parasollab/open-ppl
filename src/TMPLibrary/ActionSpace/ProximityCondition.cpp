#include "ProximityCondition.h"

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

  std::vector<std::pair<Robot*,std::vector<double>>> centerPoints;

  for(auto kv : _state) {
    auto group = kv.first;
    auto gcfg = kv.second.first->GetVertex(kv.second.second);

    for(auto robot : group->GetRobots()) {
      auto cfg = gcfg.GetRobotCfg(robot);
      std::vector<double> center(3,0.0);

      for(size_t i = 0; i < cfg.PosDOF(); i++) {
        center[i] = cfg[i];
      }

      centerPoints.push_back(std::make_pair(robot,center));
    }
  }

  for(size_t i = 0; i < centerPoints.size(); i++) {
    for(size_t j = i+1; j < centerPoints.size(); j++) {
      auto& center1 = centerPoints[i].second;
      auto& center2 = centerPoints[j].second;

      auto x = center1[0] - center2[0];
      auto y = center1[1] - center2[1];
      auto z = center1[2] - center2[2];

      x = x*x;
      y = y*y;
      z = z*z;

      auto distance = sqrt(x+y+z);

      if(distance > m_threshold)
        return nullptr;
    }
  }

  return _state.begin()->first;
}

/*--------------------- Helper Functions ---------------------*/ 

void
ProximityCondition::
ParseXML(XMLNode& _node) {
  this->SetName("ProximityCondition");

  m_threshold = _node.Read("threshold", true, 0.0, 0.0, MAX_DBL,
                        "Proximity radius for condition.");
}

/*------------------------------------------------------------*/
