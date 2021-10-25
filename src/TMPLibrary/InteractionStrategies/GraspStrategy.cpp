#include "GraspStrategy.h"


/*----------------------- Construction -----------------------*/

GraspStrategy::
GraspStrategy() {
  this->SetName("GraspStrategy");
}

GraspStrategy::
GraspStrategy(XMLNode& _node) {
  this->SetName("GraspStrategy");
}

GraspStrategy::
~GraspStrategy() {}

/*------------------------ Interface -------------------------*/

bool
GraspStrategy::
operator()(Interaction* _interaction, State& _start) {

  // TODO::Handle empty start state, sample object placement

  // TODO::Iterate through stages

  // TODO::For each stage
    //TODO::Set/sample object pose based off of static status

    //TODO::Compute EE placement relative object pose

    //TODO::Compute joint angles using IK

    //TODO::Compute path from previous stage to next stage

  return true;
}

/*--------------------- Helper Functions ---------------------*/
