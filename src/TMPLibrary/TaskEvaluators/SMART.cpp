#include "SMART.h"

/*------------------------------- Construction -------------------------------*/

SMART::
SMART() {
  this->SetName("SMART");
}

SMART::
SMART(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("SMART");
}

/*---------------------------- Initialization --------------------------------*/

void
SMART::
Initialize() {

}

/*-------------------------------- Helpers -----------------------------------*/

bool
SMART::
Run(Plan* _plan) {
  return false;
}

/*----------------------------------------------------------------------------*/
