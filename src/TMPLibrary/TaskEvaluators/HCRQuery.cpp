#include "HCRQuery.h"

#include "TMPLibrary/Solution/Plan.h"

/*----------------------- Construction -----------------------*/

HCRQuery::
HCRQuery() {
  this->SetName("HCRQuery");
}

HCRQuery::
HCRQuery(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("HCRQuery");
}

HCRQuery::
~HCRQuery() {}

/*------------------------ Interface -------------------------*/

bool
HCRQuery::
Run(Plan* _plan) {
  
  temp++;
  return temp >= 10;

  return false;
}

/*--------------------- Helper Functions ---------------------*/


/*------------------------------------------------------------*/
