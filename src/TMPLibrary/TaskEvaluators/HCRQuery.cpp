#include "HCRQuery.h"

#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"
#include "TMPLibrary/Solution/Plan.h"

#include "Utilities/Hypergraph.h"

/*----------------------- Construction -----------------------*/

HCRQuery::
HCRQuery() {
  this->SetName("HCRQuery");
}

HCRQuery::
HCRQuery(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("HCRQuery");
  m_sgLabel = _node.Read("sgLabel", true, "", 
           "Temp till stategraph is embedded in plan.");
}

HCRQuery::
~HCRQuery() {}

/*------------------------ Interface -------------------------*/

bool
HCRQuery::
Run(Plan* _plan) {

  auto hcr = dynamic_cast<CombinedRoadmap*>(
              this->GetStateGraph(m_sgLabel).get());
  
  if(m_debug) {
    hcr->GetHypergraph()->Print();
  }

  temp++;
  return temp >= 10;

  return false;
}

/*--------------------- Helper Functions ---------------------*/


/*------------------------------------------------------------*/
