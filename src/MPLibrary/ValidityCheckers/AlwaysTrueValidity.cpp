#include "AlwaysTrueValidity.h"


/*------------------------------ Construction --------------------------------*/

AlwaysTrueValidity::
AlwaysTrueValidity() {
  this->SetName("AlwaysTrueValidity");
}


AlwaysTrueValidity::
AlwaysTrueValidity(XMLNode& _node) : ValidityCheckerMethod(_node) {
  this->SetName("AlwaysTrueValidity");
}

/*------------------------- ValidityChecker Interface ------------------------*/

bool
AlwaysTrueValidity::
IsValidImpl(Cfg& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
  _cfg.SetLabel("Lazy", true);
  return true;
}


bool
AlwaysTrueValidity::
IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  return true;
}

/*----------------------------------------------------------------------------*/
