#include "AlwaysTrueValidity.h"

AlwaysTrueValidity::
AlwaysTrueValidity() : ValidityCheckerMethod() {
  m_name = "AlwaysTrueValidity";
}

AlwaysTrueValidity::
AlwaysTrueValidity(XMLNodeReader& _node, MPProblem* _problem) : ValidityCheckerMethod(_node, _problem) {
  _node.verifyName("AlwaysTrueValidity");
  m_name = "AlwaysTrueValidity";
}

AlwaysTrueValidity::
~AlwaysTrueValidity() {}

bool 
AlwaysTrueValidity::
IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
            CDInfo& _cdInfo, string *_callName) {
  _cfg.SetLabel("Lazy", true);
  return true;
}

