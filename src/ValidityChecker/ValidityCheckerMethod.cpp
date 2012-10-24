#include "ValidityCheckerMethod.h"

ValidityCheckerMethod::
ValidityCheckerMethod() : MPBaseObject(), m_validity(true) {
}

ValidityCheckerMethod::
ValidityCheckerMethod(XMLNodeReader& _node, MPProblem* _problem) : MPBaseObject(_node, _problem), m_validity(true) {
}

ValidityCheckerMethod::
~ValidityCheckerMethod() {
}
  
bool
ValidityCheckerMethod::
IsInsideObstacle(const Cfg& _cfg, Environment* _env, CDInfo& _cdInfo) {
  cerr << "error: IsInsideObstacle() not defined." << endl;
  exit(-1);
}

