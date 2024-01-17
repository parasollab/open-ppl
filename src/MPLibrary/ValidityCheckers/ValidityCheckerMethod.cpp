#include "ValidityCheckerMethod.h"

#include <string>

/*------------------------ Validity Checker Interface ------------------------*/

bool
ValidityCheckerMethod::
GetValidity() const {
  return m_validity;
}


void
ValidityCheckerMethod::
ToggleValidity() {
  m_validity = !m_validity;
}


bool
ValidityCheckerMethod::
IsValid(Cfg& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  return m_validity == IsValidImpl(_cfg, _cdInfo, _caller);
}


bool
ValidityCheckerMethod::
IsValid(Cfg& _cfg, const std::string& _caller) {
  CDInfo cdInfo;
  return IsValid(_cfg, cdInfo, _caller);
}


bool
ValidityCheckerMethod::
IsValid(GroupCfgType& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  return m_validity == IsValidImpl(_cfg, _cdInfo, _caller);
}


bool
ValidityCheckerMethod::
IsValid(GroupCfgType& _cfg, const std::string& _caller) {
  CDInfo cdInfo;
  return IsValid(_cfg, cdInfo, _caller);
}

/*--------------------------------- Helpers ----------------------------------*/

bool
ValidityCheckerMethod::
IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  throw NotImplementedException(WHERE) << "No base class implementation is "
                                       << "provided.";
}

/*----------------------------------------------------------------------------*/
