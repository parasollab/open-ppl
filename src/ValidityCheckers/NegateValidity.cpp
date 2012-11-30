#include "NegateValidity.h"
#include "MPProblem.h"
#include "ValidityChecker.h"

NegateValidity::
NegateValidity() : ValidityCheckerMethod() {
  m_name = "NegateValidity";
}

NegateValidity::
NegateValidity(string _label, std::vector<ElementSet<ValidityCheckerMethod>::MethodPointer> _vec, 
               ComposeNegate<InputIterator, ComposeFunctor> _composeNegate) : 
    ValidityCheckerMethod(), m_vcLabel(_label), m_vcMethods(_vec), m_composeNegate(_composeNegate) {
  m_name = "NegateValidity";
}

NegateValidity::
NegateValidity(XMLNodeReader& _node, MPProblem* _problem) :
  ValidityCheckerMethod(_node, _problem) {
  _node.verifyName("NegateValidity");
  m_name = "NegateValidity";
  m_vcLabel = _node.stringXMLParameter("method",true,"","method");    
}

bool
NegateValidity::
IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, CDInfo& _cdInfo, 
	std::string *_callName) {
  ValidityChecker* vc = this->GetMPProblem()->GetValidityChecker();
  
  if (m_vcMethods.size() == 0) 
    m_vcMethods.push_back(vc->GetMethod(m_vcLabel));
  
  ComposeFunctor com_func(_cfg, _env, _stats, _cdInfo, _callName); 
  
  return m_composeNegate(m_vcMethods.begin(), com_func);
}

