#ifndef NEGATEVALIDITY_H
#define NEGATEVALIDITY_H

#include "ValidityCheckerMethod.h"
#include "ValidityCheckerFunctor.h"

template<class MPTraits>
class NegateValidity : public ValidityCheckerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    NegateValidity(string _label = "");
    NegateValidity(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~NegateValidity() { }

    virtual bool 
      IsValidImpl(CfgType& _cfg, Environment* _env, StatClass& _stats, 
          CDInfo& _cdInfo, string *_callName);

  private:
    string m_vcLabel;
};

template<class MPTraits>
NegateValidity<MPTraits>::NegateValidity(string _label) : 
  ValidityCheckerMethod<MPTraits>(), m_vcLabel(_label) {
    this->m_name = "NegateValidity";
  }

template<class MPTraits>
NegateValidity<MPTraits>::NegateValidity(MPProblemType* _problem, XMLNodeReader& _node) :
  ValidityCheckerMethod<MPTraits>(_problem, _node) {
    _node.verifyName("NegateValidity");
    this->m_name = "NegateValidity";
    m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "validity checker method");    
  }

template<class MPTraits>
bool
NegateValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, Environment* _env, StatClass& _stats, CDInfo& _cdInfo, 
    string *_callName) {
  vector<ValidityCheckerPointer> vcMethods;
  typedef typename vector<ValidityCheckerPointer>::iterator VCIterator;
  vcMethods.push_back(this->GetMPProblem()->GetValidityChecker(m_vcLabel));

  ValidityCheckerFunctor<MPTraits> comFunc(_cfg, _env, _stats, _cdInfo, _callName); 

  ComposeNegate<VCIterator, ValidityCheckerFunctor<MPTraits> > composeNegate;
  return composeNegate(vcMethods.begin(), comFunc);
}

#endif// #ifndef NEGATEVALIDITY_H
