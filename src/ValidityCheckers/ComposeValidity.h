//////////////////////////////////////////////
//ComposeValidity.h
//
//This class composes one or more validity 
//checker methods.
/////////////////////////////////////////////

#ifndef COMPOSEVALIDITY_H
#define COMPOSEVALIDITY_H

#include "ValidityCheckerMethod.h"
#include "ValidityCheckerFunctor.h"

template<class MPTraits>
class ComposeValidity : public ValidityCheckerMethod<MPTraits> {  
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    enum LogicalOperator { AND, OR };

    ComposeValidity(LogicalOperator _logicalOperator = AND,
        const vector<string>& _vcLabel = vector<string>());
    ComposeValidity(MPProblemType* _problem, XMLNodeReader& _node);   
    virtual ~ComposeValidity() {}

    virtual bool 
      IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
          CDInfo& _cdInfo, string* _callName = NULL);

  private:
    LogicalOperator m_logicalOperator;
    vector<string> m_label;
};

template<class MPTraits>
ComposeValidity<MPTraits>::ComposeValidity(LogicalOperator _operator,
    const vector<string>& _vcLabel) : 
  ValidityCheckerMethod<MPTraits>(), m_logicalOperator(_operator), m_label(_vcLabel) {
    this->m_name = "ComposeValidity";
  }

template<class MPTraits>
ComposeValidity<MPTraits>::ComposeValidity(MPProblemType* _problem, XMLNodeReader& _node) :
  ValidityCheckerMethod<MPTraits>(_problem, _node) {
    _node.verifyName("ComposeValidity");
    this->m_name = "ComposeValidity";

    string logicalOperator = _node.stringXMLParameter("operator", true, "", "operator");    
    if(logicalOperator == "AND" || logicalOperator == "and") {
      m_logicalOperator = AND;
    }
    else if(logicalOperator == "OR" || logicalOperator == "or") {
      m_logicalOperator = OR;
    }
    else {
      cout << "unknown logical operator label is read " << endl;
      exit(-1);
    }

    for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
      if(citr->getName() == "ValidityChecker") {
        string methodLabel = citr->stringXMLParameter("label", true, "", "validity checker method");
        m_label.push_back(methodLabel);
      }
      else {
        citr->warnUnknownNode();
      }
    }
  }

template<class MPTraits>
bool
ComposeValidity<MPTraits>::IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, CDInfo& _cdInfo, string* _callName) {
  vector<ValidityCheckerPointer> vcMethod;
  typedef typename vector<ValidityCheckerPointer>::iterator VCIterator;
  for(vector<string>::iterator it = m_label.begin(); it != m_label.end(); ++it) {
    vcMethod.push_back(this->GetMPProblem()->GetValidityChecker(*it));
  }

  ValidityCheckerFunctor comFunc(_cfg, _env, _stats, _cdInfo, _callName); 

  if (m_logicalOperator == AND) {
    Compose<VCIterator, logical_and<bool>, ValidityCheckerFunctor> andRelation;
    return andRelation(vcMethod.begin(), vcMethod.end(), logical_and<bool>(), comFunc);
  }
  else if (m_logicalOperator == OR) {
    Compose<VCIterator, logical_or<bool>, ValidityCheckerFunctor> orRelation;
    return orRelation(vcMethod.begin(), vcMethod.end(), logical_or<bool>(), comFunc);
  }
  else { 
    cerr << "Error::Compose Validity read unknown label." << endl;
    return false;
  }        
}

#endif
