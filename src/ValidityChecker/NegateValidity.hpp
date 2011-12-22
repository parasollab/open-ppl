#ifndef _NEGATE_VALIDITY_HPP_
#define _NEGATE_VALIDITY_HPP_

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

template<typename CFG> class ValidityChecker;


template<typename CFG>
class NegateValidity : public ValidityCheckerMethod 
{
public:
  
  typedef typename vector<typename ValidityChecker<CFG>::VCMethodPtr>::iterator InputIterator;
  
  NegateValidity() { }
  NegateValidity(string label, vector<typename ValidityChecker<CFG>::VCMethodPtr> vec, ComposeNegate<InputIterator, ComposeFunctor<CFG> > _com_neg);
  NegateValidity(XMLNodeReader& in_Node, MPProblem* in_pProblem);   
  ~NegateValidity() { }
  
  virtual bool 
  IsValid(Cfg& _cfg, Environment* env, Stat_Class& Stats, 
	  CDInfo& _cdInfo, bool enablePenetration, std::string *pCallName);

  virtual vector< pair<CfgType,CfgType> > GetHistory();
  virtual void ClearHistory();

  
private:
  string m_vcLabel;
  vector<typename ValidityChecker<CFG>::VCMethodPtr> m_vec_vcMethod;
  ComposeNegate<InputIterator, ComposeFunctor<CFG> > com_neg;
};

template<typename CFG>
NegateValidity<CFG>::
NegateValidity(string label, std::vector<typename ValidityChecker<CFG>::VCMethodPtr> vec, ComposeNegate<InputIterator, ComposeFunctor<CFG> > _com_neg) : m_vcLabel(label), m_vec_vcMethod(vec), com_neg(_com_neg) {}



template<typename CFG>
NegateValidity<CFG>::
NegateValidity(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
  ValidityCheckerMethod(in_Node, in_pProblem) 
{
  in_Node.verifyName("NegateValidity");
  m_vcLabel = in_Node.stringXMLParameter("method",true,"","method");    
}


template<typename CFG>
bool
NegateValidity<CFG>::
IsValid(Cfg& _cfg, Environment* env, Stat_Class& Stats, CDInfo& _cdInfo, 
	bool enablePenetration, std::string *pCallName = NULL) 
{
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  
  if (m_vec_vcMethod.size() == 0) 
    m_vec_vcMethod.push_back(vc->GetVCMethod(m_vcLabel));
  
  ComposeFunctor<CFG> com_func(vc, _cfg, env, Stats, _cdInfo, enablePenetration, pCallName); 
  
  return com_neg(m_vec_vcMethod.begin(), com_func);
}

template<typename CFG>
vector< pair<CfgType,CfgType> > 
NegateValidity<CFG>::GetHistory() {
  vector< pair<CfgType,CfgType> > empty;
  return empty;
}

template<typename CFG>
void 
NegateValidity<CFG>::ClearHistory() { }


#endif// #ifndef _NEGATE_VALIDITY_HPP_
