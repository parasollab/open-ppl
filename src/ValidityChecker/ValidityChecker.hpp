#ifndef _VALIDITY_CHECKER_HPP_
#define _VALIDITY_CHECKER_HPP_

//////////////////////////////////////////////////////////////////////////////////////////
#include "CollisionDetection.h"
#include "ValidityCheckerMethod.hpp"
#include "CollisionDetectionValidity.hpp"
#include "ComposeVC.hpp"
#include "NegateValidity.hpp"
#include "boost/shared_ptr.hpp"
//////////////////////////////////////////////////////////////////////////////////////////

template<typename CFG>
class ValidityChecker : public MPBaseObject
{   
public:
  typedef boost::shared_ptr<ValidityCheckerMethod> VCMethodPtr;
  
  ValidityChecker() { }
  ValidityChecker(XMLNodeReader& in_Node,  MPProblem* in_pProblem) : MPBaseObject(in_Node, in_pProblem) { 
    LOG_DEBUG_MSG("ValidityChecker::ValidityChecker()");
    in_Node.verifyName(std::string("validity_test"));
    
    XMLNodeReader::childiterator citr;
    for(citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr) {    
      if(citr->getName() == "CollisionDetection") {
        VCMethodPtr vc(new CollisionDetectionValidity<CFG>(*citr, in_pProblem));
        AddVCMethod(vc->GetObjectLabel(), vc);
      } else if(citr->getName() == "JointLimit") {
	//VCMethodPtr vc(new JointLimit<CFG>(*citr, in_pProblem));
        //AddVCMethod(vc->GetLabel(), vc);
      } else if(citr->getName() == "ComposeValidity") {
        VCMethodPtr vc(new ComposeValidity<CFG>(*citr, in_pProblem));
        AddVCMethod(vc->GetObjectLabel(), vc);
      } else if(citr->getName() == "NegateValidity") {
        VCMethodPtr vc(new NegateValidity<CFG>(*citr, in_pProblem));
        AddVCMethod(vc->GetObjectLabel(), vc);
      } else {
        citr->warnUnknownNode();
      }
    }    
    LOG_DEBUG_MSG("~ValidityChecker::ValidityChecker()");
  }
  
  ~ValidityChecker() {}
  
  inline bool IsValid(VCMethodPtr _vc, CFG& _cfg, Environment* env, Stat_Class& Stats, 
		      CDInfo& _cdInfo, bool enablePenetration, std::string *pCallName = NULL) {
    return _vc->IsValid(_cfg, env, Stats, _cdInfo, enablePenetration, pCallName);
  }
  
  inline void AddVCMethod(std::string name, VCMethodPtr m) {
    m_map_vcmethods[name] = m;
  }
  
  inline VCMethodPtr GetVCMethod(std::string name) {
    return m_map_vcmethods[name]; 
  }
  
protected:
  std::map<std::string, VCMethodPtr> m_map_vcmethods;
};


#endif // #ifndef _VALIDITY_CHECKER_HPP
