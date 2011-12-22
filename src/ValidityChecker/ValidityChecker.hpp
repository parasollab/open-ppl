#ifndef _VALIDITY_CHECKER_HPP_
#define _VALIDITY_CHECKER_HPP_

//////////////////////////////////////////////////////////////////////////////////////////
#include "CollisionDetection.h"
#include "ValidityCheckerMethod.hpp"
#include "CollisionDetectionValidity.hpp"
#include "NodeClearanceValidity.h"
#include "MedialAxisClearanceValidity.h"
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
  ValidityChecker(std::map<std::string, VCMethodPtr> _map) : m_map_vcmethods(_map) {};


  ValidityChecker(XMLNodeReader& in_Node,  MPProblem* in_pProblem) : MPBaseObject(in_Node, in_pProblem) { 
    if(m_debug) cout << "ValidityChecker::ValidityChecker()" << endl;
    in_Node.verifyName(std::string("validity_test"));

    m_Validity=true;
    
    XMLNodeReader::childiterator citr;
    for(citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr) {    
      if(citr->getName() == "CollisionDetection") {
        VCMethodPtr vc(new CollisionDetectionValidity<CFG>(*citr, in_pProblem));
        AddVCMethod(vc->GetLabel(), vc);
      } else if(citr->getName() == "JointLimit") {
	//VCMethodPtr vc(new JointLimit<CFG>(*citr, in_pProblem));
        //AddVCMethod(vc->GetLabel(), vc);
      } else if(citr->getName() == "ComposeValidity") {
        VCMethodPtr vc(new ComposeValidity<CFG>(*citr, in_pProblem));
        AddVCMethod(vc->GetLabel(), vc);
      } else if(citr->getName() == "NodeClearance") {
        VCMethodPtr vc(new NodeClearanceValidity(*citr, in_pProblem));
        AddVCMethod(vc->GetLabel(), vc);
      } else if(citr->getName() == "MedialAxisClearance") {
        VCMethodPtr vc(new MedialAxisClearanceValidity(*citr, in_pProblem));
        AddVCMethod(vc->GetLabel(), vc);
      } else if(citr->getName() == "NegateValidity") {
        VCMethodPtr vc(new NegateValidity<CFG>(*citr, in_pProblem));
        AddVCMethod(vc->GetLabel(), vc);
      } else {
        citr->warnUnknownNode();
      }
    }    
    if(m_debug) cout << "~ValidityChecker::ValidityChecker()" << endl;
  }
  
  ~ValidityChecker() {}
   
   bool GetValidity(){return m_Validity;}
  void ToggleValidity(){m_Validity=!m_Validity;}
  
  inline bool IsValid(VCMethodPtr _vc, Cfg& _cfg, Environment* env, Stat_Class& Stats, 
		      CDInfo& _cdInfo, bool enablePenetration, std::string *pCallName = NULL) {
      if(m_Validity)
         return _vc->IsValid(_cfg, env, Stats, _cdInfo, enablePenetration, pCallName);
      else
         return !_vc->IsValid(_cfg, env, Stats, _cdInfo, enablePenetration, pCallName);
  }
  
  inline void AddVCMethod(std::string name, VCMethodPtr m) {
    m_map_vcmethods[name] = m;
  }
  
  inline VCMethodPtr GetVCMethod(std::string name) {
    return m_map_vcmethods[name]; 
  }
  
protected:
  std::map<std::string, VCMethodPtr> m_map_vcmethods;
  bool m_Validity;
};


#endif // #ifndef _VALIDITY_CHECKER_HPP
