#ifndef SSSURFACEVALIDITY_H_
#define SSSURFACEVALIDITY_H_

#ifdef PMPSSSurfaceMult

#include "ValidityCheckerMethod.h"
#include "MPProblem/Environment.h"
#include "SurfaceValidity.h"
#include "ValidityCheckers/ValidityCheckerMethod.h"
#include "ValidityCheckers/CollisionDetection/CDInfo.h"
#include "ValidityCheckers/CollisionDetection/CollisionDetectionMethod.h"
#include "ValidityCheckers/CollisionDetection/RapidCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/VClipCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/SolidCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/SpheresCollisionDetection.h"

template<class MPTraits>
class SSSurfaceValidity : public ValidityCheckerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename CfgType::SurfaceMPProblemType TmpProblemType;
    typedef typename CfgType::CompositeCfgType TmpCfgType;
    typedef typename CfgType::TraitsType TmpTraitsType;

    SSSurfaceValidity(string _vcLabel="");
    SSSurfaceValidity(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) ;

    virtual ~SSSurfaceValidity() {}

    virtual bool 
      IsValidImpl(CfgType& _cfg, Environment* _env, StatClass& _stats, CDInfo& _cdInfo, string* _callName);

  private:
    string m_vcLabel;
    TmpProblemType m_prob;
};//end SSSurfaceValidity class definition


template<class MPTraits>
SSSurfaceValidity<MPTraits>::SSSurfaceValidity(string _vcLabel) : ValidityCheckerMethod<MPTraits>(){
  this->m_name = "SSSurfaceValidity";
  this->m_vcLabel = _vcLabel;

  m_prob.AddValidityChecker(typename TmpProblemType::ValidityCheckerPointer(new SurfaceValidity<TmpTraitsType>(m_vcLabel)), "temp");
  m_prob.AddValidityChecker(typename TmpProblemType::ValidityCheckerPointer(new CollisionDetectionValidity<TmpTraitsType>(new Rapid())), "cd1");
  m_prob.SetMPProblem();
}

  template<class MPTraits>
SSSurfaceValidity<MPTraits>::SSSurfaceValidity(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) 
  : ValidityCheckerMethod<MPTraits>(_problem, _node){
    _node.verifyName("SSSurfaceValidity");
    this->m_name = "SSSurfaceValidity";
    this->m_vcLabel = _node.stringXMLParameter("vc_method", true, "", "Validity Checker Method");

    m_prob.AddValidityChecker(typename TmpProblemType::ValidityCheckerPointer(new SurfaceValidity<TmpTraitsType>(m_vcLabel)), "temp");
    m_prob.AddValidityChecker(typename TmpProblemType::ValidityCheckerPointer(new CollisionDetectionValidity<TmpTraitsType>(new Rapid())), "cd1");
    m_prob.SetMPProblem();
  }

template<class MPTraits>
bool 
SSSurfaceValidity<MPTraits>::IsValidImpl(CfgType& _cfg, Environment* _env, StatClass& _stats, CDInfo& _cdInfo, string* _callName){

  vector<TmpCfgType>& cfgs = _cfg.GetCfgs();
  for(typename vector<TmpCfgType>::iterator vecIter = cfgs.begin(); vecIter != cfgs.end(); ++vecIter){
    bool result = m_prob.GetValidityChecker("temp")->IsValid(*vecIter, _env, _stats, _cdInfo, _callName);
    if (!result){
      _cfg.SetLabel("VALID", false);
      return false;
    }
  }

  _cfg.SetLabel("VALID", true);
  return true;
}

#endif
#endif
