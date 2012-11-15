#include "ObstacleClearanceValidity.h"
#include "Environment.h"
#include "CDInfo.h"

ObstacleClearanceValidity::ObstacleClearanceValidity(const ClearanceParams& _cParams) :
  m_cParams(_cParams) {
  m_name = "ObstacleClearance";
}

ObstacleClearanceValidity::ObstacleClearanceValidity(XMLNodeReader& _node, MPProblem* _problem) :
  ValidityCheckerMethod(_node, _problem), m_cParams(_node, _problem) {
  m_name = "ObstacleClearance";
}

bool 
ObstacleClearanceValidity::IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
    CDInfo& _cdInfo, string* _callName) {

  shared_ptr<Boundary> bb = _env->GetBoundary();
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  CfgType cfg = _cfg;
  CfgType dummy;

  bool valid = CalculateCollisionInfo(cfg, dummy, _stats, _cdInfo, m_cParams, bb);

  if(m_debug){
    cout << "CFG::" << _cfg << endl;
    cout << "ClrCfg::" << dummy << endl;
    cout << "VALID::" << valid << endl;
    cout << "Dist::" << _cdInfo.m_minDist << endl;
  }

  if (!valid || _cdInfo.m_minDist < m_cParams.m_clearanceRays){
    _cfg.SetLabel("VALID", false);
    return false;
  }
  else{
    _cfg.SetLabel("VALID", true);
    return true;
  }
}

