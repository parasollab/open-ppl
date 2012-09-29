#include "ObstacleClearanceValidity.h"
#include "Environment.h"
#include "CDInfo.h"


ObstacleClearanceValidity::ObstacleClearanceValidity(
    string _dmLabel, string _vcLabel, 
    double _clearance, bool _useBBX, 
    bool _cExact, size_t _clearanceRays, bool _positional) : 
  m_dmLabel(_dmLabel), m_vcLabel(_vcLabel), m_clearance(_clearance), 
  m_useBBX(_useBBX), m_cExact(_cExact), 
  m_clearanceRays(_clearanceRays), m_positional(_positional) { 
    m_name = "ObstacleClearance";
  }

ObstacleClearanceValidity::ObstacleClearanceValidity(XMLNodeReader& _node, MPProblem* _problem) 
  : ValidityCheckerMethod(_node, _problem) {
    m_name = "ObstacleClearance";
    m_dmLabel = _node.stringXMLParameter("dmMethod", true, "euclidean", "Distance Metric Method");
    m_vcLabel = _node.stringXMLParameter("vcMethod", true, "cd2", "Validity Checker Method");
    m_clearance = _node.numberXMLParameter("obstClearance", true, 0.05, 0.0, MAX_DBL, "Obstacle Validity Clearance");
    m_useBBX = _node.boolXMLParameter("useBBX", false, true, "Use the Bounding Box as an Obstacle");
    string cType = _node.stringXMLParameter("clearanceType", true, "exact", "Clearance Computation (exact or approx)");
    m_clearanceRays = _node.numberXMLParameter("clearanceRays", false, 20, 0, MAX_INT, "Num clearance rays for approx calculation");
    m_positional = _node.boolXMLParameter("positional", false, true, "Use only positional DOFs");

    m_cExact = (cType == "exact")?true:false;
  }

bool 
ObstacleClearanceValidity::IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
    CDInfo& _cdInfo, string* _callName) {

  shared_ptr<Boundary> bb = _env->GetBoundary();
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  CfgType cfg = _cfg;
  CfgType dummy;

  bool valid = CalculateCollisionInfo(GetMPProblem(), cfg, dummy, _env, bb,
      _stats, _cdInfo, m_vcLabel, m_dmLabel, m_cExact, m_clearanceRays, 
      0, m_useBBX, m_positional);

  if(m_debug){
    cout << "CFG::" << _cfg << endl;
    cout << "ClrCfg::" << dummy << endl;
    cout << "VALID::" << valid << endl;
    cout << "Dist::" << _cdInfo.m_minDist << endl;
  }

  if (!valid || _cdInfo.m_minDist < m_clearance){
    _cfg.SetLabel("VALID", false);
    return false;
  }
  else{
    _cfg.SetLabel("VALID", true);
    return true;
  }
}

