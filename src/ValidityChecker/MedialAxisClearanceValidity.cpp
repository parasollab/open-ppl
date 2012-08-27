#include "MedialAxisClearanceValidity.h"
#include "DistanceMetrics.h"
#include "MPProblem.h"

using namespace std;

MedialAxisClearanceValidity::MedialAxisClearanceValidity(string _dmLabel, string _vcLabel, bool _useBBX, 
    bool _cExact, bool _pExact, int _cRay, int _pRay, int _historyLen, double _epsilon, double _clearance) : 
  m_dmLabel(_dmLabel), m_vcLabel(_vcLabel), m_useBBX(_useBBX), m_cExact(_cExact), m_pExact(_pExact), m_cRay(_cRay), 
  m_pRay(_pRay), m_historyLen(_historyLen), m_epsilon(_epsilon), m_clearance(_clearance) { 
  }

MedialAxisClearanceValidity::MedialAxisClearanceValidity(XMLNodeReader& _node, MPProblem* _problem) :
  ValidityCheckerMethod(_node, _problem) {

    m_dmLabel    = _node.stringXMLParameter("dm_method", true, "", "Distance Metric Method");
    m_vcLabel    = _node.stringXMLParameter("vc_method", true, "", "Validity Checker Method");
    m_useBBX     = _node.boolXMLParameter("use_bbx", false, true, "Use the Bounding Box as an Obstacle");
    string _cType = _node.stringXMLParameter("clearance_type", true, "", "Clearance Computation (exact or approx)");
    string _pType = _node.stringXMLParameter("penetration_type", true, "", "Penetration Computation (exact or approx)");
    m_cRay       = _node.numberXMLParameter("clearance_rays", false, 10, 1, 1000, "Clearance Number");
    m_pRay       = _node.numberXMLParameter("penetration_rays", false, 10, 1, 1000, "Penetration Number");
    m_historyLen = _node.numberXMLParameter("history_len", false, 5, 3, 1000, "History Length");
    m_epsilon    = _node.numberXMLParameter("epsilon", false, 0.1, 0.0, 100.0, "Epsilon-Close to the MA");
    m_clearance  = _node.numberXMLParameter("ma_clearance", false, 0.1, 0.0, 100.0, "Medial Axis Validity Clearance");
    m_positional = _node.boolXMLParameter("positional", false, true, "Use only positional DOFs");
    m_history.clear();
    m_cExact = (_cType.compare("exact")==0)?true:false;
    m_pExact = (_pType.compare("exact")==0)?true:false;

  }

bool 
MedialAxisClearanceValidity::IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
    CDInfo& _cdInfo, bool _enablePenetration, string * _callName) {

  CfgType origCfg,tmpCfg;
  origCfg = _cfg;
  tmpCfg = _cfg;
  if ( !PushToMedialAxis(GetMPProblem(),_env,tmpCfg,_stats,m_vcLabel,m_dmLabel,m_cExact,
        m_cRay,m_pExact,m_pRay,m_useBBX,m_epsilon,m_historyLen,this->m_debug,m_positional) ) {
    _cfg.SetLabel("VALID", false);
    return false;
  }

  m_history.push_back( make_pair(origCfg,tmpCfg) );

  double dist = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dmLabel)->Distance(_env, tmpCfg, _cfg);
  bool result = dist < m_clearance;
  _cfg.SetLabel("VALID", result);
  return result;
}

vector< pair<CfgType,CfgType> > 
MedialAxisClearanceValidity::GetHistory() {
  return m_history; 
}

void 
MedialAxisClearanceValidity::ClearHistory() { 
  m_history.clear(); 
}
