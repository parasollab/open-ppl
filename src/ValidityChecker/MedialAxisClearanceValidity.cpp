#include "MedialAxisClearanceValidity.h"

#include "DistanceMetrics.h"
#include "MPProblem.h"

MedialAxisClearanceValidity::MedialAxisClearanceValidity(const ClearanceParams& _cParams,
  int _historyLen, double _epsilon, double _clearance):
  ValidityCheckerMethod(), m_cParams(_cParams),
  m_historyLength(_historyLen), m_epsilon(_epsilon), m_clearance(_clearance){
    m_name = "MedialAxisClearance";
    m_cParams.SetDebug(this->m_debug);
  }

MedialAxisClearanceValidity::MedialAxisClearanceValidity(XMLNodeReader& _node, MPProblem* _problem) :
  ValidityCheckerMethod(_node, _problem),m_cParams(_node,_problem,"MedialAxisClearanceValidity",this->m_debug){
    m_history.clear();
  }
void MedialAxisClearanceValidity::ParseXML(XMLNodeReader& _node){
  m_epsilon = _node.numberXMLParameter("epsilon", false, 0.1, 0.0, 1.0, "Epsilon-Close to the MA (fraction of the resolution)");
  m_historyLength = _node.numberXMLParameter("historyLength", false,5,3,100,"History Length");
  m_clearance  = _node.numberXMLParameter("ma_clearance", false, 0.1, 0.0, 100.0, "Medial Axis Validity Clearance");
  _node.warnUnrequestedAttributes();
}
bool 
MedialAxisClearanceValidity::IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
    CDInfo& _cdInfo, string * _callName) {
  CfgType origCfg = _cfg;
  CfgType tmpCfg = _cfg;
  if ( !PushToMedialAxis(tmpCfg,_stats,m_cParams,m_epsilon,m_historyLength) ) {
    _cfg.SetLabel("VALID", false);
    return false;
  }

  m_history.push_back( make_pair(origCfg,tmpCfg) );

  double dist = GetMPProblem()->GetDistanceMetric()->GetMethod(m_cParams.m_dmLabel)->Distance(_env, tmpCfg, _cfg);
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
