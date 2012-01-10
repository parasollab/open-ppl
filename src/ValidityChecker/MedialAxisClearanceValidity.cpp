#include "MedialAxisClearanceValidity.h"
#include "DistanceMetrics.h"
#include "MPProblem/MPRegion.h"

using namespace std;

MedialAxisClearanceValidity::MedialAxisClearanceValidity(string _dmLabel, string _vcLabel, bool _useBBX, 
  string _cType, string _pType, int _cRay, int _pRay, int _historyLen, double _epsilon, double _clearance) : 
  m_dmLabel(_dmLabel), m_vcLabel(_vcLabel), m_useBBX(_useBBX), m_cType(_cType), m_pType(_pType), m_cRay(_cRay), 
  m_pRay(_pRay), m_historyLen(_historyLen), m_epsilon(_epsilon), m_clearance(_clearance) { }

MedialAxisClearanceValidity::MedialAxisClearanceValidity(XMLNodeReader& _node, MPProblem* _problem) :
  ValidityCheckerMethod(_node, _problem) {

	  m_dmLabel    = _node.stringXMLParameter("dm_method", true, "", "Distance Metric Method");
	  m_vcLabel    = _node.stringXMLParameter("vc_method", true, "", "Validity Checker Method");
		m_useBBX     = _node.boolXMLParameter("use_bbx", false, true, "Use the Bounding Box as an Obstacle");
		m_cType      = _node.stringXMLParameter("clearance_type", true, "", "Clearance Computation (exact or approx)");
		m_pType      = _node.stringXMLParameter("penetration_type", true, "", "Penetration Computation (exact or approx)");
		m_cRay       = _node.numberXMLParameter("clearance_rays", false, 10, 1, 1000, "Clearance Number");
		m_pRay       = _node.numberXMLParameter("penetration_rays", false, 10, 1, 1000, "Penetration Number");
		m_historyLen = _node.numberXMLParameter("history_len", false, 5, 3, 1000, "History Length");
		m_epsilon    = _node.numberXMLParameter("epsilon", false, 0.1, 0.0, 100.0, "Epsilon-Close to the MA");
		m_clearance  = _node.numberXMLParameter("ma_clearance", false, 0.1, 0.0, 100.0, "Medial Axis Validity Clearance");
    m_history.clear();

  }

bool MedialAxisClearanceValidity::IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
  CDInfo& _cdInfo, bool _enablePenetration, string * _callName) {

    DistanceMetric::DistanceMetricPointer dm = GetMPProblem()->GetDistanceMetric()->GetDMMethod(m_dmLabel);
		bool _cExact = (m_cType.compare("exact")==0)?true:false;
		bool _pExact = (m_pType.compare("exact")==0)?true:false;

    CfgType origCfg,tmpCfg;
    origCfg = _cfg;
    tmpCfg = _cfg;
    PushToMedialAxis(GetMPProblem(),_env,tmpCfg,_stats,m_vcLabel,m_dmLabel,
                     _cExact,m_cRay,_pExact,m_pRay,m_useBBX,m_epsilon,m_historyLen,false);

    m_history.push_back( make_pair(origCfg,tmpCfg) );

    double gap = 0.0;
		for(int i=0; i<_cfg.DOF(); ++i) {
			if ( i < _cfg.PosDOF() ) gap += pow((tmpCfg.GetSingleParam(i)-_cfg.GetSingleParam(i)),2);
			else                    gap += pow((DirectedAngularDistance(tmpCfg.GetSingleParam(i),_cfg.GetSingleParam(i))),2);
		}

    //double dist = dm->Distance(_env, tmpCfg, _cfg);
    //bool result = dist < m_clearance;

    bool result = sqrt(gap) < m_clearance;
    _cfg.SetLabel("VALID", result);
    return result;

  }

vector< pair<CfgType,CfgType> > MedialAxisClearanceValidity::GetHistory() {
  return m_history;
}

void MedialAxisClearanceValidity::ClearHistory() {
  m_history.clear();
}
