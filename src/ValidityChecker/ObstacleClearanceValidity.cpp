#include "ObstacleClearanceValidity.h"
#include "DistanceMetrics.h"
#include "MPProblem/MPRegion.h"

using namespace std;
  
  
  ObstacleClearanceValidity::ObstacleClearanceValidity() {
  m_dmLabel    = "euclidean";
  m_vcLabel    = "cd2";
  m_useBBX     = "true";
  m_cType      = "exact";
  m_clearance  = 0.1;
  m_positional = "true";
   }
  
ObstacleClearanceValidity::ObstacleClearanceValidity(string _dmLabel, string _vcLabel, bool _useBBX, 
  string _cType, double _clearance, bool _positional) : 
  m_dmLabel(_dmLabel), m_vcLabel(_vcLabel), m_useBBX(_useBBX), m_cType(_cType), 
  m_clearance(_clearance), m_positional(_positional) { }

ObstacleClearanceValidity::ObstacleClearanceValidity(XMLNodeReader& _node, MPProblem* _problem) 
	: ValidityCheckerMethod(_node, _problem) {
	  m_dmLabel    = _node.stringXMLParameter("dm_method", true, "euclidean", "Distance Metric Method");
	  m_vcLabel    = _node.stringXMLParameter("vc_method", true, "cd2", "Validity Checker Method");
		m_useBBX     = _node.boolXMLParameter("use_bbx", false, true, "Use the Bounding Box as an Obstacle");
		m_cType      = _node.stringXMLParameter("clearance_type", true, "exact", "Clearance Computation (exact or approx)");
		m_clearance  = _node.numberXMLParameter("obst_clearance", true, 0.1, 0.0, 100.0, "Obstacle Validity Clearance");
		m_positional = _node.boolXMLParameter("positional", false, true, "Use only positional DOFs");
  }

 bool ObstacleClearanceValidity::IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
     CDInfo& _cdInfo, bool _enablePenetration, string * _callName) {

   bool _cExact = (m_cType.compare("exact")==0)?true:false;

   MPProblem* mp = GetMPProblem();
   //shared_ptr<BoundingBox> bb = _env->GetBoundingBox();
   shared_ptr<Boundary> bb = _env->GetBoundary();
   CDInfo cdInfo;
   cdInfo.ResetVars();
   cdInfo.ret_all_info = true;

   CfgType cfg = _cfg;
   CfgType dummy;

   bool valid = CalculateCollisionInfo(mp, cfg, dummy, _env, bb, _stats, cdInfo, m_vcLabel, m_dmLabel, _cExact, m_clearance, 0, m_useBBX, m_positional);
   
   if(m_debug){
     cout << "CFG::" << _cfg << endl;
     cout << "ClrCfg::" << dummy << endl;
     cout << "VALID::" << valid << endl;
     cout << "Dist::" << cdInfo.min_dist << endl;
   }
   if (!valid || cdInfo.min_dist < m_clearance){
     _cfg.SetLabel("VALID", false);
     return false;
   }
   else{
     _cfg.SetLabel("VALID", true);
     return true;
   }

 }

