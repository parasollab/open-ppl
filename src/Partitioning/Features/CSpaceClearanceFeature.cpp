#include "CSpaceClearanceFeature.h"

#include "MPStrategy.h"
#include "LocalPlanners.h"
#include "Roadmap.h"
#include "CfgTypes.h"
#include "Weight.h"
#include "MPRegion.h"
#include "CollisionDetection.h"

CSpaceClearanceFeature::CSpaceClearanceFeature():MPFeature(){}

CSpaceClearanceFeature::CSpaceClearanceFeature(XMLNodeReader& in_Node, MPProblem* in_pProblem):MPFeature(in_pProblem){
	ParseXML(in_Node);
}

void CSpaceClearanceFeature::ParseXML(XMLNodeReader& in_Node){
   SetLabel(in_Node.stringXMLParameter(string("Label"), true, string(""), string("Feature Value")));
   m_vc = in_Node.stringXMLParameter(string("vc_method"), true, string(""), string("CD Library"));
   m_dm = in_Node.stringXMLParameter(string("distance_metric"), true, string(""), string("Distance Metric"));
   in_Node.warnUnrequestedAttributes();
}


vector<double> CSpaceClearanceFeature::Collect(vector<VID>& vids) {
   vector<double> clearance;
   
   RoadmapGraph<CfgType, WeightType>* rdmp = GetMPProblem()->GetMPRegion(0)->GetRoadmap()->m_pRoadmap;
   Stat_Class* pStatClass = GetMPProblem()->GetMPRegion(0)->GetStatClass();
   Environment *env = GetMPProblem()->GetEnvironment();
   ValidityChecker<CfgType>::VCMethodPtr vc=GetMPProblem()->GetValidityChecker()->GetVCMethod(m_vc);
   
   typedef vector<VID>::iterator VIT;
   for(VIT vit = vids.begin(); vit!=vids.end(); vit++){
     CDInfo _cdInfo;
     _cdInfo.ret_all_info=true;
     CfgType cfg=rdmp->find_vertex(*vit)->property();
     clearance.push_back(cfg.ApproxCSpaceClearance(GetMPProblem(), env, *pStatClass, m_vc, _cdInfo, m_dm, 20, true));
   

   return clearance;
   }
}
