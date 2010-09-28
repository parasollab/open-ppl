#include "ClearanceFeature.h"

#include "MPStrategy.h"
#include "LocalPlanners.h"
#include "Roadmap.h"
#include "CfgTypes.h"
#include "Weight.h"
#include "MPRegion.h"
#include "CollisionDetection.h"

ClearanceFeature::ClearanceFeature():MPFeature(){}

ClearanceFeature::ClearanceFeature(XMLNodeReader& in_Node, MPProblem* in_pProblem):MPFeature(in_pProblem){
	ParseXML(in_Node);
}

void ClearanceFeature::ParseXML(XMLNodeReader& in_Node){
   SetLabel(in_Node.stringXMLParameter(string("Label"), true, string(""), string("Feature Value")));
   m_vc = in_Node.stringXMLParameter(string("vc_method"), true, string(""), string("CD Library"));
   in_Node.warnUnrequestedAttributes();
}

vector<double> ClearanceFeature::Collect(vector<VID>& vids) {
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
     string callee = "ClearanceFeature::Collect";
     vc->IsValid(cfg, env, *pStatClass, _cdInfo, true, &callee);
     clearance.push_back(_cdInfo.min_dist);
   }

   return clearance;
}
