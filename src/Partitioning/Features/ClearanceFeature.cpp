#include "ClearanceFeature.h"
#include "MPProblem.h"
#include "Roadmap.h"
#include "CfgTypes.h"
#include "MPRegion.h"
#include "ValidityChecker.hpp"
#include "MPUtils.h"

ClearanceFeature::ClearanceFeature():MPFeature(){}

ClearanceFeature::ClearanceFeature(string _vc):MPFeature(), m_vc(_vc) {}

ClearanceFeature::ClearanceFeature(XMLNodeReader& in_Node, MPProblem* in_pProblem):MPFeature(in_Node, in_pProblem){
  ParseXML(in_Node);
}

void ClearanceFeature::ParseXML(XMLNodeReader& in_Node){
  m_vc = in_Node.stringXMLParameter("vc_method", true, "", "CD Library");
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

CSpaceClearanceFeature::CSpaceClearanceFeature():MPFeature(){}

CSpaceClearanceFeature::CSpaceClearanceFeature(string vc, string dm) : MPFeature(), m_vc(vc), m_dm(dm) {};

CSpaceClearanceFeature::CSpaceClearanceFeature(XMLNodeReader& in_Node, MPProblem* in_pProblem):MPFeature(in_Node, in_pProblem){
  ParseXML(in_Node);
}

void CSpaceClearanceFeature::ParseXML(XMLNodeReader& in_Node){
  m_vc = in_Node.stringXMLParameter("vc_method", true, "", "CD Library");
  m_dm = in_Node.stringXMLParameter("distance_metric", true, "", "Distance Metric");
  in_Node.warnUnrequestedAttributes();
}

vector<double> CSpaceClearanceFeature::Collect(vector<VID>& vids) {

  vector<double> clearance;
	CDInfo tmp_info;
  RoadmapGraph<CfgType, WeightType>* rdmp = GetMPProblem()->GetMPRegion(0)->GetRoadmap()->m_pRoadmap;
  Stat_Class* pStatClass = GetMPProblem()->GetMPRegion(0)->GetStatClass();
  Environment *env = GetMPProblem()->GetEnvironment();
  ValidityChecker<CfgType>::VCMethodPtr vc=GetMPProblem()->GetValidityChecker()->GetVCMethod(m_vc);
  typedef vector<VID>::iterator VIT;

  for(VIT vit = vids.begin(); vit!=vids.end(); vit++){
    CDInfo _cdInfo;
    _cdInfo.ret_all_info=true;
    CfgType cfg=rdmp->find_vertex(*vit)->property();
		GetApproxCollisionInfo(GetMPProblem(),cfg,env,*pStatClass,_cdInfo,m_vc,m_dm,20,20,true);
		clearance.push_back(_cdInfo.min_dist);
  }
  return clearance;
}

