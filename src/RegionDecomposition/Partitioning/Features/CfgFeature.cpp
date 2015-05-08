#include "CfgFeature.h"
#include "MPProblem.h"

CfgFeature::CfgFeature():MPFeature(){
   m_index = 0;
}

CfgFeature::CfgFeature(int index = 0) : MPFeature(), m_index(index) {}

CfgFeature::CfgFeature(XMLNode& in_Node, MPProblem* in_pProblem):MPFeature(in_Node, in_pProblem){
   ParseXML(in_Node);
}

void CfgFeature::ParseXML(XMLNode& in_Node){
   m_index = in_Node.Read("index", true, 0, 0, MAX_INT, "index");
   in_Node.warnUnrequestedAttributes();
}

vector<double> CfgFeature::Collect(vector<VID>& vids){
   RoadmapGraph<CfgType, WeightType>* rdmp = GetMPProblem()->GetRoadmap()->m_pRoadmap;
   vector<double> data;

   typedef vector<VID>::iterator VIT;
   for(VIT vit = vids.begin(); vit!=vids.end(); vit++){
      data.push_back(rdmp->find_vertex(*vit)->property().GetData()[m_index]);
   }

   return data;
}
