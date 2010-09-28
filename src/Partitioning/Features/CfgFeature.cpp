#include "CfgFeature.h"

CfgFeature::CfgFeature():MPFeature(){
   m_index = 0;
}

CfgFeature::CfgFeature(XMLNodeReader& in_Node, MPProblem* in_pProblem):MPFeature(in_pProblem){
   ParseXML(in_Node);
}

void CfgFeature::ParseXML(XMLNodeReader& in_Node){
   SetLabel(in_Node.stringXMLParameter(string("Label"), true, string(""), string("Feature Value")));
   m_index = in_Node.numberXMLParameter(string("index"), true, 0, 0, MAX_INT, string("index"));
   in_Node.warnUnrequestedAttributes();
}

vector<double> CfgFeature::Collect(vector<VID>& vids){
   RoadmapGraph<CfgType, WeightType>* rdmp = GetMPProblem()->GetMPRegion(0)->GetRoadmap()->m_pRoadmap;
   vector<double> data;
   
   typedef vector<VID>::iterator VIT;
   for(VIT vit = vids.begin(); vit!=vids.end(); vit++){
      data.push_back(rdmp->find_vertex(*vit)->property().GetData()[m_index]);
   }

   return data;
}
