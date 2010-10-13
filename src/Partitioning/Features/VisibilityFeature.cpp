#include "VisibilityFeature.h"

#include "MPProblem.h"
#include "MPRegion.h"
#include "MPStrategy.h"
#include "LocalPlanners.h"
#include "RoadmapGraph.h"

VisibilityFeature::VisibilityFeature():MPFeature(NULL){
}

VisibilityFeature::VisibilityFeature(XMLNodeReader& in_Node, MPProblem* mp):MPFeature(mp){
   ParseXML(in_Node);
}

void VisibilityFeature::ParseXML(XMLNodeReader& in_Node){
   SetLabel(in_Node.stringXMLParameter(string("Label"), true, string(""), string("Feature Value")));
   k = in_Node.numberXMLParameter(string("k"), true, 0, 0, MAX_INT, string("k value"));
   in_Node.warnUnrequestedAttributes();
   nfLabel=in_Node.stringXMLParameter(string("nf_method"),true,string(""),string("Neighborhood Finder Method"));
   dmLabel=in_Node.stringXMLParameter(string("dm_method"),false,string("default"),string("Neighborhood Finder Method"));
}

vector<double> VisibilityFeature::Collect(vector<VID>& vids){
   RoadmapGraph<CfgType, WeightType>* rdmp = GetMPProblem()->GetMPRegion(0)->GetRoadmap()->m_pRoadmap;
   typedef vector<VID>::iterator VIT;
   LocalPlanners<CfgType, WeightType>* sl=GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
   Stat_Class* pStatClass = GetMPProblem()->GetMPRegion(0)->GetStatClass();
   Environment *env = GetMPProblem()->GetEnvironment();
   NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
   LPOutput<CfgType, WeightType> lp;

   vector<CfgType> cfgs;
   vector<VID> allVIDs;
   rdmp->GetVerticesVID(allVIDs);
   rdmp->GetVerticesData(cfgs);

   typedef vector<VID>::iterator VIT;
   
   vector<double> data;

   for(VIT vit = vids.begin(); vit!=vids.end(); vit++){
      double visibility=0;
      vector<VID> kclosest;
      
      CfgType cfg = rdmp->find_vertex(*vit)->property();
      
      nf->KClosest(nf->GetNFMethod(nfLabel), GetMPProblem()->GetMPRegion(0)->GetRoadmap(), *vit, k, back_insert_iterator<vector<VID> >(kclosest));
      vector< pair<size_t,VID> > ccs;
      stapl::vector_property_map< RoadmapGraph<CfgType, WeightType>,size_t > cmap;
      get_cc_stats(*(rdmp),cmap,ccs);
      typedef vector<VID>::iterator IIT;
      for(IIT vit2 = kclosest.begin(); vit2!=kclosest.end(); vit2++){
         if(is_same_cc(*rdmp, cmap, *vit, allVIDs[*vit2]))visibility+=1;
         else if(sl->IsConnected(env, *pStatClass, GetMPProblem()->GetDistanceMetric()->GetDMMethod(dmLabel), rdmp->find_vertex(*vit)->property(), rdmp->find_vertex(allVIDs[*vit2])->property(),&lp, .1, .1))
            visibility+=1;
      }
      visibility/=(double)kclosest.size();
      data.push_back(visibility);
   }
   
   return data;
}
