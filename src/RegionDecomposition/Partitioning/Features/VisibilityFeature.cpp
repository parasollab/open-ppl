#include "VisibilityFeature.h"
#include "MPProblem.h"
#include "MPStrategy.h"
#include "LocalPlanners.h"
#include "DistanceMetrics.h"

VisibilityFeature::VisibilityFeature():MPFeature(){
}

VisibilityFeature::VisibilityFeature(int _k, string _nf, string _dm)  : MPFeature(), k(_k), nfLabel(_nf), dmLabel(_dm) {}

VisibilityFeature::VisibilityFeature(XMLNode& in_Node, MPProblem* mp):MPFeature(in_Node, mp){
   ParseXML(in_Node);
}

void VisibilityFeature::ParseXML(XMLNode& in_Node){
   //SetLabel(in_Node.Read(string("Label"), true, string(""), string("Feature Value")));
   k = in_Node.Read(string("k"), true, 0, 0, MAX_INT, string("k value"));
   nfLabel=in_Node.Read(string("nf_method"),true,string(""),string("Neighborhood Finder Method"));
   dmLabel=in_Node.Read(string("dm_method"),false,string("default"),string("Neighborhood Finder Method"));
   m_lpLabel = in_Node.Read(string("lp_method"),false,string("default"),string("Local Planner Method"));
   in_Node.warnUnrequestedAttributes();
}

vector<double> VisibilityFeature::Collect(vector<VID>& vids){
   RoadmapGraph<CfgType, WeightType>* rdmp = GetMPProblem()->GetRoadmap()->m_pRoadmap;
   typedef vector<VID>::iterator VIT;
   LocalPlanners<CfgType, WeightType>* lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
   StatClass* pStatClass = GetMPProblem()->GetStatClass();
   Environment *env = GetMPProblem()->GetEnvironment();
   NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
   LPOutput<CfgType, WeightType> lpOutput;

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

      nf->GetMethod(nfLabel)->KClosest(GetMPProblem()->GetRoadmap(), *vit, k, back_insert_iterator<vector<VID> >(kclosest));
      vector< pair<size_t,VID> > ccs;
      stapl::sequential::vector_property_map< RoadmapGraph<CfgType, WeightType>,size_t > cmap;
      get_cc_stats(*(rdmp),cmap,ccs);
      typedef vector<VID>::iterator IIT;
      for(IIT vit2 = kclosest.begin(); vit2!=kclosest.end(); vit2++){
         CfgType _col;
         if(is_same_cc(*rdmp, cmap, *vit, allVIDs[*vit2]))visibility+=1;
         else if(lp->GetMethod(m_lpLabel)->
                   IsConnected(env, *pStatClass,GetMPProblem()->GetDistanceMetric()->GetMethod(dmLabel),
                               rdmp->find_vertex(*vit)->property(), rdmp->find_vertex(allVIDs[*vit2])->property(),
                               _col, &lpOutput, .1, .1))
            visibility+=1;
      }
      visibility/=(double)kclosest.size();
      data.push_back(visibility);
   }

   return data;
}
