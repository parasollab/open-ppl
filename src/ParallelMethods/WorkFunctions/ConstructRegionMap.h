#ifndef CONSTRUCTADAPTIVEREGION_H
#define CONSTRUCTADAPTIVEREGION_H

#include "ParallelSBMPHeader.h"


typedef typename stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Region, WeightType> RRGraph;
typedef RoadmapGraph<CfgType, WeightType>::VID VID;

using namespace psbmp;
using namespace stapl;


class NodeGenerator 
{
  private:
  typedef Sampler<CfgType>::SamplerPointer NGM;
  typedef MPRegion<CfgType,WeightType>  MPR;
  typedef shared_ptr<ValidityCheckerMethod> VCM;
  
  
  NGM m_sp;
  MPR* m_region;
  Environment* m_env;
  VCM m_vcm;
  int m_attempts;

  public:

  NodeGenerator(MPR* _mpr, Environment* _env,NGM _ngm, VCM _vcm, int _num):
                m_attempts(_num){
    m_sp = _ngm;
    m_region = _mpr;
    m_env = _env;
    m_vcm = _vcm;
  }
  
  void define_type(stapl::typer &t)  
  {
    t.member(m_region);
  }
  
  template <typename BBView, typename RGView>
  void operator()(BBView _v1, RGView _v2) const
  {
    
    
      vector<CfgType> outNodes, colNodes;
      vector<VID> regionVIDs;
      CDInfo cdInfo;
      string callee("Generator");
      BoundingBox bb = _v1;
      shared_ptr<BoundingBox> boundary = shared_ptr<BoundingBox>(new BoundingBox(bb));
      Environment* env = const_cast<Environment*>(m_env);
      //PrintValue("v2 desc:", _v2.descriptor());
      //boundary->testPrint(2);
      ////Generate Node
      m_sp->Sample(env, boundary, *(m_region->GetStatClass()),m_attempts,10, 
	         back_inserter(outNodes),back_inserter(colNodes));
    
    
      typedef vector<CfgType>::iterator VIT;
      for(VIT vit = outNodes.begin(); vit  != outNodes.end(); ++vit) {
        CfgType tmp = *vit;
        ///Add Valid Node Only
        if(m_vcm->IsValid(tmp, env, *(m_region->GetStatClass()),
                         cdInfo, true, &callee)) {
        VID vid = m_region->GetRoadmap()->m_pRoadmap->add_vertex(tmp);
        regionVIDs.push_back(vid);
        }
      
      }
      Region bbInfo(boundary,regionVIDs);
      
      _v2.property() = bbInfo;
   
    
  }
        
	
};

class NodeConnector 
{
  private:
  typedef Connector<CfgType, WeightType> NC;
  typedef NC::ConnectionPointer NCP;
  typedef MPRegion<CfgType,WeightType>  MPR;
  
  NC* m_nc;
  MPR* m_region;
 
  public:

  NodeConnector(MPR* _mpr, NC* _nc) {
    m_nc = _nc;
    m_region = _mpr;
  }
  
  void define_type(stapl::typer &t)  
  {
    t.member(m_region);
  }
  
  template<typename regionView> 
  void operator()(regionView _view) const
  {
    
    vector<VID> regionVIDs = _view.property().RegionVIDs();
    //TODO: pass as string from strategy
    NCP ncp = m_nc->GetMethod("Closest");
    
    m_nc->Connect(ncp, m_region->GetRoadmap(),*(m_region->GetStatClass()), 
	         regionVIDs.begin(), regionVIDs.end());
  }
        
};


class RegionConnector 
{
  private:
  typedef Connector<CfgType, WeightType> NC;
  typedef NC::ConnectionPointer NCP;
  typedef MPRegion<CfgType,WeightType>  MPR;
  
  NC* m_nc;
  MPR* m_region;
  int m_k;
  RRGraph* m_g;
 
  public:

  RegionConnector(MPR* _mpr, RRGraph* _g, NC* _nc, int _k) :m_k(_k),m_g(_g){
    m_nc = _nc;
    m_region = _mpr;
  }
  
  void define_type(stapl::typer &t)  
  {
    t.member(m_region);
  }
  
  template<typename regionView> 
  void operator()(regionView _view) const
  {
    NCP ncp = m_nc->GetMethod("Closest");
    vector<VID> sVids = _view.property().RegionVIDs();
    random_shuffle(sVids.begin(), sVids.end());
    vector<VID> sCand;
    sCand.resize(m_k);
    //TO DO: rewrite copy to optimize space complexity
    PrintValue("source size", sVids.size());
    copy(sVids.begin(), sVids.begin() + std::min(static_cast<int>(sVids.size()), m_k), sCand.begin());
    typedef typename regionView::adj_edges_type ADJV;
    ADJV  edges = _view.edges();
    for(typename regionView::adj_edge_iterator ei = edges.begin(); ei != edges.end(); ++ei){
      Region tBBI = (*(m_g->find_vertex((*ei).target()))).property();
      vector<VID> tVids = tBBI.RegionVIDs();
      random_shuffle(tVids.begin(), tVids.end());
      PrintValue("target size", tVids.size());
      vector<VID> tCand;
      tCand.resize(m_k);
      copy(tVids.begin(), tVids.begin() + std::min(static_cast<int>(tVids.size()), m_k), tCand.begin());
      m_nc->Connect(ncp, m_region->GetRoadmap(),*(m_region->GetStatClass()), 
	         sCand.begin(),sCand.end(),
	         tCand.begin(),tCand.end());
      
   }
	   
  }
        
};




#endif

