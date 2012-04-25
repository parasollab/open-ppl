#ifndef CONSTRUCTREGIONMAP_H
#define CONSTRUCTREGIONMAP_H

#include "ParallelSBMPHeader.h"


typedef typename stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Region, WeightType> RRGraph;
typedef RoadmapGraph<CfgType, WeightType>::VID VID;

using namespace psbmp;
using namespace stapl;



/// General Framework for roadmap construction, calls MPStrategy method (i.e. BasicPRM, Basic RRT) directly
/// to be called if region migration is not needed
class ConstructRoadmap
{
  private:
    typedef MPRegion<CfgType,WeightType>  MPR_type;
    
    MPR_type* m_region;
    MPStrategyMethod* m_strategyMethod;
    int m_regionId;
  
  public:

  ConstructRoadmap(MPR_type* _mpr, MPStrategyMethod* _mpsm, int _id ):m_region(_mpr),m_strategyMethod(_mpsm),m_regionId(_id){ }
                

  void define_type(stapl::typer &_t){
        _t.member(m_region);
  }

  ConstructRoadmap(const ConstructRoadmap& _wf, std::size_t offset)  {}

  template<typename View, typename bbView>
  void operator()(const View& _view,  bbView _bbview) const 
  {
    int sample_size = _view.size()/_bbview.size();
    
    for(int i =0; i<_bbview.size(); ++i){
       int index = i + (_bbview.size()*stapl::get_location_id());
       cout << "Setting up BBX with index " << index << " with param = ";
       BoundingBox bb = _bbview[index];
       shared_ptr<BoundingBox> boundary = shared_ptr<BoundingBox>(new BoundingBox(bb));
       boundary->Print(cout);
       cout << "\n\n" << endl;
       m_strategyMethod->SetBoundary(boundary);
       //m_strategyMethod->SetBoundaryIndex(index);
       //add support to set num nodes in MPStrategy method 
       //m_strategyMethod->SetNumNodes(sample_size);
       m_strategyMethod->Run(m_regionId);
    }
  }
  
};
                                                                                                                                


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
  
  void define_type(stapl::typer &_t)  
  {
    _t.member(m_region);
  }
  
  template<typename regionView> 
  void operator()(regionView _view) const
  {
    
    vector<VID> regionVIDs = _view.property().RegionVIDs();
    //TODO: pass as string from strategy
    NCP ncp = m_nc->GetMethod("Closest");
    
    // temporary fix for Parallel code to compile
    stapl::sequential::vector_property_map< RoadmapGraph<CfgType, WeightType>::GRAPH,size_t > cmap;
    ncp->Connect( m_region->GetRoadmap(),*(m_region->GetStatClass()), cmap, 
	         regionVIDs.begin(), regionVIDs.end(), regionVIDs.begin(), regionVIDs.end() );
  }
        
};





#endif

