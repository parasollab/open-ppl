/////////////////////////////
//HEADER ConstructRegionMap.h
/////////////////////////////
#ifndef CONSTRUCTREGIONMAP_H_
#define CONSTRUCTREGIONMAP_H_

#include "ParallelMethods/ParallelSBMPHeader.h"
#include "MPProblem/BoundingBox.h"

//typedef typename stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Region<BoundingBox>, WeightType> RRGraph;
//typedef RoadmapGraph<CfgType, WeightType>::VID VID;

using namespace psbmp;
using namespace stapl;


/// General Framework for roadmap construction, calls MPStrategy method (i.e. BasicPRM, Basic RRT) directly
/// to be called if region migration is not needed
template<class MPTraits>
class ConstructRoadmap {

private:
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::VID VID;
  typedef typename MPProblemType::MPStrategyPointer MPStrategyPointer;

  MPStrategyPointer m_strategyMethod;

public:
  ConstructRoadmap(MPStrategyPointer _mpsm ): m_strategyMethod(_mpsm){ }    
  void define_type(stapl::typer& _t){
  }
  ConstructRoadmap(const ConstructRoadmap& _wf, std::size_t offset)  {}
  
  template<typename View, typename bbView>
  void operator()(View _view,  bbView _bbview) const {
    //int sample_size = _view.size()/_bbview.size();
    
    // for(int i =0; i<_bbview.size(); ++i){
    //int index = i + (_bbview.size()*stapl::get_location_id());
    // cout << "Setting up BBX with index " << index << " with param = ";
    // BoundingBox bb = _bbview[index];
    BoundingBox bb = _bbview;
    shared_ptr<BoundingBox> boundary = shared_ptr<BoundingBox>(new BoundingBox(bb));
    vector<VID> dummy;
    Region<BoundingBox, MPTraits> bbInfo(boundary,dummy);
    _view.property() = bbInfo;
    boundary->Print(cout);
    cout << "\n\n" << endl;
    m_strategyMethod->SetBoundary(boundary);
    //m_strategyMethod->SetBoundaryIndex(index);
    //add support to set num nodes in MPStrategy method 
    //m_strategyMethod->SetNumNodes(sample_size);
    m_strategyMethod->Run();
    //}
  }
};

template<class MPTraits>
class NodeGenerator {

private:
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::VID VID;
  typedef typename MPProblemType::SamplerPointer NGM;
  //typedef typename MPProblemType::ValidityCheckerPointer VCM;
  
  MPProblemType* m_problem;
  NGM m_sp;
  //VCM m_vcm;
  int m_attempts;

public:

  NodeGenerator(MPProblemType* _problem, NGM _ngm, int _num):
    m_attempts(_num){
    m_problem = _problem;
    m_sp = _ngm;
   // m_vcm = _vcm;
  
  }
  
  void define_type(stapl::typer& _t) {
  }
  
  template <typename BBView, typename RGView>
  void operator()(BBView _v1, RGView _v2) const {

    vector<CfgType> outNodes, colNodes;
    vector<VID> regionVIDs;
    CDInfo cdInfo;
    string callee("Generator");
    BoundingBox bb = _v1;
    shared_ptr<BoundingBox> boundary = shared_ptr<BoundingBox>(new BoundingBox(bb));
    //PrintValue("v2 desc:", _v2.descriptor());
    //boundary->testPrint(2);
    ////Generate Node
    
    
    StatClass* stat = m_problem->GetStatClass();
    m_sp->Sample(m_problem->GetEnvironment(), boundary, *stat, m_attempts, 10, back_inserter(outNodes), back_inserter(colNodes));
    
    typedef typename vector<CfgType>::iterator VIT;
    for(VIT vit = outNodes.begin(); vit  != outNodes.end(); ++vit) {
      
      CfgType tmp = *vit;
      ///Add Valid Node Only
      //TODO: Pass validity checker label as string
      if(m_problem->GetValidityChecker("cd1")->IsValid(tmp, m_problem->GetEnvironment(), *stat,
        cdInfo, &callee)) {
        
        VID vid = m_problem->GetRoadmap()->GetGraph()->add_vertex(tmp);
        regionVIDs.push_back(vid); 
      }
    }
    Region<BoundingBox, MPTraits> bbInfo(boundary,regionVIDs);   
    _v2.property() = bbInfo;
  }
};

template<class MPTraits>
class NodeConnector {
private:
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::VID VID;
  typedef typename MPProblemType::GraphType GraphType;
  typedef typename MPProblemType::ConnectorPointer NCP;
 
  MPProblemType* m_problem;
  NCP m_ncp;

public:

  NodeConnector(MPProblemType* _problem, NCP _ncp) {
    m_problem = _problem;
    m_ncp = _ncp;
  }
  
  void define_type(stapl::typer& _t) {
  }
  
  template<typename regionView> 
  void operator()(regionView _view) const {

    vector<VID> regionVIDs = _view.property().RegionVIDs();
   
    //PrintValue("CONNECTOR-vid size:  ", regionVIDs.size());
    for(typename vector<VID>::iterator vit = regionVIDs.begin(); vit != regionVIDs.end(); ++vit){
      //PrintValue("CONNECTOR VID = " , *vit);
    }
    
    StatClass* stat = m_problem->GetStatClass();
    
    // temporary fix for Parallel code to compile
    stapl::sequential::vector_property_map<typename GraphType::GRAPH,size_t > cmap;
    m_ncp->Connect(m_problem->GetRoadmap(), *stat, cmap, 
	         regionVIDs.begin(), regionVIDs.end(), regionVIDs.begin(), regionVIDs.end() );
  }    
};

/*TO DO- combine with similar class
class ConstructBioRoadmap {
private:
    MPStrategyMethod* m_strategyMethod;
  
public:

  ConstructBioRoadmap(MPStrategyMethod* _mpsm) {  
    m_strategyMethod = _mpsm;
  }
                
  void define_type(stapl::typer& _t) {
  }
  
  template<typename View>
  void operator()(View _view) const {
       m_strategyMethod->Run();
  }
};


//TODO :: Find a way to combine with similar class
class BioGenerator {
private:
  typedef Sampler<CfgType>::SamplerPointer NGM_type;

  NGM_type m_pNodeGen;
  Environment* m_env;
  int m_attempts;

public:
  BioGenerator(NGM_type _ngm, Environment* _env, int _att) {
    m_pNodeGen = _ngm;
    m_env = _env;
    m_attempts = _att;
  }

  void define_type(stapl::typer& _t) {
  }

  template<typename View> 
  void operator()(const View& _view) const {
    int num_nodes = _view.size();
    vector<CfgType> outNodes;
    vector<CfgType> inNodes(num_nodes);
    
    Environment* _env = const_cast<Environment*>(m_env);
    StatClass* stat = _env->GetMPProblem()->GetStatClass();
	
    m_pNodeGen->Sample(_env, *stat,
      inNodes.begin(),inNodes.end(), m_attempts, back_inserter(outNodes));

    size_t j(0);
    typedef vector<CfgType>::iterator VIT;
    for(VIT vit = outNodes.begin(); vit  != outNodes.end(); ++vit, j++) {
      CfgType tmp = *vit;
      _env->GetMPProblem()->GetRoadmap()->m_pRoadmap->add_vertex(tmp);
      //PrintValue("#contact", tmp.contacts(_env));
    }
  }
};


////TODO - Move to region class
class BioRegion {
 public:
  typedef pair<int,int> region_range_type;
  typedef vector<size_t> region_data_type;
 protected:
  region_range_type   m_range;
  region_data_type m_data;
  double m_step;
  vector<pair<RVID, size_t> > m_ccs;

 public:
  BioRegion(double _step_size = 999) : m_step(_step_size) { }
  BioRegion(region_data_type& _data) : m_data(_data) { }
  BioRegion(const BioRegion& _other) 
    : m_range(_other.m_range), m_data(_other.m_data), 
      m_step(_other.m_step),m_ccs(_other.m_ccs){ }

  region_range_type GetRange() { return m_range; }
  void SetRange(const region_range_type& _r) { m_range = _r; }
  bool InRange(int _contact){
    if (_contact >= m_range.first && _contact < m_range.second)
      return true;
    else
      return false;
  }
  int RangeID(double _contact){
    return floor(_contact/m_step);
  }
  region_data_type RegionVIDs() { return m_data; }
  void SetCCs(std::vector<pair<VID, size_t> > _ccs){m_ccs = _ccs;}
  vector<pair<VID, size_t> > GetCCs() { return m_ccs; }
  void SetData(const size_t _vid) { m_data.push_back(_vid); }
  void define_type(stapl::typer& _t) { 
    _t.member(m_range); 
    _t.member(m_data);
    _t.member(m_ccs);
  }
};

namespace stapl {
template <typename Accessor>
class proxy<BioRegion, Accessor> 
  : public Accessor {
  
  private:
    friend class proxy_core_access;
    typedef BioRegion target_t;

  public:
    typedef pair<int, int> region_range_type;
    typedef vector<size_t> region_data_type;

    explicit proxy(Accessor const& _acc) : Accessor(_acc) { }
    operator target_t() const { return Accessor::read(); }
    proxy const& operator=(proxy const& _rhs) { Accessor::write(_rhs); return *this; }
    proxy const& operator=(target_t const& _rhs) { Accessor::write(_rhs); return *this;}

    region_range_type GetRange() { return Accessor::invoke(&target_t::GetRange); }
    void SetRange(const region_range_type& _r) { Accessor::invoke(&target_t::SetRange, _r); }
    bool InRange(int _contact) { return Accessor::invoke(&target_t::InRange, _contact); }
    int RangeID(double _contact) { return Accessor::invoke(&target_t::RangeID, _contact); }
    region_data_type RegionVIDs() { return Accessor::invoke(&target_t::RegionVIDs); }
    void SetData(const size_t _vid) { Accessor::invoke(&target_t::SetData, _vid); }
    vector<pair<VID, size_t> > GetCCs() {return Accessor::invoke(&target_t::GetCCs); }
    void SetCCs(std::vector<pair<VID, size_t> > _ccs) { Accessor::invoke(&target_t::SetCCs, _ccs); }
  }; //struct proxy
}


//TODO :: Find a way to combine with similar class
class BioRegionSampler {
  
private:
  typedef Sampler<CfgType>::SamplerPointer NGM_type;

  NGM_type m_pNodeGen;
  int m_num;
  MPProblem* m_problem;
  
public:
  BioRegionSampler(NGM_type _ngm, MPProblem* _problem, int _num) {
    m_pNodeGen = _ngm;
    m_problem = _problem;
    m_num = _num;
  }
  void define_type(stapl::typer& _t) {
  }

  template<typename View> 
  void operator()(View _view) const {
    vector<CfgType> outNodes;
    vector<CfgType> inNodes(m_num);
    vector<VID>  regionVIDs;
    regionVIDs.clear();
    
    StatClass* stat = m_problem->GetStatClass();
    Environment* env = m_problem->GetEnvironment();

    m_pNodeGen->Sample(env, *stat,inNodes.begin(),inNodes.end(), m_num, back_inserter(outNodes));
	
    //size_t j(0);
    PrintValue("BioRegionSampler outNode size : " , outNodes.size());
    typedef vector<CfgType>::iterator VIT;
    for(VIT vit = outNodes.begin(); vit  != outNodes.end(); ++vit) {
      CfgType tmp = *vit;
      VID vid = m_problem->GetRoadmap()->m_pRoadmap->add_vertex(tmp);
      //PrintValue("BioSampler vid: ", vid);
      regionVIDs.push_back(vid);
    }
    PrintValue("BioRegionSampler regionVID size : " , regionVIDs.size());
    BioRegion regionData(regionVIDs);
    _view.property() = regionData;
  }
};*/

#endif
