//////////////////////////////
//HEADER ParallelPRMStrategy.h
/////////////////////////////
#ifndef PARALLELPRMSTRATEGY_H_
#define PARALLELPRMSTRATEGY_H_

#include "Roadmap.h"

#include "MetricUtils.h"
#include "CollisionDetection.h"
#include "Connector.h"
#include "LocalPlanners.h"
#include "Sampler.h"
#include "MPProblem.h"
#include "MPStrategy/MPStrategyMethod.h"
#include "runtime.h"
#include "runtime/counter/default_counters.h"
#include "pRange.h"
#include "p_array.h"
#include "views/view_base.hpp"
#include "views/balance_view.hpp"
#include "RoadmapGraph.h"
#include "pContainers/pgraph/view/p_graph_view.h"
#include "views/native_view.hpp"
#include "views/replicated_view.hpp"

using namespace std;
typedef stapl::p_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType, WeightType> PGRAPH;
typedef PGRAPH::vertex_descriptor VID; 
typedef PGRAPH::vertex_iterator VI;

class SampleWF {
private:
  typedef Sampler<CfgType>::SamplerPointer NGM_type;
  typedef MPRegion<CfgType,WeightType>  MPR_type;
	
  NGM_type m_nodeGen;
  MPR_type* m_region;
  Environment* m_env;
	
public:
  SampleWF(NGM_type _ngm, MPR_type* _mpr, Environment* _penv) {
    m_nodeGen = _ngm;
    m_region = _mpr;
    m_env = _penv;
		
  }
  void define_type(stapl::typer& t) {
    t.member(m_region);
  }

  Sample_WF(const SampleWF& _wf, std::size_t _offset)  {} 

  template<typename View> 
  void operator()(const View& _view) const {
		
    int num_nodes = _view.size();
    vector<CfgType> outNodes;
    vector<CfgType> inNodes(num_nodes);
    LOG_DEBUG_MSG("ParallelPRMStrategy::SampleWF- view.size= " << _view.size());
		
    Environment* _env = const_cast<Environment*>(m_env);
		
    m_nodeGen->Sample(_env,*(m_region->GetStatClass()),inNodes.begin(),inNodes.end(), 100, 
    back_inserter(outNodes));
		
    size_t j(0);
    typedef vector<CfgType>::iterator VIT;
    for(VIT vit = outNodes.begin(); vit  != outNodes.end(); ++vit, j++) {			
      CfgType tmp = *vit;
      VID NEW = m_region->GetRoadmap()->m_pRoadmap->add_vertex(tmp);		
    }		
  }
};

class ConnectWF {
private:
  typedef MPRegion<CfgType,WeightType>  MPR_type;
  typedef Connector<CfgType, WeightType>::ConnectionPointer NCM_type;
  typedef LocalPlanners<CfgType,WeightType> LP_type;
  MPR_type* m_region;
  NCM_type m_pNodeCon;
  LP_type*  m_lp;
	
public:
  ConnectWF(NCM_type _ncm, MPR_type* _mpr,LP_type* _plp) {
    m_pNodeCon = _ncm;
    m_region = _mpr;
    m_lp = _plp;
  }
  void define_type(stapl::typer& _t) {
    _t.member(m_region);
  }
  template <typename PartitionedView, typename OverlapView>
  void operator()(PartitionedView _v1, OverlapView _v2) const {
    LOG_DEBUG_MSG("ParallelPRMStrategy::ConnectWF- native view.size= " << _v1.size());
    LOG_DEBUG_MSG("ParallelPRMStrategy::ConnectWF- overlap view.size= " << _v2.size())
		
    typename PartitionedView::vertex_iterator pv_first = _v1.begin();
    typename PartitionedView::vertex_iterator pv_last = _v1.end();
    typename OverlapView::vertex_iterator ov_first = _v2.begin();
    typename OverlapView::vertex_iterator ov_last = _v2.end();
		
    LocalPlanners<CfgType,WeightType/>* __lp = const_cast<LocalPlanners<CfgType,WeightType>*>(m_lp);
			
    stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
    cmap.reset();
    NodeCon->Connect(m_region->GetRoadmap(), *(m_region->GetStatClass()), cmap,
      pv_first,pv_last, ov_first, ov_last);	
  }
};

template<typename View>
void p_sample(View& _view, Sampler<CfgType>::SamplerPointer _ng, MPRegion<CfgType,WeightType>* _region, Environment* _env) {
  SampleWF wf(_ng,_region,_env);	
  stapl::map_func(wf,stapl::balance_view(_view,stapl::get_num_locations()));	
}

template<typename PartitionedView, typename OverlapView>
void p_connect(PartitionedView& _v1,OverlapView& _v2, Connector<CfgType, WeightType>::ConnectionPointer _ncm, MPRegion<CfgType,WeightType>* _region, 
  LocalPlanners<CfgType, WeightType>* _lp) {
  
  ConnectWF wf(_ncm,_region,_lp);
	
  LOG_DEBUG_MSG("ParallelPRMStrategy::p_connect()");
  stapl::map_func(wf, _v1, _v2);	
}

class ParallelPRMRoadmap : public MPStrategyMethod {
public:	
  ParallelPRMRoadmap(XMLNodeReader& _node, MPProblem* _problem) :
  MPStrategyMethod(_node,_problem) {
    LOG_DEBUG_MSG("ParallelPRMRoadmap::ParallelPRMRoadmap()");
    ParseXML(_node);    	
  };
  virtual ~ParallelPRMRoadmap() {}
  virtual void PrintOptions(ostream& _outOs) { };
  virtual void Initialize(int _inRegionID){};
  virtual void ParseXML(XMLNodeReader& _node) {
    LOG_DEBUG_MSG("ParallelPRMRoadmap::ParseXML()");
		
    XMLNodeReader::childiterator m_citr;
    for( m_citr = _node.children_begin(); m_citr!= _node.children_end(); ++m_citr) {
      if(citr->getName() == "node_generation_method") {
        string node_gen_method = m_citr->stringXMLParameter(string("Method"), true,
          string(""), string("Node Generation Method"));
        int numPerIteration = m_citr->numberXMLParameter(string("Number"), true, 
          int(1), int(0), MAX_INT, string("Number of samples"));
        m_vecStrNodeGenLabels.push_back(pair<string, int>(node_gen_method, numPerIteration));
        m_citr->warnUnrequestedAttributes();
      } else if(m_citr->getName() == "node_connection_method") {
        string connect_method = m_citr->stringXMLParameter(string("Method"), true,
          string(""), string("Node Connection Method"));
        m_vecStrNodeConnectionLabels.push_back(connect_method);
        m_citr->warnUnrequestedAttributes();
      } else {	
        m_citr->warnUnknownNode();
      }
    }
    LOG_DEBUG_MSG("ParallelPRMRoadmap::ParseXML()");
  };
	
virtual void Run(int _inRegionID) {
  LOG_DEBUG_MSG("ParallelPRMRoadmap::Run()");
  SRand(getSeed()); 
  m_region = GetMPProblem()->GetMPRegion(_inRegionID);
  pStatClass = m_region->GetStatClass();
  CollisionDetection*  pCd = GetMPProblem()->GetCollisionDetection();
  Environment* pEnv = GetMPProblem()->GetEnvironment();
  LocalPlanners<CfgType, WeightType>* pLp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
		
  RoadmapGraph<CfgType,WeightType>* rmg = m_region->GetRoadmap()->m_pRoadmap; 
  	
  stapl::counter<stapl::default_timer> t1,t2;
		
  double sample_timer=0.0, connect_timer=0.0 ;
				
  for(int it =1; it<= m_iterations; ++it) {       	
    typedef vector<pair<string, int> >::iterator I;
      
    //---------------------------
    // Generate roadmap nodes
    //---------------------------
						
    for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr) {	
      Sampler<CfgType>::SamplerPointer  m_nodeGen;
      m_nodeGen = GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(itr->first);
			      				
      stapl::array<CfgType> PA(num_nodes);
      stapl::array_1D_view<stapl::array<CfgType> > v(PA);
					
      t1.start();
      p_sample(v,m_nodeGen,m_region,pEnv);
      sample_timer = t1.stop();
				
      cout<<"\n processor #----->["<<stapl::get_location_id()<<"] NodeGeneration time  = "  << sample_timer << endl; 
    }	
    //---------------------------
    // Connect roadmap nodes
    //---------------------------
			
    typedef vector<string>::iterator J;
    for(J itr = m_vecStrNodeConnectionLabels.begin(); itr != m_vecStrNodeConnectionLabels.end(); ++itr) {      
      LOG_DEBUG_MSG("ParallelPRMStrategy::graph size " << rmg->size());
				
      Connector<CfgType, WeightType>::ConnectionPointer pConnection;
      pConnection = GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*itr);
      typedef stapl::p_graph_view_base<RoadmapGraph<CfgType,WeightType> >   VType;
      VType g_view(*rmg);
      stapl::replicated_view<stapl::replicated_container<VType> > voverlap =
      stapl::replicate(g_view);
      stapl::part_native_view<VType>::view_type vnative = 
      stapl::part_native_view<VType>()(g_view);
				
      t2.start();
      p_connect(vnative,voverlap,pConnection,m_region,pLp);
      connect_timer = t2.stop();
      for (VI vi=m_region->GetRoadmap()->m_pRoadmap->begin(); vi!= m_region->GetRoadmap()->m_pRoadmap->end(); ++vi){  
	  
      }
      cout<<"\n processor #["<<stapl::get_location_id()<<"] NodeConnection time  = "  << connect_timer << endl; 	
      }
    }		
  }
	
  virtual void Finalize(int _inRegionID){
		
    LOG_DEBUG_MSG("ParallelPRMStrategy::Finalize()");
    //---------------------------
    // Write roadmap to file
    //---------------------------

    if( stapl::get_location_id() == 0){
      string str;
      str = getBaseFilename() + ".map";
      ofstream osMap(str.c_str());
      if(!osMap){
        LOG_ERROR_MSG("ParallelPRMStrategy::Finalize(): can't open outfile: ");
        exit(-1);
      }else{
        m_region->WriteRoadmapForVizmo(osMap);
        osMap.close();
      }
    }
    cout << "!!ALL FINISHED!!"<< endl;
  }
	
private:
  vector<pair<string, int> > m_vecStrNodeGenLabels;
  vector<string> m_vecStrNodeConnectionLabels;
  MPRegion<CfgType,WeightType>* m_region;
  StatClass* m_statClass;
};


#endif

