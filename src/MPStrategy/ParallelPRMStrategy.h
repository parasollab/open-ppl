#ifndef ParallelPRMStrategy_h
#define ParallelPRMStrategy_h

#include "Roadmap.h"



#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "LocalPlanners.h"

#include "Sampler.h"



#include "util.h"
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
#include "views/native_partition.hpp"
#include "views/replicated_view.hpp"

using namespace std;
typedef stapl::p_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType, WeightType> PGRAPH;
typedef PGRAPH::vertex_descriptor VID; 
typedef PGRAPH::vertex_iterator VI;

class sample_wf 
{
	private:
	typedef Sampler<CfgType>::SamplerPointer NGM_type;
	typedef MPRegion<CfgType,WeightType>  MPR_type;
	
	NGM_type pNodeGen;
	MPR_type* region;
	Environment* env;
	
	
	
	public:
	
	sample_wf(NGM_type _ngm, MPR_type* _mpr, Environment* _penv) {
		pNodeGen = _ngm;
		region = _mpr;
		env = _penv;
		
	}
	void define_type(stapl::typer &t)  
	{
		t.member(region);
	}
	
	sample_wf(const sample_wf& _wf, std::size_t offset)  {} 
	
	template<typename View> 
	void operator()(const View& view) const
	{
		
		int num_nodes = view.size();
		vector<CfgType> outNodes;
		vector<CfgType> inNodes(num_nodes);
		LOG_DEBUG_MSG("ParallelPRMStrategy::sample_wf- view.size= " << view.size());
		
		Environment* _env = const_cast<Environment*>(env);
		
		pNodeGen->Sample(_env,*(region->GetStatClass()),inNodes.begin(),inNodes.end(), 100, 
			         back_inserter(outNodes));
		
		size_t j(0);
		typedef vector<CfgType>::iterator VIT;
		for(VIT vit = outNodes.begin(); vit  != outNodes.end(); ++vit, j++) {
			
			CfgType tmp = *vit;
			VID NEW = region->GetRoadmap()->m_pRoadmap->add_vertex(tmp);
			
			}
		
	}
	
	
};


class connect_wf {
	private:
	typedef MPRegion<CfgType,WeightType>  MPR_type;
	typedef ConnectMap<CfgType, WeightType>::NodeConnectionPointer NCM_type;
	typedef LocalPlanners<CfgType,WeightType> LP_type;
	MPR_type* region;
	NCM_type pNodeCon;
	LP_type*  lp;
	
	public:
	connect_wf(NCM_type _ncm, MPR_type* _mpr,LP_type* _plp) {
		pNodeCon = _ncm;
		region = _mpr;
		lp = _plp;
	}
	void define_type(stapl::typer &t)  
	{
		
		t.member(region);
	}
	template <typename PartitionedView, typename OverlapView>
	void operator()(PartitionedView v1, OverlapView v2) const {
		LOG_DEBUG_MSG("ParallelPRMStrategy::connect_wf- native view.size= " << v1.size());
		LOG_DEBUG_MSG("ParallelPRMStrategy::connect_wf- overlap view.size= " << v2.size())
		
		typename PartitionedView::vertex_iterator pv_first = v1.begin();
		typename PartitionedView::vertex_iterator pv_last = v1.end();
		typename OverlapView::vertex_iterator ov_first = v2.begin();
		typename OverlapView::vertex_iterator ov_last = v2.end();
		
		LocalPlanners<CfgType,WeightType>* __lp = const_cast<LocalPlanners<CfgType,WeightType>*>(lp);
		
		
		pNodeCon->GetConnectMap()->pConnectNodes(pNodeCon,
			region->GetRoadmap(),
			*(region->GetStatClass()),
			lp,
			false,false,
			pv_first,pv_last,
			ov_first, ov_last);
		
	}
	
};




template<typename View>
void p_sample(View& view, Sampler<CfgType>::SamplerPointer _ng, MPRegion<CfgType,WeightType>* _region, Environment * _env)
{
	sample_wf wf(_ng,_region,_env);	
	stapl::map_func(wf,stapl::balance_view(view,stapl::get_num_locations()));
	
	
	
}



template<typename PartitionedView, typename OverlapView>
void p_connect(PartitionedView& v1,OverlapView& v2, ConnectMap<CfgType, WeightType>::NodeConnectionPointer _ncm, MPRegion<CfgType,WeightType>* _region, 
	LocalPlanners<CfgType, WeightType>* _lp)
{
	connect_wf wf(_ncm,_region,_lp);
	
	LOG_DEBUG_MSG("ParallelPRMStrategy::p_connect()");
	stapl::map_func(wf, v1, v2);
	
	
	
}




class ParallelPRMRoadmap : public MPStrategyMethod {
	public:
	
	
	ParallelPRMRoadmap(XMLNodeReader& in_pNode, MPProblem* in_pProblem) :
	MPStrategyMethod(in_pNode,in_pProblem) {
		LOG_DEBUG_MSG("ParallelPRMRoadmap::ParallelPRMRoadmap()");
		ParseXML(in_pNode);    
		
	};
	virtual ~ParallelPRMRoadmap() {}
	
	virtual void PrintOptions(ostream& out_os) { };
	virtual void Initialize(int in_RegionID){};
	
	
	virtual void ParseXML(XMLNodeReader& in_pNode) {
		LOG_DEBUG_MSG("ParallelPRMRoadmap::ParseXML()");
		
		XMLNodeReader::childiterator citr;
		for( citr = in_pNode.children_begin(); citr!= in_pNode.children_end(); ++citr) {
			if(citr->getName() == "node_generation_method") {
				string node_gen_method = citr->stringXMLParameter(string("Method"), true,
					string(""), string("Node Generation Method"));
				int numPerIteration = citr->numberXMLParameter(string("Number"), true, 
					int(1), int(0), MAX_INT, string("Number of samples"));
				m_vecStrNodeGenLabels.push_back(pair<string, int>(node_gen_method, numPerIteration));
				citr->warnUnrequestedAttributes();
			} else if(citr->getName() == "node_connection_method") {
				string connect_method = citr->stringXMLParameter(string("Method"), true,
					string(""), string("Node Connection Method"));
				m_vecStrNodeConnectionLabels.push_back(connect_method);
				citr->warnUnrequestedAttributes();
			} else {
				
				citr->warnUnknownNode();
			}
		}
		
		
		
		LOG_DEBUG_MSG("ParallelPRMRoadmap::ParseXML()");
	};
	
	
	
	
	virtual void Run(int in_RegionID) {
		LOG_DEBUG_MSG("ParallelPRMRoadmap::Run()");
		OBPRM_srand(getSeed()); 
		region = GetMPProblem()->GetMPRegion(in_RegionID);
		pStatClass = region->GetStatClass();
		CollisionDetection *  pCd = GetMPProblem()->GetCollisionDetection();
		Environment * pEnv = GetMPProblem()->GetEnvironment();
		LocalPlanners<CfgType, WeightType>* pLp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
		
		RoadmapGraph<CfgType,WeightType> * rmg = region->GetRoadmap()->m_pRoadmap; 
		
		
		stapl::counter<stapl::default_timer> t1,t2;
		
		double sample_timer=0.0, connect_timer=0.0 ;
		
		
		
		for(int it =1; it<= m_iterations; ++it)
		{       
			
			typedef vector<pair<string, int> >::iterator I;
			
			
			//---------------------------
			// Generate roadmap nodes
			//---------------------------
			
			
			for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr)
			{
				
				Sampler<CfgType>::SamplerPointer  pNodeGen;
				pNodeGen = GetMPProblem()->GetMPStrategy()->GetSampler()->GetSamplingMethod(itr->first);
				
				
				
				
				stapl::p_array<CfgType> PA(num_nodes);
				stapl::array_1D_view<stapl::p_array<CfgType> > v(PA);
				
				
				t1.start();
				p_sample(v,pNodeGen,region,pEnv);
				sample_timer = t1.stop();
				
				cout<<"\n processor #----->["<<stapl::get_location_id()<<"] NodeGeneration time  = "  << sample_timer << endl; 
			}
			
			//---------------------------
			// Connect roadmap nodes
			//---------------------------
			
			
			typedef vector<string>::iterator J;
			for(J itr = m_vecStrNodeConnectionLabels.begin(); itr != m_vecStrNodeConnectionLabels.end(); ++itr)
			{      
				LOG_DEBUG_MSG("ParallelPRMStrategy::graph size " << rmg->size());
				
				ConnectMap<CfgType, WeightType>::NodeConnectionPointer pConnection;
				pConnection = GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetNodeMethod(*itr);
				typedef stapl::p_graph_view_base<RoadmapGraph<CfgType,WeightType> >   VType;
				VType g_view(*rmg);
				stapl::replicated_view<stapl::replicated_container<VType> > voverlap =
                                stapl::replicate(g_view);
				stapl::part_native_view<VType>::view_type vnative = 
				stapl::part_native_view<VType>()(g_view);
				
				t2.start();
				p_connect(vnative,voverlap,pConnection,region,pLp);
				connect_timer = t2.stop();
				for (VI vi=region->GetRoadmap()->m_pRoadmap->begin(); vi!= region->GetRoadmap()->m_pRoadmap->end(); ++vi){  
				}
				cout<<"\n processor #["<<stapl::get_location_id()<<"] NodeConnection time  = "  << connect_timer << endl; 
				
			}
			
			
			
			
			
		}
		
		
	}
	
	virtual void Finalize(int in_RegionID){
		
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
				region->WriteRoadmapForVizmo(osMap);
				osMap.close();
			}
		}
		cout << "!!ALL FINISHED!!"<< endl;
	}
	
	
	
	private:
	vector<pair<string, int> > m_vecStrNodeGenLabels;
	vector<string> m_vecStrNodeConnectionLabels;
	MPRegion<CfgType,WeightType>* region;
	Stat_Class * pStatClass;
	
	
};


#endif

