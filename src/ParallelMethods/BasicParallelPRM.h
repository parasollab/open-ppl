#ifndef BASICPARALLELPRM_H
#define BASICPARALLELPRM_H
/*
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
*/

#include "ParallelSBMPHeader.h"
using namespace std;
typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType, WeightType> PGRAPH;
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
        cout << "Entered sample wf" << endl;
        int num_nodes = view.size();
        vector<CfgType> outNodes;
        cout << "Num_Nodes::" << num_nodes << endl;
        vector<CfgType> inNodes(num_nodes);
        //if(m_debug) cout << "ParallelPRMStrategy::sample_wf- view.size= " << view.size() << endl;

        Environment* _env = const_cast<Environment*>(env);

        pNodeGen->Sample(_env,*(region->GetStatClass()),inNodes.begin(),inNodes.end(), 100, 
            back_inserter(outNodes));

        cout << "Sampled something" << endl;
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
    typedef Connector<CfgType, WeightType>::ConnectionPointer NCM_type;
    typedef LocalPlanners<CfgType,WeightType> LP_type;
    MPR_type* m_region;
    NCM_type m_nodeCon;
    LP_type*  m_lp;

  public:
    connect_wf(NCM_type _ncm, MPR_type* _mpr,LP_type* _lp) {
      m_nodeCon = _ncm;
      m_region = _mpr;
      m_lp = _lp;
    }
    void define_type(stapl::typer &t)  
    {

      t.member(m_region);
    }
    template <typename PartitionedView, typename OverlapView>
      void operator()(PartitionedView v1, OverlapView v2) const {
        cout << "ParallelPRMStrategy::connect_wf- native view.size= " << v1.size() << endl;
        cout << "ParallelPRMStrategy::connect_wf- overlap view.size= " << v2.size() << endl;

        typename PartitionedView::vertex_iterator pv_first = v1.begin();
        typename PartitionedView::vertex_iterator pv_last = v1.end();
        typename OverlapView::vertex_iterator ov_first = v2.begin();
        typename OverlapView::vertex_iterator ov_last = v2.end();

        LocalPlanners<CfgType,WeightType>* __lp = const_cast<LocalPlanners<CfgType,WeightType>*>(m_lp);

      //  ######## PGRAPH used to be GRAPH #########
        stapl::sequential::vector_property_map< PGRAPH,size_t > cmap;
        cmap.reset();
        vector<VID> dummyVec;
        m_nodeCon->Connect(
            m_region->GetRoadmap(),
            *(m_region->GetStatClass()),
            cmap); /*
            ov_first,ov_last,
            ov_first, ov_last, back_inserter(dummyVec));
*/
      }

};




template<typename View>
void p_sample(View& view, Sampler<CfgType>::SamplerPointer _ng, MPRegion<CfgType,WeightType>* _region, Environment * _env)
{
  sample_wf wf(_ng,_region,_env);	
  stapl::map_func(wf,stapl::balance_view(view,stapl::get_num_locations()));



}



template<typename PartitionedView, typename OverlapView>
void p_connect(PartitionedView& v1,OverlapView& v2, Connector<CfgType, WeightType>::ConnectionPointer _ncm, MPRegion<CfgType,WeightType>* _region, 
    LocalPlanners<CfgType, WeightType>* _lp)
{
  connect_wf wf(_ncm,_region,_lp);

  stapl::map_func(wf, v1, v2);



}

class BasicParallelPRM : public MPStrategyMethod {
  public:
    BasicParallelPRM(XMLNodeReader& in_pNode, MPProblem* in_pProblem); 
    //BasicParallelPRM() {}
    virtual ~BasicParallelPRM() {};

    virtual void PrintOptions(ostream& out_os) {};
    virtual void ParseXML(XMLNodeReader& in_pNode); 

    virtual void Initialize(int in_RegionID) {};
    virtual void Run(int in_RegionID); 
    virtual void Finalize(int in_RegionID);

  private:
    vector<pair<string, int> > m_vecStrNodeGenLabels;
    vector<string> m_vecStrNodeConnectionLabels;
    MPRegion<CfgType,WeightType>* region;
    StatClass * pStatClass;
    string m_nodeGen;
    int m_numIterations;
    int m_numSamples;
};


#endif

