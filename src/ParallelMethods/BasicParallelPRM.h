#ifndef BASICPARALLELPRM_H
#define BASICPARALLELPRM_H

#include "ParallelSBMPHeader.h"
using namespace std;


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
          region->GetRoadmap()->m_pRoadmap->add_vertex(tmp);

        }

      }


};


class connect_wf {
  private:
    typedef MPRegion<CfgType,WeightType>  MPR_type;
    typedef Connector<CfgType, WeightType>::ConnectionPointer NCM_type;
    MPR_type* m_region;
    NCM_type m_nodeCon;

  public:
    connect_wf(NCM_type _ncm, MPR_type* _mpr) {
      m_nodeCon = _ncm;
      m_region = _mpr;
    }
    void define_type(stapl::typer &t)  
    {

      t.member(m_region);
    }
    template <typename NativeView, typename RepeatView>
      void operator()(NativeView vw1, RepeatView vw2) const {
	
	PrintValue("Basic Parallel- Native View : " , vw1.size());
	PrintValue("Basic Parallel - Repeat View : " , vw2.size());

        
	vector<VID> v1;
	vector<VID> v2; 
	
	///ass views directly to connect
	for(typename NativeView::vertex_iterator vit1 = vw1.begin(); vit1!= vw1.end(); ++vit1){
	  v1.push_back((*vit1).descriptor());
	}
	
	for(typename RepeatView::vertex_iterator vit2 = vw2.begin(); vit2!= vw2.end(); ++vit2){
	  v2.push_back((*vit2).descriptor());
	}

        stapl::sequential::vector_property_map<RoadmapGraph<CfgType, WeightType>::GRAPH,size_t > cmap;
        cmap.reset();
        vector<VID> dummyVec;
        m_nodeCon->Connect(m_region->GetRoadmap(),
                           *(m_region->GetStatClass()),
			   cmap,
                           v1.begin(),v1.end(),
                           v2.begin(), v2.end());

      }

};


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

