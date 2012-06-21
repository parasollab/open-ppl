///////////////////////////
//Header BasicParallelPRM
//////////////////////////

#ifndef BASICPARALLELPRM_H_
#define BASICPARALLELPRM_H_

#include "ParallelSBMPHeader.h"

using namespace std;

class SampleWF {
  private:
    typedef Sampler<CfgType>::SamplerPointer NGM_type;
    typedef MPRegion<CfgType,WeightType>  MPR_type;

    NGM_type m_pNodeGen;
    MPR_type* m_region;
    Environment* m_env;

  public:
    SampleWF(NGM_type _ngm, MPR_type* _mpr, Environment* _pEnv) {
      m_pNodeGen = _ngm;
      m_region = _mpr;
      m_env = _pEnv;

    }
    void define_type(stapl::typer& _t) { 
      _t.member(m_region);
    }

    SampleWF(const SampleWF& _wf, std::size_t _offset) {} 

    template<typename View> 
    void operator()(const View& _view) const {
      cout << "Entered sample wf" << endl;
      int num_nodes = _view.size();
      vector<CfgType> outNodes;
      cout << "Num_Nodes::" << num_nodes << endl;
      vector<CfgType> inNodes(num_nodes);
      //if(m_debug) cout << "ParallelPRMStrategy::SampleWF- view.size= " << view.size() << endl;

      Environment* _env = const_cast<Environment*>(m_env);

      m_pNodeGen->Sample(_env,*(m_region->GetStatClass()),inNodes.begin(),inNodes.end(), 100, 
        back_inserter(outNodes));

      cout << "Sampled something" << endl;
      size_t j(0);
      typedef vector<CfgType>::iterator VIT;
      for(VIT vit = outNodes.begin(); vit  != outNodes.end(); ++vit, j++) {
          
        CfgType tmp = *vit;
        m_region->GetRoadmap()->m_pRoadmap->add_vertex(tmp);

      }
    }
};

class ConnectWF {
  private:
    typedef MPRegion<CfgType,WeightType>  MPR_type;
    typedef Connector<CfgType, WeightType>::ConnectionPointer NCM_type;
    MPR_type* m_region;
    NCM_type m_nodeCon;

  public:
    ConnectWF(NCM_type _ncm, MPR_type* _mpr) {
      m_nodeCon = _ncm;
      m_region = _mpr;
    }
    void define_type(stapl::typer& _t) {
      _t.member(m_region);
    }
    template <typename NativeView, typename RepeatView>
    void operator()(NativeView _vw1, RepeatView _vw2) const {
	
      PrintValue("Basic Parallel- Native View : " , _vw1.size());
      PrintValue("Basic Parallel - Repeat View : " , _vw2.size());
        
      vector<VID> v1;
      vector<VID> v2; 
	
      ///ass views directly to connect
      for(typename NativeView::vertex_iterator vit1 = _vw1.begin(); vit1!= _vw1.end(); ++vit1){
	v1.push_back((*vit1).descriptor());
      }
	
      for(typename RepeatView::vertex_iterator vit2 = _vw2.begin(); vit2!= _vw2.end(); ++vit2){
	v2.push_back((*vit2).descriptor());
      }

      stapl::sequential::vector_property_map<RoadmapGraph<CfgType, WeightType>::GRAPH,size_t > cmap;
      cmap.reset();
      vector<VID> dummyVec;
      m_nodeCon->Connect(m_region->GetRoadmap(),
        *(m_region->GetStatClass()), cmap, v1.begin(),v1.end(), v2.begin(), v2.end());
    }
};


class BasicParallelPRM : public MPStrategyMethod {
  public:
    BasicParallelPRM(XMLNodeReader& _node, MPProblem* _problem); 
    virtual ~BasicParallelPRM() {};

    virtual void PrintOptions(ostream& _outOs) {};
    virtual void ParseXML(XMLNodeReader& _inPNode); 

    virtual void Initialize(int _inRegionID) {};
    virtual void Run(int _inRegionID); 
    virtual void Finalize(int _inRegionID);

  private:
    vector<pair<string, int> > m_vecStrNodeGenLabels;
    vector<string> m_vecStrNodeConnectionLabels;
    MPRegion<CfgType,WeightType>* m_region;
    StatClass* m_statClass;
    string m_nodeGen;
    int m_numIterations;
    int m_numSamples;
};


#endif

