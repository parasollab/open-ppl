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

    NGM_type m_nodeGen;
    MPProblem* m_problem;

  public:
    SampleWF(NGM_type _ngm, MPProblem* _problem) {
      m_nodeGen = _ngm;
      m_problem = _problem;

    }
    void define_type(stapl::typer& _t) { 
      //_t.member(m_nodeGen);
      // _t.member(m_problem);
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

      StatClass* stat = m_problem->GetStatClass();
      m_nodeGen->Sample(m_problem->GetEnvironment(),*stat,inNodes.begin(),inNodes.end(), 100, back_inserter(outNodes));

      cout << "Sampled something" << endl;
      size_t j(0);
      typedef vector<CfgType>::iterator VIT;
      for(VIT vit = outNodes.begin(); vit  != outNodes.end(); ++vit, j++) {
          
        CfgType tmp = *vit;
        m_problem->GetRoadmap()->m_pRoadmap->add_vertex(tmp);

      }
    }
};

class ConnectWF {
  private:
    typedef Connector<CfgType, WeightType>::ConnectionPointer NCM_type;
    typedef typename RoadmapGraph<CfgType, WeightType>::VID VID;
    NCM_type m_nodeCon;
    MPProblem* m_problem;

  public:
    ConnectWF(NCM_type _ncm, MPProblem* _problem) {
      m_nodeCon = _ncm;
      m_problem = _problem;
    }
    void define_type(stapl::typer& _t) {
       _t.member(m_problem);
       _t.member(m_nodeCon);
    }
    template <typename NativeView, typename RepeatView>
    void operator()(NativeView _vw1, RepeatView _vw2) const {
	
      psbmp::PrintValue("Basic Parallel- Native View : " , _vw1.size());
      psbmp::PrintValue("Basic Parallel - Repeat View : " , _vw2.size());
        
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
      m_nodeCon->Connect(m_problem->GetRoadmap(),
        *(m_problem->GetStatClass()), cmap, v1.begin(),v1.end(), v2.begin(), v2.end());
    }
};


class BasicParallelPRM : public MPStrategyMethod {
  public:
    BasicParallelPRM(XMLNodeReader& _node, MPProblem* _problem); 
    virtual ~BasicParallelPRM() {};

    virtual void PrintOptions(ostream& _outOs) {};
    virtual void ParseXML(XMLNodeReader& _inPNode); 

    virtual void Initialize() {};
    virtual void Run(); 
    virtual void Finalize();

  private:
    vector<pair<string, int> > m_vecStrNodeGenLabels;
    vector<string> m_vecStrNodeConnectionLabels;
    StatClass* m_statClass;
    string m_nodeGen;
    int m_numIterations;
    int m_numSamples;
};


#endif

