///////////////////////////
//Header BasicParallelPRM
//////////////////////////

#ifndef BASICPARALLELPRM_H_
#define BASICPARALLELPRM_H_

#include "ParallelSBMPHeader.h"

using namespace std;

template<class MPTraits>
class SampleWF {
  private:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::SamplerPointer SamplerPointer;

    SamplerPointer m_nodeGen;
    MPProblemType* m_problem;

  public:
    SampleWF(SamplerPointer _ngm, MPProblemType* _problem) {
      m_nodeGen = _ngm;
      m_problem = _problem;

    }

    typedef void result_type;

    void define_type(stapl::typer& _t) {
    }

    SampleWF(const SampleWF& _wf, std::size_t _offset) {}

    template<typename View>
    void operator()(const View& _view) const {
      cout << "Entered sample wf" << endl;
      int num_nodes = _view.size();
      vector<CfgType> outNodes;
      cout << "Num_Nodes::" << num_nodes << endl;
      vector<CfgType> inNodes(num_nodes);

      StatClass* stat = m_problem->GetStatClass();
      m_nodeGen->Sample(m_problem->GetEnvironment(),*stat,inNodes.begin(),inNodes.end(), 100, back_inserter(outNodes));

      cout << "Add to Graph" << endl;
      size_t j(0);
      typedef typename vector<CfgType>::iterator VIT;
      for(VIT vit = outNodes.begin(); vit  != outNodes.end(); ++vit, j++) {

        CfgType tmp = *vit;
        m_problem->GetRoadmap()->GetGraph()->add_vertex(tmp);

      }
    }
};

template<class MPTraits>
class ConnectWF {
  private:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;

    ConnectorPointer m_nodeCon;
    MPProblemType* m_problem;

  public:
    ConnectWF(ConnectorPointer _ncm, MPProblemType* _problem) {
      m_nodeCon = _ncm;
      m_problem = _problem;
    }

    typedef void result_type;

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

      ///why not pass views directly to connector?
      for(typename NativeView::vertex_iterator vit1 = _vw1.begin(); vit1!= _vw1.end(); ++vit1){
	v1.push_back((*vit1).descriptor());
      }

      for(typename RepeatView::vertex_iterator vit2 = _vw2.begin(); vit2!= _vw2.end(); ++vit2){
	v2.push_back((*vit2).descriptor());
      }

      stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> cmap;
      cmap.reset();
      //vector<VID> dummyVec;
      m_nodeCon->Connect(m_problem->GetRoadmap(),
        *(m_problem->GetStatClass()), cmap, v1.begin(),v1.end(), v2.begin(), v2.end());
    }
};


template<class MPTraits>
class BasicParallelPRM : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::SamplerPointer SamplerPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;

    BasicParallelPRM(const vector<pair<string, int> >& _vecStrNodeGenLabels = vector<pair<string, int> >(),
       const vector<string>& _vecStrNodeConnectionLabels = vector<string>(),
       string _nodeGen = "", int _numIterations = 0, int _numSamples = 0);

    BasicParallelPRM(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);

    virtual ~BasicParallelPRM() {};

    virtual void Print(ostream& _os) const;
    virtual void ParseXML(XMLNodeReader& _inPNode);

    virtual void Initialize() {};
    virtual void Run();
    virtual void Finalize();

  private:
    vector<pair<string, int> > m_vecStrNodeGenLabels;
    vector<string> m_vecStrNodeConnectionLabels;
    string m_nodeGen;
    int m_numIterations;
    int m_numSamples;
};

template<class MPTraits>
BasicParallelPRM<MPTraits>::BasicParallelPRM(const vector<pair<string, int> >& _vecStrNodeGenLabels, const vector<string>& _vecStrNodeConnectionLabels,
  string _nodeGen, int _numIterations, int _numSamples)
  :m_vecStrNodeGenLabels(_vecStrNodeGenLabels), m_vecStrNodeConnectionLabels(_vecStrNodeConnectionLabels),
  m_nodeGen(_nodeGen), m_numIterations(_numIterations), m_numSamples(_numSamples){
  this->SetName("BasicParallelPRM");
}

template<class MPTraits>
BasicParallelPRM<MPTraits>::BasicParallelPRM(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node):
  MPStrategyMethod<MPTraits>(_problem, _node){
  if (this->m_debug) cout << "BasicParallelPRM::BasicParallelPRM" << endl;
  ParseXML(_node);
  this->SetName("BasicParallelPRM");
}


template<class MPTraits>
void
BasicParallelPRM<MPTraits>::ParseXML(XMLNodeReader& _node) {
  if (this->m_debug) cout << "BasicParallelPRM::ParseXML" << endl;

  m_numIterations = _node.numberXMLParameter("iterations", true, int(1), int(0), MAX_INT, "iterations of strategy");

  XMLNodeReader::childiterator citr;
  for( citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
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

  if (this->m_debug) cout << "BasicParallelPRM::ParseXML()" << endl;
}

template<class MPTraits>
void
BasicParallelPRM<MPTraits>::Print(ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);
  typedef vector<pair<string, int> >::const_iterator VIter;
  typedef vector<string>::const_iterator StringIter;
  _os<<"\nSamplers\n";
  for(VIter vIter=m_vecStrNodeGenLabels.begin();
      vIter!=m_vecStrNodeGenLabels.end(); vIter++){
    _os<<"\t"<<vIter->first<<"\tNumber:"<<vIter->second;
  }

  _os<<"\nNodeConnectors\n";
  for(StringIter sIter=m_vecStrNodeConnectionLabels.begin(); sIter!=m_vecStrNodeConnectionLabels.end(); sIter++){
    _os<<"\t"<<*sIter;
  }

}

template<class MPTraits>
void
BasicParallelPRM<MPTraits>::Run() {
  if (this->m_debug) cout << "BasicParallelPRM::Run()" << endl;

  GraphType* rmg = this->GetMPProblem()->GetRoadmap()->GetGraph();

  stapl::counter<stapl::default_timer> t1,t2;

  double sample_timer=0.0, connect_timer=0.0 ;
  for(int it =1; it<= m_numIterations; ++it) {
    typedef vector<pair<string, int> >::iterator I;

    //---------------------------
    // Generate roadmap nodes
    //---------------------------

    cout << "Generation Phase" << endl;
    for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr) {
      SamplerPointer nodeGen = this->GetMPProblem()->GetSampler(itr->first);
      typedef stapl::array<CfgType> cfgArray;
      typedef stapl::array_view<cfgArray> viewCfgArray;
      cfgArray PA(itr->second);
      viewCfgArray v(PA);

      t1.start();
      SampleWF<MPTraits> sampleWf(nodeGen, this->GetMPProblem());
      stapl::map_func(sampleWf,stapl::balance_view(v,stapl::get_num_locations()));
      sample_timer = t1.stop();

      if (this->m_debug)
        cout<<"\n processor #----->["<<stapl::get_location_id()<<"] NodeGeneration time  = "  << sample_timer << endl;
    }
    stapl::rmi_fence();
    //---------------------------
    // Connect roadmap nodes
    //---------------------------
    cout << "Connecting Phase" << endl;
    typedef vector<string>::iterator J;
    for(J itr = m_vecStrNodeConnectionLabels.begin(); itr != m_vecStrNodeConnectionLabels.end(); ++itr) {
      if (this->m_debug) cout << "BasicParallelPRM::graph size " << rmg->size() << endl;

      ConnectorPointer connector = this->GetMPProblem()->GetConnector(*itr);
      typedef stapl::graph_view<GraphType>   VType;
      VType g_view(*rmg);

      t2.start();
      ConnectWF<MPTraits> connWf(connector, this->GetMPProblem());
      stapl::map_func(connWf, stapl::native_view(g_view), stapl::make_repeat_view(g_view));
      connect_timer = t2.stop();
      if (this->m_debug) {
          cout<<"\n processor #["<<stapl::get_location_id()<<"] NodeConnection time  = "  << connect_timer << endl;

      }
    }
  }
}

template<class MPTraits>
void
BasicParallelPRM<MPTraits>::Finalize() {
  if (this->m_debug) cout << "BasicParallelPRM::Finalize()";
  //---------------------------
  // Write roadmap to file
  //---------------------------
  stapl::rmi_fence();

  string str = this->GetBaseFilename() + ".map";
  this->GetMPProblem()->GetRoadmap()->Write(str, this->GetMPProblem()->GetEnvironment());

  stapl::rmi_fence();
  if (this->m_debug) cout << "!!ALL FINISHED!!"<< endl;
}

#endif

