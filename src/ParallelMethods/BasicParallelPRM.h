#ifndef BASIC_PARALLEL_PRM_H_
#define BASIC_PARALLEL_PRM_H_

#include "ParallelSBMPHeader.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class SampleWF {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::SamplerPointer SamplerPointer;

    SampleWF(SamplerPointer _sp, MPProblemType* _problem);
    SampleWF(const SampleWF& _wf, std::size_t _offset) {}

    typedef void result_type;

    void define_type(stapl::typer& _t) {}

    template<typename View>
      void operator()(const View& _view) const;

  private:
    SamplerPointer m_sp;
    MPProblemType* m_problem;
};

template<class MPTraits>
SampleWF<MPTraits>::
SampleWF(SamplerPointer _sp, MPProblemType* _problem) : m_sp(_sp), m_problem(_problem) {}

template<class MPTraits>
template<typename View>
void
SampleWF<MPTraits>::
operator()(const View& _view) const {
  cout << "Entered sample wf" << endl;

  int numNodes = _view.size();
  vector<CfgType> outNodes;

  cout << "Num_Nodes::" << numNodes << endl;

  //StatClass* stat = m_problem->GetStatClass();

  // We should use the same method as BasicPRM.h
  m_sp->Sample(numNodes, 100, m_problem->GetEnvironment()->GetBoundary(), back_inserter(outNodes));

  cout << "Add to Graph" << endl;

  typedef typename vector<CfgType>::iterator VIT;
  for(auto& cfg : outNodes)
    m_problem->GetRoadmap()->GetGraph()->add_vertex(cfg);
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ConnectWF {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;

    ConnectWF(ConnectorPointer _cp, MPProblemType* _problem) {
      m_cp = _cp;
      m_problem = _problem;
    }

    typedef void result_type;

    void define_type(stapl::typer& _t) {
      _t.member(m_cp);
      _t.member(m_problem);
    }

    template <typename NativeView, typename RepeatView>
      void operator()(NativeView _vw1, RepeatView _vw2) const;

  private:
    ConnectorPointer m_cp;
    MPProblemType* m_problem;
};

template<class MPTraits>
template<typename NativeView, typename RepeatView>
void
ConnectWF<MPTraits>::
operator()(NativeView _vw1, RepeatView _vw2) const {
  psbmp::PrintValue("Basic Parallel- Native View : " , _vw1.size());
  psbmp::PrintValue("Basic Parallel - Repeat View : " , _vw2.size());

  vector<VID> v1;
  vector<VID> v2;

  ///why not pass views directly to connector?
  for(typename NativeView::vertex_iterator vit1 = _vw1.begin(); vit1 != _vw1.end(); ++vit1) {
    v1.push_back((*vit1).descriptor());
  }

  for(typename RepeatView::vertex_iterator vit2 = _vw2.begin(); vit2 != _vw2.end(); ++vit2) {
    v2.push_back((*vit2).descriptor());
  }

  m_cp->Connect(m_problem->GetRoadmap(), v1.begin(), v1.end(), v2.begin(), v2.end());
}


////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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
    typedef typename MPProblemType::MapEvaluatorPointer MapEvaluatorPointer;

    BasicParallelPRM(const vector<pair<string, int> >& _samplerLabels = vector<pair<string, int> >(),
        const vector<string>& _connectorLabels = vector<string>(),
        const vector<string>& _evaluatorLabels = vector<string>());

    BasicParallelPRM(MPProblemType* _problem, XMLNode& _node);

    virtual ~BasicParallelPRM() {};

    virtual void Print(ostream& _os) const;
    virtual void ParseXML(XMLNode& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

  private:
    vector<pair<string, int> > m_samplerLabels;
    vector<string> m_connectorLabels;
    vector<string> m_evaluatorLabels;
};

template<class MPTraits>
BasicParallelPRM<MPTraits>::
BasicParallelPRM(const vector<pair<string, int> >& _samplerLabels,
    const vector<string>& _connectorLabels,
    const vector<string>& _evaluatorLabels) :
  m_samplerLabels(_samplerLabels),
  m_connectorLabels(_connectorLabels),
  m_evaluatorLabels(_evaluatorLabels) {
    this->SetName("BasicParallelPRM");
  }

template<class MPTraits>
BasicParallelPRM<MPTraits>::
BasicParallelPRM(MPProblemType* _problem, XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node) {
    this->SetName("BasicParallelPRM");
    ParseXML(_node);
  }


template<class MPTraits>
void
BasicParallelPRM<MPTraits>::
ParseXML(XMLNode& _node) {
  for(auto& child : _node) {
    if(child.Name() == "node_generation_method") {
      string node_gen_method = child.Read("Method", true, "",
          "Node Generation Method");
      int numPerIteration = child.Read("Number", true, 1, 0, MAX_INT,
          "Number of samples");
      m_samplerLabels.push_back(
          make_pair(node_gen_method, numPerIteration));
    }
    else if(child.Name() == "node_connection_method") {
      m_connectorLabels.push_back(
          child.Read("Method", true, "", "Node Connection Method"));
    }
    else if(child.Name() == "Evaluator") {
      string e = child.Read("method", "true", "", "Evaluator Label");
      m_evaluatorLabels.push_back(e);
    }
  }
}

template<class MPTraits>
void
BasicParallelPRM<MPTraits>::
Print(ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);
  typedef vector<pair<string, int> >::const_iterator VIter;
  typedef vector<string>::const_iterator StringIter;

  _os<<"\nSamplers\n";

  for(auto& label : m_samplerLabels)
    _os << "\t" << label.first << "\tNumber:" << label.second;

  _os<<"\nNodeConnectors\n";

  for(auto& label : m_connectorLabels) {
    _os << "\t" << label;
  }

  _os << "\nMapEvaluators\n";
  for(auto& label : m_evaluatorLabels) {
    _os << "\t" << label;
  }
}

template<class MPTraits>
void
BasicParallelPRM<MPTraits>::
Initialize() {
  // Reload map evaluators
  typedef vector<string>::const_iterator SIT;
  for(auto& sit : m_evaluatorLabels) {
    MapEvaluatorPointer evaluator = this->GetMapEvaluator(sit);
    if(evaluator->HasState())
      evaluator->operator()();
  }
}

template<class MPTraits>
void
BasicParallelPRM<MPTraits>::
Run() {
  GraphType* rmg = this->GetRoadmap()->GetGraph();

  stapl::counter<stapl::default_timer> t1,t2;

  double sample_timer=0.0, connect_timer=0.0 ;

  // Now, lets do this the correct way using a map evaluator

  bool done = this->EvaluateMap(m_evaluatorLabels);
  while(!done) {

    typedef vector<pair<string, int> >::iterator I;

    //---------------------------
    // Generate roadmap nodes
    //---------------------------

    cout << "Generation Phase" << endl;
    for(auto& sampler : m_samplerLabels) {
      SamplerPointer nodeGen = this->GetSampler(sampler.first);
      typedef stapl::array<CfgType> CfgArray;
      typedef stapl::array_view<CfgArray> ViewCfgArray;
      CfgArray pa(sampler.second);
      ViewCfgArray v(pa);

      t1.start();
      SampleWF<MPTraits> sampleWf(nodeGen, this->GetMPProblem());
      stapl::map_func(sampleWf,stapl::balance_view(v,stapl::get_num_locations()));
      sample_timer = t1.stop();

      if (this->m_debug)
        cout << "\n processor #----->[" << stapl::get_location_id() <<
          "] NodeGeneration time  = "  << sample_timer << endl;
    }
    stapl::rmi_fence();

    // Connect roadmap nodes
    cout << "Connecting Phase" << endl;
    typedef vector<string>::iterator J;
    for(auto& connectorLabel : m_connectorLabels) {
      if (this->m_debug)
        cout << "BasicParallelPRM::graph size " << rmg->size() << endl;

      ConnectorPointer connector = this->GetConnector(connectorLabel);
      typedef stapl::graph_view<GraphType> VType;
      VType view(*rmg);

      t2.start();
      ConnectWF<MPTraits> connWf(connector, this->GetMPProblem());
      stapl::map_func(connWf, stapl::native_view(view), stapl::make_repeat_view(view));
      connect_timer = t2.stop();
      if (this->m_debug) {
        cout << "\n processor #[" << stapl::get_location_id() <<
          "] NodeConnection time  = " << connect_timer << endl;
      }
    }

    stapl::rmi_fence();

    // Re-evaluate the roadmap
    cout << "Try to evaluate the map" << endl << flush;
    done = this->EvaluateMap(m_evaluatorLabels);
  }
}

template<class MPTraits>
void
BasicParallelPRM<MPTraits>::
Finalize() {
  if(this->m_debug)
    cout << "BasicParallelPRM::Finalize()";

  stapl::rmi_fence();

  // Write roadmap to file
  string str = this->GetBaseFilename() + ".map";
  this->GetRoadmap()->Write(str, this->GetEnvironment());

  stapl::rmi_fence();
}

#endif

