#ifndef DMTESTSTRATEGY_H_
#define DMTESTSTRATEGY_H_

#include "MPStrategies/MPStrategyMethod.h"

template <class MPTraits>
class DMTestStrategy : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType::RoadmapType RoadmapType;

    DMTestStrategy();
    DMTestStrategy(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~DMTestStrategy();

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _out) const;

    virtual void Initialize(){}
    virtual void Run();
    virtual void Finalize(){}

  private:
    string m_inputRoadmapFilename;
    RoadmapType* m_rdmp;
    string m_dmMethod;
    size_t m_numToVerify;
};


template <class MPTraits>
DMTestStrategy<MPTraits>::DMTestStrategy() : MPStrategyMethod<MPTraits>(),
  m_rdmp(NULL), m_numToVerify(0) {
    this->SetName("DMTest");
  }

template <class MPTraits>
DMTestStrategy<MPTraits>::
DMTestStrategy(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node)
  : MPStrategyMethod<MPTraits>(_problem, _node), m_rdmp(NULL) {
    this->SetName("DMTest");
    ParseXML(_node);
  }

template <class MPTraits>
DMTestStrategy<MPTraits>::
~DMTestStrategy()
{}

template <class MPTraits>
void
DMTestStrategy<MPTraits>::
ParseXML(XMLNodeReader& _node)
{
  m_inputRoadmapFilename = _node.stringXMLParameter("input_roadmap", false, "", "filename of input roadmap, if none provded, uses current one from MPProblem");
  m_dmMethod = _node.stringXMLParameter("dm_method", true, "", "distance metric label");
  m_numToVerify = _node.numberXMLParameter("num_to_verify", false, MAX_INT, 0, MAX_INT, "number of nodes to verify distances");
  _node.warnUnrequestedAttributes();
}

template <class MPTraits>
void
DMTestStrategy<MPTraits>::PrintOptions(ostream& _out) const {
  _out << "DMTestStrategy ::  m_inputRoadmapFilename = \"" << m_inputRoadmapFilename
    << "\"\tm_dmMethod = " << m_dmMethod
    << "\tm_numToVerify = " << m_numToVerify << endl;
}

struct less_second : public binary_function<pair<size_t, double>, pair<size_t, double>, bool> {
  bool operator()(const pair<size_t, double>& p1, const pair<size_t, double>& p2) const {
    return p1.second < p2.second;
  }
};

template <class MPTraits>
void
DMTestStrategy<MPTraits>::
Run()
{
  PrintOptions(cout);

  if(m_inputRoadmapFilename == "") {
    m_rdmp = this->GetMPProblem()->GetRoadmap();
  } else {
    m_rdmp = new RoadmapType();
    m_rdmp->Read(m_inputRoadmapFilename.c_str());
  }

  ClockClass clock;
  StatClass *stats = this->GetMPProblem()->GetStatClass();
  stats->StartClock("Distance Metric");

  typename MPTraits::MPProblemType::DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmMethod);
  dm->PrintOptions(cout);
  cout << endl;

  size_t numVerified = 0;
  typedef typename RoadmapType::GraphType GraphType;
  GraphType* g = m_rdmp->GetGraph();
  for(typename GraphType::VI vi = g->begin();
      numVerified < m_numToVerify && vi != g->end();
      ++vi, ++numVerified){
    stats->StartClock("Iteration");
    cout << "testing distances to node: " << g->GetVertex(vi) << endl;
    vector<pair<size_t, double> > d;
    for(typename GraphType::VI vi2 = g->begin(); vi2!=g->end(); ++vi2){
      d.push_back(make_pair(distance(g->begin(), vi2),
            dm->Distance(this->GetMPProblem()->GetEnvironment(),
              g->GetVertex(vi), g->GetVertex(vi2))));
      cout << "\t" << d.back().second << endl;
    }
    cout << endl;
    sort(d.begin(), d.end(), less_second());
    cout << "sorted indices:";
    for(vector<pair<size_t, double> >::const_iterator D = d.begin(); D != d.end(); ++D)
      cout << " " << D->first;
    cout << endl;
    stats->StopPrintClock("Iteration", cout);
  }

  stats->StopClock("Distance Metric");
  cout << ":" << stats->GetSeconds("Distance Metric") << " sec (ie, " << stats->GetUSeconds("Distance Metric") << " usec)";
  cout << endl;

  if(g->get_num_vertices() > 1) {
    typename GraphType::VI vi = g->begin(), vi2 = vi+1;
    typename MPTraits::CfgRef origin = g->GetVertex(vi);
    typename MPTraits::CfgType c = g->GetVertex(vi2);
    double dist = dm->Distance(this->GetMPProblem()->GetEnvironment(), origin, c);
    cout << "\nScale Cfg: 1/2x\n\torigin = " << origin << "\n\tc = " << c << "\n\tscaled distance = " << dist * 0.5 << endl;
    dm->ScaleCfg(this->GetMPProblem()->GetEnvironment(), dist * 0.5, origin, c);
    cout << "\n\tc' = " << c << "\n\tnew distance = " << dm->Distance(this->GetMPProblem()->GetEnvironment(), origin, c) << endl;
    c = g->GetVertex(vi2);
    cout << "\nScale Cfg: 2x\n\torigin = " << origin << "\n\tc = " << c << "\n\tscaled distance = " << dist * 2 << endl;
    dm->ScaleCfg(this->GetMPProblem()->GetEnvironment(), dist * 2, origin, c);
    cout << "\n\tc' = " << c << "\n\tnew distance = " << dm->Distance(this->GetMPProblem()->GetEnvironment(), origin, c) << endl;
  }
}

#endif
