#include "DMTestStrategy.h"
#include "DistanceMetrics.h"

DMTestStrategy::
DMTestStrategy(XMLNodeReader& _node, MPProblem* _problem)
 : MPStrategyMethod(_node, _problem)
{
  ParseXML(_node);
}

DMTestStrategy::
~DMTestStrategy()
{}

void
DMTestStrategy::
ParseXML(XMLNodeReader& _node)
{
  m_inputRoadmapFilename = _node.stringXMLParameter("input_roadmap", false, "", "filename of input roadmap, if none provded, uses current one from MPProblem");
  m_dmMethod = _node.stringXMLParameter("dm_method", true, "", "distance metric label");
  m_numToVerify = _node.numberXMLParameter("num_to_verify", false, MAX_INT, 0, MAX_INT, "number of nodes to verify distances");
  _node.warnUnrequestedAttributes();
}

void
DMTestStrategy::
PrintOptions(ostream& _out)
{
  _out << "DMTestStrategy ::  m_inputRoadmapFilename = \"" << m_inputRoadmapFilename 
    << "\"\tm_dmMethod = " << m_dmMethod 
    << "\tm_numToVerify = " << m_numToVerify << endl;
}

struct less_second : public binary_function<pair<size_t, double>, pair<size_t, double>, bool> {
  bool operator()(const pair<size_t, double>& p1, const pair<size_t, double>& p2) const {
    return p1.second < p2.second;
  }
};

void
DMTestStrategy::
Run()
{
  PrintOptions(cout);

  if(m_inputRoadmapFilename == "") {
    m_rdmp = GetMPProblem()->GetRoadmap();
  } else {
    m_rdmp = new Roadmap<CfgType, WeightType>();
    m_rdmp->SetEnvironment(GetMPProblem()->GetEnvironment());
    m_rdmp->ReadRoadmapGRAPHONLY(m_inputRoadmapFilename.c_str());
  }

  ClockClass clock;
  StatClass *stats = GetMPProblem()->GetStatClass();
  stats->StartClock("Distance Metric");

  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dmMethod);
  dm->PrintOptions(cout);
  cout << endl;

  vector<CfgType> nodes;
  m_rdmp->m_pRoadmap->GetVerticesData(nodes);

  m_numToVerify = min(m_numToVerify, nodes.size());
  for(size_t i=0; i<m_numToVerify; ++i) {
    stats->StartClock("Iteration");
    cout << "testing distances to node " << i << ": " << nodes[i] << endl;
    vector<pair<size_t, double> > d;
    for(vector<CfgType>::iterator N = nodes.begin(); N != nodes.end(); ++N) {
      d.push_back(make_pair(distance(nodes.begin(), N), dm->Distance(m_rdmp->GetEnvironment(), nodes[i], *N)));
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

  if(nodes.size() > 1) {
    CfgType origin(nodes[0]);
    CfgType c(nodes[1]);
    cout << "\nScale Cfg: 1/2x\n\torigin = " << origin << "\n\tc = " << c << "\n\tscaled distance = " << dm->Distance(m_rdmp->GetEnvironment(), nodes[0], nodes[1]) * 0.5 << endl;
    dm->ScaleCfg(m_rdmp->GetEnvironment(), dm->Distance(m_rdmp->GetEnvironment(), nodes[0], nodes[1]) * 0.5, origin, c);
    cout << "\n\tc' = " << c << "\n\tnew distance = " << dm->Distance(m_rdmp->GetEnvironment(), origin, c) << endl;
    origin = nodes[0];
    c = nodes[1];
    cout << "\nScale Cfg: 2x\n\torigin = " << origin << "\n\tc = " << c << "\n\tscaled distance = " << dm->Distance(m_rdmp->GetEnvironment(), nodes[0], nodes[1]) * 2 << endl;
    dm->ScaleCfg(m_rdmp->GetEnvironment(), dm->Distance(m_rdmp->GetEnvironment(), nodes[0], nodes[1]) * 2, origin, c);
    cout << "\n\tc' = " << c << "\n\tnew distance = " << dm->Distance(m_rdmp->GetEnvironment(), origin, c) << endl;
  }
}
 
