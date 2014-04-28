#ifndef LPCOMPARE_H_
#define LPCOMPARE_H_

#include "boost/tuple/tuple.hpp"
using boost::tuple;

#include "MPStrategyMethod.h"

template<class MPTraits>
class LPCompare : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType::GraphType GraphType;

    LPCompare<MPTraits>();
    LPCompare(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

  private:
    tuple<double, double, double> CompareEdge(CfgType& _s, CfgType& _g, WeightType& _w1, WeightType& _w2);
    double ComparePaths(vector<CfgType>& _p1, vector<CfgType>& _p2);
    double ComparePaths2(vector<CfgType>& _p1, vector<CfgType>& _p2);

    string m_rdmp1in, m_rdmp2in, m_dmLabel, m_lpLabel1, m_lpLabel2;
    Roadmap<MPTraits>* m_rdmp1,* m_rdmp2;

    double m_q1, m_q2, m_q3;
    size_t m_numSimilar, m_numOnlyInR1, m_numOnlyInR2;
};


template<class MPTraits>
LPCompare<MPTraits>::LPCompare() {
  this->SetName("LPCompare");
}

template<class MPTraits>
LPCompare<MPTraits>::LPCompare(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node) {
    this->SetName("LPCompare");
    m_rdmp1in = _node.stringXMLParameter("rdmp1", true, "", "Roadmap file 1");
    m_rdmp2in = _node.stringXMLParameter("rdmp2", true, "", "Roadmap file 2");
    m_lpLabel1 = _node.stringXMLParameter("lpLabel1", true, "", "LP for Roadmap 1");
    m_lpLabel2 = _node.stringXMLParameter("lpLabel2", true, "", "LP for Roadmap 2");
    m_dmLabel = _node.stringXMLParameter("dmLabel", true, "", "DM for similarity");
  }

template<class MPTraits>
void
LPCompare<MPTraits>::Initialize() {
  m_rdmp1 = new Roadmap<MPTraits>();
  m_rdmp1->Read(m_rdmp1in);

  m_rdmp2 = new Roadmap<MPTraits>();
  m_rdmp2->Read(m_rdmp2in);
}

template<class MPTraits>
void
LPCompare<MPTraits>::Run() {
  GraphType* g1 = m_rdmp1->GetGraph();
  GraphType* g2 = m_rdmp2->GetGraph();

  cout << "LPCompare: " << endl
    << "RDMP1: " << g1->get_num_vertices() << " " << g1->get_num_edges() << endl
    << "RDMP2: " << g2->get_num_vertices() << " " << g2->get_num_edges() << endl;

  m_numSimilar = 0;

  //for each edge in g1
  //  find corresponding edge in g2
  //  quality compare the two edges and add to average
  typedef typename GraphType::edge_iterator EI;
  typedef typename GraphType::EI AEI;
  for(EI ei1 = g1->edges_begin(); ei1 != g1->edges_end(); ++ei1) {
    AEI ei2;
    if(g2->IsEdge((*ei1).source(), (*ei1).target(), ei2)) {
      m_numSimilar++;

      //compare quality
      WeightType& w1 = (*ei1).property();
      WeightType& w2 = (*ei2).property();
      tuple<double, double, double> q = CompareEdge(
          g1->GetVertex((*ei1).source()),
          g1->GetVertex((*ei1).target()),
          w1, w2);
      m_q1 += q.get<0>();
      m_q2 += q.get<1>();
      m_q3 += q.get<2>();
    }
    else {
      //not an edge
    }
  }

  m_q1 /= m_numSimilar;
  m_q2 /= m_numSimilar;
  m_q3 /= m_numSimilar;
  m_numOnlyInR1 = g1->get_num_edges() - m_numSimilar;
  m_numOnlyInR2 = g2->get_num_edges() - m_numSimilar;

  cout << "LP Similarity: " << m_q1 << " " << m_q2 << " " << m_q3 << endl;
  cout << "Num Similar: " << m_numSimilar << endl;
  cout << "Num Only In R1: " << m_numOnlyInR1 << endl;
  cout << "Num Only In R2: " << m_numOnlyInR2 << endl;
}

template<class MPTraits>
void
LPCompare<MPTraits>::Finalize() {
  //setup variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  string str;

  //output stats
  str = this->GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  osStat << "NodeGen+Connection Stats" << endl;
  stats->PrintAllStats(osStat, this->GetMPProblem()->GetRoadmap());
  osStat << "LPSimilarity: " << m_q1 << " " << m_q2 << " " << m_q3 << endl;
  osStat << "NumSimilar: " << m_numSimilar << endl;
  osStat << "NumOnlyInR1: " << m_numOnlyInR1 << endl;
  osStat << "NumOnlyInR2: " << m_numOnlyInR2 << endl;
}

template<class MPTraits>
tuple<double, double, double>
LPCompare<MPTraits>::CompareEdge(CfgType& _s, CfgType& _g, WeightType& _w1, WeightType& _w2) {
  Environment* env = this->GetMPProblem()->GetEnvironment();
  double posRes = env->GetPositionRes();
  double oriRes = env->GetOrientationRes();

  tuple<double, double, double> q;

  //collect paths
  typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
  vector<CfgType> p1(1, _s), p2(1, _s);

  //path on edge 1
  LocalPlannerPointer lp1 = this->GetMPProblem()->GetLocalPlanner(m_lpLabel1);
  p1 = lp1->ReconstructPath(_s, _g, _w1.GetIntermediates(), posRes, oriRes);
  p1.push_back(_g);

  //path on edge 2
  LocalPlannerPointer lp2 = this->GetMPProblem()->GetLocalPlanner(m_lpLabel2);
  p2 = lp2->ReconstructPath(_s, _g, _w2.GetIntermediates(), posRes, oriRes);
  p2.push_back(_g);

  //compare distance between corresponding intermediates to get metrics
  q.get<0>() = ComparePaths(p1, p2);
  q.get<1>() = ComparePaths(p2, p1);
  q.get<2>() = ComparePaths2(p1, p2);

  return q;
}

template<class MPTraits>
double
LPCompare<MPTraits>::ComparePaths(vector<CfgType>& _p1, vector<CfgType>& _p2) {
  typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);

  //for each cfg in p1
  //  find corresponding cfg in p2
  //  add to average distance

  double dist = 0;
  for(size_t i = 0; i < _p1.size(); ++i) {
    size_t j = double(i)/_p1.size()*_p2.size();
    dist += dm->Distance(_p1[i], _p2[j]);
  }

  dist /= _p1.size();
  return dist;
}

template<class MPTraits>
double
LPCompare<MPTraits>::ComparePaths2(vector<CfgType>& _p1, vector<CfgType>& _p2) {
  typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);

  double dist = 0;
  for(size_t i = 0; i < _p1.size(); ++i) {
    size_t j = double(i)/_p1.size()*_p2.size();
    dist += dm->Distance(_p1[i], _p2[j]);
  }
  for(size_t i = 0; i < _p2.size(); ++i) {
    size_t j = double(i)/_p2.size()*_p1.size();
    dist += dm->Distance(_p2[i], _p1[j]);
  }


  dist /= (_p1.size() + _p2.size());
  return dist;
}

#endif

