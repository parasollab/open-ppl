#ifndef UTILITY_GUIDED_GENERATOR_H_
#define UTILITY_GUIDED_GENERATOR_H_

#include "MPStrategyMethod.h"

#include <algorithm>


////////////////////////////////////////////////////////////////////////////////
/// @TODO
/// @ingroup MotionPlanningStrategyUtils
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
struct ApproximateCSpaceModel {

  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPTraits::MPLibrary MPLibrary;
  typedef typename MPLibrary::DistanceMetricPointer DistanceMetricPointer;

  typedef typename std::pair<CfgType, double> ModelPair;

  std::vector<ModelPair> m_modelNodes;
  DistanceMetricPointer m_dm;

  ApproximateCSpaceModel(DistanceMetricPointer _dm) : m_dm(_dm) {}
  ~ApproximateCSpaceModel() {}

  void AddSample(const CfgType& _c, double _coll) {
    m_modelNodes.emplace_back(_c, _coll);
  }

  double FreeProbability(const CfgType& _c, int _k) {
    // Make a comparator to sort the model nodes.
    auto comparator = [&_c, this](const ModelPair& _p1, const ModelPair& _p2)
    {
      return this->m_dm->Distance(_c, _p1.first)
           < this->m_dm->Distance(_c, _p2.first);
    };

    std::sort(m_modelNodes.begin(), m_modelNodes.end(), comparator);

    int size = min<int>(_k, m_modelNodes.size());
    if(size == 0)
      return 0.0;
    else
    {
      auto adder = [](const double _sum, const ModelPair& _p)
      {
        return std::plus<double>()(_sum, _p.second);
      };
      return std::accumulate(m_modelNodes.begin(), m_modelNodes.begin() + size,
          0., adder) / size;
    }
  }

};


////////////////////////////////////////////////////////////////////////////////
/// TODO
/// @todo Configure for pausible execution.
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class UtilityGuidedGenerator : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;

    UtilityGuidedGenerator(string _vcLabel = "", string _nfLabel = "",
        string _connector = "", vector<string> _evaluators = vector<string>(),
        double _componentDist = 0.5, double _tao = 0.01,
        int _kNeighbors = 10, int _kSamples = 5);
    UtilityGuidedGenerator(XMLNode& _node);
    virtual ~UtilityGuidedGenerator();

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    virtual void Initialize() {}
    virtual void Run();
    virtual void Finalize();

  protected:

    CfgType GenerateEntropyGuidedSample();

    //data
    string m_vcLabel, m_nfLabel, m_connectorLabel;
    double m_componentDist, m_tao;
    int m_kNeighbors, m_kSamples;
};


template <class MPTraits>
UtilityGuidedGenerator<MPTraits>::
UtilityGuidedGenerator(string _vcLabel, string _nfLabel, string _connector,
    vector<string> _evaluators, double _componentDist, double _tao,
    int _kNeighbors, int _kSamples) :
    m_vcLabel(_vcLabel), m_nfLabel(_nfLabel), m_connectorLabel(_connector),
    m_componentDist(_componentDist), m_tao(_tao),
    m_kNeighbors(_kNeighbors), m_kSamples(_kSamples) {
  this->m_meLabels = _evaluators;
  this->SetName("UtilityGuidedGenerator");
}

template <class MPTraits>
UtilityGuidedGenerator<MPTraits>::UtilityGuidedGenerator(XMLNode& _node) :
        MPStrategyMethod<MPTraits>(_node) {
  this->SetName("UtilityGuidedGenerator");
  ParseXML(_node);
}

template <class MPTraits>
UtilityGuidedGenerator<MPTraits>::~UtilityGuidedGenerator(){}

template <typename MPTraits>
void
UtilityGuidedGenerator<MPTraits>::ParseXML(XMLNode& _node) {
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Checker to verify validity of nodes.");
  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder used in approximate c-space model.");
  m_connectorLabel = _node.Read("connectorLabel", true, "", "Node Connector used for connecting nodes. ");

  m_componentDist = _node.Read("componentDist", false, 0.5, 0.0, MAX_DBL, "distance threshold between ccs");
  m_tao = _node.Read("tao", false, 0.01, 0.0, MAX_DBL, "perturb amount");
  m_kNeighbors = _node.Read("kneighbors", false, 10, 0, MAX_INT, "number of neighbors to look at when determining the probability a sample is free");
  m_kSamples = _node.Read("ksamples", false, 5, 0, MAX_INT, "number of samples to select from during each round");

  for(auto& child : _node)
    if(child.Name() == "Evaluator")
      this->m_meLabels.push_back(
          child.Read("label", true, "", "Evaluation Method"));
}

template <class MPTraits>
void
UtilityGuidedGenerator<MPTraits>::Print(ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);
  _os << "\tValidity Checker: " << m_vcLabel << endl;
  _os << "\tNeighborhood Finder: " << m_nfLabel << endl;
  _os << "\tComponent Distance: " << m_componentDist << endl;
  _os << "\tTao: " << m_tao << endl;
  _os << "\tKNeighbors: " << m_kNeighbors << endl;
  _os << "\tKSamples: " << m_kSamples << endl;
  _os << "\tNode Connector: " << m_connectorLabel << endl;
  _os << "\tEvaluators\n";
  for(auto& l : this->m_meLabels)
    _os << "\t" << l;
  _os << endl;
}


template <class MPTraits>
void
UtilityGuidedGenerator<MPTraits>::Run() {
  if(this->m_debug) cout << "\nRunning UtilityGuidedGenerator::" << endl;

  //setup variables
  StatClass* stats = this->GetStatClass();
  RoadmapType* rmap = this->GetRoadmap();
  Environment* env = this->GetEnvironment();
  auto bb = env->GetBoundary();

  auto dm = this->GetDistanceMetric(this->GetNeighborhoodFinder(m_nfLabel)->
      GetDMLabel());
  ApproximateCSpaceModel<MPTraits> model(dm);

  auto vcm = this->GetValidityChecker(m_vcLabel);
  string callee(this->GetNameAndLabel() + "::Run()");

  auto connector = this->GetConnector(m_connectorLabel);


  stats->StartClock("Map Generation");

  bool mapPassedEvaluation = false;
  while(!mapPassedEvaluation) {

    CfgType q(this->GetTask()->GetRobot());

    //if roadmap empty, simply add a free random sample
    if(rmap->GetGraph()->get_num_vertices() < 1) {
      stats->StartClock("Total Sampling Time");
      bool inBBX = false, isValid = false;
      while(!inBBX && !isValid) {
        stats->IncNodesAttempted(this->GetNameAndLabel());
        q.GetRandomCfg(bb);
        inBBX = q.InBounds(bb);
        if(inBBX) {
          isValid = vcm->IsValid(q, callee);
          if(isValid)
            stats->IncNodesGenerated(this->GetNameAndLabel());
        }
      }
      if(this->m_debug) cout << "roadmap empty, adding free random sample\n";
      model.AddSample(q, 1);
      rmap->GetGraph()->AddVertex(q);
      stats->StopClock("Total Sampling Time");

    }
    else {
      stats->StartClock("Total Sampling Time");
      //generate a entropy guided sample
      q = GenerateEntropyGuidedSample();
      double qProbFree = model.FreeProbability(q, m_kNeighbors);
      if(this->m_debug) cout << "q (" << qProbFree << ") = " << q << endl;

      for(int j=1; j<m_kSamples; ++j) {
        CfgType qNew = GenerateEntropyGuidedSample();
        double qNewProbFree = model.FreeProbability(qNew, m_kNeighbors);
        if(this->m_debug) cout << "\tq_" << j << " (" << qNewProbFree << ") = " << qNew << endl;

        if(qNewProbFree > qProbFree) {
          if(this->m_debug) cout << "\tprobability greater, swapping to q_" << j << endl;
          q = qNew;
          qProbFree = qNewProbFree;
        }
        else
          if(this->m_debug) cout << "\tprobablity not greater, keeping q\n";
      }
      if(this->m_debug) cout << "q (" << qProbFree << ") = " << q << endl;

      //add the sample to the model and roadmap (if free)
      stats->IncNodesAttempted(this->GetNameAndLabel());
      stats->StopClock("Total Sampling Time");
      bool isColl = !q.InBounds(bb) || !vcm->IsValid(q, callee);
      if(!isColl) {
        if(this->m_debug) cout << "valid, adding to roadmap and model\n";
        stats->IncNodesGenerated(this->GetNameAndLabel());

        model.AddSample(q, 1);
        VID qvid = rmap->GetGraph()->AddVertex(q);

        //connect sample
        stats->StartClock("Total Connection Time");
        connector->Connect(rmap, qvid);
        stats->StopClock("Total Connection Time");
        if(this->m_debug) cout << "connecting sample, roadmap now has " << rmap->GetGraph()->get_num_vertices() << " nodes and " << rmap->GetGraph()->get_num_edges() << " edges\n";

      }
      else {
        if(this->m_debug) cout << "invalid, adding to model only\n";
        model.AddSample(q, 0);
      }
      if(this->m_debug) cout << endl;
    }

    mapPassedEvaluation = this->EvaluateMap();
  }

  stats->StopClock("Map Generation");
  if(this->m_debug) stats->PrintClock("Map Generation", cout);

  if(this->m_debug) cout << "\nEnd Running UtilityGuidedGenerator::" << endl;
}

template <class MPTraits>
void
UtilityGuidedGenerator<MPTraits>::Finalize() {
  if(this->m_debug) cout<<"\nFinalizing UtilityGuidedGenerator::"<<endl;

  //output final map
  this->GetRoadmap()->Write(this->GetBaseFilename() + ".map", this->GetEnvironment());

  //output stats
  string str = this->GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  osStat << "NodeGen+Connection Stats" << endl;
  StatClass* stats = this->GetStatClass();
  stats->PrintAllStats(osStat, this->GetRoadmap());
  stats->PrintClock("Map Generation", osStat);
  osStat.close();

  if(this->m_debug) cout << "\nEnd Finalizing UtilityGuidedGenerator" << endl;
}

template <class MPTraits>
typename MPTraits::CfgType
UtilityGuidedGenerator<MPTraits>::
GenerateEntropyGuidedSample() {
  RoadmapType* rmap = this->GetRoadmap();
  Environment* env = this->GetEnvironment();
  auto bb = env->GetBoundary();
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  auto dm = this->GetDistanceMetric(nf->GetDMLabel());
  auto robot = this->GetTask()->GetRobot();

  CfgType q1(robot), q2(robot);

  stapl::sequential::vector_property_map<GraphType, size_t > cmap;
  vector<pair<size_t, VID> > ccs;
  if(get_cc_stats(*rmap->GetGraph(), cmap, ccs) == 1) {
    int index = (int)floor((double)DRand()*(double)rmap->GetGraph()->get_num_vertices());
    q1 = (rmap->GetGraph()->begin() + index)->property();
    q2.GetRandomCfg(bb);
    if(this->m_debug)
      cout << "\t\tIn GenerateEntropyGuidedSample: only 1 cc, randomly selected vertex " << (rmap->GetGraph()->begin() + index)->descriptor() << " and a random sample\n";
  }
  else {
    //randomly select 2 ccs that are within a threshold m_componentDist of each other
    if(this->m_debug)
      cout << "\t\tIn GenerateEntropyGuidedSample: there are " << ccs.size() << " ccs, looking for a pair less than " << m_componentDist << " apart\n";
    VID cc1Vid, cc2Vid, minCC1Vid = -1, minCC2Vid = -1;
    vector<VID> cc1, cc2;
    double dist = 0, minDist = 0;
    int possibleCombos = 1;
    for(size_t i=ccs.size(); i>2; --i)
      possibleCombos *= i;
    do {
      cc1Vid = ccs[(int)floor((double)DRand()*(double)ccs.size())].second;
      do {
	cc2Vid = ccs[(int)floor((double)DRand()*(double)ccs.size())].second;
      }
      while (cc1Vid == cc2Vid);

      cmap.reset();
      get_cc(*rmap->GetGraph(), cmap, cc1Vid, cc1);
      cmap.reset();
      get_cc(*rmap->GetGraph(), cmap, cc2Vid, cc2);

      vector<pair<pair<VID, VID>, double> > kp;
      nf->FindNeighborPairs(rmap, cc1.begin(), cc1.end(), cc2.begin(), cc2.end(), back_inserter(kp));
      dist = kp[0].second;

      if(this->m_debug)
        cout << "\t\t\tdist between cc " << cc1Vid << " and " << cc2Vid << " is " << dist << endl;
      if(dist < minDist) {
        minCC1Vid = cc1Vid;
        minCC2Vid = cc2Vid;
      }
    }
    while (dist > m_componentDist && possibleCombos-- > 0);
    if(dist <= m_componentDist) {
      cc1Vid = minCC1Vid;
      cc2Vid = minCC2Vid;

      cmap.reset();
      get_cc(*rmap->GetGraph(), cmap, cc1Vid, cc1);
      cmap.reset();
      get_cc(*rmap->GetGraph(), cmap, cc2Vid, cc2);
    }
    if(this->m_debug)
      cout << "\t\t\tselected cc pair " << cc1Vid << " and " << cc2Vid << endl;

    //randomly select a node in each cc
    q1 = rmap->GetGraph()->GetVertex(cc1[(int)floor(DRand()*cc1.size())]);
    q2 = rmap->GetGraph()->GetVertex(cc2[(int)floor(DRand()*cc2.size())]);
  }

  //return perturbation of the midpoint between the two nodes
  CfgType qn = (q1 + q2)/2;
  CfgType ray = qn;
  ray.GetRandomRay(DRand()*m_tao, dm);
  return qn + ray;
}

#endif
