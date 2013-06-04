#ifndef UTILITYGUIDEDGENERATOR_H_
#define UTILITYGUIDEDGENERATOR_H_

#include "MPStrategyMethod.h"

template <class MPTraits>
struct ApproximateCSpaceModel {
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPTraits::MPProblemType::DistanceMetricPointer DistanceMetricPointer;
  
  vector<pair<CfgType, double> > m_modelNodes;
  Environment* m_env;
  DistanceMetricPointer m_dm;

  ApproximateCSpaceModel(Environment* _env, DistanceMetricPointer _dm) : m_env(_env), m_dm(_dm) {}
  ~ApproximateCSpaceModel() {}

  void AddSample(const CfgType& _c, double _coll) {
    m_modelNodes.push_back(make_pair(_c, _coll));
  }

  double FreeProbability(const CfgType& _c, int _k) {
    sort(m_modelNodes.begin(), m_modelNodes.end(), 
	 DistanceCompareFirst<MPTraits, pair<CfgType,double> >(m_env, m_dm, _c));
    int size = (int)min(_k, m_modelNodes.size());
    if(size == 0)
      return 0.0;
    else
      return accumulate(m_modelNodes.begin(), m_modelNodes.begin() + size,
			0.0, PlusSecond<pair<CfgType,double> >()) / size;
  }
};


template <class MPTraits>
class UtilityGuidedGenerator : public MPStrategyMethod<MPTraits> {
 public:
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPProblemType::RoadmapType RoadmapType;
  typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
  typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
  typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
  typedef typename MPProblemType::VID VID;
  typedef typename MPProblemType::GraphType::GRAPH GRAPH;
  typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
  
  UtilityGuidedGenerator(string _vcLabel = "", string _nfLabel = "", 
        string _connector = "", vector<string> _evaluators = vector<string>(), 
        double _componentDist = 0.5, double _tao = 0.01, 
        int _kNeighbors = 10, int _kSamples = 5);
  UtilityGuidedGenerator(MPProblemType* _problem, XMLNodeReader& _node);
  virtual ~UtilityGuidedGenerator();

  virtual void ParseXML(XMLNodeReader& _node);
  virtual void PrintOptions(ostream& _os);

  virtual void Initialize();
  virtual void Run();
  virtual void Finalize();

 protected:
  CfgType GenerateEntropyGuidedSample();

  //data
  string m_vcLabel, m_nfLabel, m_connectorLabel;
  vector<string> m_evaluatorLabels;
  double m_componentDist, m_tao;
  int m_kNeighbors, m_kSamples;
};


template <class MPTraits>
UtilityGuidedGenerator<MPTraits>::UtilityGuidedGenerator(string _vcLabel, string _nfLabel, 
        string _connector, vector<string> _evaluators, double _componentDist, double _tao, 
        int _kNeighbors, int _kSamples) :
        m_vcLabel(_vcLabel), m_nfLabel(_nfLabel), m_connectorLabel(_connector), 
        m_evaluatorLabels(_evaluators), m_componentDist(_componentDist), m_tao(_tao), 
        m_kNeighbors(_kNeighbors), m_kSamples(_kSamples) {
  this->SetName("UtilityGuidedGenerator");
}

template <class MPTraits>
UtilityGuidedGenerator<MPTraits>::UtilityGuidedGenerator(MPProblemType* _problem, XMLNodeReader& _node) :
        MPStrategyMethod<MPTraits>(_problem, _node) {
  this->SetName("UtilityGuidedGenerator");
  ParseXML(_node);
}

template <class MPTraits>
UtilityGuidedGenerator<MPTraits>::~UtilityGuidedGenerator(){}

template<class MPTraits>
void
UtilityGuidedGenerator<MPTraits>::ParseXML(XMLNodeReader& _node) {
  m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Checker to verify validity of nodes.");
  m_nfLabel = _node.stringXMLParameter("nfLabel", true, "", "Neighborhood Finder used in approximate c-space model.");
  m_connectorLabel = _node.stringXMLParameter("connectorLabel", true, "", "Node Connector used for connecting nodes. ");

  m_componentDist = _node.numberXMLParameter("componentDist", false, 0.5, 0.0, MAX_DBL, "distance threshold between ccs");
  m_tao = _node.numberXMLParameter("tao", false, 0.01, 0.0, MAX_DBL, "perturb amount");
  m_kNeighbors = _node.numberXMLParameter("kneighbors", false, 10, 0, MAX_INT, "number of neighbors to look at when determining the probability a sample is free");
  m_kSamples = _node.numberXMLParameter("ksamples", false, 5, 0, MAX_INT, "number of samples to select from during each round");

  for(XMLNodeReader::childiterator cIter = _node.children_begin(); cIter != _node.children_end(); ++cIter) {
    if(cIter->getName() == "Evaluator") {
      string evalMethod = cIter->stringXMLParameter("label", true, "", "Evaluation Method");
      m_evaluatorLabels.push_back(evalMethod);
      cIter->warnUnrequestedAttributes();
    }
    else
      cIter->warnUnknownNode();
  }
}

template <class MPTraits>
void
UtilityGuidedGenerator<MPTraits>::PrintOptions(ostream& _os) {
  MPStrategyMethod<MPTraits>::PrintOptions(_os);
  _os << "\tValidity Checker: " << m_vcLabel << endl;
  _os << "\tNeighborhood Finder: " << m_nfLabel << endl;
  _os << "\tComponent Distance: " << m_componentDist << endl;
  _os << "\tTao: " << m_tao << endl;
  _os << "\tKNeighbors: " << m_kNeighbors << endl;
  _os << "\tKSamples: " << m_kSamples << endl;
  _os << "\tNode Connector: " << m_connectorLabel << endl;
  _os << "\tEvaluators\n";
  for(vector<string>::const_iterator E = m_evaluatorLabels.begin(); E != m_evaluatorLabels.end(); ++E)
    _os << "\t" << *E;
  _os << endl;
}

template <class MPTraits>
void
UtilityGuidedGenerator<MPTraits>::Initialize() {
  if(this->m_debug) cout << "\nInitializing UtilityGuidedGenerator::" << endl;
  if(this->m_debug) cout << "\nEnding Initializing UtilityGuidedGenerator::" << endl;
}

template <class MPTraits>
void 
UtilityGuidedGenerator<MPTraits>::Run() {
  if(this->m_debug) cout << "\nRunning UtilityGuidedGenerator::" << endl;

  //setup variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  RoadmapType* rmap = this->GetMPProblem()->GetRoadmap();
  Environment* env = this->GetMPProblem()->GetEnvironment();
  shared_ptr<Boundary> bb = env->GetBoundary();
  
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(m_nfLabel)->GetDMMethod();
  ApproximateCSpaceModel<MPTraits> model(env, dm);
  
  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  string callee(this->GetNameAndLabel() + "::Run()");
  CDInfo cdInfo;

  ConnectorPointer connector = this->GetMPProblem()->GetConnector(m_connectorLabel);
 

  stats->StartClock("Map Generation");

  bool mapPassedEvaluation = false;
  while(!mapPassedEvaluation) {

    CfgType q;
    
    //if roadmap empty, simply add a free random sample
    if(rmap->GetGraph()->get_num_vertices() < 1) {
      stats->StartClock("Total Sampling Time");
      bool inBBX = false, isValid = false;
      while(!inBBX && !isValid) {
        stats->IncNodesAttempted(this->GetNameAndLabel());
        q.GetRandomCfg(env, bb);
        inBBX = env->InBounds(q, bb);
        if(inBBX) {
          isValid = vcm->IsValid(q, env, *stats, cdInfo, &callee);
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
      bool isColl = !env->InBounds(q, bb) || !vcm->IsValid(q, env, *stats, cdInfo, &callee);
      if(!isColl) {
        if(this->m_debug) cout << "valid, adding to roadmap and model\n";
        stats->IncNodesGenerated(this->GetNameAndLabel());
        
        model.AddSample(q, 1);
        VID qvid = rmap->GetGraph()->AddVertex(q);
        
        //connect sample
        vector<VID> v1(1, qvid);
        stapl::sequential::vector_property_map<GRAPH, size_t> cmap;
        stats->StartClock("Total Connection Time");
        connector->Connect(rmap, *stats, cmap, v1.begin(), v1.end());
        stats->StopClock("Total Connection Time");
        if(this->m_debug) cout << "connecting sample, roadmap now has " << rmap->GetGraph()->get_num_vertices() << " nodes and " << rmap->GetGraph()->get_num_edges() << " edges\n";

      } 
      else {
        if(this->m_debug) cout << "invalid, adding to model only\n";
        model.AddSample(q, 0);
      }
      if(this->m_debug) cout << endl;
    }
  
    mapPassedEvaluation = this->EvaluateMap(m_evaluatorLabels);
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
  string str = this->GetBaseFilename() + ".map";
  ofstream osMap(str.c_str());
  this->GetMPProblem()->GetRoadmap()->Write(osMap, this->GetMPProblem()->GetEnvironment());
  osMap.close();
  
  //output stats
  str = this->GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  osStat << "NodeGen+Connection Stats" << endl;
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  stats->PrintAllStats(osStat, this->GetMPProblem()->GetRoadmap());
  stats->PrintClock("Map Generation", osStat);
  osStat.close();
  
  if(this->m_debug) cout << "\nEnd Finalizing UtilityGuidedGenerator" << endl;
}

template <class MPTraits>
typename MPTraits::CfgType
UtilityGuidedGenerator<MPTraits>::
GenerateEntropyGuidedSample() {
  RoadmapType* rmap = this->GetMPProblem()->GetRoadmap();
  Environment* env = this->GetMPProblem()->GetEnvironment();
  shared_ptr<Boundary> bb = env->GetBoundary();
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nfLabel);
  DistanceMetricPointer dm = nf->GetDMMethod();
  
  CfgType q1, q2;
  
  stapl::sequential::vector_property_map<GRAPH, size_t > cmap;
  vector<pair<size_t, VID> > ccs;
  if(get_cc_stats(*rmap->GetGraph(), cmap, ccs) == 1) {
    int index = (int)floor((double)DRand()*(double)rmap->GetGraph()->get_num_vertices());
    q1 = (rmap->GetGraph()->begin() + index)->property();
    q2.GetRandomCfg(env, bb);
    if(this->m_debug) 
      cout << "\t\tIn GenerateEntropyGuidedSample: only 1 cc, randomly selected vertex " << (rmap->GetGraph()->begin() + index)->descriptor() << " and a random sample\n";
  } 
  else {
    //randomly select 2 ccs that are within a threshold m_componentDist of each other
    if(this->m_debug) 
      cout << "\t\tIn GenerateEntropyGuidedSample: there are " << ccs.size() << " ccs, looking for a pair less than " << m_componentDist << " apart\n";
    VID cc1Vid, cc2Vid, minCC1Vid, minCC2Vid;
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
      
      vector<pair<VID, VID> > kp(1);
      nf->KClosestPairs(rmap, cc1.begin(), cc1.end(), cc2.begin(), cc2.end(), 1, kp.begin());
      dist = dm->Distance(env, 
			  (*(rmap->GetGraph()->find_vertex(kp[0].first))).property(),
			  (*(rmap->GetGraph()->find_vertex(kp[0].second))).property());
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
    q1 = (*(rmap->GetGraph()->find_vertex(cc1[(int)floor((double)DRand()*(double)cc1.size())]))).property();
    q2 = (*(rmap->GetGraph()->find_vertex(cc2[(int)floor((double)DRand()*(double)cc2.size())]))).property();
  }

  //return perturbation of the midpoint between the two nodes
  CfgType qn = (q1 + q2)/2;
  CfgType ray = qn;
  ray.GetRandomRay(DRand()*m_tao, env, dm);
  return qn + ray;
}

#endif

