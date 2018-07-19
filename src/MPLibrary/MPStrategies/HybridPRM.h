#ifndef HYBRID_PRM_H_
#define HYBRID_PRM_H_

#include "MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
struct Visibility {
    int m_attempts, m_connections;

    Visibility(int _a = 0, int _c = 0) : m_attempts(_a), m_connections(_c) {}
    ~Visibility() {}

    double Ratio() const {
	if(m_attempts == 0)
	  return 0.0;
	else
	  return (double)m_connections / (double)m_attempts;
      }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
struct NodeTypeCounts {
    int m_numCreate, m_numMerge, m_numExpand, m_numOversample;

    NodeTypeCounts() : m_numCreate(0), m_numMerge(0), m_numExpand(0), m_numOversample(0) {}
    ~NodeTypeCounts() {}

    friend std::ostream& operator<<(std::ostream& _os, const NodeTypeCounts& _nt);
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief TODO
///
/// TODO
///
/// \todo Configure for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class HybridPRM : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID       VID;

    HybridPRM() {
     this->SetName("HybridPRM");
     }

    HybridPRM(std::string _samplerSelectionDistribution,
    bool _countCost, double _percentageRandom, bool _fixedCost, bool _resettingLearning,
    int _binSize, const std::map<std::string, std::pair<int, int> >& _samplerLabels,
    const std::vector<std::string>& _connectorLabels, const std::vector<std::string>& _evaluatorLabels);

    HybridPRM(XMLNode& _node);
    virtual ~HybridPRM() {}

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(std::ostream& _os) const;

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

    void InitializeWeightsProbabilitiesCosts();
    void CopyLearnedProbToProbUse();
    void PrintWeightsProbabilitiesCosts(std::ostream& _out);

    std::string SelectNextSamplingMethod(bool _learning);

    double ComputeVisibilityReward(std::string _nextnode, double _visibility, double _threshold, int _prevcCcCount, int _currCcCount, NodeTypeCounts& _nodeTypes);

    bool InLearningWindow(int _totalSamples) const;

    void RewardAndUpdateWeightsProbabilities(std::string _method, double _rew,unsigned long int _cost);

  protected:
    std::vector<std::string> m_samplerLabels;
    std::vector<std::string> m_connectorLabels;

    std::map<std::string,double> m_nodeWeights;
    std::map<std::string,double> m_nodeProbability;
    std::map<std::string,double> m_uniformProbability;
    std::map<std::string,double> m_learnedProbability;
    std::map<std::string,double> m_noCostProbability;
    std::map<std::string, unsigned long int> m_nodeCosts;
    std::map<std::string,int> m_nodeNumSampled;
    std::map<std::string,int> m_nodeNumOversampled;


    double m_percentageRandom; //lambda
    double m_windowPercent;
    bool m_countCost;
    bool m_fixedCost;
    bool m_resettingLearning;
    int  m_binSize;
    std::string m_samplerSelectionDistribution;

};

inline std::ostream& operator<<(std::ostream& _os, const NodeTypeCounts& _nt){
  _os << ":" << _nt.m_numCreate << ":" << _nt.m_numMerge << ":" << _nt.m_numExpand << ":" << _nt.m_numOversample;
  return _os;
}

template <typename MPTraits>
HybridPRM<MPTraits>::
HybridPRM(std::string _samplerSelectionDistribution,
    bool _countCost, double _percentageRandom, bool _fixedCost,
    bool _resettingLearning, int _binSize,
    const std::map<std::string, std::pair<int, int>>& _samplerLabels,
    const std::vector<std::string>& _connectorLabels,
    const std::vector<std::string>& _evaluatorLabels) :
    m_samplerSelectionDistribution(_samplerSelectionDistribution),
    m_countCost(_countCost), m_percentageRandom(_percentageRandom),
    m_fixedCost(_fixedCost), m_resettingLearning(_resettingLearning),
    m_binSize(_binSize), m_samplerLabels(_samplerLabels),
    m_connectorLabels(_connectorLabels) {
  this->m_meLabels = _evaluatorLabels;
  this->SetName("HybridPRM");
}

template <typename MPTraits>
HybridPRM<MPTraits>::
HybridPRM(XMLNode& _node) :
    MPStrategyMethod<MPTraits>(_node) {
  this->SetName("HybridPRM");
  ParseXML(_node);
}

template <typename MPTraits>
void HybridPRM<MPTraits>::
ParseXML(XMLNode& _node) {
  for(auto& child : _node) {
    if(child.Name() == "node_generation_method"){
      std::string generationMethod = child.Read("Method",true,"","Method");
      m_samplerLabels.push_back(generationMethod);
      int initialCost = child.Read("initialCost",false,1,1,MAX_INT,"initialCost");
      m_nodeCosts[generationMethod] = initialCost;
    }
    else if(child.Name() == "node_connection_method")
      m_connectorLabels.push_back(child.Read("Method",true,"","Method"));
    else if(child.Name() == "evaluation_method")
      this->m_meLabels.push_back(
          child.Read("Method", true, "", "Evaluation Method"));
 }

  m_percentageRandom = _node.Read("percent_random", true, 0.5, 0.0, 1.0, "percent_random");
  m_binSize = _node.Read("bin_size", true, 5, 1, MAX_INT, "bin_size");
  m_windowPercent = _node.Read("window_percent", true, 0.5, 0.0, 1.0, "window_percent");
  m_countCost = _node.Read("Count_Cost", true, true, "Count_Cost");
  m_fixedCost = _node.Read("fixed_cost", true, false, "fixed_cost");
  m_resettingLearning = _node.Read("resetting_learning", true, false, "resetting_learning");
  m_samplerSelectionDistribution = _node.Read("sampler_selection_distribution", false, "", "sampler_selection_distribution");
  if (m_samplerSelectionDistribution == "nowindow")
    m_windowPercent = 1.0; // 100% of the time learning
}

template <typename MPTraits>
void
HybridPRM<MPTraits>::Print(std::ostream& _os) const {
  _os << "HybridPRM<MPTraits>::\n";
  _os << "\tpercent_random = " << m_percentageRandom << std::endl;
  _os << "\tbin_size = " << m_binSize << std::endl;
  _os << "\twindow_percent = " << m_windowPercent << std::endl;
  _os << "\tcount_cost = " << m_countCost << std::endl;
  _os << "\tfixed_cost = " << m_fixedCost << std::endl;
  _os << "\tresetting_learning = " << m_resettingLearning << std::endl;
  _os << "\tsampler_selection_distribution = " << m_samplerSelectionDistribution << std::endl;

  _os << "\tnode_generation_methods: ";
  for(auto&  l : m_samplerLabels)
    _os << l << " ";
  _os << "\n\tnode_connection_methods: ";
  for(auto&  l : m_connectorLabels)
    _os << l << " ";
  _os << "\n\tevaluator_methods: ";
  for(auto&  l : this->m_meLabels)
    _os << l << " ";

}

template <typename MPTraits>
void HybridPRM<MPTraits>::Initialize(){
  Print(std::cout);
  this->GetStatClass()->StartClock("Map Generation");

  InitializeWeightsProbabilitiesCosts();
  CopyLearnedProbToProbUse();

}

template <typename MPTraits>
void HybridPRM<MPTraits>::Run(){
  StatClass* stats = this->GetStatClass();
  int totalSamples = 0;
  bool mapPassedEvaluation = this->EvaluateMap();
  std::map<VID, Visibility> visMap;
   NodeTypeCounts nodeTypes;
   stapl::sequential::vector_property_map<typename GraphType::STAPLGraph, size_t>
       cmap;
   stats->StartClock("Total Node Generation");
   while(!mapPassedEvaluation){

    if(this->m_debug)
    PrintWeightsProbabilitiesCosts(std::cout);

    do {
      std::string nextNodeGen = SelectNextSamplingMethod(InLearningWindow(totalSamples));
      if(this->m_debug)
      std::cout << "selecting sampler \"" << nextNodeGen << "\"\n";


      unsigned long int numcdbeforegen = stats->GetIsCollTotal();
      std::vector<CfgType> vectorCfgs;
      auto pNodeGen = this->GetSampler(nextNodeGen);
      pNodeGen->Sample(1, 1, this->GetEnvironment()->GetBoundary(),
          std::back_inserter(vectorCfgs));
      unsigned long int numcdaftergen = stats->GetIsCollTotal();
      for(auto C = vectorCfgs.begin(); C != vectorCfgs.end(); ++C) {
        if(C->IsLabel("VALID") && C->GetLabel("VALID")) {
          cmap.reset();
          int nNumPrevCCs = get_cc_count(*(this->GetRoadmap()->GetGraph()), cmap);

          //add node to roadmap
          VID newVID = this->GetRoadmap()->GetGraph()->AddVertex(*C);
          std::vector<std::pair<std::pair<VID,VID>,bool> > connectionattempts;
    	  for(auto&  label : m_connectorLabels) {
            auto connector = this->GetConnector(label);
            connector->ClearConnectionAttempts();
            connector->Connect(this->GetRoadmap(), newVID);
            connectionattempts.insert(connectionattempts.end(),
                connector->ConnectionAttemptsBegin(),
                connector->ConnectionAttemptsEnd());

    	  }

          for(auto&  attempt : connectionattempts) {
            visMap[attempt.first.first].m_attempts++;
            visMap[attempt.first.second].m_attempts++;
            if(attempt.second){
              visMap[attempt.first.first].m_connections++;
              visMap[attempt.first.second].m_connections++;
            }
          }

          unsigned long int cost = (double)(numcdaftergen - numcdbeforegen) / (double)vectorCfgs.size();
          if(this->m_debug){
	  std::cout << "avg node gen cost = " << (double)(numcdaftergen - numcdbeforegen) / (double)vectorCfgs.size() << std::endl;
          std::cout << "cost used = " << cost << std::endl;
	  }
          cmap.reset();
          int nNumCurrCCs = get_cc_count(*(this->GetRoadmap()->GetGraph()), cmap);
          double reward = ComputeVisibilityReward(nextNodeGen, visMap[newVID].Ratio(), 0.3, nNumPrevCCs, nNumCurrCCs, nodeTypes);
          if(InLearningWindow(totalSamples)){
    	    RewardAndUpdateWeightsProbabilities(nextNodeGen, reward, cost);
            if(this->m_debug){
	    std::cout << "new weights and probabilities:";
            PrintWeightsProbabilitiesCosts(std::cout);
	    }
          }
          else
          m_nodeNumSampled[nextNodeGen]++;

          ++totalSamples;
	}
          if(totalSamples % m_binSize == 0) {
            if(m_samplerSelectionDistribution == "nowindow"){
              CopyLearnedProbToProbUse();
              PrintWeightsProbabilitiesCosts(std::cout);
            }
    	    if(m_resettingLearning){
              InitializeWeightsProbabilitiesCosts();
              PrintWeightsProbabilitiesCosts(std::cout);
            }
    	  }

          cmap.reset();


    }
      	} while((totalSamples % m_binSize) > 0);
      	mapPassedEvaluation = this->EvaluateMap();

      }

     stats->StopClock("Total Node Generation");
     if(this->m_debug) {
    stats->PrintClock("Total Node Generation", std::cout);

  }
}

template <typename MPTraits>
void HybridPRM<MPTraits>::Finalize() {
  StatClass* stats = this->GetStatClass();

  //output map
  this->GetRoadmap()->Write(this->GetBaseFilename() + ".map", this->GetEnvironment());


  //output stats
  stats->StopClock("Map Generation");
  std::string outStatname = this->GetBaseFilename() + ".stat";
  std::ofstream  osStat(outStatname.c_str());
  osStat << "NodeGen+Connection Stats" << std::endl;
  stats->PrintAllStats(osStat, this->GetRoadmap());
  stats->PrintClock("Map Generation", osStat);
  osStat.close();
}

template <typename MPTraits>
void
HybridPRM<MPTraits>::
InitializeWeightsProbabilitiesCosts(){
  m_nodeWeights.clear();
  m_nodeProbability.clear();
  m_uniformProbability.clear();
  m_noCostProbability.clear();
  m_learnedProbability.clear();

  double uniformProbability = double(1.0 / m_samplerLabels.size());
  for(std::vector<std::string>::const_iterator NG = m_samplerLabels.begin(); NG != m_samplerLabels.end(); ++NG) {
    m_nodeWeights[*NG] = 1.0;
    m_nodeProbability[*NG] = uniformProbability;
    m_uniformProbability[*NG] = uniformProbability;
    m_noCostProbability[*NG] = uniformProbability;
    m_learnedProbability[*NG] = uniformProbability;
  }
}


template <typename MPTraits>
void
HybridPRM<MPTraits>::
CopyLearnedProbToProbUse() {
  m_learnedProbability.clear();
  for(std::vector<std::string>::const_iterator NG = m_samplerLabels.begin(); NG != m_samplerLabels.end(); ++NG)
    m_learnedProbability[*NG] = m_nodeProbability[*NG];
}


template <typename MPTraits>
void
HybridPRM<MPTraits>::
PrintWeightsProbabilitiesCosts(std::ostream& _out) {
  _out << std::endl;
  _out << "Sampler::\tWeight\tProNoCost\tPro\tCost\n";
  for(std::vector<std::string>::const_iterator NG = m_samplerLabels.begin(); NG != m_samplerLabels.end(); ++NG) {
    _out << *NG << "::";
    _out << "\t" << m_nodeWeights[*NG];
    _out << "\t" << m_noCostProbability[*NG];
    _out << "\t" << m_nodeProbability[*NG];
    _out << "\t" << m_nodeCosts[*NG];
    _out << "\n";
  }
  _out << std::endl;
}


template <typename MPTraits>
std::string
HybridPRM<MPTraits>::
SelectNextSamplingMethod(bool _learning) {
  double proSum = 0;
  std::map<std::string, std::pair<double, double> > mapProRange;

  if(this->m_debug)
    std::cout << "\nmethod\tprob\trange_min\trange_max\n";
  for(std::vector<std::string>::const_iterator NG = m_samplerLabels.begin(); NG != m_samplerLabels.end(); ++NG) {
    double genProb = 0;
    if(_learning && m_samplerSelectionDistribution == "window_uniform")
      genProb = m_uniformProbability[*NG];
    else if(m_samplerSelectionDistribution == "nowindow")
      genProb = m_learnedProbability[*NG];
    else {
      genProb = m_nodeProbability[*NG];
    }
    double upperBound = 0;
    if(NG+1 != m_samplerLabels.end())
      upperBound = proSum + genProb;
    else
      upperBound = 1.0;

    mapProRange[*NG] = std::make_pair(proSum, upperBound);
    proSum += genProb;
 if(this->m_debug)
      std::cout << *NG << "\t" << genProb << "\t" << mapProRange[*NG].first << "\t" << mapProRange[*NG].second << std::endl;
  }

  if(!_learning && m_samplerSelectionDistribution == "window_hybrid_outsidewindow_highest")  {
    double maxProbability = 0;
    std::string maxGen = "";
    for(std::map<std::string, std::pair<double, double> >::const_iterator MPR = mapProRange.begin(); MPR != mapProRange.end(); ++MPR)
      if((MPR->second.second - MPR->second.first) > maxProbability) {
        maxProbability = MPR->second.second - MPR->second.first;
        maxGen = MPR->first;
      }
      if(this->m_debug)
    std::cout << "***\tHighest sampler after learning :: " << maxGen << std::endl;
    return maxGen;
  }
  else {
    double randomNum = DRand();
    for(std::vector<std::string>::const_iterator NG = m_samplerLabels.begin(); NG != m_samplerLabels.end(); ++NG)
      if(mapProRange[*NG].first <= randomNum && randomNum < mapProRange[*NG].second) {
        if(this->m_debug)
	std::cout << "***   The next node generator is::  " << *NG << std::endl;
        return *NG;
      }

    std::cerr << std::endl << std::endl << "This can't be good, exiting.";
    std::cerr << std::endl;
    for(std::vector<std::string>::const_iterator NG = m_samplerLabels.begin(); NG != m_samplerLabels.end(); ++NG)
      std::cerr << *NG << ":: [" << mapProRange[*NG].first << ", " << mapProRange[*NG].second << "]" << std::endl;
    std::cerr << "Random Number:: " << randomNum << std::endl;
    std::exit(-1);
  }
}


template <typename MPTraits>
double
HybridPRM<MPTraits>::
ComputeVisibilityReward(std::string _nextNodeGen, double _visibility, double _threshold, int _prevCcCount, int _currCcCount, NodeTypeCounts& _nodeTypes) {
  if(_currCcCount > _prevCcCount) {
    _nodeTypes.m_numCreate++;
    return 1.0;
  }
  else if(_currCcCount < _prevCcCount) {
    _nodeTypes.m_numMerge++;
    return 1.0;
  }
  else {
    if(_visibility < _threshold)
      _nodeTypes.m_numExpand++;
    else {
      _nodeTypes.m_numOversample++;
      m_nodeNumOversampled[_nextNodeGen]++;
    }
    return std::exp(-4 * std::pow(_visibility, 2));
  }
}


template <typename MPTraits>
bool
HybridPRM<MPTraits>::
InLearningWindow(int _totalSamples) const {
  double d1 = _totalSamples % m_binSize;
  double d2 = double(m_binSize) * m_windowPercent;
  return d1 < d2;
}


template <typename MPTraits>
void
HybridPRM<MPTraits>::
RewardAndUpdateWeightsProbabilities(std::string _nodeSelected, double _reward, unsigned long int _cost) {
  int K = m_samplerLabels.size();

  //update costs:
  if(!m_fixedCost)
  {
    int numSampled = m_nodeNumSampled[_nodeSelected];
    int prevAvgCost = m_nodeCosts[_nodeSelected];
    int newAvgCost = (prevAvgCost * numSampled + _cost) / (numSampled + 1);
    m_nodeCosts[_nodeSelected] = newAvgCost;
  }


  if(this->m_debug)
  std::cout << "mpr: " << m_percentageRandom << " K: " << K << std::endl;
  for(std::vector<std::string>::const_iterator NG = m_samplerLabels.begin(); NG != m_samplerLabels.end(); ++NG){
    double adjustedReward = 0.0;
    double new_weight=0.0;
    if(*NG == _nodeSelected) {
      adjustedReward = _reward / m_noCostProbability[*NG];
    if(this->m_debug)
    std::cout << "mngw: " << m_nodeWeights[*NG] << " ar: " << adjustedReward << ": " << *NG <<std::endl;
    new_weight = m_nodeWeights[*NG] * std::exp(double((m_percentageRandom) * adjustedReward / double(K)));
    m_nodeWeights[*NG] = new_weight;
  }
  }


  double weightTotal = 0;
  for(std::vector<std::string>::const_iterator NG = m_samplerLabels.begin(); NG != m_samplerLabels.end(); ++NG)
    weightTotal += m_nodeWeights[*NG];
  for(std::vector<std::string>::const_iterator NG = m_samplerLabels.begin(); NG != m_samplerLabels.end(); ++NG) {
    m_noCostProbability[*NG] = (1 - m_percentageRandom) * (m_nodeWeights[*NG] / weightTotal) + (m_percentageRandom / K);
  }

  double probNoCostTotal = 0;
  for(std::vector<std::string>::const_iterator NG = m_samplerLabels.begin(); NG != m_samplerLabels.end(); ++NG)
    probNoCostTotal += (m_noCostProbability[*NG] / double(m_nodeCosts[*NG]));
  for(std::vector<std::string>::const_iterator NG = m_samplerLabels.begin(); NG != m_samplerLabels.end(); ++NG){
    if(!m_countCost) {
      m_nodeProbability[*NG] = m_noCostProbability[*NG];
    }
    else
      m_nodeProbability[*NG] = (m_noCostProbability[*NG] / double(m_nodeCosts[*NG])) / probNoCostTotal;
  }


  double smallestWeight = -1;
  for(std::vector<std::string>::const_iterator NG = m_samplerLabels.begin(); NG != m_samplerLabels.end(); ++NG) {
    double weight = m_nodeWeights[*NG];
    if(weight < smallestWeight || smallestWeight == -1)
      smallestWeight = weight;
  }
  for(std::vector<std::string>::const_iterator NG = m_samplerLabels.begin(); NG != m_samplerLabels.end(); ++NG)
    m_nodeWeights[*NG] = m_nodeWeights[*NG] / smallestWeight;
}



#endif
