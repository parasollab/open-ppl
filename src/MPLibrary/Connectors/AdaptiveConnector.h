#ifndef PMPL_ADAPTIVE_CONNECTOR_H_
#define PMPL_ADAPTIVE_CONNECTOR_H_

#include "ConnectorMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// TODO
/// @ingroup Connectors
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class AdaptiveConnector: public ConnectorMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;

    ///@}
    ///@name Construction
    ///@{

    AdaptiveConnector(const std::vector<std::string>& _neigborGenLabels =
        std::vector<std::string>(),
        std::string _lpLabel = "", bool _setUniform = false,
        double _percentageRandom = .5, bool _fixedCost = false,
        bool _fixedReward = false, bool _checkIfSameCC = false,
        bool _countFailures = false, size_t _fail = 5);

    AdaptiveConnector(XMLNode& _node);

    virtual ~AdaptiveConnector() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    virtual void Initialize() override;

    ///@}
    ///@name Connector Interface
    ///@{

    template<typename InputIterator1, typename InputIterator2,
        typename OutputIterator>
    void Connect(RoadmapType* _rm,
        InputIterator1 _itr1First, InputIterator1 _itr1Last,
        InputIterator2 _itr2First, InputIterator2 _itr2Last,
        bool _fromFullRoadmap,
        OutputIterator _collision);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    template<typename InputIterator, typename OutputIterator>
    void ConnectNeighbors(RoadmapType* _rm, VID _vid,
        InputIterator _closestFirst, InputIterator _closestLast,
        OutputIterator _collision);

    std::string UpdateNFChoice();

    void RewardUpdateProbability(double _reward, unsigned long int _cost,
        int _prevConnectionAttempt);

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::vector<std::string> m_neigborGenLabels;
    bool m_setUniform;
    double m_percentageRandom;
    bool m_fixedCost;
    bool m_fixedReward;
    bool m_checkIfSameCC; //do not test connections inside of a CC if true. Creates a tree.
    bool m_countFailures; //count and limit the failures per iteration
    size_t m_fail; //number of allowed failures per iteration

    std::map<std::string, double> m_nfProbabilities;
    std::map<std::string, int> m_nfConnected;
    std::map<std::string, double> m_nfWeights;
    std::map<std::string, double> m_nfProbabilitiesWithNoCost;
    std::map<std::string, unsigned long int> m_nfCosts;
    std::string m_lastUse;

    ///@}

};


template <typename MPTraits>
AdaptiveConnector<MPTraits>::
AdaptiveConnector(const std::vector<std::string>& _neigborGenLabels, std::string _lpLabel,
    bool _setUniform, double _percentageRandom, bool _fixedCost,
    bool _fixedReward,bool _checkIfSameCC, bool _countFailures, size_t _fail) :
    ConnectorMethod<MPTraits>("",_lpLabel),
    m_neigborGenLabels(_neigborGenLabels),
    m_setUniform(_setUniform),
    m_percentageRandom(_percentageRandom),
    m_fixedCost(_fixedCost),
    m_fixedReward(_fixedReward),
    m_checkIfSameCC(_checkIfSameCC),
    m_countFailures(_countFailures),
    m_fail(_fail) {
  this->SetName("AdaptiveConnector");
}


template <typename MPTraits>
AdaptiveConnector<MPTraits>::
AdaptiveConnector(XMLNode& _node) : ConnectorMethod<MPTraits>(_node) {
  this->SetName("AdaptiveConnector");

  m_checkIfSameCC = _node.Read("checkIfSameCC", false, true, "If true, do not "
      "connect if edges are in the same CC");
  m_countFailures = _node.Read("countFailures", false, false, "if false, ignore "
      "failure count and just attempt k; if true, attempt k neighbors until too "
      "many failures detected");
  m_fail = _node.Read("fail", false, 5, 0, 10000, "amount of failed connections "
      "allowed before operation terminates");
  m_percentageRandom = _node.Read("percentRandom", true, 0.5, 0.0, 1.0, "percent"
      " that a learned one is chosen");
  m_setUniform = _node.Read("uniformProbability", false, false, "give all "
      "connection methods the same probability of getting chosen");
  m_fixedCost = _node.Read("fixedCost", true, false, "set a fixed cost");
  m_fixedReward = _node.Read("fixedReward", true, false, "set a fixed reward");

  for(auto& child : _node) {
    if(child.Name() == "NeighborFinder"){
      std::string nodeNfMethod = child.Read("Method",true,"","Method");
      m_neigborGenLabels.push_back(nodeNfMethod);
      int initialCost = child.Read("initialCost", false, 1, 1, MAX_INT,
          "initialCost at the start of the learn phase");
      m_nfCosts[nodeNfMethod] = initialCost;
      double initialWeight = child.Read("initialWeight", false, 1, 1, MAX_INT,
          "initialWeight at the start of the learn phase");
      m_nfWeights[nodeNfMethod] = initialWeight;
    }
  }

  if(_node.Read("nfLabel", true, "", "Neighborhood Finder") != "")
    throw ParseException(_node.Where(), "nfLabel should be specified as ''.");
}


template <typename MPTraits>
void
AdaptiveConnector<MPTraits>::
Print(std::ostream& _os) const {
  ConnectorMethod<MPTraits>::Print(_os);
  _os << "\tfail = " << m_fail
      << "\tcountFailures = " << m_countFailures
      << "\tpercentRandom = " << m_percentageRandom
      << "\tsetUniform = " << m_setUniform
      << "\tfixedCost = " << m_fixedCost
      << "\tfixedReward = " << m_fixedReward
      << "\tcheckIfSameCC = " << m_checkIfSameCC
      << "\tNeighborFinders Used = ";
  for(const auto& label : m_neigborGenLabels)
    _os << " " << label;

  _os << "\nmethod\tprob\tprob_no_cost\tweight\tavg_cost\ttimes_used\n";
  for(const auto& label : m_neigborGenLabels) {
    _os << label << "\t"
        << m_nfProbabilities.at(label) << "\t"
        << m_nfProbabilitiesWithNoCost.at(label) << "\t"
        << m_nfWeights.at(label) << "\t"
        << m_nfCosts.at(label) << "\t"
        << m_nfConnected.at(label) << "\t"
        << std::endl;
  }
  _os << "last used = " << m_lastUse << std::endl;
}

template <typename MPTraits>
void
AdaptiveConnector<MPTraits>::
Initialize() {
  if(this->m_debug)
    std::cout<<"initializing"<<std::endl;

  m_lastUse="";
  m_nfProbabilities.clear();
  m_nfWeights.clear();
  m_nfConnected.clear();
  m_nfProbabilitiesWithNoCost.clear();
  m_nfCosts.clear();

  for(auto NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF){
    m_nfCosts[*NF]=1;
    m_nfWeights[*NF]=1;
    m_nfProbabilities[*NF] = double(1.0/m_neigborGenLabels.size());  //all the probalities are assigned one at the beginning.
    m_nfProbabilitiesWithNoCost[*NF] = double(1.0/m_neigborGenLabels.size());
  }
}

template <typename MPTraits>
template<typename InputIterator1, typename InputIterator2,
    typename OutputIterator>
void
AdaptiveConnector<MPTraits>::
Connect(RoadmapType* _rm,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision) {
  if(m_nfProbabilities.empty() ||
      m_neigborGenLabels.size() != m_nfProbabilities.size())
    Initialize();

  // the vertices in this iteration are the source for the connection operation
  for(InputIterator1 itr1 = _itr1First; itr1 != _itr1Last; ++itr1) {
     static double prevConnectionAttempt = 0, prevConnectionSuccess = 0;
     static unsigned long int prevConnectionCollision = 0;

     double currAttempts = get<0>(this->GetStatClass()->m_lpInfo.begin()->second);
     double currSuccess  = get<1>(this->GetStatClass()->m_lpInfo.begin()->second);
     unsigned long int currCollision = this->GetStatClass()->GetIsCollTotal();

     double reward = 0;
     unsigned long int cost=0;
     if(m_lastUse !=""){
       reward = currSuccess/currAttempts;
       if(prevConnectionAttempt != 0)
         reward = (currSuccess - prevConnectionSuccess) /
           (currAttempts - prevConnectionAttempt);
       cost = (double)(currCollision - prevConnectionCollision);
     }

     if(this->m_debug){
       std::cout << "curr collision" << currCollision << std::endl;
       std::cout << "pre collision" << prevConnectionCollision << std::endl;
     }
     prevConnectionAttempt = currAttempts;
     prevConnectionSuccess = currSuccess;
     prevConnectionCollision= currCollision;
     if(m_nfConnected[this->m_lastUse] !=0){
       RewardUpdateProbability(reward, cost,prevConnectionAttempt);
     }
     this->m_lastUse = UpdateNFChoice();
     m_nfConnected[this->m_lastUse]++;

     // find cfg pointed to by itr1
     VID vid = _rm->GetGraph()->GetVID(itr1);
     CfgType& vCfg = _rm->GetGraph()->GetVertex(itr1);
     if(this->m_debug)
       std::cout << (itr1 - _itr1First)
         << "\tAttempting connections: VID = "
         << vid << "  --> Cfg = " << vCfg << std::endl;

     //determine nearest neighbors
     std::vector<Neighbor> closest;
     auto nfptr = this->GetNeighborhoodFinder(this->m_lastUse);
     nfptr->FindNeighbors(_rm, _itr2First, _itr2Last, _fromFullRoadmap, vCfg,
         std::back_inserter(closest));
     if(this->m_debug){
       std::cout << "Neighbors | ";
       for(auto nit = closest.begin(); nit != closest.end(); ++nit)
         std::cout << nit->target << " ";
     }

     //test connections through LP
     ConnectNeighbors(_rm, vid, closest.begin(), closest.end(), _collision);
  }
}


template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
void
AdaptiveConnector<MPTraits>::
ConnectNeighbors(RoadmapType* _rm, VID _vid,
    InputIterator _closestFirst, InputIterator _closestLast,
    OutputIterator _collision) {

  Environment* env = this->GetEnvironment();
  auto lp = this->GetLocalPlanner(this->m_lpLabel);
  GraphType* g = _rm->GetGraph();

  LPOutput<MPTraits> lpOutput;
  size_t failure = 0;

  // connect the found k-closest to the current iteration's CfgType
  for(InputIterator itr2 = _closestFirst; itr2 != _closestLast; ++itr2){

    const VID v2 = itr2->target;

    if(this->m_debug)
      std::cout << "\tfailures = " << failure
                << " | VID = " << v2
                << " | dist = " << itr2->distance
                << std::endl;

    // stopping conditions
    if(this->m_countFailures && failure >= m_fail) {
      if(this->m_debug)
        std::cout << " | stopping... failures exceeded" << std::endl;
      break;
    }

    // don't attempt the connection if it already failed once before
    if(this->IsCached(_vid, v2) && !this->GetCached(_vid, v2)) {
      if(this->m_debug)
        std::cout << " | skipping... this connection already failed once"
                  << " | failure incremented" << std::endl;
      failure++;
      continue;
    }

    // if the edge already exists, so no need to call LP. Count as success.
    if(g->IsEdge(_vid, v2)) {
      if(this->m_debug)
        std::cout << " | edge already exists in roadmap | skipping" << std::endl;
      continue;
    }

    if(m_checkIfSameCC) {
      // if the nodes are in the same connected component count as success
      typename GraphType::ColorMap colorMap;
      if(stapl::sequential::is_same_cc(*g, colorMap, _vid, v2)) {
        if(this->m_debug)
          std::cout << " | nodes in the same connected component | skipping"
                    << std::endl;
        continue;
      }
    }

    // attempt connection with the local planner
    CfgType& c1 = g->GetVertex(_vid);
    CfgType& c2 = g->GetVertex(v2);

    auto robot = this->GetTask()->GetRobot();
    CfgType col(robot);
    bool connectable = lp->IsConnected(c1, c2, col, &lpOutput,
          env->GetPositionRes(), env->GetOrientationRes(), true);
    if(col != CfgType(robot))
      *_collision++ = col;

    //add connection attempt to caches
    this->AddConnectionAttempt(_vid, v2, connectable);

    c1.IncrementStat("totalConnectionAttempts", 1);
    c2.IncrementStat("totalConnectionAttempts", 1);

    if(connectable) {
      if(this->m_debug)
        std::cout << " | connection was successful | success incremented"
                  << std::endl;
      // increment # of successful connection attempts
      c1.IncrementStat("succConnectionAttempts", 1);
      c2.IncrementStat("succConnectionAttempts", 1);
      // if connection was made, add edge and record the successful connection
      _rm->GetGraph()->AddEdge(_vid, v2, lpOutput.m_edge);
    }
    else {
      if(this->m_debug)
        std::cout << " | connection failed | failure incremented" << std::endl;
      failure++;
    }
  }
}


template <typename MPTraits>
std::string
AdaptiveConnector<MPTraits>::
UpdateNFChoice() {
  double proSum = 0;
  double nfProb =0;
  std::map<std::string, std::pair<double, double>> mapProRange;

  if(this->m_debug)
    std::cout << "\nmethod\tprob\trange_min\trange_max\n";
  for(std::vector<std::string>::const_iterator NF = m_neigborGenLabels.begin();
      NF != m_neigborGenLabels.end(); ++NF) {
    if(m_setUniform)
      nfProb = double(1.0/m_neigborGenLabels.size());
    else
      nfProb = m_nfProbabilities[*NF];

    double upperBound = 0;
    if(NF +1 != m_neigborGenLabels.end())
      upperBound = proSum + nfProb;
    else
      upperBound = 1.0; // do this only in the last number
    mapProRange[*NF] = std::make_pair(proSum, upperBound);
    proSum += nfProb;

    if(this->m_debug)
      std::cout << *NF << "\t" << nfProb << "\t" << mapProRange[*NF].first
                << "\t" << mapProRange[*NF].second
                << std::endl;
  }

  if(m_setUniform) {
    std::string maxGen = "";
    int rand =int(DRand() * m_neigborGenLabels.size()) ;
    maxGen= m_neigborGenLabels[rand];
    return maxGen;
  }
  else {
    //select NF based on probability ranges
    double randomNum = DRand();
    for(std::vector<std::string>::const_iterator NF = m_neigborGenLabels.begin();
        NF != m_neigborGenLabels.end(); ++NF){
      if(mapProRange[*NF].first <=randomNum && randomNum < mapProRange[*NF].second) {
        if(this->m_debug)
          std::cout << "randomNum = " << randomNum << "\tselecting method "
                    << *NF << std::endl;
        return *NF;
      }
    }
    throw RunTimeException(WHERE) << "Failed to pick a NeigborhoodFinder.";
  }
}


template <typename MPTraits>
void
AdaptiveConnector<MPTraits>::
RewardUpdateProbability(double _reward, unsigned long int _cost,
    int _prevConnectionAttempt) {
  int neigborSize = m_neigborGenLabels.size();

  //update costs
  if(!m_fixedCost){
    int numConnected = m_nfConnected[this->m_lastUse];
    if(this->m_debug)
      std::cout<<"num connected = "<<numConnected<<std::endl;
    int prevAvgCost = m_nfCosts[this->m_lastUse];
    int newAvgCost= (prevAvgCost *(numConnected -1) + _cost)/numConnected;
    m_nfCosts[this->m_lastUse] = newAvgCost;
  }

  //update weight
  if(!m_fixedReward) {
    if (_prevConnectionAttempt !=0) {
      double adjustedReward = _reward/m_nfProbabilitiesWithNoCost[this->m_lastUse];
      double newWeight = m_nfWeights[this->m_lastUse] * exp(double((m_percentageRandom) * adjustedReward/ double(neigborSize)));
      m_nfWeights[this->m_lastUse] = newWeight;
    }
  }

  //update probability
  double weightTotal= 0;
  for(std::vector<std::string>::const_iterator NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF)
    weightTotal +=m_nfWeights[*NF];

  double probCostTotal = 0.0;
  for(std::vector<std::string>::const_iterator NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF){
    m_nfProbabilitiesWithNoCost[*NF] = (1 - m_percentageRandom) * (m_nfWeights[*NF] / weightTotal) + (m_percentageRandom /neigborSize);
    probCostTotal += double(m_nfProbabilitiesWithNoCost[*NF]/ double(m_nfCosts[*NF]));
  }

  if(this->m_debug)
    std::cout << "new probabilities: ";
  for(std::vector<std::string>::const_iterator NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF) {
    m_nfProbabilities[*NF] =  (m_nfProbabilitiesWithNoCost[*NF]/double(m_nfCosts[*NF]) )/ probCostTotal;
    if(this->m_debug){
      std::cout<<m_nfProbabilities[*NF]<<" ";
    }

    std::cout << std::endl;
  }

  if(!m_fixedReward) {
    double smallestWeight = -1;
    for(std::vector<std::string>::const_iterator NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF){
      double weight = m_nfWeights[*NF];

      if(weight < smallestWeight || smallestWeight == -1)
        smallestWeight = weight;
    }
    for(std::vector<std::string>::const_iterator NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF)
      m_nfWeights[*NF] = m_nfWeights[*NF] / smallestWeight;
  }
}

#endif
