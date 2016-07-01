#ifndef ADAPTIVE_CONNECTOR_H
#define ADAPTIVE_CONNECTOR_H

#include "ConnectorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class AdaptiveConnector: public ConnectorMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::CfgRef CfgRef;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename RoadmapType::GraphType GraphType;

    AdaptiveConnector(const vector<string>& _neigborGenLabels = vector<string>(),string _lpLabel = "", bool _setUniform=false, double _percentageRandom=0.5,bool _fixedCost =false, bool _fixedReward=false,bool _checkIfSameCC = false, bool _countFailures = false, size_t _fail = 5);
    AdaptiveConnector(MPProblemType* _problem, XMLNode& _node);

    virtual void Print(ostream& _os) const;
    virtual void ParseXML(XMLNode& _node);
    virtual void Initialize();

    template<typename InputIterator1, typename InputIterator2,
      typename OutputIterator>
      void Connect(RoadmapType* _rm,
          InputIterator1 _itr1First, InputIterator1 _itr1Last,
          InputIterator2 _itr2First, InputIterator2 _itr2Last,
          bool _fromFullRoadmap,
          OutputIterator _collision);

  protected:
    template<typename InputIterator, typename OutputIterator>
      void ConnectNeighbors(RoadmapType* _rm, VID _vid,
          InputIterator _closestFirst, InputIterator _closestLast,
          OutputIterator _collision);

    string UpdateNFChoice();
    void RewardUpdateProbability(double _reward, unsigned long int _cost,int _prevConnectionAttempt);


  private:
    void PrintData(ostream& _os);

    //input parameters
    vector<string> m_neigborGenLabels;
    bool m_setUniform;
    double m_percentageRandom;
    bool m_fixedCost;
    bool m_fixedReward;
    bool m_checkIfSameCC; //do not test connections inside of a CC if true. Creates a tree.
    bool m_countFailures; //count and limit the failures per iteration
    size_t m_fail; //number of allowed failures per iteration

    //state variables
    map<string, double> m_nfProbabilities;
    map<string,int> m_nfConnected;
    map<string,double> m_nfWeights;
    map<string,double> m_nfProbabilitiesWithNoCost;
    map<string, unsigned long int> m_nfCosts;
    string m_lastUse;
};

template<class MPTraits>
AdaptiveConnector<MPTraits>::
AdaptiveConnector(const vector<string>& _neigborGenLabels, string _lpLabel,
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

template<class MPTraits>
AdaptiveConnector<MPTraits>::
AdaptiveConnector(MPProblemType* _problem, XMLNode& _node) :
  ConnectorMethod<MPTraits>(_problem, _node) {
    this->SetName("AdaptiveConnector");
    ParseXML(_node);
  }

template<class MPTraits>
void
AdaptiveConnector<MPTraits>::ParseXML(XMLNode& _node){
  m_checkIfSameCC = _node.Read("checkIfSameCC", false, true, "If true, do not connect if edges are in the same CC");
  m_countFailures = _node.Read("countFailures", false, false, "if false, ignore failure count and just attempt k; if true, attempt k neighbors until too many failures detected");
  m_fail = _node.Read("fail", false, 5, 0, 10000, "amount of failed connections allowed before operation terminates");
  m_percentageRandom = _node.Read("percentRandom", true, 0.5, 0.0, 1.0, "percent that a learned one is chosen");
  m_setUniform = _node.Read("uniformProbability", false, false, "give all connection methods the same probability of getting chosen");
  m_fixedCost = _node.Read("fixedCost", true, false, "set a fixed cost");
  m_fixedReward = _node.Read("fixedReward", true, false, "set a fixed reward");

  for(auto& child : _node) {
    if(child.Name() == "NeighborFinder"){
      string nodeNfMethod = child.Read("Method",true,"","Method");
      m_neigborGenLabels.push_back(nodeNfMethod);
      int initialCost = child.Read("initialCost",false,1,1,MAX_INT,"initialCost at the start of the learn phase");
      m_nfCosts[nodeNfMethod] = initialCost;
      double initialWeight = child.Read("initialWeight",false,1,1,MAX_INT,"initialWeight at the start of the learn phase");
      m_nfWeights[nodeNfMethod] = initialWeight;
    }
  }

  if(_node.Read("nfLabel", true, "", "Neighborhood Finder") != "")
    throw ParseException(_node.Where(), "nfLabel should be specified as ''.");
}

template<class MPTraits>
void
AdaptiveConnector<MPTraits>::Print(ostream& _os) const {
  ConnectorMethod<MPTraits>::Print(_os);
  _os << "\tfail = " << m_fail << endl;
  _os << "\tcountFailures = " << m_countFailures << endl;
  _os << "\tpercentRandom = " << m_percentageRandom << endl;
  _os << "\tsetUniform = " << m_setUniform << endl;
  _os << "\tfixedCost = " << m_fixedCost << endl;
  _os << "\tfixedReward = " << m_fixedReward << endl;
  _os << "\tcheckIfSameCC = " << m_checkIfSameCC << endl;
  _os << "\tList of NeighborFinders Used"<< endl;
  for(vector<string>::const_iterator nf = m_neigborGenLabels.begin(); nf != m_neigborGenLabels.end(); nf++)
    _os <<"     "<<*nf << endl;
}

template<class MPTraits>
void
AdaptiveConnector<MPTraits>::
Initialize() {
  if(this->m_debug)
    cout<<"initializing"<<endl;

  m_lastUse="";
  m_nfProbabilities.clear();
  m_nfWeights.clear();
  m_nfConnected.clear();
  m_nfProbabilitiesWithNoCost.clear();
  m_nfCosts.clear();

  for(vector<string>::const_iterator NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF){
    m_nfCosts[*NF]=1;
    m_nfWeights[*NF]=1;
    m_nfProbabilities[*NF] = double(1.0/m_neigborGenLabels.size());  //all the probalities are assigned one at the beginning.
    m_nfProbabilitiesWithNoCost[*NF] = double(1.0/m_neigborGenLabels.size());
  }

  if(this->m_debug)
    PrintData(cout);
}

template<class MPTraits>
template<typename InputIterator1, typename InputIterator2, typename OutputIterator>
void
AdaptiveConnector<MPTraits>::
Connect(RoadmapType* _rm,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision) {

  if(m_nfProbabilities.empty() || m_neigborGenLabels.size() != m_nfProbabilities.size())
    Initialize();

  // the vertices in this iteration are the source for the connection operation
  for(InputIterator1 itr1 = _itr1First; itr1 != _itr1Last; ++itr1){

     static double prevConnectionAttempt=0, prevConnectionSuccess=0;
     static unsigned long int prevConnectionCollision=0;


    double  currAttempts =   get<0>(this->GetStatClass()->m_lpInfo.begin()->second);
    double  currSuccess  =   get<1>(this->GetStatClass()->m_lpInfo.begin()->second);
    unsigned long int  currCollision = this->GetStatClass()->GetIsCollTotal();

    double reward = 0;
    unsigned long int cost=0;
    if(m_lastUse !=""){
      reward = currSuccess/currAttempts;
      if (prevConnectionAttempt !=0)
	reward = (currSuccess - prevConnectionSuccess) / (currAttempts -prevConnectionAttempt);
        cost = (double)(currCollision - prevConnectionCollision);
      }
    //if (m_lastUse !="")

    if(this->m_debug){
      cout<<"curr collision"<<currCollision<<endl;
      cout<<"pre collision"<<prevConnectionCollision<<endl;
    }
    prevConnectionAttempt =  currAttempts;
    prevConnectionSuccess =  currSuccess;
    prevConnectionCollision= currCollision;
    if(m_nfConnected[this->m_lastUse] !=0){
    RewardUpdateProbability(reward, cost,prevConnectionAttempt);
    }
    this->m_lastUse = UpdateNFChoice();
    m_nfConnected[this->m_lastUse]++;
    if(this->m_debug)
      PrintData(cout);

    // find cfg pointed to by itr1
    VID vid = _rm->GetGraph()->GetVID(itr1);
    CfgRef vCfg = _rm->GetGraph()->GetVertex(itr1);
    if(this->m_debug)
      cout << (itr1 - _itr1First)
        << "\tAttempting connections: VID = "
        << vid << "  --> Cfg = " << vCfg << endl;

    //determine nearest neighbors
    vector<pair<VID, double> > closest;
    NeighborhoodFinderPointer nfptr = this->GetMPProblem()->GetNeighborhoodFinder(this->m_lastUse);
    nfptr->FindNeighbors(_rm, _itr2First, _itr2Last, _fromFullRoadmap, vCfg,
        back_inserter(closest));
    if(this->m_debug){
      cout << "Neighbors | ";
      for(typename vector<pair<VID, double> >::iterator nit = closest.begin(); nit!=closest.end(); ++nit)
        cout << nit->first << " ";
    }

    //test connections through LP
    ConnectNeighbors(_rm, vid, closest.begin(), closest.end(), _collision);
  }
}

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
void
AdaptiveConnector<MPTraits>::
ConnectNeighbors(RoadmapType* _rm, VID _vid,
    InputIterator _closestFirst, InputIterator _closestLast,
    OutputIterator _collision) {

  Environment* env = this->GetMPProblem()->GetEnvironment();
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(this->m_lpLabel);
  GraphType* map = _rm->GetGraph();

  LPOutput<MPTraits> lpOutput;
  size_t failure = 0;

  // connect the found k-closest to the current iteration's CfgType
  for(InputIterator itr2 = _closestFirst; itr2 != _closestLast; ++itr2){

    VID v2 = itr2->first;

    if(this->m_debug)
      cout << "\tfailures = " << failure
        << " | VID = " << v2
        << " | dist = " << itr2->second;

    // stopping conditions
    if(this->m_countFailures && failure >= m_fail){
      if(this->m_debug) cout << " | stopping... failures exceeded" << endl;
      break;
    }

    // don't attempt the connection if it already failed once before
    if(this->IsCached(_vid, v2) && !this->GetCached(_vid, v2)){
      if(this->m_debug) {
        cout << " | skipping... this connection already failed once"
          << " | failure incremented" << endl;
      }
      failure++;
      continue;
    }

    // if the edge already exists, so no need to call LP. Count as success.
    if(map->IsEdge(_vid, v2)){
      if(this->m_debug)
        cout << " | edge already exists in roadmap | skipping" << endl;
      continue;
    }

    if(m_checkIfSameCC){
      // if the nodes are in the same connected component count as success
      typename GraphType::ColorMap colorMap;
      if(stapl::sequential::is_same_cc(*map, colorMap, _vid, v2)){
        if(this->m_debug)
          cout << " | nodes in the same connected component | skipping" << endl;
        continue;
      }
    }

    // attempt connection with the local planner
    CfgRef c1 = map->GetVertex(_vid);
    CfgRef c2 = map->GetVertex(v2);

    CfgType col;
    bool connectable = lp->IsConnected(c1, c2, col, &lpOutput,
          env->GetPositionRes(), env->GetOrientationRes(), true);
    if(col != CfgType())
      *_collision++ = col;

    //add connection attempt to caches
    this->AddConnectionAttempt(_vid, v2, connectable);

    c1.IncStat("totalConnectionAttempts", 1);
    c2.IncStat("totalConnectionAttempts", 1);

    if(connectable){
      if(this->m_debug) cout << " | connection was successful | success incremented" << endl;
      // increment # of successful connection attempts
      c1.IncStat("succConnectionAttempts", 1);
      c2.IncStat("succConnectionAttempts", 1);
      // if connection was made, add edge and record the successful connection
      _rm->GetGraph()->AddEdge(_vid, v2, lpOutput.m_edge);
    }
    else {
      if(this->m_debug) cout << " | connection failed | failure incremented" << endl;
      failure++;
    }
  }
}

template<class MPTraits>
string
AdaptiveConnector<MPTraits>::
UpdateNFChoice()
{
  double proSum = 0;
  double nfProb =0;
  map<string, pair<double, double> > mapProRange;

  if(this->m_debug)
    cout << "\nmethod\tprob\trange_min\trange_max\n";
  for(vector<string>::const_iterator NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF){
     if(m_setUniform)
       nfProb = double(1.0/m_neigborGenLabels.size());
    else
      nfProb = m_nfProbabilities[*NF];
    double upperBound = 0;
    if(NF +1 != m_neigborGenLabels.end())
      upperBound = proSum + nfProb;
    else
      upperBound = 1.0; // do this only in the last number
    mapProRange[*NF] = make_pair(proSum, upperBound);
    proSum += nfProb;

    if(this->m_debug)
      cout << *NF << "\t" << nfProb << "\t" << mapProRange[*NF].first << "\t" << mapProRange[*NF].second << endl;
  }

  if(m_setUniform) {
     string maxGen = "";
     int rand =int(DRand() * m_neigborGenLabels.size()) ;
     maxGen= m_neigborGenLabels[rand];
     return maxGen;
  }
   else {
    //select NF based on probability ranges
    double randomNum = DRand();
    for(vector<string>::const_iterator NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF){
       if(mapProRange[*NF].first <=randomNum && randomNum < mapProRange[*NF].second) {
         if(this->m_debug)
          cout << "randomNum = " << randomNum << "\tselecting method " << *NF << endl;
         return *NF;
       }
    }
    cerr << "Neigbor Finder not picked, Exiting \n" ;
    exit(1);
  }
}


template<class MPTraits>
void
AdaptiveConnector<MPTraits>::
RewardUpdateProbability(double _reward, unsigned long int _cost,int _prevConnectionAttempt){
  int neigborSize = m_neigborGenLabels.size();

  //update costs
   if(!m_fixedCost){
     int numConnected = m_nfConnected[this->m_lastUse];
     if(this->m_debug)
       cout<<"num connected = "<<numConnected<<endl;
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
   for(vector<string>::const_iterator NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF)
     weightTotal +=m_nfWeights[*NF];

     double probCostTotal = 0.0;
   for(vector<string>::const_iterator NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF){
     m_nfProbabilitiesWithNoCost[*NF] = (1 - m_percentageRandom) * (m_nfWeights[*NF] / weightTotal) + (m_percentageRandom /neigborSize);
     probCostTotal += double(m_nfProbabilitiesWithNoCost[*NF]/ double(m_nfCosts[*NF]));
   }

   if(this->m_debug)
    cout << "new probabilities: ";
   for(vector<string>::const_iterator NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF) {
     m_nfProbabilities[*NF] =  (m_nfProbabilitiesWithNoCost[*NF]/double(m_nfCosts[*NF]) )/ probCostTotal;
     if(this->m_debug){
     cout<<m_nfProbabilities[*NF]<<" ";
     }

   }
   if(this->m_debug) cout << endl;

   if(!m_fixedReward) {
    double smallestWeight = -1;
    for(vector<string>::const_iterator NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF){
       double weight = m_nfWeights[*NF];

      if(weight < smallestWeight || smallestWeight == -1)
        smallestWeight = weight;
    }
    for(vector<string>::const_iterator NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF)
      m_nfWeights[*NF] = m_nfWeights[*NF] / smallestWeight;
   }
}

template<class MPTraits>
void
AdaptiveConnector<MPTraits>::
PrintData(ostream& _os) {
  _os << "\nmethod\tprob\tprob_no_cost\tweight\tavg_cost\ttimes_used\n";
  for(vector<string>::const_iterator NF = m_neigborGenLabels.begin(); NF != m_neigborGenLabels.end(); ++NF) {
    _os << *NF << "\t";
    _os << m_nfProbabilities[*NF] << "\t";
    _os << m_nfProbabilitiesWithNoCost[*NF] << "\t";
    _os << m_nfWeights[*NF] << "\t";
    _os << m_nfCosts[*NF] << "\t";
    _os << m_nfConnected[*NF] << "\t";
    _os << endl;
  }
  _os << "last used = " << m_lastUse << endl;
}

#endif
