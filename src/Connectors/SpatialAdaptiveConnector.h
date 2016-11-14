#ifndef SPATIAL_ADAPTIVE_CONNECTOR_H
#define SPATIAL_ADAPTIVE_CONNECTOR_H

///////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @brief Spatial ANC chooses a connection method to connect every new node
/// to the graph based on its local neighborhood
/// @tparam MPTraits Motion planning universe
///
/// CMType is a pair of a neighbor finder (distance metric) and a local
/// planner.
/// This allows us to make pairs in a such a way that the neighbors returned
/// by the nf are likely connectable by its paired local planner.
///////////////////////////////////////////////////////////////////////////
#include "ConnectorMethod.h"

template<class MPTraits>
class SpatialAdaptiveConnector: public ConnectorMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType                        CfgType;
    typedef typename MPTraits::CfgRef                         CfgRef;
    typedef typename MPTraits::MPProblemType                  MPProblemType;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::LocalPlannerPointer       LocalPlannerPointer;
    typedef typename MPProblemType::RoadmapType               RoadmapType;
    typedef typename MPProblemType::VID                       VID;
    typedef typename RoadmapType::GraphType                   GraphType;
    typedef pair<string, string>                              CMType;
    typedef tuple<pair< CMType,int>, double, double >         pc_data;

    SpatialAdaptiveConnector(const vector<CMType>& _CMLabels = vector<CMType>(),
        const map<CMType,int>& _lastUseID = map<CMType,int>(),
        bool _setUniform = false,
        double _percentageRandom = 0.5,
        bool _fixedCost = false,
        bool _fixedReward = false,
        bool _checkIfSameCC = false,
        bool _countFailures = false,
        size_t _fail = 5);

    SpatialAdaptiveConnector(MPProblemType* _problem, XMLNode& _node);

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
      void ConnectNeighbors(
          RoadmapType* _rm,VID _vid,
          InputIterator _closestFirst, InputIterator _closestLast,
          OutputIterator _collision);

    CMType UpdateCMChoice(VID _nit);

    void RewardUpdateProbability(VID _nit, double _reward, unsigned long int _cost,
        int _prevConnectionAttempt);

  private:
    void PrintData(ostream& _os);

    //input parameters
    vector<CMType> m_CMLabels;
    map<CMType,int>m_lastUseID;
    bool m_setUniform;
    double m_percentageRandom;
    bool m_fixedCost;
    bool m_fixedReward;
    bool m_checkIfSameCC; //do not test connections inside of a CC if true.
    bool m_countFailures; //count and limit the failures per iteration
    size_t m_fail; //number of allowed failures per iteration
    double m_prevConnectionAttempt;

    double m_reward;
    unsigned long int m_cost;

    map<CMType, double> m_cmProbabilities;
    map<CMType,double> m_cmProbabilitiesnormal;
    map<CMType,int> m_cmConnected;
    map<CMType,double> m_cmWeights;
    map<CMType,double> m_cmProbabilitiesWithNoCost;
    map<CMType, unsigned long int> m_cmCosts;
    map<VID, vector <pc_data> > m_local_info;
    CMType m_lastUse;
    CMType m_localCM;
    map<CMType, double> m_counterUpdate;
};

template<class MPTraits>
SpatialAdaptiveConnector<MPTraits>::
SpatialAdaptiveConnector(const vector<CMType>& _CMLabels, const map<CMType,int>&_lastUseID,
    bool _setUniform, double _percentageRandom, bool _fixedCost,
    bool _fixedReward,bool _checkIfSameCC, bool _countFailures, size_t _fail) :
  ConnectorMethod<MPTraits>("",""),
  m_CMLabels(_CMLabels), m_lastUseID(_lastUseID),
  m_setUniform(_setUniform),
  m_percentageRandom(_percentageRandom),
  m_fixedCost(_fixedCost),
  m_fixedReward(_fixedReward),
  m_checkIfSameCC(_checkIfSameCC),
  m_countFailures(_countFailures),
  m_fail(_fail) {
    this->SetName("SpatialAdaptiveConnector");
  }

template<class MPTraits>
SpatialAdaptiveConnector<MPTraits>::
SpatialAdaptiveConnector(MPProblemType* _problem, XMLNode& _node):
  ConnectorMethod<MPTraits>(_problem, _node) {
    ParseXML(_node);
  }

template<class MPTraits>
void
SpatialAdaptiveConnector<MPTraits>::
ParseXML(XMLNode& _node) {
  this->SetName("SpatialAdaptiveConnector");
  m_checkIfSameCC = _node.Read("checkIfSameCC", false, true,
      "If true, do not connect if edges are in the same CC");
  m_countFailures = _node.Read("countFailures", false, false,
      "if false, ignore failure count and just attempt k; if true, attempt k neighbors until too many failures detected");
  m_fail = _node.Read("fail", false, 5, 0, 10000,
      "amount of failed connections allowed before operation terminates");
  m_percentageRandom = _node.Read("percentRandom", true, 0.5, 0.0, 1.0,
      "percent that a learned one is chosen");
  m_setUniform = _node.Read("uniformProbability", false, false,
      "give all connection methods the same probability of getting chosen");
  m_fixedCost = _node.Read("fixedCost", true, false, "set a fixed m_cost");
  m_fixedReward = _node.Read("fixedReward", true, false, "set a fixed m_reward");
  string localNF = _node.Read("LocalNeighborFinder",true,
      "", "Local Neighborhood Finder" );
  string localLP = _node.Read("LocalLocalPlanner", true, "", "Local local planner");
  m_localCM = make_pair(localNF, localLP);
  int nid = 1;
  for(auto& child : _node) {
    if(child.Name() == "ConnectionMethod"){
      string nodeNfMethod = child.Read("NeighborFinder",true,"","neighbor finder");
      string nodeLpMethod= child.Read("LocalPlanner", true, "", "local planar");
      CMType connector = make_pair(nodeNfMethod, nodeLpMethod);
      m_CMLabels.push_back(connector);
      m_lastUseID[connector] = nid;
      int initialCost = child.Read("initialCost",false,1,1,MAX_INT,
          "initialCost at the start of the learn phase");
      m_cmCosts[connector] = initialCost;
      double initialWeight = child.Read("initialWeight",false,1,1,MAX_INT,
          "initialWeight at the start of the learn phase");
      m_cmWeights[connector] = initialWeight;
    }
    nid++;
  }
}

template<class MPTraits>
void
SpatialAdaptiveConnector<MPTraits>::
Print(ostream& _os) const {
  ConnectorMethod<MPTraits>::Print(_os);
  _os << "\tfail = " << m_fail << endl;
  _os << "\tcountFailures = " << m_countFailures << endl;
  _os << "\tpercentRandom = " << m_percentageRandom << endl;
  _os << "\tsetUniform = " << m_setUniform << endl;
  _os << "\tfixedCost = " << m_fixedCost << endl;
  _os << "\tfixedReward = " << m_fixedReward << endl;
  _os << "\tcheckIfSameCC = " << m_checkIfSameCC << endl;
  _os << "\tList of NeighborFinders Used"<< endl;
  for(vector<CMType>::const_iterator cm = m_CMLabels.begin();
      cm != m_CMLabels.end(); cm++)
    _os <<"     "<< cm->first <<", "<< cm->second << endl;
}

template<class MPTraits>
void
SpatialAdaptiveConnector<MPTraits>::
Initialize() {
  if(this->m_debug)
    cout<<"initializing"<<endl;
  m_prevConnectionAttempt = 0;
  m_lastUse = m_localCM;
  m_cmProbabilities.clear();
  m_cmWeights.clear();
  m_cmConnected.clear();
  m_cmProbabilitiesWithNoCost.clear();
  m_cmCosts.clear();
  if(this->m_debug)
    m_counterUpdate.clear();

  for(vector<CMType>::const_iterator CM = m_CMLabels.begin();
      CM != m_CMLabels.end(); ++CM) {
    m_cmCosts[*CM] = 1;
    m_cmWeights[*CM] = 1;
    m_cmProbabilities[*CM] = double(1.0 / m_CMLabels.size());
    m_cmProbabilitiesWithNoCost[*CM] = double(1.0 / m_CMLabels.size());
    if(this->m_debug)
      m_counterUpdate[*CM] = 0;
  }
}

template<class MPTraits>
template<typename InputIterator1, typename InputIterator2, typename OutputIterator>
void
SpatialAdaptiveConnector<MPTraits>::
Connect(RoadmapType* _rm,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision) {

  if(m_cmProbabilities.empty() )
    Initialize();

  for(InputIterator1 itr1 = _itr1First; itr1 < _itr1Last; ++itr1) {
    VID vid = _rm->GetGraph()->GetVID(itr1);
    CfgRef vCfg = _rm->GetGraph()->GetVertex(itr1);
    if(m_cmConnected[this->m_lastUse] != 0 ) {
      vector<pair<VID, double> > localN;
      NeighborhoodFinderPointer nfptrlocal =
        this->GetMPProblem()->GetNeighborhoodFinder(m_localCM.first);
      nfptrlocal->FindNeighbors(_rm, _itr2First, _itr2Last, _fromFullRoadmap,
          vCfg, back_inserter(localN));
      for(typename vector<pair<VID, double> >::
          iterator nit = localN.begin(); nit != localN.end(); ++nit)  {

        RewardUpdateProbability(nit->first,m_reward, m_cost,m_prevConnectionAttempt);
        this->m_lastUse = UpdateCMChoice(nit->first);
      }


    }
    m_cmConnected[this->m_lastUse]++;


    if(this->m_debug)
      cout << (itr1 - _itr1First)
        << "\tAttempting connections: VID = "
        << vid << "  --> Cfg = " << vCfg << endl;

    vector<pair<VID, double> > closest;
    NeighborhoodFinderPointer nfptr =
      this->GetMPProblem()->GetNeighborhoodFinder(this->m_lastUse.first);
    nfptr->FindNeighbors(_rm, _itr2First, _itr2Last, _fromFullRoadmap, vCfg,
        back_inserter(closest));
    if(this->m_debug) {
      cout << "Neighbors | ";
      for(typename vector<pair<VID, double> >::iterator nit =
          closest.begin(); nit!=closest.end(); ++nit)
        cout << nit->first << " ";
    }

    ConnectNeighbors(_rm,vid, closest.begin(),
        closest.end(), _collision);

  }

}

template<class MPTraits>
template <typename InputIterator, typename OutputIterator>
void
SpatialAdaptiveConnector<MPTraits>::
ConnectNeighbors(RoadmapType* _rm, VID _vid,
    InputIterator _closestFirst, InputIterator _closestLast,
    OutputIterator _collision) {

  Environment* env = this->GetMPProblem()->GetEnvironment();
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(
      this->m_lastUse.second);
  GraphType* map = _rm->GetGraph();

  LPOutput<MPTraits> lpOutput;
  size_t failure = 0;

  static double prevConnectionSuccess=0;
  static unsigned long int prevConnectionCollision=0;

  for(InputIterator itr2 = _closestFirst; itr2 < _closestLast; ++itr2) {
    VID v2 = itr2->first;
    if(this->m_debug)
      cout << "\tfailures = " << failure
        << " | VID = " << v2
        << " | dist = " << itr2->second;

    // stopping conditions
    if(this->m_countFailures && failure >= m_fail) {
      if(this->m_debug) cout << " | stopping... failures exceeded" << endl;
      break;
    }

    // don't attempt the connection if it already failed once before
    if(this->IsCached(_vid, v2) && !this->GetCached(_vid, v2)) {
      if(this->m_debug) {
        cout << " | skipping... this connection already failed once"
          << " | failure incremented" << endl;
      }
      failure++;
      continue;
    }

    // if the edge already exists, no need to call LP.
    if(map->IsEdge(_vid, v2)) {
      if(this->m_debug)
        cout << " | edge already exists in roadmap | skipping" << endl;
      continue;
    }

    if(m_checkIfSameCC) {
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
    //int countid = 0, nid1=0, nid2=0, nid3 =0, nid4=0;
    if(connectable) {
      if(this->m_debug)
        cout << " | connection was successful | success incremented" << endl;
      // increment # of successful connection attempts
      c1.IncStat("succConnectionAttempts", 1);
      c2.IncStat("succConnectionAttempts", 1);
      _rm->GetGraph()->AddEdge(_vid, v2, lpOutput.m_edge);
      // if connection was made, add edge and record the successful connection
      // I don't know what this section is for...
      /* why do we stop at 3?
      _rm->GetGraph()->AddEdge(_vid, v2, lpOutput.m_edge);
      countid++;
      if (m_lastUseID[m_lastUse] == 1)
        nid1++;
      else if (m_lastUseID[m_lastUse] == 2)
        nid2++;
      else if (m_lastUseID[m_lastUse] == 3)
        nid3++;
      else
        nid4++;
      if(countid ==50)
      {
        for (int i = 1; i<countid; i++){
          if (nid1 > nid2 &&  nid1>nid3 && nid1 > nid4)
            m_lastUseID[m_lastUse] = 1;
          else if (nid2 > nid1 &&  nid2>nid3 && nid2>nid4)
            m_lastUseID[m_lastUse] = 2;
          else if (nid3 > nid1 &&  nid3>nid2 && nid3>nid4)
            m_lastUseID[m_lastUse] = 3;
          else if (nid4 > nid1 &&  nid4>nid2 && nid4>nid3)
            m_lastUseID[m_lastUse] = 4;

          cout<<"i"<<i;
        }
      }*/

      double  currSuccess  =   get<1>(this->GetStatClass()->m_lpInfo.begin()->second);
      double  currAttempts = currSuccess + failure;
      unsigned long int  currCollision = this->GetStatClass()->GetIsCollTotal();


      if(this->m_debug) {
        cout<<"currAttempts: "<<currAttempts<<endl;
        cout<<"currSuccess: "<<currSuccess<<endl;
      }

      if(m_lastUse.first !="" && currAttempts != 0) {

        m_reward = currSuccess / currAttempts;
        if (currAttempts - m_prevConnectionAttempt !=0) {
          m_reward = (currSuccess - prevConnectionSuccess) /
            (currAttempts -m_prevConnectionAttempt);
          m_cost = (double)(currCollision - prevConnectionCollision);

        }


      }

      if(this->m_debug) {
        cout<<"curr collision"<<currCollision<<endl;
        cout<<"pre collision"<<prevConnectionCollision<<endl;
      }
      m_prevConnectionAttempt =  currAttempts;
      prevConnectionSuccess =  currSuccess;
      prevConnectionCollision = currCollision;

      int numConnected = m_cmConnected[this->m_lastUse];
      m_local_info[v2].push_back(make_tuple(make_pair(
              this->m_lastUse,numConnected), m_cost, m_reward));

    }
    else {
      if(this->m_debug) cout << " | connection failed | failure incremented" << endl;
      failure++;
    }
  }
}



template<class MPTraits>
void
SpatialAdaptiveConnector<MPTraits>::
RewardUpdateProbability(VID _nit, double _reward, unsigned long int _cost,
    int _prevConnectionAttempt) {
  cout << endl << endl;
  int cmSize = m_CMLabels.size();
  int neighborSize = m_local_info[_nit].size();
  if(this->m_debug)
    cout<<"number of local neighbors = "<< neighborSize <<endl;

  for(vector<pc_data>::iterator It = m_local_info[_nit].begin(); It != m_local_info[_nit].end();
      It++) {

    if((get<0>(*It)).first == this->m_lastUse) {
      //update costs
      if(!m_fixedCost){
        int prevAvgCost = m_cmCosts[this->m_lastUse];
        int newAvgCost= (prevAvgCost *((get<0>(*It)).second -1)
            + _cost)/(get<0>(*It)).second;
        m_cmCosts[this->m_lastUse] = newAvgCost;
      }

      //update weight
      if(!m_fixedReward) {
        if (_prevConnectionAttempt !=0) {
          double adjustedReward = _reward/m_cmProbabilitiesWithNoCost[this->m_lastUse];
          double newWeight = exp(double((m_percentageRandom) * adjustedReward
                / double(cmSize)));
          //from the paper, this should be  (*=) instead. However doing that results in
          //infinite values for weight.
          m_cmWeights[this->m_lastUse] = newWeight;
        }

      }
    }
    else {
      if(!m_fixedCost){
        int prevAvgCost = m_cmCosts[(get<0>(*It)).first];
        int newAvgCost = (prevAvgCost * ((get<0>(*It)).second -1)
            + get<1>(*It)) / (get<0>(*It)).second;

        m_cmCosts[(get<0>(*It)).first] = newAvgCost;
      }

      //update weight
      if(!m_fixedReward) {
        if (_prevConnectionAttempt != 0) {
          double adjustedReward = get<2>(*It)
            / m_cmProbabilitiesWithNoCost[(get<0>(*It)).first];
          double newWeight = exp(double((m_percentageRandom) * adjustedReward
                / double(cmSize)));
          m_cmWeights[(get<0>(*It)).first] = newWeight;
          m_cmWeights[(get<0>(*It)).first] = newWeight;
        }
      }
    }
  }

  //update probability
  double weightTotal= 0;
  for(vector<pc_data>::iterator It = m_local_info[_nit].begin(); It != m_local_info[_nit].end();
      It++) {
    weightTotal += m_cmWeights[(get<0>(*It)).first];
  }
  if(this->m_debug)
    cout<<"total weight = " << weightTotal << endl;
  double probtotal =0;
  double probCostTotal = 0.0;
  for(vector<pc_data>::iterator It = m_local_info[_nit].begin(); It != m_local_info[_nit].end();
      It++) {
    // in the paper m is the number of cm and not of neighbors
    //m_cmProbabilitiesWithNoCost[(get<0>(m_local_info[_nit][_i])).first]
    // = (1 - m_percentageRandom) * (m_cmWeights[(get<0>(m_local_info[_nit][_i])).first]
    //   / weightTotal) + (m_percentageRandom /m_local_info[_nit].size());

    m_cmProbabilitiesWithNoCost[(get<0>(*It)).first]
      = (1 - m_percentageRandom) * (m_cmWeights[(get<0>(*It)).first]
          / weightTotal) + (m_percentageRandom / cmSize);

    probtotal += m_cmProbabilitiesWithNoCost[(get<0>(*It)).first];
    probCostTotal += double(m_cmProbabilitiesWithNoCost[(get<0>(*It)).first]
        / double(m_cmCosts[(get<0>(*It)).first]));
    if(this->m_debug)
      cout <<"denominator cost: "
        << m_cmCosts[(get<0>(*It)).first] << endl;
  }
  if(this->m_debug) {
    cout << "prob with cost : ";
    cout<<probCostTotal<<endl;
  }

  double probs = 0;
  if(this->m_debug)
    cout << "Normalized probs: ";
  for(vector<pc_data>::iterator It = m_local_info[_nit].begin(); It != m_local_info[_nit].end();
      It++) {
    //this is multiplying instead of dividing from formula 3 in paper
    /*m_cmProbabilitiesnormal[(get<0>(m_local_info[_nit][_i])).first]
      =  ((m_cmProbabilitiesWithNoCost[(get<0>(m_local_info[_nit][_i])).first]
      / double(m_cmCosts[(get<0>(m_local_info[_nit][_i])).first]) ) * probCostTotal);  //+ (m_percentageRandom /m_local_info[_nit].size()) + 0.5;
      */
    m_cmProbabilitiesnormal[(get<0>(*It)).first]
      =  m_cmProbabilitiesWithNoCost[(get<0>(*It)).first]
      / (double(m_cmCosts[(get<0>(*It)).first]) * probCostTotal);
    if(this->m_debug)
      cout << m_cmProbabilitiesnormal[(get<0>(*It)).first] << " ";

  }
  cout<<endl;
  double max= 0.0;
  double min=0.0;
  max = std::max_element(m_cmProbabilitiesnormal.begin(),
      m_cmProbabilitiesnormal.end())->second ;
  min =std::min_element(m_cmProbabilitiesnormal.begin(),
      m_cmProbabilitiesnormal.end())->second ;
  if(this->m_debug) {
    cout<<"min: "<<min<<endl;
    cout<<"max: "<<max<<endl;
    cout << "new probabilities normalised: ";
  }
  for(vector<pc_data>::iterator It = m_local_info[_nit].begin(); It != m_local_info[_nit].end();
      It++) {
    if(max -  min == 0) {
      m_cmProbabilities[(get<0>(*It)).first] = 1 ;
    }
    else
      m_cmProbabilities[(get<0>(*It)).first]
        = (m_cmProbabilitiesnormal[(get<0>(*It)).first] * min)
        / (max - min) ;

    probs += m_cmProbabilities[(get<0>(*It)).first];
    if(this->m_debug)
      cout<<m_cmProbabilities[(get<0>(*It)).first]<<" ";
  }

  if(this->m_debug) {
    cout<<endl;
    cout<<"real probs"<<probs<<endl;
  }
  if(this->m_debug)
    cout << endl;

  if(!m_fixedReward) {
    double smallestWeight = 1;
    for(vector<pc_data>::iterator It = m_local_info[_nit].begin(); It != m_local_info[_nit].end();
        It++) {
      double weight = m_cmWeights[(get<0>(*It)).first];

      if(weight < smallestWeight || smallestWeight == -1)
        smallestWeight = weight;
    }
    for(vector<pc_data>::iterator It = m_local_info[_nit].begin(); It != m_local_info[_nit].end();
        It++) {
      m_cmWeights[(get<0>(*It)).first]
        = m_cmWeights[(get<0>(*It)).first] / smallestWeight;
    }
  }
}

template<class MPTraits>
typename SpatialAdaptiveConnector<MPTraits>::CMType
SpatialAdaptiveConnector<MPTraits>::
UpdateCMChoice(VID _nit) {
  double proSum = 0;
  double nfProb =0;
  map<CMType, pair<double, double> > mapProRange;

  int neighborSize = m_local_info[_nit].size();
  if(this->m_debug)
    cout << "\nmethod\tprob\trange_min\trange_max\n";

  for(vector<pc_data>::iterator It = m_local_info[_nit].begin(); It != m_local_info[_nit].end();
     It++) {

    if(m_setUniform)
      nfProb = double(1.0/m_CMLabels.size());
    else
      nfProb = m_cmProbabilities[(get<0>(*It)).first];
    double upperBound = 0;
    if(It + 1 < m_local_info[_nit].end())
      upperBound = proSum + nfProb;
    else
      upperBound = 1.0; // do this only in the last number
    mapProRange[(get<0>(*It)).first] = make_pair(proSum, upperBound);
    proSum += nfProb;

    if(this->m_debug)
      cout << (get<0>(*It)).first.first <<"_"
        << (get<0>(*It)).first.second
        << "\t" << nfProb << "\t"
        << mapProRange[(get<0>(*It)).first].first
        << "\t" << mapProRange[(get<0>(*It)).first].second
        << endl;
  }

  if(m_setUniform) {
    CMType maxGen = make_pair("", "");
    int rand =int(DRand() * m_CMLabels.size()) ;
    maxGen= m_CMLabels[rand];
    return maxGen;
  }
  else {
    double randomNum = DRand();
    if(neighborSize < 2) {
      CMType maxGen = make_pair("", "");
      int rand =int(DRand() * m_CMLabels.size()) ;
      maxGen= m_CMLabels[rand];
      return maxGen;
    }
    else {
  for(vector<pc_data>::iterator It = m_local_info[_nit].begin(); It != m_local_info[_nit].end();
     It++) {
        if(mapProRange[(get<0>(*It)).first].first <= randomNum
            || randomNum < mapProRange[(get<0>(*It)).first].second ) {
          if(this->m_debug) {
            cout << "randomNum = "
              << randomNum
              << "\tselecting method "
              << (get<0>(*It)).first.first <<"_"
              << (get<0>(*It)).first.second
              << endl;

            m_counterUpdate[(get<0>(*It)).first]++;
            cout << (get<0>(*It)).first.first <<"_"
              << (get<0>(*It)).first.second
              << "==" << "attempts"
              << "\t\t" << m_counterUpdate[(get<0>(*It)).first]
              <<endl;
          }
          return (get<0>(*It)).first;
        }
      }
      cerr << "Neigbor Finder not picked, Exiting \n" ;
      exit(1);
    }
  }
}

template<class MPTraits>
void
SpatialAdaptiveConnector<MPTraits>::
PrintData(ostream& _os) {
  _os << "\nmethod\tprob\tprob_no_m_cost\tweight\tavg_cost\ttimes_used\n";
  for(vector<CMType>::const_iterator CM = m_CMLabels.begin();
      CM < m_CMLabels.end(); ++CM) {


    _os << CM->first <<", "<< CM->second << "\t";
    _os << m_cmProbabilities[*CM] << "\t";
    _os << m_cmProbabilitiesWithNoCost[*CM] << "\t";
    _os << m_cmWeights[*CM] << "\t";
    _os << m_cmCosts[*CM] << "\t";
    _os << m_cmConnected[*CM] << "\t";
    _os << endl;
  }
  _os << "last used = " << m_lastUse.first <<", "<< m_lastUse.second << endl;
  _os<<"Connection ID"<<endl;
  _os<<m_lastUseID[m_lastUse]<<endl;
}

#endif
