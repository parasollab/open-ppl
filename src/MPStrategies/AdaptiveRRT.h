/**
 * AdaptiveRRT.h
 *
 * Description: AdaptiveRRT employs structural filtering to the RRT paradigm by
 * providing a two-level cost-adaptive strategy to select the RRT growth method.
 * First using the "visibility" of a node the method selects a set of RRT
 * methods to choose from based on some probability distribution. This
 * probability distribution is updated based upon the success/fail of the growth
 * and its cost.
 */

#ifndef ADAPTIVERRT_H_
#define ADAPTIVERRT_H_

#include <stdint.h>

//assembly code to measure cpu cycles
static inline uint64_t GetCycles(){
  uint64_t n;
  __asm__ __volatile__ ("rdtsc" : "=A"(n));
  return n;
}

#include "OBRRTStrategy.h"

template<class MPTraits>
class AdaptiveRRT : public OBRRTStrategy<MPTraits> {
  public:

    //cost calculation for the AdaptiveRRT
    enum CostMethod {FIXED, REWARD, CYCLES};
    //map<string, pair<double, double> > is a single Growth Set representing
    //growth method, cost, weight tuple
    typedef map<string, pair<pair<double, long>, double> > GrowthSet;
    //double is visibility threshold for the GrowthSet
    typedef map<double, GrowthSet> GrowthSets;

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;

    AdaptiveRRT(double _wallPenalty = 0.5, double _gamma = 0.5,
        const GrowthSets& _growthSets = GrowthSets(), CostMethod _c = FIXED);
    AdaptiveRRT(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void Initialize();

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os);

  protected:
    // Helper functions
    virtual VID ExpandTree(CfgType& _dir);
    string SelectGrowthMethod(GrowthSet& _gs);
    void UpdateCost(double _cost, string _s, GrowthSet& _gs);
    void RewardGrowthMethod(double _r, string _s, GrowthSet& _gs);
    CfgType Grow(string _s, CfgType& _nearest, CfgType& _dir, bool& _verifiedValid);
    VID UpdateTree(VID _nearest, CfgType& _new, CfgType& _dir);
    VID UpdateTree(CfgType& _newCfg, VID _nearVID, bool _againstWall, double _ratio);


  private:
    //This function incorporates _val into the running average of visibility in
    //_cfg
    void AvgVisibility(CfgType& _cfg, double _val);
    //simple helper to grab the visibility from _cfg
    double GetVisibility(CfgType& _cfg);
    double GetCost(string _s, GrowthSet& _gs);
    double CostInsensitiveProb(string _s, GrowthSet& _gs);

    double m_wallPenalty; //initial visibility for nodes which extend into C-obst
    double m_gamma;

    GrowthSets m_growthSets;

    CostMethod m_costMethod;
};

template<class MPTraits>
AdaptiveRRT<MPTraits>::AdaptiveRRT(double _wallPenalty, double _gamma,
    const GrowthSets& _growthSets, CostMethod _c) :
  m_wallPenalty(_wallPenalty), m_gamma(_gamma), m_growthSets(_growthSets), m_costMethod(_c) {
    this->SetName("AdaptiveRRT");
  }

template<class MPTraits>
AdaptiveRRT<MPTraits>::AdaptiveRRT(MPProblemType* _problem, XMLNodeReader& _node) :
  OBRRTStrategy<MPTraits>(_problem, _node, false, false){
    this->SetName("AdaptiveRRT");
    ParseXML(_node);
    _node.warnUnrequestedAttributes();
  };

template<class MPTraits>
void
AdaptiveRRT<MPTraits>::ParseXML(XMLNodeReader& _node) {
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr){
    if(citr->getName() == "GrowthSet"){
      double threshold = citr->numberXMLParameter("threshold", true, 0.0, 0.0, 1.0, "Threshold of visibility for selecting GrowthSet");
      GrowthSet growthSet;
      for(XMLNodeReader::childiterator citr2 = citr->children_begin(); citr2 != citr->children_end(); ++citr2){
        if(citr2->getName() == "Method"){
          string method = citr2->stringXMLParameter("method", true, "", "Growth strategy");
          growthSet[method] = make_pair(make_pair(0, 0), 1.0);
          citr2->warnUnrequestedAttributes();
        }
        else
          citr2->warnUnknownNode();
      }
      m_growthSets[threshold] = growthSet;
    }
    else
      citr->warnUnknownNode();
  }

  m_wallPenalty = _node.numberXMLParameter("wallPenalty", false, 0.5, 0.0, 1.0, "Initial visibility for nodes agains C-obst");
  m_gamma = _node.numberXMLParameter("gamma", true, 0.5, 0.0, 1.0, "Gamma for adaptivity formulas");
  string costMethod = _node.stringXMLParameter("cost", true, "fixed", "Cost method");
  transform(costMethod.begin(), costMethod.end(), costMethod.begin(), ::tolower);

  if(costMethod == "fixed")
    m_costMethod = FIXED;
  else if(costMethod == "reward")
    m_costMethod = REWARD;
  else if(costMethod == "cycles")
    m_costMethod = CYCLES;
  else{
    cerr << "Error::Unknown cost method in Adaptive RRT::" << costMethod << endl;
    exit(1);
  }
}

template<class MPTraits>
void
AdaptiveRRT<MPTraits>::PrintOptions(ostream& _os){
  BasicRRTStrategy<MPTraits>::PrintOptions(_os);
  _os << "\tWallPenalty::" << m_wallPenalty << endl;
  _os << "\tGamma::" << m_gamma << endl;
  _os << "\tGrowthSets::" << endl;
  typedef typename GrowthSets::iterator GSIT;
  typedef typename GrowthSet::iterator GIT;
  for(GSIT gsit = m_growthSets.begin(); gsit!=m_growthSets.end(); ++gsit){
    _os << "\t\tGrowthSet::" << gsit->first << endl;
    for(GIT git = gsit->second.begin(); git!=gsit->second.end(); ++git){
      _os << "\t\t\t" << git->first << endl;
    }
  }
  _os << "\tCostMethod::";
  if(m_costMethod == FIXED) _os << "fixed";
  else if(m_costMethod == REWARD) _os << "reward";
  else _os << "cycles";
  _os << endl;
}

template<class MPTraits>
void
AdaptiveRRT<MPTraits>::Initialize(){
  BasicRRTStrategy<MPTraits>::Initialize();

  //for each root of the graph, make sure to initialize variables.
  GraphType* rdmp = this->GetMPProblem()->GetRoadmap()->GetGraph();
  for(typename GraphType::VI v = rdmp->begin(); v!=rdmp->end(); v++){
    v->property().SetStat("Parent", 0);
    v->property().SetStat("Success", 1);
    v->property().SetStat("Fail", 0);
    v->property().SetStat("Visibility", 1);
  }
}

template<class MPTraits>
typename AdaptiveRRT<MPTraits>::VID
AdaptiveRRT<MPTraits>::ExpandTree(CfgType& _dir){

  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dm);
  VID recentVID = INVALID_VID;

  //get the expand node from the roadmap
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nf);
  vector<pair<VID, double> > kClosest;
  nf->FindNeighbors(this->GetMPProblem()->GetRoadmap(),
      this->m_currentTree->begin(), this->m_currentTree->end(),
      _dir, back_inserter(kClosest));
  CfgType& nearest = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVertex(kClosest[0].first);

  //get visibility of near node to decide which growth method to do
  double visibility = GetVisibility(nearest);

  if(this->m_debug)
    cout << "nearest:: " << nearest << "\tvisibility:: " << visibility << endl;

  if(dm->Distance(env, _dir, nearest) < this->m_minDist){
    //chosen a q_rand which is too close. Penalize nearest with 0.
    nearest.IncStat("Fail");
    AvgVisibility(nearest, 0);
    return recentVID;
  }

  //select the growth set based upon the visibility of the nearest node
  bool verifiedValid = false;
  GrowthSets::reverse_iterator rgsit = m_growthSets.rbegin();
  for(; rgsit!=m_growthSets.rend(); ++rgsit){
    if(visibility > rgsit->first){
      break;
    }
  }

  //select the growth method from the selected growth set
  string gm = SelectGrowthMethod(rgsit->second);

  //grow the RRT using growth method gm
  //start timing from cycles
  uint64_t start = GetCycles();

  CfgType newCfg = Grow(gm, nearest, _dir, verifiedValid);

  //end timing from cycles
  uint64_t end = GetCycles();
  uint64_t cost = end - start;

  //update cost based upon cycles
  if(m_costMethod == CYCLES)
    UpdateCost(cost, gm, rgsit->second);

  //expansion Failed Penalize nearest with 0
  if(!verifiedValid) {
    if(this->m_debug) cout << "Growth Failed on::" << nearest << ", with visibility::" << visibility << endl;
    nearest.IncStat("Fail");
    AvgVisibility(nearest, 0);

    if(m_costMethod == REWARD)
      UpdateCost(this->m_delta, gm, rgsit->second);

    return recentVID;
  }

  // If good to go, add to roadmap
  double dist = dm->Distance(env, newCfg, nearest);

  if(m_costMethod == REWARD)
    UpdateCost(max(this->m_delta - dist, 0.0) + 1E-6, gm, rgsit->second);

  if(dist >= this->m_minDist) {
    //expansion success
    nearest.IncStat("Success");
    //reward the growth strategy based upon expanded distance in proportion to
    //delta_q
    RewardGrowthMethod(dist/this->m_delta, gm, rgsit->second);
    //update the tree
    //Generate Waypoints is from AdaptiveMultiResRRT, but this one does not
    //acutally add nodes.
    recentVID = UpdateTree(kClosest[0].first, newCfg, _dir);
    this->m_currentTree->push_back(recentVID);
  }
  else{
    //could not expand at least m_minDist. Penalize nearest with 0;
    nearest.IncStat("Fail");
    AvgVisibility(nearest, 0);
  }

  return recentVID;
}

template<class MPTraits>
string
AdaptiveRRT<MPTraits>::SelectGrowthMethod(GrowthSet& _gs){
  if(this->m_debug) cout << ":::::Selecting Growth Method:::::" << endl;
  map<string, double> pistars, pis;
  double spc = 0.0;
  //compute p_i*/c_i and the sum over all growth methods
  for(GrowthSet::const_iterator gsit = _gs.begin(); gsit!=_gs.end(); ++gsit){
    if(this->m_debug)
      cout << "Method::" << gsit->first
        << "::Cost::" << GetCost(gsit->first, _gs)
        << "::Weight::" << gsit->second.second << endl;
    double pistar = CostInsensitiveProb(gsit->first, _gs)/GetCost(gsit->first, _gs);
    pistars[gsit->first] = pistar;
    spc += pistar;
  }

  //compute p_i for each method
  for(GrowthSet::const_iterator gsit = _gs.begin(); gsit!=_gs.end(); ++gsit){
    pis[gsit->first] = pistars[gsit->first]/spc;
    if(this->m_debug)
      cout << "Method::" << gsit->first
        << "::Prob::" << pis[gsit->first] << endl;
  }

  if(this->m_debug) cout << endl << endl;

  //select method based upon probability
  double r = DRand(), cumm = 0.0;
  for(map<string, double>::const_iterator mit = pis.begin(); mit!=pis.end(); ++mit){
    cumm += mit->second;
    if(r <= cumm) {
      if(this->m_debug) {
        cout << "r::" << r << endl;
        cout << "MethodSelected::" << mit->first << endl;
      }
      return mit->first;
    }
  }

  cerr << "Error::Growth method not selected." << endl;
  exit(1);
}

template<class MPTraits>
void
AdaptiveRRT<MPTraits>::UpdateCost(double _cost, string _s, GrowthSet& _gs){
  //update the average cost of the growth method
  if(_gs[_s].first.second == 0){
    _gs[_s].first.second = 1;
    _gs[_s].first.first = _cost;
  }
  else{
    _gs[_s].first.first += _cost;
    _gs[_s].first.second++;
  }
}

template<class MPTraits>
void
AdaptiveRRT<MPTraits>::RewardGrowthMethod(double _r, string _s, GrowthSet& _gs){
  if(this->m_debug) cout << "Reward::" << _s << "::" << _r << endl;
  //factor is e^(g * x_i' / K) where g is gamma, x_i' = x_i/p_i* where x_i is
  //the reward r, and K is the number of growth methods
  double factor = exp(m_gamma * (_r/CostInsensitiveProb(_s, _gs)) / double(_gs.size()));
  //update the weight on growth method _s
  _gs[_s].second *= factor;
}

template<class MPTraits>
typename AdaptiveRRT<MPTraits>::CfgType
AdaptiveRRT<MPTraits>::Grow(string _s, CfgType& _nearest, CfgType& _dir, bool& _verifiedValid){
  if(_s == "G0")
    return this->g0(_nearest, _dir, _verifiedValid);
  else if(_s == "G1")
    return this->g1(_nearest, _dir, _verifiedValid);
  else if(_s == "G2")
    return this->g2(_nearest, _dir, _verifiedValid, false);
  else if(_s == "G3")
    return this->g2(_nearest, _dir, _verifiedValid, true);
  else if(_s == "G4")
    return this->g4(_nearest, _dir, _verifiedValid);
  else if(_s == "G5")
    return this->g5(_nearest, _dir, _verifiedValid, false);
  else if(_s == "G6")
    return this->g5(_nearest, _dir, _verifiedValid, true);
  else if(_s == "G7")
    return this->g7(_nearest, _dir, _verifiedValid);
  else if(_s == "G8")
    return this->g8(_nearest, _dir, _verifiedValid);
  else{
    cerr << "Error::Invalid growth method requested::" << _s << endl;
    exit(1);
  }
}

//This function simply calls the correct update tree based on expansion
//type.
template<class MPTraits>
typename AdaptiveRRT<MPTraits>::VID
AdaptiveRRT<MPTraits>::UpdateTree(VID _expandNode, CfgType& _newCfg, CfgType& _dir){
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dm);

  CfgType& nearest = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVertex(_expandNode);

  double visibility = GetVisibility(nearest);
  double distToNear = dm->Distance(env, nearest, _newCfg);
  //if expansion did not reach at least m_delta * visibility and q_new is not q_rand
  //Then it will be a partial expansion
  bool partial = distToNear < this->m_delta && _newCfg != _dir;
  double ratio = distToNear / this->m_delta;

  VID returnVID = UpdateTree(_newCfg, _expandNode, partial, ratio);

  if(this->m_debug){
    cout << "near vid::" << _expandNode << "\tvisibility::" << visibility << endl;
    cout << "new vid::" << returnVID << "\tvisibility::" << GetVisibility(_newCfg) << endl;
    cout << "expansionRatio::" << ratio << endl;
  }

  return returnVID;
}

//add node to tree, and update visibility
template<class MPTraits>
typename AdaptiveRRT<MPTraits>::VID
AdaptiveRRT<MPTraits>::UpdateTree(CfgType& _newCfg, VID _nearVID, bool _againstWall, double _ratio){
  GraphType* rdmp = this->GetMPProblem()->GetRoadmap()->GetGraph();

  //add the vertex to the graph
  VID newVID = rdmp->AddVertex(_newCfg);

  //calculate edge weight and add edge
  CfgType& parentcfg = rdmp->GetVertex(_nearVID);
  if(this->m_debug) cout << "parentcfg::" << parentcfg << endl;
  int weight;
  CfgType incr;
  Environment* env = this->GetMPProblem()->GetEnvironment();
  incr.FindIncrement(parentcfg, _newCfg, &weight, env->GetPositionRes(), env->GetOrientationRes());
  pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", weight), WeightType("RRTExpand", weight));
  rdmp->AddEdge(_nearVID, newVID, weights);

  //update the visibility at the new node and near node
  CfgType& newcfg = rdmp->GetVertex(newVID);
  newcfg.SetStat("Parent", _nearVID);

  //The new node is against a wall. Set visibility of new node to wall penalty.
  //Update parent with expansion ratio
  if(_againstWall){
    newcfg.SetStat("Visibility", m_wallPenalty);
    AvgVisibility(parentcfg, _ratio);
  }
  //newCfg inherits half of parents visibility and half of the ratio currently
  //parent is updated with ratio (mostly 1 in this case)
  else{
    newcfg.SetStat("Visibility", (_ratio + parentcfg.GetStat("Visibility"))/2.0);
    AvgVisibility(parentcfg, _ratio);
  }

  //intialize success and fail
  newcfg.SetStat("Success", 1);
  newcfg.SetStat("Fail", 0);
  VDAddTempEdge(parentcfg, _newCfg);

  return newVID;
}

//note:: this function assumes that the total attempts has already been updated
//which is why we multiply by (total-1) and divide by (total) instead of (total)
//and (total+1)
template<class MPTraits>
void
AdaptiveRRT<MPTraits>::AvgVisibility(CfgType& _cfg, double _val){
  double success = _cfg.GetStat("Success");
  double fail = _cfg.GetStat("Fail");
  double total = success+fail;
  _cfg.SetStat("Visibility", (_cfg.GetStat("Visibility")*(total-1) + _val)/total);
}

//Visibility is stored with the cfg. Isolate definition of visibility to this
//function.
template<class MPTraits>
double
AdaptiveRRT<MPTraits>::GetVisibility(CfgType& _cfg){
  if(!_cfg.IsStat("Visibility")){
    cerr << "Warning::Visibility not a statistic of node::" << _cfg << endl;
    cerr << "Setting and Returning 1 as its visibility." << endl;
    _cfg.SetStat("Success", 1);
    _cfg.SetStat("Fail", 0);
    _cfg.SetStat("Visibility", 1);
  }
  return _cfg.GetStat("Visibility");
}

template<class MPTraits>
double
AdaptiveRRT<MPTraits>::GetCost(string _s, GrowthSet& _gs){
  if(m_costMethod == FIXED)
    return 1;
  if(_gs[_s].first.second == 0)
    return 1;
  else
    return _gs[_s].first.first/(double)_gs[_s].first.second;
}

template<class MPTraits>
double
AdaptiveRRT<MPTraits>::CostInsensitiveProb(string _s, GrowthSet& _gs){
  double sw = 0.0;
  for(GrowthSet::const_iterator gsit = _gs.begin(); gsit!=_gs.end(); ++gsit){
    sw+=gsit->second.second;
  }
  return (1.-m_gamma)*_gs[_s].second/sw + m_gamma*double(_gs.size());
}

#endif
