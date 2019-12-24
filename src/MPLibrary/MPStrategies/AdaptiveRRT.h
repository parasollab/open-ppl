#ifndef ADAPTIVE_RRT_H_
#define ADAPTIVE_RRT_H_

#include <stdint.h>

//assembly code to measure cpu cycles
static inline uint64_t GetCycles(){
  uint64_t n;
  __asm__ __volatile__ ("rdtsc" : "=A"(n));
  return n;
}

#include "BasicRRTStrategy.h"

////////////////////////////////////////////////////////////////////////////////
/// Adaptively selects growth methods in RRT.
///
/// AdaptiveRRT employs structural filtering to the RRT paradigm by providing a
/// two-level cost-adaptive strategy to select the RRT growth method. First,
/// it uses the "visibility" of a node the method selects a set of RRT methods
/// to choose from based on some probability distribution. This probability
/// distribution is updated based upon the success/fail of the growth and its
/// cost.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class AdaptiveRRT : public BasicRRTStrategy<MPTraits> {
  public:

    //cost calculation for the AdaptiveRRT
    enum CostMethod {FIXED, REWARD, CYCLES};
    //std::map<std::string, std::pair<double, double> > is a single Growth Set representing
    //growth method, cost, weight tuple
    typedef std::map<std::string, std::pair<std::pair<double, long>, double> > GrowthSet;
    //double is visibility threshold for the GrowthSet
    typedef std::map<double, GrowthSet> GrowthSets;

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    AdaptiveRRT(double _wallPenalty = 0.5, double _gamma = 0.5,
        const GrowthSets& _growthSets = GrowthSets(), CostMethod _c = FIXED);
    AdaptiveRRT(XMLNode& _node);

    virtual void Initialize();

    virtual void Print(std::ostream& _os) const;

  protected:
    // Helper functions
    virtual VID ExpandTree(CfgType& _dir);
    std::string SelectGrowthMethod(GrowthSet& _gs);
    void UpdateCost(double _cost, std::string _s, GrowthSet& _gs);
    void RewardGrowthMethod(double _r, std::string _s, GrowthSet& _gs);
    VID UpdateTree(VID _nearest, CfgType& _new, CfgType& _dir, double _delta);
    VID UpdateTree(CfgType& _newCfg, VID _nearVID, bool _againstWall,
        double _ratio);


  private:
    //This function incorporates _val into the running average of visibility in
    //_cfg
    void AvgVisibility(CfgType& _cfg, double _val);
    //simple helper to grab the visibility from _cfg
    double GetVisibility(CfgType& _cfg);
    double GetCost(std::string _s, GrowthSet& _gs);
    double CostInsensitiveProb(std::string _s, GrowthSet& _gs);

    double m_wallPenalty; //initial visibility for nodes which extend into C-obst
    double m_gamma;

    GrowthSets m_growthSets;

    CostMethod m_costMethod;
};

template <typename MPTraits>
AdaptiveRRT<MPTraits>::
AdaptiveRRT(double _wallPenalty, double _gamma, const GrowthSets& _growthSets,
    CostMethod _c) : BasicRRTStrategy<MPTraits>(),
    m_wallPenalty(_wallPenalty), m_gamma(_gamma), m_growthSets(_growthSets),
    m_costMethod(_c) {
  this->SetName("AdaptiveRRT");
}

template <typename MPTraits>
AdaptiveRRT<MPTraits>::
AdaptiveRRT(XMLNode& _node) : BasicRRTStrategy<MPTraits>(_node){
  this->SetName("AdaptiveRRT");

  for(auto& child : _node) {
    if(child.Name() == "GrowthSet"){
      double threshold = child.Read("threshold", true, 0.0, 0.0,
          1.0, "Threshold of visibility for selecting GrowthSet");
      GrowthSet growthSet;
      for(auto& child2 : child) {
        if(child2.Name() == "Extender"){
          std::string label = child2.Read("label", true, "", "Extender strategy");
          growthSet[label] = std::make_pair(std::make_pair(0, 0), 1.0);
        }
      }
      m_growthSets[threshold] = growthSet;
    }
  }

  m_wallPenalty = _node.Read("wallPenalty", false, 0.5, 0.0, 1.0,
      "Initial visibility for nodes agains C-obst");
  m_gamma = _node.Read("gamma", true, 0.5, 0.0, 1.0,
      "Gamma for adaptivity formulas");
  std::string costMethod = _node.Read("cost", true, "fixed", "Cost method");
  transform(costMethod.begin(), costMethod.end(),
      costMethod.begin(), ::tolower);

  if(costMethod == "fixed")
    m_costMethod = FIXED;
  else if(costMethod == "reward")
    m_costMethod = REWARD;
  else if(costMethod == "cycles")
    m_costMethod = CYCLES;
  else
    throw ParseException(_node.Where(), "Unknown cost method '" + costMethod +
        "'. Choices are 'fixed', 'reward', or 'cycles'.");
}

template <typename MPTraits>
void
AdaptiveRRT<MPTraits>::Print(std::ostream& _os) const {
  BasicRRTStrategy<MPTraits>::Print(_os);
  _os << "\tWallPenalty::" << m_wallPenalty << std::endl;
  _os << "\tGamma::" << m_gamma << std::endl;
  _os << "\tGrowthSets::" << std::endl;
  typedef typename GrowthSets::const_iterator GSIT;
  typedef typename GrowthSet::const_iterator GIT;
  for(GSIT gsit = m_growthSets.begin(); gsit!=m_growthSets.end(); ++gsit){
    _os << "\t\tGrowthSet::" << gsit->first << std::endl;
    for(GIT git = gsit->second.begin(); git!=gsit->second.end(); ++git){
      _os << "\t\t\t" << git->first << std::endl;
    }
  }
  _os << "\tCostMethod::";
  if(m_costMethod == FIXED) _os << "fixed";
  else if(m_costMethod == REWARD) _os << "reward";
  else _os << "cycles";
  _os << std::endl;
}

template <typename MPTraits>
void
AdaptiveRRT<MPTraits>::Initialize(){
  BasicRRTStrategy<MPTraits>::Initialize();

  //for each root of the graph, make sure to initialize variables.
  auto rdmp = this->GetRoadmap();
  for(auto v = rdmp->begin(); v!=rdmp->end(); v++){
    v->property().SetStat("Success", 1);
    v->property().SetStat("Fail", 0);
    v->property().SetStat("Visibility", 1);
  }
}

template <typename MPTraits>
typename AdaptiveRRT<MPTraits>::VID
AdaptiveRRT<MPTraits>::ExpandTree(CfgType& _dir){

  // Setup MP Variables
  auto dm = this->GetDistanceMetric(this->m_dmLabel);
  VID recentVID = INVALID_VID;

  //get the expand node from the roadmap
  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  std::vector<std::pair<VID, double> > kClosest;
  nf->FindNeighbors(this->GetRoadmap(),
      this->m_currentTree->begin(), this->m_currentTree->end(),
      this->m_currentTree->size() ==
      this->GetRoadmap()->get_num_vertices(),
      _dir, std::back_inserter(kClosest));
  CfgType& nearest =
      this->GetRoadmap()->GetVertex(kClosest[0].first);

  //get visibility of near node to decide which growth method to do
  double visibility = GetVisibility(nearest);

  if(this->m_debug)
    std::cout << "nearest:: " << nearest << "\tvisibility:: " << visibility << std::endl;

  double minDist = this->GetExtender(this->m_exLabel)->GetMinDistance();
  if(dm->Distance(_dir, nearest) < minDist){
    //chosen a q_rand which is too close. Penalize nearest with 0.
    nearest.IncrementStat("Fail");
    AvgVisibility(nearest, 0);
    return recentVID;
  }

  //select the growth set based upon the visibility of the nearest node
  GrowthSets::reverse_iterator rgsit = m_growthSets.rbegin();
  for(; rgsit!=m_growthSets.rend(); ++rgsit){
    if(visibility > rgsit->first){
      break;
    }
  }

  //select the growth method from the selected growth set
  std::string gm = SelectGrowthMethod(rgsit->second);

  //grow the RRT using growth method gm
  //start timing from cycles
  uint64_t start = GetCycles();

  CfgType newCfg(this->GetTask()->GetRobot());
  LPOutput<MPTraits> lpOutput;
  auto e = this->GetExtender(gm);
  bool verifiedValid = e->Extend(nearest, _dir, newCfg, lpOutput);
  double delta = e->GetMaxDistance();

  //end timing from cycles
  uint64_t end = GetCycles();
  uint64_t cost = end - start;

  //update cost based upon cycles
  if(m_costMethod == CYCLES)
    UpdateCost(cost, gm, rgsit->second);

  //expansion Failed Penalize nearest with 0
  if(!verifiedValid) {
    if(this->m_debug)
      std::cout << "Growth Failed on::" << nearest << ", with visibility::"
           << visibility << std::endl;
    nearest.IncrementStat("Fail");
    AvgVisibility(nearest, 0);

    if(m_costMethod == REWARD)
      UpdateCost(delta, gm, rgsit->second);

    //reward the growth strategy based upon expanded distance in proportion to
    //delta_q
    RewardGrowthMethod(-minDist, gm, rgsit->second);

    return recentVID;
  }

  // If good to go, add to roadmap
  double dist = dm->Distance(newCfg, nearest);

  if(m_costMethod == REWARD)
    UpdateCost(max(delta - dist, 0.0) + 1E-6, gm, rgsit->second);

  if(dist >= minDist) {
    //expansion success
    nearest.IncrementStat("Success");
    //update the tree
    //Generate Waypoints is from AdaptiveMultiResRRT, but this one does not
    //acutally add nodes.
    recentVID = UpdateTree(kClosest[0].first, newCfg, _dir, delta);
    if(recentVID > this->m_currentTree->back()) {
      this->m_currentTree->push_back(recentVID);
      //reward the growth strategy based upon expanded distance in proportion to
      //delta_q
      RewardGrowthMethod(dist/delta, gm, rgsit->second);
    }
    else {
      //node already existed in the roadmap. decrement reward
      RewardGrowthMethod(-minDist, gm, rgsit->second);
    }
  }
  else{
    //could not expand at least minDist. Penalize nearest with 0;
    nearest.IncrementStat("Fail");
    AvgVisibility(nearest, 0);
    RewardGrowthMethod(-minDist, gm, rgsit->second);
  }

  return recentVID;
}

template <typename MPTraits>
std::string
AdaptiveRRT<MPTraits>::SelectGrowthMethod(GrowthSet& _gs){
  if(this->m_debug)
    std::cout << ":::::Selecting Growth Method:::::" << std::endl;
  std::map<std::string, double> pistars, pis;
  double spc = 0.0;
  //compute p_i*/c_i and the sum over all growth methods
  for(GrowthSet::const_iterator gsit = _gs.begin(); gsit!=_gs.end(); ++gsit){
    if(this->m_debug)
      std::cout << "Method::" << gsit->first
        << "::Cost::" << GetCost(gsit->first, _gs)
        << "::Weight::" << gsit->second.second << std::endl;
    double pistar = CostInsensitiveProb(gsit->first, _gs)/GetCost(gsit->first,
        _gs);
    pistars[gsit->first] = pistar;
    spc += pistar;
  }

  //compute p_i for each method
  for(GrowthSet::const_iterator gsit = _gs.begin(); gsit!=_gs.end(); ++gsit){
    pis[gsit->first] = pistars[gsit->first]/spc;
    if(this->m_debug)
      std::cout << "Method::" << gsit->first
        << "::Prob::" << pis[gsit->first] << std::endl;
  }

  if(this->m_debug)
    std::cout << std::endl << std::endl;

  //select method based upon probability
  double r = DRand(), cumm = 0.0;
  for(std::map<std::string, double>::const_iterator mit = pis.begin(); mit!=pis.end();
      ++mit){
    cumm += mit->second;
    if(r <= cumm) {
      if(this->m_debug) {
        std::cout << "r::" << r << std::endl;
        std::cout << "MethodSelected::" << mit->first << std::endl;
      }
      return mit->first;
    }
  }

  std::cerr << "Error::Growth method not selected." << std::endl;
  std::exit(1);
}

template <typename MPTraits>
void
AdaptiveRRT<MPTraits>::UpdateCost(double _cost, std::string _s, GrowthSet& _gs){
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

template <typename MPTraits>
void
AdaptiveRRT<MPTraits>::RewardGrowthMethod(double _r, std::string _s, GrowthSet& _gs){
  if(this->m_debug)
    std::cout << "Reward::" << _s << "::" << _r << std::endl;
  //factor is e^(g * x_i' / K) where g is gamma, x_i' = x_i/p_i* where x_i is
  //the reward r, and K is the number of growth methods
  double factor = std::exp(m_gamma * (_r/CostInsensitiveProb(_s, _gs)) /
      double(_gs.size()));
  //update the weight on growth method _s
  _gs[_s].second *= factor;
}

//This function simply calls the correct update tree based on expansion
//type.
template <typename MPTraits>
typename AdaptiveRRT<MPTraits>::VID
AdaptiveRRT<MPTraits>::UpdateTree(VID _expandNode, CfgType& _newCfg,
    CfgType& _dir, double _delta){
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

  CfgType& nearest = this->GetRoadmap()->GetVertex(_expandNode);

  double visibility = GetVisibility(nearest);
  double distToNear = dm->Distance(nearest, _newCfg);
  //if expansion did not reach at least delta * visibility and q_new is not
  //q_rand. Then it will be a partial expansion
  bool partial = distToNear < _delta && _newCfg != _dir;
  double ratio = distToNear / _delta;

  VID returnVID = UpdateTree(_newCfg, _expandNode, partial, ratio);

  if(this->m_debug){
    std::cout << "near vid::" << _expandNode << "\tvisibility::" << visibility
         << std::endl;
    std::cout << "new vid::" << returnVID << "\tvisibility::"
         << GetVisibility(_newCfg) << std::endl;
    std::cout << "expansionRatio::" << ratio << std::endl;
  }

  return returnVID;
}

//add node to tree, and update visibility
template <typename MPTraits>
typename AdaptiveRRT<MPTraits>::VID
AdaptiveRRT<MPTraits>::UpdateTree(CfgType& _newCfg, VID _nearVID,
    bool _againstWall, double _ratio){
  auto rdmp = this->GetRoadmap();

  //add the vertex to the graph
  VID newVID = rdmp->AddVertex(_newCfg);

  //calculate edge weight and add edge
  CfgType& parentcfg = rdmp->GetVertex(_nearVID);
  if(this->m_debug)
    std::cout << "parentcfg::" << parentcfg << std::endl;
  int weight;
  CfgType incr(this->GetTask()->GetRobot());
  Environment* env = this->GetEnvironment();
  incr.FindIncrement(parentcfg, _newCfg, &weight, env->GetPositionRes(),
      env->GetOrientationRes());
  std::pair<WeightType, WeightType> weights = std::make_pair(WeightType("",
      weight), WeightType("", weight));
  rdmp->AddEdge(_nearVID, newVID, weights);

  //update the visibility at the new node and near node
  CfgType& newcfg = rdmp->GetVertex(newVID);

  //The new node is against a wall. Set visibility of new node to wall penalty.
  //Update parent with expansion ratio
  if(_againstWall){
    newcfg.SetStat("Visibility", m_wallPenalty);
    AvgVisibility(parentcfg, _ratio);
  }
  //newCfg inherits half of parents visibility and half of the ratio currently
  //parent is updated with ratio (mostly 1 in this case)
  else{
    newcfg.SetStat("Visibility", (_ratio + parentcfg.GetStat("Visibility"))/
        2.0);
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
template <typename MPTraits>
void
AdaptiveRRT<MPTraits>::AvgVisibility(CfgType& _cfg, double _val){
  double success = _cfg.GetStat("Success");
  double fail = _cfg.GetStat("Fail");
  double total = success+fail;
  _cfg.SetStat("Visibility", (_cfg.GetStat("Visibility")*(total-1) + _val)/
      total);
}

//Visibility is stored with the cfg. Isolate definition of visibility to this
//function.
template <typename MPTraits>
double
AdaptiveRRT<MPTraits>::GetVisibility(CfgType& _cfg){
  if(!_cfg.IsStat("Visibility")){
    std::cerr << "Warning::Visibility not a statistic of node::" << _cfg << std::endl;
    std::cerr << "Setting and Returning 1 as its visibility." << std::endl;
    _cfg.SetStat("Success", 1);
    _cfg.SetStat("Fail", 0);
    _cfg.SetStat("Visibility", 1);
  }
  return _cfg.GetStat("Visibility");
}

template <typename MPTraits>
double
AdaptiveRRT<MPTraits>::GetCost(std::string _s, GrowthSet& _gs){
  if(m_costMethod == FIXED)
    return 1;
  if(_gs[_s].first.second == 0)
    return 1;
  else
    return _gs[_s].first.first/(double)_gs[_s].first.second;
}

template <typename MPTraits>
double
AdaptiveRRT<MPTraits>::CostInsensitiveProb(std::string _s, GrowthSet& _gs){
  double sw = 0.0;
  for(GrowthSet::const_iterator gsit = _gs.begin(); gsit!=_gs.end(); ++gsit){
    sw+=log(gsit->second.second+1);
  }
  return (1.-m_gamma)*log(_gs[_s].second+1)/sw + m_gamma*double(_gs.size());
}

#endif
