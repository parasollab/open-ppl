#ifndef UNIFORM_OBSTACLE_BASED_SAMPLER_H_
#define UNIFORM_OBSTACLE_BASED_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// This sampler generates obstacle-based configurations that uniformly cover
/// the contact surface. It first generates a set of uniformly distributed fixed length
/// segments, and then tests intermediate points on the segments in order to find 
/// valid configurations adjacent to invalid configurations. Those are retained as
/// roadmap nodes.
///
/// Reference: https://bit.ly/3P1f8dq
///   Hsin-Yi Yeh and Shawna Thomas and David Eppstein and Nancy M. Amato. 
///   "UOBPRM: A uniformly distributed obstacle-based PRM". TRO 2012.
///
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
template<typename MPTraits>
class UniformObstacleBasedSampler : virtual public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    UniformObstacleBasedSampler(XMLNode& _node);

    virtual ~UniformObstacleBasedSampler() = default;

    UniformObstacleBasedSampler(string _vcLabel = "", string _dmLabel = "",
    double _margin = 0, bool _useBoundary = false);

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    void Print(ostream& _os) const override;

    ///@}
    ///@name Sampler Interface
    ///@{

    /// Takes the input configuration and applies the sampler rule to
    /// generate output configurations on the contact surface.
    /// @param _cfg The input configuration.
    /// @param _boundary The sampling boundary.
    /// @param _result The resulting output configurations. 
    /// @param _invalid The (optional) return for failed attempts. 
    /// @return true if a valid configuration was generated, false otherwise.
    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _invalid) override;

    virtual bool Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
    vector<GroupCfgType>& _result, vector<GroupCfgType>& _invalid) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    double m_margin; //The length of line segments 
    bool m_useBoundary; //Use bounding box as obstacle 
    string m_vcLabel, m_dmLabel; //Validity checker label, distance metric label

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
UniformObstacleBasedSampler<MPTraits>::
UniformObstacleBasedSampler(string _vcLabel, string _dmLabel, double _margin,
    bool _useBoundary) :
    m_margin(_margin), m_useBoundary(_useBoundary),
    m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
  this->SetName("UniformObstacleBasedSampler");
}


template <typename MPTraits>
UniformObstacleBasedSampler<MPTraits>::
UniformObstacleBasedSampler(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {
  this->SetName("UniformObstacleBasedSampler");

  m_margin = _node.Read("d", true, 0.0, 0.0, MAX_DBL,
      "Set the length of line segments d away from obstacles"); 
  m_useBoundary = _node.Read("useBBX", true, false,
      "Use bounding box as obstacle");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity checker label");
  m_dmLabel =_node.Read("dmLabel", true, "default", "Distance metric label");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
UniformObstacleBasedSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tmargin = " << m_margin << endl
      << "\tuseBoundary = " << m_useBoundary << endl
      << "\tvcLabel = " << m_vcLabel << endl
      << "\tdmLabel = " << m_dmLabel << endl;
}

/*--------------------------- Sampler Interface ------------------------------*/

template <typename MPTraits>
bool
UniformObstacleBasedSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    vector<CfgType>& _result, vector<CfgType>& _invalid) {

  Environment* env = this->GetEnvironment();
  string callee(this->GetNameAndLabel() + "::SampleImpl()");

  //Check validity 
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto robot = this->GetTask()->GetRobot();

  bool generated = false;
  int attempts = 0;


  bool cfg1Free;
  double margin = m_margin != 0.
                ? m_margin
                : _cfg.GetMultiBody()->GetBase()->GetPolyhedron().GetMaxRadius();

  attempts++;

  //Generate first cfg
  CfgType& cfg1 = _cfg;

  cfg1Free = vc->IsValid(cfg1, callee);

  CfgType cfg2(robot);
  CfgType incr(robot);

  //Generate a random direction ray using margin and distance metric method.
  incr.GetRandomRay(margin, dm);  

  //Extend segment cfg1 with distance incr.
  cfg2 = cfg1 + incr;

  CfgType inter(robot);
  CfgType tick = cfg1;
  int nTicks;

  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();

  bool tempFree = cfg1Free;
  bool tickFree;
  CfgType temp = cfg1;

  inter.FindIncrement(cfg1, cfg2, &nTicks, positionRes, orientationRes);

  //Generate nTicks intermediate points along ray
  for(int i = 1; i < nTicks; i++) {
    tick += inter;
    tickFree = vc->IsValid(tick, callee);

    if(m_useBoundary)
      tickFree = tickFree && tick.InBounds(_boundary);

    if(tempFree == tickFree) {
      tempFree = tickFree;
      temp = tick;
    }
    else {	//tempFree != tickFree
      generated = true;

      //Store the vector created to _result 
      if(tempFree and temp.InBounds(_boundary)) {
        _result.push_back(temp);
        tempFree = tickFree;
        temp = tick;
      }
      else if(tickFree and tick.InBounds(_boundary)) { //tickFree
        _result.push_back(tick);
        tempFree = tickFree;
        temp = tick;
      }
    }
  }
  return generated;
}

/*----------------------------Group Sampler ---------------------*/

template <typename MPTraits>
bool
UniformObstacleBasedSampler<MPTraits>::
Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
    vector<GroupCfgType>& _result, vector<GroupCfgType>& _invalid) {

  Environment* env = this->GetEnvironment();
  string callee(this->GetNameAndLabel() + "::SampleImpl()");

  //Check validity 
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto groupRoadmap = this->GetGroupRoadmap();

  bool generated = false;
  int attempts = 0;

  bool cfg1Free;

  if (m_margin == 0) {
    throw RunTimeException(WHERE, "margin must not be equal to zero");
  }

  attempts++;

  //Generate first cfg
  GroupCfgType& cfg1 = _cfg;

  cfg1Free = vc->IsValid(cfg1, callee);

  GroupCfgType cfg2(groupRoadmap, false);
  GroupCfgType incr(groupRoadmap, false);

  //Generate a random direction ray using margin and distance metric method.
  //increment direction to group robots
  incr.GetRandomRay(m_margin, dm);

  //Extend segment cfg1 with distance incr.
  cfg2 = cfg1 + incr;

  GroupCfgType inter(groupRoadmap, false);
  GroupCfgType tick = cfg1;
  int nTicks;

  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();

  bool tempFree = cfg1Free;
  bool tickFree;
  GroupCfgType temp = cfg1;

  inter.FindIncrement(cfg1, cfg2, &nTicks, positionRes, orientationRes);

  //Generate nTicks intermediate points along ray
  for(int i = 1; i < nTicks; i++) {
    tick += inter;
    tickFree = vc->IsValid(tick, callee);

    if(m_useBoundary)
      tickFree = tickFree && tick.InBounds(_boundary);

    if(tempFree == tickFree) {
      tempFree = tickFree;
      temp = tick;
    }
    else {	//tempFree != tickFree
      generated = true;

      //Store the vector created to _result 
      if(tempFree and temp.InBounds(_boundary)) {
        _result.push_back(temp);
        tempFree = tickFree;
        temp = tick;
      }
      else if(tickFree and tick.InBounds(_boundary)) { //tickFree
        _result.push_back(tick);
        tempFree = tickFree;
        temp = tick;
      }
    }
  }
  return generated;
}



/*----------------------------------------------------------------------------*/

#endif
