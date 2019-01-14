#ifndef UNIFORM_OBSTACLE_BASED_SAMPLER_H_
#define UNIFORM_OBSTACLE_BASED_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// This sampler generates obstacle-based configurations that uniformly cover
/// the contact surface.
///
/// @TODO Add paper reference.
///
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
template<typename MPTraits>
class UniformObstacleBasedSampler : public SamplerMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    UniformObstacleBasedSampler(string _vcLabel = "", string _dmLabel = "",
        double _margin = 0, bool _useBoundary = false);

    UniformObstacleBasedSampler(XMLNode& _node);

    virtual ~UniformObstacleBasedSampler() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    void Print(ostream& _os) const override;

    ///@}
    ///@name Sampler Interface
    ///@{

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    double m_margin;
    bool m_useBoundary;
    string m_vcLabel, m_dmLabel;

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
      "set the bounding box whose margin is d away from obstacles");
  m_useBoundary = _node.Read("useBBX", true, false,
      "Use bounding box as obstacle");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_dmLabel =_node.Read("dmLabel", true, "default", "Distance Metric Method");
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
    vector<CfgType>& _result, vector<CfgType>& _collision) {
  Environment* env = this->GetEnvironment();
  string callee(this->GetNameAndLabel() + "::SampleImpl()");
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto robot = this->GetTask()->GetRobot();

  bool generated = false;
  int attempts = 0;
  bool cfg1Free;
  double margin = m_margin;
  if(margin == 0)
    margin = _cfg.GetMultiBody()->GetMaxAxisRange();

  vector<pair<double, double> > origBoundary;
  for(size_t i = 0; i < 3; i++) {
    const auto& r = _boundary->GetRange(i);
    origBoundary.emplace_back(r.min, r.max);
  }

  env->ResetBoundary(margin, _cfg.GetMultiBody());

  attempts++;
  //Generate first cfg
  CfgType& cfg1 = _cfg;

  cfg1Free = vc->IsValid(cfg1, callee);

  CfgType cfg2(robot);
  CfgType incr(robot);

  incr.GetRandomRay(margin, dm);
  cfg2 = cfg1 + incr;

  //TODO: Check GetRandomRay actually scales the correct distance.
  //scale the distance between c1 and c2
  /*Vector3d c1, c2, dir;
  for(size_t i = 0; i < CfgType::PosDOF(); ++i) {
    c1[i] = cfg1[i];
    c2[i] = cfg2[i];
  }
  dir = c2 - c1;
  double dist = dir.norm();
  double r = margin/dist;
  cfg2 = cfg1 + incr*r;*/

  CfgType inter(robot);
  CfgType tick = cfg1;
  int nTicks;
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  bool tempFree = cfg1Free;
  bool tickFree;
  CfgType temp = cfg1;

  inter.FindIncrement(cfg1, cfg2, &nTicks, positionRes, orientationRes);
  env->ResetBoundary(origBoundary, 0);
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
