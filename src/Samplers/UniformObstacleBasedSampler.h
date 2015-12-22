#ifndef UNIFORM_OBSTACLE_BASED_SAMPLER_H_
#define UNIFORM_OBSTACLE_BASED_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<typename MPTraits>
class UniformObstacleBasedSampler : public SamplerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    UniformObstacleBasedSampler(string _vcLabel = "", string _dmLabel = "",
        double _margin = 0, bool _useBoundary = false);

    UniformObstacleBasedSampler(MPProblemType* _problem, XMLNode& _node);

    void ParseXML(XMLNode& _node);
    void Print(ostream& _os) const;

    virtual bool Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision);

  private:
    double m_margin;
    bool m_useBoundary;
    string m_vcLabel, m_dmLabel;
};

template<class MPTraits>
UniformObstacleBasedSampler<MPTraits>::
UniformObstacleBasedSampler(string _vcLabel, string _dmLabel,
    double _margin, bool _useBoundary) :
  m_margin(_margin), m_useBoundary(_useBoundary),
  m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
    this->SetName("UniformObstacleBasedSampler");
  }

template<class MPTraits>
UniformObstacleBasedSampler<MPTraits>::
UniformObstacleBasedSampler(MPProblemType* _problem, XMLNode& _node) :
  SamplerMethod<MPTraits>(_problem, _node) {
    this->SetName("UniformObstacleBasedSampler");
    ParseXML(_node);
  }

template<class MPTraits>
void
UniformObstacleBasedSampler<MPTraits>::
ParseXML(XMLNode& _node) {
  m_margin = _node.Read("d", true, 0.0, 0.0, MAX_DBL,
      "set the bounding box whose margin is d away from obstacles");
  m_useBoundary = _node.Read("useBBX", true, false,
      "Use bounding box as obstacle");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_dmLabel =_node.Read("dmLabel", true, "default", "Distance Metric Method");
}

template<class MPTraits>
void
UniformObstacleBasedSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tmargin = " << m_margin << endl;
  _os << "\tuseBoundary = " << m_useBoundary << endl;
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tdmLabel = " << m_dmLabel << endl;
}

template<class MPTraits>
bool
UniformObstacleBasedSampler<MPTraits>::
Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {
  Environment* env = this->GetEnvironment();
  string callee(this->GetNameAndLabel() + "::SampleImpl()");
  ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dmLabel);

  bool generated = false;
  int attempts = 0;
  bool cfg1Free;
  double margin = m_margin;
  if(margin == 0)
    margin = env->GetRobot(_cfg.GetRobotIndex())->GetMaxAxisRange();

  vector<pair<double, double> > origBoundary;
  for(size_t i = 0; i < 3; i++)
    origBoundary.push_back(_boundary->GetRange(i));

  env->ResetBoundary(margin, _cfg.GetRobotIndex());
  shared_ptr<Boundary> boundaryNew = env->GetBoundary();

  attempts++;
  //Generate first cfg
  CfgType& cfg1 = _cfg;

  cfg1Free = (vc->IsValid(cfg1, callee)) && (!vc->IsInsideObstacle(cfg1));

  CfgType cfg2;
  CfgType incr;

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

  CfgType inter;
  CfgType tick = cfg1;
  int nTicks;
  double positionRes = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  bool tempFree = cfg1Free;
  bool tickFree;
  CfgType temp = cfg1;

  inter.FindIncrement(cfg1, cfg2, &nTicks, positionRes, orientationRes);
  env->GetBoundary()->ResetBoundary(origBoundary, 0);
  for(int i = 1; i < nTicks; i++) {
    tick += inter;
    tickFree = (vc->IsValid(tick, callee)) && (!vc->IsInsideObstacle(tick));
    if(m_useBoundary)
      tickFree = tickFree && env->InBounds(tick, _boundary);

    if(tempFree == tickFree) {
      tempFree = tickFree;
      temp = tick;
    }
    else {	//tempFree != tickFree
      generated = true;
      if(tempFree && env->InBounds(temp, _boundary)) {
        _result.push_back(temp);
        tempFree = tickFree;
        temp = tick;
      }
      else if(tickFree && env->InBounds(tick, _boundary)) { //tickFree
        _result.push_back(tick);
        tempFree = tickFree;
        temp = tick;
      }
    }
  }
  return generated;
}

#endif

