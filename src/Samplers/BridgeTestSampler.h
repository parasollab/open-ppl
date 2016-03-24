#ifndef BRIDGE_TEST_SAMPLER_H_
#define BRIDGE_TEST_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class BridgeTestSampler : public SamplerMethod<MPTraits> {

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    BridgeTestSampler(string _vcLabel = "", string _dmLabel = "",
        double _d = 0.5, bool _useBoundary = false);

    BridgeTestSampler(MPProblemType* _problem, XMLNode& _node);

    void ParseXML(XMLNode& _node);

    virtual void Print(ostream& _os) const;

    virtual bool Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision);

  private:
    double m_d;  //Gaussian d-value obtained from distribution
    bool m_useBoundary;   //use Bbox as obstacle?
    string m_vcLabel, m_dmLabel;
};

template<class MPTraits>
BridgeTestSampler<MPTraits>::
BridgeTestSampler(string _vcLabel, string _dmLabel,
    double _d, bool _useBoundary) :
  m_d(_d), m_useBoundary(_useBoundary),
  m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
    this->SetName("BridgeTestSampler");
  }

template<class MPTraits>
BridgeTestSampler<MPTraits>::
BridgeTestSampler(MPProblemType* _problem, XMLNode& _node) :
  SamplerMethod<MPTraits>(_problem, _node) {
    this->SetName("BridgeTestSampler");
    ParseXML(_node);
  }

template<class MPTraits>
void
BridgeTestSampler<MPTraits>::
ParseXML(XMLNode& _node) {
  m_d = _node.Read("d", true, 0.0, 0.0, 100.0, "bridge_d");
  m_useBoundary = _node.Read("useBBX", false, true,
      "Use the Boundary as an Obstacle");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_dmLabel = _node.Read("dmLabel", true, "default", "Distance Metric Method");
}

//Display values of class variables at calling time
template<class MPTraits>
void
BridgeTestSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\td = " << m_d << endl;
  _os << "\tuseBoundary = " << m_useBoundary << endl;
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tdmLabel = " << m_dmLabel << endl;
}

template<class MPTraits>
bool
BridgeTestSampler<MPTraits>::
Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  string callee(this->GetNameAndLabel() + "::SampleImpl()");
  Environment* env = this->GetEnvironment();
  ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dmLabel);
  bool generated = false;

  if(this->m_debug)
    VDClearAll();

  if(this->m_debug)
    cout << "cfg::" << _cfg << endl;

  //If using Bbox as obstacle
  if(m_useBoundary) {
    //If _cfg is valid configuration extend rays in opposite directions
    //at length Gaussian d/2
    if(env->InBounds(_cfg, _boundary) &&
        vc->IsValid(_cfg, callee)) {
      CfgType mid = _cfg, incr, cfg1;
      incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d))/2, dm);
      cfg1 = mid - incr;
      if(this->m_debug)
        cout << "cfg1::" << cfg1 << endl;

      //If cfg1 is invalid (including Bbox) after adjustment, create cfg2
      if(!env->InBounds(cfg1, _boundary) ||
          !vc->IsValid(cfg1, callee)) {
        CfgType cfg2 = mid + incr;
        if(this->m_debug)
          cout << "cfg2::" << cfg2 << endl;

        //If cfg2 also invalid, node generation successful
        if(!env->InBounds(cfg2, _boundary) ||
            !vc->IsValid(cfg2, callee)) {
          generated = true;
          if(this->m_debug)
            cout << "Generated::" << _cfg << endl;
          _result.push_back(_cfg);
          _collision.push_back(_cfg);
        }
      }
    }
    //If _cfg not in Bbox and invalid, already have good cfg1 and
    //extend random ray at length Gaussian D
    else {
      CfgType cfg1 = _cfg, incr, cfg2;
      incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d)), dm);
      cfg2 = cfg1 + incr;
      if(this->m_debug) {
        cout << "cfg1::" << cfg1 << endl;
        cout << "cfg2::" << cfg2 << endl;
      }
      //If both cfg1 and cfg2 invalid, create mid and generate node
      //successfully if mid is valid
      if(!env->InBounds(cfg2, _boundary) ||
          !vc->IsValid(cfg2, callee)) {
        CfgType mid;
        mid.WeightedSum(cfg1, cfg2, 0.5);
        if(env->InBounds(mid, _boundary) && (vc->IsValid(mid, callee))) {
          generated = true;
          if(this->m_debug)
            cout << "Generated::" << mid << endl;
          _result.push_back(mid);
          _collision.push_back(_cfg);
        }
      }
    }

  }
  //If not using Bbox as obstacle
  else {
    //If _cfg is valid configuration extend rays in opposite directions
    //at length Gaussian d/2
    if(vc->IsValid(_cfg, callee)) {
      CfgType mid = _cfg, incr, cfg1;
      incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d))/2, dm);
      cfg1 = mid - incr;
      if(this->m_debug)
        cout << "cfg1::" << cfg1 << endl;

      //If invalid cfg1, create cfg2. Want both invalid.
      if(!vc->IsValid(cfg1, callee)) {
        CfgType cfg2 = mid + incr;
        if(this->m_debug)
          cout << "cfg2::" << cfg2 << endl;

        //If both cfg1 and cfg2 invalid; valid _cfg is successfully generated
        //midpoint node
        if(!vc->IsValid(cfg2, callee)) {
          generated = true;
          if(this->m_debug)
            cout << "Generated::" << _cfg << endl;
          _result.push_back(_cfg);
          _collision.push_back(_cfg);
        }
      }
    }
    //If _cfg not in Bbox and invalid, already have good cfg1 and
    //extend random ray at length Gaussian D
    else {
      CfgType cfg1 = _cfg, incr, cfg2;
      incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d)), dm);
      cfg2 = cfg1 + incr;
      if(this->m_debug) {
        cout << "cfg1::" << cfg1 << endl;
        cout << "cfg2::" << cfg2 << endl;
      }

      //If both cfg1 and cfg2 invalid, get midpoint
      if(!vc->IsValid(cfg2, callee)) {
        CfgType mid;
        mid.WeightedSum(cfg1, cfg2, 0.5);

        //If valid midpoint, successful node is generated
        if(vc->IsValid(mid, callee)) {
          generated = true;
          if(this->m_debug)
            cout << "Generated::" << mid << endl;
          _result.push_back(mid);
          _collision.push_back(_cfg);
        }
      }
    }
  }
  return generated;
}

#endif

