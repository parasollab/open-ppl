#ifndef GAUSSIAN_SAMPLER_H_
#define GAUSSIAN_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class GaussianSampler : public SamplerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    GaussianSampler(string _vcLabel = "", string _dmLabel = "",
        double _d = 0.5, bool _useBoundary = false);

    GaussianSampler(MPProblemType* _problem, XMLNode& _node);

    void ParseXML(XMLNode& _node);

    virtual void Print(ostream& _os) const;

    virtual bool Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision);

  private:
    double m_d;  //Gaussian d-value obtained from distribution
    bool m_useBoundary;  //Use Bbox as an obstacle?
    string m_vcLabel, m_dmLabel;
};

template<class MPTraits>
GaussianSampler<MPTraits>::
GaussianSampler(string _vcLabel, string _dmLabel,
    double _d, bool _useBoundary) :
  m_d(_d), m_useBoundary(_useBoundary),
  m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
    this->SetName("GaussianSampler");
  }

template<class MPTraits>
GaussianSampler<MPTraits>::
GaussianSampler(MPProblemType* _problem, XMLNode& _node)
  : SamplerMethod<MPTraits>(_problem, _node) {
    this->SetName("GaussianSampler");
    ParseXML(_node);
  }

template<class MPTraits>
void
GaussianSampler<MPTraits>::
ParseXML(XMLNode& _node) {
  m_d = _node.Read("d", true, 0.0, 0.0, MAX_DBL, "Gaussian D value");
  m_useBoundary = _node.Read("useBBX", true, false,
      "Use bounding box as obstacle");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_dmLabel =_node.Read("dmLabel", true, "default", "Distance Metric Method");
}

//Display values of class variables at calling time
template<class MPTraits>
void
GaussianSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\td = " << m_d << endl;
  _os << "\tuseBoundary = " << m_useBoundary << endl;
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tdmLabel = " << m_dmLabel << endl;
}

template<class MPTraits>
bool
GaussianSampler<MPTraits>::
Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  string callee(this->GetNameAndLabel() + "::SampleImpl()");
  Environment* env = this->GetEnvironment();
  ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dmLabel);

  if(this->m_debug)
    VDClearAll();

  //Start configuration, which was passed in.
  CfgType& cfg1 = _cfg;

  //First define validity of first CFG. If useBBX, require it to be in BBX
  //and only check validity with obstacles. Otherwise have both conditions.
  bool cfg1Free;
  if(!m_useBoundary) {
    if(!env->InBounds(cfg1, _boundary)) {
      if(this->m_debug){
        VDAddTempCfg(cfg1, false);
        VDComment("GaussianSampler::Attempt out of bounds.");
      }
      return false;
    }
    //We are in the box
    cfg1Free = vc->IsValid(cfg1, callee);
  }
  else {
    cfg1Free = env->InBounds(cfg1, _boundary) &&
      vc->IsValid(cfg1, callee);
  }

  if(this->m_debug) {
    cout << "cfg1::" << cfg1 << endl;
    cout << "cfg1Free::" << cfg1Free << endl;
    VDAddTempCfg(cfg1, cfg1Free);
    if(cfg1Free)
      VDComment("GaussianSampler::Cfg1 is valid.");
    else
      VDComment("GaussianSamper::Cfg1 is invalid.");
  }

  //Determine cfg2 attributes with similar consdirations/method to cfg1
  CfgType cfg2;
  bool cfg2Free;
  CfgType incr;
  if(!m_useBoundary) {
    do {
      incr.GetRandomRay(fabs(GaussianDistribution(fabs(m_d), fabs(m_d))), dm);
      cfg2 = cfg1 + incr;
    } while(!env->InBounds(cfg2, _boundary));

    cfg2Free = vc->IsValid(cfg2, callee);
  }
  else {
    incr.GetRandomRay(fabs(GaussianDistribution(fabs(m_d), fabs(m_d))), dm);
    cfg2 = cfg1 + incr;

    cfg2Free = env->InBounds(cfg2, _boundary) &&
      vc->IsValid(cfg2, callee);
  }

  if(this->m_debug) {
    cout << "incr::" << incr << endl;
    cout << "cfg2::" << cfg2 << endl;
    VDAddTempRay(incr);
    cout << "cfg2Free::" << cfg2Free << endl;
    VDAddTempCfg(cfg2, cfg2Free);
    if(cfg2Free)
      VDComment("GaussianSampler::Cfg2 is valid.");
    else
      VDComment("GaussianSampler::Cfg2 is invalid.");
  }

  //If validities are different, retain the free node
  if(cfg1Free != cfg2Free) {
    CfgType gen = cfg1Free ? cfg1 : cfg2;
    CfgType col = cfg1Free ? cfg2 : cfg1;
    _result.push_back(gen);
    _collision.push_back(col);

    if(this->m_debug) {
      cout << "Generated::" << gen << endl; //May be repetitive..
      VDComment("GaussianSampling::Successful");
    }

    return true;
  }
  return false;
}

#endif

