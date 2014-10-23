#ifndef GAUSSIAN_SAMPLER_H_
#define GAUSSIAN_SAMPLER_H_

#include "SamplerMethod.h"

template<class MPTraits>
class GaussianSampler : public SamplerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    GaussianSampler(Environment* _env = NULL, string _vcLabel = "",
        string _dmLabel = "", double _d = 0.5, bool _useBoundary = false);

    GaussianSampler(MPProblemType* _problem, XMLNodeReader& _node);

    void ParseXML(XMLNodeReader& _node);

    virtual void Print(ostream& _out) const;

    virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb,
        StatClass& _stats, CfgType& _cfgIn, vector<CfgType>& _cfgOut,
        vector<CfgType>& _cfgCol);

  private:
    double m_d;  //Gaussian d-value obtained from distribution
    bool m_useBoundary;  //Use Bbox as an obstacle?
    string m_vcLabel, m_dmLabel;
};

template<class MPTraits>
GaussianSampler<MPTraits>::
GaussianSampler(Environment* _env, string _vcLabel, string _dmLabel,
    double _d, bool _useBoundary)
  : m_d(_d), m_useBoundary(_useBoundary), m_vcLabel(_vcLabel),
  m_dmLabel(_dmLabel) {
    this->SetName("GaussianSampler");
  }

template<class MPTraits>
GaussianSampler<MPTraits>::
GaussianSampler(MPProblemType* _problem, XMLNodeReader& _node)
  : SamplerMethod<MPTraits>(_problem, _node) {
    this->SetName("GaussianSampler");
    ParseXML(_node);
  }

template<class MPTraits>
void
GaussianSampler<MPTraits>::
ParseXML(XMLNodeReader& _node) {
  m_d = _node.numberXMLParameter("d", true, 0.0, 0.0, MAX_DBL, "Gaussian D value");
  m_useBoundary = _node.boolXMLParameter("useBBX", true, false,
      "Use bounding box as obstacle");
  m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");
  m_dmLabel =_node.stringXMLParameter("dmLabel", true, "default",
      "Distance Metric Method");

  _node.warnUnrequestedAttributes();
}

//Display values of class variables at calling time
template<class MPTraits>
void
GaussianSampler<MPTraits>::
Print(ostream& _out) const {
  SamplerMethod<MPTraits>::Print(_out);
  _out << "\td = " << m_d << endl;
  _out << "\tuseBoundary = " << m_useBoundary << endl;
  _out << "\tvcLabel = " << m_vcLabel << endl;
  _out << "\tdmLabel = " << m_dmLabel << endl;
}

template<class MPTraits>
bool
GaussianSampler<MPTraits>::
Sampler(Environment* _env, shared_ptr<Boundary> _bb,
    StatClass& _stats, CfgType& _cfgIn, vector<CfgType>& _cfgOut,
    vector<CfgType>& _cfgCol){

  string callee(this->GetNameAndLabel() + "::SampleImpl()");
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);

  if(this->m_debug)
    VDClearAll();

  //Start configuration, which was passed in. Use random as a default.
  CfgType cfg1 = _cfgIn;
  if(cfg1 == CfgType())
    cfg1.GetRandomCfg(_env,_bb);

  //First define validity of first CFG. If useBBX, require it to be in BBX
  //and only check validity with obstacles. Otherwise have both conditions.
  bool cfg1Free;
  if(!m_useBoundary) {
    if(!_env->InBounds(cfg1, _bb)){
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
    cfg1Free = _env->InBounds(cfg1, _bb) &&
      vc->IsValid(cfg1, callee);
  }

  if(this->m_debug){
    cout << "cfg1::" << cfg1 << endl;
    cout << "cfg1Free::" << cfg1Free << endl;
    VDAddTempCfg(cfg1, cfg1Free);
    if(cfg1Free)
      VDComment("GaussianSampler::Cfg1 is valid.");
    else
      VDComment("GaussianSamper::Cfg1 is invalid.");
  }

  //Attempt to generate node (cfg2) while not at max attempts
  _stats.IncNodesAttempted(this->GetNameAndLabel());

  //Determine cfg2 attributes with similar consdirations/method to cfg1
  CfgType cfg2;
  bool cfg2Free;
  CfgType incr;
  if(!m_useBoundary) {
    do {
      incr.GetRandomRay(fabs(GaussianDistribution(fabs(m_d), fabs(m_d))), _env, dm);
      cfg2 = cfg1 + incr;
    } while(!_env->InBounds(cfg2, _bb));

    cfg2Free = vc->IsValid(cfg2, callee);
  }
  else {
    incr.GetRandomRay(fabs(GaussianDistribution(fabs(m_d), fabs(m_d))), _env, dm);
    cfg2 = cfg1 + incr;

    cfg2Free = _env->InBounds(cfg2, _bb) &&
      vc->IsValid(cfg2, callee);
  }

  if(this->m_debug){
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
    _stats.IncNodesGenerated(this->GetNameAndLabel());

    CfgType gen = cfg1Free ? cfg1 : cfg2;
    CfgType col = cfg1Free ? cfg2 : cfg1;
    _cfgOut.push_back(gen);
    _cfgCol.push_back(col);

    if(this->m_debug){
      cout << "Generated::" << gen << endl; //May be repetitive..
      VDComment("GaussianSampling::Successful");
    }

    return true;
  }
  return false;
}

#endif

