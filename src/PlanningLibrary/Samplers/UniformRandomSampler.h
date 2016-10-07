#ifndef UNIFORM_RANDOM_SAMPLER_H_
#define UNIFORM_RANDOM_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class UniformRandomSampler : public SamplerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    UniformRandomSampler(string _vcLabel = "");
    UniformRandomSampler(MPProblemType* _problem, XMLNode& _node);

    virtual void Print(ostream& _os) const;

  protected:
    virtual bool Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision);

  private:
    string m_vcLabel;
};

template<class MPTraits>
UniformRandomSampler<MPTraits>::
UniformRandomSampler(string _vcLabel) : m_vcLabel(_vcLabel) {
  this->SetName("UniformRandomSampler");
}

template<class MPTraits>
UniformRandomSampler<MPTraits>::
UniformRandomSampler(MPProblemType* _problem, XMLNode& _node) :
  SamplerMethod<MPTraits>(_problem, _node) {
    this->SetName("UniformRandomSampler");
    m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  }

template<class MPTraits>
void
UniformRandomSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel << endl;
}

template<class MPTraits>
bool
UniformRandomSampler<MPTraits>::
Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  string callee(this->GetNameAndLabel() + "::SampleImpl()");
  Environment* env = this->GetEnvironment();
  ValidityCheckerPointer vcm = this->GetValidityChecker(m_vcLabel);

  if(this->m_debug)
    VDClearAll();

  //Is configuration within environment boundary?
  bool inBBX = env->InBounds(_cfg);
  if(this->m_debug) {
    cout << "_cfg::" << _cfg << endl;
    cout << "InBoudary::" << inBBX << endl;
  }

  //Good. Now determine validity.
  if(inBBX) {
    bool isValid = vcm->IsValid(_cfg, callee);
    if(this->m_debug) {
      cout << "IsValid::" << isValid << endl;
      VDAddTempCfg(_cfg, isValid);
      if(isValid)
        VDComment("UniformSampling::Cfg valid");
      else
        VDComment("UniformSampling::Cfg invalid");
    }
    //Record valid node and confirm successful generation.
    if(isValid) {
      if(this->m_debug)
        cout << "Generated::" << _cfg << endl;
      _result.push_back(_cfg);
      return true;
    }
    //Otherwise, unsuccessful.
    else {
      _collision.push_back(_cfg);
      return false;
    }
  }
  //Sampled outside of boundary
  else if(this->m_debug) {
    cout << "Attempt outside of boundary" << endl;
    VDAddTempCfg(_cfg, false);
    VDComment("UniformSampling::Cfg outside of boundary");
  }
  return false;
}

#endif

