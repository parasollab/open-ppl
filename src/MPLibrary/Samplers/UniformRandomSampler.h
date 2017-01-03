#ifndef UNIFORM_RANDOM_SAMPLER_H_
#define UNIFORM_RANDOM_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class UniformRandomSampler : public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    UniformRandomSampler(string _vcLabel = "");
    UniformRandomSampler(XMLNode& _node);
    virtual ~UniformRandomSampler() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}

  protected:

    ///@name Sampler Rule
    ///@{

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    string m_vcLabel;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
UniformRandomSampler<MPTraits>::
UniformRandomSampler(string _vcLabel) : m_vcLabel(_vcLabel) {
  this->SetName("UniformRandomSampler");
}


template <typename MPTraits>
UniformRandomSampler<MPTraits>::
UniformRandomSampler(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {
  this->SetName("UniformRandomSampler");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
UniformRandomSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel << endl;
}

/*------------------------------ Sampler Rule --------------------------------*/

template <typename MPTraits>
bool
UniformRandomSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  string callee(this->GetNameAndLabel() + "::SampleImpl()");
  Environment* env = this->GetEnvironment();
  auto vcm = this->GetValidityChecker(m_vcLabel);

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

/*----------------------------------------------------------------------------*/

#endif
