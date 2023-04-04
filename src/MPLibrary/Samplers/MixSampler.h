#ifndef MIX_SAMPLER_H_
#define MIX_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// TODO
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
// Samples by "snapping" random configurations to lattice points in a grid
template <typename MPTraits>
class MixSampler : virtual public SamplerMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;

    MixSampler();
    MixSampler(XMLNode& _node);

    void ParseXML(XMLNode& _node);

    virtual void Print(ostream& _os) const;

  protected:
    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision);

    vector<pair <string, double> > m_samplers; // Stores method label with cumulative probability

    pair<string, int> m_samplerLabels; // Stores label history and index
};

template <typename MPTraits>
MixSampler<MPTraits>::
MixSampler() {
  this->SetName("MixSampler");
}

template <typename MPTraits>
MixSampler<MPTraits>::
MixSampler(XMLNode& _node) :
  SamplerMethod<MPTraits>(_node) {
    this->SetName("MixSampler");
    ParseXML(_node);
  }

template <typename MPTraits>
void
MixSampler<MPTraits>::
ParseXML(XMLNode& _node) {
  // Read samplers
  double totalP = 0.0;
  for(auto& child : _node) {
    if(child.Name() == "Sampler") {
      string method = child.Read("label", true,
          "Default", "Sampling method");
      double p = child.Read("p", true, 0.0,
          0.0, 1.0, "Probability");
      if(!p)
        cerr << "Warning: MixSampler sampler method " << method
          << " has 0 probability." << endl;
      totalP += p;
      m_samplers.push_back(make_pair(method, totalP));
    }
  }

  if(m_samplers.empty())
    throw ParseException(_node.Where(),
        "You must specify at least one sampler.");
  if(totalP > 1.000001)
    throw ParseException(_node.Where(),
        "Total probability greater than 1.");
  if(totalP < 0.999999) {
    cerr << "Warning: MixSampler total probability less than 1."
      "Adding extra probability to last sampler." << endl;
    m_samplers[m_samplers.size()-1].second = 1.0;
  }
}

template <typename MPTraits>
void
MixSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  for(auto&  sampler : m_samplers)
    cout << "\tSampler = " << sampler.first
      << ", cumulative probability = " << sampler.second << endl;
}

// Attempts to sample, returns true if successful
template <typename MPTraits>
bool
MixSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {
  double rand = DRand();
  int samplerIndex = 0;
  for(auto&  sampler : m_samplers){
    if(rand < sampler.second){
      m_samplerLabels = make_pair(this->GetSampler(sampler.first)->GetNameAndLabel(), samplerIndex);
      return this->GetSampler(sampler.first)->
        Sampler(_cfg, _boundary, _result, _collision);
    }

    samplerIndex = samplerIndex + 1;
  }
  return false;
}

#endif
