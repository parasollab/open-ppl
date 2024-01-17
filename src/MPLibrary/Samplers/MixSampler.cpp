#include "MixSampler.h"

#include "MPLibrary/MPLibrary.h"


MixSampler::
MixSampler() {
  this->SetName("MixSampler");
}


MixSampler::
MixSampler(XMLNode& _node) :
  SamplerMethod(_node) {
    this->SetName("MixSampler");
    ParseXML(_node);
}


void
MixSampler::
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


void
MixSampler::
Print(ostream& _os) const {
  SamplerMethod::Print(_os);
  for(auto&  sampler : m_samplers)
    cout << "\tSampler = " << sampler.first
      << ", cumulative probability = " << sampler.second << endl;
}


// Attempts to sample, returns true if successful
bool
MixSampler::
Sampler(Cfg& _cfg, const Boundary* const _boundary,
    std::vector<Cfg>& _result, std::vector<Cfg>& _collision) {
  double rand = DRand();
  int samplerIndex = 0;
  for(auto&  sampler : m_samplers){
    if(rand < sampler.second){
      m_samplerLabels = make_pair(this->GetMPLibrary()->GetSampler(sampler.first)->GetNameAndLabel(), 
                                  samplerIndex);
      return this->GetMPLibrary()->GetSampler(sampler.first)->
        Sampler(_cfg, _boundary, _result, _collision);
    }

    samplerIndex = samplerIndex + 1;
  }
  return false;
}
