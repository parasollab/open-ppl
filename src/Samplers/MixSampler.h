// Samples by "snapping" random configurations to lattice points in a grid

#ifndef MIX_SAMPLER_H_
#define MIX_SAMPLER_H_

#include "SamplerMethod.h"

template<class MPTraits>
class MixSampler : public SamplerMethod<MPTraits> {

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    MixSampler();
    MixSampler(MPProblemType* _problem, XMLNodeReader& _node);

    void ParseXML(XMLNodeReader& _node);

    virtual void Print(ostream& _os) const;

  protected:
    // Attempts to sample, returns true if successful
    virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb,
          StatClass& _stats, CfgType& _cfgIn,
          vector<CfgType>& _cfgOut, vector<CfgType>& _cfgCol);

    vector<pair <string, double> > samplers; // Stores method label with cumulative probability
};

template<class MPTraits>
MixSampler<MPTraits>::
MixSampler() {
  this->SetName("MixSampler");
}

template<class MPTraits>
MixSampler<MPTraits>::
MixSampler(MPProblemType* _problem, XMLNodeReader& _node) :
  SamplerMethod<MPTraits>(_problem, _node) {
    this->SetName("MixSampler");
    ParseXML(_node);
  }

template<class MPTraits>
void
MixSampler<MPTraits>::
ParseXML(XMLNodeReader& _node) {
  // Read samplers
  double totalP = 0.0;
  for(XMLNodeReader::childiterator citr = _node.children_begin();
      citr != _node.children_end(); citr++) {
    if(citr->getName() == "Sampler") {
      string method = citr->stringXMLParameter("label", true,
          "Default", "Sampling method");
      double p = citr->numberXMLParameter("p", true, 0.0,
          0.0, 1.0, "Probability");
      if(!p)
        cerr << "Warning: MixSampler sampler method " << method <<
          " has 0 probability." << endl;
      totalP += p;
      citr->warnUnrequestedAttributes();
      samplers.push_back(make_pair(method, totalP));
    }
    else
      citr->warnUnknownNode();
  }
  _node.warnUnrequestedAttributes();

  if(samplers.empty())
    throw ParseException(WHERE, "Error: in MixSampler, you must specify at least one sampler.");
  if(totalP > 1.000001)
    throw ParseException(WHERE, "Error: MixSampler total probability greater than 1.");
  if(totalP < 0.999999) {
    cerr << "Warning: MixSampler total probability less than 1. Adding extra probability to last sampler." << endl;
    samplers[samplers.size()-1].second = 1.0;
  }
}

template<class MPTraits>
void
MixSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  for(vector<pair <string, double> >::const_iterator it = samplers.begin();
      it != samplers.end(); it++)
    cout << "\tSampler = " << it->first <<
      ", cumulative probability = " << it->second << endl;
}

// Attempts to sample, returns true if successful
template<class MPTraits>
bool
MixSampler<MPTraits>::
Sampler(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats,
    CfgType& _cfgIn, vector<CfgType>& _cfgOut, vector<CfgType>& _cfgCol) {
  double rand = DRand();
  for(vector<pair <string, double> >::iterator it = samplers.begin();
      it != samplers.end(); it++) {
    if(rand < it->second) {
      return this->GetMPProblem()->GetSampler(it->first)->
        Sampler(_env, _bb, _stats, _cfgIn, _cfgOut, _cfgCol);
    }
  }
  return false;
}

#endif
