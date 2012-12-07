// Samples by "snapping" random configurations to lattice points in a grid

#ifndef MIXSAMPLER_H_
#define MIXSAMPLER_H_

#include "SamplerMethod.h"

template<class MPTraits>
class MixSampler : public SamplerMethod<MPTraits> {

  protected:
    vector<pair<string, double> > samplers; // Stores method label with cumulative probability

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    MixSampler() { this->SetName("MixSampler"); }
    MixSampler(MPProblemType* _problem, XMLNodeReader& _node) : SamplerMethod<MPTraits>(_problem, _node) {
      this->SetName("MixSampler");
      ParseXML(_node);
    }

    ~MixSampler() {}

    void ParseXML(XMLNodeReader& _node) {
      // Read samplers
      double totalP = 0.0;
      for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); citr++) {
        if(citr->getName() == "Sampler") {
          string method = citr->stringXMLParameter("label", true, "Default", "Sampling method");
          double p = citr->numberXMLParameter("p", true, 0.0, 0.0, 1.0, "Probability");
          if(!p)
            cerr << "Warning: MixSampler sampler method " << method << " has 0 probability." << endl;
          totalP += p;
          citr->warnUnrequestedAttributes();
          samplers.push_back(make_pair(method, totalP));
        }
        else
          citr->warnUnknownNode();
      }
      _node.warnUnrequestedAttributes();

      if(!samplers.size()) {
        cerr << "Error: in MixSampler, you must specify at least one sampler. Exiting..." << endl;
        exit(1);
      }

      if(totalP > 1.000001) {
        cerr << "Error: MixSampler total probability greater than 1. Exiting..." << endl;
        exit(1);
      }

      if(totalP < 0.999999) {
        cerr << "Warning: MixSampler total probability less than 1. Adding extra probability to last sampler." << endl;
        samplers[samplers.size()-1].second = 1.0;
      }
    }

    virtual void PrintOptions(ostream& _os) const {
      SamplerMethod<MPTraits>::PrintOptions(_os);
      for(vector<pair<string, double> >::const_iterator it = samplers.begin(); it != samplers.end(); it++)
        cout << "\tSampler = " << it->first << ", cumulative probability = " << it->second << endl;
    }

  protected:
    // Attempts to sample, returns true if successful
    virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats, 
        CfgType& _cfgIn, vector<CfgType>& _cfgOut, vector<CfgType>& _cfgCol) {
      
      double rand = DRand();
      for(vector<pair<string, double> >::iterator it = samplers.begin(); it != samplers.end(); it++)
        if(rand < it->second)
          return this->GetMPProblem()->GetSampler(it->first)->Sampler(_env, _bb, _stats, _cfgIn, _cfgOut, _cfgCol);
      return false;
    }
};

#endif

