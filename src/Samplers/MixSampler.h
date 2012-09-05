// Samples by "snapping" random configurations to lattice points in a grid

#ifndef MIXSAMPLER_H_
#define MIXSAMPLER_H_

#include "SamplerMethod.h"
#include <algorithm>

template <typename CFG>
class MixSampler : public SamplerMethod<CFG> {

  protected:
    vector<pair<string, double> > samplers; // Stores method label with cumulative probability

  public:

    MixSampler() { this->SetName("MixSampler"); }
    MixSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("MixSampler");
      ParseXML(_node);
      if(this->m_debug) 
        PrintOptions(cout);
    }

    ~MixSampler() {}

    void ParseXML(XMLNodeReader& _node) {
      // Read samplers
      double totalP = 0.0;
      for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); citr++) {
        if(citr->getName() == "Sampler") {
          string method = citr->stringXMLParameter("method", true, "Default", "Sampling method");
          double p = citr->numberXMLParameter("p", true, 0.0, 0.0, 1.0, "Probability");
          if(!p)
            cout << "Warning: MixSampler sampler method " << method << " has 0 probability." << endl;
          totalP += p;
          citr->warnUnrequestedAttributes();
          samplers.push_back(make_pair(method, totalP));
        }
        else
          citr->warnUnknownNode();
      }
      _node.warnUnrequestedAttributes();

      if(!samplers.size()) {
        cout << "Error: in MixSampler, you must specify at least one sampler. Exiting..." << endl;
        exit(1);
      }

      if(totalP > 1.000001) {
        cout << "Error: MixSampler total probability greater than 1. Exiting..." << endl;
        exit(1);
      }

      if(totalP < 0.999999) {
        cout << "Warning: MixSampler total probability less than 1. Adding extra probability to last sampler." << endl;
        samplers[samplers.size()-1].second = 1.0;
      }
    }

    virtual void PrintOptions(ostream& _os) const {
      SamplerMethod<CFG>::PrintOptions(_os);
      for(size_t i = 0; i < samplers.size(); i++)
        cout << "\tSampler = " << samplers[i].first << ", cumulative probability = " << samplers[i].second << endl;
//      for(typename vector<pair<string, double> >::iterator it = samplers.begin(); it != samplers.end(); it++)
//        cout << "\tSampler = " << it->first << ", p = " << it->second << endl;
    }

    // Attempts to sample, returns true if successful
    virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats, 
        CFG& _cfgIn, vector<CFG>& _cfgOut, vector<CFG>& _cfgCol) {
      
      double rand = DRand();
      for(size_t i = 0; i < samplers.size(); i++)
//      for(typename vector<pair<string, double> >::iterator it = samplers.begin(); it != samplers.end(); it++)
        if(rand < samplers[i].second)
          return this->GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(samplers[i].first)
              ->Sampler(_env, _bb, _stats, _cfgIn, _cfgOut, _cfgCol);
      return false;
    }
};

#endif

