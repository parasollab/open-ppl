#ifndef MEDIALAXISSAMPLERS_H_
#define MEDIALAXISSAMPLERS_H_

#include "SamplerMethod.h"
#include "Utilities/MedialAxisUtilities.h"

template<class MPTraits>
class MedialAxisSampler : public SamplerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    MedialAxisSampler(const MedialAxisUtility<MPTraits>& _medialAxisUtility = MedialAxisUtility<MPTraits>()):
      m_medialAxisUtility(_medialAxisUtility) {
        this->SetName("MedialAxisSampler");
      }

    MedialAxisSampler(MPProblemType* _problem, XMLNodeReader& _node):
      SamplerMethod<MPTraits>(_problem, _node), m_medialAxisUtility(_problem, _node){
        this->SetName("MedialAxisSampler");
        _node.warnUnrequestedAttributes();
      }

    ~MedialAxisSampler() {}

    virtual void Print(ostream& _os) const {
      SamplerMethod<MPTraits>::Print(_os);
      m_medialAxisUtility.Print(_os);
    }

    virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb,
        StatClass& _stats, CfgType& _cfgIn, vector<CfgType>& _cfgOut, vector<CfgType>& _cfgCol) {

      string call = "MedialAxisSampler::sampler()";
      bool generated = false;

      _stats.IncNodesAttempted(this->GetNameAndLabel());

      // If just a new cfg, get a random CfgType
      CfgType tmpCfg = _cfgIn;
      if(tmpCfg == CfgType())
        tmpCfg.GetRandomCfg(_env,_bb);

      // If pushed properly and the new CfgType is valid, increment generated
      if(m_medialAxisUtility.PushToMedialAxis(tmpCfg, _bb)){
        _stats.IncNodesGenerated(this->GetNameAndLabel());
        generated = true;
        _cfgOut.push_back(tmpCfg);
      }
      return generated;
    }

  private:
    MedialAxisUtility<MPTraits> m_medialAxisUtility;
};

#endif

