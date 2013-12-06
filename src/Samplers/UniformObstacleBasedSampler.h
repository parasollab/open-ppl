#ifndef UNIFORMOBSTACLEBASEDSAMPLER_H_
#define UNIFORMOBSTACLEBASEDSAMPLER_H_

#include "SamplerMethod.h"

template<typename MPTraits>
class UniformObstacleBasedSampler : public SamplerMethod<MPTraits> {
  private:
    double m_margin;
    bool m_useBoundary;
    string m_vcLabel, m_dmLabel;

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    UniformObstacleBasedSampler(Environment* _env = NULL, string _vcLabel = "", string _dmLabel = "", double _margin = 0, bool _useBoundary = false)
      : m_margin(_margin), m_useBoundary(_useBoundary), m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
        this->SetName("UniformObstacleBasedSampler");
      }

    UniformObstacleBasedSampler(MPProblemType* _problem, XMLNodeReader& _node) : SamplerMethod<MPTraits>(_problem, _node) {
      this->SetName("UniformObstacleBasedSampler");
      ParseXML(_node);
    }

    ~UniformObstacleBasedSampler() {}

    void ParseXML(XMLNodeReader& _node) {
      m_margin = _node.numberXMLParameter("d", true, 0.0, 0.0, MAX_DBL, "set the bounding box whose margin is d away from obstacles");
      m_useBoundary = _node.boolXMLParameter("useBBX", true, false, "Use bounding box as obstacle");
      m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");
      m_dmLabel =_node.stringXMLParameter("dmLabel", true, "default", "Distance Metric Method");

      _node.warnUnrequestedAttributes();
    }

    virtual void PrintOptions(ostream& _os) const {
      SamplerMethod<MPTraits>::PrintOptions(_os);
      _os << "\tmargin = " << m_margin << endl;
      _os << "\tuseBoundary = " << m_useBoundary << endl;
      _os << "\tvcLabel = " << m_vcLabel << endl;
      _os << "\tdmLabel = " << m_dmLabel << endl;
    }

    virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats, CfgType& _cfgIn, vector<CfgType>& _cfgOut, vector<CfgType>& _cfgCol) {
      string callee(this->GetNameAndLabel() + "::SampleImpl()");
      ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
      DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);

      bool generated = false;
      int attempts = 0;
      bool cfg1Free;
      double margin = m_margin;
      if(margin == 0){
        margin = _env->GetMultiBody(_cfgIn.GetRobotIndex())->GetMaxAxisRange();
      }
      _env->ResetBoundary(margin, _cfgIn.GetRobotIndex());
      shared_ptr<Boundary> bbNew = _env->GetBoundary();

      _stats.IncNodesAttempted(this->GetNameAndLabel());
      attempts++;
      //Generate first cfg
      CfgType cfg1 = _cfgIn;
      if(cfg1 == CfgType())
        cfg1.GetRandomCfg(_env, bbNew);

      cfg1Free = (vc->IsValid(cfg1, callee)) && (!vc->IsInsideObstacle(cfg1));

      CfgType cfg2;
      CfgType incr;
      double dist, r;

      incr.GetRandomRay(margin, _env, dm);
      cfg2 = cfg1 + incr;

      //scale the distance between c1 and c2
      Vector3d c1, c2, dir;
      c1[0] = cfg1[0];
      c1[1] = cfg1[1];
      c1[2] = cfg1[2];
      c2[0] = cfg2[0];
      c2[1] = cfg2[1];
      c2[2] = cfg2[2];
      dir = c2 - c1;
      dist = sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
      r = margin/dist;
      cfg2 = cfg1 + incr*r;

      CfgType inter;
      CfgType tick = cfg1;
      int nTicks;
      double positionRes = _env->GetPositionRes();
      double orientationRes = _env->GetOrientationRes();
      bool tempFree = cfg1Free;
      bool tickFree;
      CfgType temp = cfg1;

      inter.FindIncrement(cfg1, cfg2, &nTicks, positionRes, orientationRes);
      for(int i=1; i<nTicks; i++) {
        tick += inter;
        tickFree = (vc->IsValid(tick, callee)) && (!vc->IsInsideObstacle(tick));
        _env->SetBoundary(_bb);
        if(m_useBoundary)
          tickFree = tickFree && _env->InBounds(tick, _bb);

        if(tempFree == tickFree) {
          tempFree = tickFree;
          temp = tick;
        }
        else {	//tempFree != tickFree
          _stats.IncNodesGenerated(this->GetNameAndLabel());
          generated = true;
          if(tempFree && _env->InBounds(temp, _bb)) {
            _cfgOut.push_back(temp);
            tempFree = tickFree;
            temp = tick;
          }
          else if(tickFree && _env->InBounds(tick, _bb)) {       //tickFree
            _cfgOut.push_back(tick);
            tempFree = tickFree;
            temp = tick;
          }
        }
      }
      return generated;
    }
};

#endif

