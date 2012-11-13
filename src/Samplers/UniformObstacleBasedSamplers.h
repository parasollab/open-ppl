#ifndef UNIFORMOBSTACLEBASEDSAMPLER_H_
#define UNIFORMOBSTACLEBASEDSAMPLER_H_

#include "SamplerMethod.h"


template <typename CFG>
class UniformObstacleBasedSampler : public SamplerMethod<CFG>
{
  private:
    double m_margin;
    bool m_useBoundary;  
    string m_vcLabel, m_dmLabel;

  public:

    UniformObstacleBasedSampler(Environment* _env = NULL, string _vcLabel = "", string _dmLabel = "", double _margin = 0, bool _useBoundary = false)
      : m_margin(_margin), m_useBoundary(_useBoundary), m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
        this->SetName("UniformObstacleBasedSampler");
        if(m_margin == 0){
          if(_env != NULL)
            m_margin = (_env->GetMultiBody(_env->GetRobotIndex()))->GetMaxAxisRange();
        }

        if(this->m_debug)
          PrintOptions(cout);
      }

    UniformObstacleBasedSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("UniformObstacleBasedSampler");
      ParseXML(_node);

      if(this->m_debug)
        PrintOptions(cout);
    }

    ~UniformObstacleBasedSampler() {}

    void ParseXML(XMLNodeReader& _node) {
      m_margin = _node.numberXMLParameter("d", true, 0.0, 0.0, MAX_DBL, "set the bounding box whose margin is d away from obstacles");
      m_useBoundary = _node.boolXMLParameter("useBBX", true, false, "Use bounding box as obstacle");
      m_vcLabel = _node.stringXMLParameter("vcMethod", true, "", "Validity Test Method");
      m_dmLabel =_node.stringXMLParameter("dmMethod", true, "default", "Distance Metric Method");

      _node.warnUnrequestedAttributes();
    }

    virtual void PrintOptions(ostream& _out) const {
      SamplerMethod<CFG>::PrintOptions(_out);
      _out << "\tmargin = " << m_margin << endl;
      _out << "\tuseBoundary = " << m_useBoundary << endl;
      _out << "\tvcLabel = " << m_vcLabel << endl;
      _out << "\tdmLabel = " << m_dmLabel << endl;
    }

    virtual bool Sampler(Environment* env, shared_ptr<Boundary> _bb, StatClass& Stat, CFG& cfg_in, vector<CFG>& cfg_out, vector<CFG>& _cfgCol)
    {
      string callee(this->GetName());
      callee += "::sampler()";
      CDInfo cdInfo;
      ValidityChecker* vc = this->GetMPProblem()->GetValidityChecker();
      shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetDistanceMetric()->GetMethod(m_dmLabel);

      bool generated = false;
      int attempts = 0;
      bool cfg1_free;

      env->ResetBoundingBox(m_margin);
      shared_ptr<Boundary> bbox_new = env->GetBoundary();

      Stat.IncNodesAttempted(this->GetNameAndLabel());
      attempts++;
      //Generate first cfg
      CFG cfg1 = cfg_in;
      if(cfg1 == CFG())
        cfg1.GetRandomCfg(env, bbox_new);

      cfg1_free = (vc->GetMethod(m_vcLabel)->IsValid(cfg1, env, Stat, cdInfo, &callee)) && (!vc->GetMethod(m_vcLabel)->IsInsideObstacle(cfg1, env, cdInfo));

      CFG cfg2;
      CFG incr;
      CFG tmp;
      double dist, r;

      incr.GetRandomRay(m_margin, env, dm);
      cfg2.add(cfg1, incr);

      //scale the distance between c1 and c2
      Vector3D c1, c2, dir;
      c1[0] = cfg1.GetSingleParam(0);
      c1[1] = cfg1.GetSingleParam(1);
      c1[2] = cfg1.GetSingleParam(2);
      c2[0] = cfg2.GetSingleParam(0);
      c2[1] = cfg2.GetSingleParam(1);
      c2[2] = cfg2.GetSingleParam(2);
      dir = c2 - c1;
      dist = sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
      r = m_margin/dist;
      tmp.multiply(incr, r);
      cfg2.add(cfg1, tmp);
      //bool cfg2_free = (vc->GetMethod(m_vcLabel)->IsValid(cfg2, env, Stat, cdInfo, &callee)) && (!vc->GetMethod(m_vcLabel)->IsInsideObstacle(cfg2, env, cdInfo));

      CFG inter;
      CFG tick = cfg1;
      int n_ticks;
      double positionRes = env->GetPositionRes(); 
      double orientationRes = env->GetOrientationRes();
      bool temp_free = cfg1_free;
      bool tick_free;
      CFG temp = cfg1;

      inter.FindIncrement(cfg1, cfg2, &n_ticks, positionRes, orientationRes);
      for(int i=1; i<n_ticks; i++)
      {
        tick.Increment(inter);
        tick_free = (vc->GetMethod(m_vcLabel)->IsValid(tick, env, Stat, cdInfo, &callee)) && (!vc->GetMethod(m_vcLabel)->IsInsideObstacle(tick, env, cdInfo));
        env->SetBoundary(_bb);
        if(m_useBoundary)
        {
          tick_free = tick_free && (tick.InBoundary(env, _bb));
        }
        if(temp_free == tick_free)
        {
          temp_free = tick_free;
          temp = tick;
        }
        else	//temp_free != tick_free
        {
          Stat.IncNodesGenerated(this->GetNameAndLabel());
          generated = true;
          if(temp_free && (temp.InBoundary(env, _bb)))
          {
            cfg_out.push_back(temp);
            temp_free = tick_free;
            temp = tick;
          }
          else if(tick_free && (tick.InBoundary(env, _bb)))       //tick_free
          {
            cfg_out.push_back(tick);
            temp_free = tick_free;
            temp = tick;
          }
        }
      }
      return generated;
    }   
};

#endif

