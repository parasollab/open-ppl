#ifndef BRIDGETESTSAMPLER_H_
#define BRIDGETESTSAMPLER_H_

#include "SamplerMethod.h"

template <class MPTraits>
class BridgeTestSampler : public SamplerMethod<MPTraits> {
  private:
    double m_d;  //Gaussian d-value obtained from distribution
    bool m_useBoundary;   //use Bbox as obstacle?
    string m_vcLabel, m_dmLabel;

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    ////////////////////
    //Constructors
    ////////////////////

    BridgeTestSampler(Environment* _env = NULL, string _vcLabel = "", string _dmLabel = "",
        double _d = 0.5, bool _useBoundary = false)
      : m_d(_d), m_useBoundary(_useBoundary), m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
        this->SetName("BridgeTestSampler");
      }

    BridgeTestSampler(MPProblemType* _problem, XMLNodeReader& _node) : SamplerMethod<MPTraits>(_problem, _node) {
      this->SetName("BridgeTestSampler");
      ParseXML(_node);
    }

    ~BridgeTestSampler() {}

    /////////////////////
    //Accessors
    /////////////////////

    void
      ParseXML(XMLNodeReader& _node) {
        m_d = _node.numberXMLParameter("d",true, 0.0, 0.0,100.0,"bridge_d");
        m_useBoundary = _node.boolXMLParameter("useBBX", false, true, "Use the Boundary as an Obstacle");
        m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");
        m_dmLabel = _node.stringXMLParameter("dmLabel", true, "default", "Distance Metric Method");

        _node.warnUnrequestedAttributes();
      }

    //Display values of class variables at calling time
    virtual void Print(ostream& _out) const {
        SamplerMethod<MPTraits>::Print(_out);
        _out << "\td = " << m_d << endl;
        _out << "\tuseBoundary = " << m_useBoundary << endl;
        _out << "\tvcLabel = " << m_vcLabel << endl;
        _out << "\tdmLabel = " << m_dmLabel << endl;
      }

    virtual bool
      Sampler(Environment* _env, shared_ptr<Boundary> _bb,
          StatClass& _stats, CfgType& _cfgIn, vector<CfgType>& _cfgOut,
          vector<CfgType>& _cfgCol){

        string callee(this->GetNameAndLabel() + "::SampleImpl()");
        ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
        DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
        CfgType blankCfg;
        bool generated = false;

        if(this->m_debug)
          VDClearAll();

        _stats.IncNodesAttempted(this->GetNameAndLabel());

        ///Acquire first Cfg.
        CfgType tmp = _cfgIn;
        if (tmp == blankCfg)
          tmp.GetRandomCfg(_env,_bb);
        if(this->m_debug)
          cout << "tmp::" << tmp << endl;

        //If using Bbox as obstacle
        if(m_useBoundary) {
          //If tmp is valid configuration extend rays in opposite directions
          //at length Gaussian d/2
          if(_env->InBounds(tmp, _bb) &&
              vc->IsValid(tmp, callee)) {
            CfgType mid = tmp, incr, cfg1;
            incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d))/2, _env, dm);
            cfg1 = mid - incr;
            if(this->m_debug)
              cout << "cfg1::" << cfg1 << endl;

            //If cfg1 is invalid (including Bbox) after adjustment, create cfg2
            if(!_env->InBounds(cfg1, _bb) ||
                !vc->IsValid(cfg1, callee)) {
              CfgType cfg2 = mid + incr;
              if(this->m_debug)
                cout << "cfg2::" << cfg2 << endl;

              //If cfg2 also invalid, node generation successful
              if(!_env->InBounds(cfg2, _bb) ||
                  !vc->IsValid(cfg2, callee)) {
                _stats.IncNodesGenerated(this->GetNameAndLabel());
                generated = true;
                if(this->m_debug)
                  cout << "Generated::" << tmp << endl;
                _cfgOut.push_back(tmp);
                _cfgCol.push_back(tmp);
              }
            }
          }
          //If tmp not in Bbox and tmp invalid, already have good cfg1 and
          //extend random ray at length Gaussian D
          else {
            CfgType cfg1 = tmp, incr, cfg2;
            incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d)), _env, dm);
            cfg2 = cfg1 + incr;
            if(this->m_debug){
              cout << "cfg1::" << cfg1 << endl;
              cout << "cfg2::" << cfg2 << endl;
            }
            //If both cfg1 and cfg2 invalid, create mid and generate node
            //successfully if mid is valid
            if(!_env->InBounds(cfg2, _bb) ||
                !vc->IsValid(cfg2, callee)) {
              CfgType mid;
              mid.WeightedSum(cfg1, cfg2, 0.5);
              if(_env->InBounds(mid, _bb) && (vc->IsValid(mid, callee))) {
                _stats.IncNodesGenerated(this->GetNameAndLabel());
                generated = true;
                if(this->m_debug)
                  cout << "Generated::" << mid << endl;
                _cfgOut.push_back(mid);
                _cfgCol.push_back(tmp);
              }
            }
          }

        }
        //If not using Bbox as obstacle
        else {
          //If tmp is valid configuration extend rays in opposite directions
          //at length Gaussian d/2
          if(vc->IsValid(tmp, callee)) {
            CfgType mid = tmp, incr, cfg1;
            incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d))/2, _env, dm);
            cfg1 = mid - incr;
            if(this->m_debug)
              cout << "cfg1::" << cfg1 << endl;

            //If invalid cfg1, create cfg2. Want both invalid.
            if(!vc->IsValid(cfg1, callee)) {
              CfgType cfg2 = mid + incr;
              if(this->m_debug)
                cout << "cfg2::" << cfg2 << endl;

              //If both cfg1 and cfg2 invalid; valid tmp is successfully generated
              //midpoint node
              if(!vc->IsValid(cfg2, callee)) {
                _stats.IncNodesGenerated(this->GetNameAndLabel());
                generated = true;
                if(this->m_debug)
                  cout << "Generated::" << tmp << endl;
                _cfgOut.push_back(tmp);
                _cfgCol.push_back(tmp);
              }
            }
          }
          //If tmp not in Bbox and tmp invalid, already have good cfg1 and
          //extend random ray at length Gaussian D
          else {
            CfgType cfg1 = tmp, incr, cfg2;
            incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d)), _env, dm);
            cfg2 = cfg1 + incr;
            if(this->m_debug){
              cout << "cfg1::" << cfg1 << endl;
              cout << "cfg2::" << cfg2 << endl;
            }

            //If both cfg1 and cfg2 invalid, get midpoint
            if(!vc->IsValid(cfg2, callee)) {
              CfgType mid;
              mid.WeightedSum(cfg1, cfg2, 0.5);

              //If valid midpoint, successful node is generated
              if(vc->IsValid(mid, callee)) {
                _stats.IncNodesGenerated(this->GetNameAndLabel());
                generated = true;
                if(this->m_debug)
                  cout << "Generated::" << mid << endl;
                _cfgOut.push_back(mid);
                _cfgCol.push_back(tmp);
              }
            }
          }
        }
        return generated;
      }
};

#endif

