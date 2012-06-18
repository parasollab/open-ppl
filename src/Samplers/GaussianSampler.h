#ifndef GAUSSIANSAMPLER_H_
#define GAUSSIANSAMPLER_H_

#include "SamplerMethod.h"

template <typename CFG>
class GaussianSampler : public SamplerMethod<CFG>
{
  private:
    double m_d;  //Gaussian d-value obtained from distribution 
    bool m_useBBX;  //Use Bbox as an obstacle? 
    string m_vcLabel, m_dmLabel;

  public: 

    //////////////////////
    //Constructors
    //////////////////////

    GaussianSampler(Environment* _env = NULL, string _vcLabel = "", string _dmLabel = "", double _d = 0, bool _useBoundingBox = false)
      : m_d(_d), m_useBBX(_useBoundingBox), m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
        this->SetName("GaussianSampler");
        if(m_d == 0){
          if(_env != NULL)
            m_d = (_env->GetMultiBody(_env->GetRobotIndex()))->GetMaxAxisRange();
        }
        if(this->m_debug)
          PrintOptions(cout); 
      }

    GaussianSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("GaussianSampler");
      ParseXML(_node);
      if(this->m_debug)
        PrintOptions(cout); 
    }

    ~GaussianSampler() {} 

    ///////////////////
    //Accessors
    ///////////////////

    void  
      ParseXML(XMLNodeReader& _node) {
        m_d = _node.numberXMLParameter("d", true, 0.0, 0.0, MAX_DBL, "Gaussian D value");
        m_useBBX = _node.boolXMLParameter("useBBX", true, false, "Use bounding box as obstacle");
        m_vcLabel = _node.stringXMLParameter("vcMethod", true, "", "Validity Test Method");
        m_dmLabel =_node.stringXMLParameter("dmMethod", true, "default", "Distance Metric Method");

        _node.warnUnrequestedAttributes();
      }

    //Display values of class vairables at calling time 
    virtual void 
      PrintOptions(ostream& _out) const {
        SamplerMethod<CFG>::PrintOptions(_out);
        _out << "\td = " << m_d << endl; 
        _out << "\tuseBoundingBox = " << m_useBBX << endl; 
        _out << "\tvcLabel = " << m_vcLabel << endl; 
        _out << "\tdmLabel = " << m_dmLabel << endl; 
      }

    virtual bool 
      Sampler(Environment* _env, shared_ptr<BoundingBox> _bb, 
          StatClass& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, 
          vector<CFG>& _cfgCol, int _maxAttempts) {

        string callee(this->GetName() + "::SampleImpl()");
        bool generated = false;
        int attempts = 0;
        ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
        CDInfo cdInfo; 
        shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetDistanceMetric()->GetMethod(m_dmLabel); 
        if(this->m_debug) 
          VDClearAll();  

        //Start configuration, which was passed in. Use random as a default. 
        CFG cfg1 = _cfgIn;
        if(cfg1 == CFG())
          cfg1.GetRandomCfg(_env,_bb);
        if(this->m_debug) 
          cout << "cfg1::" << cfg1 << endl; 

        //First define validity of first CFG. If useBBX, require it to be in BBX
        //and only check validity with obstacles. Otherwise have both conditions.
        bool cfg1Free;
        if(!m_useBBX) {
          if(!cfg1.InBoundingBox(_env,_bb))
            return false;

          cfg1Free = vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg1, _env,
              _stats, cdInfo, true, &callee);
        } 
        else {
          cfg1Free = cfg1.InBoundingBox(_env,_bb) && 
            vc->IsValid(vc->GetVCMethod(m_vcLabel), 
                cfg1, _env, _stats, cdInfo, true, &callee);
        }

        if(this->m_debug){ 
          cout << "cfg1Free::" << cfg1Free << endl;  
          VDAddTempCfg(cfg1, cfg1Free);
        }

        //Attempt to generate node (cfg2) while not at max attempts  
        do {
          _stats.IncNodesAttempted(this->GetNameAndLabel());
          attempts++;

          //Determine cfg2 attributes with similar consdirations/method to cfg1
          CFG cfg2;
          bool cfg2Free;   
          if(!m_useBBX) {
            do {
              CFG incr;
              incr.GetRandomRay(fabs(GaussianDistribution(fabs(m_d), fabs(m_d))), _env, dm);
              cfg2.add(cfg1, incr);
              if(this->m_debug){
                cout << "incr::" << incr << endl;    
                cout << "cfg2::" << cfg2 << endl; 
                VDAddTempRay(incr); 
              }
            } while(!cfg2.InBoundingBox(_env,_bb));

            cfg2Free = vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg2, _env, 
                _stats, cdInfo, true, &callee);
          } 
          else {
            CFG incr;
            incr.GetRandomRay(fabs(GaussianDistribution(fabs(m_d), fabs(m_d))), _env, dm);
            cfg2.add(cfg1, incr);
            if(this->m_debug){
              cout << "incr::" << incr << endl;  
              cout << "cfg2::" << cfg2 << endl;  
              VDAddTempRay(incr);
            }

            cfg2Free = cfg2.InBoundingBox(_env,_bb) && 
              vc->IsValid(vc->GetVCMethod(m_vcLabel), 
                  cfg2, _env, _stats, cdInfo, true, &callee);
          }

          if(this->m_debug){
            cout << "cfg2Free::" << cfg2Free << endl; 
            VDAddTempCfg(cfg2, cfg2Free);
          }

          //If validities are different, retain the free node 
          if(cfg1Free != cfg2Free) {
            _stats.IncNodesGenerated(this->GetNameAndLabel());
            generated = true;
            ostringstream oss;
            oss << "Gaussian node generated::";
            if(cfg1Free) {
              oss << cfg1;
              if(this->m_debug)
                cout << "Generated::" << cfg1 << endl; //May be repetitive.. 
              _cfgOut.push_back(cfg1);
              _cfgCol.push_back(cfg2);
            } else {
              oss << cfg2;
              if(this->m_debug)
                cout << "Generated::" << cfg2 << endl;  
              _cfgOut.push_back(cfg2);
              _cfgCol.push_back(cfg1);
            }
            if(this->m_debug)
              VDComment(oss.str());
          }
        } while (!generated && (attempts < _maxAttempts));
        return generated;
      }
};

#endif

