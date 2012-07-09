#ifndef UNIFORMRANDOMSAMPLER_H_      
#define UNIFORMRANDOMSAMPLER_H_

#include "SamplerMethod.h"

class Environment;
class StatClass;
class CDInfo;
template <typename CFG> class ValidityChecker;

template <typename CFG>
class UniformRandomSampler : public SamplerMethod<CFG> {
  private:
    std::string m_vcLabel;  //Validity-checker label 

  public:
    //////////////////////
    //Constructors
    //////////////////////
    UniformRandomSampler(string _vcLabel = "") : m_vcLabel(_vcLabel) {
      this->SetName("UniformRandomSampler");
      if(this->m_debug)
        PrintOptions(cout);
    } 

    UniformRandomSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("UniformRandomSampler");
      ParseXML(_node);
      if(this->m_debug)
        PrintOptions(cout);
    }

    ~UniformRandomSampler() {}         

    ///////////////////
    //Accessors
    ///////////////////
    void 
      ParseXML(XMLNodeReader& _node) {
        m_vcLabel = _node.stringXMLParameter("vcMethod", true, "", "Validity Test Method");
      }

    virtual void 
      PrintOptions(ostream& _out) const {
        SamplerMethod<CFG>::PrintOptions(_out);
        _out << "\tvcLabel = " << m_vcLabel << endl;
      }

  protected:
    virtual bool 
      Sampler(Environment* _env, shared_ptr<Boundary> _bb, 
          StatClass& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, 
          vector<CFG>& _cfgCol) { 

        string callee(this->GetName() + "::SampleImpl()");
        ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
        CDInfo cdInfo;

        if(this->m_debug) 
          VDClearAll();

        _stats.IncNodesAttempted(this->GetNameAndLabel());

        //Obtain a random configuration 
        CFG tmp;
        tmp.GetRandomCfg(_env,_bb);     

        //Is configuration within boundary? 
        bool inBBX = tmp.InBoundary(_env, _bb);          
        if(this->m_debug){ 
          cout << "tmp::" << tmp << endl;
          cout << "InBoudary::" << inBBX << endl;
        }

        //Good. Now determine validity. 
        if(inBBX) { 
          bool isValid = vc->IsValid(vc->GetVCMethod(m_vcLabel), tmp, _env,   
              _stats, cdInfo, true, &callee);                      
          if(this->m_debug){ 
            cout << "IsValid::" << isValid << endl;
            VDAddTempCfg(tmp, isValid); 
            if(isValid) 
              VDComment("UniformSampling::Cfg valid");   
            else
              VDComment("UniformSampling::Cfg invalid"); 
          }
          //Record valid node and confirm successful generation. 
          if(isValid) {                                        
            _stats.IncNodesGenerated(this->GetNameAndLabel());          
            if(this->m_debug) 
              cout << "Generated::" << tmp << endl; 
            _cfgOut.push_back(tmp);
            return true;
          }
          //Otherwise, unsuccessful. 
          else {
            _cfgCol.push_back(tmp);
            return false;
          }  
        } 
        //Sampled outside of boundary
        else if(this->m_debug){
          cout << "Attempt outside of boundary" << endl;
          VDAddTempCfg(tmp, false);
          VDComment("UniformSampling::Cfg outside of boundary");
        }
        return false;
      } 
};

#endif

