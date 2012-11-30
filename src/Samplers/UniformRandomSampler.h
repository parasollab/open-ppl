#ifndef UNIFORMRANDOMSAMPLER_H_      
#define UNIFORMRANDOMSAMPLER_H_

#include "SamplerMethod.h"

class Environment;
class StatClass;
class CDInfo;

template <class MPTraits>
class UniformRandomSampler : public SamplerMethod<MPTraits> {
  private:
    string m_vcLabel; 

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    
    UniformRandomSampler(string _vcLabel = "") : m_vcLabel(_vcLabel) {
      this->SetName("UniformRandomSampler");
    } 

    UniformRandomSampler(MPProblemType* _problem, XMLNodeReader& _node) : SamplerMethod<MPTraits>(_problem, _node) {
      this->SetName("UniformRandomSampler");
      ParseXML(_node);
    }

    ~UniformRandomSampler() {}         

    void ParseXML(XMLNodeReader& _node) {
      m_vcLabel = _node.stringXMLParameter("vcMethod", true, "", "Validity Test Method");
    }

    virtual void PrintOptions(ostream& _out) const {
      SamplerMethod<MPTraits>::PrintOptions(_out);
      _out << "\tvcLabel = " << m_vcLabel << endl;
    }

  protected:
    virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb, 
        StatClass& _stats, CfgType& _cfgIn, vector<CfgType>& _cfgOut, 
        vector<CfgType>& _cfgCol) { 

      string callee(this->GetNameAndLabel() + "::SampleImpl()");
      ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
      CDInfo cdInfo;

      if(this->m_debug) 
        VDClearAll();

      _stats.IncNodesAttempted(this->GetNameAndLabel());

      //Obtain a random configuration 
      CfgType tmp;
      tmp.GetRandomCfg(_env,_bb);     

      //Is configuration within boundary? 
      bool inBBX = tmp.InBoundary(_env, _bb);          
      if(this->m_debug){ 
        cout << "tmp::" << tmp << endl;
        cout << "InBoudary::" << inBBX << endl;
      }

      //Good. Now determine validity. 
      if(inBBX) { 
        bool isValid = vcm->IsValid(tmp, _env, _stats, cdInfo, &callee);                      
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

