#ifndef BRIDGETESTSAMPLER_H_
#define BRIDGETESTSAMPLER_H_

#include "SamplerMethod.h"

template <typename CFG>
class BridgeTestSampler : public SamplerMethod<CFG> {
  private:
    double m_d;  //Gaussian d-value obtained from distribution 
    bool m_useBBX;   //use Bbox as obstacle? 
    string m_vcLabel, m_dmLabel;

  public:

    ////////////////////
    //Constructors
    ////////////////////

    BridgeTestSampler(Environment* _env = NULL, string _vcLabel = "", string _dmLabel = "", 
        double _d = 0, bool _useBoundingBox = false)
      : m_d(_d), m_useBBX(_useBoundingBox), m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
        this->SetName("BridgeTestSampler");
        if(m_d == 0){
          if(_env != NULL)   
            m_d = (_env->GetMultiBody(_env->GetRobotIndex()))->GetMaxAxisRange();
          else{
            cerr << "Error:: Environment set to NULL in Bridge-Test non-XML constructor. Exiting." << endl; 
            exit(1);
          } 
        }
        if(this->m_debug)
          PrintOptions(cout); 
      }

    BridgeTestSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("BridgeTestSampler");
      ParseXML(_node);
      if(this->m_debug)
        PrintOptions(cout); 
    }

    ~BridgeTestSampler() {}

    /////////////////////
    //Accessors
    /////////////////////

    void  
      ParseXML(XMLNodeReader& _node) {
        m_d = _node.numberXMLParameter("d",true, 0.0, 0.0,100.0,"bridge_d"); 
        m_useBBX = _node.boolXMLParameter("useBBX", false, true, "Use the Bounding Box as an Obstacle");
        m_vcLabel = _node.stringXMLParameter("vcMethod", true, "", "Validity Test Method");
        m_dmLabel = _node.stringXMLParameter("dmMethod", true, "default", "Distance Metric Method");

        _node.warnUnrequestedAttributes();
      }

    //Display values of class variables at calling time 
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
        ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
        CDInfo cdInfo;
        shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetDistanceMetric()->GetMethod(m_dmLabel); 
        CFG blankCfg;         
        bool generated = false;
        int attempts = 0;

        if(this->m_debug) 
          VDClearAll(); 

        do {
          _stats.IncNodesAttempted(this->GetNameAndLabel());
          attempts++;  

          ///Acquire first Cfg.
          CFG tmp = _cfgIn;
          if (tmp == blankCfg)  
            tmp.GetRandomCfg(_env,_bb);
          if(this->m_debug)
            cout << "tmp::" << tmp << endl;  

          //If using Bbox as obstacle  
          if ( m_useBBX ) {
            //If tmp is valid configuration extend rays in opposite directions
            //at length Gaussian d/2
            if ( tmp.InBoundingBox(_env,_bb) && 
                vc->IsValid(vc->GetVCMethod(m_vcLabel), tmp, _env, _stats, cdInfo, true, &callee)) {  
              CFG mid = tmp, incr, cfg1;
              incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d))/2, _env, dm);
              cfg1.subtract(mid, incr); 
              if(this->m_debug)
                cout << "cfg1::" << cfg1 << endl;  

              //If cfg1 is invalid (including Bbox) after adjustment, create cfg2 
              if ( !cfg1.InBoundingBox(_env,_bb) || 
                  !vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg1, _env, _stats, cdInfo, true, &callee)) {
                CFG cfg2;
                cfg2.add(mid, incr);
                if(this->m_debug)
                  cout << "cfg2::" << cfg2 << endl;  

                //If cfg2 also invalid, node generation successful 
                if(!cfg2.InBoundingBox(_env,_bb) || 
                    !vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg2, _env, _stats, cdInfo, true, &callee)) {
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
              CFG cfg1 = tmp, incr, cfg2; 
              incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d)), _env, dm);
              cfg2.add(cfg1, incr);
              if(this->m_debug){
                cout << "cfg1::" << cfg1 << endl; 
                cout << "cfg2::" << cfg2 << endl; 
              }
              //If both cfg1 and cfg2 invalid, create mid and generate node
              //successfully if mid is valid 
              if ( !cfg2.InBoundingBox(_env,_bb) || 
                  !vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg2, _env, _stats, cdInfo, true, &callee)) {
                CFG mid;
                mid.WeightedSum(cfg1, cfg2, 0.5);
                if ( mid.InBoundingBox(_env,_bb) && (vc->IsValid(vc->GetVCMethod(m_vcLabel), 
                        mid, _env, _stats, cdInfo, true, &callee))) {
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
            if ( vc->IsValid(vc->GetVCMethod(m_vcLabel), tmp, _env, _stats, cdInfo, true, &callee) ) { 
              CFG mid = tmp, incr, cfg1;
              incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d))/2, _env, dm);
              cfg1.subtract(mid, incr);
              if(this->m_debug)
                cout << "cfg1::" << cfg1 << endl;  

              //If invalid cfg1, create cfg2. Want both invalid. 
              if( !vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg1, _env, _stats, cdInfo, true, &callee) ) {
                CFG cfg2;
                cfg2.add(mid, incr);
                if(this->m_debug)
                  cout << "cfg2::" << cfg2 << endl;  

                //If both cfg1 and cfg2 invalid; valid tmp is successfully generated
                //midpoint node
                if( !vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg2, _env, _stats, cdInfo, true, &callee)) {
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
              CFG cfg1 = tmp, incr, cfg2;
              incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d)), _env, dm);
              cfg2.add(cfg1, incr);
              if(this->m_debug){
                cout << "cfg1::" << cfg1 << endl; 
                cout << "cfg2::" << cfg2 << endl; 
              } 

              //If both cfg1 and cfg2 invalid, get midpoint 
              if ( !vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg2, _env, _stats, cdInfo, true, &callee)) {
                CFG mid;
                mid.WeightedSum(cfg1, cfg2, 0.5);

                //If valid midpoint, successful node is generated 
                if( (vc->IsValid(vc->GetVCMethod(m_vcLabel), mid, _env, _stats, cdInfo, true, &callee)) ) {
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
        } while (!generated && (attempts < _maxAttempts));
        return generated;
      }
};

#endif

