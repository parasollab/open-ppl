#ifndef UniformSampler_h
#define UniformSampler_h

#include "SamplerMethod.h"
class Environment;
class Stat_Class;
class CDInfo;
template <typename CFG> class ValidityChecker;

template <typename CFG>
class UniformRandomSampler : public SamplerMethod<CFG>
{
  Environment* env;
  
 public:
  UniformRandomSampler() {
    this->SetName("UniformRandomSampler");
  }
  
  UniformRandomSampler(Environment* _env) :
    env(_env) {
      this->SetName("UniformRandomSampler");
    } 
  
  UniformRandomSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem) 
  {
    this->SetName("UniformRandomSampler");
    LOG_DEBUG_MSG("UniformRandomSampler::UniformRandomSampler()");
    ParseXML(in_Node);
    cout << "UniformRandomSampler";
    string strLabel= this->ParseLabelXML( in_Node);
    this->SetLabel(strLabel);
    LOG_DEBUG_MSG("~UniformRandomSampler::UniformRandomSampler()");
  }
 
  ~UniformRandomSampler() {}
  
  void ParseXML(XMLNodeReader& in_Node) 
  {
    LOG_DEBUG_MSG("UniformRandomSampler::ParseXML()");
    //print(cout);
    cout << "UniformRandomSampler";
    LOG_DEBUG_MSG("~UniformRandomSampler::ParseXML()");
  }
  
 private:
  template <typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
          OutputIterator result, OutputIterator collision)  
  {
    LOG_DEBUG_MSG("UniformRandomSampler::UnbiasedSample()");
    /* string callee(name());
    callee += "::GenerateNode()";*/
    // cout << "num of nodes = " << num_nodes << endl;
    // for (int i =0; i< num_nodes; i++){ 
    bool generated = false;
    int attempts = 0;
    for (int i =0; i< num_nodes; i++) { 
      //cout << "Node number  = " << i << endl;
      do {
	Stat.IncNodes_Attempted();
	attempts++;
	
	CFG tmp;
	
	tmp.GetRandomCfg(env);
	
	if(tmp.InBoundingBox(env)) {
		
	  Stat.IncNodes_Generated();
	  generated = true;
	 
	  // cfg_out.push_back(tmp);
	  *result++ = tmp;
          *collision++ = tmp;
	
	}
      } while (!generated && (attempts < max_attempts));
      
    }
    return result;
  }

  template <typename InputIterator, typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, InputIterator first, InputIterator last, int max_attempts,
 	  OutputIterator result, OutputIterator collision)  
  {
    LOG_DEBUG_MSG("UniformRandomSampler::BiasedSample()");
    int num_nodes = distance(first,last);
    _Sample(env, Stat, num_nodes, max_attempts, result, collision);
    return result;
  }   

 public:
  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = back_insert_iterator<vector<CFG> >
  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
         back_insert_iterator<vector<CFG> > result, back_insert_iterator<vector<CFG> > collision)  
  {
    return _Sample(env, Stat, num_nodes, max_attempts, result, collision);
  }

  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
	 back_insert_iterator<vector<CFG> > result, back_insert_iterator<vector<CFG> > collision)  
  {
    return _Sample(env, Stat, first, last, max_attempts, result, collision);
  }   
 
  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
         back_insert_iterator<vector<CFG> > result)  
  {
    vector<CFG> collision;
    return _Sample(env, Stat, num_nodes, max_attempts, result, back_inserter(collision));
  }

  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
	 back_insert_iterator<vector<CFG> > result)  
  {
    vector<CFG> collision;
    return _Sample(env, Stat, first, last, max_attempts, result, back_inserter(collision));
  }   
  
  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = vector<CFG>::iterator
  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
         typename vector<CFG>::iterator result, typename vector<CFG>::iterator collision)  
  {
    return _Sample(env, Stat, num_nodes, max_attempts, result, collision);
  }

  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
	 typename vector<CFG>::iterator result, typename vector<CFG>::iterator collision)  
  {
    return _Sample(env, Stat, first, last, max_attempts, result, collision);
  }
  
  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
         typename vector<CFG>::iterator result)  
  {
    vector<CFG> collision(max_attempts * num_nodes);
    return _Sample(env, Stat, num_nodes, max_attempts, result, collision.begin());
  }

  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
	 typename vector<CFG>::iterator result)  
  {
    vector<CFG> collision(max_attempts * distance(first, last));
    return _Sample(env, Stat, first, last, max_attempts, result, collision.begin());
  }
};


template <typename CFG>
class UniformRandomFreeSampler : public SamplerMethod<CFG>
{
 private:
  ValidityChecker<CFG>* vc;
  std::string strVcmethod;

 public:
  UniformRandomFreeSampler() {
    this->SetName("UniformRandomFreeSampler");
  }
  
  UniformRandomFreeSampler(ValidityChecker<CFG>* v, string vcMethod) :
    vc(v), strVcmethod(vcMethod) {
      this->SetName("UniformRandomFreeSampler");
  } 
  
  UniformRandomFreeSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem)
  {
    this->SetName("UniformRandomFreeSampler");
    LOG_DEBUG_MSG("UniformRandomFreeSampler::UniformRandomFreeSampler()");
    ParseXML(in_Node);
    cout << "UniformRandomFreeSampler";
    strVcmethod = in_Node.stringXMLParameter(string("vc_method"), true,
                                             string(""), string("Validity Test Method"));
    cout << "strVcmethod = " << strVcmethod << endl;
    vc = in_pProblem->GetValidityChecker();
    string strLabel= this->ParseLabelXML( in_Node);
    this->SetLabel(strLabel);
    LOG_DEBUG_MSG("~UniformRandomFreeSampler::UniformRandomFreeSampler()");
  }
  
  ~UniformRandomFreeSampler() {}
 
  void ParseXML(XMLNodeReader& in_Node) 
  {
    LOG_DEBUG_MSG("UniformRandomFreeSampler::ParseXML()");
    //print(cout);
    cout << "UniformRandomFreeSampler";
    LOG_DEBUG_MSG("~UniformRandomFreeSampler::ParseXML()");
  }

 private:
  template <typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
          OutputIterator result, OutputIterator collision)   
  {
    string callee(this->GetName());
    callee += "::_Sample()";
    CDInfo cdInfo;
    // cout << "num of nodes = " << num_nodes << endl;
    for (int i =0; i< num_nodes; i++){    
      bool generated = false;
      int attempts = 0;
	
      do {
	Stat.IncNodes_Attempted();
	attempts++;
	  
	CFG tmp;
	//need to set label here
	//tmp.SetLabel("UniformRandomFree_Sampler",true);
	tmp.GetRandomCfg(env);
	//cout << "RandomCfg::tmp = " << tmp << endl;
	if(tmp.InBoundingBox(env)){
	 if(vc->IsValid(vc->GetVCMethod(strVcmethod), tmp, env, 
	            Stat, cdInfo, true, &callee))
	//cout << "num of nodes after IsValid = " << num_nodes << endl;
         {
            Stat.IncNodes_Generated();
            generated = true;
            // cfg_out.push_back(tmp);
            *result++ = tmp;
         }
         else{
            *collision++ = tmp;
         }
        }
      } while (!generated && (attempts < max_attempts));
      
    }
    return result;
  }
    
  template <typename InputIterator, typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, InputIterator first, InputIterator last, int max_attempts,
          OutputIterator result, OutputIterator collision)  
  {
    int num_nodes = distance(first,last);
    // cout << "_Sample2 num of nodes = " << num_nodes << endl;
    _Sample(env, Stat, num_nodes, max_attempts, result, collision);
    return result;
  }

 public:
  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = back_insert_iterator<vector<CFG> >
  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
         back_insert_iterator<vector<CFG> > result, back_insert_iterator<vector<CFG> > collision)   
  {
    return _Sample(env, Stat, num_nodes, max_attempts, result, collision);
  }
    
  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
         back_insert_iterator<vector<CFG> > result, back_insert_iterator<vector<CFG> > collision)  
  {
    return _Sample(env, Stat, first, last, max_attempts, result, collision);
  }

  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
         back_insert_iterator<vector<CFG> > result)   
  {
    vector<CFG> collision;
    return _Sample(env, Stat, num_nodes, max_attempts, result, back_inserter(collision));
  }
    
  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
         back_insert_iterator<vector<CFG> > result)  
  {
    vector<CFG> collision;
    return _Sample(env, Stat, first, last, max_attempts, result, back_inserter(collision));
  }

  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = back_insert_iterator<vector<CFG> >
  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
         typename vector<CFG>::iterator result, typename vector<CFG>::iterator collision)
  {
    return _Sample(env, Stat, num_nodes, max_attempts, result, collision);
  }
    
  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
         typename vector<CFG>::iterator result, typename vector<CFG>::iterator collision)
  {
    return _Sample(env, Stat, first, last, max_attempts, result, collision);
  }
  
  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
         typename vector<CFG>::iterator result)
  {
    vector<CFG> collision(max_attempts * num_nodes);
    return _Sample(env, Stat, num_nodes, max_attempts, result, collision.begin());
  }
    
  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
         typename vector<CFG>::iterator result)
  {
    vector<CFG> collision(max_attempts * distance(first, last));
    return _Sample(env, Stat, first, last, max_attempts, result, collision.begin());
  }

};

#endif

