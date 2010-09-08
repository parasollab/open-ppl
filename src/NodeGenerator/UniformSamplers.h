#ifndef UniformSampler_h
#define UniformSampler_h

#include "SamplerMethod.h"
class Environment;
class Stat_Class;
class CollisionDetection;
class CDInfo;
template <typename CFG> class ValidityChecker;

#include <sstream>


template <typename CFG>
class UniformRandomSampler : public SamplerMethod<CFG>
{
 private:
  Environment* env;
  Stat_Class *Stats;
  std::string strLabel;
  CFG cfg;
  
 public:
  UniformRandomSampler() {}
  
  UniformRandomSampler(Environment* _env, Stat_Class& _Stats) :
    env(_env), Stats(&_Stats) {} 
  
  UniformRandomSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem) 
  {
    LOG_DEBUG_MSG("UniformRandomSampler::UniformRandomSampler()");
    ParseXML(in_Node);
    cout << "UniformRandomSampler here";
    strLabel= this->ParseLabelXML( in_Node);
    this->SetLabel(strLabel);
    LOG_DEBUG_MSG("~UniformRandomSampler::UniformRandomSampler()");
  }
 
  ~UniformRandomSampler() {}

  virtual const char* name() const { return "UniformRandomSampler"; }
  
  void ParseXML(XMLNodeReader& in_Node) 
  {
    LOG_DEBUG_MSG("UniformRandomSampler::ParseXML()");
    //print(cout);
    cout << "UniformRandomSampler";
    string filename = in_Node.stringXMLParameter(string("filename"), true, string(""),string("Links File Name"));
    cout<<"filename="<<filename<<endl;
    cout<<"parsing file"<<endl;
    if(filename!=""){
      cfg.ParseLinksFile(filename.c_str());
      cout<<"file "<<filename<<" has been parsed"<<endl; 
    }
    LOG_DEBUG_MSG("~UniformRandomSampler::ParseXML()");
  }

  void print(ostream& os) const 
  {
    os << name();
  }
  
 private:
  template <typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
          OutputIterator result)  
  {
    cout<<"in urs"<<endl;
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
	
	//CFG tmp;
	cout<<"before ger ran"<<endl;
	cfg.GetRandomCfg(env);
	cout<<"after ger ran"<<endl;
	if(cfg.InBoundingBox(env)) {
		
	  Stat.IncNodes_Generated();
	  generated = true;
	 
	  // cfg_out.push_back(tmp);
	  *result++ = cfg;
	
	}
      } while (!generated && (attempts < max_attempts));
      
    }
    return result;
  }

  template <typename InputIterator, typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, InputIterator first, InputIterator last, int max_attempts,
 	  OutputIterator result)  
  {
    LOG_DEBUG_MSG("UniformRandomSampler::BiasedSample()");
    int num_nodes = distance(first,last);
    _Sample(env, Stat, num_nodes, max_attempts, result);
    return result;
  }   

 public:
  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = back_insert_iterator<vector<CFG> >
  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
         back_insert_iterator<vector<CFG> > result)  
  {
    return _Sample(env, Stat, num_nodes, max_attempts, result);
  }

  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
	 back_insert_iterator<vector<CFG> > result)  
  {
    return _Sample(env, Stat, first, last, max_attempts, result);
  }   
  
  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = vector<CFG>::iterator
  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
         typename vector<CFG>::iterator result)  
  {
    return _Sample(env, Stat, num_nodes, max_attempts, result);
  }

  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
	 typename vector<CFG>::iterator result)  
  {
    return _Sample(env, Stat, first, last, max_attempts, result);
  }
};


template <typename CFG>
class UniformRandomFreeSampler : public SamplerMethod<CFG>
{
 private:
  Environment* env;
  Stat_Class *Stats;
  CollisionDetection* cd;
  ValidityChecker<CFG>* vc;
  std::string strVcmethod;
  CDInfo *cdInfo;
  std::string strLabel;
  CFG cfg;

 public:
  UniformRandomFreeSampler() {}
  
  UniformRandomFreeSampler(Environment* _env, Stat_Class& _Stats, 
                           CollisionDetection* _cd, CDInfo& _cdInfo) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo) {} 
  
  UniformRandomFreeSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem)
  {
    LOG_DEBUG_MSG("UniformRandomFreeSampler::UniformRandomFreeSampler()");
    ParseXML(in_Node);
    cout << "UniformRandomFreeSampler";
    strVcmethod = in_Node.stringXMLParameter(string("vc_method"), true,
                                             string(""), string("Validity Test Method"));
    cout << "strVcmethod = " << strVcmethod << endl;
    vc = in_pProblem->GetValidityChecker();
    strLabel= this->ParseLabelXML( in_Node);
    this->SetLabel(strLabel);
    LOG_DEBUG_MSG("~UniformRandomFreeSampler::UniformRandomFreeSampler()");
  }
  
  ~UniformRandomFreeSampler() {}
 
  void ParseXML(XMLNodeReader& in_Node) 
  {
    LOG_DEBUG_MSG("UniformRandomFreeSampler::ParseXML()");
    cout << "UniformRandomSampler";
    string filename = in_Node.stringXMLParameter(string("filename"), true, string(""),string("Links File Name"));
    cout<<"filename="<<filename<<endl;
    cout<<"parsing file in urf"<<endl;
    if(filename!=""){
      cfg.ParseLinksFile(filename.c_str());
      cout<<"file "<<filename<<" has been parsed"<<endl; 
    }
    //print(cout);
    cout << "UniformRandomFreeSampler";
    LOG_DEBUG_MSG("~UniformRandomFreeSampler::ParseXML()");
  }
  
  virtual const char* name() const { return "UniformRandomFreeSampler"; }

  void print(ostream& os) const
  {
    os << name();
  }
 
 private:
  template <typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
          OutputIterator result)   
  {
    cout<<"in __sample"<<endl;
    string callee(name());
    callee += "::_Sample()";
    CDInfo cdInfo;
    // cout << "num of nodes = " << num_nodes << endl;
    for (int i =0; i< num_nodes; i++){    
      bool generated = false;
      int attempts = 0;
	
      do {
	Stat.IncNodes_Attempted();
	attempts++;
	  
	//CFG tmp;
	//need to set label here
	//tmp.SetLabel("UniformRandomFree_Sampler",true);
	cout<<"before get ran 2"<<endl;
	cfg.GetRandomCfg(env, *Stats, cd, cdInfo);
	//cfg.GetRandomCfg(env);
	cout<<"after get ran 2"<<endl;
	//cout << "RandomCfg::tmp = " << tmp << endl;
	if(cfg.InBoundingBox(env) && 
           //!tmp.isCollision(env, Stat, cd, cdInfo, true, &callee)) 
	vc->IsValid(vc->GetVCMethod(strVcmethod), cfg, env, 
	            Stat, cdInfo, true, &callee))
	//cout << "num of nodes after IsValid = " << num_nodes << endl;
	{
	  Stat.IncNodes_Generated();
	  generated = true;
	  // cfg_out.push_back(tmp);
	  *result++ = cfg;
	}
      } while (!generated && (attempts < max_attempts));
      
    }
    return result;
  }
    
  template <typename InputIterator, typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, InputIterator first, InputIterator last, int max_attempts,
          OutputIterator result)  
  {
    int num_nodes = distance(first,last);
     cout << "_Sample2 num of nodes = " << num_nodes << endl;
    _Sample(env, Stat, num_nodes, max_attempts, result);
    return result;
  }

 public:
  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = back_insert_iterator<vector<CFG> >
  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
         back_insert_iterator<vector<CFG> > result)   
  {
    cout<<"in sample 1"<<endl;
    return _Sample(env, Stat, num_nodes, max_attempts, result);
  }
    
  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
         back_insert_iterator<vector<CFG> > result)  
  {
    cout<<"in sample 2"<<endl;
    return _Sample(env, Stat, first, last, max_attempts, result);
  }

  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = back_insert_iterator<vector<CFG> >
  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
         typename vector<CFG>::iterator result)
  {
    cout<<" in sample 3"<<endl;
    return _Sample(env, Stat, num_nodes, max_attempts, result);
  }
    
  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
         typename vector<CFG>::iterator result)
  {
    cout<<" in sample 4"<<endl;
    return _Sample(env, Stat, first, last, max_attempts, result);
  }
};


template <typename CFG>
class UniformRandomCollisionSampler : public SamplerMethod<CFG>
{
 private:
  Environment* env;
  Stat_Class  *Stats;
  CollisionDetection* cd;
  CDInfo  *cdInfo;
  std::string strLabel;
  CFG cfg;

 public:
  UniformRandomCollisionSampler() {}
  
  UniformRandomCollisionSampler(Environment* _env, Stat_Class& _Stats, 
				CollisionDetection* _cd, CDInfo& _cdInfo) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo) {} 
  
  UniformRandomCollisionSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem)
  {
    LOG_DEBUG_MSG("UniformRandomCollisionSampler::UniformRandomCollisionSampler()");
    ParseXML(in_Node);
    cd = in_pProblem->GetCollisionDetection();
    cout << "UniformRandomCollisionSampler";
    LOG_DEBUG_MSG("~UniformRandomCollisionSampler::UniformRandomCollisionSampler()");
  }
  
  void ParseXML(XMLNodeReader& in_Node) 
  {
    LOG_DEBUG_MSG("UniformRandomCollisionSampler::ParseXML()");
    cout << "UniformRandomSampler";
    string filename = in_Node.stringXMLParameter(string("filename"), true, string(""),string("Links File Name"));
    cout<<"filename="<<filename<<endl;
    cout<<"parsing file"<<endl;
    if(filename!=""){
      cfg.ParseLinksFile(filename.c_str());
      cout<<"file "<<filename<<" has been parsed"<<endl; 
    }
    //print(cout);
    cout << "UniformRandomCollisionSampler";
    strLabel= this->ParseLabelXML( in_Node);
    this->SetLabel(strLabel);
    LOG_DEBUG_MSG("~UniformRandomCollisionSampler::ParseXML()");
  }
  
  ~UniformRandomCollisionSampler() {}

  virtual const char* name() const { return "UniformRandomCollisionSampler"; }

  void print(ostream& os) const
  {
    os << name();
  }

 private:
  template <typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
         OutputIterator result)   
  {
    string callee(name());
    callee += "::_Sample()";
    CDInfo cdInfo;
    // cout << "num of nodes = " << num_nodes << endl;
    for (int i =0; i< num_nodes; i++){ 
      bool generated = false;
      int attempts = 0;
	
      do {
        Stat.IncNodes_Attempted();
        attempts++;
	  
        //CFG tmp;
        cfg.GetRandomCfg(env);
        //change to is valid
        // cout << "tmp b4 IsValid = " << tmp << endl;
        if(cfg.InBoundingBox(env) && cfg.isCollision(env, Stat, cd, cdInfo, true, &callee)) {
          Stat.IncNodes_Generated();
          // cout << "tmp after IsValid = " << tmp << endl;
          generated = true;
          // cfg_out.push_back(tmp);
          *result++ = cfg;
        }
      } while (!generated && (attempts < max_attempts));
    }
    return result;
  }
    
  template <typename InputIterator, typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, InputIterator first, InputIterator last, int max_attempts,
	 OutputIterator result)  
  {
    int num_nodes = distance(first,last);
    _Sample(env, Stat, num_nodes, max_attempts, result);
    return result;
  }

 public:
  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = back_insert_iterator<vector<CFG> >
  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
         back_insert_iterator<vector<CFG> > result)   
  {
    return _Sample(env, Stat, num_nodes, max_attempts, result);
  }
    
  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
	 back_insert_iterator<vector<CFG> > result)  
  {
    return _Sample(env, Stat, first, last, max_attempts, result);
  }
    
  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = vector<CFG>::iterator
  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
         typename vector<CFG>::iterator result)   
  {
    return _Sample(env, Stat, num_nodes, max_attempts, result);
  }
    
  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
	 typename vector<CFG>::iterator result)  
  {
    return _Sample(env, Stat, first, last, max_attempts, result);
  }
};

#endif
