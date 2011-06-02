#ifndef NegateSampler_h
#define NegateSampler_h

#include "SamplerMethod.h"
#include "MPProblemAccess.h"

template <typename CFG> class ValidityChecker;

template <typename CFG>
class NegateSampler : public SamplerMethod<CFG>
{
  ValidityChecker<CFG>* vc;
  MPProblem* mps;
  string samplingMethod;

 public:
  NegateSampler() {
    this->SetName("NegateSampler");
  }

  NegateSampler(ValidityChecker<CFG>* v, MPStrategy* s, string sm) : vc(v), mps(s), samplingMethod(sm){
    this->SetName("NegateSampler");
  }
  
  NegateSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem)
  {
    this->SetName("NegateSampler");
    LOG_DEBUG_MSG("NegateSampler::NegateSampler()");
    ParseXML(in_Node);
    cout << "NegateSampler";
    vc = in_pProblem->GetValidityChecker();
    mps = in_pProblem;
    string strLabel= this->ParseLabelXML( in_Node);
    this->SetLabel(strLabel);
    LOG_DEBUG_MSG("~NegateSampler::NegateSampler()");
  }
  
  ~NegateSampler() {}
 
  void ParseXML(XMLNodeReader& in_Node) 
  {
    LOG_DEBUG_MSG("NegateSampler::ParseXML()");
    samplingMethod = in_Node.stringXMLParameter("Method", true, "", "Sampling method to collect collision nodes for");
    //print(cout);
    cout << "NegateSampler";
    LOG_DEBUG_MSG("~NegateSampler::ParseXML()");
  }

 private:
  template <typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
          OutputIterator result, OutputIterator collision)   
  {
    vc->ToggleValidity();
    GetSamplingMethod(mps, samplingMethod)->Sample(env, Stat, num_nodes, max_attempts, result, collision);
    vc->ToggleValidity();
    return result;
  }
    
  template <typename InputIterator, typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, InputIterator first, InputIterator last, int max_attempts,
          OutputIterator result, OutputIterator collision)  
  {
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

