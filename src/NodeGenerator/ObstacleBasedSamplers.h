#ifndef ObstacleBasedSamplers_h
#define ObstacleBasedSamplers_h

#include "SamplerMethod.h"
#include "Environment.h"
class Environment;
class Stat_Class;
class CDInfo;
class DistanceMetric;

//#include <sstream>

template <typename CFG>
class ObstacleBasedSampler : public SamplerMethod<CFG>
{
 private:
  Environment* env;
  shared_ptr<DistanceMetricMethod >dm;
  ValidityChecker<CFG>* vc;
  std::string strVcmethod;
  int n_shells_free, n_shells_coll;
  double step_size;
  bool useBBX;
  
 public:
  ObstacleBasedSampler() {
    this->SetName("ObstacleBasedSampler");
  }

  ObstacleBasedSampler(Environment* _env, 
		       shared_ptr<DistanceMetricMethod> _dm, int _free = 1, int _coll = 0, 
		       double _step = 0)
    :  env(_env), dm(_dm), n_shells_free(_free), n_shells_coll(_coll), step_size(_step)
  { 
    this->SetName("ObstacleBasedSampler");
    if(step_size <= 0)
      step_size = min(env->GetPositionRes(), env->GetOrientationRes());
  }
  
  ObstacleBasedSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem)
  {
    LOG_DEBUG_MSG("ObstacleBasedSampler::ObstacleBasedSampler()");
    this->SetName("ObstacleBasedSampler");
    ParseXML(in_Node);
    cout << "ObstacleBasedSampler";
    strVcmethod = in_Node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
    vc = in_pProblem->GetValidityChecker();
    string dm_label = in_Node.stringXMLParameter("dm_method", true, "default", "Distance Metric Method");
    dm = in_pProblem->GetDistanceMetric()->GetDMMethod(dm_label);
    env = in_pProblem->GetEnvironment();
    
    string strLabel= this->ParseLabelXML( in_Node);
    this->SetLabel(strLabel);
    cout << "step_size = " << step_size << endl;
    if(step_size <= 0.0)
      step_size = min(env->GetPositionRes(), env->GetOrientationRes());
    
    LOG_DEBUG_MSG("~ObstacleBasedSampler::ObstacleBasedSampler()");
  }
  
  ~ObstacleBasedSampler() {}
  
  void  ParseXML(XMLNodeReader& in_Node) 
  {
    LOG_DEBUG_MSG("ObstacleBasedSampler::ParseXML()");
    XMLNodeReader::childiterator citr;
    useBBX = in_Node.boolXMLParameter("usebbx", true, false, "Use bounding box as obstacle");
    cout << "from parseXML,useBBX = " <<useBBX<< endl;
    for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
      if(citr->getName() == "n_shells_coll") {
        n_shells_coll = citr->numberXMLParameter("number",true, 3,0,10, "Number of Col Shells");
        citr->warnUnrequestedAttributes();
      } else if(citr->getName() == "n_shells_free") {
        n_shells_free = citr->numberXMLParameter("number",true, 3,0,10, "Number of Free Shells");
        citr->warnUnrequestedAttributes();
      } else if(citr->getName() == "step_size") {
        step_size = citr->numberXMLParameter("number",true, 0.0,0.0,10.0,
            "step size used in increment of cfg position towards or away from obstacles");
        citr->warnUnrequestedAttributes();
      }
    }
    cout << "ObstacleBasedSampler";
    LOG_DEBUG_MSG("~ObstacleBasedSampler::ParseXML()");
  }

  virtual void print(ostream& os) const
  {
    os << this->GetName() 
       << " (n_shells_free = " << n_shells_free 
       << ", n_shells_coll = " << n_shells_coll 
       << ", stepsize = " << step_size << ")";
  }
 
  template <typename OutputIterator>
  OutputIterator GenerateShells(Stat_Class& Stats,CFG c_free, CFG c_coll, CFG incr, 
                                OutputIterator result) 
  {
    CDInfo cdInfo;
    string callee(this->GetName());
    callee += "::GenerateShells";
    // cout << "n_shells_coll = " << n_shells_coll << endl;
    for(int i=0; i<n_shells_free; ++i) {
      if(c_free.InBoundingBox(env) && 
        vc->IsValid(vc->GetVCMethod(strVcmethod), c_free, env, 
                    Stats, cdInfo, true, &callee))
      {
        Stats.IncNodes_Generated();
        *result = c_free;
        result++;
      }
      c_free.Increment(incr);
    }
      
    CFG tmp;
    // cout << "n_shells_coll = " << n_shells_coll << endl;
    incr.subtract(tmp, incr);
    for(int i=0; i<n_shells_coll; ++i) {
      if(c_coll.InBoundingBox(env) && 
         !vc->IsValid(vc->GetVCMethod(strVcmethod), c_coll, env, 
                      Stats, cdInfo, true, &callee))
      {
        Stats.IncNodes_Generated();
        *result = c_coll;
        result++;
      }
      c_coll.Increment(incr);
    }
      
    return result;
  }
 
  bool sampler(Environment* env,Stat_Class& Stat,const CFG& cfg_in, vector<CFG>& cfg_out, int max_attempts, vector<CFG>& cfg_out_collision)
  {  
    string callee(this->GetName());
    callee += "::sampler()";
    CDInfo cdInfo;
    bool generated = false;
    
    int attempts = 0;
    do {
      Stat.IncNodes_Attempted();
      attempts++;

      CFG c1 = cfg_in;
      if(cfg_in==CFG())
        c1.GetRandomCfg(env);//random configurations taken inside bounding box
      
      bool c1_bbox = c1.InBoundingBox(env);

      bool c1_free = vc->IsValid(vc->GetVCMethod(strVcmethod), c1, env, 
                                 Stat, cdInfo, true, &callee);

      CFG c2 = c1;
      bool c2_bbox = c1_bbox;
      bool c2_free = c1_free;

      CFG r;
      r.GetRandomRay(step_size, env, dm);

      while(c2_bbox && (c1_free == c2_free)) { 
        c1 = c2;
        c1_bbox = c2_bbox;
        c1_free = c2_free;

        c2.Increment(r);
        c2_bbox = c2.InBoundingBox(env);

        c2_free = vc->IsValid(vc->GetVCMethod(strVcmethod), c2, env, 
                              Stat, cdInfo, true, &callee);

      }
      if(c2_bbox) 
      {
        generated = true;

        if(c1_free)
        {
          CFG tmp;
          r.subtract(tmp, r);
          GenerateShells(Stat,c1, c2, r, 
                         back_insert_iterator<vector<CFG> >(cfg_out));
          cfg_out_collision.push_back(c2);
        }
        else 
        {
          GenerateShells(Stat,c2, c1, r,back_insert_iterator<vector<CFG> >(cfg_out));
          cfg_out_collision.push_back(c1);
        }
      }
      else if(c1_bbox && useBBX && c1_free && !c2_bbox)
      {
        generated = true;
        CFG tmp;
        r.subtract(tmp, r);
        GenerateShells(Stat,c1, c2, r, 
                       back_insert_iterator<vector<CFG> >(cfg_out));
        cfg_out_collision.push_back(c2);
      }
    } while (!generated && (attempts < max_attempts));

    return generated;
  }
    
 private:
  template <typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
          OutputIterator result, OutputIterator collision)  
  {
    CFG my_cfg;
    
    vector<CFG> out1, collision_out;
    for (int i =0; i< num_nodes; i++) {
      my_cfg.GetRandomCfg(env);
      while(!sampler(env, Stat,my_cfg, out1, max_attempts, collision_out))
	my_cfg.GetRandomCfg(env);
      }
    result = copy(out1.begin(), out1.end(), result);
    collision = copy(collision_out.begin(), collision_out.end(), collision);
    return result;
  }

  template <typename InputIterator, typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, InputIterator first, InputIterator last, int max_attempts,
 	  OutputIterator result, OutputIterator collision)  
  {
    while(first != last) {
      vector<CFG> result_cfg, collision_cfg;
      if(sampler(env, Stat, *first, result_cfg, max_attempts, collision_cfg)){
        result = copy(result_cfg.begin(), result_cfg.end(), result);
        collision = copy(collision_cfg.begin(), collision_cfg.begin(), collision);
      }
      first++;
    }
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

#endif
 
