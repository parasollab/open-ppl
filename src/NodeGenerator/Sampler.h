//////////////////////////////////////////////////////////////////////////////////////////
/**@file Sampler.h
  *This set of classes supports a "RoadMap Node Generation Algobase".
  *Generate roadmap nodes and enter each as a graph node.
  *
  *
  *
  *
  *@author Sam Ade Jacobs
  *@date   5/6/09
  * 
  */
////////////////////////////////////////////////////////////////////////////////////////////
#ifndef Sampler_h
#define Sampler_h

#include <boost/mpl/list.hpp>
#include "PMPL_Element_Set.h"
#include "CollisionDetection.h"
#include "Clock_Class.h"
#include "GMSPolyhedron.h"
#include  "UniformSamplers.h"
#include  "MedialAxisSamplers.h"
#include  "GaussianSamplers.h"
#include  "ObstacleBasedSamplers.h"
#include  "WorkspaceObstacleBasedSamplers.h"
#include "util.h"
#include "CfgTypes.h"
#include <sstream>
#include "SamplerMethod.h"


class Body;
class MPProblem;
class n_str_param;
class MultiBody;
class Input;
class Environment;
template <class CFG, class WEIGHT> class Roadmap;


//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class Sampler
//
//
//////////////////////////////////////////////////////////////////////////////////////////


namespace pmpl_detail { //hide SamplerMethodList in pmpl_detail namespace
  typedef boost::mpl::list<
    UniformRandomSampler<CfgType>,
    UniformRandomFreeSampler<CfgType>,
    UniformRandomCollisionSampler<CfgType>,
    GaussRandomSampler<CfgType, true>,
    GaussRandomSampler<CfgType, false>,
    BridgeTestRandomFreeSampler<CfgType>,
    ObstacleBasedSampler<CfgType>,
    WorkspaceObstacleBasedSampler<CfgType>,
    FreeMedialAxisSampler<CfgType>
    > SamplerMethodList;
}


template <class CFG>
class Sampler : private element_set<SamplerMethod<CFG> >, public MPBaseObject
{			 
 public:
  typedef typename element_set<SamplerMethod<CFG> >::method_pointer SamplerPointer;
  template <typename MethodList>
  Sampler() : element_set<SamplerMethod<CFG> >(pmpl_detail::SamplerMethodList()) {}
  Sampler() : element_set<SamplerMethod<CFG> >(pmpl_detail::SamplerMethodList()) {}    
  template <typename MethodList>
  Sampler(XMLNodeReader& in_Node, MPProblem* in_pProblem, MethodList const&)
    : element_set<SamplerMethod<CFG> >(MethodList()), MPBaseObject(in_pProblem) 
  { 
    for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) 
      if(!element_set<SamplerMethod<CFG> >::add_element(citr->getName(), *citr, in_pProblem))
        citr->warnUnknownNode();
  }

  Sampler(XMLNodeReader& in_Node, MPProblem* in_pProblem)
    : element_set<SamplerMethod<CFG> >(pmpl_detail::SamplerMethodList()), MPBaseObject(in_pProblem) 
  { 
    for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) 
      if(!element_set<SamplerMethod<CFG> >::add_element(citr->getName(), *citr, in_pProblem))
        citr->warnUnknownNode();
  }
  
  ~Sampler() {}
  
  SamplerPointer GetSamplingMethod(string in_strLabel) 
  {
    SamplerPointer to_return = element_set<SamplerMethod<CFG> >::get_element(in_strLabel);
    if(to_return.get() == NULL) 
      exit(-1);
    return to_return;
  }

  void AddSamplingMethod(string in_strLabel, SamplerPointer in_ptr) 
  {
    element_set<SamplerMethod<CFG> >::add_element(in_strLabel, in_ptr);
  }
  
  virtual void PrintOptions(ostream& out_os) { }


  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = back_insert_iterator<vector<CFG> >
  back_insert_iterator<vector<CFG> >
  Sample(SamplerPointer _sp, Environment* env, Stat_Class& Stats, typename vector<CFG>::iterator
  _input_first, typename vector<CFG>::iterator _input_last, int _attempts,
  back_insert_iterator<vector<CFG> > _out, back_insert_iterator<vector<CFG> > _out_collision) 
  {
    return _sp->Sample(env, Stats, _input_first, _input_last, _attempts, _out, _out_collision);
  }
  
  back_insert_iterator<vector<CFG> >
  Sample(SamplerPointer _sp, Environment* env, Stat_Class& Stats, int _num_nodes, int _attempts,
  back_insert_iterator<vector<CFG> > _out, back_insert_iterator<vector<CFG> > _out_collision) 
  {
    return _sp->Sample(env, Stats, _num_nodes, _attempts, _out, _out_collision);
  }


  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = vector<CFG>::iterator
  typename vector<CFG>::iterator
  Sample(SamplerPointer _sp, Environment* env, Stat_Class& Stats, typename vector<CFG>::iterator
  _input_first, typename vector<CFG>::iterator _input_last, int _attempts, typename
  vector<CFG>::iterator _out, typename vector<CFG>::iterator _out_collision) 
  {
    return _sp->Sample(env, Stats, _input_first, _input_last, _attempts, _out, _out_collision);
  }
  
  typename vector<CFG>::iterator
  Sample(SamplerPointer _sp, Environment* env, Stat_Class& Stats, int _num_nodes, int _attempts,
  typename vector<CFG>::iterator _out, typename vector<CFG>::iterator _out_collision) 
  {
    return _sp->Sample(env, Stats, _num_nodes, _attempts, _out, _out_collision);
  }

};

#endif

