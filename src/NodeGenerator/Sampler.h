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
#include "MPUtils.h"
#include "UniformSamplers.h"
#include "MedialAxisSamplers.h"
#include "GaussianSamplers.h"
#include "ObstacleBasedSamplers.h"
#include "WorkspaceObstacleBasedSamplers.h"
#include "NegateSampler.h"
#include "GridSampler.h"
#include "CfgTypes.h"
#include "SamplerMethod.h"

class MPProblem;
class Environment;

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
    GaussianSampler<CfgType>,
    BridgeTestSampler<CfgType>,
    ObstacleBasedSampler<CfgType>,
    WorkspaceObstacleBasedSampler<CfgType>,
    MedialAxisSampler<CfgType>,
    NegateSampler<CfgType>,
    GridSampler<CfgType>
    > SamplerMethodList;
}


template <class CFG>
class Sampler : private ElementSet<SamplerMethod<CFG> >, public MPBaseObject
{			 
 public:
  typedef typename ElementSet<SamplerMethod<CFG> >::MethodPointer SamplerPointer;
  
  template <typename MethodList>
  Sampler() : ElementSet<SamplerMethod<CFG> >(MethodList()) {}
  
  Sampler() : ElementSet<SamplerMethod<CFG> >(pmpl_detail::SamplerMethodList()) {}    
  
  template <typename MethodList>
  Sampler(XMLNodeReader& in_Node, MPProblem* in_pProblem, MethodList const&)
    : ElementSet<SamplerMethod<CFG> >(MethodList()), MPBaseObject(in_pProblem) 
  { 
    for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) 
      if(!ElementSet<SamplerMethod<CFG> >::AddElement(citr->getName(), *citr, in_pProblem))
        citr->warnUnknownNode();
  }

  Sampler(XMLNodeReader& in_Node, MPProblem* in_pProblem)
    : ElementSet<SamplerMethod<CFG> >(pmpl_detail::SamplerMethodList()), MPBaseObject(in_pProblem) 
  { 
    for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
      if(!ElementSet<SamplerMethod<CFG> >::AddElement(citr->getName(), *citr, in_pProblem))
        citr->warnUnknownNode();
    }
  }
  
  ~Sampler() {}
  
  SamplerPointer GetSamplingMethod(string in_strLabel) 
  {
    SamplerPointer to_return = ElementSet<SamplerMethod<CFG> >::GetElement(in_strLabel);
    if(to_return.get() == NULL) 
      exit(-1);
    return to_return;
  }

  void AddSamplingMethod(string in_strLabel, SamplerPointer in_ptr) 
  {
    ElementSet<SamplerMethod<CFG> >::AddElement(in_strLabel, in_ptr);
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

