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
#ifndef SAMPLER_H_
#define SAMPLER_H_

#include <boost/mpl/list.hpp>
#include "MPUtils.h"
#include "UniformSamplers.h"
#include "MedialAxisSamplers.h"
#include "GaussianSampler.h"
#include "BridgeTestSampler.h"
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
class Sampler : private ElementSet<SamplerMethod<CFG> >, public MPBaseObject {			 
  public:
    typedef typename ElementSet<SamplerMethod<CFG> >::MethodPointer SamplerPointer;
    
    template <typename MethodList>
    Sampler() : ElementSet<SamplerMethod<CFG> >(MethodList()) {}
    
    Sampler() : ElementSet<SamplerMethod<CFG> >(pmpl_detail::SamplerMethodList()) {}    
    
    template <typename MethodList>
    Sampler(XMLNodeReader& _node, MPProblem* _problem, MethodList const&)
      : ElementSet<SamplerMethod<CFG> >(MethodList()), MPBaseObject(_problem) { 
      for(XMLNodeReader::childiterator citr = _node.children_begin(); citr!= _node.children_end(); ++citr) 
        if(!ElementSet<SamplerMethod<CFG> >::AddElement(citr->getName(), *citr, _problem))
          citr->warnUnknownNode();
    }
  
    Sampler(XMLNodeReader& _node, MPProblem* _problem)
      : ElementSet<SamplerMethod<CFG> >(pmpl_detail::SamplerMethodList()), MPBaseObject(_problem) { 
      for(XMLNodeReader::childiterator citr = _node.children_begin(); citr!= _node.children_end(); ++citr) 
        if(!ElementSet<SamplerMethod<CFG> >::AddElement(citr->getName(), *citr, _problem))
          citr->warnUnknownNode();
    }
    
    ~Sampler() {}
    
    SamplerPointer GetMethod(string _label) {
      SamplerPointer toReturn = ElementSet<SamplerMethod<CFG> >::GetElement(_label);
      if(toReturn.get() == NULL) 
        exit(-1);
      return toReturn;
    }
  
    virtual void PrintOptions(ostream& _out) {
      _out << "Sampler methods available:\n";
      for(typename map<string, boost::shared_ptr<SamplerMethod<CFG> > >::const_iterator S = ElementSet<SamplerMethod<CFG> >::ElementsBegin(); 
          S != ElementSet<SamplerMethod<CFG> >::ElementsEnd(); ++S) {
        _out << "  " << S->first << ":: ";
        S->second->PrintOptions(_out);
      }
    }

};

#endif

