
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

//////////////////////////////////////////////////////////////////////////////////////////
// Include OBPRM headers
#include "CollisionDetection.h"
#include "Clock_Class.h"
#include "GMSPolyhedron.h"

//Include node generation methods
///Note:: we will like to get rid of these and use the sampler class instead - sjacobs

/////
//#include  "BaseSampler.h"
#include  "UniformSamplers.h"
#include  "MedialAxisSamplers.h"
#include  "GaussianSamplers.h"
#include  "ObstacleBasedSamplers.h"

#include "util.h"
#include "CfgTypes.h"
#include "PMPL_Container_Base.h"
#include <sstream>
#include "SamplerMethod.h"

//////////////////////////////////////////////////////////////////////////////////////////
class Body;
class MPProblem;
class n_str_param;
class MultiBody;
class Input;
class Environment;
template <class CFG, class WEIGHT> class Roadmap;

//////////////////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class Sampler
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**This is the main generator class. It contains two vectors: all and selected.
 *all contains all of the different types of node generation methods.  selected
 *contains only those selected by the user.
 */
namespace pmpl_detail { //hide SamplerMethodList in pmpl_detail namespace
  typedef boost::mpl::list<
    UniformRandomSampler<CfgType>,
    UniformRandomFreeSampler<CfgType>,
    UniformRandomCollisionSampler<CfgType>,
    GaussRandomSampler<CfgType,true>,
    GaussRandomSampler<CfgType,false>,
    BridgeTestRandomFreeSampler<CfgType>,
    ObstacleBasedSampler<CfgType>,
    FreeMedialAxisSampler<CfgType>
    > SamplerMethodList;
}

template <class CFG>
class Sampler : private PMPL_Container_Base< SamplerMethod<CFG>,
                 pmpl_detail::SamplerMethodList>, public MPBaseObject{
			 
 private:
  typedef PMPL_Container_Base< SamplerMethod<CFG>, pmpl_detail::SamplerMethodList> SamplerContainer;
    
  public:
    typedef typename SamplerContainer::method_pointer SamplerPointer;
    
 public:

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Constructors and Destructor*/
  //@{

  ///Default Constructor.
  Sampler() { };
  Sampler(XMLNodeReader& in_Node, MPProblem* in_pProblem)
    : MPBaseObject(in_pProblem) { 
     //LOG_DEBUG_MSG("Sampler::Sampler(XMLNodeReader)");
     XMLNodeReader::childiterator citr;
    for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
	   
          //cout << "input string label to add = " << citr->getName() << endl;
      if(citr->getName() == "UniformRandomSampler") {
        SamplerMethod<CFG>* sp = new UniformRandomSampler<CfgType>(*citr, in_pProblem);
	// cout << "sp->GetObjectLabel = " << sp->GetObjectLabel() << endl;
        AddSamplingMethod(sp->GetObjectLabel(), SamplerPointer(sp));
      } else if(citr->getName() == "UniformRandomFreeSampler") {
        SamplerMethod<CFG>* sp = new UniformRandomFreeSampler<CfgType>(*citr, in_pProblem);
	//cout << "sp->GetObjectLabel = " << sp->GetObjectLabel() << endl;
        AddSamplingMethod(sp->GetObjectLabel(), SamplerPointer(sp));
      } else if(citr->getName() == "UniformRandomCollisionSampler") {
        SamplerMethod<CFG>* sp = new UniformRandomCollisionSampler<CfgType>(*citr, in_pProblem);
        AddSamplingMethod(sp->GetObjectLabel(), SamplerPointer(sp));
      } else if(citr->getName() == "GaussRandomFreeSampler") {
        SamplerMethod<CFG>* sp = new GaussRandomSampler<CfgType,true>(*citr, in_pProblem);
        AddSamplingMethod(sp->GetObjectLabel(), SamplerPointer(sp));
      } else if(citr->getName() == "GaussRandomCollisionSampler") {
        SamplerMethod<CFG>* sp = new GaussRandomSampler<CfgType,false>(*citr, in_pProblem);
        AddSamplingMethod(sp->GetObjectLabel(), SamplerPointer(sp));
      } else if(citr->getName() == "BridgeTestRandomFreeSampler") {
        SamplerMethod<CFG>* sp = new BridgeTestRandomFreeSampler<CfgType>(*citr, in_pProblem);
        AddSamplingMethod(sp->GetObjectLabel(), SamplerPointer(sp));
      } else if(citr->getName() == "ObstacleBasedSampler") {
        SamplerMethod<CFG>* sp = new ObstacleBasedSampler<CfgType>(*citr, in_pProblem);
        AddSamplingMethod(sp->GetObjectLabel(), SamplerPointer(sp));
     } else if(citr->getName() == "FreeMedialAxisSampler") {
        SamplerMethod<CFG>* sp = new FreeMedialAxisSampler<CfgType>(*citr, in_pProblem);
	AddSamplingMethod(sp->GetObjectLabel(), SamplerPointer(sp));
      }else {
        citr->warnUnknownNode();
      }
    }
  };
  ///Destructor.	
  ~Sampler() {}
  
  ///\name Access Methods
  ///
  //@{
  SamplerPointer GetSamplingMethod(string in_strLabel) {
    SamplerPointer to_return = 
                            SamplerContainer::GetMethod(in_strLabel);
    LOG_DEBUG_MSG("Sampler::GetSamplingMethod()");
    if(to_return.get() == NULL) {
      exit(-1);
    }
    return to_return;
  }

  void AddSamplingMethod(string in_strLabel, SamplerPointer in_ptr) {
	  LOG_DEBUG_MSG("Sampler::AddSamplingMethod()");
          SamplerContainer::AddMethod(in_strLabel, in_ptr);
  }
  //@}
  

  
  virtual void PrintOptions(ostream& out_os) { }

 

/////////////////////////////////////////////////////////////////////
//
//  definitions for class Sampler declarations
//
/////////////////////////////////////////////////////////////////////


template <typename InputIterator, typename OutputIterator>
OutputIterator
Sample(SamplerPointer _sp,Environment* env,Stat_Class& Stats,InputIterator _input_first, InputIterator _input_last,
	  int _attempts, OutputIterator _out) {
       return _Sample(_sp,env, Stats, _input_first, _input_last,_attempts, _out,
        typename SamplerContainer::MethodTypes_begin(),
        typename SamplerContainer::MethodTypes_end());
  }
  
template <typename OutputIterator>
OutputIterator
Sample(SamplerPointer _sp,Environment* env,Stat_Class& Stats,int _num_nodes, int _attempts, OutputIterator _out) {
	return _Sample(_sp,env, Stats,_num_nodes,_attempts, _out,
        typename SamplerContainer::MethodTypes_begin(),
        typename SamplerContainer::MethodTypes_end());
  }

/////////
//// Begin private boost functions
/////////
private: 

 template <typename InputIterator,typename OutputIterator,typename First, typename Last>
  OutputIterator
  _Sample( SamplerPointer _sp,Environment* env,Stat_Class& Stats,InputIterator _input_first, InputIterator _input_last,int _attempts,
    OutputIterator _out, First, Last) {
                typedef typename boost::mpl::deref<First>::type MethodType;
		if(MethodType* sampler = dynamic_cast<MethodType*>(_sp.get()))
		{
			return sampler->Sample(env, Stats,_input_first, _input_last,_out,_attempts);
			cout << "Sample::_Sample 1 set InputIterator- ";// << _sp->GetObjectLabel();
			return _out;
		}
		else 
		{
			typedef typename boost::mpl::next<First>::type Next;
			return _Sample(_sp,env, Stats,_input_first, _input_last, _attempts, _out, Next(), Last());  
		}
	}

  template <typename InputIterator, typename OutputIterator,typename Last>
  OutputIterator
  _Sample(SamplerPointer _sp, Environment* env,Stat_Class& Stats,InputIterator _input_first, InputIterator _input_last, int attempts,
    OutputIterator _out, Last, Last) {
    cerr << "ERROR, dynamic_cast of SampleMethod failed, method type not found!\n\n";
    exit(-1);
  }
  
   template <typename OutputIterator,typename First, typename Last>
  OutputIterator
  _Sample( SamplerPointer _sp,Environment* env,Stat_Class& Stats,int _num_nodes,int _attempts,
    OutputIterator _out, First, Last) {
                typedef typename boost::mpl::deref<First>::type MethodType;
		if(MethodType* sampler = dynamic_cast<MethodType*>(_sp.get()))
		{
			return sampler->Sample(env, Stats,_num_nodes,_out,_attempts);
			cout << "Sample::_Sample 1 set InputIterator- ";// << _sp->GetObjectLabel();
			return _out;
		}
		else 
		{
			typedef typename boost::mpl::next<First>::type Next;
			return _Sample( _sp,env, Stats,_num_nodes, _attempts, _out, Next(), Last());  
		}
	}

  template <typename OutputIterator,typename Last>
  OutputIterator
  _Sample(SamplerPointer _sp,Environment* env,Stat_Class& Stats, int _num_nodes, int attempts,
    OutputIterator _out, Last, Last) {
    cerr << "ERROR, dynamic_cast of SampleMethod failed, method type not found!\n\n";
    exit(-1);
  }

};




#endif


