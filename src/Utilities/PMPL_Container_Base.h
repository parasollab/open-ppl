#ifndef _PMPL_Container_Base_H_
#define _PMPL_Container_Base_H_

//defines basic method container class to derive from
//  (for classes like DistanceMetric, LocalPlanner, NeighborhoodFinder,NodeGenerator etc)
//
//derived class must specify the Method type and MethodTypeList it contains
//  e.g., NeighborhoodFinder: Method = NeighborhoodFinderMethod
//                            MethodTypeList = boost::mpl::list<BruteForce,ANN,...>
//  e.g., LocalPlanner: Method = LocalPlannerMethod
//                      MethodTypeList = boost::mpl::list<Straightline,RotateAtS,...>
//  e.g., GenerateMapNodes: Method = NodeGenerationMethod
//                      MethodTypeList = boost::mpl::list<UniformSampler,ObstacleBasedSampler,...>
//
//MethodTypeList stores the available method types in 1 place 
//  used in dynamic_cast generation for dispatching 


//stl includes
#include <map>
#include <vector>
#include <string>
#include <iostream>

using std::map;
using std::vector;
using std::string;
using std::cout;
using std::cerr;


//boost includes
#include "boost/shared_ptr.hpp"

#include "boost/mpl/list.hpp"
#include "boost/mpl/sort.hpp"
#include "boost/type_traits/is_base_of.hpp"
#include "boost/mpl/begin.hpp"
#include "boost/mpl/end.hpp"
#include "boost/mpl/next_prior.hpp"

using boost::shared_ptr;


//base pmpl container class that holds a list available method objects
template <typename Method, typename TypeList>
class PMPL_Container_Base
{
 public:
  typedef shared_ptr<Method> method_pointer;
	PMPL_Container_Base() {} 
	virtual ~PMPL_Container_Base() {}

	void AddMethod(string name, shared_ptr<Method> m)
	{
		if(methods.find(name) != methods.end())
			cerr << "\nWarning, method list already has a method pointer associated with \"" << name << "\", not added\n";
		else
			methods[name] = m;
	}
	shared_ptr<Method> GetMethod(string name) { 
	      LOG_DEBUG_MSG("PMPL_Container_Base::GetMethod()");
              return methods[name]; 
	}
	
 protected:
	//MethodTypes: sorted list of method types class will use, puts most derived first
	typedef typename boost::mpl::sort<TypeList, boost::is_base_of<boost::mpl::_2, boost::mpl::_1> >::type MethodTypes;
	
	//MethodTypes_begin, MethodTypes_end: used for iteration over type lists
	typedef typename boost::mpl::begin<MethodTypes>::type MethodTypes_begin;
	typedef typename boost::mpl::end<MethodTypes>::type MethodTypes_end;
	
	map<string, shared_ptr<Method> > methods;
	//shared_ptr used instead of regular pointer to handle memory 
	//  allocation/release automatically
};

#endif
