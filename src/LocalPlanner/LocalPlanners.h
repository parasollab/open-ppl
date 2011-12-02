#ifndef LOCALPLANNERS_H_
#define LOCALPLANNERS_H_

#include <boost/mpl/list.hpp>
#include "MPUtils.h"
#include "CollisionDetection.h"
#include "ValidityChecker.hpp"
#include "Weight.h"
#include "LocalPlannerMethod.h"

#include "StraightLine.h"
#include "RotateAtS.h"
#include "TransformAtS.h"
#include "AStar.h"
//#include "ApproxSpheres.h"

#ifdef _PARALLEL
#include "runtime.h"
#endif

#include "CfgTypes.h"

class DistanceMetricMethod;

template <class CFG, class WEIGHT>
struct LPOutput {
  vector<CFG> path;          // Path found by local planner.
  pair<WEIGHT, WEIGHT> edge; // Contains weights of edges defined in path.
  vector< pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > > savedEdge;  // Failed Edge: savedEdge.second -> position failed.
};

// Hide LocalPlannerMethod List in pmpl_detail namespace
namespace pmpl_detail {
  typedef boost::mpl::list<
    StraightLine<CfgType,WeightType>,
    RotateAtS<CfgType,WeightType>,
    TransformAtS<CfgType,WeightType>,
    AStar<CfgType,WeightType>
    > LocalPlannerMethodList;
}

template <class CFG, class WEIGHT>
#ifdef _PARALLEL
class LocalPlanners : private ElementSet<LocalPlannerMethod<CFG,WEIGHT> >, public MPBaseObject, public stapl::p_object {
#else
class LocalPlanners : private ElementSet<LocalPlannerMethod<CFG,WEIGHT> >, public MPBaseObject {
#endif
 public:
	typedef typename ElementSet<LocalPlannerMethod<CFG,WEIGHT> >::MethodPointer LocalPlannerPointer;

  template <typename MethodList>
	LocalPlanners() : ElementSet<LocalPlannerMethod<CFG,WEIGHT> >(MethodList()) {}
	LocalPlanners() : ElementSet<LocalPlannerMethod<CFG,WEIGHT> >(pmpl_detail::LocalPlannerMethodList()) {}
  virtual ~LocalPlanners() {};

  template <typename MethodList>
  LocalPlanners(XMLNodeReader& in_Node, MPProblem* in_pProblem, MethodList const&)
  : ElementSet<LocalPlannerMethod<CFG,WEIGHT> >(MethodList()), MPBaseObject(in_pProblem) {
    for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr)
      if(!ElementSet<LocalPlannerMethod<CFG,WEIGHT> >::AddElement(citr->getName(), *citr, in_pProblem))
        citr->warnUnknownNode();
    PrintOptions(cout);
  }

  LocalPlanners(XMLNodeReader& in_Node, MPProblem* in_pProblem)
  : ElementSet<LocalPlannerMethod<CFG,WEIGHT> >(pmpl_detail::LocalPlannerMethodList()), MPBaseObject(in_pProblem) {
    for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr)
      if(!ElementSet<LocalPlannerMethod<CFG,WEIGHT> >::AddElement(citr->getName(), *citr, in_pProblem))
        citr->warnUnknownNode();
    PrintOptions(cout);
  }

  LocalPlannerPointer GetLocalPlannerMethod(string in_strLabel) {
    LocalPlannerPointer to_return = ElementSet<LocalPlannerMethod<CFG,WEIGHT> >::GetElement(in_strLabel);
      if ( to_return.get() == NULL ) {
        cout << "LocalPlanners::GetLocalPlannerMethod::ERROR: could not find " << in_strLabel << endl;
        exit(-1);
      }
    return to_return;
  }

  void AddLocalPlannerMethod(string in_strLabel, LocalPlannerPointer in_ptr) {
    ElementSet<LocalPlannerMethod<CFG,WEIGHT> >::AddElement(in_strLabel, in_ptr);
  }

  virtual void PrintOptions(ostream& out_os) { 
    out_os << "  Local Planners" << endl;
    for ( typename std::map<string,shared_ptr<LocalPlannerMethod<CFG,WEIGHT> > >::const_iterator M = this->ElementsBegin(); 
          M != this->ElementsEnd(); 
          ++M )
      out_os <<"\t\"" << M->first << "\" (" << M->second->GetName() << ")" << endl;
  }

 private:
  CDInfo cdInfo;

  // Swept Volume Friend Classes
  friend class LPSweptDistance;
  friend class BinaryLPSweptDistance;
};

#endif
