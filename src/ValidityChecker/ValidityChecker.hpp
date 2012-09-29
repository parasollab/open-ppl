#ifndef VALIDITYCHECKER_H
#define VALIDITYCHECKER_H

//////////////////////////////////////////////////////////////////////////////////////////

#include "SurfaceValidity.h"
#include "AlwaysTrueValidity.h"
#include "NodeClearanceValidity.h"
#include "MedialAxisClearanceValidity.h"
#include "ObstacleClearanceValidity.h"
#include "ComposeVC.hpp"
#include "NegateValidity.hpp"
#include "CollisionDetectionValidity.hpp"

//////////////////////////////////////////////////////////////////////////////////////////

namespace pmpl_detail { //hide ValidityCheckerMethodList in pmpl_detail namespace
  typedef boost::mpl::list<
#ifdef PMPCfgSurface
    SurfaceValidity,
#endif
    AlwaysTrueValidity,
    NodeClearanceValidity,
    MedialAxisClearanceValidity,
    ObstacleClearanceValidity,
    ComposeValidity,
    NegateValidity,
    CollisionDetectionValidity
    > ValidityCheckerMethodList;
}

class ValidityChecker : private ElementSet<ValidityCheckerMethod>, public MPBaseObject {
 public:
   typedef ElementSet<ValidityCheckerMethod>::MethodPointer ValidityCheckerPointer;
   
   template<typename MethodList>
   ValidityChecker() : ElementSet<ValidityCheckerMethod>(MethodList()) {}

   ValidityChecker() : ElementSet<ValidityCheckerMethod>(pmpl_detail::ValidityCheckerMethodList()) {}
  
   template <typename MethodList>
   ValidityChecker(XMLNodeReader& _node, MPProblem* _problem, MethodList const&)
     : ElementSet<ValidityCheckerMethod>(MethodList()), MPBaseObject(_problem) {
     for(XMLNodeReader::childiterator citr = _node.children_begin(); citr!= _node.children_end(); ++citr)
       if(!AddElement(citr->getName(), *citr, _problem))
        citr->warnUnknownNode();
   }
   
   ValidityChecker(XMLNodeReader& _node, MPProblem* _problem)
     : ElementSet<ValidityCheckerMethod>(pmpl_detail::ValidityCheckerMethodList()), MPBaseObject(_problem) {
     for(XMLNodeReader::childiterator citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
       if(!AddElement(citr->getName(), *citr, _problem))
        citr->warnUnknownNode();
    }
   }
   virtual ~ValidityChecker();

   ValidityCheckerPointer GetMethod(string _label);

   void AddMethod(string const& _label, ValidityCheckerPointer _dmm);
  
   virtual void SetMPProblem(MPProblem* _mp);

   void PrintOptions(ostream& _os) const;

   vector<cd_predefined> GetSelectedCDTypes() const;
};

#endif // #ifndef VALIDITYCHECKER_H
