/**
 * @file DistanceMetrics.h
 *
 * @author Daniel Vallejo
 * @date 8/21/1998
 */

////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DISTANCEMETRICS_H
#define DISTANCEMETRICS_H


#include <boost/mpl/list.hpp>
#include "MPUtils.h"
#include "EuclideanDistance.h"
#include "ScaledEuclideanDistance.h"
#include "MinkowskiDistance.h"
#include "ManhattanDistance.h"
#include "CenterOfMassDistance.h"
#include "RSMDDistance.h"
#include "LPSweptDistance.h"
#include "BinaryLPSweptDistance.h"
#include "KnotTheoryDistance.h"
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
#include "ReachableDistance.h"
#endif
#include "DistanceMetricMethod.h"


//////////////////////////////////////////////////////////////////////////////////////////


namespace pmpl_detail { //hide DistanceMetricMethodList in pmpl_detail namespace
  typedef boost::mpl::list<
    EuclideanDistance,
    ScaledEuclideanDistance,
    MinkowskiDistance,
    ManhattanDistance,
    CenterOfMassDistance,
    RMSDDistance,
    LPSweptDistance,
    BinaryLPSweptDistance,
    #if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
    ReachableDistance, 
    #endif
    KnotTheoryDistance
    > DistanceMetricMethodList;
}


class DistanceMetric : private ElementSet<DistanceMetricMethod>, public MPBaseObject {
 public:
   typedef ElementSet<DistanceMetricMethod>::MethodPointer DistanceMetricPointer;
   
   template<typename MethodList>
   DistanceMetric() : ElementSet<DistanceMetricMethod>(MethodList()) {}

   DistanceMetric() : ElementSet<DistanceMetricMethod>(pmpl_detail::DistanceMetricMethodList()) {}
  
   template <typename MethodList>
   DistanceMetric(XMLNodeReader& _node, MPProblem* _problem, MethodList const&)
     : ElementSet<DistanceMetricMethod>(MethodList()), MPBaseObject(_problem) {
     for(XMLNodeReader::childiterator citr = _node.children_begin(); citr!= _node.children_end(); ++citr)
       if(!AddElement(citr->getName(), *citr, _problem))
        citr->warnUnknownNode();
   }
   
   DistanceMetric(XMLNodeReader& _node, MPProblem* _problem)
     : ElementSet<DistanceMetricMethod>(pmpl_detail::DistanceMetricMethodList()), MPBaseObject(_problem) {
     for(XMLNodeReader::childiterator citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
       if(!AddElement(citr->getName(), *citr, _problem))
        citr->warnUnknownNode();
    }
   }
   virtual ~DistanceMetric();

   DistanceMetricPointer GetMethod(string _label);
   
   void PrintOptions(ostream& _os) const;

};

#endif
