#ifndef _BRUTE_FORCE_FIND_NEIGHBORHOOD_FINDER_H_
#define _BRUTE_FORCE_FIND_NEIGHBORHOOD_FINDER_H_

#include "NeighborhoodFinderMethod.hpp"
#include "OBPRMDef.h"
#include "DistanceMetrics.h"
#include "util.h"
#include "MPProblem.h"

#include "Clock_Class.h"
#include <vector>
#include <functional>

class Cfg;
class MultiBody;
class Input;
class Environment;
class n_str_param;
class MPProblem;
template <class CFG, class WEIGHT> class Roadmap;

using namespace std;
/**Compare two distances in DIST_TYPE instances.
 *return (_cc1.second < _cc2.second)
 */
template <class T>
class T_DIST_Compare2 : public binary_function<const pair<T,double>,
              const pair<T,double>,
              bool> {
 public:
  bool operator()(const pair<T,double> _cc1,
      const pair<T,double> _cc2) {
    return (_cc1.second < _cc2.second);
  }
};

template<typename CFG, typename WEIGHT>
class BFFNF: public NeighborhoodFinderMethod {

public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  BFFNF(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    NeighborhoodFinderMethod(ParseLabelXML(in_Node),in_Node,in_pProblem) {
    m_scale = in_Node.numberXMLParameter("k_2", true, double(0.0), double(0.0), double(100.0), "K value for BFFNF");
    string dm2_label = in_Node.stringXMLParameter(string("dm2_method"),true,string(""),string("Distance Metric Method the second one"));
    dmm2 = in_pProblem->GetDistanceMetric()->GetDMMethod(dm2_label);
    
      nf1 = new BFNF<CFG,WEIGHT>(dmm,"label");
    nf2 = new BFNF<CFG,WEIGHT>(dmm2, "label");
  }

  BFFNF(shared_ptr<DistanceMetricMethod> _dmm,shared_ptr<DistanceMetricMethod>_dmm2) :
    NeighborhoodFinderMethod() {
//cout<<"initiliazing other constructor "<<endl;
    
    dmm= _dmm ;
    dmm2=_dmm2;
}


  virtual const std::string GetName () const {
    return BFFNF::GetClassName();
  }
  static const std::string GetClassName() {
    return "BFFNF";
  }
  virtual void PrintOptions(std::ostream& out_os) const {
    out_os << this->GetClassName() << std::endl;
  }


  template <typename InputIterator, typename OutputIterator>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v, int k,
    OutputIterator _out);
  
  // do the work here, and have the function above obtain the CFG and call this one
  template <typename InputIterator, typename OutputIterator>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, CFG _cfg, int k,
    OutputIterator _out);
  
  
  // KClosest that operate over the entire roadmap to find the kclosest to a VID or CFG
  //
  // NOTE: These are the prefered methods for kClosest computations
  template <typename OutputIterator>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    VID _v, int k, OutputIterator _out);
  
  template <typename OutputIterator>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    CFG _cfg, int k, OutputIterator _out);
  

  // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
  // represent the kclosest pairs of VIDs between the two ranges.
  template <typename InputIterator, typename OutputIterator>
  OutputIterator
  KClosestPairs( Roadmap<CFG,WEIGHT>* _rmp,
    InputIterator _in1_first, InputIterator _in1_last, 
    InputIterator _in2_first, InputIterator _in2_last, 
    int k, OutputIterator _out) {};


private:
 shared_ptr<DistanceMetricMethod> dmm2;
  BFNF<CFG,WEIGHT> *nf1;
  BFNF<CFG,WEIGHT> *nf2;
  double m_scale;
};

template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
BFFNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v,
    int k, OutputIterator _out) {


}

template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
BFFNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  InputIterator _input_first, InputIterator _input_last, CFG _cfg, 
  int k, OutputIterator _out) {


  
}

template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator
BFFNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp,
  VID _v, int k, OutputIterator _out) {
cout<<"calling the 2nd one"<<endl;
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
 // CFG _v_cfg = pMap->find_vertex(_v).property();
  CFG _v_cfg = (*(pMap->find_vertex(_v))).property();
  return KClosest(_rmp, _v_cfg, k, _out);
  
 ;
}

template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator
BFFNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp,
  CFG _cfg, int k, OutputIterator _out) {

    vector<VID> closest;
 
   // typename vector<VID>::iterator myint;
   nf1->KClosest(_rmp,_cfg,m_scale,back_insert_iterator<vector<VID> >(closest));
nf2->KClosest(_rmp,closest.begin(),closest.end(),_cfg,k,_out);

}

#endif //end #ifndef _BRUTE_FORCE_FIND_NEIGHBORHOOD_FINDER_H_
