#ifndef _BRUTE_FORCE_NEIGHBORHOOD_FINDER_H_
#define _BRUTE_FORCE_NEIGHBORHOOD_FINDER_H_

#include "NeighborhoodFinderMethod.hpp"
#include "OBPRMDef.h"
#include "DistanceMetrics.h"
#include "util.h"

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
class T_DIST_Compare : public binary_function<const pair<T,double>,
              const pair<T,double>,
              bool> {
 public:
  bool operator()(const pair<T,double> _cc1,
      const pair<T,double> _cc2) {
    return (_cc1.second < _cc2.second);
  }
};


namespace pmpl_detail
{ 
  //helper function to call dereferece on an iterator whose value_type is VID and convert to CfgType
  template <typename T>
  struct get_cfg
    : public unary_function<T, CfgType&>
  {
    const RoadmapGraph<CfgType, WeightType>* pMap;
    
    get_cfg(const RoadmapGraph<CfgType, WeightType>* _pMap) : pMap(_pMap) {}
    ~get_cfg() {}

    CfgType operator()(const T& t) const
    {
      return (pMap->find_vertex(*t))->property();
    }
  };
  //specialization for a roadmap graph iterator, calls property()
  template <>
  struct get_cfg<RoadmapGraph<CfgType, WeightType>::VI>
    : public unary_function<RoadmapGraph<CfgType, WeightType>::VI, CfgType&>
  {
    const RoadmapGraph<CfgType, WeightType>* pMap;
    
    get_cfg(const RoadmapGraph<CfgType, WeightType>* _pMap) : pMap(_pMap) {}
    ~get_cfg() {}
    
    CfgType operator()(const RoadmapGraph<CfgType, WeightType>::VI& t) const
    {
      return t->property();
    }
  };

  //helper function to call dereferece on an iterator whose value_type is VID
  //needed to get around the fact that a roadmap graph iterator requires an extra descriptor() call
  template <typename T>
  struct get_vid
    : public unary_function<T, RoadmapGraph<CfgType, WeightType>::vertex_descriptor>
  {
    RoadmapGraph<CfgType, WeightType>::vertex_descriptor operator()(const T& t) const
    {
      return *t;
    }
  };
  //specialization for a roadmap graph iterator, calls descriptor()
  template <>
  struct get_vid<RoadmapGraph<CfgType, WeightType>::VI>
    : public unary_function<RoadmapGraph<CfgType, WeightType>::VI, RoadmapGraph<CfgType, WeightType>::vertex_descriptor>
  {
    RoadmapGraph<CfgType, WeightType>::vertex_descriptor operator()(const RoadmapGraph<CfgType, WeightType>::VI& t) const
    {
      return t->descriptor();
    }
  };
}


template<typename CFG, typename WEIGHT>
class BFNF: public NeighborhoodFinderMethod {

public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  BFNF(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    NeighborhoodFinderMethod(ParseLabelXML(in_Node), in_Node, in_pProblem) {
}

  BFNF(shared_ptr<DistanceMetricMethod>_dmm) :
    NeighborhoodFinderMethod() {
    dmm = _dmm;
  }

  virtual const std::string GetName () const {
    return BFNF::GetClassName();
  }
  static const std::string GetClassName() {
    return "BFNF";
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
    int k, OutputIterator _out);

};

template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
BFNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v,
    int k, OutputIterator _out) {
  return KClosest(_rmp, _input_first, _input_last, (_rmp->m_pRoadmap->find_vertex(_v))->property(), k, _out);
}

template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
BFNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  InputIterator _input_first, InputIterator _input_last, CFG _cfg, 
  int k, OutputIterator _out) {
  //cout << "BFNF::KClosest - pair iterator & CFG: k = "<< k << endl << flush;
  IncrementNumQueries();
  StartTotalTime();
  StartQueryTime();

  Environment* _env = _rmp->GetEnvironment();
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;

  int max_index = 0;
  double max_value = MAX_DIST;
  vector< pair< VID, double > > closest(k, make_pair(-999, max_value));

  // now go through all kp and find closest k
  InputIterator V1;
  int count = 0;
  for(V1 = _input_first; V1 != _input_last; ++V1) {
    count++;
    CFG v1 = pmpl_detail::get_cfg<InputIterator>(pMap)(V1);
    
    if(v1 == _cfg)
      continue; //don't connect same

    double dist = dmm->Distance(_env, _cfg, v1);
    
    if(dist < closest[max_index].second) { 
      closest[max_index] = make_pair(pmpl_detail::get_vid<InputIterator>()(V1), dist);
      max_value = dist;
  
      //search for new max_index (faster O(k) than sort O(k log k) )
      for (size_t p = 0; p < closest.size(); ++p) {
        if (max_value < closest[p].second) {
          max_value = closest[p].second;
          max_index = p;
        }
      }
    }
  }
 
  sort(closest.begin(), closest.end(), T_DIST_Compare<VID>());
    
  // now add VIDs from closest to output
  for (size_t p = 0; p < closest.size(); p++) {
    if (closest[p].first != VID(-999)) {
      *_out = closest[p].first;
      ++_out;
    }
  }
  
  EndQueryTime();
  EndTotalTime();
  return _out;
}


template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator
BFNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  VID _v, int k, OutputIterator _out) {
  return KClosest(_rmp, (_rmp->m_pRoadmap->find_vertex(_v))->property(), k, _out);
}

template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator
BFNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  CFG _cfg, int k, OutputIterator _out) {
  return KClosest(_rmp, _rmp->m_pRoadmap->begin(), _rmp->m_pRoadmap->end(), _cfg, k, _out);
}


template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
BFNF<CFG,WEIGHT>::
KClosestPairs( Roadmap<CFG,WEIGHT>* _rmp,
  InputIterator _in1_first, InputIterator _in1_last, 
  InputIterator _in2_first, InputIterator _in2_last, 
  int k, OutputIterator _out) {
   
  Environment* _env = _rmp->GetEnvironment();
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  int max_index = 0;
  double max_value = MAX_DIST;
  vector< pair< pair< VID, VID >, double > > kall;
 
  // now go through all kp and find closest k                                     
  InputIterator V1, V2;
  for(V1 = _in1_first; V1 != _in1_last; ++V1) {
    // initialize w/ k elements each with huge distance...                        
    vector<pair<pair<VID,VID>,double> > kp(k, make_pair(make_pair(-999,-999),
    max_value));
    CFG v1 = (*(pMap->find_vertex(*V1))).property();
    for(V2 = _in2_first; V2 != _in2_last; ++V2) {
      //marcom/08nov03 check if results in other functions is same                      
      if(*V1 == *V2)
        continue; //don't connect same                                                  
    
      double dist = dmm->Distance(_env, v1, (*(pMap->find_vertex(*V2))).property());
      if(dist < kp[max_index].second) {
        kp[max_index] = make_pair(make_pair(*V1,*V2),dist);
        max_value = dist;
      
        //search for new max_index (faster O(k) than sort O(k log k) )                  
        for (int p = 0; p < kp.size(); ++p) {
          if (max_value < kp[p].second) {
            max_value = kp[p].second;
            max_index = p;
          }
        }
      }
    }//endfor c2                                                                  
    kall.insert(kall.end(),kp.begin(),kp.end());
  }//endfor c1                                                                    
 
  sort(kall.begin(), kall.end(), DIST_Compare<VID>());
  
  for (int p = 0; p < k; ++p) {
    if (kall[p].first.first != -999 && kall[p].first.second != -999){
      *_out = kall[p].first;
      ++_out;
    }
  }
  return _out;
}


#endif //end #ifndef _BRUTE_FORCE_NEIGHBORHOOD_FINDER_H_
