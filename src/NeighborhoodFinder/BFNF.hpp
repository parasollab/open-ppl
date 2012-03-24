#ifndef _BRUTE_FORCE_NEIGHBORHOOD_FINDER_H_
#define _BRUTE_FORCE_NEIGHBORHOOD_FINDER_H_

#include "NeighborhoodFinderMethod.hpp"
#include "Environment.h"
#include <vector>
#include <functional>
using namespace std;


template<typename CFG, typename WEIGHT>
class BFNF: public NeighborhoodFinderMethod {

public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  BFNF(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    NeighborhoodFinderMethod(in_Node, in_pProblem) {
}
  
  BFNF(shared_ptr<DistanceMetricMethod>_dmm,string _label="",MPProblem* in_pProblem=NULL): NeighborhoodFinderMethod(_dmm,_label, in_pProblem) {}

  virtual ~BFNF() {}

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
  IncrementNumQueries();
  //TO DO NOTE: A temporary fix to support parallel runtime. The problem here is that is the way
  // we pass pointer around which is a bit ugly with parallelism. In this particular case
  // the pointer to GetMPProblem became invalid because of the way BFNF is called from
  //Connector, thus call to timing stats below seg fault. One fix is to call BFNF(_node, _problem)
  //constructor and this will be done when parallel code supports all NF. What this means is 
  // that I can not just support BFNF by itself.
  #ifndef _PARALLEL
  StartTotalTime();
  StartQueryTime();
  #endif

  Environment* _env = _rmp->GetEnvironment();
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;

  int max_index = 0;
  double max_value = MAX_DIST;
  vector< pair< VID, double > > closest(k, make_pair(-999, max_value));

  // now go through all kp and find closest k
  int count = 0;
  for(InputIterator V1 = _input_first; V1 != _input_last; ++V1) {
    count++;
    CFG v1 = pmpl_detail::GetCfg<InputIterator>(pMap)(V1);
    
    
    if(v1 == _cfg)
      continue; //don't connect same

    double dist = dmm->Distance(_env, _cfg, v1);
   
    if(dist < closest[max_index].second) { 
      
      closest[max_index] = make_pair(*V1, dist);
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
 
  sort(closest.begin(), closest.end(), compare_second<VID, double>());
    
  // now add VIDs from closest to output
  for (size_t p = 0; p < closest.size(); p++) {
    if (closest[p].first != VID(-999)) {
      *_out = closest[p].first;
      ++_out;
    }
  }
  #ifndef _PARALLEL
  EndQueryTime();
  EndTotalTime();
  #endif
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
  //return KClosest(_rmp, _rmp->m_pRoadmap->begin(), _rmp->m_pRoadmap->end(), _cfg, k, _out);
  return KClosest(_rmp, _rmp->m_pRoadmap->descriptor_begin(), _rmp->m_pRoadmap->descriptor_end(), _cfg, k, _out);
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
    CFG v1 = pmpl_detail::GetCfg<InputIterator>(pMap)(V1);
    for(V2 = _in2_first; V2 != _in2_last; ++V2) {
      //marcom/08nov03 check if results in other functions is same                      
      if(*V1 == *V2)
        continue; //don't connect same                                                  
    CFG v2 = pmpl_detail::GetCfg<InputIterator>(pMap)(V2);
      double dist = dmm->Distance(_env, v1, v2);
      if(dist < kp[max_index].second) {
        kp[max_index] = make_pair(make_pair(*V1,*V2),dist);
        max_value = dist;
      
        //search for new max_index (faster O(k) than sort O(k log k) )                  
        for (size_t p = 0; p < kp.size(); ++p) {
          if (max_value < kp[p].second) {
            max_value = kp[p].second;
            max_index = p;
          }
        }
      }
    }//endfor c2                                                                  
    kall.insert(kall.end(),kp.begin(),kp.end());
  }//endfor c1                                                                    
 
  sort(kall.begin(), kall.end(), compare_second<pair<VID, VID>, double>());
  
  for (int p = 0; p < k; ++p) {
    if (kall[p].first.first != (VID)-999 && kall[p].first.second != (VID)-999){
      *_out = kall[p].first;
      ++_out;
    }
  }
  return _out;
}


#endif //end #ifndef _BRUTE_FORCE_NEIGHBORHOOD_FINDER_H_
