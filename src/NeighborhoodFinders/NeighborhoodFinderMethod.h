#ifndef NEIGHBORHOODFINDERMETHOD_H_
#define NEIGHBORHOODFINDERMETHOD_H_

#include <boost/mpl/list.hpp>
#include <boost/mpl/for_each.hpp>
#include <string>
#include <iostream>
#include "Utilities/MPUtils.h"

//these methods allow for dynamic dispatch for the templated functions within
//these neighborhood finder classes
namespace pmpl_detail {
  
  template<typename NF, typename RDMP, typename I, typename CFG, typename O>
      struct VirtualKClosest{
        public:
          VirtualKClosest(NF* _v, RDMP* _r, I _f, I _l, CFG _c, size_t _k, O _o) : 
            m_memory(_v), m_rdmp(_r), m_first(_f), m_last(_l), m_cfg(_c), m_k(_k), m_output(_o){
            }

          template<typename T>
            void operator()(T& _t) {
              T* tptr = dynamic_cast<T*>(m_memory);
              if(tptr != NULL){
                tptr->KClosest(m_rdmp, m_first, m_last, m_cfg, m_k, m_output);
              }
            }
        private:
          NF* m_memory;
          RDMP* m_rdmp;
          I m_first, m_last;
          CFG m_cfg;
          size_t m_k;
          O m_output;
      };
  
  template<typename NF, typename RDMP, typename I, typename O>
      struct VirtualKClosestPairs{
        public:
          VirtualKClosestPairs(NF* _v, RDMP* _r, I _f1, I _l1, I _f2, I _l2, size_t _k, O _o) : 
            m_memory(_v), m_rdmp(_r), m_first1(_f1), m_last1(_l1), m_first2(_f2), m_last2(_l2), m_k(_k), m_output(_o){
            }

          template<typename T>
            void operator()(T& _t) {
              T* tptr = dynamic_cast<T*>(m_memory);
              if(tptr != NULL){
                tptr->KClosestPairs(m_rdmp, m_first1, m_last1, m_first2, m_last2, m_k, m_output);
              }
            }
        private:
          NF* m_memory;
          RDMP* m_rdmp;
          I m_first1, m_last1, m_first2, m_last2;
          size_t m_k;
          O m_output;
      };
}

template<class MPTraits>
class NeighborhoodFinderMethod : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;

    NeighborhoodFinderMethod(MPProblemType* _problem, string _dmLabel = "", string _label = "");
    NeighborhoodFinderMethod(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~NeighborhoodFinderMethod() {}

    typename MPProblemType::DistanceMetricPointer GetDMMethod() const;

    virtual void PrintOptions(ostream& _os) const{
      _os << "Name: " << this->GetName() << " "
        << "dmMethod: " << m_dmLabel << " ";
    }

    double GetTotalTime() const;
    double GetQueryTime() const;
    double GetConstructionTime() const;
    size_t GetNumQueries() const;
    
    ////////////////////////////////////////////////////////////////////////////////////
    //   Neighborhood Finder Methods. 
    //
    //   KClosest and KClosestPairs need to be 
    //   implemented in base classes
    ////////////////////////////////////////////////////////////////////////////////////
    
    template<typename InputIterator, typename OutputIterator>
      OutputIterator KClosest(RoadmapType* _rmp, 
          InputIterator _first, InputIterator _last, VID _v, size_t _k, OutputIterator _out) {
        return KClosest(_rmp, _first, _last, _rmp->GetGraph()->GetCfg(_v), _k, _out);
      }

    // KClosest that operate over the entire roadmap to find the _kclosest to a VID or CFG
    // NOTE: These are the prefered methods for _kClosest computations
    template<typename OutputIterator>
      OutputIterator KClosest(RoadmapType* _rmp, VID _v, size_t _k, OutputIterator _out){
        return KClosest(_rmp, _rmp->Graph()->GetCfg(_v), _k, _out);
      }

    template<typename OutputIterator>
      OutputIterator KClosest(RoadmapType* _rmp, CfgType _cfg, size_t _k, OutputIterator _out){
        m_fromRDMPVersion = true;
        return KClosest(_rmp, _rmp->GetGraph()->descriptor_begin(), _rmp->GetGraph()->descriptor_end(), _cfg, _k, _out);
      }

    // do the work here, and have the function above obtain the CFG and call this one
    template<typename InputIterator, typename OutputIterator>
      OutputIterator KClosest(RoadmapType* _rmp, 
          InputIterator _first, InputIterator _last, CfgType _cfg, size_t _k, OutputIterator _out){
        typedef typename MPTraits::NeighborhoodFinderMethodList MethodList;
        boost::mpl::for_each<MethodList>(pmpl_detail::VirtualKClosest<
            NeighborhoodFinderMethod, RoadmapType, 
            InputIterator, CfgType, OutputIterator>(this, _rmp, _first, _last, _cfg, _k, _out));
        return _out;
      }

    // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
    // represent the _kclosest pairs of VIDs between the two ranges.
    template<typename InputIterator, typename OutputIterator>
      OutputIterator KClosestPairs(RoadmapType* _rmp,
          InputIterator _first1, InputIterator _last1, 
          InputIterator _first2, InputIterator _last2, 
          size_t _k, OutputIterator _out){
        typedef typename MPTraits::NeighborhoodFinderMethodList MethodList;
        boost::mpl::for_each<MethodList>(pmpl_detail::VirtualKClosestPairs<
            NeighborhoodFinderMethod, RoadmapType, 
            InputIterator, OutputIterator>(this, _rmp, _first1, _last1, _first2, _last2, _k, _out));
        return _out;
      }
  
  protected:
    void StartTotalTime();
    void EndTotalTime();
    void StartQueryTime();
    void EndQueryTime();
    void StartConstructionTime();
    void EndConstructionTime();
    void IncrementNumQueries();

    string m_dmLabel;
    bool m_fromRDMPVersion;
};

template<class MPTraits>
NeighborhoodFinderMethod<MPTraits>::NeighborhoodFinderMethod(MPProblemType* _problem, string _dmLabel, string _label) 
  : MPBaseObject<MPTraits>(_problem, _label), m_dmLabel(_dmLabel), m_fromRDMPVersion(false) {}

template<class MPTraits>
NeighborhoodFinderMethod<MPTraits>::NeighborhoodFinderMethod(MPProblemType* _problem, XMLNodeReader& _node) 
  : MPBaseObject<MPTraits>(_problem, _node), m_fromRDMPVersion(false){ 
    m_dmLabel = _node.stringXMLParameter("dmLabel", true, "default", "Distance Metric Method");
  }

template<class MPTraits>
typename MPTraits::MPProblemType::DistanceMetricPointer 
NeighborhoodFinderMethod<MPTraits>::GetDMMethod() const {
  return this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
}

template<class MPTraits>
double
NeighborhoodFinderMethod<MPTraits>::GetTotalTime() const{
  return this->GetMPProblem()->GetStatClass()->GetSeconds(this->GetNameAndLabel()+"::Total");
}

template<class MPTraits>
double 
NeighborhoodFinderMethod<MPTraits>::GetQueryTime() const{
  return this->GetMPProblem()->GetStatClass()->GetSeconds(this->GetNameAndLabel()+"::Query");
}

template<class MPTraits>
double 
NeighborhoodFinderMethod<MPTraits>::GetConstructionTime() const{
  return this->GetMPProblem()->GetStatClass()->GetSeconds(this->GetNameAndLabel()+"::Construction");
}

template<class MPTraits>
size_t 
NeighborhoodFinderMethod<MPTraits>::GetNumQueries() const{
  return this->GetMPProblem()->GetStatClass()->GetNFStat(this->GetNameAndLabel()+"::NumQueries");
}

template<class MPTraits>
void 
NeighborhoodFinderMethod<MPTraits>::StartTotalTime(){  
  this->GetMPProblem()->GetStatClass()->StartClock(this->GetNameAndLabel()+"::Total");    
}

template<class MPTraits>
void 
NeighborhoodFinderMethod<MPTraits>::EndTotalTime(){
  this->GetMPProblem()->GetStatClass()->StopClock(this->GetNameAndLabel()+"::Total");
}

template<class MPTraits>
void 
NeighborhoodFinderMethod<MPTraits>::StartQueryTime(){
  this->GetMPProblem()->GetStatClass()->StartClock(this->GetNameAndLabel()+"::Query");    
}

template<class MPTraits>
void 
NeighborhoodFinderMethod<MPTraits>::EndQueryTime(){
  this->GetMPProblem()->GetStatClass()->StopClock(this->GetNameAndLabel()+"::Query");
}

template<class MPTraits>
void 
NeighborhoodFinderMethod<MPTraits>::StartConstructionTime(){
  this->GetMPProblem()->GetStatClass()->StartClock(this->GetNameAndLabel()+"::Construction");    
}

template<class MPTraits>
void 
NeighborhoodFinderMethod<MPTraits>::EndConstructionTime(){
  this->GetMPProblem()->GetStatClass()->StopClock(this->GetNameAndLabel()+"::Construction");
}

template<class MPTraits>
void 
NeighborhoodFinderMethod<MPTraits>::IncrementNumQueries(){
  this->GetMPProblem()->GetStatClass()->IncNFStat(this->GetNameAndLabel()+"::NumQueries");
}

#endif
