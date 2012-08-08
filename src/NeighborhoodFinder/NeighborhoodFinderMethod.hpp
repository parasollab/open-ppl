#ifndef NEIGHBORHOODFINDERMETHOD_H_
#define NEIGHBORHOODFINDERMETHOD_H_

#include <boost/mpl/list.hpp>
#include <boost/mpl/for_each.hpp>
#include <string>
#include <iostream>
#include "MPUtils.h"
#include "MPProblem.h"

class BFNF;
class BFFNF;
class RadiusNF;
//template<class CFG, class WEIGHT>
//class DPESNF;
//template<class CFG, class WEIGHT>
//class MPNNNF;
class CGALNF;
//template<class CFG, class WEIGHT>
//class STNF;
//template<class CFG, class WEIGHT>
//class MTNF;
class BandsNF;

namespace pmpl_detail { //hide NeighborhoodFinderMethodList in pmpl_detail namespace
  typedef boost::mpl::list<
    BFNF,
    BFFNF,
    RadiusNF,
    //DPESNF<CfgType,WeightType>,
    //MPNNNF<CfgType,WeightType>,
    CGALNF,
    //STNF<CfgType,WeightType>, 
    //MTNF<CfgType,WeightType>,
    BandsNF
    > NeighborhoodFinderMethodList;
  
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

class NeighborhoodFinderMethod : public MPBaseObject {
  public:
    NeighborhoodFinderMethod(string _dmLabel = "", string _label = "",  MPProblem* _problem = NULL);
    NeighborhoodFinderMethod(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~NeighborhoodFinderMethod() {}

    shared_ptr<DistanceMetricMethod> GetDMMethod() const;

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
    
    template<typename RDMP, typename InputIterator, typename OutputIterator>
      OutputIterator KClosest(RDMP* _rmp, 
          InputIterator _first, InputIterator _last, typename RDMP::VID _v, size_t _k, OutputIterator _out) {
        return KClosest(_rmp, _first, _last, (_rmp->m_pRoadmap->find_vertex(_v))->property(), _k, _out);
      }

    // KClosest that operate over the entire roadmap to find the _kclosest to a VID or CFG
    // NOTE: These are the prefered methods for _kClosest computations
    template<typename RDMP, typename OutputIterator>
      OutputIterator KClosest(RDMP* _rmp, typename RDMP::VID _v, size_t _k, OutputIterator _out){
        return KClosest(_rmp, (_rmp->m_pRoadmap->find_vertex(_v))->property(), _k, _out);
      }

    template<typename RDMP, typename OutputIterator>
      OutputIterator KClosest(RDMP* _rmp, typename RDMP::CfgType _cfg, size_t _k, OutputIterator _out){
        m_fromRDMPVersion = true;
        return KClosest(_rmp, _rmp->m_pRoadmap->descriptor_begin(), _rmp->m_pRoadmap->descriptor_end(), _cfg, _k, _out);
      }

    // do the work here, and have the function above obtain the CFG and call this one
    template<typename RDMP, typename InputIterator, typename OutputIterator>
      OutputIterator KClosest(RDMP* _rmp, 
          InputIterator _first, InputIterator _last, typename RDMP::CfgType _cfg, size_t _k, OutputIterator _out){
        typedef pmpl_detail::NeighborhoodFinderMethodList MethodList;
        boost::mpl::for_each<MethodList>(pmpl_detail::VirtualKClosest<
            NeighborhoodFinderMethod, RDMP, 
            InputIterator, typename RDMP::CfgType, OutputIterator>(this, _rmp, _first, _last, _cfg, _k, _out));
        return _out;
      }

    // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
    // represent the _kclosest pairs of VIDs between the two ranges.
    template<typename RDMP, typename InputIterator, typename OutputIterator>
      OutputIterator KClosestPairs(RDMP* _rmp,
          InputIterator _first1, InputIterator _last1, 
          InputIterator _first2, InputIterator _last2, 
          size_t _k, OutputIterator _out){
        typedef pmpl_detail::NeighborhoodFinderMethodList MethodList;
        boost::mpl::for_each<MethodList>(pmpl_detail::VirtualKClosestPairs<
            NeighborhoodFinderMethod, RDMP, 
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

#endif //end #ifndef
