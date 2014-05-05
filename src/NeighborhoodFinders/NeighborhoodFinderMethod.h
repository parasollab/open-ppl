#ifndef NEIGHBORHOODFINDERMETHOD_H_
#define NEIGHBORHOODFINDERMETHOD_H_

#include <boost/mpl/list.hpp>
#include <boost/mpl/for_each.hpp>
#include "Utilities/MPUtils.h"
#include "MPProblem/RoadmapGraph.h"

//these methods allow for dynamic dispatch for the templated functions within
//these neighborhood finder classes
namespace pmpl_detail {

  template<typename NF, typename RDMP, typename I, typename CFG, typename O>
    struct VirtualFindNeighbors{
      public:
        VirtualFindNeighbors(NF* _v, RDMP* _r, I _f, I _l, const CFG& _c, O _o) :
          m_memory(_v), m_rdmp(_r), m_first(_f), m_last(_l), m_cfg(_c), m_output(_o){
          }

        template<typename T>
          void operator()(T& _t) {
            T* tptr = dynamic_cast<T*>(m_memory);
            if(tptr != NULL){
              tptr->FindNeighbors(m_rdmp, m_first, m_last, m_cfg, m_output);
            }
          }
      private:
        NF* m_memory;
        RDMP* m_rdmp;
        I m_first, m_last;
        const CFG& m_cfg;
        O m_output;
    };

  template<typename NF, typename RDMP, typename I, typename O>
    struct VirtualFindNeighborPairs{
      public:
        VirtualFindNeighborPairs(NF* _v, RDMP* _r, I _f1, I _l1, I _f2, I _l2, O _o) :
          m_memory(_v), m_rdmp(_r), m_first1(_f1), m_last1(_l1), m_first2(_f2), m_last2(_l2), m_output(_o){
          }

        template<typename T>
          void operator()(T& _t) {
            T* tptr = dynamic_cast<T*>(m_memory);
            if(tptr != NULL){
              tptr->FindNeighborPairs(m_rdmp, m_first1, m_last1, m_first2, m_last2, m_output);
            }
          }
      private:
        NF* m_memory;
        RDMP* m_rdmp;
        I m_first1, m_last1, m_first2, m_last2;
        O m_output;
    };
}

// K      - NF will find k-closest neighbors
// RADIUS - NF will find all neighbors within a radius
// OPTIMAL- NF will find optimal neighbors
// APPROX - Not currently used
// OTHER  - NF will find neighbors in some other way
enum NFType {K, RADIUS, OPTIMAL, APPROX, OTHER};

template<class MPTraits>
class NeighborhoodFinderMethod : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;

    NeighborhoodFinderMethod(string _dmLabel = "", bool _unconnected = false);
    NeighborhoodFinderMethod(MPProblemType* _problem, XMLNodeReader& _node, bool _requireDM = true);

    virtual void PrintOptions(ostream& _os) const {
      _os << this->GetNameAndLabel() << endl
        << "\tdmLabel: " << m_dmLabel << endl
        << "\tunconnected: " << m_unconnected << endl;
    }

    NFType GetNFType() const {return m_nfType;}
    size_t& GetK() {return m_k;}
    double& GetRadius() {return m_radius;}

    virtual typename MPProblemType::DistanceMetricPointer GetDMMethod() const;

    double GetTotalTime() const;
    double GetQueryTime() const;
    double GetConstructionTime() const;
    size_t GetNumQueries() const;

    ////////////////////////////////////////////////////////////////////////////////////
    //   Neighborhood Finder Methods.
    //
    //   FindNeighbors and FindNeighborPairs need to be
    //   implemented in base classes
    ////////////////////////////////////////////////////////////////////////////////////

    template<typename OutputIterator>
      OutputIterator FindNeighbors(RoadmapType* _rmp, const CfgType& _cfg, OutputIterator _out){
        m_fromRDMPVersion = true;
        return FindNeighbors(_rmp, _rmp->GetGraph()->begin(), _rmp->GetGraph()->end(), _cfg, _out);
      }

    template<typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighbors(RoadmapType* _rmp,
          InputIterator _first, InputIterator _last, const CfgType& _cfg, OutputIterator _out){
        typedef typename MPTraits::NeighborhoodFinderMethodList MethodList;
        boost::mpl::for_each<MethodList>(pmpl_detail::VirtualFindNeighbors<
            NeighborhoodFinderMethod, RoadmapType,
            InputIterator, CfgType, OutputIterator>(this, _rmp, _first, _last, _cfg, _out));
        return _out;
      }

    // FindNeighbors that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
    // represent the _kclosest pairs of VIDs between the two ranges.
    template<typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighborPairs(RoadmapType* _rmp,
          InputIterator _first1, InputIterator _last1,
          InputIterator _first2, InputIterator _last2,
          OutputIterator _out){
        typedef typename MPTraits::NeighborhoodFinderMethodList MethodList;
        boost::mpl::for_each<MethodList>(pmpl_detail::VirtualFindNeighborPairs<
            NeighborhoodFinderMethod, RoadmapType,
            InputIterator, OutputIterator>(this, _rmp, _first1, _last1, _first2, _last2, _out));
        return _out;
      }

  protected:
    bool CheckUnconnected(RoadmapType* _rmp, const CfgType& _c, VID _v);

    void StartTotalTime();
    void EndTotalTime();
    void StartQueryTime();
    void EndQueryTime();
    void StartConstructionTime();
    void EndConstructionTime();
    void IncrementNumQueries();

    //type of the neighbor finder.
    //This is set by the derived methods.
    NFType m_nfType;
    size_t m_k;
    double m_radius;

    string m_dmLabel;
    bool m_unconnected;
    bool m_fromRDMPVersion;
};

template<class MPTraits>
NeighborhoodFinderMethod<MPTraits>::NeighborhoodFinderMethod(string _dmLabel, bool _unconnected)
  : MPBaseObject<MPTraits>(),
  m_nfType(OTHER), m_k(0), m_radius(0),
  m_dmLabel(_dmLabel), m_unconnected(_unconnected), m_fromRDMPVersion(false) {}

template<class MPTraits>
NeighborhoodFinderMethod<MPTraits>::NeighborhoodFinderMethod(MPProblemType* _problem, XMLNodeReader& _node, bool _requireDM)
  : MPBaseObject<MPTraits>(_problem, _node),
  m_nfType(OTHER), m_k(0), m_radius(0),
  m_fromRDMPVersion(false){
    m_dmLabel = _node.stringXMLParameter("dmLabel", _requireDM, "", "Distance Metric Method");
    m_unconnected = _node.boolXMLParameter("unconnected", false, false, "Require neighbors to be non adjacent to the query configuration");
  }

template<class MPTraits>
typename MPTraits::MPProblemType::DistanceMetricPointer
NeighborhoodFinderMethod<MPTraits>::GetDMMethod() const {
  return this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
}

template<class MPTraits>
bool
NeighborhoodFinderMethod<MPTraits>::CheckUnconnected(RoadmapType* _rmp, const CfgType& _c, VID _v){
  if(this->m_unconnected){
    VID vid = _rmp->GetGraph()->GetVID(_c);
    if(vid != INVALID_VID){
      if(_rmp->GetGraph()->IsEdge(_v, vid))
        return true;
    }
  }
  return false;
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
