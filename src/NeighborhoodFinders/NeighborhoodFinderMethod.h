#ifndef NEIGHBORHOOD_FINDER_METHOD_H_
#define NEIGHBORHOOD_FINDER_METHOD_H_

#include <boost/mpl/list.hpp>
#include <boost/mpl/for_each.hpp>
#include "Utilities/MPUtils.h"
#include "MPProblem/RoadmapGraph.h"

namespace pmpl_detail {

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @brief Dynamic dispatch for template member functions of neighborhood
///        finders.
/// @tparam NF NeighborhoodFinderMethod type
/// @tparam RDMP Roadmap type
/// @tparam I Input iterator type
/// @tparam CFG Configuration type
/// @tparam O Output iterator type
////////////////////////////////////////////////////////////////////////////////
template<typename NF, typename RDMP, typename I, typename CFG, typename O>
  struct VirtualFindNeighbors{
    public:
      VirtualFindNeighbors(NF* _v, RDMP* _r, I _f, I _l, bool _b,
          const CFG& _c, O _o) :
        m_memory(_v), m_rdmp(_r), m_first(_f), m_last(_l),
        m_fromFullRoadmap(_b), m_cfg(_c), m_output(_o){
        }

      template<typename T>
        void operator()(T& _t) {
          T* tptr = dynamic_cast<T*>(m_memory);
          if(tptr != NULL){
            tptr->FindNeighbors(m_rdmp, m_first, m_last, m_fromFullRoadmap,
                m_cfg, m_output);
          }
        }
    private:
      NF* m_memory;
      RDMP* m_rdmp;
      I m_first, m_last;
      bool m_fromFullRoadmap;
      const CFG& m_cfg;
      O m_output;
  };

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @brief Dynamic dispatch for template member functions of neighborhood
///        finders.
/// @tparam NF NeighborhoodFinderMethod type
/// @tparam RDMP Roadmap type
/// @tparam I Input iterator type
/// @tparam O Output iterator type
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinders
/// @brief Base algorithm abstraction for \ref NeighborhoodFinders.
///
/// NeighborhoodFinderMethod has two important functions: @c FindNeighbors and
/// @c FindNeighborPairs.
///
/// @c FindNeighbors takes an input configuration and a set of candidate
/// neighbors and returns the computed set of "nearest" neighbors.
///
/// @c FindNeighborPairs determines the "closest" pairs of configurations
/// betweeen two sets of nodes located in a roadmap.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class NeighborhoodFinderMethod : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;

    NeighborhoodFinderMethod(string _dmLabel = "", bool _unconnected = false);
    NeighborhoodFinderMethod(MPProblemType* _problem, XMLNode& _node, bool _requireDM = true);

    virtual void Print(ostream& _os) const {
      MPBaseObject<MPTraits>::Print(_os);
      _os << "\tdmLabel: " << m_dmLabel << endl
        << "\tunconnected: " << m_unconnected << endl;
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @return Type of neighborhood finder
    ////////////////////////////////////////////////////////////////////////////
    NFType GetNFType() const {return m_nfType;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of closest neighbors to find
    ////////////////////////////////////////////////////////////////////////////
    size_t& GetK() {return m_k;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Distance of farthest potential neighbor
    ////////////////////////////////////////////////////////////////////////////
    double& GetRadius() {return m_radius;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Distance Metric for this neighborhood finder
    ////////////////////////////////////////////////////////////////////////////
    virtual typename MPProblemType::DistanceMetricPointer GetDMMethod() const;

    ////////////////////////////////////////////////////////////////////////////
    /// @return Total time of neighborhood finding
    ////////////////////////////////////////////////////////////////////////////
    double GetTotalTime() const;
    ////////////////////////////////////////////////////////////////////////////
    /// @return Time of neighborhood finding query
    ////////////////////////////////////////////////////////////////////////////
    double GetQueryTime() const;
    ////////////////////////////////////////////////////////////////////////////
    /// @return Time of neighborhood finding construction
    ////////////////////////////////////////////////////////////////////////////
    double GetConstructionTime() const;
    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of neighborhood finding queries
    ////////////////////////////////////////////////////////////////////////////
    size_t GetNumQueries() const;

    ////////////////////////////////////////////////////////////////////////////
    //   Neighborhood Finder Methods.
    //
    //   FindNeighbors and FindNeighborPairs need to be
    //   implemented in base classes
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Finds "closest" neighbors in a set of nodes to an input
    ///        configuration
    ///
    /// @overload
    /// Uses the entire roadmap as set of nodes.
    ////////////////////////////////////////////////////////////////////////////
    template<typename OutputIterator>
      OutputIterator FindNeighbors(RoadmapType* _rmp, const CfgType& _cfg,
          OutputIterator _out){
        return FindNeighbors(_rmp,
            _rmp->GetGraph()->begin(), _rmp->GetGraph()->end(), true,
            _cfg, _out);
      }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Finds "closest" neighbors in a set of nodes to an input
    ///        configuration
    /// @param _rmp The roadmap when input nodes are found
    /// @param _first Begin iterator of the set of VIDs
    /// @param _last End iterator of the set of VIDs
    /// @param _fromFullRoadmap If true, saved internal NF model will be used
    ///                         instead of computing a temporary one. Only
    ///                         applies to advanced NFMethods.
    /// @param _cfg The query configuration to find neighbors of
    /// @param _out Output iterator for neighbor set. Underlying data structure
    ///        is of pair<VID, double> representing the neighbor and distance to
    ///        _cfg
    /// @return The final output iterator _out
    ///
    /// @usage
    /// @code
    /// NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nfLabel);
    /// CfgType c;
    /// vector<VID> nodes;
    /// bool b;
    /// vector<pair<VID, double> > neighbors;
    /// nf->FindNeighbors(this->GetMPProblem()->GetRoadmap(),
    ///                   nodes.begin(), nodes.end(), b, c,
    ///                   back_inserter(neighbors));
    /// @endcode
    ////////////////////////////////////////////////////////////////////////////
    template<typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighbors(RoadmapType* _rmp,
          InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
          const CfgType& _cfg, OutputIterator _out) {
        typedef typename MPTraits::NeighborhoodFinderMethodList MethodList;
        boost::mpl::for_each<MethodList>(pmpl_detail::VirtualFindNeighbors<
            NeighborhoodFinderMethod, RoadmapType,
            InputIterator, CfgType, OutputIterator>(
              this, _rmp, _first, _last, _fromFullRoadmap, _cfg, _out));
        return _out;
      }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Finds "closest" pairs of neighbors between two set of nodes
    /// @param _rmp The roadmap when input nodes are found
    /// @param _first1 Begin iterator of the first set of VIDs
    /// @param _last1 End iterator of the first set of VIDs
    /// @param _first2 Begin iterator of the second set of VIDs
    /// @param _last2 End iterator of the second set of VIDs
    /// @param _out Output iterator for neighbor set. Underlying data structure
    ///        is of pair<pair<VID,VID>, double> representing the neighbor pair
    ///        and its corresponing distance
    /// @return The final output iterator _out
    ///
    /// @usage
    /// @code
    /// NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nfLabel);
    /// vector<VID> s1, s2;
    /// vector<pair<pair<VID, VID>, double> > neighbors;
    /// nf->FindNeighbors(this->GetMPProblem()->GetRoadmap(),
    ///                   s1.begin(), s1.end(), s2.begin(), s2.end(),
    ///                   back_inserter(neighbors));
    /// @endcode
    ////////////////////////////////////////////////////////////////////////////
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
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Checks that there is no direct edge to potential neighbor
    /// @param _rmp Roadmap to find neighbor
    /// @param _c Query configuration
    /// @param _v Potential neighbor
    /// @return Edge existance
    ////////////////////////////////////////////////////////////////////////////
    bool CheckUnconnected(RoadmapType* _rmp, const CfgType& _c, VID _v);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Starts special timer for total neighborhood finding
    ////////////////////////////////////////////////////////////////////////////
    void StartTotalTime();
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Ends special timer for total neighborhood finding
    ////////////////////////////////////////////////////////////////////////////
    void EndTotalTime();
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Starts special timer for neighborhood finding query time
    ////////////////////////////////////////////////////////////////////////////
    void StartQueryTime();
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Ends special timer for neighborhood finding query time
    ////////////////////////////////////////////////////////////////////////////
    void EndQueryTime();
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Starts special timer for neighborhood finding construction time
    ////////////////////////////////////////////////////////////////////////////
    void StartConstructionTime();
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Ends special timer for neighborhood finding construction time
    ////////////////////////////////////////////////////////////////////////////
    void EndConstructionTime();
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Increment total number of neighborhood finding requests
    ////////////////////////////////////////////////////////////////////////////
    void IncrementNumQueries();

    NFType m_nfType; ///< Type of neighborhood finder, e.g., radius or k based. Set by derived methods
    size_t m_k; ///< How many closest neighbors to find
    double m_radius; ///< Maximum distance of closest neighbors

    string m_dmLabel; ///< Distance Metric
    bool m_unconnected; ///< Require neighbor to not have direct edge
};

template<class MPTraits>
NeighborhoodFinderMethod<MPTraits>::NeighborhoodFinderMethod(string _dmLabel, bool _unconnected)
  : MPBaseObject<MPTraits>(),
  m_nfType(OTHER), m_k(0), m_radius(0),
  m_dmLabel(_dmLabel), m_unconnected(_unconnected) {}

template<class MPTraits>
NeighborhoodFinderMethod<MPTraits>::NeighborhoodFinderMethod(MPProblemType* _problem, XMLNode& _node, bool _requireDM)
  : MPBaseObject<MPTraits>(_problem, _node),
  m_nfType(OTHER), m_k(0), m_radius(0) {
    m_dmLabel = _node.Read("dmLabel", _requireDM, "", "Distance Metric Method");
    m_unconnected = _node.Read("unconnected", false, false, "Require neighbors to be non adjacent to the query configuration");
  }

template<class MPTraits>
typename MPTraits::MPProblemType::DistanceMetricPointer
NeighborhoodFinderMethod<MPTraits>::GetDMMethod() const {
  return this->GetDistanceMetric(m_dmLabel);
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
  return this->GetStatClass()->GetSeconds(this->GetNameAndLabel()+"::Total");
}

template<class MPTraits>
double
NeighborhoodFinderMethod<MPTraits>::GetQueryTime() const{
  return this->GetStatClass()->GetSeconds(this->GetNameAndLabel()+"::Query");
}

template<class MPTraits>
double
NeighborhoodFinderMethod<MPTraits>::GetConstructionTime() const{
  return this->GetStatClass()->GetSeconds(this->GetNameAndLabel()+"::Construction");
}

template<class MPTraits>
size_t
NeighborhoodFinderMethod<MPTraits>::GetNumQueries() const{
  return this->GetStatClass()->GetNFStat(this->GetNameAndLabel()+"::NumQueries");
}

template<class MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::StartTotalTime(){
  this->GetStatClass()->StartClock(this->GetNameAndLabel()+"::Total");
}

template<class MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::EndTotalTime(){
  this->GetStatClass()->StopClock(this->GetNameAndLabel()+"::Total");
}

template<class MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::StartQueryTime(){
  this->GetStatClass()->StartClock(this->GetNameAndLabel()+"::Query");
}

template<class MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::EndQueryTime(){
  this->GetStatClass()->StopClock(this->GetNameAndLabel()+"::Query");
}

template<class MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::StartConstructionTime(){
  this->GetStatClass()->StartClock(this->GetNameAndLabel()+"::Construction");
}

template<class MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::EndConstructionTime(){
  this->GetStatClass()->StopClock(this->GetNameAndLabel()+"::Construction");
}

template<class MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::IncrementNumQueries(){
  this->GetStatClass()->IncNFStat(this->GetNameAndLabel()+"::NumQueries");
}

#endif
