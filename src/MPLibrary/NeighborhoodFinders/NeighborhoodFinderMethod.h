#ifndef NEIGHBORHOOD_FINDER_METHOD_H_
#define NEIGHBORHOOD_FINDER_METHOD_H_

#include <boost/mpl/list.hpp>
#include <boost/mpl/for_each.hpp>
#include "Utilities/MPUtils.h"
#include "ConfigurationSpace/RoadmapGraph.h"

namespace pmpl_detail {

  //////////////////////////////////////////////////////////////////////////////
  /// Dynamic dispatch for template member functions of neighborhood finders.
  /// @tparam NF NeighborhoodFinderMethod type
  /// @tparam RDMP Roadmap type
  /// @tparam I Input iterator type
  /// @tparam CFG Configuration type
  /// @tparam O Output iterator type
  //////////////////////////////////////////////////////////////////////////////
  template<typename NF, typename RDMP, typename I, typename CFG, typename O>
  struct VirtualFindNeighbors {

    public:

      ///@name Construction
      ///@{

      VirtualFindNeighbors(NF* _v, RDMP* _r, I _f, I _l, bool _b,
          const CFG& _c, O _o) :
          m_methodPointer(_v), m_rdmp(_r), m_first(_f), m_last(_l),
          m_fromFullRoadmap(_b), m_cfg(_c), m_output(_o) { }

      ///@}
      ///@name Interface
      ///@{

      template <typename T>
      void operator()(T& _t) {
        T* tptr = dynamic_cast<T*>(m_methodPointer);
        if(tptr != nullptr)
          tptr->FindNeighbors(m_rdmp, m_first, m_last, m_fromFullRoadmap,
                              m_cfg, m_output);
      }

      ///@}

    private:

      ///@name Internal State
      ///@{

      NF* m_methodPointer;
      RDMP* m_rdmp;
      I m_first, m_last;
      bool m_fromFullRoadmap;
      const CFG& m_cfg;
      O m_output;

      ///@}

  };


  ////////////////////////////////////////////////////////////////////////////////
  /// Dynamic dispatch for template member functions of neighborhood finders.
  /// @tparam NF NeighborhoodFinderMethod type
  /// @tparam RDMP Roadmap type
  /// @tparam I Input iterator type
  /// @tparam O Output iterator type
  ////////////////////////////////////////////////////////////////////////////////
  template<typename NF, typename RDMP, typename I, typename O>
  struct VirtualFindNeighborPairs {

    public:

      ///@name Construction
      ///@{

      VirtualFindNeighborPairs(NF* _v, RDMP* _r, I _f1, I _l1, I _f2, I _l2,
          O _o) :
            m_methodPointer(_v), m_rdmp(_r), m_first1(_f1), m_last1(_l1),
            m_first2(_f2), m_last2(_l2), m_output(_o) { }

      ///@}
      ///@name Interface
      ///@{

      template<typename T>
      void operator()(T& _t) {
        T* tptr = dynamic_cast<T*>(m_methodPointer);
        if(tptr != nullptr)
          tptr->FindNeighborPairs(m_rdmp, m_first1, m_last1, m_first2,
                                  m_last2, m_output);
      }

      ///@}

    private:

      ///@name Internal State
      ///@{

      NF* m_methodPointer;
      RDMP* m_rdmp;
      I m_first1, m_last1, m_first2, m_last2;
      O m_output;
//      std::vector<size_t> m_formation;

      ///@}

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
/// Base algorithm abstraction for \ref NeighborhoodFinders.
///
/// NeighborhoodFinderMethod has two important functions: @c FindNeighbors and
/// @c FindNeighborPairs.
///
/// @c FindNeighbors takes an input configuration and a set of candidate
/// neighbors and returns the computed set of "nearest" neighbors.
/// @usage
/// @code
/// NeighborhoodFinderPointer nf = this->GetNeighborhoodFinder(m_nfLabel);
/// CfgType c;
/// vector<VID> nodes;
/// bool b;
/// vector<pair<VID, double> > neighbors;
/// nf->FindNeighbors(this->GetRoadmap(),
///                   nodes.begin(), nodes.end(), b, c,
///                   back_inserter(neighbors));
/// @endcode
///
/// @c FindNeighborPairs determines the "closest" pairs of configurations
/// betweeen two sets of nodes located in a roadmap.
/// @usage
/// @code
/// NeighborhoodFinderPointer nf = this->GetNeighborhoodFinder(m_nfLabel);
/// vector<VID> s1, s2;
/// vector<pair<pair<VID, VID>, double> > neighbors;
/// nf->FindNeighbors(this->GetRoadmap(),
///                   s1.begin(), s1.end(), s2.begin(), s2.end(),
///                   back_inserter(neighbors));
/// @endcode
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NeighborhoodFinderMethod : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename MPTraits::CfgType                CfgType;
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    NeighborhoodFinderMethod(std::string _dmLabel = "",
                             bool _unconnected = false);

    NeighborhoodFinderMethod(XMLNode& _node, bool _requireDM = true);

    virtual ~NeighborhoodFinderMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinder Interface
    ///@{

    /// @return Type of neighborhood finder
    NFType GetNFType() const noexcept;

    /// @return Number of closest neighbors to find
    size_t& GetK() noexcept;

    /// @return Distance of farthest potential neighbor
    double& GetRadius() noexcept;

    /// Set the distance metric label.
    /// @param _label The new DM label to use.
    void SetDMLabel(const std::string& _label) noexcept;

    /// Get the distance metric label.
    /// @return The label for the current DM.
    const std::string& GetDMLabel() const noexcept;

    /// Finds "closest" neighbors in a set of nodes to an input configuration.
    /// Uses the entire roadmap as set of nodes.
    template<typename OutputIterator>
    OutputIterator FindNeighbors(RoadmapType* _rmp, const CfgType& _cfg,
                OutputIterator _out) {
      auto graph = _rmp->GetGraph();
      return FindNeighbors(_rmp, graph->begin(), graph->end(), true, _cfg, _out);
    }

    /// Group overload
    template<typename OutputIterator>
    OutputIterator FindNeighbors(GroupRoadmapType* _rmp, const GroupCfgType& _cfg,
                                 OutputIterator _out) {
      return FindNeighbors(_rmp, _rmp->begin(), _rmp->end(), true, _cfg, _out);
    }

    /// Finds "closest" neighbors in a set of nodes to an input configuration.
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

    /// Group overload
    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(GroupRoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out) {
      typedef typename MPTraits::NeighborhoodFinderMethodList MethodList;
      boost::mpl::for_each<MethodList>(pmpl_detail::VirtualFindNeighbors<
          NeighborhoodFinderMethod, GroupRoadmapType,
          InputIterator, GroupCfgType, OutputIterator>(
            this, _rmp, _first, _last, _fromFullRoadmap, _cfg, _out));
      return _out;
    }

    /// Finds "closest" pairs of neighbors between two set of nodes
    /// @param _rmp The roadmap when input nodes are found
    /// @param _first1 Begin iterator of the first set of VIDs
    /// @param _last1 End iterator of the first set of VIDs
    /// @param _first2 Begin iterator of the second set of VIDs
    /// @param _last2 End iterator of the second set of VIDs
    /// @param _out Output iterator for neighbor set. Underlying data structure
    ///        is of pair<pair<VID,VID>, double> representing the neighbor pair
    ///        and its corresponing distance
    /// @return The final output iterator _out
    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(RoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out) {
      typedef typename MPTraits::NeighborhoodFinderMethodList MethodList;
      boost::mpl::for_each<MethodList>(pmpl_detail::VirtualFindNeighborPairs<
          NeighborhoodFinderMethod, RoadmapType,
          InputIterator, OutputIterator>(this, _rmp, _first1, _last1, _first2,
              _last2, _out));
      return _out;
    }

    /// Group overload
    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(GroupRoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out) {
      typedef typename MPTraits::NeighborhoodFinderMethodList MethodList;
      boost::mpl::for_each<MethodList>(pmpl_detail::VirtualFindNeighborPairs<
          NeighborhoodFinderMethod, GroupRoadmapType,
          InputIterator, OutputIterator>(this, _rmp, _first1, _last1, _first2,
              _last2, _out));
      return _out;
    }

  protected:

    /// Checks that there is no direct edge to potential neighbor
    /// @param _rmp Roadmap to find neighbor
    /// @param _c Query configuration
    /// @param _v Potential neighbor
    /// @return Edge existence
    template <typename Roadmap, typename Cfg>
    bool CheckUnconnected(Roadmap* _rmp, const Cfg& _c,
                          typename Roadmap::VID _v);

    /// Increment total number of neighborhood finding requests
    void IncrementNumQueries() const;

    ///@}
    ///@name Internal State
    ///@{

    NFType m_nfType;    ///< Type of neighborhood finder.
    size_t m_k;         ///< How many closest neighbors to find?
    double m_radius;    ///< Maximum distance of closest neighbors.

    std::string m_dmLabel;   ///< The distance metric to use.
    bool m_unconnected; ///< Require neighbors with no direct edge.

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
NeighborhoodFinderMethod<MPTraits>::
NeighborhoodFinderMethod(std::string _dmLabel, bool _unconnected) :
    MPBaseObject<MPTraits>(), m_nfType(OTHER), m_k(0), m_radius(0),
    m_dmLabel(_dmLabel), m_unconnected(_unconnected) { }


template <typename MPTraits>
NeighborhoodFinderMethod<MPTraits>::
NeighborhoodFinderMethod(XMLNode& _node, bool _requireDM) :
    MPBaseObject<MPTraits>(_node), m_nfType(OTHER), m_k(0), m_radius(0) {
  m_dmLabel = _node.Read("dmLabel", _requireDM, "", "Distance Metric Method");
  m_unconnected = _node.Read("unconnected", false, false, "Require neighbors "
      "to be non-adjacent to the query configuration");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::
Print(std::ostream& _os) const {
  MPBaseObject<MPTraits>::Print(_os);
  _os << "\tdmLabel: " << m_dmLabel
      << "\n\tunconnected: " << m_unconnected
      << std::endl;
}

/*----------------------- NeighborhoodFinder Interface -----------------------*/

template <typename MPTraits>
NFType
NeighborhoodFinderMethod<MPTraits>::
GetNFType() const noexcept {
  return m_nfType;
}


template <typename MPTraits>
size_t&
NeighborhoodFinderMethod<MPTraits>::
GetK() noexcept {
  return m_k;
}


template <typename MPTraits>
double&
NeighborhoodFinderMethod<MPTraits>::
GetRadius() noexcept {
  return m_radius;
}


template <typename MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::
SetDMLabel(const std::string& _label) noexcept {
  m_dmLabel = _label;
}


template <typename MPTraits>
const std::string&
NeighborhoodFinderMethod<MPTraits>::
GetDMLabel() const noexcept {
  return m_dmLabel;
}


template <typename MPTraits>
template <typename Roadmap, typename Cfg>
bool
NeighborhoodFinderMethod<MPTraits>::
CheckUnconnected(Roadmap* _r, const Cfg& _c, typename Roadmap::VID _v) {
  if(this->m_unconnected) {
    typename Roadmap::VID vid = _r->GetGraph()->GetVID(_c);
    if(vid != INVALID_VID and _r->GetGraph()->IsEdge(_v, vid))
      return true;
  }
  return false;
}

template <typename MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::
IncrementNumQueries() const {
  this->GetStatClass()->IncStat(this->GetName() + "::NumQueries");
}

#endif
