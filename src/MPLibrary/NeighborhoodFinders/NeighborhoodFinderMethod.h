#ifndef PMPL_NEIGHBORHOOD_FINDER_METHOD_H_
#define PMPL_NEIGHBORHOOD_FINDER_METHOD_H_

#include <boost/mpl/list.hpp>
#include <boost/mpl/for_each.hpp>

#include "Neighbors.h"
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

      ///@}

  };
}


////////////////////////////////////////////////////////////////////////////////
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
///
/// @todo I believe we can do away with the virtual find neighbors thing by
///       creating a non-template iterator wrapper which accepts a
///       std::forward_iterator and roadmap as input. Dereference should call
///       GetVID appropriately. Another alternative is to write a helper for
///       the roadmap graph iters and accept only std::forward_iterator for the
///       primary virtual function.
///
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NeighborhoodFinderMethod : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename RoadmapType::VID                 VID;
    typedef typename RoadmapType::VertexSet           VertexSet;
    typedef typename MPTraits::CfgType                CfgType;
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    /// The type of neighbors found.
    enum class Type {
      K,       ///< k-closest neighbors
      RADIUS,  ///< All neighbors within a radius
      APPROX,  ///< Approximate nearest neighbors
      OTHER    ///< Something else
    };

    /// Output iterator for writing discovered neighbors to a container.
    typedef typename std::back_insert_iterator<std::vector<Neighbor>>
        OutputIterator;

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
    ///@name Accessors
    ///@{

    /// @return Type of neighborhood finder
    Type GetType() const noexcept;

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

    ///@}
    ///@name NeighborhoodFinder Interface
    ///@{

    /// Some methods can be implemented more efficiently if the candidates are
    /// provided in a hash set. This function is to support that; the default
    /// implementation forwards to the iterator version.
    /// @param _r The roadmap.
    /// @param _cfg The query configuration.
    /// @param _candidates The set of candidate VIDs.
    /// @param _out Output iterator.
    virtual void FindNeighbors(RoadmapType* _r, const CfgType& _cfg,
        const VertexSet& _candidates, OutputIterator _out);

    /// @overload This version is for group roadmaps
    virtual void FindNeighbors(GroupRoadmapType* _r, const GroupCfgType& _cfg,
        const VertexSet& _candidates, OutputIterator _out);

    /// Finds all vertices in the graph as neighbors.
    /// @param _r The roadmap.
    /// @param _cfg The query configuration.
    /// @param _out Output iterator to the neighbor set.
    void FindNeighbors(RoadmapType* _r, const CfgType& _cfg,
        OutputIterator _out);

    /// Finds all vertices in the graph as neighbors.
    /// @param _r The group roadmap.
    /// @param _cfg The query group configuration.
    /// @param _out Output iterator to the neighbor set.
    void FindNeighbors(GroupRoadmapType* _r, const GroupCfgType& _cfg,
        OutputIterator _out);

    /// Finds "closest" neighbors in a set of nodes to an input configuration.
    /// @param _r The roadmap when input nodes are found
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
    template <typename InputIterator>
    void FindNeighbors(RoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    /// Group overload
    template <typename InputIterator>
    void FindNeighbors(GroupRoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out);

    /// Finds "closest" pairs of neighbors between two set of nodes
    /// @param _r The roadmap when input nodes are found
    /// @param _first1 Begin iterator of the first set of VIDs
    /// @param _last1 End iterator of the first set of VIDs
    /// @param _first2 Begin iterator of the second set of VIDs
    /// @param _last2 End iterator of the second set of VIDs
    /// @param _out Output iterator for neighbor set. Underlying data structure
    ///        is of pair<pair<VID,VID>, double> representing the neighbor pair
    ///        and its corresponing distance
    /// @return The final output iterator _out
    template <typename InputIterator>
    void FindNeighborPairs(RoadmapType* _r,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    /// Group overload
    template <typename InputIterator>
    void FindNeighborPairs(GroupRoadmapType* _r,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    /// Finds all vertices in the graph as neighbors. The results will not be
    /// sorted and will not include configurations with infinite distance to the
    /// query. This is called whenever m_k is 0.
    template <typename GraphType, typename NodeType, typename InputIterator>
    void FindAllNeighbors(GraphType* _g,
        InputIterator _first, InputIterator _last,
        const NodeType& _query, OutputIterator _out);

    /// Finds all vertices in the graph as neighbors. The results will not be
    /// sorted and will not include configurations with infinite distance to the
    /// query. This is called whenever m_k is 0.
    template <typename GraphType, typename InputIterator>
    void FindAllNeighborPairs(GraphType* _g,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Checks if there is a direct edge to potential neighbor.
    /// @param _g The roadmap graph we are searching.
    /// @param _c The query configuration.
    /// @param _v A potential neighbor for _c.
    /// @return True if m_unconnected and there is already a direct edge from _c
    ///         to _v. Always returns false if m_unconnected is false.
    template <typename RoadmapGraph, typename Cfg>
    bool DirectEdge(const RoadmapGraph* _g, const Cfg& _c,
        const typename RoadmapGraph::VID _v) const;

    ///@}
    ///@name Internal State
    ///@{
    /// @todo Remove m_k and m_radius - these don't apply to all NFs so it
    ///       doesn't make sense to have them here. Fix design errors in
    ///       SRT method which require this.

    Type m_nfType{Type::OTHER}; ///< Type of neighborhood finder.
    size_t m_k{0};              ///< How many closest neighbors to find?
    double m_radius{0};         ///< Maximum distance of closest neighbors.

    std::string m_dmLabel;      ///< The distance metric to use.
    bool m_unconnected{false};  ///< Require neighbors with no direct edge.

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
NeighborhoodFinderMethod<MPTraits>::
NeighborhoodFinderMethod(std::string _dmLabel, bool _unconnected) :
    MPBaseObject<MPTraits>(), m_dmLabel(_dmLabel), m_unconnected(_unconnected) { }


template <typename MPTraits>
NeighborhoodFinderMethod<MPTraits>::
NeighborhoodFinderMethod(XMLNode& _node, bool _requireDM) :
    MPBaseObject<MPTraits>(_node) {
  m_dmLabel = _node.Read("dmLabel", _requireDM, "", "Distance Metric Method");

  m_unconnected = _node.Read("unconnected", false, m_unconnected,
      "Require neighbors to be non-adjacent to the query configuration");
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

/*-------------------------------- Accessors ---------------------------------*/

template <typename MPTraits>
inline
typename NeighborhoodFinderMethod<MPTraits>::Type
NeighborhoodFinderMethod<MPTraits>::
GetType() const noexcept {
  return m_nfType;
}


template <typename MPTraits>
inline
size_t&
NeighborhoodFinderMethod<MPTraits>::
GetK() noexcept {
  return m_k;
}


template <typename MPTraits>
inline
double&
NeighborhoodFinderMethod<MPTraits>::
GetRadius() noexcept {
  return m_radius;
}


template <typename MPTraits>
inline
void
NeighborhoodFinderMethod<MPTraits>::
SetDMLabel(const std::string& _label) noexcept {
  m_dmLabel = _label;
}


template <typename MPTraits>
inline
const std::string&
NeighborhoodFinderMethod<MPTraits>::
GetDMLabel() const noexcept {
  return m_dmLabel;
}

/*----------------------- NeighborhoodFinder Interface -----------------------*/

template <typename MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::
FindNeighbors(RoadmapType* _r, const CfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  // Default impl should do the same as regular FindNeighbors.
  FindNeighbors(_r, _candidates.begin(), _candidates.end(),
      _candidates.size() == _r->Size(), _cfg, _out);
}


template <typename MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::
FindNeighbors(GroupRoadmapType* _r, const GroupCfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  // Default impl should do the same as regular FindNeighbors.
  FindNeighbors(_r, _candidates.begin(), _candidates.end(),
      _candidates.size() == _r->Size(), _cfg, _out);
}


template <typename MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::
FindNeighbors(RoadmapType* _r, const CfgType& _cfg, OutputIterator _out) {
  FindNeighbors(_r, _r->begin(), _r->end(), true, _cfg, _out);
}


template <typename MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::
FindNeighbors(GroupRoadmapType* _r, const GroupCfgType& _cfg,
    OutputIterator _out) {
  FindNeighbors(_r, _r->begin(), _r->end(), true, _cfg, _out);
}


template <typename MPTraits>
template <typename InputIterator>
void
NeighborhoodFinderMethod<MPTraits>::
FindNeighbors(RoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::FindNeighbors");
  stats->IncStat(this->GetNameAndLabel() + "::NumQueries");

  typedef typename MPTraits::NeighborhoodFinderMethodList MethodList;

  boost::mpl::for_each<MethodList>(pmpl_detail::VirtualFindNeighbors<
      NeighborhoodFinderMethod, RoadmapType,
      InputIterator, CfgType, OutputIterator>(
        this, _r, _first, _last, _fromFullRoadmap, _cfg, _out));
}


template <typename MPTraits>
template <typename InputIterator>
void
NeighborhoodFinderMethod<MPTraits>::
FindNeighbors(GroupRoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const GroupCfgType& _cfg, OutputIterator _out) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::FindNeighbors");
  stats->IncStat(this->GetNameAndLabel() + "::NumQueries");

  typedef typename MPTraits::NeighborhoodFinderMethodList MethodList;

  boost::mpl::for_each<MethodList>(pmpl_detail::VirtualFindNeighbors<
      NeighborhoodFinderMethod, GroupRoadmapType,
      InputIterator, GroupCfgType, OutputIterator>(
        this, _r, _first, _last, _fromFullRoadmap, _cfg, _out));
}


template <typename MPTraits>
template <typename InputIterator>
void
NeighborhoodFinderMethod<MPTraits>::
FindNeighborPairs(RoadmapType* _r,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::FindNeighborPairs");
  stats->IncStat(this->GetNameAndLabel() + "::NumQueries");

  typedef typename MPTraits::NeighborhoodFinderMethodList MethodList;

  boost::mpl::for_each<MethodList>(pmpl_detail::VirtualFindNeighborPairs<
      NeighborhoodFinderMethod, RoadmapType,
      InputIterator, OutputIterator>(this, _r, _first1, _last1, _first2,
          _last2, _out));
}


template <typename MPTraits>
template <typename InputIterator>
void
NeighborhoodFinderMethod<MPTraits>::
FindNeighborPairs(GroupRoadmapType* _r,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::FindNeighborPairs");
  stats->IncStat(this->GetNameAndLabel() + "::NumQueries");

  typedef typename MPTraits::NeighborhoodFinderMethodList MethodList;

  boost::mpl::for_each<MethodList>(pmpl_detail::VirtualFindNeighborPairs<
      NeighborhoodFinderMethod, GroupRoadmapType,
      InputIterator, OutputIterator>(this, _r, _first1, _last1, _first2,
          _last2, _out));
}

/*-------------------------------- Helpers -----------------------------------*/

template <typename MPTraits>
template <typename RoadmapGraph, typename Cfg>
bool
NeighborhoodFinderMethod<MPTraits>::
DirectEdge(const RoadmapGraph* _g, const Cfg& _c,
    typename RoadmapGraph::VID _v) const {
  // Consider all nodes to be non-neighbors if we aren't using this check.
  if(!this->m_unconnected)
    return false;

  // The nodes are neighbors if _c is in the graph and the edge (_c, _v) exists.
  const typename RoadmapGraph::VID vid = _g->GetVID(_c);
  return vid != INVALID_VID and _g->IsEdge(vid, _v);
}

/*----------------------------------------------------------------------------*/

#endif
