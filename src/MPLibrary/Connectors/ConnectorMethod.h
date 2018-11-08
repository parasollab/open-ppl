#ifndef PMPL_CONNECTION_METHOD_H_
#define PMPL_CONNECTION_METHOD_H_

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/MPUtils.h"
#include "Utilities/MetricUtils.h"

#include <boost/mpl/for_each.hpp>


namespace pmpl_detail {

  //////////////////////////////////////////////////////////////////////////////
  /// Facilitate function calls for general connection methods.
  /// @tparam CM Connector method derived class type
  /// @tparam RDMP Roadmap type
  /// @tparam I1 Input iterator 1 type
  /// @tparam I2 Input iterator 2 type
  /// @tparam O Output iterator tyep
  ///
  /// Facilitates simulation of pure virtualism of template functions in C++,
  /// which is explicitly disallowed for the language. Essentially iterates over
  /// a type list and attempts dynamic casts to that type to find appropriate
  /// derived class.
  /// @ingroup Connectors
  //////////////////////////////////////////////////////////////////////////////
  template <typename CM, typename RDMP, typename I1, typename I2, typename O>
  struct VirtualConnect final {

    ///@name Construction
    ///@{

    VirtualConnect(CM* _v, RDMP* _r, I1 _i1f, I1 _i1l, I2 _i2f, I2 _i2l,
        bool _b, O _o) :
        m_memory(_v), m_rdmp(_r), m_i1first(_i1f), m_i1last(_i1l),
        m_i2first(_i2f), m_i2last(_i2l), m_fromFullRoadmap(_b), m_output(_o) { }

    ///@}
    ///@name Interface
    ///@{

    template<typename T>
    void operator()(T& _t) {
      T* tptr = dynamic_cast<T*>(m_memory);
      if(tptr)
        tptr->Connect(m_rdmp, m_i1first, m_i1last,
            m_i2first, m_i2last, m_fromFullRoadmap, m_output);
    }

    ///@}

    private:

      ///@name Internal State
      ///@{

      CM* m_memory;
      RDMP* m_rdmp;
      I1 m_i1first, m_i1last;
      I2 m_i2first, m_i2last;
      bool m_fromFullRoadmap;
      O m_output;

      ///@}
  };

}


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref Connectors.
///
/// ConnectorMethod essentially has one important function, @c Connect which can
/// be called in a multitude of ways. In its basic forms it takes two sets of
/// configurations and generates edges in the roadmap between them.
/// @ingroup Connectors
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
#ifdef _PARALLEL
class ConnectorMethod : public MPBaseObject<MPTraits>, public stapl::p_object
#else
class ConnectorMethod : public MPBaseObject<MPTraits>
#endif
{
  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::pair<VID, VID>               ConnectionAttempt;
    typedef std::map<ConnectionAttempt, bool> ConnectionAttemptsCache;

    ///@}
    ///@name Construction
    ///@{

    ConnectorMethod(std::string _lpLabel = "", std::string _nfLabel = "");

    ConnectorMethod(XMLNode& _node);

    virtual ~ConnectorMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    virtual void Initialize() override;

    ///@}
    ///@name Connection Interface
    ///@{

    /// Generate edges between two sets of nodes. Uses the entire roadmap as the
    /// first and second set of nodes.
    template <typename OutputIterator = NullOutputIterator>
    void Connect(RoadmapType* _r, OutputIterator _collision = OutputIterator());

    /// Generate edges between two sets of nodes. Uses a single node as the
    /// first set of nodes and the entire roadmap as second set of nodes.
    template <typename OutputIterator = NullOutputIterator>
    void Connect(RoadmapType* _r, VID _vid,
        OutputIterator _collision = OutputIterator());

    /// Generate edges between two sets of nodes. Uses entire roadmap as second
    /// set of nodes.
    template <typename InputIterator,
              typename OutputIterator = NullOutputIterator>
    void Connect(RoadmapType* _r, InputIterator _itrFirst,
        InputIterator _itrLast, OutputIterator _collision = OutputIterator());

    /// Generate edges between two sets of nodes. Uses a single VID as the first
    /// set of nodes.
    template <typename InputIterator,
              typename OutputIterator = NullOutputIterator>
    void Connect(RoadmapType* _r, VID _vid,
        InputIterator _itrFirst, InputIterator _itrLast,
        bool _fromFullRoadmap, OutputIterator _collision = OutputIterator());

    /// Generate edges between two sets of nodes.
    /// @param _r The roadmap to generate edges in and where the input nodes
    ///           are found
    /// @param _itr1First Begin iterator of first set of VIDs
    /// @param _itr1Last End iterator of first set of VIDs
    /// @param _itr2First Begin iterator of second set of VIDs
    /// @param _itr2Last End iterator of second set of VIDs
    /// @param _fromFullRoadmap True if [_itr2First, _itr2Last) is the full
    ///                         roadmap. This implies using the saved internal
    ///                         NF model for finding neighbors with advanced
    ///                         NFMethods.
    /// @param _collision Output iterator to store collision witnesses
    template <typename InputIterator1, typename InputIterator2,
              typename OutputIterator = NullOutputIterator>
    void Connect(RoadmapType* _r,
        InputIterator1 _itr1First, InputIterator1 _itr1Last,
        InputIterator2 _itr2First, InputIterator2 _itr2Last,
        bool _fromFullRoadmap,
        OutputIterator _collision = OutputIterator());

    ///@}

  protected:

    ///@name Connection Caching
    ///@{

    /// Add connection attempt to the caches.
    /// @param _source The source VID.
    /// @param _target The target VID.
    /// @param _success True if the attempt to connect _source to _target
    ///                 succeeded, false otherwise.
    void AddConnectionAttempt(const VID _source, const VID _target,
        const bool _success) noexcept;

    /// Check if attempt is in cache
    /// @param _source The source VID.
    /// @param _target The target VID.
    /// @return True if the attempt to connect _source to _target is cached.
    bool IsCached(const VID _source, const VID _target) const noexcept;

    /// Check value of attempt in cache
    /// @param _source The source VID.
    /// @param _target The target VID.
    /// @return True if the cached attempt to connect _source to _target
    ///         succeeded.
    bool GetCached(const VID _source, const VID _target) const noexcept;

    /// Clear the attempts cache.
    void ClearConnectionAttempts();

    ///@}
    ///@name Internal State
    ///@{

    ConnectionAttemptsCache m_attemptsCache; ///< All time connection attempts.
    std::string m_nfLabel;                   ///< Neighborhood Finder
    std::string m_lpLabel;                   ///< Local Planner

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ConnectorMethod<MPTraits>::
ConnectorMethod(std::string _nfLabel, std::string _lpLabel) :
    m_nfLabel(_nfLabel), m_lpLabel(_lpLabel) {
}


template <typename MPTraits>
ConnectorMethod<MPTraits>::
ConnectorMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");
  m_lpLabel = _node.Read("lpLabel", true, "", "Local Planner");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
ConnectorMethod<MPTraits>::
Print(std::ostream& _os) const {
  MPBaseObject<MPTraits>::Print(_os);
  _os << "\tnfLabel: " << m_nfLabel << endl;
  _os << "\tlpLabel: " << m_lpLabel << endl;
}


template <typename MPTraits>
void
ConnectorMethod<MPTraits>::
Initialize() {
  ClearConnectionAttempts();
}

/*---------------------------- Connection Interface --------------------------*/

template <typename MPTraits>
template <typename OutputIterator>
void
ConnectorMethod<MPTraits>::
Connect(RoadmapType* _r, OutputIterator _collision) {
  GraphType* g = _r->GetGraph();
  Connect(_r, g->begin(), g->end(), g->begin(), g->end(), true, _collision);
}


template <typename MPTraits>
template <typename OutputIterator>
void
ConnectorMethod<MPTraits>::
Connect(RoadmapType* _r, VID _vid, OutputIterator _collision) {
  GraphType* g = _r->GetGraph();
  Connect(_r, _vid, g->begin(), g->end(), true, _collision);
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
void
ConnectorMethod<MPTraits>::
Connect(RoadmapType* _r, InputIterator _itrFirst, InputIterator _itrLast,
    OutputIterator _collision) {
  GraphType* g = _r->GetGraph();
  Connect(_r, _itrFirst, _itrLast, g->begin(), g->end(), true, _collision);
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
void
ConnectorMethod<MPTraits>::
Connect(RoadmapType* _r, VID _vid, InputIterator _itrFirst,
    InputIterator _itrLast, bool _fromFullRoadmap, OutputIterator _collision) {
  std::vector<VID> vids(1, _vid);
  Connect(_r, vids.begin(), vids.end(), _itrFirst, _itrLast,
      _fromFullRoadmap, _collision);
}


template <typename MPTraits>
template <typename InputIterator1, typename InputIterator2,
          typename OutputIterator>
void
ConnectorMethod<MPTraits>::
Connect(RoadmapType* _r,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap, OutputIterator _collision) {
  MethodTimer mt(this->GetStatClass(), this->GetName() + "::Connect");

  typedef typename MPTraits::ConnectorMethodList MethodList;
  boost::mpl::for_each<MethodList>(
      pmpl_detail::VirtualConnect<
      ConnectorMethod<MPTraits>, RoadmapType,
      InputIterator1, InputIterator2, OutputIterator>(
        this, _r, _itr1First, _itr1Last,
        _itr2First, _itr2Last, _fromFullRoadmap, _collision)
      );
}

/*------------------------------ Connection Caching --------------------------*/

template <typename MPTraits>
void
ConnectorMethod<MPTraits>::
AddConnectionAttempt(const VID _source, const VID _target, const bool _success)
    noexcept {
  m_attemptsCache[{_source, _target}] = _success;
}


template <typename MPTraits>
bool
ConnectorMethod<MPTraits>::
IsCached(const VID _source, const VID _target) const noexcept {
  return m_attemptsCache.count({_source, _target}) > 0;
}


template <typename MPTraits>
bool
ConnectorMethod<MPTraits>::
GetCached(const VID _source, const VID _target) const noexcept {
  try {
    return m_attemptsCache.at({_source, _target});
  }
  catch(const std::out_of_range&) {
    // If the attempt isn't in the cache, repropagate the out-of-range error
    // with WHERE info.
    throw RunTimeException(WHERE) << "Connection attempt (" << _source << ", "
                                  << _target << ") is not in the cache for "
                                  << this->GetNameAndLabel() << ".";
  }
}


template <typename MPTraits>
void
ConnectorMethod<MPTraits>::
ClearConnectionAttempts() {
  m_attemptsCache.clear();
}

/*----------------------------------------------------------------------------*/

#endif
