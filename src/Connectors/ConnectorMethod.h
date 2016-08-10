#ifndef CONNECTION_METHOD_H_
#define CONNECTION_METHOD_H_

#include <boost/mpl/for_each.hpp>
#include "MPProblem/MPBaseObject.h"
#include "Utilities/MPUtils.h"
#include "Utilities/MetricUtils.h"

namespace pmpl_detail {

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @brief Facilitate function calls for general connection methods.
/// @tparam CM Connector method derived class type
/// @tparam RDMP Roadmap type
/// @tparam I1 Input iterator 1 type
/// @tparam I2 Input iterator 2 type
/// @tparam O Output iterator tyep
///
/// Facilitates simulation of pure virtualism of template functions in C++,
/// which is explicitly disallowed for the language. Essentially iterates over a
/// type list and attempts dynamic casts to that type to find appropriate
/// derived class.
////////////////////////////////////////////////////////////////////////////////
template<typename CM, typename RDMP, typename I1, typename I2, typename O>
  struct VirtualConnect{
    public:
      VirtualConnect(CM* _v, RDMP* _r, I1 _i1f, I1 _i1l,
          I2 _i2f, I2 _i2l, bool _b, O _o) :
        m_memory(_v), m_rdmp(_r), m_i1first(_i1f), m_i1last(_i1l),
        m_i2first(_i2f), m_i2last(_i2l), m_fromFullRoadmap(_b), m_output(_o) {
        }

      template<typename T>
        void operator()(T& _t) {
          T* tptr = dynamic_cast<T*>(m_memory);
          if(tptr != NULL){
            tptr->Connect(m_rdmp, m_i1first, m_i1last,
                m_i2first, m_i2last, m_fromFullRoadmap, m_output);
          }
        }

    private:
      CM* m_memory;
      RDMP* m_rdmp;
      I1 m_i1first, m_i1last;
      I2 m_i2first, m_i2last;
      bool m_fromFullRoadmap;
      O m_output;
  };

}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @brief Base algorithm abstraction for \ref Connectors.
/// @tparam MPTraits Motion planning universe
///
/// ConnectorMethod essentially has one important function, @c Connect which can
/// be called in a multitude of ways. In its basic forms it takes two sets of
/// configurations and generates edges in the roadmap between them.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
#ifdef _PARALLEL
class ConnectorMethod : public MPBaseObject<MPTraits>, public stapl::p_object
#else
class ConnectorMethod : public MPBaseObject<MPTraits>
#endif
{
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    ///Needed for BlindRRT
    typedef typename MPTraits::WeightType WeightType;
    typedef typename stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType, WeightType> SequentialGraphType;

    ConnectorMethod(string _lpLabel = "", string _nfLabel = "");
    ConnectorMethod(MPProblemType* _problem, XMLNode& _node);

    virtual void Print(ostream& _os) const;

    ////////////////////////////////////////////////////////////////////////////
    // Connection Methods
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    ///
    /// @overload
    /// Uses the entire roadmap as the first and second set of nodes.
    template<typename OutputIterator = NullOutputIterator>
      void Connect(RoadmapType* _rm,
          OutputIterator _collision = OutputIterator()) {
        GraphType* g = _rm->GetGraph();
        Connect(_rm, g->begin(), g->end(),
            g->begin(), g->end(), true, _collision);
      }


    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    ///
    /// @overload
    /// Uses a single node as the first set of nodes and the entire roadmap as
    /// second set of nodes.
    template<typename OutputIterator = NullOutputIterator>
      void Connect(RoadmapType* _rm, VID _vid,
          OutputIterator _collision = OutputIterator()) {
        GraphType* g = _rm->GetGraph();
        Connect(_rm, _vid, g->begin(), g->end(), true, _collision);
      }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    ///
    /// @overload
    /// Uses entire roadmap as second set of nodes.
    template<typename InputIterator,
      typename OutputIterator = NullOutputIterator>
        void Connect(RoadmapType* _rm,
            InputIterator _itrFirst, InputIterator _itrLast,
            OutputIterator _collision = OutputIterator()) {
          GraphType* g = _rm->GetGraph();
          Connect(_rm, _itrFirst, _itrLast,
              g->begin(), g->end(), true, _collision);
        }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    ///
    /// @overload
    /// Uses a single VID as the first set of nodes.
    template<typename InputIterator,
      typename OutputIterator = NullOutputIterator>
      void Connect(RoadmapType* _rm, VID _vid,
          InputIterator _itrFirst, InputIterator _itrLast,
          bool _fromFullRoadmap,
          OutputIterator _collision = OutputIterator()) {
        vector<VID> vids(1, _vid);
        Connect(_rm, vids.begin(), vids.end(),
            _itrFirst, _itrLast, _fromFullRoadmap, _collision);
      }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    /// @tparam InputIterator1 Iterator type of first set
    /// @tparam InputIterator2 Iterator type of second set
    /// @tparam OutputIterator Iterator type of output for collision nodes
    /// @param _rm The roadmap to generate edges in and where the input nodes
    ///        are found
    /// @param _itr1First Begin iterator of first set of VIDs
    /// @param _itr1Last End iterator of first set of VIDs
    /// @param _itr2First Begin iterator of second set of VIDs
    /// @param _itr2Last End iterator of second set of VIDs
    /// @param _fromFullRoadmap True if [_itr2First, _itr2Last) is the full
    ///                         roadmap. This implies using the saved internal
    ///                         NF model for finding neighbors with advanced
    ///                         NFMethods.
    /// @param _collision Output iterator to store collision witnesses
    ///
    /// @usage
    /// @code
    /// ConnectorPointer c = this->GetMPProblem()->GetConnector(m_cLabel);
    /// ColorMapType cm;
    /// vector<VID> c1, c2;
    /// bool b;
    /// vector<CfgType> col;
    /// c->Connect(this->GetMPProblem()->GetRoadmap(),
    ///            this->GetMPProblem()->GetStatClass(),
    ///            cmap, c1.begin(), c1.end(), c2.begin(), c2.end(),
    ///            b, back_inserter(col));
    /// @endcode
    template<typename InputIterator1, typename InputIterator2,
      typename OutputIterator = NullOutputIterator>
      void Connect(RoadmapType* _rm,
          InputIterator1 _itr1First, InputIterator1 _itr1Last,
          InputIterator2 _itr2First, InputIterator2 _itr2Last,
          bool _fromFullRoadmap,
          OutputIterator _collision = OutputIterator()) {
        typedef typename MPTraits::ConnectorMethodList MethodList;
        boost::mpl::for_each<MethodList>(
            pmpl_detail::VirtualConnect<
            ConnectorMethod<MPTraits>, RoadmapType,
            InputIterator1, InputIterator2, OutputIterator>(
              this, _rm, _itr1First, _itr1Last,
              _itr2First, _itr2Last, _fromFullRoadmap, _collision)
            );
      }

    typedef pair<VID, VID> ConnectionAttempt;
    typedef vector<pair<ConnectionAttempt, bool> > ConnectionAttempts;
    typedef map<ConnectionAttempt, bool> ConnectionAttemptsCache;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add connection attempt
    /// @param _v1 Source VID
    /// @param _v2 Target VID
    /// @param _b Success/failed connection attempt
    ///
    /// Add connection attempt to both iteration and all time connection attempt
    /// caches.
    ////////////////////////////////////////////////////////////////////////////
    void AddConnectionAttempt(VID _v1, VID _v2, bool _b);
    ////////////////////////////////////////////////////////////////////////////
    /// @return Begin iterator of this iteration's attempts
    ////////////////////////////////////////////////////////////////////////////
    typename ConnectionAttempts::const_iterator ConnectionAttemptsBegin() const { return m_attempts.begin(); }
    ////////////////////////////////////////////////////////////////////////////
    /// @return End iterator of this iteration's attempts
    ////////////////////////////////////////////////////////////////////////////
    typename ConnectionAttempts::const_iterator ConnectionAttemptsEnd() const { return m_attempts.end(); }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Check if attempt is in cache
    /// @param _v1 Source VID
    /// @param _v2 Target VID
    /// @return Yes/no attempt is cached
    ////////////////////////////////////////////////////////////////////////////
    bool IsCached(VID _v1, VID _v2);
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Check value of attempt in cache
    /// @param _v1 Source VID
    /// @param _v2 Target VID
    /// @return Success/failed connection attempt
    ////////////////////////////////////////////////////////////////////////////
    bool GetCached(VID _v1, VID _v2);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Clear this iteration's attempts cache
    ////////////////////////////////////////////////////////////////////////////
    void ClearConnectionAttempts() { m_attempts.clear(); }

    ///Needed for BlindRRT
    void SetLocalGraph(SequentialGraphType* _localGraph) { m_localGraph = _localGraph;}
    void SetRemoteGraph(SequentialGraphType* _remoteGraph) { m_remoteGraph = _remoteGraph;}

  protected:
    ConnectionAttempts m_attempts; ///< Single iteration connection attempts. Attempt is a pair<VID, VID> (edge) and bool (success/fail)
    ConnectionAttemptsCache m_attemptsCache; ///< All time connection attempts. Attempt is a pair<VID, VID> (edge) and bool (success/fail)
    string  m_nfLabel; ///< Neighborhood Finder
    string  m_lpLabel; ///< Local Planner

    ///Needed for BlindRRT
    SequentialGraphType* m_localGraph;
    SequentialGraphType* m_remoteGraph;
};

template<class MPTraits>
ConnectorMethod<MPTraits>::ConnectorMethod(string _nfLabel, string _lpLabel) :
  m_nfLabel(_nfLabel), m_lpLabel(_lpLabel) {
    this->SetName("ConnectorMethod");
  }

  template<class MPTraits>
ConnectorMethod<MPTraits>::ConnectorMethod(MPProblemType* _problem, XMLNode& _node)
  : MPBaseObject<MPTraits>(_problem, _node) {
    this->SetName("ConnectorMethod");
    m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");
    m_lpLabel = _node.Read("lpLabel", true, "", "Local Planner");
  }

template<class MPTraits>
void
ConnectorMethod<MPTraits>::Print(ostream& _os) const {
  MPBaseObject<MPTraits>::Print(_os);
  _os << "\tnfLabel: " << m_nfLabel << endl;
  _os << "\tlpLabel: " << m_lpLabel << endl;
}

template<class MPTraits>
void
ConnectorMethod<MPTraits>::AddConnectionAttempt(VID _v1, VID _v2, bool _b){
  ConnectionAttempt att(_v1, _v2);
  m_attempts.push_back(make_pair(att, _b));
  m_attemptsCache[att] = _b;
}

template<class MPTraits>
bool
ConnectorMethod<MPTraits>::IsCached(VID _v1, VID _v2) {
  ConnectionAttempt att(_v1, _v2);
  return m_attemptsCache.count(att) > 0;
}

template<class MPTraits>
bool
ConnectorMethod<MPTraits>::GetCached(VID _v1, VID _v2) {
  ConnectionAttempt att(_v1, _v2);
  return m_attemptsCache[att];
}

#endif

