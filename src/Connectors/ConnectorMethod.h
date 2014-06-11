#ifndef CONNECTIONMETHOD_H_
#define CONNECTIONMETHOD_H_

#include <boost/mpl/for_each.hpp>
#include "Utilities/MPUtils.h"
#include "Utilities/MetricUtils.h"

namespace pmpl_detail{
  template<typename CM, typename RDMP, typename STATS, typename CMAP,
    typename I1, typename I2, typename O>
      struct VirtualConnect{
        public:
          VirtualConnect(CM* _v, RDMP* _r, STATS& _s, CMAP& _c,
              I1 _i1f, I1 _i1l, I2 _i2f, I2 _i2l, O _o) :
            m_memory(_v), m_rdmp(_r), m_stats(_s), m_cmap(_c), m_i1first(_i1f),
            m_i1last(_i1l), m_i2first(_i2f), m_i2last(_i2l), m_output(_o){
            }

          template<typename T>
            void operator()(T& _t) {
              T* tptr = dynamic_cast<T*>(m_memory);
              if(tptr != NULL){
                tptr->Connect(m_rdmp, m_stats, m_cmap, m_i1first, m_i1last,
                    m_i2first, m_i2last, m_output);
              }
            }
        private:
          CM* m_memory;
          RDMP* m_rdmp;
          STATS& m_stats;
          CMAP& m_cmap;
          I1 m_i1first, m_i1last;
          I2 m_i2first, m_i2last;
          O m_output;
      };
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @brief Base algorithm abstraction for \ref Connectors.
///
/// ConnectorMethod essentially has one important function, @c Connect which can
/// be called in a multitude of ways. In its basic forms it takes two sets of
/// configurations and generates edges in the roadmap between them.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
#ifdef _PARALLEL
class ConnectorMethod : public MPBaseObject<MPTraits>, public stapl::p_object {
#else
class ConnectorMethod : public MPBaseObject<MPTraits> {
#endif
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;

    ConnectorMethod(string _lpLabel = "", string _nfLabel = "");
    ConnectorMethod(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void Print(ostream& _os) const;

    ////////////////////////////////////////////////////////////////////////////
    // Connection Methods
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    ///
    /// @overload
    /// Uses the entire roadmap as the first and second set of nodes and no
    /// output of collision witnesses.
    ////////////////////////////////////////////////////////////////////////////
    template<typename ColorMap>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap){
        vector<CfgType> collision;
        Connect(_rm, _stats, _cmap, back_inserter(collision));
      }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    ///
    /// @overload
    /// Uses the entire roadmap as the first and second set of nodes.
    ////////////////////////////////////////////////////////////////////////////
    template<typename ColorMap, typename OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          OutputIterator _collision){
        Connect(_rm, _stats, _cmap,
            _rm->GetGraph()->begin(), _rm->GetGraph()->end(),
            _rm->GetGraph()->begin(), _rm->GetGraph()->end(),
            _collision);
      }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    ///
    /// @overload
    /// Uses a single node as the first set of nodes, the entire roadmap as
    /// second set of nodes, and no output of collision witnesses.
    ////////////////////////////////////////////////////////////////////////////
    template<typename ColorMap>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap, VID _vid){
        vector<CfgType> collision;
        Connect(_rm, _stats, _cmap, _vid, back_inserter(collision));
      }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    ///
    /// @overload
    /// Uses a single node as the first set of nodes and the entire roadmap as
    /// second set of nodes.
    ////////////////////////////////////////////////////////////////////////////
    template<typename ColorMap, typename OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          VID _vid, OutputIterator _collision){
        Connect(_rm, _stats, _cmap, _vid, _rm->GetGraph()->begin(), _rm->GetGraph()->end(), _collision);
      }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    ///
    /// @overload
    /// Uses entire roadmap as second set of nodes and no output of collision
    /// witnesses.
    ////////////////////////////////////////////////////////////////////////////
    template<typename ColorMap, typename InputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          InputIterator _itrFirst, InputIterator _itrLast){
        vector<CfgType> collision;
        Connect(_rm, _stats, _cmap, _itrFirst, _itrLast, back_inserter(collision));
      }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    ///
    /// @overload
    /// Uses entire roadmap as second set of nodes.
    ////////////////////////////////////////////////////////////////////////////
    template<typename ColorMap, typename InputIterator, typename OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          InputIterator _itrFirst, InputIterator _itrLast,
          OutputIterator _collision){
        Connect(_rm, _stats, _cmap,
            _itrFirst, _itrLast,
            _rm->GetGraph()->begin(), _rm->GetGraph()->end(),
            _collision);
      }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    ///
    /// @overload
    /// Uses a single VID as the first set of nodes and no output of collision
    /// witnesses.
    ////////////////////////////////////////////////////////////////////////////
    template<typename ColorMap, typename InputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          VID _vid,
          InputIterator _itrFirst, InputIterator _itrLast){
        vector<CfgType> collision;
        Connect(_rm, _stats, _cmap, _vid, _itrFirst, _itrLast, back_inserter(collision));
      }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    ///
    /// @overload
    /// Uses a single VID as the first set of nodes.
    ////////////////////////////////////////////////////////////////////////////
    template<typename ColorMap, typename InputIterator, typename OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          VID _vid,
          InputIterator _itrFirst, InputIterator _itrLast,
          OutputIterator _collision){
        vector<VID> vids(1, _vid);
        Connect(_rm, _stats, _cmap, vids.begin(), vids.end(), _itrFirst, _itrLast, _collision);
      }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    ///
    /// @overload
    /// No output of collision witnesses.
    ////////////////////////////////////////////////////////////////////////////
    template<typename ColorMap, typename InputIterator1, typename InputIterator2>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          InputIterator1 _itr1First, InputIterator1 _itr1Last,
          InputIterator2 _itr2First, InputIterator2 _itr2Last){
        vector<CfgType> collision;
        Connect(_rm, _stats, _cmap, _itr1First, _itr1Last, _itr2First, _itr2Last, back_inserter(collision));
      }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Generate edges between two sets of nodes
    /// @param _rm The roadmap to generate edges in and where the input nodes
    ///        are found
    /// @param _stats The stat class inside the MPProblem
    /// @param _cmap A color map over the underlying RoadmapGraph, used in CC
    ///        computations
    /// @param _itr1First Begin iterator of first set of VIDs
    /// @param _itr1Last End iterator of first set of VIDs
    /// @param _itr2First Begin iterator of second set of VIDs
    /// @param _itr2Last End iterator of second set of VIDs
    /// @param _collision Output iterator to store collision witnesses
    ///
    /// @usage
    /// @code
    /// ConnectorPointer c = this->GetMPProblem()->GetConnector(m_cLabel);
    /// ColorMapType cm;
    /// vector<VID> c1, c2;
    /// vector<CfgType> col;
    /// c->Connect(this->GetMPProblem()->GetRoadmap(),
    ///            this->GetMPProblem()->GetStatClass(),
    ///            cmap, c1.begin(), c1.end(), c2.begin(), c2.end(),
    ///            back_inserter(col));
    /// @endcode
    ////////////////////////////////////////////////////////////////////////////
    template<typename ColorMap, typename InputIterator1, typename InputIterator2, typename OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          InputIterator1 _itr1First, InputIterator1 _itr1Last,
          InputIterator2 _itr2First, InputIterator2 _itr2Last,
          OutputIterator _collision){
        typedef typename MPTraits::ConnectorMethodList MethodList;
        boost::mpl::for_each<MethodList>(
            pmpl_detail::VirtualConnect<
            ConnectorMethod<MPTraits>, RoadmapType, StatClass, ColorMap,
            InputIterator1, InputIterator2, OutputIterator>(
              this, _rm, _stats, _cmap, _itr1First, _itr1Last,
              _itr2First, _itr2Last, _collision)
            );
      }

    /////////////////////////////////////////////
    // Utility Methods
    typedef pair<VID, VID> ConnectionAttempt;
    void AddConnectionAttempt(VID _v1, VID _v2, bool _b);

    //Connection Attempts storage. Used for a single connection iteration
    typedef vector<pair<ConnectionAttempt, bool> > ConnectionAttempts;
    typename ConnectionAttempts::const_iterator ConnectionAttemptsBegin() const { return m_attempts.begin(); }
    typename ConnectionAttempts::const_iterator ConnectionAttemptsEnd() const { return m_attempts.end(); }
    void ClearConnectionAttempts() { m_attempts.clear(); }

    //Connection attempt cache storage. Used for all time
    typedef map<ConnectionAttempt, bool> ConnectionAttemptsCache;
    bool IsCached(VID _v1, VID _v2);
    bool GetCached(VID _v1, VID _v2);

  protected:
    ConnectionAttempts m_attempts;
    ConnectionAttemptsCache m_attemptsCache;
    string  m_nfLabel;
    string  m_lpLabel;
    bool    m_addPartialEdge;
};

template<class MPTraits>
ConnectorMethod<MPTraits>::ConnectorMethod(string _nfLabel, string _lpLabel) :
  m_nfLabel(_nfLabel), m_lpLabel(_lpLabel),
  m_addPartialEdge(false) {
    this->SetName("ConnectorMethod");
    m_addPartialEdge = false;
}

template<class MPTraits>
ConnectorMethod<MPTraits>::ConnectorMethod(MPProblemType* _problem, XMLNodeReader& _node)
  : MPBaseObject<MPTraits>(_problem, _node) {
    this->SetName("ConnectorMethod");
    m_addPartialEdge = false;
    m_nfLabel = _node.stringXMLParameter("nfLabel", true, "", "Neighborhood Finder");
    m_lpLabel = _node.stringXMLParameter("lpLabel", true, "", "Local Planner");
}

template<class MPTraits>
void
ConnectorMethod<MPTraits>::Print(ostream& _os) const {
  _os << "Name: " << this->GetNameAndLabel() << endl;
  _os << "\tnfLabel: " << m_nfLabel << endl;
  _os << "\tlpLabel: " << m_lpLabel << endl;
  _os << "\taddPartialEdge: " << m_addPartialEdge << endl;
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

