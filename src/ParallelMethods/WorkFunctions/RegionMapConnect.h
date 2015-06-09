#ifndef REGIONMAPCONNECTH
#define REGIONMAPCONNECT_H

#include "ParallelMethods/ParallelSBMPHeader.h"

///needed for proxy of pair specialization
#include <stapl/../tools/libstdc++/proxy/pair.h>

using namespace std;
using namespace stapl;
using namespace psbmp;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
template <class T>
struct RegionCCSort : public std::binary_function<T, T, bool> {
  bool operator()(T _x, T _y) { return _x.second > _y.second; }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
///DEGUG
struct Size_wf {
  typedef size_t result_type;
  template <typename T>
  result_type operator() (T _t) {
    PrintValue("size_wf: SIZE: ", _t.size());
    return _t.size();
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
template<class MPTraits>
struct LocVec {
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::VID VID;
  typedef vector<VID> result_type;

  template <typename View>
  result_type operator() (View _v) {
    return _v;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
template<class MPTraits>
struct MergeVec {
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::VID VID;
  typedef vector<VID> result_type;

  template <typename View1, typename View2>
  result_type operator() (View1 _vec1, View2 _vec2); 
};

template<class MPTraits>
template<typename View1, typename View2>
typename MergeVec<MPTraits>::result_type
MergeVec<MPTraits>::
operator() (View1 _vec1, View2 _vec2) {
  result_type v3(_vec1.size() + _vec2.size());

  vector<VID> v1 = _vec1;
  vector<VID> v2 = _vec2;
  std::merge(v1.begin(), v1.end(), v2.begin(), v2.end(), v3.begin());
  return v3;
}

/**
 * Custom vertex property that implements coloring should extend Cfg class
   for internal coloring
 */
////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
///
/// Custom vertex property that implements coloring should extend Cfg class
/// for internal coloring
struct cc_color_property {
  size_t color, cc;
  cc_color_property() : color(0), cc(0) { }
  cc_color_property(size_t _c, size_t _q) : color(_c), cc(_q) { }
  void set_color(size_t _c) { color = _c; }
  void set_cc(size_t _c) { cc = _c; }
  size_t get_color() const { return color; }
  size_t get_cc() const { return cc; }
  void define_type(typer& _t) { _t.member(cc); _t.member(color); }
};

namespace stapl {
template <typename Accessor>
class proxy<cc_color_property, Accessor> : public Accessor
{
  private:
    friend class proxy_core_access;
    typedef cc_color_property target_t;

  public:
    explicit proxy(Accessor const& _acc)
      : Accessor(_acc) { }

    operator target_t() const { return Accessor::read(); }

    proxy const& operator=(proxy const& _rhs) {
      Accessor::write(_rhs);
      return *this;
    }

    proxy const& operator=(target_t const& _rhs) {
      Accessor::write(_rhs);
      return *this;
    }

    void set_color(size_t _c) { Accessor::invoke(&target_t::set_color, _c); }
    size_t get_color() const { return Accessor::const_invoke(&target_t::get_color); }
    void set_cc(size_t _c) { Accessor::invoke(&target_t::set_cc, _c); }
    size_t get_cc() const { return Accessor::const_invoke(&target_t::get_cc); }
};
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
/// TODO: Combine the two set cc workfunctions
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
struct SetRegionCC{
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::VID VID;
  typedef void result_type;

  template <typename RGView, typename CCView>
  void operator() (RGView _v1, CCView _v2);
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
template <typename RGView, typename CCView>
void
SetRegionCC<MPTraits>::
operator()(RGView _v1, CCView _v2) {
  typedef typename CCView::iterator CCit;
  
  std::vector< pair<VID, size_t> > ccVec;
  for(CCit ccit = _v2.begin(); ccit != _v2.end(); ++ccit) {
    ccVec.push_back(*ccit);
  }
  _v1.property().SetCCs(ccVec);
}

template<class MPTraits>
struct SetRegionCCVIDS{

  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::VID VID;
  typedef void result_type;

  template <typename RGView, typename CCView>
  void operator()(RGView _v1, CCView _v2);
};

template<class MPTraits>
template <typename RGView, typename CCView>
void
SetRegionCCVIDS<MPTraits>::
operator()(RGView _v1, CCView _v2) {
  typedef typename CCView::iterator CCit;
  
  std::vector<VID> ccVec;
  for(CCit ccit = _v2.begin(); ccit != _v2.end(); ++ccit) {
    ccVec.push_back((*ccit).second);
  }
  _v1.property().SetVIDs(ccVec);
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<typename RGType, typename RType, class MPTraits>
class RegionCCConnector {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef CCsConnector<MPTraits>* NCP;
    typedef std::tuple<NCP,string, int> ConnectTuple;
    typedef void result_type;
    
    RegionCCConnector(MPProblemType* _problem, RGType* _g, ConnectTuple _ct)
      : m_problem(_problem), m_g(_g) {
        ///Tuple needs explicit assignment? //copy failed
      m_ct = _ct;
    }

    void define_type(stapl::typer &_t) {}

    template<typename RegionView>
    void operator()(RegionView _view) const;
  
  private:
    MPProblemType* m_problem;
    RGType* m_g;
    ConnectTuple m_ct;
};

template<typename RGType, typename RType, class MPTraits>
template<typename RegionView>
void
RegionCCConnector<RGType, RType, MPTraits>::
operator()(RegionView _view) const {
  typedef typename RegionView::adj_edges_type ADJV;
  /// CONNECTOR PARAMETERS
  NCP ncp = std::get<0>(m_ct);             /// connection method pointer
  string connectType = std::get<1>(m_ct); ///random,closest or largest
  int k = std::get<2>(m_ct);    ////k ccs or vids to attempt from each region

  typedef graph_view<GraphType> view_type;
  view_type rmView(*(m_problem->GetRoadmap()->GetGraph()));

  ADJV edges = _view.edges();

  //SOURCE REGION
  vector<pair<VID, size_t> > sCCs = _view.property().GetCCs();
  std::sort(sCCs.begin(), sCCs.end(), RegionCCSort<pair<VID, size_t> >());
  for(typename vector<pair<VID, size_t> >::iterator sIT = sCCs.begin();
      sIT != sCCs.begin() + std::min(static_cast<int>(sCCs.size()), k); 
      ++sIT) {
    vector<VID> sCand;
    sCand.clear();
    static_array<std::vector<VID> > sArrayCand(get_num_locations());
    array_view<static_array<std::vector<VID> > > scandView(sArrayCand);
    cc_stats(rmView,(*sIT).first,scandView);
    sCand = map_reduce(LocVec<MPTraits>(), MergeVec<MPTraits>(), scandView);

    //TARGET REGIONS
    for(typename RegionView::adj_edge_iterator ei = _view.begin();
        ei != _view.end(); ++ei) {
      RType tRegion = (*(m_g->find_vertex((*ei).target()))).property();
      vector<pair<VID, size_t> > tCCs = tRegion.GetCCs();
      vector<VID> tCand;
      std::sort(tCCs.begin(), tCCs.end(), RegionCCSort<pair<VID, size_t> >());
      tCand.clear();
      for(typename vector<pair<VID, size_t> >::iterator tIT = tCCs.begin();
          tIT != tCCs.begin() + std::min(static_cast<int>(tCCs.size()), k);
          ++tIT) {
        static_array<std::vector<VID> > tArrayCand(get_num_locations());
        array_view<static_array<std::vector<VID> > > tcandView(tArrayCand);

        ///@note crashes at p>2
        ///@todo 1: replace rmView with native_view and use sequential ccstats
        ///      2: call outside workfunction
        ///      3: implement and use inverse property map
        cc_stats(rmView,(*tIT).first,tcandView);
        tCand = map_reduce(LocVec<MPTraits>(), MergeVec<MPTraits>(), tcandView);

        //NOW CONNECT
        ///@todo : Check whether to connect small or big CCs

        vector<CfgType> col;
        ncp->ConnectCC(m_problem->GetRoadmap(), sCand, tCand, back_inserter(col));
      }
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<typename RGType, typename RType, class MPTraits>
class RegionRandomConnector
{
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::ConnectorPointer NCP;
    typedef std::tuple<NCP, string, int> ConnectTuple;
    typedef void result_type;
   
    RegionRandomConnector(MPProblemType* _problem, RGType* _g, ConnectTuple _ct)
      : m_problem(_problem), m_g(_g) {
        m_ct = _ct;
      }

    void define_type(stapl::typer& _t) {

    }

    template<typename regionView>
    void operator()(regionView _view) const;
  
  private:
    MPProblemType* m_problem;
    RGType* m_g;
    ConnectTuple m_ct;
};

template<typename RGType, typename RType, class MPTraits>
template<typename RegionView>
void
RegionRandomConnector<RGType, RType, MPTraits>::
operator() (RegionView _view) const {
  typedef typename RegionView::adj_edges_type ADJV;
  
  /// CONNECTOR PARAMETERS
  NCP ncp = std::get<0>(m_ct);             /// connection method pointer
  string connectType = std::get<1>(m_ct); ///random,closest or largest
  int  k = std::get<2>(m_ct);    ////k ccs or vids to attempt from each region

  //SOURCE REGION
  vector<VID> sCand;
  vector<VID> sVids = _view.property().RegionVIDs();
  std::random_shuffle(sVids.begin(), sVids.end());
  sCand.clear();
  for(typename vector<VID>::iterator sVIT = sVids.begin();
      sVIT != sVids.begin() + std::min(static_cast<int>(sVids.size()), k);
      ++sVIT) {
    sCand.push_back(*sVIT);
  }

  //TARGET REGION
  for(typename RegionView::adj_edge_iterator ei = _view.begin();
      ei != _view.end(); ++ei) {
    RType tRegion = (*(m_g->find_vertex((*ei).target()))).property();
    vector<VID> tCand = tRegion.RegionVIDs();

    /// NOW CONNECT
    ncp->Connect(m_problem->GetRoadmap(),
        sCand.begin(),sCand.end(),
        tCand.begin(),tCand.end());
  }
}

#endif

