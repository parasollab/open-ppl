#ifndef REGIONMAPCONNECTH
#define REGIONMAPCONNECT_H

#include "ParallelSBMPHeader.h"

using namespace std;
using namespace stapl;
using namespace psbmp;


typedef RoadmapGraph<CfgType, WeightType>::VID VID;



template <class T>
struct RegionCCSort : public std::binary_function<T, T, bool> {
  bool operator()(T x, T y) { return x.second > y.second; }
};

///DEGUG
struct size_wf {
  typedef size_t result_type;
  template <typename T>
  result_type operator() (T t) {
    PrintValue("size_wf: SIZE: ", t.size());
    //std::for_each(t.begin(), t.end(), cout << _1);
    //for(typename T::iterator it = t.begin(); it != t.end(); ++it)
      //PrintValue("size_wf: VID: ", *it);
    return t.size();
  }
};

struct locVec {
  typedef vector<VID> result_type;
  template <typename View>
  result_type operator() (View _v) {
    return _v;
  }
};

struct mergeVec {
  typedef vector<VID> result_type;
  template <typename View1, typename View2>
  result_type operator() (View1 vec1, View2 vec2) {
    result_type v3(vec1.size() + vec2.size());
    ///problem with const iterator, proxy issue try const cast
    //temporary fix- but bad for performance (copying)
    vector<VID> v1 = vec1;
    vector<VID> v2 = vec2;
    std::merge(v1.begin(),v1.end(),v2.begin(),v2.end(),v3.begin());
    return v3;
  }
};

/**
 * Custom vertex property that implements coloring should extend Cfg class 
   for internal coloring
 */
 struct cc_color_property {
  size_t color, cc;
  cc_color_property() : color(0), cc(0) { }
  cc_color_property(size_t c, size_t q) : color(c), cc(q) { }
  void set_color(size_t c) { color = c; }
  void set_cc(size_t c) { cc = c; }
  size_t get_color() const { return color; }
  size_t get_cc() const { return cc; }
  void define_type(typer& t) { t.member(cc); t.member(color); }
};

namespace stapl {
template <typename Accessor>
class proxy<cc_color_property, Accessor> 
  : public Accessor
{ 
private:
  friend class proxy_core_access;
  typedef cc_color_property   target_t;

public:
  explicit proxy(Accessor const& acc) 
    : Accessor(acc) { }

  operator target_t() const { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs) { 
    Accessor::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs) { 
    Accessor::write(rhs); 
    return *this; 
  }

  void set_color(size_t _c) { Accessor::invoke(&target_t::set_color, _c); }
  size_t get_color() const { return Accessor::const_invoke(&target_t::get_color); }
  void set_cc(size_t _c) { Accessor::invoke(&target_t::set_cc, _c); }
  size_t get_cc() const { return Accessor::const_invoke(&target_t::get_cc); }
};
};



struct SetRegionCC{
  template <typename RGView, typename CCView>
  void operator()(RGView _v1, CCView _v2) {
     std::vector<pair<VID,size_t> > ccVec;
     for(typename CCView::iterator ccit = _v2.begin(); ccit != _v2.end(); ++ccit){
       ccVec.push_back(*ccit);
      // PrintValue("SetRegionCC::CC VID: " , (*ccit).first);
      // PrintValue("SetRegionCC::CC size: " , (*ccit).second);
     }
    // TODO vector
    //_v1.property().SetCCs(ccVec);
    //PrintValue("SetRegionCC::CCsize: " , ccVec.size());
 }
};


template<typename RGType, typename RType, typename CMap>
class RegionCCConnector 
{
  private:
  typedef ConnectCCs<CfgType, WeightType>* NCP;
  typedef std::tr1::tuple<NCP,string, int> ConnectTuple;
  
  RGType* m_g;
  CMap m_cmap;
  ConnectTuple m_ct;
  MPProblem* m_problem;
  
  public:

  RegionCCConnector(MPProblem* _problem, RGType* _g,CMap _cmap, ConnectTuple _ct) :m_problem(_problem),m_g(_g), m_cmap(_cmap){
    ///Tuple needs explicit assignment? //copy failed
    m_ct = _ct;
  }
  
  void define_type(stapl::typer &_t)  
  {
  }
  
  template<typename regionView> 
  void operator()(regionView _view) const
  {
    /// CONNECTOR PARAMETERS
    
    NCP ncp = std::tr1::get<0>(m_ct);             /// connection method pointer
    string connectType = std::tr1::get<1>(m_ct); ///random,closest or largest
    int  k = std::tr1::get<2>(m_ct);    ////k ccs or vids to attempt from each region
    
    typedef graph_view<RoadmapGraph<CfgType,WeightType> >  view_type;
    view_type rmView(*(m_problem->GetRoadmap()->m_pRoadmap));
    
    
    typedef typename regionView::adj_edges_type ADJV;
    ADJV  edges = _view.edges();
    
       //SOURCE REGION
    vector<pair<VID, size_t> > sCCs = _view.property().GetCCs();
    std::sort(sCCs.begin(), sCCs.end(), RegionCCSort<pair<VID, size_t> >());
    for(vector<pair<VID, size_t> >::iterator sIT = sCCs.begin(); sIT != sCCs.begin() + std::min(static_cast<int>(sCCs.size()), k); ++sIT) {
      vector<VID> sCand;
      sCand.clear();
      static_array<std::vector<VID> > sArrayCand(get_num_locations());
      array_view<static_array<std::vector<VID> > > scandView(sArrayCand);
      cc_stats(rmView, m_cmap,(*sIT).first,scandView);
      sCand = map_reduce(locVec(),mergeVec(), scandView);
	 
	 //TARGET REGIONS
      for(typename regionView::adj_edge_iterator ei = edges.begin(); ei != edges.end(); ++ei){
        RType tRegion = (*(m_g->find_vertex((*ei).target()))).property();
        vector<pair<VID, size_t> > tCCs = tRegion.GetCCs();
        vector<VID> tCand;
        std::sort(tCCs.begin(), tCCs.end(), RegionCCSort<pair<VID, size_t> >());
        tCand.clear();
        for(vector<pair<VID, size_t> >::iterator tIT = tCCs.begin(); tIT != tCCs.begin() + std::min(static_cast<int>(tCCs.size()), k); ++tIT) {	
          static_array<std::vector<VID> > tArrayCand(get_num_locations());
          array_view<static_array<std::vector<VID> > > tcandView(tArrayCand);
          ///NOTE : crashes at p>2
          //\TODO : 1 replace rmView with native_view and use sequential ccstats
          //2: call outside workfunction
          //3: implement and use inverse property map
	  cc_stats(rmView, m_cmap,(*tIT).first,tcandView);
	  tCand = map_reduce(locVec(),mergeVec(), tcandView);
	     
          //NOW CONNECT
          ///\TODO : Check whether to connect small or big CCs
	     
          vector<CfgType> col;
	  //ncp->ConnectBigCC(m_region->GetRoadmap(),*(m_region->GetStatClass()), sCand, tCand, col);
	  //stapl::sequential::vector_property_map< RoadmapGraph<CfgType, WeightType>::GRAPH,size_t > cmap;
          /*ncp->Connect(m_region->GetRoadmap(),*(m_region->GetStatClass()), cmap,
	  sCand.begin(),sCand.end(),
	  tCand.begin(),tCand.end());*/
	  // PrintValue("LARGEST - source size : ", sCand.size());
          // PrintValue("LARGEST - target size : ", tCand.size());
	  ncp->ConnectBigCC(m_problem->GetRoadmap(),*(m_problem->GetStatClass()),sCand, tCand, col.begin());
	     
        }
    }
  }
    
 }
        
};


template<typename RGType, typename RType>
class RegionRandomConnector 
{
  private:
  typedef Connector<CfgType, WeightType>::ConnectionPointer NCP;
  typedef std::tr1::tuple<NCP,string, int> ConnectTuple;
  
  RGType* m_g;
  ConnectTuple m_ct;
  MPProblem* m_problem; 
 
  public:

  RegionRandomConnector(MPProblem* _problem, RGType* _g, ConnectTuple _ct) :m_problem(_problem),m_g(_g){
    m_ct = _ct;
  }
  
  
  void define_type(stapl::typer &_t)  
  {
  }
  
  template<typename regionView> 
  void operator()(regionView _view) const
  {
    /// CONNECTOR PARAMETERS
    NCP ncp = std::tr1::get<0>(m_ct);             /// connection method pointer
    string connectType = std::tr1::get<1>(m_ct); ///random,closest or largest
    int  k = std::tr1::get<2>(m_ct);    ////k ccs or vids to attempt from each region
    
    typedef typename regionView::adj_edges_type ADJV;
    ADJV  edges = _view.edges();
    
     //Algo: Randomly pick vid(cfg) in source region and attempt connection with k-closest vid (cfg) in target region 
      //SOURCE REGION
      vector<VID> sCand;
      vector<VID> sVids = _view.property().RegionVIDs(); 
      std::random_shuffle(sVids.begin(), sVids.end());
      sCand.clear();
      for(vector<VID>::iterator sVIT = sVids.begin(); sVIT != sVids.begin() + std::min(static_cast<int>(sVids.size()), k); ++sVIT) {
        sCand.push_back(*sVIT);
      }
      
      //TARGET REGION
      for(typename regionView::adj_edge_iterator ei = edges.begin(); ei != edges.end(); ++ei){
        RType tRegion = (*(m_g->find_vertex((*ei).target()))).property();
       // vector<VID> tCand;
        vector<VID> tCand = tRegion.RegionVIDs();
        //std::random_shuffle(tVids.begin(), tVids.end());
       // tCand.clear();
       //	for(vector<VID>::iterator tVIT = tVids.begin(); tVIT != tVids.begin() + std::min(static_cast<int>(tVids.size()), k); ++tVIT) {
	//  tCand.push_back(*tVIT);
       // }
      
      /// NOW CONNECT
      stapl::sequential::vector_property_map< RoadmapGraph<CfgType, WeightType>::GRAPH,size_t > cmap;
      ncp->Connect(m_problem->GetRoadmap(),*(m_problem->GetStatClass()), cmap,
	         sCand.begin(),sCand.end(),
	         tCand.begin(),tCand.end());
     // PrintValue("RANDOM - source size : ", sCand.size());
     // PrintValue("RANDOM - target size : ", tCand.size());
     // ncp->ConnectSmallCC(m_region->GetRoadmap(),*(m_region->GetStatClass()),sCand, tCand, col.begin());
       
     }
	     
  }
        
};


#endif

