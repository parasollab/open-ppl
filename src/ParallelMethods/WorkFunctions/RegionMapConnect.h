#ifndef REGIONMAPCONNECT_H
#define REGIONMAPCONNECT_H

#include "ParallelSBMPHeader.h"

using namespace std;
using namespace stapl;
using namespace psbmp;

typedef typename stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Region, WeightType> RRGraph;
typedef RoadmapGraph<CfgType, WeightType>::VID VID;



template <class T>
struct RegionCCSort : public std::binary_function<T, T, bool> {
  bool operator()(T x, T y) { return x.second > y.second; }
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
     }
    _v1.property().SetCCs(ccVec);
 }
};


template<typename RGType, typename RType, typename CMap>
class RegionConnector 
{
  private:
  typedef Connector<CfgType, WeightType> CON;
  typedef CON::ConnectionPointer CP;
  typedef MPRegion<CfgType,WeightType>  MPR;
  typedef std::tr1::tuple<string,string, int> ConnectTuple;
  
  
  
  MPR* m_region;
  RGType* m_g;
  CMap m_cmap;
  CON* m_con;
  ConnectTuple m_ct;
  
 
  public:

  RegionConnector(MPR* _mpr, RGType* _g,CMap _cmap, CON* _c, ConnectTuple _ct) :m_region(_mpr),m_g(_g), m_cmap(_cmap), m_con(_c){
    ///Tuple needs explicit assignment? //copy failed
    m_ct = _ct;
  }
  
  
  void define_type(stapl::typer &_t)  
  {
    _t.member(m_region);
  }
  
  template<typename regionView> 
  void operator()(regionView _view) const
  {
    
     
    /// CONNECTOR PARAMETERS
    string cm = std::tr1::get<0>(m_ct);             /// connection method
    string connectType = std::tr1::get<1>(m_ct); ///random,closest or largest
    int k = std::tr1::get<2>(m_ct);    ////k ccs or vids to attempt from each region
    
    CP cp = m_con->GetMethod(cm);
    typedef graph_view<RoadmapGraph<CfgType,WeightType> >  view_type;
    view_type rmView(*(m_region->GetRoadmap()->m_pRoadmap));
    ///SOURCE REGION
    vector<VID> sCand;
    if(connectType == "random"){
      vector<VID> sVids = _view.property().RegionVIDs(); 
      std::random_shuffle(sVids.begin(), sVids.end());
      sCand.clear();
      for(vector<VID>::iterator sVIT = sVids.begin(); sVIT != sVids.begin() + std::min(static_cast<int>(sVids.size()), k); ++sVIT) { 
        sCand.push_back(*sVIT);
      }
    }else if (connectType == "largest"){
       vector<pair<VID, size_t> > sCCs = _view.property().GetCCs();
       ///use lambda
       std::sort(sCCs.begin(), sCCs.end(), RegionCCSort<pair<VID, size_t> >());
       sCand.clear();
       for(vector<pair<VID, size_t> >::iterator sIT = sCCs.begin(); sIT != sCCs.begin() + std::min(static_cast<int>(sCCs.size()), k); ++sIT) {
         sCand.push_back((*sIT).first);
	 ///call get_cc here
      }
    }else{
	    cerr << "ERROR: Unknown connection type specified" << endl;
	exit(-1);
    }
    
    ////TARGET REGIONS 
    
    typedef typename regionView::adj_edges_type ADJV;
    ADJV  edges = _view.edges();
    for(typename regionView::adj_edge_iterator ei = edges.begin(); ei != edges.end(); ++ei){
      RType tRegion = (*(m_g->find_vertex((*ei).target()))).property();
      vector<VID> tCand;
      array_view<std::vector<VID> > tView(tCand);
      if(connectType == "random"){
        vector<VID> tVids = tRegion.RegionVIDs();
        std::random_shuffle(tVids.begin(), tVids.end());
        tCand.clear();
	for(vector<VID>::iterator tVIT = tVids.begin(); tVIT != tVids.begin() + std::min(static_cast<int>(tVids.size()), k); ++tVIT) {
	  tCand.push_back(*tVIT);
        }
      }else if (connectType == "largest"){
        vector<pair<VID, size_t> > tCCs = tRegion.GetCCs();
        ///use lambda
        std::sort(tCCs.begin(), tCCs.end(), RegionCCSort<pair<VID, size_t> >());
        tCand.clear();
        for(vector<pair<VID, size_t> >::iterator tIT = tCCs.begin(); tIT != tCCs.begin() + std::min(static_cast<int>(tCCs.size()), k); ++tIT) {	
	  tCand.push_back((*tIT).first);
	  //cc_stats(rmView, m_cmap,(*tIT).first,tView);
	  ///call get_cc here
        }
      }
      
      
    
      /// NOW CONNECT
     // temporary fix for Parallel code to compile
      stapl::sequential::vector_property_map< RoadmapGraph<CfgType, WeightType>::GRAPH,size_t > cmap;
      cp->Connect(m_region->GetRoadmap(),*(m_region->GetStatClass()), cmap,
	         sCand.begin(),sCand.end(),
	         tCand.begin(),tCand.end());
      
    }
	   
  }
        
};




#endif

