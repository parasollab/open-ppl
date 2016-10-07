#ifndef ParallelSRTStrategy_h
#define ParallelSRTStrategy_h

#include "ParallelSBMPHeader.h"
#include "NonDecomposition.h"
using namespace std;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class rmap_rrt_wf {
 private:
  typedef MPRegion<CfgType,WeightType>  MPR_type;
  typedef vector<pair<CfgType,vector<VID> > > CP_type;
  MPR_type* region;
  SRTStrategy* strategyMethod;
  CP_type* candPairs;
  int regionId;

 public:
  rmap_rrt_wf(MPR_type* _mpr, SRTStrategy* _mpsm, int _id, CP_type* pairs);

  void define_type(stapl::typer &t);

  rmap_rrt_wf(const rmap_rrt_wf& _wf, std::size_t offset);

  template<typename View, typename bbView>
    void operator()(const View& view,  bbView bb_view) const {
    int sample_size = view.size()/bb_view.size();
    cout << "\n processor #----->[" <<stapl::get_location_id() << "]  rmap_rrt_wf view_size/bb_view_size/sample_size: "
         << view.size() << "/"
         << bb_view.size() << "/"
         << sample_size << endl;
    stapl::counter<stapl::default_timer> t;
    double wf_timer=0.0;
    t.start();
    for(int i=0; i<bb_view.size(); ++i){
      cout << "rmap_rrt_wf (" << i << ") view size (sample_size):" << sample_size << endl;
      int index = i + (bb_view.size()*stapl::get_location_id());
      BoundingBox bb = bb_view[index];
      bb.Print(cout);
      strategyMethod->Run(regionId, candPairs);
    }
    wf_timer = t.stop();
    cout<<"\n processor #----->["<<stapl::get_location_id()<<"] rmap_rrt_wf time: " << wf_timer << endl;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class region_rrt_con_wf {
 private:
  typedef MPRegion<CfgType,WeightType>  MPR_type;
  typedef LocalPlanners<CfgType,WeightType> LP_type;
  typedef Connector<CfgType, WeightType>::ConnectionPointer CCM_type;
  MPR_type* region;
  LP_type*  lp;
  CCM_type pCCon;
  Environment* env;
  BoundingBox *bbox;

 public:
  region_rrt_con_wf(MPR_type* _mpr, LP_type* _plp, CCM_type _ccm, Environment* _penv, BoundingBox& _bbox);

  void define_type(stapl::typer &t);
  template <typename PartitionedView>
    void operator()(PartitionedView v1) const {


    LocalPlanners<CfgType,WeightType>* _lp = const_cast<LocalPlanners<CfgType,WeightType>*>(lp);
    Environment* _env = const_cast<Environment*>(env);
    vector<CfgType> collision;


    stapl::counter<stapl::default_timer> t;
    double wf_timer=0.0;
    t.start();


    for(typename PartitionedView::iterator vit = v1.begin(); vit  != v1.end(); ++vit) {


      for(typename PartitionedView::adj_edge_iterator ei = (*vit).begin(); ei != (*vit).end(); ++ei){

	SRTInfo srt_s = (*(region->GetSRTRegionGraph()->find_vertex((*ei).source()))).property();
	SRTInfo srt_t = (*(region->GetSRTRegionGraph()->find_vertex((*ei).target()))).property();

	vector<VID> sourceVIDs = srt_s.GetVIDs();
	vector<VID> targetVIDs = srt_t.GetVIDs();

stapl::sequential::vector_property_map<RoadmapGraph<CfgType, WeightType>::GRAPH,size_t > cmap;
  cmap.reset();
  pCCon->Connect(region->GetRoadmap(),
                        *(region->GetStatClass()),
                        cmap,
                        sourceVIDs.begin(),sourceVIDs.end(),
                        targetVIDs.begin(), targetVIDs.end());
    }

    wf_timer = t.stop();
    cout<<"\n processor #----->["<<stapl::get_location_id()<<"] rmap_rrt_con_wf time: " << wf_timer << endl;
}
  }
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class region_rrt_edge_wf {
 private:
  typedef MPRegion<CfgType,WeightType> MPR_type;
  typedef LocalPlanners<CfgType,WeightType> LP_type;
  typedef Connector<CfgType, WeightType>::ConnectionPointer CCM_type;
  MPR_type* region;
  LP_type*  lp;
  CCM_type pCCon;
  Environment* env;
  BoundingBox bbox;
  DistanceMetric* dm;
  string dm_label;
  int nc, nr;

 public:
  region_rrt_edge_wf(MPR_type* _mpr, LP_type* _plp,// CCM_type _ccm,
		     Environment* _penv, BoundingBox _bbox,
		     DistanceMetric* _dm, string _dm_label,
		     int _nc, int _nr);

  void define_type(stapl::typer &t);

  template <typename PartitionedView, typename rmView, typename CFG, typename WEIGHT>
    void operator()(PartitionedView v1, rmView v2) const {


    Environment* _env = const_cast<Environment*>(env);
    DistanceMetric* _dm = const_cast<DistanceMetric*>(dm);
    BoundingBox _bbox = bbox;
    shared_ptr<DistanceMetricMethod> _dmm = _dm->GetMethod(dm_label);
    vector<CfgType> collision;


    stapl::counter<stapl::default_timer> t;
    double wf_timer=0.0;
    t.start();


    for(typename PartitionedView::iterator vit = v1.begin(); vit  != v1.end(); ++vit) {
      SRTInfo rrt1 = (*vit).property();
      VID vid1 = (*vit).descriptor();
      vector<VID> nc_vid, nr_vid;
      vector<double> nc_dist, nr_dist;

      for(typename PartitionedView::iterator vit2 = v1.begin(); vit2 != v1.end(); ++vit2) {


	SRTInfo rrt2 = (*vit2).property();
	VID vid2 = (*vit2).descriptor();

	const CfgType c1 = rrt1.GetCandidate();
	const CfgType c2 = rrt2.GetCandidate();
	double dist = _dmm->Distance(_env,
        c1, c2);

	if (dist > 0) {
	  if (nc_vid.size() < nc) {
	    nc_vid.push_back(vid2);
	    nc_dist.push_back(dist);
	  } else {
	    double lm = nc_dist[0];
	    int lm_i = 0;
	    for (int i=1; i<nc_dist.size(); ++i) {
	      if (nc_dist[i] > lm) {
		lm   = nc_dist[i];
		lm_i = i;
	      }
	    }
	    if (dist < lm) {
	      nc_dist[lm_i] = dist;
	      nc_vid[lm_i] = vid2;
	    }
	  }
	}
      }
    }

    wf_timer = t.stop();
    cout<<"\n processor #----->["<<stapl::get_location_id()<<"] rmap_rrt_wf time: " << wf_timer << endl;

  }
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
template<typename View, typename bbView>
  void constructRRTRoadmap(View& view, bbView& bb_view, MPRegion<CfgType,WeightType>* _region,
			   SRTStrategy* _strategy, int in_regionID,
			   vector<pair<CfgType,vector<VID> > >* pairs) {
  rmap_rrt_wf wf(_region,_strategy,in_regionID,pairs);
  stapl::map_func(wf,stapl::balance_view(view,stapl::get_num_locations()),stapl::balance_view(bb_view,stapl::get_num_locations()));
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
template<typename PartitionedView>
  void connectRRTRegion(PartitionedView& v1, MPRegion<CfgType,WeightType>* _region,
			LocalPlanners<CfgType, WeightType>* _lp,Connector<CfgType,
			WeightType>::ConnectionPointer _ccm,
			Environment * _env, BoundingBox& bbox) {
  region_rrt_con_wf wf(_region,_lp,_ccm,_env,bbox);
  stapl::map_func(wf, v1);
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
template<typename PartitionedView, typename rmView>
  void determineRGEdges(PartitionedView& v1, rmView& v2,
			MPRegion<CfgType,WeightType>* _region,
			LocalPlanners<CfgType, WeightType>* _lp,
			Environment * _env, BoundingBox& bbox,
			DistanceMetric* dm, string dm_label,
			int nc, int nr) {
  region_rrt_edge_wf wf(_region,_lp,_env,bbox,dm,dm_label, nc, nr);
  stapl::map_func(wf, v1,v2);
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class ParallelSRTStrategy : public MPStrategyMethod {

 public:
  ParallelSRTStrategy(XMLNode& in_pNode, MPProblem* in_pProblem);

    virtual ~ParallelSRTStrategy();

    virtual void Print(ostream& out_os) const;
    virtual void Initialize(int in_RegionID);
    virtual double RRTDistance(SRTStrategy* srt, Environment* env, BoundingBox bb, CfgType c1, CfgType c2);

    virtual void ParseXML(XMLNode& in_pNode);

    virtual void Run(int in_RegionID) ;

    virtual void Finalize(int in_RegionID);

 private:

    vector<string> m_ComponentConnectionLabels;
    MPRegion<CfgType,WeightType>* region;
    RegionGraph<SRTInfo,WeightType>* rg;
    vector<string> m_strategiesLabels;
    int nc, nr, n_runs;
    string dm_label;
    StatClass * stats;
    RoadmapGraph<CfgType, WeightType>* cGraph;
};

#endif
