/////////////////////////////////////
//AdaptiveSubdivisionMethod.cpp
////////////////////////////////////

#include "AdaptiveSubdivisionMethod.h"
#include "BasicDecomposition.h"
#include "ConstructAdaptiveRegion.h"
#include "ConstructRegionMap.h"
#include "HierarchicalViewPartitioner.h"
#include "RegionMapConnect.h"


using namespace std;
using namespace stapl;
using namespace psbmp;


/// Edge functor (relationship at outer level):move to Hview file
struct EF {
  typedef int value_type;

  template<class Graph>
  void operator()(Graph* _g, size_t _lvl) const {
    size_t blk_sz = _g->num_vertices()/get_num_locations();
    size_t blk_bg = blk_sz*get_location_id();
    typename Graph::vertex_iterator vi = _g->begin(), vi_end = _g->end(), vi_temp;
    for (size_t i=0; i<blk_bg; ++i) { ++vi; }
    vi_temp = vi; ++vi_temp;

    for (size_t i=blk_bg; i<blk_bg+blk_sz; ++i) {
      if (vi_temp == vi_end)
        vi_temp = _g->begin();
      _g->add_edge_async((*vi).descriptor(), (*vi_temp).descriptor());
      ++vi, ++vi_temp;
    }
  }
};


AdaptiveSubdivisionMethod::AdaptiveSubdivisionMethod(XMLNode& _node, MPProblem* _problem) : MPStrategyMethod(_node, _problem) {
  ParseXML(_node);
}

AdaptiveSubdivisionMethod::~AdaptiveSubdivisionMethod() { }

void AdaptiveSubdivisionMethod::ParseXML(XMLNode& _node){
  XMLNode::childiterator citr;
  for( citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
    if(citr->getName() == "sequential_strategy") {
      string strategy_string = citr->Read(string("Strategy"), true,
        string(""), string("Sequential Strategy"));
      m_strategiesLabels.push_back(strategy_string);
      m_useOuterBB = citr->Read("useOuterBB", false, false, "if true, use outer boundary for region map constr");
      citr->warnUnrequestedAttributes();

    }else if(citr->getName() == "adaptive_region_constr") {
      string node_gen_method = citr->Read(string("Sampler"), true,
        string(""), string("Node Generation Method"));
      int numPerIteration = citr->Read(string("Number"), true,
        int(1), int(0), MAX_INT, string("Number of initial training  samples"));
      m_vecStrNodeGenLabels.push_back(pair<string, int>(node_gen_method, numPerIteration));
      m_dmm = citr->Read("DMM", true,
        "", "Distance metric for region construction");
      m_vcm = citr->Read("VCM", true,
	"", "Validity checker method for region construction");
      m_tk = citr->Read("KClosest", true,
        1, 0, MAX_INT, "training k closest to select in each region");
      m_tr = citr->Read("Radius", true,
	0.0, 0.0, 9999.0, "training radius for region construction");
      citr->warnUnrequestedAttributes();

    }else if(citr->getName() == "region_classifier") {
      double ratio = citr->Read("Ratio", true, 0.0, 0.0, 1.0, "Ratio of invalid to total nodes");
      int tries = citr->Read(string("Tries"), true,
        int(1), int(0), MAX_INT, string("Number of tries"));
      int samples = citr->Read(string("Samples"), true,
        int(1), int(0), MAX_INT, string("Number of additional samples for classification"));
      m_classifierParam = std::tr1::make_tuple(ratio,tries,samples);
      citr->warnUnrequestedAttributes();

    }else if(citr->getName() == "region_connection_method"){
      string connectRegionMethod = citr->Read(string("Method"), true,
        string(""), string("Region Connection Method"));
      m_regionConnectionLabels.push_back(connectRegionMethod);

      m_k1 = citr->Read("K1", true,
        1, 0, MAX_INT, "K Largest CC from source region");
      m_k2 = citr->Read("K2", true,
	1, 0, MAX_INT, "K Largest CC from target region");
      m_nf = citr->Read("NF", true,
	"", "Neighborhood Finder for Region Connect");
      m_lp = citr->Read("LP", true,
	"", "Local Planner for Region Connect");
      m_ccc = citr->Read("Type", true,
	"", "CC connection strategy option");

      citr->warnUnrequestedAttributes();

    }else if(citr->getName() == "num_row") {
      m_meshRow = citr->Read(string("Row"), true,
	int(1), int(1), MAX_INT, string("number of partition on x"));
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "n_col") {
      m_meshCol = citr->Read(string("Col"), true,
	int(1), int(1), MAX_INT, string("number of partition on y"));
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "num_runs") {
      n_runs = citr->Read(string("Runs"), true,
	int(1), int(0), MAX_INT, string("number of runs"));
      citr->warnUnrequestedAttributes();
    }else if(citr->getName() == "overlap") {
      // All 3 epsilon values are doubles between 0.0 to 1.0 with default value 0.0.
      m_xEpsilon = citr->Read("Xeps", true, 0.0, 0.0, 1.0, "x overlap percentage");
      m_yEpsilon = citr->Read("Yeps", true, 0.0, 0.0, 1.0, "y overlap percentage");
      m_zEpsilon = citr->Read("Zeps", true, 0.0, 0.0, 1.0, "z overlap percentage");
      citr->warnUnrequestedAttributes();

    }else {
     citr->warnUnknownNode();
    }
  }
};


void AdaptiveSubdivisionMethod::Initialize(int _regionID) {
}

void AdaptiveSubdivisionMethod::Run(int _regionID) {



  m_region = GetMPProblem()->GetMPRegion(_regionID);
  m_adaptiveRegion = m_region->GetAdaptiveRegionGraph();
  Environment* env = GetMPProblem()->GetEnvironment();
  RoadmapGraph<CfgType,WeightType>* rmg = m_region->GetRoadmap()->m_pRoadmap;
  BasicDecomposition* decomposer = new BasicDecomposition();
  shared_ptr<DistanceMetricMethod> dmm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dmm);
  shared_ptr<ValidityCheckerMethod> vcm = GetMPProblem()->GetValidityChecker()->GetVCMethod(m_vcm);
  typedef Connector<CfgType, WeightType> NC;
  typedef NC::ConnectionPointer NCP;
  NC* nc = GetMPProblem()->GetMPStrategy()->GetConnector();

  m_ccConnector = new ConnectCCs<CfgType,WeightType>(m_lp,m_nf);
  m_ccConnector->SetMPProblem(GetMPProblem());



  int mesh_size = m_meshRow * m_meshCol;
  int init_sample;
  shared_ptr<BoundingBox> bbox = env->GetBoundingBox();

  ////TYPEDEFS
  typedef Sampler<CfgType>::SamplerPointer sPointer;
  typedef vector<pair<string, int> >::iterator I;
  typedef vector<string>::iterator J, K;
  typedef std::tr1::tuple<double,int, int> constrParam;
  typedef ConnectCCs<CfgType, WeightType>* CCP;
  typedef std::tr1::tuple<NCP,string, int> connectParam;
  typedef std::tr1::tuple<CCP,string, int> ccConnectParam;

  typedef array<BoundingBox> arrayBbox;
  typedef array_view <arrayBbox> viewBbox;
  arrayBbox pArrayBbox(mesh_size,*bbox);

  ///TIMER STUFF

  stapl::counter<stapl::default_timer> t1, t2, t3, t4, t5;
  double regionConstr=0.0, rdmapConstr=0.0, regionConnect=0.0, total=0.0 , ccTime=0.0;

  t4.start();

  ////CONSTRUCT OUTER REGION
  t1.start();
  if( stapl::get_location_id() == 0){
    decomposer->DecomposeWS(env,*bbox, m_meshRow, m_meshCol,1, pArrayBbox.begin(), m_xEpsilon,m_yEpsilon, m_zEpsilon);
  }
  rmi_fence();

  PrintOnce("# OUTER REGIONS: ", mesh_size);
  rmi_fence();

  ////CONSTRUCT AND CLASSIFY INNER REGION

  viewBbox pViewBbox(pArrayBbox);

  for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr){
    sPointer sp = GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(itr->first);
    init_sample = itr->second;
    constrParam cParam = std::tr1::make_tuple(m_tr,init_sample,m_tk);
    ConstructAdaptiveRegion constrRegion(m_region,env,dmm,sp,vcm,cParam,m_classifierParam);
    new_algorithms::for_each(pViewBbox,constrRegion);
  }
  regionConstr = t1.stop();
  rmi_fence();

  PrintValue("REGION CONSTR: ", regionConstr);
  PrintOnce("# INNER REGIONS: ", m_adaptiveRegion->num_vertices());
  PrintOnce("INNER REGIONS EDGES: ", m_adaptiveRegion->num_edges());
  rmi_fence();

  t2.start();
  graph_view<RGraph> rgView(*m_adaptiveRegion);
  rmi_fence();

  ///////HIERARCHICAL VIEW AND CONSTRUCT REGION ROADMAP
  for(J itr1 = m_strategiesLabels.begin(); itr1 != m_strategiesLabels.end(); ++itr1) {

    MPStrategyMethod* strategy = GetMPProblem()->GetMPStrategy()->GetMPStrategyMethod(*itr1);
    ConstructOuterRegion constrRegionMap(m_region, env, strategy,vcm, m_useOuterBB);
    BoundaryPartitioner iPartitioner(pViewBbox);


    typedef typename hierarchical_view_type<graph_view<RGraph>, IndexPartitioner, EF>::type type;
    map_func(constrRegionMap, create_level(rgView, iPartitioner, EF()));

    rmi_fence();
  }
  int nedges = rmg->num_edges();
  rdmapConstr = t2.stop();
  rmi_fence();



  PrintValue("RDMAP CONSTR: ", rdmapConstr);


  /// COMPUTE CCs AND SET REGION CCs
  t5.start();
  typedef graph_view<RoadmapGraph<CfgType,WeightType> >  view_type;
  view_type rmView(*rmg);
  rmi_fence();
  typedef static_array<cc_color_property>           property_storage_type;
  typedef graph_external_property_map<view_type,
                                      cc_color_property,
                                      property_storage_type> property_map_type;

  ///TODO: proper fix by making cc_color_property derived from cfg class
  /// and then use internal_property_map
  property_storage_type prop_storage(2*rmView.size());
  property_map_type map(rmView, &prop_storage);

  connected_components(rmView, map);
  rmi_fence();

  std::vector<pair<VID,size_t> > ccVec1 = cc_stats(rmView,map);
  rmi_fence();

  PrintOnce("edges before region con:  " , rmg->num_edges());
  PrintOnce("cc count before region con: ", ccVec1.size());
  rmi_fence();
  ccTime = t5.stop();

  ////CONNECT REGION ROADMAPS
  t3.start();
  if(m_ccc == "largest"){
    ///SET REGION CC
    array_view<std::vector<pair<VID,size_t> > > ccView1(ccVec1);
    map_func(SetRegionCC(), rgView, balance_view(ccView1,rgView.size()));
    rmi_fence();

    // edge_set_view<RRGraph> regionEdgeView(*regularRegion); // edge set view not available in new container
    ccConnectParam conParam = std::tr1::make_tuple(m_ccConnector,m_ccc,m_k1);
    RegionCCConnector<RGraph,AdaptiveRegion<CfgType>,property_map_type> regionCon(m_region,m_adaptiveRegion,map,conParam);
    new_algorithms::for_each(rgView,regionCon);

  }else if(m_ccc == "random"){

    for(J itr2 = m_regionConnectionLabels.begin(); itr2 != m_regionConnectionLabels.end(); ++itr2){
      NCP ncp = nc->GetMethod(*itr2);
      connectParam conParam = std::tr1::make_tuple(ncp,m_ccc,m_k1);
      RegionRandomConnector<RGraph,AdaptiveRegion<CfgType> > regionCon(m_region,m_adaptiveRegion,conParam);
      new_algorithms::for_each(rgView,regionCon);
    }
  }else{
    cerr << "ERROR: Please choose an existing k_closest_cc connection type" << endl;
    cerr << "Reference this error on line" <<__LINE__ << " of file " << __FILE__ << endl;
    exit(-1);
  }

  regionConnect = t3.stop();
  rmi_fence();

  total = t4.stop();

  map.reset();
  connected_components(rmView, map);
  rmi_fence();
  std::vector<pair<VID,size_t> > ccVec2 = cc_stats(rmView,map);
  rmi_fence();

  PrintOnce("edges after region con:  " , rmg->num_edges());
  PrintOnce("cc count after region con: ", ccVec2.size());
  rmi_fence();

  PrintValue("REGION CONNECT: ", regionConnect);
  PrintValue("TOTAL: ", total);
  rmi_fence();



  ///DEBUG - WRITE USEFUL STATS

  if(get_location_id() == 0){

    std::sort(ccVec1.begin(), ccVec1.end(), RegionCCSort<pair<VID, size_t> >());
    std::sort(ccVec2.begin(), ccVec2.end(), RegionCCSort<pair<VID, size_t> >());
    stringstream basefname;
    //basefname << GetBaseFilename() << ".m" << mesh_size << ".p" << get_num_locations();
    basefname << GetBaseFilename() ;
    ofstream stat_out((basefname.str() + ".stats").c_str());

    stat_out << "#num_procs \t outer_regions \t inner_regions \t inner_region_edges "
      << "\t innerouter_ratio "
      << "\t avg_region_degree "
      << "\t init_samples \t tkclose \t tRadius \t threshold "
      << "\t rmap_size \t regionedges_added  \t total_rmap_edges "
      << "\t cc_countB4 \t LargestB4 \t cc_countAfter  \t LargestCCAfter"
      << "\t reg_const \t rmap_const \t cc_time \t reg_con \t total "
      << endl;

    stat_out << get_num_locations() << "\t" << mesh_size  << "\t" << m_adaptiveRegion->num_vertices() << "\t" << m_adaptiveRegion->num_edges()/2
      << "\t" << (int)(m_adaptiveRegion->num_vertices()/mesh_size)
      << "\t" << (int)(m_adaptiveRegion->num_edges()/(2*m_adaptiveRegion->num_vertices()))
      << "\t" << init_sample  << "\t" << m_tk  << "\t" << m_tr << "\t" << std::tr1::get<0>(m_classifierParam)
      << "\t" << rmg->num_vertices() << "\t" << rmg->num_edges() - nedges << "\t" << rmg->num_edges()
      << "\t" << ccVec1.size() << "\t" << ccVec1[0].second <<"\t" << ccVec2.size() << "\t" << ccVec2[0].second
      << "\t" << regionConstr << "\t" << rdmapConstr  << "\t" << ccTime << "\t" << regionConnect << "\t" << total
      << endl;

    stat_out.close();
  }

  rmi_fence();

  ///WRITE REGION GRAPH :: Move to finalize
  /*write_graph(rgView,"rgFile.out");
  rmi_fence();*/

}

void AdaptiveSubdivisionMethod::Finalize(int _regionID){
  cout << "AdaptiveSubdivisionMethod::Finalize()" << endl;
  ////Write region graph here
  string str;
  stringstream basefname;
  basefname << GetBaseFilename() << ".p" << stapl::get_num_locations();
  ofstream osMap((basefname.str() + ".rmap").c_str());
  if(!osMap){
    cerr << "ERROR::Can't open file. "<< endl;
    cerr << "Reference this error on line "<< __LINE__ << " of file " << __FILE__ << endl;
    exit(-1);
  }else{
    // region->WriteRoadmapForVizmo(osMap);
    write_graph(*m_adaptiveRegion,osMap);
    osMap.close();
  }
  rmi_fence();
  cout << "location [" << get_location_id() <<"] ALL FINISHED" << endl;
  rmi_fence();

}

void AdaptiveSubdivisionMethod::Print(ostream& _os) const {
  _os << "AdaptiveSubdivisionMethod:: Print \n";
}

