#include "RegularSubdivisionMethod.h"
#include "BasicDecomposition.h"
#include "ConstructRegionMap.h"
#include "RegionMapConnect.h"


RegularSubdivisionMethod::RegularSubdivisionMethod(XMLNodeReader& _node, MPProblem* _problem) : MPStrategyMethod(_node, _problem) {
  ////PMPLPrintString("RegularSubdivisionMethod::(XMLNodeReader,MPProblem)");
  ParseXML(_node);
}

RegularSubdivisionMethod::~RegularSubdivisionMethod() { }

void RegularSubdivisionMethod::ParseXML(XMLNodeReader& _node){
  XMLNodeReader::childiterator citr;
  for( citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
	if(citr->getName() == "sequential_strategy") {
	  string strategy_string = citr->stringXMLParameter(string("Strategy"), true,
		         string(""), string("Sequential Strategy"));
	  m_strategiesLabels.push_back(strategy_string);
	  citr->warnUnrequestedAttributes();
	}else if(citr->getName() == "node_generation_method") {
	  string node_gen_method = citr->stringXMLParameter(string("Method"), true,
	                 string(""), string("Node Generation Method"));
	  int numPerIteration = citr->numberXMLParameter(string("Number"), true, 
                         int(1), int(0), MAX_INT, string("Number of samples"));
	  m_vecStrNodeGenLabels.push_back(pair<string, int>(node_gen_method, numPerIteration));
	  citr->warnUnrequestedAttributes();
	} else if(citr->getName() == "node_connection_method") {
	  string connect_method = citr->stringXMLParameter(string("Method"), true,
			  string(""), string("Node Connection Method"));
	  m_vecStrNodeConnectionLabels.push_back(connect_method);
	  citr->warnUnrequestedAttributes();
	}else if(citr->getName() == "region_connection_method"){
		string connectRegionMethod = citr->stringXMLParameter(string("Method"), true, 
			string(""), string("Region Connection Method"));
		m_RegionConnectionLabels.push_back(connectRegionMethod);
		citr->warnUnrequestedAttributes();
	}else if(citr->getName() == "num_row") {
		 m_row = citr->numberXMLParameter(string("nRow"), true, 
			int(1), int(1), MAX_INT, string("number of partition on x"));
		citr->warnUnrequestedAttributes();
	}else if(citr->getName() == "n_col") {
		 m_col = citr->numberXMLParameter(string("nCol"), true, 
			int(1), int(1), MAX_INT, string("number of partition on y"));
		citr->warnUnrequestedAttributes();
	}else if(citr->getName() == "num_runs") {
		 m_runs = citr->numberXMLParameter(string("nRuns"), true, 
			int(1), int(0), MAX_INT, string("Runs number"));
		citr->warnUnrequestedAttributes();
	}else if(citr->getName() == "overlap") {
		 // All 3 epsilon values are doubles between 0.0 to 1.0 with default value 0.0.
		 m_xepsilon = citr->numberXMLParameter("xeps", true, 0.0, 0.0, 1.0, "x overlap percentage");
		 m_yepsilon = citr->numberXMLParameter("yeps", true, 0.0, 0.0, 1.0, "y overlap percentage");
		 m_zepsilon = citr->numberXMLParameter("zeps", true, 0.0, 0.0, 1.0, "z overlap percentage");
		 citr->warnUnrequestedAttributes();
	}else if(citr->getName() == "region_connect_k") {
		 m_k1 = citr->numberXMLParameter("k1", true, 
			1, 0, MAX_INT, "K Largest CC from source region");
		 m_k2 = citr->numberXMLParameter("k2", true, 
			1, 0, MAX_INT, "K Largest CC from target region");
		 m_nf = citr->stringXMLParameter("nf", true,
			"", "Neighborhood Finder for Region Connect");
		 m_ccc = citr->stringXMLParameter("type", true,
			"", "CC connection strategy option");

		 citr->warnUnrequestedAttributes();
	} else {
		citr->warnUnknownNode();
	}
  }
  
};


void RegularSubdivisionMethod::Initialize(int _regionID) {
  cout << "RegularSubdivisionMethod::Initialize()" <<endl;
}

void RegularSubdivisionMethod::Run(int _regionID) {
  
  cout << "RegularSubdivisionMethod:: Run()" << endl;
  
  m_region = GetMPProblem()->GetMPRegion(_regionID);
  Environment * env = GetMPProblem()->GetEnvironment();
  RoadmapGraph<CfgType,WeightType> * rmg = m_region->GetRoadmap()->m_pRoadmap; 
  BasicDecomposition* decomposer = new BasicDecomposition();
  shared_ptr<DistanceMetricMethod> dmm = GetMPProblem()->GetDistanceMetric()->GetMethod("scaledEuclidean");
  shared_ptr<ValidityCheckerMethod> vcm = GetMPProblem()->GetValidityChecker()->GetVCMethod("cd1");
  Connector<CfgType, WeightType>* nc = GetMPProblem()->GetMPStrategy()->GetConnector();
  
  
  typedef vector<pair<string, int> >::iterator I;
  typedef vector<string>::iterator J;
  typedef std::tr1::tuple<string,string, int> connectParam;
  typedef array<BoundingBox> arrayBbox;
  typedef array_view <arrayBbox> viewBbox;
  
  int mesh_size = m_row * m_col;
  int num_samples;
  
  shared_ptr<BoundingBox> bbox = env->GetBoundingBox();
  
  
  arrayBbox pArrayBbox(mesh_size,*bbox);
  
  
  ///MAKE A MESH GRAPH
  RRGraph regularRegion;
  add_edges_mesh<RRGraph> meshGraph(regularRegion,m_row,m_col);
  meshGraph.add_vertices();
  rmi_fence();
  meshGraph.add_edges();
  rmi_fence();
  
  graph_view<RRGraph> regionView(regularRegion);
  
  rmi_fence();
  
  ////DECOMPOSE SPACE TO REGIONS
 
  if( stapl::get_location_id() == 0){
    decomposer->DecomposeWS(env,*bbox,m_row, m_col,1, pArrayBbox.begin(), m_xepsilon,m_yepsilon, m_zepsilon);
  }
  rmi_fence();
  
  viewBbox arrView(pArrayBbox);
  rmi_fence();
  
  
   
  ////GENERATE NODES IN REGIONS
  for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr){
    num_samples = itr->second;  
    Sampler<CfgType>::SamplerPointer sp = GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(itr->first);
    NodeGenerator nodeGen(m_region,env,sp,vcm,num_samples);
    map_func(nodeGen,arrView,regionView);
  }
  rmi_fence();
  
  
  
  ///REDISTRIBUTE/MIGRATE - based on RegionVIDs and RegionWeight() 
  
  
  ///CONNECT NODES IN REGIONS
  NodeConnector nodeCon(m_region,nc);
  new_algorithms::for_each(regionView,nodeCon);
  rmi_fence();
  
  ///DEBUG
  PrintOnce("RUN::# of regions ", regularRegion.num_vertices());
  PrintOnce("RUN::# of region edges: ", regularRegion.num_edges());
  PrintOnce("RUN::roadmap graph size ", rmg->num_vertices());
  PrintOnce("RUN::roadmap graph edges before: ", rmg->num_edges());
  rmi_fence();
  
  /// COMPUTE CCs AND SET REGION CCs
  typedef graph_view<RoadmapGraph<CfgType,WeightType> >  view_type;
  view_type rmView(*rmg);
  rmi_fence();
  typedef static_array<cc_color_property>           property_storage_type;
  typedef graph_external_property_map<view_type,
                                      cc_color_property,
                                      property_storage_type> property_map_type;
				      
  ///TODO: proper fix by making cc_color_property derived from cfg class
  /// and then use internal_property_map
  property_storage_type prop_storage(2*num_samples*mesh_size);
  property_map_type     map(rmView, &prop_storage);
  
  
  connected_components(rmView, map);
  rmi_fence();
  
  
  std::vector<pair<VID,size_t> > ccVec1 = cc_stats(rmView,map);
  rmi_fence();
  
  PrintOnce("cc count before region con:", ccVec1.size());
  rmi_fence();
 
  array_view<std::vector<pair<VID,size_t> > > ccView1(ccVec1);
  map_func(SetRegionCC(), regionView, balance_view(ccView1,regionView.size()));
  rmi_fence();
  
   
  
  ///CONNECT REGIONS ROADMAP 
 // edge_set_view<RRGraph> regionEdgeView(*regularRegion); // edge set view not available in new container
 for(J itr2 = m_RegionConnectionLabels.begin(); itr2 != m_RegionConnectionLabels.end(); ++itr2){
   connectParam conParam = std::tr1::make_tuple(*itr2,m_ccc,m_k1);
   RegionConnector<RRGraph,Region,property_map_type> regionCon(m_region,&regularRegion,map, nc,conParam);
   new_algorithms::for_each(regionView,regionCon);
 }
 // rmi_fence();
  
  ///DEBUG
  PrintOnce("RUN::roadmap graph edges after: ", rmg->num_edges());
  rmi_fence();
  
  map.reset();
  connected_components(rmView, map);
  rmi_fence();
  std::vector<pair<VID,size_t> > ccVec2 = cc_stats(rmView,map);
  rmi_fence();
  
  PrintOnce("cc count after region con: ", ccVec2.size());
  rmi_fence();
   
  ///WRITE REGION GRAPH :: DEBUG
  write_graph(regionView,"rgFile.out");
  rmi_fence();
  
  //bbox->Clear();

}


void RegularSubdivisionMethod::Finalize(int _regionID){
  ///Write graph here :: DEBUG
  string str;
  stringstream basefname;
  basefname << GetBaseFilename() << ".p" << stapl::get_num_locations() << ".it" << m_runs;
  ofstream osMap((basefname.str() + ".map").c_str());
  if(!osMap){
     cout << "RegularSubdivisionMethod::Finalize(): can't open outfile: ";
     exit(-1);
  }else{
     m_region->WriteRoadmapForVizmo(osMap);
     osMap.close();
  }
  rmi_fence();
  cout << "location [" << get_location_id() <<"] ALL FINISHED" << endl;
}

void RegularSubdivisionMethod::PrintOptions(ostream& _os){
   _os << "RegularSubdivisionMethod:: PrintOptions \n";
}

