#include "RegularSubdivisionMethod.h"
#include "ConstructRegionMap.h"
#include "BasicDecomposition.h"



using namespace std;
using namespace stapl;
using namespace psbmp;


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
	}else if(citr->getName() == "component_connection_method"){
		string connectCCMethod = citr->stringXMLParameter(string("Method"), true, 
			string(""), string("Component Connection Method"));
		m_ComponentConnectionLabels.push_back(connectCCMethod);
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

		 if(m_ccc != "closest" && m_ccc != "largest")
		 {
		    cerr << "ERROR::Please choose an existing k_closest_cc connection type" << endl;
		    cerr << "Reference this error on line " << __LINE__ << " of file " << __FILE__ <<
		    endl;
		    exit(-1);
		 }
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
  //WorkspaceDecomposition* decomposer = new WorkspaceDecomposition();
  shared_ptr<DistanceMetricMethod> dmm = GetMPProblem()->GetDistanceMetric()->GetMethod("scaledEuclidean");
  shared_ptr<ValidityCheckerMethod> vcm = GetMPProblem()->GetValidityChecker()->GetVCMethod("cd1");
  Connector<CfgType, WeightType>* nc = GetMPProblem()->GetMPStrategy()->GetConnector();
  
  int mesh_size = m_row * m_col;
  
  shared_ptr<BoundingBox> bbox = env->GetBoundingBox();
  
  typedef array<BoundingBox> arrayBbox;
  typedef array_view <arrayBbox> viewBbox;
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
  
  
  
  typedef vector<pair<string, int> >::iterator I;
   
  ////GENERATE NODES IN REGIONS
  for(I itr = m_vecStrNodeGenLabels.begin(); itr != m_vecStrNodeGenLabels.end(); ++itr){
    Sampler<CfgType>::SamplerPointer sp = GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(itr->first);
    NodeGenerator nodeGen(m_region,env,sp,vcm,itr->second);
    map_func(nodeGen,arrView,regionView);
  }
  rmi_fence();
  
  
  ///REDISTRIBUTE/MIGRATE - based on RegionVIDs and RegionWeight() 
  
  
  ///CONNECT NODES IN REGIONS
  NodeConnector nodeCon(m_region,nc);
  new_algorithms::for_each(regionView,nodeCon);
  rmi_fence();
  
  ///DEBUG
  PrintOnce("RUN::roadmap graph size ", rmg->num_vertices());
  PrintOnce("RUN::roadmap graph edges before: ", rmg->num_edges());
  rmi_fence();
   
  
  ///CONNECT REGIONS ROADMAP
 // edge_set_view<RRGraph> regionEdgeView(*regularRegion); // edge set view not available in new container
  RegionConnector regionCon(m_region,&regularRegion, nc,m_k1);
  new_algorithms::for_each(regionView,regionCon);
  rmi_fence();
  
  ///DEBUG
  PrintOnce("RUN::roadmap graph edges after: ", rmg->num_edges());
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

