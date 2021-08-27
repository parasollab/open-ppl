#include "ParallelSRTStrategy.h"
#include <stapl/views/repeated_view.hpp>

//using namespace stapl;
//using namespace std;

//debut class rmap_rrt_wf
  rmap_rrt_wf::rmap_rrt_wf(MPR_type* _mpr, SRTStrategy* _mpsm, int _id, CP_type* pairs) {
    //cout << "rmap wf constructor" << endl;
    region = _mpr;
    strategyMethod = _mpsm;
    regionId = _id;
    candPairs = pairs;
  }

  void rmap_rrt_wf::define_type(stapl::typer &t) {
    t.member(region);
  }

  rmap_rrt_wf::rmap_rrt_wf(const rmap_rrt_wf& _wf, std::size_t offset)  {}
//fin class rmap_rrt_wf

//debut class region_rrt_con_wf
  region_rrt_con_wf::region_rrt_con_wf(MPR_type* _mpr, LP_type* _plp, CCM_type _ccm, Environment* _penv, BoundingBox& _bbox) {
    region = _mpr;
    lp = _plp;
    pCCon = _ccm;
    bbox = &_bbox;
  }

  void region_rrt_con_wf::define_type(stapl::typer &t) {
    t.member(region);
    t.member(bbox);
  }
//fin class region_rrt_con_wf


//debut class region_rrt_edge_wf
  region_rrt_edge_wf::region_rrt_edge_wf(MPR_type* _mpr, LP_type* _plp,// CCM_type _ccm,
		     Environment* _penv, BoundingBox _bbox,
		     DistanceMetric* _dm, string _dm_label,
		     int _nc, int _nr) {
    region = _mpr;
    lp = _plp;
    env = _penv;
    bbox = _bbox;
    dm = _dm;
    dm_label = _dm_label;
    nc = _nc;
    nr = _nr;
  }

  void region_rrt_edge_wf::define_type(stapl::typer &t) {
    t.member(region);
    t.member(bbox);
  }
//fin class region_rrt_edge_wf

//debut class ParallelRRTRoadmap
  //ParallelRRTRoadmap::ParallelRRTRoadmap(XMLNode& in_pNode, MPProblem* in_pProblem) :
   ParallelSRTStrategy::ParallelSRTStrategy(XMLNode& in_pNode, MPProblem* in_pProblem) :
    MPStrategyMethod(in_pNode,in_pProblem) {
    //LOG_DEBUG_MSG("ParallelRRTRoadmap::ParallelRRTRoadmap()");
    ParseXML(in_pNode);
    cout << "testconstr" << endl;
  };

    //virtual ~ParallelRRTRoadmap::ParallelRRTRoadmap() {}
    ParallelSRTStrategy::~ParallelSRTStrategy() {}

    void ParallelSRTStrategy::Print(ostream& out_os) const {
      cout << "test printoption" << endl;
      }
    void ParallelSRTStrategy::Initialize(int in_RegionID){
        cout << "test init" << endl;
      }
    double ParallelSRTStrategy::RRTDistance(SRTStrategy* srt, Environment* env, BoundingBox bb, CfgType c1, CfgType c2) {
    }

    void ParallelSRTStrategy::ParseXML(XMLNode& in_pNode) {
      //LOG_DEBUG_MSG("ParallelRRTRoadmap::ParseXML()");
     cout << "test parallel srt" << endl;
      XMLNode::childiterator citr;
      for( citr = in_pNode.children_begin(); citr!= in_pNode.children_end(); ++citr) {
	if (citr->getName() == "sequential_strategy") {
	  string strategy_string = citr->Read(string("Strategy"), true,
							    string(""), string("Sequential Strategy"));
	  m_strategiesLabels.push_back(strategy_string);
	  citr->warnUnrequestedAttributes();
	} else if (citr->getName() == "component_connection_method"){
	  string connectCCMethod = citr->Read(string("Method"), true,
							    string(""), string("Component Connection Method"));
	  m_ComponentConnectionLabels.push_back(connectCCMethod);
	  citr->warnUnrequestedAttributes();
	} else if(citr->getName()=="dm_label") {
	  dm_label = citr->Read(string("Method"),true,string(""),string("Distance Metric"));
	  citr->warnUnrequestedAttributes();
	} else if(citr->getName()=="nc") {
	  nc = citr->Read(string("Number"), true,
					int(1), int(0), MAX_INT, string("Number of close candidates for SRTRegion"));
	  citr->warnUnrequestedAttributes();
	} else if(citr->getName()=="nr") {
	  nr = citr->Read(string("Number"), true,
					int(1), int(0), MAX_INT, string("Number of random candidates for SRTRegion"));
	  citr->warnUnrequestedAttributes();
	} else if(citr->getName() == "num_runs") {
	  n_runs = citr->Read(string("nRuns"), true,
					    int(1), int(0), MAX_INT, string("Runs number"));
	  citr->warnUnrequestedAttributes();
	} else {
	  citr->warnUnknownNode();
	}
      }
     // LOG_DEBUG_MSG("ParallelRRTRoadmap::ParseXML()");
    }

    void ParallelSRTStrategy::Run(int in_RegionID) {
     cout << "testrun" << endl;
      // Problem Setup
      //LOG_DEBUG_MSG("ParallelRRTRoadmap::Run()");
      //OBPRM_srand(getSeed());
      region = GetMPProblem()->GetMPRegion(in_RegionID);
      stats = region->GetStatClass();
      stats->ClearStats();
      Environment * pEnv = GetMPProblem()->GetEnvironment();
      LocalPlanners<CfgType, WeightType>* pLp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
      DistanceMetric* dm = GetMPProblem()->GetDistanceMetric();
      shared_ptr <DistanceMetricMethod> dmm = GetMPProblem()->GetDistanceMetric()->GetMethod(dm_label);
     cout << "testrun2" << endl;
      // Candidate Setup
      stapl::p_vector<pair<VID,VID> > candPairs();
      cGraph = new RoadmapGraph<CfgType,WeightType>();
      typedef vector<pair<CfgType,vector<VID> > >::iterator CP_iter;
      vector<pair<CfgType,vector<VID> > > Cpairs;
      vector<pair<CfgType,vector<VID> > >* pairs = &Cpairs;

      // RoadmapGraph
      RoadmapGraph<CfgType,WeightType> * rmg = region->GetRoadmap()->m_pRoadmap;
      NonDecomposition<CfgType>* decomposer = new NonDecomposition<CfgType>();
      shared_ptr<BoundingBox> bbox = pEnv->GetBoundingBox();
      rg = region->GetSRTRegionGraph();
      int mesh_size = stapl::get_num_locations();
cout << "testrun3" << endl;
      // Timing
      stapl::counter<stapl::default_timer> t1,t2, t3, t4, t5;
      double construct_timer=0.0, integrator_timer=0.0, srt_construct=0.0, srt_edge_timer=0.0, total_timer=0.0;
      //TODO number of iteration hard coded should be removed
      for(int it =1; it<= 10; ++it) {

	//----------------------
	// Decompose space
	//----------------------
	cout << "test in run 6" << endl;
        stapl::p_array<BoundingBox> vbox_arr(mesh_size,*bbox);
	if ( stapl::get_location_id() == 0) {
	  decomposer->DecomposeWS(pEnv,*bbox, mesh_size, 1, 1, vbox_arr.begin());
	}

	typedef vector<string>::iterator I,J, K;

	//---------------------------
	// Generate && connect regional(local) roadmap
	//---------------------------
	string srt = "SRT";
	SRTStrategy* strategy = dynamic_cast<SRTStrategy*>(GetMPProblem()->GetMPStrategy()->GetMPStrategyMethod(srt));
	//TO DO number of samples below hard coded should be replaced
	stapl::p_array<CfgType> PA(10);
	stapl::array_1D_view<stapl::p_array<CfgType> > v(PA);
	stapl::array_1D_view<stapl::p_array<BoundingBox> > bb_view(vbox_arr);
	t5.start();
	t1.start();
	constructRRTRoadmap(v,bb_view,region,strategy,in_RegionID, pairs);
	construct_timer = t1.stop();

	cout<<"\n processor #----->["<<stapl::get_location_id()<<"] roadmap_construct_time = "  << construct_timer << endl;
	strategy->SetBoundary(bbox);
cout << "testrun4" << endl;
	//---------------------------
	// Create SRTInfo and add to SRTRegionGraph
	//---------------------------
	t2.start();
	double Xmin = bbox->GetRange(0).first, Xmax = bbox->GetRange(0).second;
	double Ymin = bbox->GetRange(1).first, Ymax = bbox->GetRange(1).second;
	double Zmin = bbox->GetRange(2).first, Zmax = bbox->GetRange(2).second;

	for (int l=0; l<Cpairs.size(); ++l) {
	  BoundingBox bb(bbox->GetDOFs(),bbox->GetPosDOFs());
	  bb.SetParameter(0,Xmin, Xmax);
	  bb.SetParameter(1,Ymin, Ymax);
	  bb.SetParameter(2,Zmin, Zmax);
	  SRTInfo srtinfo(bb,Cpairs[l]);

	  VID regionVID = region->GetSRTRegionGraph()->add_vertex(srtinfo);
	}
	srt_construct = t2.stop();

	cout<<"\n processor #----->["<<stapl::get_location_id()<<"] srt_region_construct_time  = "  << srt_construct << endl;
	strategy->SetBoundary(bbox);

	stapl::rmi_fence(); // Make sure processors don't start early on cand graph

	//===============
	// Find K-Closest and connect roadmap graph
	//================
	//cout << "Find K-Closest and Add Edges to attempt connection..." << endl;

	// USING 1 PROCESSOR

	if ( stapl::get_location_id() == 0) {

	  stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
	//  cmap.reset();
	  cout << " !! Useful Stats !! "<< endl;
	  cout << region->GetRoadmap()->m_pRoadmap->get_num_vertices() << " Vertices \n"
	       << region->GetRoadmap()->m_pRoadmap->get_num_edges() << " Edges \n "
	       << get_cc_count(*(region->GetRoadmap()->m_pRoadmap), cmap) << " Connected Components"
	       << endl;
	//  cmap.reset();

	  //cout << "!!! USING ONE (1) PROCESSOR !!!" << endl;

	  t3.start();

	  RegionGraph<SRTInfo,WeightType>* temp_rg = region->GetSRTRegionGraph();
	  RegionGraph<SRTInfo,WeightType>::vertex_iterator vi_1, vi_2;
	  shared_ptr<BoundingBox> bbox = pEnv->GetBoundingBox();
          //BoundingBox bbx = bbox;


	  stapl::sequential::vector_property_map< GRAPH,size_t > cmap1;
	//  cmap1.reset();
	  cout << " !! Region Graph Useful Stats !! "<< endl;
	  cout << region->GetRoadmap()->m_pRoadmap->get_num_vertices() << " Vertices \n"
	       << region->GetRoadmap()->m_pRoadmap->get_num_edges() << " Edges \n "
	       << get_cc_count(*(region->GetSRTRegionGraph()), cmap1) << " Connected Components"
	       << endl;
	//  cmap1.reset();



	  for (vi_1  = temp_rg->begin(); vi_1 != temp_rg->end(); ++vi_1) {
	    SRTInfo rrt1 = (*vi_1).property();
	    VID vid1 = (*vi_1).descriptor();
	    vector<VID> nc_vid, nr_vid;
	    vector<double> nc_dist;
	    //cout << "VID1 = " << vid1;
	    for (vi_2  = temp_rg->begin(); vi_2 != temp_rg->end(); ++vi_2) {
	      SRTInfo rrt2 = (*vi_2).property();
	      VID vid2 = (*vi_2).descriptor();
	      //cout << ", VID2 = " << vid2 << endl;
	      const CfgType c1 = rrt1.GetCandidate();
	      const CfgType c2 = rrt2.GetCandidate();
	      //cout << "Cfg1 = " << c1;
	      //cout << ", Cfg2 = " << c2 << endl;
	      double dist = dmm->Distance(pEnv, c1, c2); //bbx,

	      if (dist != 0) {
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
		if (nr_vid.size() < nr) {
		  nr_vid.push_back(vid2);
		} else {
		  if (rand() % 2 == 0) {
		    nr_vid[rand()%nr] = vid2;
		  }
		}
	      }
	    }

	    //cout << "   K-Closest VIDs: ";
	    for (int i=0; i<nc_vid.size(); ++i) {
	      //cout << " " << nc_vid[i];// << " Dist: " << nc_dist[i] << endl;
	      region->GetSRTRegionGraph()->add_edge(vid1,nc_vid[i]);
	    }
	    //cout << ",";
	    for (int i=0; i<nr_vid.size(); ++i) {
	      //cout << " " << nr_vid[i];// << " Dist: " << nc_dist[i] << endl;
	      region->GetSRTRegionGraph()->add_edge(vid1,nr_vid[i]);
	    }
	    //cout << "\n Done processing edges, next VID..." << endl;

	  }
	  srt_edge_timer = t3.stop();
	  cmap1.reset();
	  cout << " !! Region Graph Useful Stats !! "<< endl;
	  cout << region->GetRoadmap()->m_pRoadmap->get_num_vertices() << " Vertices \n"
	       << region->GetRoadmap()->m_pRoadmap->get_num_edges() << " Edges \n "
	       << get_cc_count(*(region->GetSRTRegionGraph()), cmap1) << " Connected Components"
	       << endl;
	  cmap1.reset();




	  cout<<"\n processor #----->["<<stapl::get_location_id()<<"] srt_region_determine_time = "  << srt_edge_timer << endl;

	}

	stapl::rmi_fence(); // Make sure processors don't start early on cand graph

	// END USING ONE PROCESSOR
	// BEING USING ALL PROCESSORS
	/**
	cout << "!!! NOW USING ALL PROCESSORS !!!" << endl;

	typedef stapl::p_graph_view_base<RegionGraph<SRTInfo,WeightType> >   VBType;
	VBType rge_view(*rg);

	//native or balance?
	stapl::part_native_view<VBType>::view_type vbnative =
	  stapl::part_native_view<VBType>()(rge_view);

	typedef stapl::p_graph_view_base<RoadmapGraph<CfgType,WeightType> >   gvbType;
	gvbType rme_view(*rmg);
	stapl::p_graph_partition_view_helper<gvbType>::view_type rm_innerbview =
	  stapl::p_graph_partition_view_helper<gvbType>()(rme_view);
	t3.start();
	determineRGEdges(vbnative,rm_innerbview,
			 region,pLp,
			 pEnv,*bbox,
			 dm,dm_label,
			 nc, nr);
	srt_edge_timer = t3.stop();
	*/
	// END USING ALL PROCESSORS


	//=================
	// Region connection of mesh graph
	//=============================

	Connector<CfgType, WeightType>::ConnectionPointer ccc_rrt;
	string rrt = "RRTConnect";
	ccc_rrt = GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(rrt);

	//ccc_rrt->PrintValues(cout);

	for(K itr2 = m_ComponentConnectionLabels.begin(); itr2 != m_ComponentConnectionLabels.end(); ++itr2) {
	  //cout << "Begin Connections Using: " << *itr2 << endl;

	  Connector<CfgType, WeightType>::ConnectionPointer pCCConnection;
	  pCCConnection = GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*itr2);
	  //typedef stapl::sequential::p_graph_view_base<RegionGraph<SRTInfo,WeightType> >   VType;
	  //typedef stapl::sequential::p_graph_view_base<RoadmapGraph<CfgType,WeightType> >   VType;
          typedef graph_view<RegionGraph<SRTInfo,WeightType> > VType;
          //typedef stapl::sequential::p_graph_view_base<RoadmapGraph<CfgType,WeightType> >   VType;
          VType rg_view(*rg);

	  //native or balance?
	  stapl::part_native_view<VType>::view_type vnative =
	    stapl::part_native_view<VType>()(rg_view);

	  typedef graph_view<RoadmapGraph<CfgType,WeightType> >   gvType;
          gvType rm_view(*rmg);
	  //stapl::p_graph_partition_view_helper<gvType>::view_type rm_innerview =
	  //stapl::p_graph_partition_view_helper<gvType>()(rm_view);
//It's not use in the code, why is it here?

          //stapl::graph_partition_view_helper<gvType>::view_type rm_innerview =
          //stapl::graph_partition_view_helper<gvType>()(rm_view);


	  //cout << "About to Call connectRRTRegion()..." << endl;
	  t4.start();
	  connectRRTRegion(vnative,region,pLp,pCCConnection,pEnv,*bbox);
	  integrator_timer = t4.stop();
	  cout<<"\n processor #----->["<<stapl::get_location_id()<<"] srt_region_connect_time  = "  << integrator_timer << endl;
	}
	total_timer = t5.stop();
	cout<<"\n processor #----->["<<stapl::get_location_id()<<"] total_time  = "  << total_timer << endl;
      }
    }

    void ParallelSRTStrategy::Finalize(int in_RegionID){

      //LOG_DEBUG_MSG("ParallelRRTStrategy::Finalize()");
      //---------------------------
      // Write roadmap to file
      //---------------------------
     cout << "testfin" << endl;
      if( stapl::get_location_id() == 0){

	string str, rg_str,out_stat;
        stringstream basefname;
	basefname << GetBaseFilename() << ".p" << stapl::get_num_locations() << ".it" << n_runs;
	ofstream outStat((basefname.str() + ".stats").c_str());
	ofstream osMap((basefname.str() + ".map").c_str());
	ofstream regionMap((basefname.str() + ".region.map").c_str());
	if (!osMap || !regionMap || !outStat){
//	  LOG_ERROR_MSG("ParallelPRMStrategy::Finalize(): can't open outfile: ");
	  exit(-1);
	} else{
	  region->WriteRoadmapForVizmo(osMap);
	  osMap.close();
	  write_graph(*rg,regionMap);
	  regionMap.close();
	  //stats->WriteGraphStats(outStat,region->GetRoadmap()->m_pRoadmap);
	  outStat.close();
	}

        //typedef typename RoadmapGraph<CFG,WEIGHT>::vertex_descriptor VID;
	stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
	cmap.reset();
	cout << " !! Useful Stats !! "<< endl;
	cout << region->GetRoadmap()->m_pRoadmap->get_num_vertices() << " Vertices \n "
	     << region->GetRoadmap()->m_pRoadmap->get_num_edges() << " Edges \n "
	     << get_cc_count(*(region->GetRoadmap()->m_pRoadmap), cmap) << " Connected Components"
	     << endl;

      }
      //cout << "!!ALL FINISHED!!"<< endl;

   }

//fin class ParallelRRTRoadmap
