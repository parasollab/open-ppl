#ifndef EntropyPRM_h
#define EntropyPRM_h

#include "Graph.h"
#include "EntropyModel.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG>
class EntropyPRM {
 public:

  EntropyPRM();
  ~EntropyPRM();

  //////////////////////
  // Access
  char* GetName();

  //////////////////////
  // I/O methods
  void ReadCommandLine(n_str_param* GNstrings[MAX_GN], int numGNs);
  void ParseCommandLine(int argc, char **argv);
  void PrintUsage(ostream& _os);
  void PrintValues(ostream& _os);
  void Print(ostream& out_os) const {};

  /**Basic Randomized (probabilistic) Node Generation.
   *This method generates NodeGenerationMethod::numNodes collision-free Cfgs
   *and insert there Cfgs to nodes.
   *_env Used to get free Cfg.
   *cd Used to get free Cfg
   *nodes Used to store generated nodes.
   *see See Cfg::GetFreeRandomCfg to know how to generate "one" free Cfg.
   *note If INTERMEDIATE_FILES is defined WritePathConfigurations will be
   *called.
   */
  void GenerateNodes(Environment* _env, Stat_Class& Stats,
		     CollisionDetection* cd, CDInfo* cdInfo,
		     DistanceMetric *dm, vector<CFG>& nodes);

  template <class WEIGHT>
  void  ExtractRegionPath(Roadmap<CFG,WEIGHT>* rm, CFG s_cfg, CFG g_cfg, Stat_Class& Stats,
			  CollisionDetection* cd, CDInfo* cdInfo,
			  DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp);

  template <class WEIGHT>
  void Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
               CollisionDetection* cd, CDInfo* cdInfo,
	       DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp,
               bool addPartialEdge, bool addAllEdges);
  template <class WEIGHT>
  void ConnectWithinRegion(CRegionSet<CFG>& r, int k,
	       Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
               CollisionDetection* cd, CDInfo* cdInfo,
	       DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp,
               bool addPartialEdge, bool addAllEdges);
  template <class WEIGHT>
  void ConnectBetweenRegions(CRegionSet<CFG>& r, vector<CRegionSet<CFG> >& regions, int k,
	       Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
               CollisionDetection* cd, CDInfo* cdInfo,
	       DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp,
               bool addPartialEdge, bool addAllEdges);

  template <class WEIGHT>
  int AddNodesToRoadmap(Roadmap<CFG,WEIGHT>* rm);

  //Node Generation Data
  num_param<int> numNodes; //num in initial model sampling
  num_param<int> ksamples; //num tried to improve region
  num_param<int> tries; //num times to try and resample
  num_param<int> kclosest; //num used to generate region

  //node connection
  num_param<int> kclosest_connect; //num used for kclosest connect meth.
  num_param<int> kpairs_connect; //num kpairs in connect ccs
  num_param<int> smallcc_connect;//for components

  //Node Connection Data
  double connectionPosRes, connectionOriRes;
  num_param<int> k_in_region, k_btw_region;
  num_param<int> iter;

  //Model Data
  num_param<double> low_entropy; // value used to define low entropy
  num_param<double> pctSurf;
  num_param<double> pctFree;
  CModel<CFG> cmodel;
};


/////////////////////////////////////////////////////////////////////
//
//  definitions for class EntropyPRM declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG>
EntropyPRM<CFG>::
EntropyPRM() : numNodes	       ("nodes", 10, 1, 5000000),
	       ksamples        ("ksamples",            10,   1,   5000000),
	       tries           ("tries",                5,   1,   5000000),
	       kclosest        ("kclosest",            10,   1,   5000000),
	       kclosest_connect("kclosestconnect",     10,   0,   5000000),
               kpairs_connect  ("kpairsconnect",       10,   0,   5000000),
               smallcc_connect ("smallccconnect",      3,    1,   5000000),
	       low_entropy     ("lowentropy",         0.1, 0.0,   1.0    ),
	       pctSurf         ("pctSurf",         0.1, 0.0,   1.0    ),
	       pctFree         ("pctFree",         0.01, 0.0,   1.0    ),
     k_in_region("k_in_region", 5, 0, 5000000),
     k_btw_region("k_btw_region", 10, 0, 5000000),
     iter("iter", 10, 0, 5000000),
	       connectionPosRes(0.05), connectionOriRes(0.05)
{
  numNodes.PutDesc("INTEGER","(number of nodes, default 10)");
  ksamples.PutDesc("INTEGER","(number of nodes to improve region, default 10)");
  tries.PutDesc("INTEGER","(number times to try and resample, default 5)");
  kclosest.PutDesc("INTEGER",
		   "(number of nodes used to generate initial regions, default 10)");
  low_entropy.PutDesc("DOUBLE",
		      "(value used to define low entropy, default 0.1)");
  pctSurf.PutDesc("DOUBLE",
		  "(pct free samples from surface region, default 0.1)");
  pctFree.PutDesc("DOUBLE",
		  "(pct free samples from free regions, default 0.01)");
  k_in_region.PutDesc("INTEGER","(k closest in a region)");
  k_btw_region.PutDesc("INTEGER","(k closest between regions)");

  kclosest_connect.PutDesc("INTEGER",
			   "(number of kclosest in connection, default 10)");
  kpairs_connect.PutDesc("INTEGER",
			 "(number of kpairs in component connection, default 10)");
  smallcc_connect.PutDesc("INTEGER",
			  "(size of smallcc in component connection, default 3)");
  iter.PutDesc("INTEGER","(rrt iters)");

  cout << " EntropyPRM created. " << endl;
}


template <class CFG>
EntropyPRM<CFG>::
~EntropyPRM() {
}


template <class CFG>
char*
EntropyPRM<CFG>::
GetName() {
  return "EntropyPRM";
}


template <class CFG>
void
EntropyPRM<CFG>::
ReadCommandLine(n_str_param* GNstrings[MAX_GN], int numGNs) {
  //go through the command line looking for method names
  //for(int i=0; i<numGNs; i++) {
  int i=0;
    std::istringstream _myistream(GNstrings[i]->GetValue());
    int argc = 0;
    char* argv[50];
    char cmdFields[50][100];
    while ( _myistream >> cmdFields[argc] ) {
      argv[argc] = (char*)(&cmdFields[argc]);
      ++argc;
    }
    ParseCommandLine(argc, argv);
  //}
}


template <class CFG>
void
EntropyPRM<CFG>::
ParseCommandLine(int argc, char **argv) {
  int i;
  for (i =1; i < argc; ++i) {
    if( numNodes.AckCmdLine(&i, argc, argv) ) {
    } else if (ksamples.AckCmdLine(&i, argc, argv) ) {
    } else if (tries.AckCmdLine(&i, argc, argv) ) {
    } else if (kclosest.AckCmdLine(&i, argc, argv) ) {
    } else if (kclosest_connect.AckCmdLine(&i, argc, argv) ) {
    } else if (kpairs_connect.AckCmdLine(&i, argc, argv) ) {
    } else if (smallcc_connect.AckCmdLine(&i, argc, argv) ) {
    } else if (low_entropy.AckCmdLine(&i, argc, argv) ) {
    } else if (pctSurf.AckCmdLine(&i, argc, argv) ) {
    } else if (pctFree.AckCmdLine(&i, argc, argv) ) {
    } else if (k_in_region.AckCmdLine(&i, argc, argv) ) {
    } else if (k_btw_region.AckCmdLine(&i, argc, argv) ) {
    } else if (iter.AckCmdLine(&i, argc, argv)) {
    } else {
      cerr << "\nERROR ParseCommandLine: Don\'t understand \"";
      for(int j=0; j<argc; j++)
        cerr << argv[j] << " ";
      cerr << "\"\n\n";
      PrintUsage(cerr);
      cerr << endl;
      exit (-1);
    }
  }
}


template <class CFG>
void
EntropyPRM<CFG>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);

  _os << "\n" << GetName() << " ";
  _os << "\n\t"; numNodes.PrintUsage(_os);
  _os << "\n\t"; ksamples.PrintUsage(_os);
  _os << "\n\t"; tries.PrintUsage(_os);
  _os << "\n\t"; kclosest.PrintUsage(_os);
  _os << "\n\t"; kclosest_connect.PrintUsage(_os);
  _os << "\n\t"; kpairs_connect.PrintUsage(_os);
  _os << "\n\t"; smallcc_connect.PrintUsage(_os);
  _os << "\n\t"; low_entropy.PrintUsage(_os);
  _os << "\n\t"; pctSurf.PrintUsage(_os);
  _os << "\n\t"; pctFree.PrintUsage(_os);
  _os << "\n\t"; k_in_region.PrintUsage(_os);
  _os << "\n\t"; k_btw_region.PrintUsage(_os);
  _os << "\n\t"; iter.PrintUsage(_os);
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG>
void
EntropyPRM<CFG>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << numNodes.GetFlag() << " " << numNodes.GetValue();
  _os << ksamples.GetFlag() << " " << ksamples.GetValue();
  _os << tries.GetFlag() << " " << tries.GetValue();
  _os << kclosest.GetFlag() << " " << kclosest.GetValue();
  _os << kclosest_connect.GetFlag() << " " << kclosest_connect.GetValue();
  _os << kpairs_connect.GetFlag() << " " << kpairs_connect.GetValue();
  _os << smallcc_connect.GetFlag() << " " << smallcc_connect.GetValue();
  _os << low_entropy.GetFlag() << " " << low_entropy.GetValue();
  _os << pctSurf.GetFlag() << " " << pctSurf.GetValue();
  _os << pctFree.GetFlag() << " " << pctFree.GetValue();
  _os << k_in_region.GetFlag() << " " << k_in_region.GetValue();
  _os << k_btw_region.GetFlag() << " " << k_btw_region.GetValue();
  _os << iter.GetFlag() << " " << iter.GetValue();
  _os << endl;
}


template <class CFG>
void
EntropyPRM<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats,
	      CollisionDetection* cd, CDInfo* cdInfo, DistanceMetric * dm,
	      vector<CFG>& nodes) {
  cout << "(numNodes=" << numNodes.GetValue()
       << " ksamples=" << ksamples.GetValue()
       << " tries=" << tries.GetValue()
       << " kclosest=" << kclosest.GetValue()
       << ") " << endl << flush;

  // Generate Model Nodes
  cout << "Generating Models Nodes" << endl;
  cmodel.Init( _env, numNodes.GetValue() );
  //could specify how nodes should be generated by having a GenerateMapNodes object, default is uniform sampling

  // Construct Regions
  cout << "Constructing Regions" << endl;
  cmodel.ConstructRegions( _env, dm, Stats, cd, cdInfo, kclosest.GetValue() );

  // Learn Regions
  cout << "Learning on Regions" << endl;
  cmodel.LearnRegions( _env, dm, Stats, cd, cdInfo,
		       low_entropy.GetValue(), ksamples.GetValue(), tries.GetValue());


  // print regions ... just for testing
  cout << " Number of regions generated: "
       << cmodel.region_map.get_num_vertices() << endl;
  bool local_verbose = true;
  int i=0;
  typedef typename Graph<UG<CRegionSet<CFG>, DoubleWeight>,
    NMG<CRegionSet<CFG>, DoubleWeight>,
    WG<CRegionSet<CFG>, DoubleWeight>,
    CRegionSet<CFG>, DoubleWeight>::CVI regionset_const_iterator;
  for(regionset_const_iterator R = cmodel.region_map.begin();
      R != cmodel.region_map.end(); ++R, ++i) {
    if( local_verbose )
      cout << " Region: " << i << "\t size: " << R->data.size()
	   << "\t entropy: " << R->data.entropy
	   << "\t numfree: " << R->data.NumFreeNodes
	   << "\t numcoll: " << R->data.NumCollisionNodes
	//<< "\t radius: " << R->data.radius
	   << "\t type : " << R->data.GetStringType()
	   << "\t localregs: " << (R->data.m_RegionSet).size()<< endl;

  }//end for I<cmodel.regions.size()

  // Write Regions to file (3d space)
  //cmodel.WriteRegionsToSpecFile();
};


template <class CFG, class WEIGHT>
struct distance_compare : public binary_function<VID,VID,bool> {
  Roadmap<CFG,WEIGHT>* rm;
  Environment* env;
  DistanceMetric* dm;
  CFG c;

  distance_compare(Roadmap<CFG,WEIGHT>* r, DistanceMetric* d, VID v) :
    rm(r), dm(d) {
    env = rm->GetEnvironment();
    c = rm->m_pRoadmap->find_vertex(v).property();
  }
  distance_compare(Roadmap<CFG,WEIGHT>* r, DistanceMetric* d, CFG& cfg) :
    rm(r), dm(d), c(cfg) {
    env = rm->GetEnvironment();
  }
  ~distance_compare() {}

  bool operator()(const VID& v1, const VID& v2) const {
    return (dm->Distance(env, c, rm->m_pRoadmap->find_vertex(v1).property()) <
	    dm->Distance(env, c, rm->m_pRoadmap->find_vertex(v2).property()));
  }
  bool operator()(const CFG& c1, const CFG& c2) const {
    return (dm->Distance(env, c, c1) <
	    dm->Distance(env, c, c2));
  }
};


template <class CFG>
template <class WEIGHT>
void
EntropyPRM<CFG>::
ExtractRegionPath(Roadmap<CFG,WEIGHT>* rm, CFG s_cfg, CFG g_cfg, Stat_Class& Stats,
		  CollisionDetection* cd, CDInfo* cdInfo,
		  DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp ) {
  cout << "ExtractRegionPath\n";

  //between regions, build roadmap
  cout << "Building region map\n";
  cmodel.BuildRegionMap(rm->GetEnvironment(), dm);
  cout << "Done building region map" << endl;
  // Extract path in cmodel
  //cmodel.ExtractRegionPath(rm,s_cfg, g_cfg, Stats, cd, cdInfo, dm, lp);
  VID svid = -1;
  VID gvid = -1;
  // find free regions containing start and goal cfgs
  cout << "EntropyPRM::ExtractRegionPath::Finding regions query cfgs are in. " << endl;
  Environment * env = rm->GetEnvironment();
  VID s_neighbor_vid = cmodel.ClosestFreeRegion(cmodel.region_map.begin(),
						     cmodel.region_map.end(),
						     s_cfg, env, dm);
  VID g_neighbor_vid = cmodel.ClosestFreeRegion(cmodel.region_map.begin(),
						     cmodel.region_map.end(),
						     g_cfg, env, dm);
  //CRegion<CFG> s_neighbor = cmodel.ClosestFreeRegion(cmodel.regions, s_cfg, env, dm);
  //CRegion<CFG> g_neighbor = cmodel.ClosestFreeRegion(cmodel.regions, g_cfg, env, dm);
  cout << "done CModel::ExtractRegionPath::Finding regions query cfgs are in. " << endl;

  //svid = s_neighbor.ID;
  //gvid = g_neighbor.ID;
  //svid = cmodel.region_map.GetVID( s_neighbor );
  //gvid = cmodel.region_map.GetVID( g_neighbor );
  svid = s_neighbor_vid;
  gvid = g_neighbor_vid;

  cout << " start rid: " << svid << endl;
  cout << " goal rid: " << gvid << endl;

  vector<pair< CRegionSet<CFG>, DoubleWeight> > rp;

  rp.clear();
  FindPathDijkstra(cmodel.region_map, svid, gvid, rp);

  // make path more dense (hopefully for easier connection)
  /*
  vector<pair< CRegion<CFG>, DoubleWeight> > new_rp;
if(1) {
  for(int I=0; (I<rp.size()); I++) {
    if( I==(rp.size()-1) ) {
      new_rp.push_back( rp[I] );
      break;
    }

    CRegion<CFG> & Reg1 = rp[I].first;
    if( Reg1.type == BLOCKED ) {
      //last try to convert to narrow
      Reg1.Classify( rm->GetEnvironment(), Stats, cd, cdInfo, dm,
		     low_entropy.GetValue(), ksamples.GetValue(), tries.GetValue() );
    }
    new_rp.push_back( rp[I] );
    vector< pair<VID,DoubleWeight> > neighbors1;
    CRegion<CFG> & Reg2 = rp[I+1].first;
    CFG Reg2_center = Reg2.getCenter();
    cout << "reg types: " << Reg1.GetStringType() << " and " << Reg2.GetStringType() << endl;
    if( (Reg1.type==FREE) && (Reg2.type==FREE) ) continue; //should be easy enough to connect

    cmodel.region_map.GetAllEdgesToV( (int)Reg1.ID, neighbors1 );
    cout << " Num neighbors: " << neighbors1.size();
    for(int J=0; J<neighbors1.size(); J++) {

      CRegion<CFG> Reg1_neighbor = cmodel.region_map.find_vertex(neighbors1[J].first).property();
      cout << " I: " << I << " J: " << J << " type: " << Reg1_neighbor.GetStringType()
         << " reg_id: " << Reg1_neighbor.ID << endl;

      if( (Reg2.ID == Reg1_neighbor.ID) || (Reg1_neighbor.type==BLOCKED) ) continue;

      CFG Reg1_neighbor_center = Reg1_neighbor.getCenter();
      //if( (dm->Distance(env, Reg1_neighbor_center, Reg2_center) < (Reg2.getRadius() + Reg1_neighbor.getRadius() ) ) ) {
      //if( ((Reg1_neighbor.type==NARROW) && (Reg2.type==NARROW )) ||
      //  ((Reg1.type==NARROW) && (Reg2.type==NARROW)) ) {
      if( ((Reg1.type==NARROW) && (Reg2.type==NARROW)) && (Reg1_neighbor.type!=FREE) ) {
	// add this region to path
	pair< CRegion<CFG>, DoubleWeight >neighbor_val(Reg1_neighbor,DoubleWeight(1));

	cout << "Inserting into path reg id: " << Reg1_neighbor.ID << " type: " << Reg1_neighbor.GetStringType() << endl;
	//rp.insert( rp_iterator, 1, neighbor_val );
	//rp_iterator++;
	//rp.insert( rp_iterator, neighbor_val );
	new_rp.push_back( neighbor_val );
	//cout << " done inserting " << endl;
      }//end dist check

    }//end for J<neighbors

  }//end for I<rp.size()
  rp = new_rp;
}//endif 0

  */

  //done making path more clear

  //setup path for writing
  vector<int> selected_region_index;
  for(int I=0; I<rp.size(); I++) {
    selected_region_index.push_back( rp[I].first.ID );
  }



  // writing path file
  //cmodel.WriteRegionsPathToSpecFile( selected_region_index );
  cmodel.WriteRegionsPathToSpecFile( rp );

  // add nodes in path and do simple closest

  for(int I=0; (I<rp.size()-1); I++) {
    CRegionSet<CFG> & Reg1 = rp[I].first;
    vector<CFG> samples;
    vector<VID> vids;
    switch(Reg1.type)
      {
      case FREE:
        //samples = Reg1.GetPercentFreeNodes(pctFree.GetValue());
	Reg1.GetPercentFreeNodeAndVID(pctFree.GetValue(), samples, vids);
        break;
      case UNKNOWN:
      case SURFACE:
        //samples = Reg1.GetPercentFreeNodes(pctSurf.GetValue());
	Reg1.GetPercentFreeNodeAndVID(pctSurf.GetValue(), samples, vids);
        break;
      case NARROW:
        //samples = Reg1.GetPercentFreeNodes(1.0);
	Reg1.GetPercentFreeNodeAndVID(1.0, samples, vids);
        break;
      }
    //Add Samples (don't check if already in)
    for(typename vector<CFG>::iterator S = samples.begin(); S != samples.end(); ++S) {
      //VID v = rm->m_pRoadmap->GetVID(*S);
      //if(v == INVALID_VID)
      VID v = rm->m_pRoadmap->AddVertex(*S);
      vids.push_back(v);
    }

  }//end for I<rp.size()


  cout << " Simple connection: (kclosest="<<kclosest_connect.GetValue()<<")"
       << "(connectccs kpairs="<<kpairs_connect.GetValue()
       <<" smallcc="<<smallcc_connect.GetValue()<<")"<< endl;
  //connect k pairs
  Closest<CFG,WEIGHT> closest;
  closest.cdInfo = cdInfo;
  closest.connectionPosRes = connectionPosRes;
  closest.connectionOriRes = connectionOriRes;
  closest.kclosest = kclosest_connect.GetValue();
  closest.mfailure = kclosest_connect.GetValue();
  closest.Connect(rm, Stats, cd, dm, lp, false, false);

};


#include "RRTexpand.h"

template <class CFG>
template <class WEIGHT>
void
EntropyPRM<CFG>::
Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
        CollisionDetection* cd, CDInfo* cdInfo,
        DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges) {
  /*
  // print regions ... just for testing
  cout << " Number of regions generated: "
       << cmodel.region_map.get_num_vertices() << endl;
  bool local_verbose = true;
  int i=0;
  typedef typename Graph<UG<CRegionSet<CFG>, DoubleWeight>,
    NMG<CRegionSet<CFG>, DoubleWeight>,
    WG<CRegionSet<CFG>, DoubleWeight>,
    CRegionSet<CFG>, DoubleWeight>::CVI regionset_const_iterator;
  for(regionset_const_iterator R = cmodel.region_map.begin();
      R != cmodel.region_map.end(); ++R, ++i) {
    if( local_verbose )
      cout << " Region: " << i << "\t size: " << R->data.size()
	   << "\t entropy: " << R->data.entropy
	   << "\t numfree: " << R->data.NumFreeNodes
	   << "\t numcoll: " << R->data.NumCollisionNodes
	//<< "\t radius: " << R->data.radius
	   << "\t type : " << R->data.GetStringType()
	   << "\t localregs: " << (R->data.m_RegionSet).size()<< endl;

  }//end for I<cmodel.regions.size()
  */

  cout << "Connection\n";

  /* //assume already done
  //merge regions togehter
  cout << "Building region map\n";
  cmodel.BuildRegionMap(rm->GetEnvironment(), dm);
  cout << "Merging region map\n";
  cmodel.MergeRegions(rm->GetEnvironment(), Stats, cd, cdInfo, dm);
  */

  //for each region, attempt connections in that region
  cout << "Connecting in regions\n";
  typedef typename Graph<UG<CRegionSet<CFG>, DoubleWeight>,
    NMG<CRegionSet<CFG>, DoubleWeight>,
    WG<CRegionSet<CFG>, DoubleWeight>,
    CRegionSet<CFG>, DoubleWeight>::VI regionset_iterator;
  /*
  vector<VID> Rvids;
  rm->m_pRoadmap->GetVerticesVID(Rvids);
  Closest<CFG,WEIGHT> connectCCs;
  connectCCs.cdInfo = cdInfo;
  connectCCs.connectionPosRes = connectionPosRes;
  connectCCs.connectionOriRes = connectionOriRes;
  connectCCs.kclosest = k_in_region.GetValue();
  connectCCs.mfailure = k_in_region.GetValue();
  connectCCs.Connect(rm, Stats, cd, dm, lp,
		     addPartialEdge, addAllEdges,
		     Rvids, Rvids);
  */
  stapl::sequential::vector_property_map< stapl::stapl_color<size_t> > cmap;
  for(regionset_iterator R = cmodel.region_map.begin();
      R != cmodel.region_map.end(); ++R)
    ConnectWithinRegion(R->data, k_in_region.GetValue(),
			rm, Stats, cd, cdInfo, dm, lp,
			addPartialEdge, addAllEdges);
  cmap.reset();
  cout << get_cc_count(*(rm->m_pRoadmap),cmap) << " connected components\n";
  //Stats.PrintAllStats(rm);

  //between regions, do component's like connection btw neighboring regions
  cout << "Connecting between regions\n";

  ConnectCCs<CFG,WEIGHT> connectCCs2;
  /*
  connectCCs2.cdInfo = cdInfo;
  connectCCs2.connectionPosRes = connectionPosRes;
  connectCCs2.connectionOriRes = connectionOriRes;
  connectCCs2.kpairs = kpairs_connect.GetValue();
  connectCCs2.smallcc = smallcc_connect.GetValue();
  connectCCs2.Connect(rm, Stats, cd, dm, lp,
		      addPartialEdge, addAllEdges);
  cout << Get CCcount(*(rm->m_pRoadmap)) << " connected components\n";
  */

  for(regionset_iterator R = cmodel.region_map.begin();
      R != cmodel.region_map.end(); ++R) {
    vector<VID> neighborVIDS;
    cmodel.region_map.GetAdjacentVertices(R->data, neighborVIDS);
    vector<VID>::iterator End = remove_if(neighborVIDS.begin(), neighborVIDS.end(),
					  bind2nd(less<VID>(), R->vid));
    if(End != neighborVIDS.begin()) {
      vector<CRegionSet<CFG> > neighbors;
      for(vector<VID>::const_iterator N = neighborVIDS.begin();
	  N != End; ++N)
	neighbors.push_back(cmodel.region_map.find_vertex(*N).property());
      ConnectBetweenRegions(R->data, neighbors, k_btw_region.GetValue(),
			    rm, Stats, cd, cdInfo, dm, lp,
			    addPartialEdge, addAllEdges);
    }
  }
  cmap.reset();
  cout << get_cc_count(*(rm->m_pRoadmap), cmap) << " connected components\n";


  //Stats.PrintAllStats(rm);


  //RRT
  RRTexpand<CFG,WEIGHT> rrt;
  rrt.cdInfo = cdInfo;
  rrt.connectionPosRes = connectionPosRes;
  rrt.connectionOriRes = connectionOriRes;
  rrt.iterations = iter.GetValue();
  rrt.o_clearance = 0;
  rrt.clearance_from_node = 0;
  //rrt.stepFactor = 30;
  rrt.stepFactor = 500;
  typedef typename vector<Sample<CFG> >::iterator sample_iterator;
  vector<VID> checked_vids;
  for(regionset_iterator R = cmodel.region_map.begin();
      R != cmodel.region_map.end(); ++R) {
    if(R->data.type == NARROW) {
      for(int i=0; i < R->data.m_RegionSet.size(); ++i) {
	for(sample_iterator S = R->data.m_RegionSet[i].samples.begin();
	    S != R->data.m_RegionSet[i].samples.end(); ++S) {
	  if(!S->isColl) {
	    if(find(checked_vids.begin(), checked_vids.end(), S->vid) == checked_vids.end()) {
	      checked_vids.push_back(S->vid);

	      //setup for expansion
	      Roadmap<CFG,WEIGHT> submap1;
	      submap1.environment = rm->GetEnvironment();
	      vector<VID> cc;
	      cmap.reset();
	      get_cc(*(rm->m_pRoadmap),cmap, S->vid, cc);
	      checked_vids.insert(checked_vids.end(), cc.begin(), cc.end());
	      submap1.m_pRoadmap->MergeRoadMap(rm->m_pRoadmap, cc);
	      vector<CFG> dummyU; bool dummyC = false;
	      rrt.smallcc = cc.size()+1;
	      rrt.RRT(&submap1, Stats,
		      rrt.iterations,
		      rrt.stepFactor * rrt.connectionPosRes,
		      rrt.o_clearance, rrt.clearance_from_node,
		      dummyU,
		      &(*cd), &(*lp), &(*dm), dummyC, false,
		      false, false);
	      vector<VID> verts;
	      (&submap1)->m_pRoadmap->GetVerticesVID(verts);
	      //-- map = map + submap
	      rm->m_pRoadmap->MergeRoadMap(submap1.m_pRoadmap, verts);
	      submap1.environment = NULL;
	    }
	  }
	}
      }
    }
  }
  cmap.reset();
  cout << get_cc_count(*(rm->m_pRoadmap), cmap) << " connected components\n";


  //ConnectCCs<CFG,WEIGHT> connectCCs2;
  connectCCs2.cdInfo = cdInfo;
  connectCCs2.connectionPosRes = connectionPosRes;
  connectCCs2.connectionOriRes = connectionOriRes;
  connectCCs2.kpairs = kpairs_connect.GetValue();
  connectCCs2.smallcc = smallcc_connect.GetValue();
  connectCCs2.Connect(rm, Stats, cd, dm, lp,
		      addPartialEdge, addAllEdges);
  cmap.reset();
  cout << get_cc_count(*(rm->m_pRoadmap),cmap) << " connected components\n";
};


template <class CFG, class WEIGHT>
CFG
nearest_to_centroid(const vector<CFG>& samples, Roadmap<CFG,WEIGHT>* rm,
		    DistanceMetric* dm) {
  if(samples.empty())
    return CFG();

  //compute centroid of samples
  vector<double> center_data(samples[0].DOF(), 0);
  for(typename vector<CFG>::const_iterator S = samples.begin();
      S != samples.end(); ++S) {
    vector<double> cfg_data = S->GetData();
    transform(center_data.begin(), center_data.end(), cfg_data.begin(),
	      center_data.begin(), plus<double>());
  }
  transform(center_data.begin(), center_data.end(), center_data.begin(),
	    bind2nd(divides<double>(), 1.0*samples.size()));
  CFG center(center_data);

  //return sample closest to center
  return *min_element(samples.begin(), samples.end(),
		      distance_compare<CFG,WEIGHT>(rm, dm, center));
}


template <class CFG>
template <class WEIGHT>
void
EntropyPRM<CFG>::
ConnectWithinRegion(CRegionSet<CFG>& region, int k,
	            Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
                    CollisionDetection* cd, CDInfo* cdInfo,
	            DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp,
                    bool addPartialEdge, bool addAllEdges) {
  //get region vids
  vector<VID> Rvids;
  switch(region.type)
    {
    case FREE:
      Rvids = region.GetPercentFreeVIDs(pctFree.GetValue());
      break;
    case UNKNOWN:
    case SURFACE:
      Rvids = region.GetPercentFreeVIDs(pctSurf.GetValue());
      break;
    case BLOCKED:
    case NARROW:
      Rvids = region.GetPercentFreeVIDs(1.0);
      break;
    }
  //remove any INVALID vids
  Rvids.erase(remove(Rvids.begin(), Rvids.end(), INVALID_VID), Rvids.end());

  if(Rvids.empty())
    return;

  //connect k pairs
  //Closest<CFG,WEIGHT> connectCCs;
  ClosestUnconnected<CFG,WEIGHT> connectCCs;
  connectCCs.cdInfo = cdInfo;
  connectCCs.connectionPosRes = connectionPosRes;
  connectCCs.connectionOriRes = connectionOriRes;
  connectCCs.kclosest = k;
  connectCCs.mfailure = k;
  connectCCs.Connect(rm, Stats, cd, dm, lp,
		     addPartialEdge, addAllEdges,
		     Rvids, Rvids);
}


template <class CFG>
template <class WEIGHT>
void
EntropyPRM<CFG>::
ConnectBetweenRegions(CRegionSet<CFG>& region, vector<CRegionSet<CFG> >& neighbors,
		      int k,
	              Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
                      CollisionDetection* cd, CDInfo* cdInfo,
	              DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp,
                      bool addPartialEdge, bool addAllEdges) {
  //get region vids
  vector<VID> Rvids;
  switch(region.type)
    {
    case FREE:
      Rvids = region.GetPercentFreeVIDs(pctFree.GetValue());
      break;
    case UNKNOWN:
    case SURFACE:
      Rvids = region.GetPercentFreeVIDs(pctSurf.GetValue());
      break;
    case BLOCKED:
    case NARROW:
      Rvids = region.GetPercentFreeVIDs(1.0);
      break;
    }
  //remove any INVALID vids
  Rvids.erase(remove(Rvids.begin(), Rvids.end(), INVALID_VID), Rvids.end());

  if(Rvids.empty())
    return;

  //for each neigbor, do components connection
  typedef typename vector<CRegionSet<CFG> >::iterator regionset_iterator;
  for(regionset_iterator N = neighbors.begin(); N != neighbors.end(); ++N) {
    //get region vids
    vector<VID> Nvids;
    switch(N->type)
      {
      case FREE:
	Nvids = N->GetPercentFreeVIDs(pctFree.GetValue());
        break;
      case UNKNOWN:
      case SURFACE:
	Nvids = N->GetPercentFreeVIDs(pctSurf.GetValue());
        break;
      case BLOCKED:
      case NARROW:
	Nvids = N->GetPercentFreeVIDs(1.0);
        break;
      }
    //remove any INVALID vids
    Nvids.erase(remove(Nvids.begin(), Nvids.end(), INVALID_VID), Nvids.end());

    if(Nvids.empty())
      continue;

    //connect k pairs
    ConnectCCs<CFG,WEIGHT> connectCCs;
    connectCCs.cdInfo = cdInfo;
    connectCCs.connectionPosRes = connectionPosRes;
    connectCCs.connectionOriRes = connectionOriRes;
    connectCCs.kpairs = k;
    //connectCCs.kpairs = (int)ceil(((double)k) / ((double)neighbors.size()));
    //cout << "kpairs = " << connectCCs.kpairs << endl;
    connectCCs.smallcc = 1;
    connectCCs.ConnectBigCCs(rm, Stats, cd, lp, dm,
			     Rvids, Nvids,
			     addPartialEdge, addAllEdges);
  }
}


template <class CFG>
template <class WEIGHT>
int
EntropyPRM<CFG>::
AddNodesToRoadmap(Roadmap<CFG,WEIGHT>* rm) {
  int initial_size = rm->m_pRoadmap->get_num_vertices();

  typedef typename Graph<UG<CRegionSet<CFG>, DoubleWeight>,
    NMG<CRegionSet<CFG>, DoubleWeight>,
    WG<CRegionSet<CFG>, DoubleWeight>,
    CRegionSet<CFG>, DoubleWeight>::VI regionset_iterator;
  for(regionset_iterator R = cmodel.region_map.begin();
      R != cmodel.region_map.end(); ++R) {
    //get appropriate number of samples from region
    vector<CFG> samples;
    vector<VID> vids;
    //cout << " Getting nodes from type: " << R->data.GetStringType()
    //	 << " type num: " << R->data.type << " ";
    switch(R->data.type)
      {
      case FREE:
        R->data.GetPercentFreeNodeAndVID(pctFree.GetValue(), samples, vids);
        break;
      case UNKNOWN:
      case SURFACE:
        R->data.GetPercentFreeNodeAndVID(pctSurf.GetValue(), samples, vids);
        break;
      case NARROW:
        R->data.GetPercentFreeNodeAndVID(1.0, samples, vids);
        break;
      }

    //add samples to roadmap, update vids
    typename vector<CFG>::iterator S = samples.begin();
    for(vector<VID>::const_iterator V = vids.begin();
	V != vids.end() && S != samples.end(); ++V, ++S)
      if(*V == INVALID_VID) {
	VID v = rm->m_pRoadmap->GetVID(*S);
	if(v == INVALID_VID)
	  v = rm->m_pRoadmap->AddVertex(*S);
	R->data.SetVID(*S, v);
	//call above changes functionality slightly than
	//below implementation
	/*
	vector<VID> neighbors;
	cmodel.region_map.GetAdjacentVertices(R->vid, neighbors);
	for(vector<VID>::const_iterator N = neighbors.begin();
	    N != neighbors.end(); ++N) {
	  region_iterator RR;
	  if(cmodel.region_map.IsVertex(*N, &RR))
	    RR->data.SetVID(*S, v);
	}
	*/
      }

  }//end for all regions

  return (rm->m_pRoadmap->get_num_vertices() - initial_size);
}

#endif


