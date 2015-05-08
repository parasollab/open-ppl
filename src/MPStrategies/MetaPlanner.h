#ifndef MetaPlanner_h
#define MetaPlanner_h

#include<sys/time.h>

#include "Roadmap.h"

#include "MetricUtils.h"
#include "Connector.h"
#include "DistanceMetrics.h"
#include "LocalPlanners.h"
#include "GenerateMapNodes.h"

#include "GeneratePartitions.h"

#include "MPRegion.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template<class CFG, class WEIGHT>
class CSpaceCharacterizer {
 public:
  CSpaceCharacterizer(Input& input, DistanceMetric &i_dm, CollisionDetection &i_cd, Environment &i_env, int i_number_of_samples)
    : number_of_samples(i_number_of_samples), dm(i_dm), cd(i_cd), env(i_env) {
    lp_features.ReadCommandLine(input.LPstrings, input.numLPs, input.cdtype);
    gn_features.ReadCommandLine(input.GNstrings, input.numGNs);
    cm_features.ReadCommandLine(&input,&env);

    addPartialEdge = input.addPartialEdge.GetValue();
    addAllEdges = input.addAllEdges.GetValue();
    environment_name = input.defaultFile.GetValue();
  }

  ~CSpaceCharacterizer() {
  }
  void FindPlanningStrategy() { //Characterize CSpace and find a good planning strategy
  }
  void Characterize(MPRegion<CFG,WEIGHT> *region) {
    cout << "CSpaceCharacterizer::Characterize(). TODO ALL " << endl;

    cout << "Node generation method, local planners, map connection strategy and options are selected from the input" << endl;

  //the code within the ifdef needs to be fixed
#ifdef COLLISIONCFG
  //Code to initialize stuff for collision nodes
    for(int i = 0; i< region->GetExternalBodyCount(); i++) {
      vector <vector<double> > new_vector;
      vector <double> tmp_vector;
      new_vector.push_back(tmp_vector);
      CollisionConfiguration.push_back(new_vector);

     for(int j = 0; j < gn_features.selected.size(); j++) {
       vector<CfgType> tmpCfgType;
       gn_features.selected[j]->m_vGeneratedCollisionConfiguration.push_back(tmpCfgType);
     }
    }
#endif

    //---------------------------
    // Generate feature roadmap
    //---------------------------
    // Generate feature roadmap nodes
    NodeGenClock.StartClock("Feature Node Generation");
    vector<CFG> feature_nodes;
    gn_features.GenerateNodes(&(region->feature_roadmap),region->feature_stats,&cd,&dm,feature_nodes);
    NodeGenClock.StopClock();

    // Connect feature roadmap nodes
    ConnectionClock.StartClock("Feature Node Connection");
    cm_features.ConnectComponents(&(region->feature_roadmap), region->feature_stats,
				  &cd, &dm, &lp_features,
				  addPartialEdge, addAllEdges);
    ConnectionClock.StopClock();

    //save feature roadmap: check whether we can set its name
    string ftr_map_file = environment_name + "."+ region->region_tag + ".ftr.map";
    region->feature_roadmap.WriteRoadmap(&input,&cd,&dm,&lp_features,ftr_map_file.c_str());


    //---------------------------
    // Compute Features
    //---------------------------
    //General features are computed on the fly on previous steps
    cout << "------Computing Features-------" << endl;
    region->feature_stats.ComputeIntraCCFeatures(&cm_features, &(region->feature_roadmap), &dm);
    region->feature_stats.ComputeInterCCFeatures(&cm_features, &(region->feature_roadmap), &dm);
    //feature_stats.PrintFeatures();


  }


 public: //make private as soon as I make colliding nodes available to partitioner in a different way
  int number_of_samples;
  GenerateMapNodes<CFG> gn_features;
  Connector<CFG,WEIGHT> cm_features;
  LocalPlanners<CFG,WEIGHT> lp_features;
  DistanceMetric &dm;
  CollisionDetection &cd;
  Environment &env;

  bool addPartialEdge, addAllEdges;

  ClockClass        NodeGenClock;
  ClockClass        ConnectionClock;

  //temporary variable
#ifdef COLLISIONCFG
 vector< vector < vector <  double > > > CollisionConfiguration;
#endif


 string environment_name;
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
class MetaPlanner {
 public:
  //MetaPlanner(vector< CgraphGenerator > planners, cd,lp,gn,cm,dm);
  // planners to gather features, planners to make regional roadmaps, planners to integrate regions.
  MetaPlanner(Input& input, int i_maximum_tree_height, double i_epsilon, int i_characterization_samples,  int dofs, int pos_dofs, bool randomSeed);
  ///Destructor.
  ~MetaPlanner();
/*   static vector<NodeGenerationMethod<CFG>*> GetDefault(); */

  void MakeGlobalRoadmap(/* Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, */
/* 			 CollisionDetection* cd,  */
/* 			 DistanceMetric* dm, vector<CFG>& nodes */);

  void MakeRegionRoadmap(MPRegion<CFG,WEIGHT>* region);

  void RunningStatsStart();
  void RunningStatsStop();
  void RunningStatsWrite(ostream& _os);

  void FeatureStatsInit();

  void WriteRoadmap();
  int ReadCommandLine();
  void PrintUsage(ostream& _os);
  void PrintValues(ostream& _os);
  void PrintDefaults(ostream& _os);
/*   void PrintRawLine(ostream& _os, Roadmap<CFG, WEIGHT> *rmap,  */
/* 		    Clock_Class *NodeGenClock, Clock_Class *ConnectionClock, */
/* 		    int printHeader = 0 ); */


  void FeatureSensitiveMap(MPRegion<CFG,WEIGHT>* region, int tree_node_height, int maximum_tree_height, double epsilon);


 protected:
 public:
  GenerateMapNodes<CFG> gn_map; //@todo replace by MPStrategy::m_pNodeGeneration
  Connector<CFG, WEIGHT> cm_map; //@todo replace by MPStrategy::m_pConnection
  Connector<CFG, WEIGHT> cm_combine;
  LocalPlanners<CFG, WEIGHT> lp_map; //@todo replace by MPStrategy::m_pLocalPlanners
  DistanceMetric     dm; //@todo replace by MPProblem::m_pProblem::...

  bool addPartialEdge, addAllEdges; //@todo replace by MPStrategy::... (which will be moved to Connector

  CollisionDetection cd; //@todo replace by MPProblem::m_pProblem::...
  Environment env;

  ClockClass        NodeGenClock;
  ClockClass        ConnectionClock;


  // Specific for FSMP
  CSpaceCharacterizer<CFG,WEIGHT> characterizer;
  PartitionCSpaceRegion<CFG> partitioner;

  int maximum_tree_height;
  double epsilon;

  string environment_name;

  //temporary variable
#ifdef COLLISIONCFG
 vector< vector < vector <  double > > > CollisionConfiguration;
#endif



};


/////////////////////////////////////////////////////////////////////
//
//  definitions for class GenerateMapNodes declarations
//
/////////////////////////////////////////////////////////////////////

template <class CFG, class WEIGHT>
MetaPlanner<CFG,WEIGHT>::
MetaPlanner(Input& input, int i_maximum_tree_height, double i_epsilon,
	    int i_characterization_samples, int dofs, int pos_dofs, bool randomSeed=true)
  : env(dofs,pos_dofs,&input), maximum_tree_height(i_maximum_tree_height),
     epsilon(i_epsilon),
     characterizer(input, dm, cd, env, i_characterization_samples) {

  cout << "MetaPlanner(Input& input, bool randomSeed=true). TODO SOME" << endl;

  //Initialize random number generator
  struct timeval tv;
  if (randomSeed)
    gettimeofday(&tv,NULL);
  else
    timerclear(&tv);
  srand48((unsigned int) tv.tv_usec);

/*   this->vt = vt; */
/*   this->dm = dm; */
/*   this->env = env; */

  /** Instantiate roadmap object
   * it should only need info related to the graph, not the cd, dm, lp)
   */

/*   Roadmap<CFG, WEIGHT> rmap(&input,  &cd, &dm, &lp); */
/*   input.Read(PMPL_EXIT); */
/*   env.Get(&input); */

  //CollisionDetection cd;
  cd.ReadCommandLine(input.CDstrings, input.numCDs);
  //vt.SetCD(cd);
  lp_map.ReadCommandLine(input.LPstrings, input.numLPs, input.cdtype);
  dm.ReadCommandLine(input.DMstrings, input.numDMs);
  gn_map.ReadCommandLine(input.GNstrings, input.numGNs);
  cm_map.ReadCommandLine(&input, &env);
  cm_combine.ReadCommandLine(&input,&env);

  addPartialEdge = input.addPartialEdge.GetValue();
  addAllEdges = input.addAllEdges.GetValue();

  environment_name = input.defaultFile.GetValue();

  //FSMP initialization
  partitioner.ReadCommandLine(input.partitionType);


}


template <class CFG, class WEIGHT>
MetaPlanner<CFG,WEIGHT>::
~MetaPlanner() {
  cout << "~MetaPlanner() destructor. TODO SOME" << endl;
}


/* template <class CFG> */
/* vector<NodeGenerationMethod<CFG>*>  */
/* MetaPlanner<CFG>:: */
/* GetDefault() { */

/* } */

template <class CFG, class WEIGHT>
void MetaPlanner<CFG,WEIGHT>::
MakeGlobalRoadmap(/* Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, */
/* 	      CollisionDetection* cd,  */
/* 	      DistanceMetric* dm, vector<CFG>& nodes */) {
  cout << "MakeGlobalRoadmap() main function to create subdivision tree. TODO ALL" << endl;

  //make a new environment region (be careful with the pointer
  MPRegion<CFG,WEIGHT> whole_problem(env, *(env.GetBoundary()), 0, NULL);

  //make a global feature-sensitive roadmap
  cout << "\t Start of feature sensitive roadmap generation \n\t\tglobal_roadmap = feature_sensitive_map(environment,global_bb,region_characterization,region_features,map_of_region_features,\"no-parent-characterization\",subdivision_limit,node_height=0,maximum_tree_height,epsilon,characterization_samples)" << endl;
  FeatureSensitiveMap(&whole_problem, 0, maximum_tree_height,epsilon);//still needs: subdivision_limit

  // global_roadmap is the output
  cout << "\t return whole_problem.roadmap()" << endl;
}


template <class CFG, class WEIGHT>
void MetaPlanner<CFG,WEIGHT>::
MakeRegionRoadmap(MPRegion<CFG,WEIGHT>* region) {
  cout << " MetaPlanner::MakeRegionalRoadmap(). TODO ALL " << endl;

  //the code within the ifdef needs to be fixed
#ifdef COLLISIONCFG
  //Code to initialize stuff for collision nodes
    for(int i = 0; i< region->GetExternalBodyCount(); i++) {
      vector <vector<double> > new_vector;
      vector <double> tmp_vector;
      new_vector.push_back(tmp_vector);
      CollisionConfiguration.push_back(new_vector);

     for(int j = 0; j < gn_map.selected.size(); j++) {
       vector<CfgType> tmpCfgType;
       gn_map.selected[j]->m_vGeneratedCollisionConfiguration.push_back(tmpCfgType);
     }
    }
#endif


  // Planner options were set in Characterize

  // this should be only inside the region without affecting other regions
  //region->roadmap.GetEnvironment()->DeleteObstaclesOutsideBoundingBox();

  //---------------------------
  // Generate roadmap nodes
  //---------------------------

  NodeGenClock.StartClock("Node Generation");

  vector<CFG> nodes;
  gn_map.GenerateNodes(&(region->roadmap),region->map_stats,&cd,&dm,nodes);
  NodeGenClock.StopClock();
  cout << "\n";
  cout << "Node Generation: " << NodeGenClock.GetSeconds()
       << " sec (ie, " << NodeGenClock.GetUSeconds() << " usec)";

  cout << ", "<<region->roadmap.m_pRoadmap->get_num_vertices()<<" nodes\n"<< flush;

  //---------------------------
  // Connect roadmap nodes
  //---------------------------
  ConnectionClock.StartClock("Node Connection");
  cm_map.ConnectComponents(&(region->roadmap), region->map_stats, &cd, &dm, &lp_map,
			   addPartialEdge, addAllEdges);
  ConnectionClock.StopClock();

}



template <class CFG, class WEIGHT>
void MetaPlanner<CFG,WEIGHT>::
RunningStatsStart() {
  cout << "RunningStatsStart() Start running stats. TODO ALL" << endl;
  cout << "RunningStatsStart() Initialize map generation stats" << endl;
  cout << "RunningStatsStart() Initialize map combination stats" << endl;

}

template <class CFG, class WEIGHT>
void MetaPlanner<CFG,WEIGHT>::
RunningStatsStop() {
  cout << "RunningStatsStop() Start running stats. TODO ALL" << endl;
}

template <class CFG, class WEIGHT>
void MetaPlanner<CFG,WEIGHT>::
RunningStatsWrite(ostream& _os) {
  cout << "RunningStatsWrite() Output running stats. TODO ALL" << endl;
  //---------------------------
  // Print out some useful info
  //---------------------------

/*   ofstream  myofstream(input.mapFile.GetValue(),ios::app); */
/*   if (!myofstream) { */
/*     _os <<"\nIn main_obprm: can't re-open mapfile: " */
/* 	<<input.mapFile.GetValue(); */
/*     exit(-1); */
/*   } */
/*   PrintRawLine(_os, */
/* 	       &rmap, &NodeGenClock,&ConnectionClock,1);  // to stdout */
/*   PrintRawLine(myofstream, */
/* 	       &rmap, &NodeGenClock,&ConnectionClock,0);  // to map */
/*   _os << "\n"; */
/*   ConnectionClock.PrintName(); */
/*   _os << ": " << ConnectionClock.GetClock_SEC() */
/*       << " se	c" */
/*       << ", "<<rmap.m_pRoadmap->Get EdgeCount()<<" edges\n"<< flush; */
/*   Stats.PrintAllStats(&rmap); */
}

template <class CFG, class WEIGHT>
void MetaPlanner<CFG,WEIGHT>::
WriteRoadmap() {
  cout << "WriteRoadmap() output roadmap. TODO ALL" << endl;

  //---------------------------
  // Write roadmap
  //---------------------------
/*   rmap.WriteRoadmap(&input,&cd,&dm,&lp); */

}

template <class CFG, class WEIGHT>
void MetaPlanner<CFG,WEIGHT>::
FeatureStatsInit() {
  cout << "FeatureStatsInit() Initialize feature gathering stats. TODO ALL" << endl;

}


template <class CFG, class WEIGHT>
int MetaPlanner<CFG,WEIGHT>::
ReadCommandLine() {
}

template <class CFG, class WEIGHT>
void MetaPlanner<CFG,WEIGHT>::
PrintUsage(ostream& _os) {
}

template <class CFG, class WEIGHT>
void MetaPlanner<CFG,WEIGHT>::
PrintValues(ostream& _os) {
}

template <class CFG, class WEIGHT>
void MetaPlanner<CFG,WEIGHT>::
PrintDefaults(ostream& _os) {
}

/* template <class CFG, class WEIGHT> */
/* void MetaPlanner<CFG,WEIGHT>:: */
/* PrintRawLine(ostream& _os, */
/* 	     Roadmap<CFG, WEIGHT> *rmap,  */
/* 	     Clock_Class *NodeGenClock,  */
/* 	     Clock_Class *ConnectionClock, */
/* 	     int printHeader = 0 ){ */

/*   _os << "\nraw ";               // We can grep out "raw" datalines. */
/*   _os << NodeGenClock->GetClock_SEC()     << " "; */
/*   _os << NodeGenClock->GetClock_USEC()    << " "; */
/*   _os << ConnectionClock->GetClock_SEC()  << " "; */
/*   _os << ConnectionClock->GetClock_USEC() << " "; */
/*   Stats.PrintDataLine(_os,rmap,printHeader); */
/*   _os << "\n\n"; */
/* } */

template <class CFG, class WEIGHT>
void MetaPlanner<CFG,WEIGHT>::
FeatureSensitiveMap(MPRegion<CFG,WEIGHT>* region, int tree_node_height, int maximum_tree_height, double epsilon) {

  cout << "\t (region_characterization,region_features,map_of_region_features) = characterize_features(environment,global_bb,characterization_samples)" << endl;
  characterizer.Characterize(region);//still needs: characterization_samples


  // test feasibility of subdivision
  cout << "\t if feasible_to_subdivide(region,region_characterization,parent_characterization,subdivision_limit,node_height,maximum_tree_height) then " << endl;
  if (partitioner.isSubdivide(region,tree_node_height, maximum_tree_height)) {
    // Place boundaries
    vector<BoundingBox> subregion_boundaries = partitioner.PlaceBoundaries(region,
	   characterizer.gn_features.selected[0]->m_vGeneratedCollisionConfiguration);

    // Create new subregions with those boundaries
    vector<MPRegion<CFG,WEIGHT>* > subregions;
    vector<BoundingBox>::iterator bndr_itr;
    MPRegion<CFG,WEIGHT>* new_subregion;
    int s_index=0;
    for (bndr_itr = subregion_boundaries.begin(); bndr_itr < subregion_boundaries.end(); bndr_itr++) {
      new_subregion = new MPRegion<CFG,WEIGHT>(env,
					       *bndr_itr, s_index++,
					       region);
      new_subregion->PrintValues(cout);
      subregions.push_back(new_subregion);
    }

    // subregions is the result of partitioning

    cout << "\t\t for each region ri in subregions do " << endl;

    typename vector< MPRegion<CFG,WEIGHT>* >::iterator itr;
    for (itr = subregions.begin(); itr < subregions.end(); itr++) {
      // map region ri
      cout << "\t\t\t FeatureSensitiveMap(ri, ...) " << endl;
      /**itr = */
	FeatureSensitiveMap(*itr,tree_node_height+1, maximum_tree_height,
			  epsilon);
    }

    // NICAN NIYAUH (COMBINECSPACEREGIONS:BRUTEFORCECOMBINATION) combine partial solutions
    cout << "\t\t roadmap = CombineRoadmaps(subregions) " << endl;
    cm_combine.ConnectRegions(&cd, &dm, &lp_map, false, false,
			    subregions, region);
    string map_file = environment_name + "." + region->region_tag + ".cmb.map";
    region->roadmap.WriteRoadmap(&input,&cd,&dm,&lp_map,map_file.c_str());
  } else {

    // this region is a leave. Map it
    cout << "\t\t roadmap = MapRegion(region,region_characterization) " << endl;
    MakeRegionRoadmap(region);
    string map_file = environment_name + "." + region->region_tag + ".btm.map";
    region->roadmap.WriteRoadmap(&input,&cd,&dm,&lp_map,map_file.c_str());

  }
  // the region (along with updated roadmaps) is returned
  cout << "\t\t return region " << endl;
}

#endif
