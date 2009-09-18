///////////////////////////////////////////////////////////////////////////////
//  main_obprm.c		
//
///////////////////////////////////////////////////////////////////////////////

#include "SwitchDefines.h"

#include "OBPRMDef.h"
#include "Roadmap.h"
#include "Input.h"


#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "Weight.h"
#include "LocalPlanners.h"
#include "MapGenerator.h"

#include "CfgTypes.h"

#include "UniformSamplers.h"
#include "MedialAxisSamplers.h"
#include "GaussianSamplers.h"
#include "ObstacleBasedSamplers.h"

#include "Query.h"

#include "my_program_options.hpp"

Input input;
Stat_Class Stats; 

template <typename T>
ostream& operator<<(ostream& os, const vector<T>& v)
{
  copy(v.begin(), v.end(), ostream_iterator<T>(os, "\n\t"));
  return os;
}
template <typename T>
ostream& operator<<(ostream& os, const vector<T*>& v)
{
  typename vector<T*>::const_iterator V;
  for(V = v.begin(); V+1 != v.end(); ++V)
    os << *(*V) << "\n\t";
  os << *(*V);
  return os;
}

//========================================================================
//  main
//========================================================================
int main(int argc, char** argv)
{
  MapGenerator<CfgType, WeightType> mg;
  LocalPlanners<CfgType, WeightType> lp;
  DistanceMetric dm;
  CollisionDetection cd;
  CDInfo cdInfo;
  QueryConnect<CfgType, WeightType> qcm;

  //parse commandline
  QueryCmds Qinput;
  Qinput.ReadCommandLine(&argc,argv);

  input.ReadCommandLine(argc,argv);
  CfgType::setNumofJoints(input.numofJoints.GetValue());
  cout << "Cfg_free_tree::NumofJoints = " 
       << CfgType::getNumofJoints() << endl;
  Query<CfgType, WeightType> query(&Qinput);

  //long baseSeed = OBPRM_srand();
  long baseSeed = OBPRM_srand(input.seed.GetValue());
  //long baseSeed = OBPRM_srand((unsigned int) time(NULL));
  
  // create new environment
  CfgType test_cfg;
  Environment env(test_cfg.DOF(),test_cfg.posDOF(), &input);
 
  //create new roadmap
  Roadmap<CfgType, WeightType> rmap(&input,  &cd, &dm, &lp, baseSeed, &env);
  cout<<"the RNGseed in rmap is (from main): "<<rmap.GetRNGseed()<<endl;
  
  cd.ReadCommandLine(input.CDstrings, input.numCDs);
  lp.ReadCommandLine(input.LPstrings, input.numLPs, input.cdtype);
  mg.gn.ReadCommandLine(input.GNstrings, input.numGNs);
  mg.cm.ReadCommandLine(&input, &env);
  dm.ReadCommandLine(input.DMstrings, input.numDMs);
  qcm.ReadCommandLine(&input, &env);

  //variables for node classification
  double expand_threshold = 0.5; // read as a parameter	
  double expansion_ratio = 0.0;
  double sum_cc_diameter(0.0), largest_cc_diameter(0.0);
  string features_filename;  
  int total_nodes(0), cc_create_nodes(0), cc_merge_nodes(0), 
    cc_expand_nodes(0), cc_oversample_nodes(0), num_in_passage(0);
  
  //parse biased sampler options
  int num_nodes;
  int batch_size;
  int num_tries;
  vector<string> sampler_params;
  vector<BaseSampler<CfgType>*> samplers;
  vector<BoundingBox> passages;
  try {
    //setop option description for general options
    po::options_description options("Allowed Options");
    options.add_options()
      ("nodes", po::value<int>(&num_nodes)->default_value(1000), "total number of nodes to generate")
      ("batch-size", po::value<int>(&batch_size)->default_value(100), "size of node batches/increments")
      ("tries", po::value<int>(&num_tries)->default_value(1), "the number of attempts to sample each node")
      ("sampler", po::value<vector<string> >(&sampler_params), "sampler parameters")
      ("expand-threshold", po::value<double>(&expand_threshold), "minimum expansion for a node to be considered cc-expand")
      ("features-file", po::value<string>(&features_filename), "file to store colon-separated list of features for each node sampled")
      ("passage", po::value<vector<string> >(), "narrow passage bbox")
      ;
    
    //parse and store the general options
    po::variables_map vm;
    ifstream config("samplers.in");
    po::store(po::parse_config_file(config, options), vm);
    config.close();
    po::notify(vm);

    //parse and store the sampler list
    for(vector<string>::const_iterator S = sampler_params.begin();
	S != sampler_params.end(); ++S) {
      istringstream iss(*S);
      //read sampler name
      string name;
      if(iss >> name) {
	//create new BaseSampler based on name and add to samplers list
	BaseSampler<CfgType>* psampler;
	if(name == "UniformRandom") 
	  psampler = new UniformRandomSampler<CfgType>(&env, Stats, *S);	
	else if(name == "UniformRandomFree")
	  psampler = new UniformRandomFreeSampler<CfgType>(&env, Stats, &cd, cdInfo, *S);
	else if(name == "UniformRandomCollision") 
	  psampler = new UniformRandomCollisionSampler<CfgType>(&env, Stats, &cd, cdInfo, *S);
	else if(name == "GaussRandomFree") 
	  psampler = new GaussRandomSampler<CfgType,true>(&env, Stats, &cd, cdInfo, 
							  &dm, *S);
	else if(name == "GaussRandomCollision")
	  psampler = new GaussRandomSampler<CfgType,false>(&env, Stats, &cd, cdInfo, 
							   &dm, *S);
	else if(name == "BridgeTest") 
	  psampler = new BridgeTestRandomFreeSampler<CfgType>(&env, Stats, &cd, cdInfo,
							      &dm, *S);
	else if(name == "ObstacleBased")
	  psampler = new ObstacleBasedSampler<CfgType>(&env, Stats, &cd, cdInfo,
						       &dm, *S);
	else if(name == "FreeMedialAxis") 
	  psampler = new FreeMedialAxisSampler<CfgType>(&env, Stats, &cd, cdInfo,
							&dm, *S);
	samplers.push_back(psampler);
      }
    }

    //parse and store passages list
    if(vm.count("passage")) {
      vector<string> str_vec = vm["passage"].as<vector<string> >();
      for(vector<string>::const_iterator S = str_vec.begin();
	  S != str_vec.end(); ++S) {
	vector<double> ranges;
	istringstream is(*S);
	ranges.insert(ranges.end(),
		      istream_iterator<double>(is),
		      istream_iterator<double>());
	for(vector<double>::const_iterator I = ranges.begin();
	    I != ranges.end() && (I+1) != ranges.end(); I += 2) {
	  if(*I > *(I+1)) {
	    cerr << "Error [" << *S << "] is not a valid bbox\n";
	    return -1;
	  }
	}
	CfgType tmpcfg;
	BoundingBox bbox(tmpcfg.DOF(), tmpcfg.posDOF());
	bbox.SetRanges(ranges);
	passages.push_back(bbox);
      }
    }
  }
  catch(exception& e) {
    cerr << e.what() << endl;
    return -1;
  }

  //add extension to filename
  features_filename += ".char";
    
  input.PrintValues(cout);
  cout << "The recorded samplers are: \n\t" << samplers << endl << endl;
  cout << "num_nodes = " << num_nodes << endl;
  cout << "batch_size = " << batch_size << endl;
  cout << "num_tries = " << num_tries << endl;
  cout << "expand-threshold = " << expand_threshold << endl;
  cout << "features_filename = " << features_filename << endl;
  cout << "The passage list is:";
  for(vector<BoundingBox>::const_iterator P = passages.begin();
      P != passages.end(); ++P) {
    cout << "\n\t";
    P->Print(cout);
  }
  cout << endl;

  std::ofstream features_out(features_filename.c_str());
  features_out << "#\n#" << input.envFile.GetValue() 
	       << ":" << rmap.GetRNGseed();
  features_out << ":";
  for(vector<BaseSampler<CfgType>*>::iterator S = samplers.begin();
      S != samplers.end(); ++S) {
    features_out << (*S)->name();
    if(S+1 != samplers.end())
      features_out << "-";
  }
  features_out << ":Closest";
  features_out << endl;
  ofstream timing_out("timing.dat");
  timing_out << "#nodes:time\n";

  features_out << "#numnodes:cc_create:cc_merge:cc_expand:cc_oversample:expansion_ratio:largest_cc_dia:sum_cc_dia:witness_query:num_in_passage" << endl;

  //map generation
  Clock_Class MapGenClock;
  MapGenClock.StartClock("Map Generation");

  double actual_time = 0.0;
  vector<VID> allnodesVID;
  for(int i=0; i<num_nodes; i+=batch_size) {
    //for debugging/testing
    /*
    if((i != 0) && (i % 250 == 0)) {
      char mapname[100];
      sprintf(mapname, "%s.%d.map", input.defaultFile.GetValue(), i);
      rmap.WriteRoadmap(&input,&cd,&dm,&lp,mapname);
    }
    */

    Clock_Class NodeGenClock;
    NodeGenClock.StartClock("Node Generation");

    //create an initial set of cfgs to iterate over
    vector<CfgType> nodes(batch_size, CfgType());

    //for each sampler, generate biased nodes
    for(vector<BaseSampler<CfgType>*>::iterator S = samplers.begin();
	S != samplers.end(); ++S) {
      Clock_Class SamplerClock;
      SamplerClock.StartClock((*S)->name());

      vector<CfgType> out_nodes;
      sample(**S, 
	     nodes.begin(), nodes.end(),
	     back_insert_iterator<vector<CfgType> >(out_nodes),
	     num_tries);
      //not sure if we want to trim these...
      //if(out_nodes.size() > batch_size) //e.g., for obprm with shells
      //erase(out_nodes.begin()+batch_size, out_nodes.end());
      nodes = out_nodes;

      cout << "\t";
      SamplerClock.StopPrintClock();
      cout << "\t\t (" << nodes.size() << " nodes)\n";
    }

    NodeGenClock.StopPrintClock();
    actual_time += NodeGenClock.GetClock_SEC();

    //connect and evaluate each node, one by one
    double connection_time = 0.0;
    double evaluation_time = 0.0;
    stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;
    expansion_ratio = 0.0;
    for(vector<CfgType>::iterator N = nodes.begin();
	N != nodes.end(); ++N) {
      cmap.reset();
      int previous_cc_number = get_cc_count(*(rmap.m_pRoadmap),cmap);
      int previous_lp_attempts = Stats.LPAttempts[0];
      //int previous_lp_connects = Stats.LPConnections[0];
      int previous_lp_connects = rmap.m_pRoadmap->get_num_edges()/2;
      //cout << "previous lp conn " << previous_lp_connects << endl;
      //cout << "previous lp att " << previous_lp_attempts << endl;

      Clock_Class ConnClock;
      ConnClock.StartClock("Node Connection");

      VID vid = rmap.m_pRoadmap->AddVertex(*N);
      vector<VID> v(1, vid);
      mg.cm.ConnectNodes(&rmap, Stats, &cd, &dm, &lp,
			 input.addPartialEdge.GetValue(),
			 input.addAllEdges.GetValue(),
			 v, allnodesVID);
      allnodesVID.push_back(vid);

      ConnClock.StopClock();
      connection_time += ConnClock.GetClock_SEC();
      actual_time += ConnClock.GetClock_SEC();

      Clock_Class EvalClock;
      EvalClock.StartClock("Node Evaluation");

      cmap.reset();
      int current_cc_number = get_cc_count(*(rmap.m_pRoadmap),cmap);    
      //cout << "prev_cc_num = " << previous_cc_number 
      //   << " curr_cc_num = " << current_cc_number
      //   << endl;
      if (current_cc_number > previous_cc_number) { // CC Create
	expansion_ratio += 1.0;
	cc_create_nodes++;

      } else if (current_cc_number < previous_cc_number) { // CC merge
	expansion_ratio += 1.0;
	cc_merge_nodes++;

      } else { // either CC expand or CC oversample
	int lp_attempts = Stats.LPAttempts[0] - previous_lp_attempts;
	//int lp_connects = Stats.LPConnections[0] - previous_lp_connects;
	int lp_connects = rmap.m_pRoadmap->get_num_edges()/2 - previous_lp_connects;
	////cout << "current lp conn " << Stats.LPConnections[0] << endl;
	//cout << "current lp conn " << rmap.m_pRoadmap->Get EdgeCount()/2 << endl;
	//cout << "current lp att " << Stats.LPAttempts[0] << endl;

	//cout << "lp: " << lp_connects << " / " << lp_attempts << endl;
	if (lp_attempts != 0)
	  expansion_ratio += 1.0 - double(lp_connects)/double(lp_attempts);
	else
	  cerr << "Warning: lp_attempts was 0\n";

	//compute list of neighbor neighbors to vid:
	//cout << "vid = " << vid << endl;
	vector<VID> first_neighbors;
	(rmap.m_pRoadmap)->get_successors(vid, first_neighbors);
	vector<VID> second_neighbors;
	for(vector<VID>::const_iterator FN = first_neighbors.begin();
	    FN != first_neighbors.end(); ++FN) {
	  vector<VID> adjacent;
	  rmap.m_pRoadmap->get_successors(*FN, adjacent);
	  second_neighbors.insert(second_neighbors.end(),
				  adjacent.begin(), adjacent.end());
	}
	//set_difference requires sorted ranges
	sort(first_neighbors.begin(), first_neighbors.end());
	//cout << "f_n = "; copy(first_neighbors.begin(), first_neighbors.end(), ostream_iterator<VID>(cout, " ")); cout << endl;
	sort(second_neighbors.begin(), second_neighbors.end());
	//cout << "s_n = "; copy(second_neighbors.begin(), second_neighbors.end(), ostream_iterator<VID>(cout, " ")); cout << endl;
	//make sure there are no duplicate second neighbors
	second_neighbors.erase(unique(second_neighbors.begin(),
				      second_neighbors.end()),
			       second_neighbors.end());
	//cout << "s_n = "; copy(second_neighbors.begin(), second_neighbors.end(), ostream_iterator<VID>(cout, " ")); cout << endl;
	//only keep second neighbors that are also not first neighbors
	vector<VID> neighbors;
	set_difference(second_neighbors.begin(), second_neighbors.end(),
		       first_neighbors.begin(), first_neighbors.end(),
		       back_insert_iterator<vector<VID> >(neighbors));
	//remove vid
	neighbors.erase(remove_if(neighbors.begin(), neighbors.end(),
				  bind2nd(equal_to<VID>(), vid)));
	//cout << "n = "; copy(neighbors.begin(), neighbors.end(), ostream_iterator<VID>(cout, " ")); cout << endl;
	
	//check for unconnectable edges between *N and neighbors
	bool found_unconnectable_edge = false;
	if(neighbors.empty()) //special case
	  found_unconnectable_edge = true;
	for(vector<VID>::const_iterator NN = neighbors.begin();
	    NN != neighbors.end(); ++NN) {
	  LPOutput<CfgType, WeightType> lpOutput;
	  if(!lp.IsConnected(&env, Stats, &cd, &dm,
			     *N, rmap.m_pRoadmap->find_vertex(*NN).property(),
			     &lpOutput,
			     ConnectMap<CfgType,WeightType>::connectionPosRes,
			     ConnectMap<CfgType,WeightType>::connectionOriRes)) {
	    found_unconnectable_edge = true;
	    break;
	  }
	}
	//cout << "found_unconnectable_edge = " << found_unconnectable_edge << endl;
	if(found_unconnectable_edge) // CC expand
	  cc_expand_nodes++;
	else // CC oversample
	  cc_oversample_nodes++;
      }

      //check if in any narrow passage
      for(vector<BoundingBox>::const_iterator P = passages.begin();
	  P != passages.end(); ++P) {
	/*
	cout << "com = " << N->GetRobotCenterPosition() << endl;
	cout << "satisifes bbox ";
	P->Print(cout);
	cout << " = " << P->IfSatisfiesConstraints(N->GetRobotCenterPosition()) 
	     << endl;
	*/
	if(P->IfSatisfiesConstraints(N->GetRobotCenterofMass(&env))) {
	  num_in_passage++;
	  break;
	}
      }

      EvalClock.StopClock();
      evaluation_time += EvalClock.GetClock_SEC();
    }
    cout << "Node Connection: " << connection_time << " sec\n";


    Clock_Class EvalClock;
    EvalClock.StartClock("Node Evaluation");

    total_nodes += nodes.size();
    expansion_ratio /= nodes.size(); //average expansion ratio over batch

    vector < pair < size_t, VID > > components;
    cmap.reset();
    get_cc_stats(*(rmap.m_pRoadmap), cmap, components);
    sum_cc_diameter = 0;
    largest_cc_diameter = 0;
    for (int cc=0; cc < components.size(); ++cc) {
      VID far_vertex_1, far_vertex_2;
      double cc_diameter = ComponentDiameter(*(rmap.m_pRoadmap),components[cc].second, &far_vertex_1);
      cc_diameter = ComponentDiameter(*(rmap.m_pRoadmap),far_vertex_1, &far_vertex_2);
      sum_cc_diameter += cc_diameter;
      largest_cc_diameter = max(largest_cc_diameter, cc_diameter);
    }

    bool query_solved = query.PerformQuery(&rmap, Stats, &cd, &qcm, &lp, &dm);

    EvalClock.StopClock();
    evaluation_time += EvalClock.GetClock_SEC();
    cout << "Roadmap/Node Evaluation: " << evaluation_time << " sec\n";

    // save stats in features file
    features_out << total_nodes << ":" 
		 << cc_create_nodes << ":" << cc_merge_nodes << ":" 
		 << cc_expand_nodes << ":" << cc_oversample_nodes << ":" 
		 << expansion_ratio << ":" 
		 << largest_cc_diameter << ":" << sum_cc_diameter << ":"
		 << query_solved << ":"
		 << num_in_passage
		 << endl;
    timing_out << total_nodes << ":" << actual_time << endl;
  }
  MapGenClock.StopClock();
 
  rmap.WriteRoadmap(&input,&cd,&dm,&lp);

  cout << "\n";
  MapGenClock.PrintName();
  cout << ": " << MapGenClock.GetClock_SEC()
       << " sec"
       << ", "<<rmap.m_pRoadmap->get_num_vertices()<<" nodes"
       << ", "<<rmap.m_pRoadmap->get_num_edges()<<" edges\n"<< flush;
  Stats.PrintAllStats(&rmap);
  cout << "\n  !!Bye!! \n";

  //delete samplers
  for(vector<BaseSampler<CfgType>*>::iterator S = samplers.begin();
      S != samplers.end(); ++S)
    delete *S;

  return 0;
}
