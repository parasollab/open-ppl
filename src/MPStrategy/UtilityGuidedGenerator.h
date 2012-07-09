#ifndef UtilityGuidedGenerator_h
#define UtilityGuidedGenerator_h


///////////////////////////////////////////////////////////////////////////////
//
//
//  class UtilityGuidedGenerator
//
//
///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
class UtilityGuidedGenerator {
 public:
  
  //////////////////////
  // Constructors and Destructor
  UtilityGuidedGenerator();
  ~UtilityGuidedGenerator();

  char* GetName();

  //////////////////////
  // I/O methods
  void ReadCommandLine(n_str_param* GNstrings[MAX_GN], int numGNs);
  void ParseCommandLine(int argc, char **argv);
  void PrintUsage(ostream& _os);
  void PrintValues(ostream& _os);
  void PrintOptions(ostream& out_os){};

  CFG GenerateEntropyGuidedSample(Roadmap<CFG, WEIGHT>* rmap, StatClass& Stats,
				  CollisionDetection* cd, CDInfo& cdInfo, 
				  DistanceMetric* dm,
				  double component_dist, double tao);

  void GenerateMap(Roadmap<CFG, WEIGHT>* rmap, StatClass& Stats,
		     CollisionDetection* cd, 
		     DistanceMetric* dm, vector<CFG>& nodes, 
		     LocalPlanners<CFG,WEIGHT>* lp,
		     Input* input); 

  //Data
  num_param<int> numNodes;
  num_param<int> exactNodes;
  num_param<int> ksamples;
  num_param<int> kneighbors;
  num_param<double> component_dist;
  num_param<double> tao;
  num_param<int> kclosest;
  
};


/////////////////////////////////////////////////////////////////////
//
//  definitions for class UtilityGuidedGenerator declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
UtilityGuidedGenerator<CFG,WEIGHT>::
UtilityGuidedGenerator() : 
  numNodes("nodes", 10, 1, 5000000),
     exactNodes ("exact", 0, 0, 1),
     ksamples ("ksamples", 5, 0, 5000000),
     kneighbors ("kneighbors", 10, 0, 5000000),
     component_dist("dist", 0.5, 0, 5000000),
     tao("tao", 0.01, 0, 5000000),
     kclosest("kclosest", 20, 0, 5000000)
{
  numNodes.PutDesc("INTEGER","(number of nodes, default 10)");
  exactNodes.PutDesc("INTEGER","(whether to generate exact num of nodes, default 0)");
  ksamples.PutDesc("INTEGER","(number of samples to select from during each round, default 5)");
  kneighbors.PutDesc("INTEGER","(number of neighbors to look at when determining the probability a sample is free, default 10)");
  component_dist.PutDesc("DOUBLE","(distance threshold between ccs, default 0.5)");
  tao.PutDesc("DOUBLE","(perturb amount, default 0.01)");
  kclosest.PutDesc("INTEGER","(number of neighbors to attempt connections with, default 20)");
}


template <class CFG, class WEIGHT>
UtilityGuidedGenerator<CFG,WEIGHT>::
~UtilityGuidedGenerator() {
}


template <class CFG, class WEIGHT>
char*
UtilityGuidedGenerator<CFG,WEIGHT>::
GetName() {
  return "UtilityGuidedGenerator";
}


template <class CFG, class WEIGHT>
void
UtilityGuidedGenerator<CFG,WEIGHT>::
ReadCommandLine(n_str_param* GNstrings[MAX_GN], int numGNs) {
  //go through the command line looking for method names
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
}


template <class CFG, class WEIGHT>
void
UtilityGuidedGenerator<CFG,WEIGHT>::
ParseCommandLine(int argc, char **argv) {
  int i;
  for (i=1; i < argc; ++i) {
    if(numNodes.AckCmdLine(&i, argc, argv)) {
    } else if(exactNodes.AckCmdLine(&i, argc, argv)) {
    } else if(ksamples.AckCmdLine(&i, argc, argv)) {
    } else if(kneighbors.AckCmdLine(&i, argc, argv)) {
    } else if(component_dist.AckCmdLine(&i, argc, argv)) {
    } else if(tao.AckCmdLine(&i, argc, argv)) {
    } else if(kclosest.AckCmdLine(&i, argc, argv)) {
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


template <class CFG, class WEIGHT>
void
UtilityGuidedGenerator<CFG,WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);  
  _os << "\n" << GetName() << " ";
  _os << "\n\t"; numNodes.PrintUsage(_os);
  _os << "\n\t"; exactNodes.PrintUsage(_os);
  _os << "\n\t"; ksamples.PrintUsage(_os);
  _os << "\n\t"; kneighbors.PrintUsage(_os);
  _os << "\n\t"; component_dist.PrintUsage(_os);
  _os << "\n\t"; tao.PrintUsage(_os);
  _os << "\n\t"; kclosest.PrintUsage(_os);
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
UtilityGuidedGenerator<CFG,WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << numNodes.GetFlag() << " " << numNodes.GetValue();
  _os << exactNodes.GetFlag() << " " << exactNodes.GetValue();
  _os << ksamples.GetFlag() << " " << ksamples.GetValue();
  _os << kneighbors.GetFlag() << " " << kneighbors.GetValue();
  _os << component_dist.GetFlag() << " " << component_dist.GetValue();
  _os << tao.GetFlag() << " " << tao.GetValue();
  _os << kclosest.GetFlag() << " " << kclosest.GetValue();
  _os << endl;
}

/*
template <class P>
struct distance_compare_first : public binary_function<P, P, bool> {
  Environment* env;
  DistanceMetric* dm;
  typename P::first_type cfg;

  distance_compare_first(Environment* e, 
			 DistanceMetric* d, 
			 const typename P::first_type& c) : 
    env(e), dm(d), cfg(c) {}
  ~distance_compare_first() {}

  bool operator()(const P& p1, const P& p2) const {
    return (dm->Distance(env, cfg, p1.first) <
	    dm->Distance(env, cfg, p2.first));
  }
};


template <class P>
struct plus_second : public binary_function<typename P::second_type, 
					    P, 
					    typename P::second_type> {
  typename P::second_type operator()(const typename P::second_type& p1, 
				     const P& p2) const {
    return plus<typename P::second_type>()(p1, p2.second);
  }
};


template <class CFG>
struct ApproximateCSpaceModel {
  vector<pair<CFG,double> > model_nodes;
  Environment* env;
  DistanceMetric* dm;

  ApproximateCSpaceModel(Environment* _env, 
			 DistanceMetric* _dm) : 
    env(_env), dm(_dm) {}
  ~ApproximateCSpaceModel() {}

  void AddSample(const CFG& c, double coll) {
    model_nodes.push_back(make_pair(c, coll));
  }

  double FreeProbability(const CFG& c, int k) {
    sort(model_nodes.begin(), model_nodes.end(), 
	 distance_compare_first<pair<CFG,double> >(env, dm, c));
    int size = (int)min(k, model_nodes.size());
    if(size == 0)
      return 0.0;
    else
      return accumulate(model_nodes.begin(), model_nodes.begin()+size,
			0.0, plus_second<pair<CFG,double> >()) / size;
  }
};
*/

template <class CFG, class WEIGHT>
CFG
UtilityGuidedGenerator<CFG,WEIGHT>::
GenerateEntropyGuidedSample(Roadmap<CFG, WEIGHT>* rmap, StatClass& Stats,
                            CollisionDetection* cd, CDInfo& cdInfo, 
			    DistanceMetric* dm,
			    double component_dist, double tao) {
  CFG q1, q2;
  
  stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
  vector<pair<size_t,VID> > ccs;
  if(get_cc_stats(*rmap->m_pRoadmap, cmap, ccs) == 1) {
    q1 = (*(rmap->m_pRoadmap->find_vertex(ccs[0].second))).property();
    q2.GetRandomCfg(rmap->GetEnvironment());
  } else {
    //randomly select 2 ccs that are within a threshold component_dist of each other
    VID cc1vid, cc2vid;
    vector<VID> cc1, cc2;
    double dist;
    do {
      cc1vid = ccs[(int)floor((double)OBPRM_drand()*(double)ccs.size())].second;
      do {
	cc2vid = ccs[(int)floor((double)OBPRM_drand()*(double)ccs.size())].second;
      } while (cc1vid == cc2vid);
      
      cmap.reset();
      get_cc(*rmap->m_pRoadmap, cmap, cc1vid, cc1);
      cmap.reset();
      get_cc(*rmap->m_pRoadmap, cmap, cc2vid, cc2);
      
      vector<pair<VID,VID> > kp = dm->FindKClosestPairs(rmap, cc1, cc2,1);
      dist = dm->Distance(rmap->GetEnvironment(), 
			  (*(rmap->m_pRoadmap->find_vertex(kp[0].first))).property(),
			  (*(rmap->m_pRoadmap->find_vertex(kp[1].second))).property());
    } while (dist > component_dist);
    
    //randomly select a node in each cc
    q1 = (*(rmap->m_pRoadmap->find_vertex(cc1[(int)floor((double)OBPRM_drand()*(double)cc1.size())]))).property();
    q2 = (*(rmap->m_pRoadmap->find_vertex(cc2[(int)floor((double)OBPRM_drand()*(double)cc2.size())]))).property();
  }

  //return perturbation of the midpoint between the two nodes
  CFG qn = q1;
  qn.add(qn, q2);
  qn.divide(qn, 2);
  CFG q = qn;
  q.GetRandomRay(OBPRM_drand()*tao, rmap->GetEnvironment(), dm);
  q.add(qn, q);
  return q;
}


template <class CFG, class WEIGHT>
void 
UtilityGuidedGenerator<CFG,WEIGHT>::
GenerateMap(Roadmap<CFG, WEIGHT>* rmap, StatClass& Stats,
	    CollisionDetection* cd, 
	    DistanceMetric* dm, vector<CFG>& nodes, 
	    LocalPlanners<CFG,WEIGHT>* lp,
	    Input* input) {
  cout << "numNodes = " << numNodes.GetValue() << endl;

  ApproximateCSpaceModel<CFG> model(rmap->GetEnvironment(), dm);

  CDInfo cdInfo;

  Closest<CFG, WEIGHT> connect_kclosest;
  connect_kclosest.kclosest = kclosest.GetValue();
  connect_kclosest.mfailure = kclosest.GetValue();
  connect_kclosest.cdInfo = &cdInfo;
  connect_kclosest.connectionPosRes = rmap->GetEnvironment()->GetPositionRes();
  connect_kclosest.connectionOriRes = rmap->GetEnvironment()->GetOrientationRes();

  for(int i=0; i<numNodes.GetValue(); ++i) {

    //get number of nodes, if < 1, simply add a random sample to the roadmap
    CFG q;
    if(rmap->m_pRoadmap->get_num_vertices() < 1) {
      q.GetFreeRandomCfg(rmap->GetEnvironment(), Stats, cd, cdInfo);
      model.AddSample(q, 1);
      nodes.push_back(q);
      rmap->m_pRoadmap->AddVertex(q);
      continue;
    }

    q = GenerateEntropyGuidedSample(rmap, Stats, cd, cdInfo, dm, 
				    component_dist.GetValue(), tao.GetValue());
    double q_prob_free = model.FreeProbability(q, kneighbors.GetValue());
    cout << "q (" << q_prob_free << ") = " << q << endl;
    for(int j=1; j<ksamples.GetValue(); ++j) {
      CFG q_new = GenerateEntropyGuidedSample(rmap, Stats, cd, cdInfo, dm,
					      component_dist.GetValue(), tao.GetValue());
      double q_new_prob_free = model.FreeProbability(q_new, kneighbors.GetValue());
      cout << "\tq' (" << q_new_prob_free << ") = " << q_new << endl;
      if(q_new_prob_free > q_prob_free) {
        q = q_new;
        q_prob_free = q_new_prob_free;
      }
      cout << "q (" << q_prob_free << ") = " << q << endl;
    }

    //add the sample to the model and roadmap (if free)
    string callee("UtilityGuidedGenerator::NodeGeneration");
    bool isColl = !q.InBoundary(rmap->GetEnvironment()) || 
      q.isCollision(rmap->GetEnvironment(), Stats, cd, cdInfo, true, &callee);
    cout << "isColl = " << isColl;
    if(!isColl) {
      model.AddSample(q, 1);

      vector<VID> v2;
      rmap->m_pRoadmap->GetVerticesVID(v2);

      nodes.push_back(q);
      VID qvid = rmap->m_pRoadmap->AddVertex(q);
      vector<VID> v1(1, qvid);
      cout << "\tadding to roadmap (" << rmap->m_pRoadmap->get_num_vertices() << ")";
 
      connect_kclosest.Connect(rmap, Stats, cd, dm, lp, 
			       input->addPartialEdge.GetValue(), 
			       input->addAllEdges.GetValue(),
			       v1, v2);
    } else {
      model.AddSample(q, 0);
      if(exactNodes.GetValue() == 1)
	i--;
    }
    cout << endl;
  }

}


#endif


