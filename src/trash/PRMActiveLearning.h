#ifndef PRMActiveLearning_h
#define PRMActiveLearning_h

#include "NodeGeneratorMethod.h"

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class PRMActiveLearning
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**This generates PRM nodes.  This class is derived off of NodeGenerationMethod.
 */
template <class CFG>
class PRMActiveLearning: public NodeGenerationMethod<CFG> {
 public:
  
  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Constructors and Destructor*/
  //@{

  ///Default Constructor.
  PRMActiveLearning();
  PRMActiveLearning(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  ///Destructor.	
  virtual ~PRMActiveLearning();

  //@}

  //////////////////////
  // Access
  virtual char* GetName();
  virtual int GetNextNodeIndex();
  virtual void SetNextNodeIndex(int);
  virtual void IncreaseNextNodeIndex(int);
  virtual void ParseXML(XMLNodeReader& in_Node);

  //////////////////////
  // I/O methods
  ///Used in new MPProblem framework.
  virtual void PrintOptions(ostream& out_os);
  virtual NodeGenerationMethod<CFG>* CreateCopy();

  /**Active Learning Based Node Generation.
   *This method is based of off "Towards Optimal Configuration Space Sampling"
   * by Burns and Brock, RSS 2005.
   */
  virtual void GenerateNodes(Environment* _env, Stat_Class& Stats,
			     CollisionDetection* cd, 
			     DistanceMetric *dm, vector<CFG>& nodes);
  
  virtual void GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG >  &outCfgs);

  //Index for next node 
  //used in incremental map generation
  static int nextNodeIndex;

  int ksamples; ///< number of samples to select from during each round, default 5
  int kneighbors; ///< number of neighbors to look at when determining the probability a sample is free, default 10
};


template <class CFG>
int PRMActiveLearning<CFG>::nextNodeIndex = 0;

/////////////////////////////////////////////////////////////////////
//
//  definitions for class PRMActiveLearning declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG>
PRMActiveLearning<CFG>::
PRMActiveLearning() : NodeGenerationMethod<CFG>() {
   // SetDefault();
}


template <class CFG>
PRMActiveLearning<CFG>::
~PRMActiveLearning() {
}


template <class CFG>
PRMActiveLearning<CFG>::
PRMActiveLearning(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
NodeGenerationMethod<CFG>(in_Node, in_pProblem) {
  LOG_DEBUG_MSG("PRMActiveLearning::PRMActiveLearning()");
  ParseXML(in_Node);
  LOG_DEBUG_MSG("~PRMActiveLearning::PRMActiveLearning()");
}


template <class CFG>
char*
PRMActiveLearning<CFG>::
GetName() {
  return "PRMActiveLearning";
}


template <class CFG>
void
PRMActiveLearning<CFG>::
ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("PRMActiveLearning::ParseXML()");

  in_Node.verifyName(string("PRMActiveLearning"));
  
  ksamples = in_Node.numberXMLParameter(string("ksamples"),true,5,1,MAX_INT,string("ksamples"));

  kneighbors = in_Node.numberXMLParameter(string("kneighbors"),
                  true,10,1,MAX_INT,string("kneighbors"));

  PrintOptions(cout);
  LOG_DEBUG_MSG("~PRMActiveLearning::ParseXML()");
}


template <class CFG>
int
PRMActiveLearning<CFG>::
GetNextNodeIndex() {
  return nextNodeIndex;
}


template <class CFG>
void
PRMActiveLearning<CFG>::
SetNextNodeIndex(int index) {
  nextNodeIndex = index;
}


template <class CFG>
void
PRMActiveLearning<CFG>::
IncreaseNextNodeIndex(int numIncrease) {
  nextNodeIndex += numIncrease;
}


template <class CFG>
NodeGenerationMethod<CFG>* 
PRMActiveLearning<CFG>::
CreateCopy() {
  NodeGenerationMethod<CFG> * _copy = new PRMActiveLearning<CFG>(*this);
  return _copy;
}


template <class CFG>
void
PRMActiveLearning<CFG>::
PrintOptions(ostream& out_os){
  out_os << "    " << GetName() << ":: ";
  out_os << " num nodes = " << this->numNodes << " ";
  out_os << " exact = " << this->exactNodes << " ";
  out_os << " chunk size = " << this->chunkSize << " ";
  out_os << " MaxCDCalls = " << this->m_nMaxCdCalls << " ";
  out_os << " ksamples = " << this->ksamples << " ";
  out_os << " kneighbors = " << this->kneighbors << " ";
  out_os << endl;
}


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


template <class CFG>
void
PRMActiveLearning<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats,
	      CollisionDetection* cd, DistanceMetric* dm,
	      vector<CFG>& nodes) {
  string callee("PRMActiveLearning::GenerateNodes");
  LOG_DEBUG_MSG("PRMActiveLearning::GenerateNodes()");	
#ifndef QUIET
  if (this->exactNodes==1)
     cout << "(numNodes=" << this->numNodes << ") ";
  else
    cout << "(exactNodes=" << this->exactNodes << ") ";
#endif
  
  vector<Cfg*> path;

  ApproximateCSpaceModel<CFG> model(_env, dm);

  for(int i=0; i<this->numNodes; ++i) {
    Stats.IncNodes_Attempted();

    //generate a list of random samples to select the 1 with highest probability of being free
    vector<CFG> samples;
    for(int j=0; j<ksamples; ++j) {
      CFG c;
      c.GetRandomCfg(_env);
      samples.push_back(c);
    }
    
    //compute the probability each sample is free given samples in the model already
    vector<double> free_prob;
    for(typename vector<CFG>::const_iterator S = samples.begin(); 
	S != samples.end(); ++S) {
      free_prob.push_back(model.FreeProbability(*S, kneighbors));
    }
    
    //select the sample with the greatest probability
    vector<double>::const_iterator max_prob;
    max_prob = max_element(free_prob.begin(), free_prob.end());
    typename vector<CFG>::iterator S = samples.begin() +
      distance((vector<double>::const_iterator)free_prob.begin(), max_prob);

    //add the sample to the model and roadmap (if free)
    string callee("PRMActiveLearning::GenerateNodes");
    bool isColl = !S->InBoundingBox(_env) || 
      S->isCollision(_env, Stats, cd, *this->cdInfo, true, &callee);
    if(!isColl) {
      model.AddSample(*S, 1);
      nodes.push_back(*S);
      path.push_back((Cfg*)S->CreateNewCfg());
      Stats.IncNodes_Generated();
    } else {
      model.AddSample(*S, 0);
      if(this->exactNodes == 1)
	i--;
    }
  }

#if INTERMEDIATE_FILES
  //in util.h
  WritePathConfigurations("activelearning.path", path, _env);
#endif	
  for(int i=0; i<path.size(); i++)
    if (path[i] != NULL)
      delete path[i];  
  
  LOG_DEBUG_MSG("~PRMActiveLearning::GenerateNodes()"); 
}


template <class CFG>
void 
PRMActiveLearning<CFG>::
GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG >  &outCfgs) {
  LOG_DEBUG_MSG("PRMActiveLearning::GenerateNodes()"); 

  Environment* pEnv = in_pRegion;
  Stat_Class* pStatClass = in_pRegion->GetStatClass();
  CollisionDetection* pCd = this->GetMPProblem()->GetCollisionDetection();
  DistanceMetric* pDm = this->GetMPProblem()->GetDistanceMetric();
  
  GenerateNodes(pEnv, *pStatClass, pCd, pDm, outCfgs);
    
  LOG_DEBUG_MSG("~PRMActiveLearning::GenerateNodes()"); 
};

#endif
