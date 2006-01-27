#ifndef PRMModelBased_h
#define PRMModelBased_h

#include "NodeGeneratorMethod.h"

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class PRMModelBased
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**This generates PRM nodes.  This class is derived off of NodeGenerationMethod.
 */
template <class CFG>
class PRMModelBased: public NodeGenerationMethod<CFG> {
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
  PRMModelBased();
  PRMModelBased(TiXmlNode* in_pNode, MPProblem* in_pProblem);
  ~PRMModelBased();

  virtual char* GetName();
  virtual void SetDefault();

  virtual int GetNextNodeIndex();
  virtual void SetNextNodeIndex(int);
  virtual void IncreaseNextNodeIndex(int);
  virtual void ParseXML(TiXmlNode* in_pNode);

  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  ///Used in new MPProblem framework.
  virtual void PrintOptions(ostream& out_os);
  virtual NodeGenerationMethod<CFG>* CreateCopy();

  /**Model Based Node Generation.
   *This method is based off of "Sampling-Based Motion Planing Using 
   *Predictive Models" by B. Burns and O. Brock in ICRA 2005
   */
  virtual void GenerateNodes(Environment* _env, Stat_Class& Stats,
			     CollisionDetection* cd, 
			     DistanceMetric *dm, vector<CFG>& nodes);
  
  //virtual void GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG >  &outCfgs);

  //Index for next node 
  //used in incremental map generation
  static int nextNodeIndex;

  num_param<int> initial_model_size;
  num_param<int> sample_size;
  num_param<int> reference_size;
  num_param<double> k;

protected:
  vector<double> Normalize(Environment* env, const vector<double>& v) const;
};


template <class CFG>
int PRMModelBased<CFG>::nextNodeIndex = 0;

/////////////////////////////////////////////////////////////////////
//
//  definitions for class PRMModelBased declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG>
PRMModelBased<CFG>::
PRMModelBased() : NodeGenerationMethod<CFG>(),
		  initial_model_size("init", 10, 1, 1000000),
		  sample_size("sample", 10, 1, 1000000),
		  reference_size("reference", 10, 1, 1000000),
		  k("k", 1, 0.0000001, 1000000) {
  initial_model_size.PutDesc("INT","(initial model size, default 10)");
  sample_size.PutDesc("INT","(number of samples to choose from in each round, default 10)");
  reference_size.PutDesc("INT","(number of reference samples in each round for determining min variance sample, default 10)");
  k.PutDesc("FLOAT","(smoothing parameter, default 1)");
}

template <class CFG>
PRMModelBased<CFG>::
~PRMModelBased() {
}


template <class CFG>
PRMModelBased<CFG>::
PRMModelBased(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
NodeGenerationMethod<CFG>(in_pNode, in_pProblem),
initial_model_size("init", 10, 1, 1000000),
sample_size("sample", 10, 1, 1000000),
reference_size("reference", 10, 1, 1000000),
k("k", 1, .0000001, 1000000) {
  LOG_DEBUG_MSG("PRMModelBased::PRMModelBased()");
  initial_model_size.PutDesc("INT","(initial model size, default 10)");
  sample_size.PutDesc("INT","(number of samples to choose from in each round, default 10)");
  reference_size.PutDesc("INT","(number of reference samples in each round for determining min variance sample, default 10)");
  k.PutDesc("FLOAT","(smoothing parameter, default 1)");
  ParseXML(in_pNode);
  LOG_DEBUG_MSG("~PRMModelBased::PRMModelBased()");
}





template <class CFG>
char*
PRMModelBased<CFG>::
GetName() {
  return "PRMModelBased";
}

template <class CFG>
void
PRMModelBased<CFG>::
ParseXML(TiXmlNode* in_pNode) {
  LOG_DEBUG_MSG("PRMModelBased::ParseXML()");
  initial_model_size.PutValue(10);
  sample_size.PutValue(10);
  reference_size.PutValue(10);
  k.PutValue(1);
  if(!in_pNode) {
    LOG_ERROR_MSG("Error reading tags...."); exit(-1);
  }
  if(string(in_pNode->Value()) != "PRMModelBased") {
    LOG_ERROR_MSG("Error reading <PRMModelBased> tag...."); exit(-1);
  }
  int init;
  in_pNode->ToElement()->QueryIntAttribute("init",&init);
  initial_model_size.SetValue(init);
  int sample;
  in_pNode->ToElement()->QueryIntAttribute("sample",&sample);
  sample_size.SetValue(sample);
  int reference;
  in_pNode->ToElement()->QueryIntAttribute("referemce",&reference);
  reference_size.SetValue(reference);
  double _k;
  in_pNode->ToElement()->QueryDoubleAttribute("k",&_k);
  k.SetValue(_k);
  PrintValues(cout);
  LOG_DEBUG_MSG("~PRMModelBased::ParseXML()");
}

template <class CFG>
void
PRMModelBased<CFG>::
SetDefault() {
  NodeGenerationMethod<CFG>::SetDefault();
  initial_model_size.PutValue(10);
  sample_size.PutValue(10);
  reference_size.PutValue(10);
  k.PutValue(1);
}

template <class CFG>
int
PRMModelBased<CFG>::
GetNextNodeIndex() {
  return nextNodeIndex;
}

template <class CFG>
void
PRMModelBased<CFG>::
SetNextNodeIndex(int index) {
  nextNodeIndex = index;
}

template <class CFG>
void
PRMModelBased<CFG>::
IncreaseNextNodeIndex(int numIncrease) {
  nextNodeIndex += numIncrease;
}


template <class CFG>
void
PRMModelBased<CFG>::
ParseCommandLine(int argc, char **argv) {
  int i;
  for (i =1; i < argc; ++i) {
    if( numNodes.AckCmdLine(&i, argc, argv) || 
        chunkSize.AckCmdLine(&i, argc, argv) ||
	exactNodes.AckCmdLine(&i, argc, argv) ||
	initial_model_size.AckCmdLine(&i, argc, argv) ||
	sample_size.AckCmdLine(&i, argc, argv) ||
	reference_size.AckCmdLine(&i, argc, argv) ||
	k.AckCmdLine(&i, argc, argv) ) {
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
PRMModelBased<CFG>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t"; numNodes.PrintUsage(_os); 
  _os << "\n\t"; chunkSize.PrintUsage(_os);
  _os << "\n\t"; exactNodes.PrintUsage(_os); 
  _os << "\n\t"; initial_model_size.PrintUsage(_os); 
  _os << "\n\t"; sample_size.PrintUsage(_os);
  _os << "\n\t"; reference_size.PrintUsage(_os);
  _os << "\n\t"; k.PrintUsage(_os);

  _os.setf(ios::right,ios::adjustfield);
}

template <class CFG>
void
PRMModelBased<CFG>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << numNodes.GetFlag() << " " << numNodes.GetValue() << " ";
  _os << chunkSize.GetFlag() << " " <<chunkSize.GetValue() << " ";
  _os << exactNodes.GetFlag() << " " << exactNodes.GetValue() << " ";
  _os << initial_model_size.GetFlag() << " " << initial_model_size.GetValue() << " ";
  _os << sample_size.GetFlag() << " " << sample_size.GetValue() << " ";
  _os << reference_size.GetFlag() << " " << reference_size.GetValue() << " ";
  _os << k.GetFlag() << " " << k.GetValue() << " ";
  _os << endl;
}


template <class CFG>
NodeGenerationMethod<CFG>* 
PRMModelBased<CFG>::
CreateCopy() {
  NodeGenerationMethod<CFG> * _copy = new PRMModelBased<CFG>(*this);
  return _copy;
}

template <class CFG>
void
PRMModelBased<CFG>::
PrintOptions(ostream& out_os){
  out_os << "    " << GetName() << ":: ";
  out_os << " num nodes = " << numNodes.GetValue() << " ";
  out_os << " exact = " << exactNodes.GetValue() << " ";
  out_os << " chunk size = " << chunkSize.GetValue() << " ";
  out_os << " initial model size = " << initial_model_size.GetValue() << " ";
  out_os << " sample size = " << sample_size.GetValue() << " ";
  out_os << " reference size = " << reference_size.GetValue() << " ";
  out_os << " k = " << k.GetValue() << " ";
  out_os << endl;
}

template <class T>
struct DotProduct : public binary_function<vector<T>,vector<T>,T> {
  T operator()(const vector<T>& x, const vector<T>& y) {
    return inner_product(x.begin(), x.end(), y.begin(), 0.0,
			 plus<T>(), multiplies<T>());
  }
  T operator()(const vector<T>& x) {
    return operator()(x, x);
  }
};

struct GaussianWeighting : public binary_function<vector<double>,
						  vector<double>,
						  double> {
  double k;

  GaussianWeighting(double _k) : k(_k) {}
  ~GaussianWeighting(){}

  double operator()(const vector<double>& x, const vector<double>& y) {
    vector<double> diff(x.size());
    transform(x.begin(), x.end(), y.begin(), diff.begin(),
	      minus<double>());
    DotProduct<double> dot;
    double d = dot(diff, diff);
    return exp(-1*k*d);
  }
};

template <class CFG>
vector<double>
PRMModelBased<CFG>::
Normalize(Environment* env, const vector<double>& v) const {
  vector<double> result = v;
  for(int i=0; i<result.size(); ++i) {
    pair<double,double> range = env->GetBoundingBox()->GetRange(i);
    result[i] = (result[i]-range.first) / (range.second - range.first);
  }
  return result;
}

struct Precompute {
  double a, sum_square, var_x, mean_y, var_y;
  vector<double> mean_x, covar_x_y; 

  Precompute() {}

  Precompute(int size) {
    a = sum_square = var_x = mean_y = var_y = 0;
    mean_x = covar_x_y = vector<double>(size, 0);
  }

  ~Precompute() {}
};

template <class CFG>
void
PRMModelBased<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats,
	      CollisionDetection* cd, DistanceMetric* dm,
	      vector<CFG>& nodes) {
  string callee("PRMModelBased::GenerateNodes");
  LOG_DEBUG_MSG("PRMModelBased::GenerateNodes()");	
#ifndef QUIET
  cout << "(numNodes=" << numNodes.GetValue() << ") ";
  cout << "(chunkSize=" << chunkSize.GetValue() << ") ";
  cout << "(exactNodes=" << exactNodes.GetValue() << ") ";
  cout << "(init=" << initial_model_size.GetValue() << ") ";
  cout << "(sample=" << sample_size.GetValue() << ") ";
  cout << "(reference =" << reference_size.GetValue() << ") ";
  cout << "(k=" << k.GetValue() << ") ";
#endif

  CFG __tmp;
  int dof = __tmp.DOF();

  //generate an initial model with initial_model_size random samples
  vector<pair<vector<double>,double> > model;
  typedef vector<pair<vector<double>,double> >::const_iterator model_iterator;
  for(int i=0; i<initial_model_size.GetValue(); ++i) {
    CFG x;
    x.GetRandomCfg(_env);
    if(x.isCollision(_env,Stats,cd,*cdInfo,true,&callee))
      model.push_back(make_pair(Normalize(_env,x.GetData()),-1));
    else 
      model.push_back(make_pair(Normalize(_env,x.GetData()),1));    
  }

  //generate nodes based on the model information
  GaussianWeighting w(k.GetValue());
  DotProduct<double> dot;
  for(int i=0; i<numNodes.GetValue(); ++i) {
    //generate sample_size samples, select the one that maximizes expected model variance reduction over the reference set

    //generate reference set 
    vector<vector<double> > reference;
    typedef vector<vector<double> >::const_iterator reference_iterator;
    for(int j=0; j<reference_size.GetValue(); ++j) {
      CFG r;
      r.GetRandomCfg(_env);
      reference.push_back(Normalize(_env,r.GetData()));
    }

    vector<Precompute> precompute;
    for(reference_iterator R = reference.begin(); R != reference.end(); ++R) {
      Precompute p(dof);

      for(model_iterator M = model.begin(); M != model.end(); ++M) {
	double w_R_M = w(*R,M->first);
        p.a += w_R_M;
        p.sum_square += pow(w_R_M,2);
	for(int __i = 0; __i < dof; ++__i) 
          p.mean_x[__i] += w_R_M * M->first[__i];
	p.mean_y += w_R_M * M->second;
      }
      transform(p.mean_x.begin(), p.mean_x.end(), p.mean_x.begin(),
		bind2nd(divides<double>(), p.a) );
      p.mean_y /= p.a;
      for(model_iterator M = model.begin(); M != model.end(); ++M) {
	double w_R_M = w(*R,M->first);
	vector<double> M_minus_mean_x(dof);
	transform(M->first.begin(), M->first.end(), p.mean_x.begin(), M_minus_mean_x.begin(),
		  minus<double>());
        p.var_x += w_R_M * dot(M_minus_mean_x);
	for(int __i = 0; __i < dof; ++__i) 
	  p.covar_x_y[__i] += w_R_M * (M->second - p.mean_y) * M_minus_mean_x[__i];
	p.var_y += w_R_M * pow(M->second-p.mean_y,2);
      }
      p.var_x /= p.a;
      transform(p.covar_x_y.begin(), p.covar_x_y.end(), p.covar_x_y.begin(),
		bind2nd(divides<double>(), p.a) );
      p.var_y /= p.a;

      precompute.push_back(p);
    }


    //select sample that minimizes expected variance over reference set
    CFG min_x;
    double min_exp_var;
    for(int j=0; j<sample_size.GetValue(); ++j) {
      //generate random sample
      CFG s;
      s.GetRandomCfg(_env);
      vector<double> new_x = Normalize(_env,s.GetData());

      //compute variance over reference set
      double exp_new_var_pred_s = 0;
      vector<Precompute>::const_iterator P = precompute.begin();
      for(reference_iterator R = reference.begin(); R != reference.end(); 
	  ++R, ++P) {
	//compute variables dependent on the new sample, new_x:
        double b = w(*R,new_x);

        vector<double> new_mean_x(dof);
	for(int __i = 0; __i < dof; ++__i) 
	  new_mean_x[__i] = (P->a*P->mean_x[__i] + b*new_x[__i]) / (P->a+b);

	vector<double> new_x_minus_new_mean_x(dof);
	transform(new_x.begin(), new_x.end(), new_mean_x.begin(), new_x_minus_new_mean_x.begin(),
		  minus<double>());
        double new_var_x = (P->a*P->var_x/(P->a+b)) + (P->a*b*dot(new_x_minus_new_mean_x)/pow(P->a+b,2));

        double sum_large = 0;
        for(model_iterator M = model.begin(); M != model.end(); ++M) {
	  vector<double> M_minus_new_mean_x(dof);
	  transform(M->first.begin(), M->first.end(), new_mean_x.begin(), M_minus_new_mean_x.begin(),
		    minus<double>());
          sum_large += pow(w(*R,M->first),2) * dot(M_minus_new_mean_x)/new_var_x;
	}

	double a_new = 0;
	for(model_iterator M = model.begin(); M != model.end(); ++M) 
	  a_new += w(new_x,M->first); 

	vector<double> mean_new_x(dof, 0);
	for(model_iterator M = model.begin(); M != model.end(); ++M) 
	  for(int __i = 0; __i < dof; ++__i) 
	    mean_new_x[__i] += w(new_x,M->first) * M->first[__i];
	transform(mean_new_x.begin(), mean_new_x.end(), mean_new_x.begin(),
		  bind2nd(divides<double>(), a_new) );

	vector<double> covar_new_x_y(dof,0);
	double var_new_x = 0;
	for(model_iterator M = model.begin(); M != model.end(); ++M) {
	  vector<double> M_minus_mean_new_x(dof);
	  transform(M->first.begin(), M->first.end(), mean_new_x.begin(), M_minus_mean_new_x.begin(),
		    minus<double>());
	  var_new_x += w(new_x,M->first)*dot(M_minus_mean_new_x);
	  for(int __i = 0; __i < dof; ++__i) 
	    covar_new_x_y[__i] += w(new_x,M->first) * (M->second-P->mean_y) * M_minus_mean_new_x[__i];
	}
	transform(covar_new_x_y.begin(), covar_new_x_y.end(), covar_new_x_y.begin(),
		  bind2nd(divides<double>(), a_new) );
	var_new_x /= a_new;

	vector<double> covar_new_x_y_divide_var_new_x(dof);
	transform(covar_new_x_y.begin(), covar_new_x_y.end(), covar_new_x_y_divide_var_new_x.begin(),
		  bind2nd(divides<double>(), var_new_x) );
	vector<double> new_x_minus_mean_new_x(dof);
	transform(new_x.begin(), new_x.end(), mean_new_x.begin(), new_x_minus_mean_new_x.begin(),
		  minus<double>());
	double pred_y = P->mean_y + dot(covar_new_x_y_divide_var_new_x, new_x_minus_mean_new_x);

	double cond_var_y_new_x = P->var_y*dot(covar_new_x_y)/var_new_x;

        double exp_new_var_y = P->a*P->var_y/(P->a+b) + (P->a*b*(cond_var_y_new_x+pow(pred_y-P->mean_y,2)))/pow(P->a+b,2); 

	vector<double> new_x_minus_mean_x(dof);
	transform(new_x.begin(), new_x.end(), P->mean_x.begin(), new_x_minus_mean_x.begin(),
		  minus<double>());
	vector<double> exp_new_covar_x_y(dof);
	for(int __i = 0; __i < dof; ++__i) {
	  double term_1 = P->covar_x_y[__i] * P->a / (P->a + b);
	  double term_2 = P->a * b * (pred_y-P->mean_y) * new_x_minus_mean_x[__i] / pow(P->a+b,2);
	  exp_new_covar_x_y[__i] = term_1 + term_2;
	}

        double exp_new_sqr_covar_x_y = dot(exp_new_covar_x_y) + pow(P->a,2)*pow(b,2)*cond_var_y_new_x*dot(new_x_minus_mean_x)/pow(P->a+b,4);

        double exp_new_cond_var_y_x = exp_new_var_y - exp_new_sqr_covar_x_y/new_var_x;

        vector<double> x_minus_new_mean_x(dof);
        transform(R->begin(), R->end(), new_mean_x.begin(), x_minus_new_mean_x.begin(),
		  minus<double>());
        double exp_new_var_pred_y = (exp_new_cond_var_y_x / pow(P->a+b,2)) *
	  ( P->sum_square + pow(b,2) + dot(x_minus_new_mean_x)/new_var_x + sum_large + b*dot(new_x_minus_new_mean_x)/new_var_x );

        exp_new_var_pred_s += exp_new_var_pred_y;
      }
      if(j==0 || exp_new_var_pred_s > min_exp_var) {
	min_x = s;
	min_exp_var = exp_new_var_pred_s;
      }
    }

    //check collision and add to the roadmap if free
    bool collision_check = min_x.isCollision(_env,Stats,cd,*cdInfo,true,&callee);
    if(!collision_check) //min_x is free
      nodes.push_back(min_x);	
    else 
      if(exactNodes.GetValue()) 
	i--;

    //add min_x to the model M
    if(collision_check) //min_x is collision
      model.push_back(make_pair(min_x.GetData(), -1));
    else
      model.push_back(make_pair(min_x.GetData(), 1));
  }
  
  LOG_DEBUG_MSG("~PRMModelBased::GenerateNodes()");
}


/*
template <class CFG>
void 
PRMModelBased<CFG>::
GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG >  &outCfgs) {
  LOG_DEBUG_MSG("PRMModelBased::GenerateNodes()"); 
  string callee("PRMModelBased::GenerateNodes");
  Environment* pEnv = in_pRegion;
  Stat_Class* pStatClass = in_pRegion->GetStatClass();
  CollisionDetection* pCd = GetMPProblem()->GetCollisionDetection();
  
  LOG_DEBUG_MSG("~PRMModelBased::GenerateNodes()"); 
};
*/

#endif


