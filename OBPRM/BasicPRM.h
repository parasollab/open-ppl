#ifndef BasicPRM_h
#define BasicPRM_h

#include "NodeGenerationMethod.h"

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class BasicPRM
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**This generates PRM nodes.  This class is derived off of NodeGenerationMethod.
 */
template <class CFG>
class BasicPRM: public NodeGenerationMethod<CFG> {
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
  BasicPRM();
  ///Destructor.	
  virtual ~BasicPRM();

  //@}

  //////////////////////
  // Access
  virtual char* GetName();

  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual NodeGenerationMethod<CFG>* CreateCopy();

  /**Basic Randomized (probabilistic) Node Generation.
   *This method generates NodeGenerationMethod::numNodes collision-free Cfgs
   *and insert there Cfgs to nodes.
   *@param _env Used to get free Cfg.
   *@param cd Used to get free Cfg
   *@param nodes Used to store generated nodes.
   *@see See Cfg::GetFreeRandomCfg to know how to generate "one" free Cfg.
   *@note If INTERMEDIATE_FILES is defined WritePathConfigurations will be
   *called.
   */
  virtual void GenerateNodes(Environment* _env, Stat_Class& Stats,
			     CollisionDetection* cd, 
			     DistanceMetric *dm, vector<CFG>& nodes);
};


/////////////////////////////////////////////////////////////////////
//
//  definitions for class BasicPRM declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG>
BasicPRM<CFG>::
BasicPRM() : NodeGenerationMethod<CFG>() {
}


template <class CFG>
BasicPRM<CFG>::
~BasicPRM() {
}


template <class CFG>
char*
BasicPRM<CFG>::
GetName() {
  return "BasicPRM";
}


template <class CFG>
void
BasicPRM<CFG>::
ParseCommandLine(int argc, char **argv) {
  int i;
  for (i =1; i < argc; ++i) {
    if( numNodes.AckCmdLine(&i, argc, argv) || 
	exactNodes.AckCmdLine(&i, argc, argv)) {
/*         numAttempts.AckCmdLine(&i, argc, argv) || */
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
BasicPRM<CFG>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t"; numNodes.PrintUsage(_os); _os << " ";
/*   _os << "\n\t"; numAttempts.PrintUsage(_os); */
  _os << "\n\t"; exactNodes.PrintUsage(_os);


  _os.setf(ios::right,ios::adjustfield);
}

template <class CFG>
void
BasicPRM<CFG>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << numNodes.GetFlag() << " " << numNodes.GetValue() << " ";
/*   _os << numAttempts.GetFlag() << " " << numAttempts.GetValue(); */
  _os << exactNodes.GetFlag() << " " << exactNodes.GetValue();
  _os << endl;
}


template <class CFG>
NodeGenerationMethod<CFG>* 
BasicPRM<CFG>::
CreateCopy() {
  NodeGenerationMethod<CFG> * _copy = new BasicPRM<CFG>(*this);
  return _copy;
}


template <class CFG>
void
BasicPRM<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats,
	      CollisionDetection* cd, DistanceMetric *,
	      vector<CFG>& nodes) {
	
#ifndef QUIET
  if (exactNodes.GetValue()==1)
     cout << "(numNodes=" << numNodes.GetValue() << ") ";
  else
    cout << "(exactNodes=" << exactNodes.GetValue() << ") ";
#endif
  
  //PRM style node generation -- generate in expanded bounding box
  vector<Cfg*> path;
  
  if (exactNodes.GetValue() == 1) { // we want to obtain numNodes free nodes 
    CFG tmp;
    int default_maxTries = 100;
    for (int i=0; i < numNodes.GetValue(); ++i) {
       bool collision = true;
       int j=0;
       while (collision && (j<default_maxTries)) {
         j++;
         Stats.IncNodes_Attempted();
         tmp.GetRandomCfg(_env);
         if (!tmp.isCollision(_env, Stats, cd, *cdInfo)) {
	   nodes.push_back( CFG(tmp));
	   path.push_back ( (Cfg*)tmp.CreateNewCfg() );
	   Stats.IncNodes_Generated();
           collision = false;
         }
       }
       if (j == default_maxTries)
	 cerr << "Can't generate enought nodes! " << endl;
    }
  } else { //we want to try numNodess attempts (either free or in collision)
    CFG tmp;
    for (int i=0; i < numNodes.GetValue(); ++i) {
      Stats.IncNodes_Attempted();
      tmp.GetRandomCfg(_env);
      if (!tmp.isCollision(_env, Stats, cd, *cdInfo)) {
	nodes.push_back( CFG(tmp));
	path.push_back ( (Cfg*)tmp.CreateNewCfg() );
	Stats.IncNodes_Generated();
      }
    }
  }
  
#if INTERMEDIATE_FILES
  //in util.h
  WritePathConfigurations("prm.path", path, _env);
#endif	
  int i;
  for(i=0; i<path.size(); i++)
    if (path[i]!=NULL)
      delete path[i];
};

#endif


