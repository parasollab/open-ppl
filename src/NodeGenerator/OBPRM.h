#ifndef OBPRM_h
#define OBPRM_h

#include "BasicOBPRM.h"
#include "Parameters.h"

#define MAX_NUM_NODES_TRIES 100

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class OBPRM
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**This performs obstacle-based node generation.  This class is derived
 *off of BasicOBPRM.
 */
template <class CFG>
class OBPRM : public BasicOBPRM<CFG> {
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
  OBPRM();
  OBPRM(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  ///Destructor.
  virtual ~OBPRM();
  virtual void ParseXML(XMLNodeReader& in_Node);
  virtual void ParseXMLcol_pair(XMLNodeReader& in_Node);
  virtual void ParseXMLfree_pair(XMLNodeReader& in_Node);
  //@}

  //////////////////////
  // Access
  virtual char* GetName();
  virtual void SetDefault();
  virtual int GetNextNodeIndex();
  virtual void SetNextNodeIndex(int);
  virtual void IncreaseNextNodeIndex(int);


  //////////////////////
  // I/O methods
  virtual void PrintOptions(ostream& _os);
  virtual NodeGenerationMethod<CFG>* CreateCopy();

  /** Obstacle Based Node Generation with some nifty enhancements such
   * shells, alternative seed picking heuristics, and some free nodes.
   * 
   * Algorithm:
   *   -# For each Obstacle in Environement.
   *       -# Generate Surface Cfgs using Binary Search Procedure
   *          (using GenSurfaceCfgs4Obst)
   *       -# Generate Free Cfgs using Ad Hoc procedure
   *          (using GenFreeCfgs4Obst)
   *       -# Collect Free Cfgs found in 1 and 2.
   *   -# End For
   *
   * when there is no Obstacle (i.e. only object in the Enviroment is 
   * Robot) GenCfgsFromCObst will be called instead of alg above.
   * @note number of free Cfgs genertated in algorithm line 2
   * will be (P * numNodes/(number_of_obstacle)/numShells)
   * and number of free Cfgs genertated in algorithm line 3 will
   * be ((1-P) * numNodes/(number_of_obstacle))
   * Here P is proportionSurface.
   *
   * @param _env Environment for getting geometric information.
   * @param cd Used to get free Cfg (checking collision).
   * @param nodes Used to store generated nodes.
   *
   * @bug if number_of_obstacle is zero, above "equaltion" will cause
   * "divided by zero" run time error?!
   */ 
  virtual void GenerateNodes(Environment* _env, Stat_Class& Stats,
			     DistanceMetric *, vector<CFG>& nodes);
          
  virtual void GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion,vector<CFG>& nodes);

  /**Generate Free Cfgs near the surface of Obstacle.
   *Get CollPair infomation by calling ValidatePairs for info.collPair.
   *   -# if Both element of CollPair is N_rT.
   *       -# then Cfg::GenSurfaceCfgs4ObstNORMAL will be called.
   *   -# othewise, GenSurfaceCfgs4ObstVERTEX will be called.
   *@return Return values returned by the function called by this method.
   */
  vector<CFG> GenSurfaceCfgs4Obst(Environment* env, Stat_Class& Stats,
				  DistanceMetric* dm, int obstacle,
				  int nCfgs, double clearanceFactor = 1.0);

  /**Generate Free Cfgs near the surface of Obstacle by overlapping
   *point on the robot and point on the obstacle.
   *
   *This method uses different method to generate Cfg in C-Obs
   *instead of using GenerateInsideCfg. See GenerateSeeds for
   *more infomation about how to ues another way to generate Cfgs
   *inside CObstacle.
   *
   *After getting nCfgs Cfgs in side C-Obs, like BasicOBPRM,
   *Free Cfgs in around a obstacle are generated by calling 
   *GenerateSurfaceCfg and Shells.
   *@param obstacle index for obstacle in Environment.
   *@param nCfgs number of Free Cfgs near the surface
   *       will be generated.
   *@param clearanceFactor used to calculate cleanance between Cfg and Obstacle.
   *
   *@return a list of Free Cfgs near the surface of Obstacle.
   *
   *@see GenerateSurfaceCfg for binary seacrh of Cfgs, and GenerateSeeds
   *for generating Cfgs in C-Obstacle.
   */
  vector<CFG> GenSurfaceCfgs4ObstVERTEX(Environment* env, Stat_Class& Stats,
					DistanceMetric* dm, int obstacle, 
					int nCfgs, double clearanceFactor = 1.0);

 protected:
  /**validate values specified for collision *or* free pairs
   *Call TranslateOptionCode to translate user input from string
   *to PairOptions value. 
   *@param params Contains two string values which will be converted
   *to PairOptions values.
   *@param results A pair of PairOptions values converted from 
   * two string values in params.
   *@param msg used as error message.
   *@return false if one of two strings are converted to INVALID_OPTION.
   *Otherwise true will be returned.
   *
   *@see TranslateOptionCode
   */
  static bool ValidatePairs(char* msg, n_str_param params, 
			    pair<int,int>* results);

  /**Character specification of PairOptions must be converted to enum.
   *For example string "cM" convert to cM. string "all" convert to all.
   *
   *@return INVALID_OPTION in no coorsponding PairOptions value in found.
   *Otherwise one of PairOptions values will be returned.
   *
   *@note Error message will be post if this method does not know how
   *to convert.
   *
   *@param mnemonic The string which is going to be convert to PairOptions value.
   *@param PairOptions used for print out error message.
   *
   *@warning Return value of this method actually is a interger number, which
   *coorsponds to the order in PairOptions. If the order of PairOptions changed
   *The returned value of this method should be changed, too.
   */
  static int TranslateOptionCode(char* mnemonic, n_str_param param);

  /**Generate Free Cfgs in specified (user or default) manner.
   *Another way to generate Cfgs around C-Obstacle.
   *Follow algorithm is applied to get one such Cfg:
   *   -# Generate a point, r, on robot.
   *   -# Generate a point, o, on obstacle.
   *   -# Move robot such that r and o are overlap.
   *   -# Rotate robot randomly.
   *   -# if this Cfg is collision free, then insert
   *      this Cfg to return list.
   * "nCfgs" Cfg(s) will be generated
   *
   *@param obstacle index for obstacle MultiBody in Envoriment.
   *@param nCfgs number of free Cfgs that will be generated.
   *@param _info this method uses _info.freePair information
   *to generate points in robot and obstacle. 
   *_info.freePair.first is for robot and _info.freePair.second
   *is for obstacle.
   *
   *
   *@see PointsOnMultiBody to get point on the MultiBody.
   *Cfg::GenerateOverlapCfg to get overlap Cfg.
   *@see GenerateSeeds for generating Cfg in C-Obs in
   *similar mannar.
   */
  vector<CFG> GenFreeCfgs4Obst(Environment* env, Stat_Class& Stats,
			       int obstacle, int nCfgs);

  /**Generate seeds in specified (user or default) manner.
   *Another way to generate Cfgs in C-Obstacle.
   *Follow algorithm is applied to get one such Cfg:
   *   -# Generate a point, r, on robot.
   *   -# Generate a point, o, on obstacle.
   *   -# Move robot such that r and o are overlap.
   *   -# Rotate robot randomly.
   *   -# if this Cfg causes collision, then insert
   *      this Cfg to seeds
   *nseeds Cfg(s) will be generated
   *
   *@param obst index for obstacle MultiBody in Envoriment.
   *@param nseeds number of seed that will be generated.
   *@param selectRobot PairOptions
   *@param selectObstacle PairOptions
   *@param seeds generated Cfgs
   *
   *@see PointsOnMultiBody to get point on the MultiBody.
   *Cfg::GenerateOverlapCfg to get overlap Cfg.
   */
  void GenerateSeeds(Environment* env, Stat_Class& Stats, 
		     int obst, int nseeds,
		     int selectRobot, int selectObstacle,
		     vector<CFG>* seeds);

  /**Generate several points on the given MultiBody.
   *@param mbody a pointer to MultiBody from where 
   *       points will be generated.
   *@param npts How many points are going to be generated.
   *@param select which method are going to be uesd.
   *
   *@note for robot(which have freebodys),
   *sample points on last link only. for obstacle, 
   *sample first link.
   *
   *@return a list of generated points on the mbody.
   *@see PointOnBody
   */
  static vector<Vector3D> PointsOnMultiBody(MultiBody* mbody, int npts, int select);

  /// generates random cfgs within a specified area around a given cfg
  static void Spread(CFG&, double, double, int, vector<CFG>*);

 public:
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  /**Used in OBPRM to generate Cfgs in C-Obstacle.
   *This pair contains values which tell node generation
   *methods what strategies will be used to find point in
   *Robot and point in Obstacle. The first element in this 
   *pair is for Robor and the second one is for Obstacle.
   *Each element represents a way to find point.
   *@see PairOptions for all possible ways and 
   *GenerateMapNodes::GenerateSeeds for generating 
   *Cfgs in C-Obstacles.
   */
  n_str_param       collPair;

  /**Used in OBPRM to generate collision free Cfgs.
   *This pair contains values which tell node generation
   *methods what strategies will be used to find point in
   *Robot and point in Obstacle. The first element in this 
   *pair is for Robor and the second one is for Obstacle.
   *Each element represents a way to find point.
   *@see PairOptions for all possible ways and
   *GenerateMapNodes::GenFreeCfgs4Obst for generating free Cfgs.
   */
  n_str_param       freePair;

  /**Proportion of generated Cfgs are on Surface.
   *This varible decides how many Cfgs will be 
   *created from GenSurfaceCfgs4Obst and how many 
   *Cfgs will be generated from GenFreeCfgs4Obst.
   *
   *@note this vaule should between [0,1]
   *@see GenerateMapNodes::GenSurfaceCfgs4Obst,
   *GenerateMapNodes::GenFreeCfgs4Obst and 
   *GenerateMapNodes:OBPRM.
   */
  double proportionSurface;

  /**Clearance is clearanceFactor*Position_Resoultion.
   *Tell Node genration functions to generate free 
   *Cfgs which have certain clearance.
   *@see GenerateMapNodes::GenerateSurfaceCfg
   */
  double clearanceFactor; 
  
  //Index for next node 
  //used in incremental map generation
  static int nextNodeIndex;

};


template <class CFG>
int OBPRM<CFG>::nextNodeIndex = 0;

/////////////////////////////////////////////////////////////////////
//
//  definitions for class OBPRM declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG>
OBPRM<CFG>::
OBPRM() : BasicOBPRM<CFG>(),     
  collPair         ("collPair","cM rT "),
  freePair         ("freePair","cM rV ") {

  collPair.PutDesc("STRING STRING",
			    "\n\t\t\tSpecify 2 of the following recognized mnemonics:"
			    "\n\t\t\t  cM    \"center of mass\""
			    "\n\t\t\t  rV    \"random vertex\""
			    "\n\t\t\t  rT    \"point in random triangle\""
			    "\n\t\t\t  rE    \"random extreme vertex\""
			    "\n\t\t\t  rW    \"point in random weighted triangle\""
			    "\n\t\t\t  cM_rV \"cg/random vertex\""
			    "\n\t\t\t  rV_rT \"random vertex/point in random          triangle\""
			    "\n\t\t\t  rV_rW \"random vertex/point in random weighted triangle\""
			    "\n\t\t\t  N_rT  \"normal of random triangle\""
			    "\n\t\t\t  all   \"all of the above\""
			    );
  collPair.PutNumStrings(2);
 
  freePair.PutDesc("STRING STRING","\n\t\t\tSame as above");
  freePair.PutNumStrings(2);

}


///\todo xmlize proportionSurface and clearanceFactor
template <class CFG>
OBPRM<CFG>::
OBPRM(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
  BasicOBPRM<CFG>(in_Node, in_pProblem),     
  collPair         ("collPair","cM rT "),
  freePair         ("freePair","cM rV ") {
  
  LOG_DEBUG_MSG("OBPRM::OBPRM()");
  collPair.PutDesc("STRING STRING",
                   "\n\t\t\tSpecify 2 of the following recognized mnemonics:"
                   "\n\t\t\t  cM    \"center of mass\""
                   "\n\t\t\t  rV    \"random vertex\""
                   "\n\t\t\t  rT    \"point in random triangle\""
                   "\n\t\t\t  rE    \"random extreme vertex\""
                   "\n\t\t\t  rW    \"point in random weighted triangle\""
                   "\n\t\t\t  cM_rV \"cg/random vertex\""
                   "\n\t\t\t  rV_rT \"random vertex/point in random          triangle\""
                   "\n\t\t\t  rV_rW \"random vertex/point in random weighted triangle\""
                   "\n\t\t\t  N_rT  \"normal of random triangle\""
                   "\n\t\t\t  all   \"all of the above\"");
  collPair.PutNumStrings(2); 
  freePair.PutDesc("STRING STRING","\n\t\t\tSame as above");
  freePair.PutNumStrings(2);
  ///\todo organize setting default better
  collPair.PutValue("cM rT ");
  freePair.PutValue("cM rV "); 
  proportionSurface = 1.0;
  clearanceFactor = 1.0;
  ParseXML(in_Node);
  LOG_DEBUG_MSG("~OBPRM::OBPRM()");
}

template <class CFG>
void OBPRM<CFG>::
ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("OBPRM::ParseXML()");
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "coll_pair") {
      ParseXMLcol_pair(*citr);
    } else if(citr->getName() == "free_pair") {
      ParseXMLfree_pair(*citr);
    }
  }
  LOG_DEBUG_MSG("~OBPRM::ParseXML()");
}

template <class CFG>
void OBPRM<CFG>::
ParseXMLcol_pair(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("OBPRM::ParseXMLcol_pair()");
  
  string str_a_pair = in_Node.stringXMLParameter(string("a"),true,string(""),string("col_pair_a"));
  string str_b_pair = in_Node.stringXMLParameter(string("b"),true,string(""),string("col_pair_b"));
 
  string final_col_pair = str_a_pair + string(" ") + str_b_pair + string(" ");
  char str_char[100];
  strcpy(str_char, final_col_pair.c_str());

  collPair.PutValue(str_char);
  LOG_DEBUG_MSG("~OBPRM::ParseXMLcol_pair()");
}
    
template <class CFG>
void OBPRM<CFG>::
ParseXMLfree_pair(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("OBPRM::ParseXMLcol_pair()");

  string str_a_pair = in_Node.stringXMLParameter(string("a"),true,string(""),string("col_pair_a"));
  string str_b_pair = in_Node.stringXMLParameter(string("b"),true,string(""),string("col_pair_b"));

  string final_free_pair = str_a_pair + string(" ") + str_b_pair + string(" ");
  char str_char[100];
  strcpy(str_char, final_free_pair.c_str());
  freePair.PutValue(str_char);
  LOG_DEBUG_MSG("~OBPRM::ParseXMLcol_pair()");
}


template <class CFG>
OBPRM<CFG>::
~OBPRM() {
}


template <class CFG>
char*
OBPRM<CFG>::
GetName() {
  return "OBPRM";
}


template <class CFG>
void
OBPRM<CFG>::
SetDefault() {
  BasicOBPRM<CFG>::SetDefault();
  collPair.PutValue("cM rT ");
  freePair.PutValue("cM rV "); 
  proportionSurface = 1.0;
  clearanceFactor = 1.0;
}

template <class CFG>
int
OBPRM<CFG>::
GetNextNodeIndex() {
  return nextNodeIndex;
}

template <class CFG>
void
OBPRM<CFG>::
SetNextNodeIndex(int index) {
  nextNodeIndex = index;
}

template <class CFG>
void
OBPRM<CFG>::
IncreaseNextNodeIndex(int numIncrease) {
  nextNodeIndex += numIncrease;
}









template <class CFG>
    void
OBPRM<CFG>::
    PrintOptions(ostream& _os){
  _os << "    " << GetName() << ":: ";
  _os << " num nodes = " << this->numNodes << " ";
  _os << " exact  = " << this->exactNodes << " ";
  _os << " chunk size = " << this->chunkSize << " ";
  _os << " num shells = " << this->numShells << " ";
  _os << " validity method = " << this->vcMethod << " ";  
  _os << endl << "                    ";
  _os << "  proportionSurface = " << proportionSurface << " ";
  _os << "  collPair = " << collPair.GetValue() << " ";
  _os << "  freePair = " << freePair.GetValue() << " ";
  _os << "  clearanceFactor = " << clearanceFactor << " ";
  _os << endl;
}


template <class CFG>
NodeGenerationMethod<CFG>* 
OBPRM<CFG>::
CreateCopy() {
  NodeGenerationMethod<CFG> * _copy = new OBPRM<CFG>(*this);
  return _copy;
}


template <class CFG>
void 
OBPRM<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats, 
	      DistanceMetric *dm, vector<CFG>& nodes) {
}
         
         
template <class CFG>
void 
OBPRM<CFG>::
GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector<CFG>& nodes) {
  Environment* _env = in_pRegion;
  Stat_Class& Stats = *(in_pRegion->GetStatClass());
//  CollisionDetection* cd = this->GetMPProblem()->GetCollisionDetection();
  DistanceMetric* dm =  this->GetMPProblem()->GetDistanceMetric();
         
  int origNumNodes = this->numNodes;        
         
         
#ifndef QUIET
  cout << "(numNodes="          << this->numNodes      << ", ";
  cout << "(exactNodes="        << this->exactNodes    << ", ";
  cout << "(chunkSize="         << this->chunkSize     << ", ";
  cout << "validity method = "  << this->vcMethod      << ", ";
  cout << "\tproportionSurface="<< proportionSurface   << ", ";
  cout << "\nnumShells="        << this->numShells     << ", ";
  cout << "collPair="           << collPair.GetValue() << ", ";
  cout << "freePair="           << freePair.GetValue() << ", ";
  cout << "clearanceFactor="    << clearanceFactor     << ") ";
#endif
  
  bool bExact = this->exactNodes == 1? true: false;

/*   if  (this->exactNodes == 1){ */
/*     cerr << "\nFunction to generate exact numbers for OBPRM not implemented yet." << endl; */
/*   }  */

  pair<int,int> seedSelect,freeSelect;
	
  ValidatePairs("seedSelect", collPair, &seedSelect);
  ValidatePairs("freeSelect", freePair, &freeSelect);
  
  vector<CFG> tmp, preshells, shells;
  
  int numMultiBody = _env->GetMultiBodyCount();
  int robot        = _env->GetRobotIndex();
  int numExternalBody = _env->GetExternalBodyCount();
  
  double P = proportionSurface;
  
  // Subtract # of robots (ie, numMultiBody-1)
  // int NSEED = (int)(P * this->numNodes/(numMultiBody-1)/this->numShells);
  // int NFREE = (int)((1.0-P) * this->numNodes/(numMultiBody-1));
  
  // Subtract # of robots (ie, numExternalBody-1)
  

  int NSEED = (int)(P * this->numNodes/(numExternalBody-1)/this->numShells);
  int NFREE = (int)((1.0-P) * this->numNodes/(numExternalBody-1));
  
  if (NSEED < 1) NSEED = 1;
  int nNodesGap = this->numNodes - nodes.size();  
  int nNumTries = 0;
 
  

  vector<CFG> surface;
  do{
    vector<CFG> obstSurface, obstFree, nodesBuffer;  
    surface.clear();
    for(int obstacle = 0 ; obstacle < numExternalBody ; obstacle++) {
    
      if(obstacle != robot) {
      
	// Generate Surface Cfgs using Binary Search Procedure
	obstSurface = GenSurfaceCfgs4Obst(_env, Stats,dm, obstacle, NSEED,
					  clearanceFactor);
      
	// Generate Free Cfgs using Ad Hoc procedure
	obstFree = GenFreeCfgs4Obst(_env, Stats, obstacle, NFREE);
      
	// Collect free & surface nodes for return
	int i;
	for (i = 0; i < obstSurface.size(); i++) {
	  obstSurface[i].obst = obstacle;
 	  nodesBuffer.push_back(obstSurface[i]); 
	}
	for (i = 0; i < obstFree.size(); i++) {
	  obstFree[i].obst = obstacle;
 	  nodesBuffer.push_back(obstFree[i]); 
	}

#if INTERMEDIATE_FILES
	surface.insert(surface.end(),
		       obstSurface.begin(),obstSurface.end());
	surface.insert(surface.end(),
		       obstFree.begin(),obstFree.end());
#endif
      
 	obstSurface.erase(obstSurface.begin(),obstSurface.end()); 
 	obstFree.erase(obstFree.begin(), obstFree.end()); 
      
      } // if(obstacle != robot)
      else 
	//if(numMultiBody == 1) { //if robot is the only object
	if(numExternalBody == 1) { //if robot is the only object
	  vector<CFG> CobstNodes = this->GenCfgsFromCObst(_env, Stats, dm, obstacle,
						    this->numNodes);
	  for(int i=0; i<CobstNodes.size(); ++i){
	    CobstNodes[i].obst = obstacle;
 	    nodesBuffer.push_back(CobstNodes[i]); 
	  }
#if INTERMEDIATE_FILES
	  surface.insert(surface.end(),CobstNodes.begin(), CobstNodes.end());
#endif
	}
    
    } // for(obstacle)

    if (bExact){ //exact nodes
      int nActualNodes = nodesBuffer.size();
      if ( nActualNodes < nNodesGap){
	nodes.insert(nodes.end(), nodesBuffer.begin(), nodesBuffer.end()); 	
	nNodesGap = nNodesGap - nActualNodes;
        ///\todo figure out this logic
        this->numNodes = nNodesGap;/////////why?
 	nodesBuffer.erase(nodesBuffer.begin(), nodesBuffer.end()); 
	
	NSEED = (int)(P * nNodesGap/(numExternalBody-1)/this->numShells);
	NFREE = (int)((1.0-P) * nNodesGap/(numExternalBody-1));
	if (NSEED < 1) NSEED = 1;
      }else if (nActualNodes == nNodesGap){//Generated exact number of nodes
	nodes.insert(nodes.end(), nodesBuffer.begin(), nodesBuffer.end()); 	
	nodesBuffer.erase(nodesBuffer.begin(),nodesBuffer.end());
	nNodesGap = 0;
	NSEED = 0;	
      }else{ // nActualNodes > nNodesGap ;
	//+++++++++++Need to sample here	
	vector <bool> indices(nActualNodes);
	int iSelect;
	for (int j = 0; j < nNodesGap; j++) //randomly sample #nNodesGap 
	  {
	    do { iSelect = OBPRM_lrand() % nActualNodes; } while (indices[iSelect] == true);
	    indices[iSelect] = true;
	  }

	for (int j = 0; j < nActualNodes; j++) //push those selected one in to nodes
	  if (indices[j])
	    nodes.push_back(nodesBuffer[j]);

	nodesBuffer.erase(nodesBuffer.begin(),nodesBuffer.end());
	nNodesGap = 0;
	NSEED = 0;
      }
    }else { // not asked for exact nodes
      nodes.insert(nodes.end(), nodesBuffer.begin(), nodesBuffer.end()); 
      nodesBuffer.erase(nodesBuffer.begin(),nodesBuffer.end());
      nNodesGap=0;
/*       NSEED = 0; */
    }
    
    nNumTries++; // make sure the loop will end when no enough nodes can be generated;
  }while(nNodesGap>0 && nNumTries < MAX_NUM_NODES_TRIES) ;

  if (nNumTries >= MAX_NUM_NODES_TRIES)
    cerr << GetName() << ": Tried "<< nNumTries << " times, can\'t generate engough nodes! " << endl;

#if INTERMEDIATE_FILES
  WritePathConfigurations("surface.path", surface, _env);
#endif
  ///reset numNodes back to origional value
  this->numNodes = origNumNodes;
}


template <class CFG>
bool
OBPRM<CFG>::
ValidatePairs(char* msg, n_str_param params, pair<int,int>* results) {  
  int k;
  char *p;
  bool success = false;
  
  //-- separate out the 2 fields
  
  //-- field 1
  
  char str1[20]; strcpy(str1,params.GetValue());
  for (k=0,p=str1; *p != ' ' && p<str1+strlen(str1);++p,++k)
    str1[k]=*p;
  str1[k]='\0';
  
  //-- field 2
  
  char str2[20]; strcpy(str2,params.GetValue());
  for (k=0,p=strchr(str2,' ')+1; *p != ' ' && p<str2+strlen(str2);++p,++k)
    str2[k]=*p;
  str2[k]='\0';
  
  //-- Valid field values will translate to integer options codes
  
  int code1 = TranslateOptionCode(str1,params);
  int code2 = TranslateOptionCode(str2,params);
  if (code1!=BasicOBPRM<CFG>::INVALID_OPTION && code2!=BasicOBPRM<CFG>::INVALID_OPTION){
    //-- If storage was provided, store field values
    if (results){
      results->first = code1;
      results->second = code2;
    }
    
    //-- Indicate success
    success = true;
    return true;
  }
  
  //-- Indicate failure
  if (!success){
    cout <<"\nERROR: Bad " << msg <<"pair values\n\n";
    exit(-1);
  }
  return false;
}


template <class CFG>
int
OBPRM<CFG>::
TranslateOptionCode(char* mnemonic, n_str_param param) {  
  //-- Implemented options are:
  
  vector<str_param <char*> >MM; MM.reserve(8);
  MM.push_back(str_param<char*>("cM","(for \"center of mass\")") );
  MM.push_back(str_param<char*>("rV","(for \"random vertex\")") );
  MM.push_back(str_param<char*>("rT","(for \"point in random triangle\")") );
  MM.push_back(str_param<char*>("rE","(for \"random extreme vertex\")") );
  MM.push_back(str_param<char*>("rW","(for \"point in random weighted triangle\")") );
  MM.push_back(str_param<char*>("cM_rV","(for \"cg/random vertex\")") );
  MM.push_back(str_param<char*>("rV_rT","(for \"random vertex/point in random triangle\")") );
  MM.push_back(str_param<char*>("rV_rW","(for \"random vertex/point in random weighted triangle\")") );
  MM.push_back(str_param<char*>("N_rT","(for \"normal of a random triangle\")") );
  MM.push_back(str_param<char*>("all","(for \"all of the basics\")") );
  
  //-- NOT YET Implemented options are:
  
  vector<str_param<char*> >NI; NI.reserve(1);
  NI.push_back(str_param<char*>("other","(for \"other \")") );
  
  const char *DISCLAIMER = "\n  ** Code for mnemonic not yet implemented **";
  
  int i;
  
  if (strlen(mnemonic)>0) {
    
    //-- Exhaustive search of tbl for field values
    for (i=0;i<MM.size();++i)
      if (!strcmp(MM[i].GetFlag(),mnemonic)) return i;
    
    //-- Exhaustive search of "not implemented" tbl for field values
    for (i=0;i<NI.size();++i){
      if (!strcmp(NI[i].GetFlag(),mnemonic)) {
	cout << "\n\nSORRY, mnemonic \""
	     << mnemonic << "\" is not yet implemented ";
	cout << "\n  Implemented mnemonics are:";
	for (i=0;i<MM.size();++i)
	  cout << "\n\t" <<setw(5)<< MM[i].GetFlag() << "     " <<MM[i].GetValue();
	// return MM.size()+i;
	return BasicOBPRM<CFG>::INVALID_OPTION;
      }
    }
  }
  
  //-- if value is not recognized, let'em know it
  
  cout << "\n\nERROR: invalid mnemonic specified \""
       << param.GetFlag() << " " << param.GetValue() << "\"\n";
  cout << "\n  You must specify 2 of the following recognized mnemonics:";
  for (i=0;i<MM.size();++i)
    cout << "\n\t" <<setw(5)<< MM[i].GetFlag() << "     " <<MM[i].GetValue();
  for (i=0;i<NI.size();++i)
    cout << "\n\t" <<setw(5)<< NI[i].GetFlag() << "   **" <<NI[i].GetValue();
  cout << DISCLAIMER;
  cout << "\n\n  If you specify the flag option as \""<<
    param.GetFlag() << " cM rV\","
    "\n\tcM will be used as the option for the robot "
    "\n\tand rV will be used as the option for the obstacle.\n\n";
  return BasicOBPRM<CFG>::INVALID_OPTION;  
};


template <class CFG>
vector<CFG>
OBPRM<CFG>::
GenSurfaceCfgs4Obst(Environment* env, Stat_Class& Stats,
		    DistanceMetric* dm, 
		    int obstacle, int nCfgs, double clearanceFactor) {
  CollisionDetection* cd = this->GetMPProblem()->GetCollisionDetection(); 

  pair<int,int> seedSelect;
  ValidatePairs("seedSelect", collPair, &seedSelect);
  
  if(seedSelect.first == this->N_rT && seedSelect.second == this->N_rT) {
    CFG cfg;
    vector<Cfg*> pResult;
    cfg.GenSurfaceCfgs4ObstNORMAL(env, Stats, cd, obstacle, nCfgs, 
				  *this->cdInfo, pResult);
    vector<CFG> result;
    int i;
    for(i = 0; i < pResult.size(); i++)
      result.push_back((CFG)*pResult[i]);

    for(i = 0; i < pResult.size(); i++)
      delete pResult[i];

    return result;

  } else
    return GenSurfaceCfgs4ObstVERTEX(env, Stats, dm, obstacle, 
				     nCfgs, clearanceFactor);
}


template <class CFG>
vector<CFG>
OBPRM<CFG>::
GenSurfaceCfgs4ObstVERTEX(Environment* env, Stat_Class& Stats, 
			  DistanceMetric* dm, 
			  int obstacle, int nCfgs, double clearanceFactor) {  
  CollisionDetection* cd = this->GetMPProblem()->GetCollisionDetection(); 

  pair<int,int> seedSelect;
  ValidatePairs("seedSelect", collPair, &seedSelect);
  
  vector<CFG> obstSeeds;
  
  GenerateSeeds(env, Stats, obstacle, nCfgs,
		seedSelect.first, seedSelect.second, &obstSeeds);
  
  int robot = env->GetRobotIndex();
  vector<CFG> tmp, preshells, shells, surface;
  for(int i = 0 ; i < obstSeeds.size() ; i++) {
    //Generate Random direction
    CFG incrCfg = GenerateRandomDirection(env,dm,obstSeeds[i]);     
    // Generate outside cfg
    CFG OutsideNode = GenerateOutsideCfg(env,Stats,robot,obstacle,
					 obstSeeds[i],incrCfg);
    //move inside node to the bounding box if required
    bool inBB = PushCfgToBoundingBox(env,obstSeeds[i],OutsideNode);
    if (inBB) {
      if (OutsideNode.AlmostEqual(obstSeeds[i]) ||
	  !obstSeeds[i].isCollision(env,Stats,cd,robot,obstacle,*this->cdInfo))
	continue; //no valid outside or inside node was found
    }
    else
      continue; //no valid inside node was found
    
    // Generate surface cfgs
    tmp = GenerateSurfaceCfg(env,Stats,dm,
			     robot,obstacle,obstSeeds[i],OutsideNode,
			     clearanceFactor);
    
    // Choose as many as nshells
    preshells = Shells(tmp, this->numShells);
    shells = InsideBoundingBox(env, preshells);
    preshells.erase(preshells.begin(), preshells.end());
    
    // Collect the cfgs for this obstacle
    surface.insert(surface.end(),shells.begin(),shells.end());
  }
  return surface;
};


template <class CFG>
vector<CFG>
OBPRM<CFG>::
GenFreeCfgs4Obst(Environment* env, Stat_Class& Stats, 
		 int obstacle, int nCfgs) { 
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();         
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(this->vcMethod);

  pair<int,int> freeSelect;
  ValidatePairs("freeSelect", freePair, &freeSelect);
  
  int robot = env->GetRobotIndex();
  vector<Vector3D> ptsRobot, ptsObstacle;
  
  ptsRobot = PointsOnMultiBody(env->GetMultiBody(robot), nCfgs,
			       freeSelect.first);
  ptsObstacle = PointsOnMultiBody(env->GetMultiBody(obstacle), nCfgs,
				  freeSelect.second);
  
#if INTERMEDIATE_FILES
  vector<CFG> att; 
  att.reserve(nCfgs);
#endif
  
  std::string Callee(GetName());
  {std::string Method("-OBPRM::GenFreeCfgs2Obst");Callee=Callee+Method;}

  CFG cfg;
  vector<CFG> free; 
  free.reserve(nCfgs);
  for(int i = 0 ; i < nCfgs ; i++){
    if( cfg.GenerateOverlapCfg(env, robot, ptsRobot[i], ptsObstacle[i], &cfg) ) {
      // check if it is possible to generate a Cfg with this pose.
      //if(!cfg.isCollision(env, *this->cdInfo) && CfgInsideBB(env,cfg)) {
      if( // !cfg.isCollision(env, Stats, cd, *this->cdInfo,true, &Callee)
	  vc->IsValid(vcm, cfg, env, Stats, *this->cdInfo, true, &Callee)    
	) {
	free.push_back(cfg);
      }
    }
#if INTERMEDIATE_FILES
    att.push_back(cfg);
#endif
  }
  ptsRobot.erase(ptsRobot.begin(),ptsRobot.end());
  ptsObstacle.erase(ptsObstacle.begin(),ptsObstacle.end());
  
#if INTERMEDIATE_FILES
  WritePathConfigurations("att.path", att, env);
#endif
  
  return free;
};


template <class CFG>
void
OBPRM<CFG>::
GenerateSeeds(Environment* env, Stat_Class& Stats,
              int obst, int nseeds,
              int selectRobot, int selectObstacle,
              vector<CFG>* seeds) {
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();           
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(this->vcMethod);

  int rob = env->GetRobotIndex();
  std::string Callee(GetName());
  {std::string Method("-OBPRM::GenerateSeeds");Callee=Callee+Method;}

  vector<Vector3D> ptsRobot, ptsObstacle;
  ptsRobot = PointsOnMultiBody(env->GetMultiBody(rob), nseeds, selectRobot);
  
  ptsObstacle = PointsOnMultiBody(env->GetMultiBody(obst), nseeds, selectObstacle);
  
  CFG cfg;
  for(int i = 0 ; i < nseeds ; i++){
    if(cfg.GenerateOverlapCfg(env, rob, ptsRobot[i], ptsObstacle[i], &cfg)){
      // check if it is possible to generate a Cfg with this pose.
      if( // cfg.isCollision(env, Stats, cd, *this->cdInfo,true,&Callee)
          !vc->IsValid(vcm, cfg, env, Stats, *this->cdInfo, true, &Callee)
	) {
	seeds->push_back(cfg);
      }
    }
  }
  ptsRobot.erase(ptsRobot.begin(),ptsRobot.end());
  ptsObstacle.erase(ptsObstacle.begin(),ptsObstacle.end());
}


template <class CFG>
vector<Vector3D>
OBPRM<CFG>::
PointsOnMultiBody(MultiBody* mbody, int npts, int select) {
  int nFree = mbody->GetFreeBodyCount();
  
  vector<Vector3D> pts;
  pts.reserve(npts);
  
  // for robot(which have freebodys, sample points on last link only.
  // for obstacle, sample first link. (in fact, only ONE link?! )
  if(nFree)
    for(int j = 0 ; j < npts ; j++){
      pts.push_back(OBPRM<CFG>::PointOnBody(mbody->GetFreeBody(nFree-1),select,1));
    }
  else
    for(int j = 0 ; j < npts ; j++){
      pts.push_back(OBPRM<CFG>::PointOnBody(mbody->GetFixedBody(0), select, 0));
    }
  
  return pts;
}


template <class CFG>
void
OBPRM<CFG>::
Spread(CFG& pivot, double tStep, double rStep, int nspread,
       vector<CFG>* spread){
  for(int j = 0; j < nspread; j++) {
    CFG cfg;
    cfg.GetRandomCfg(tStep, rStep);
    cfg.add(pivot, cfg);
    spread->push_back(cfg);
  }
}

#endif


