#ifndef BasicOBPRM_h
#define BasicOBPRM_h

#include "NodeGenerationMethod.h"

#define EXPANSION_FACTOR 100
#define MAX_NUM_NODES_TRIES 100

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class BasicOBPRM
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**This performs basic, no frills, obstacle-based node generation.  This class is derived
 *off of NodeGenerationMethod.
 */
template <class CFG>
class BasicOBPRM : public NodeGenerationMethod<CFG> {
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
  BasicOBPRM();
  ///Destructor.
  ~BasicOBPRM();

  //@}

  //////////////////////
  // Access
  virtual char* GetName();
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual NodeGenerationMethod<CFG>* CreateCopy();

  /** Basic, no frills,  Obstacle Based Node Generation.
   * 
   * Algorithm:
   *   -# For each Obstacle in Environement.
   *       -# Generate Cfg inside C-Obstacle
   *          (using GenerateInsideCfg)
   *       -# Generate a ray shooting from Cfg in 1.
   *          (using Cfg::GetRandomRay)
   *       -# Generate a Free Cfg alone the direction of ray in 2.
   *          (using GenerateOutsideCfg)
   *       -# Generate Surface Cfgs using Binary Search Procedure.
   *       -# Generate Shells (using Shells)
   *   -# End For
   *
   * when there is no Obstacle (i.e. only object in the Enviroment is 
   * Robot) GenCfgsFromCObst will be called instead of alg above.
   * @note number of free Cfgs genertated in this algorithm
   * will be GN::numNodes/(number_of_obstacle)/GN::numShells
   *
   * @param _env Environment for getting geometric information.
   * @param cd Used to get free Cfg (checking collision).
   * @param nodes Used to store generated nodes.
   *
   *
   * @bug if number_of_obstacle is zero, above "equaltion" will cause
   * "divided by zero" run time error?!
   */
  virtual void GenerateNodes(Environment* _env, Stat_Class& Stats,
			     CollisionDetection* cd, 
			     DistanceMetric *dm, vector<CFG>& nodes);

  /**Generate Cfg in C-Free but near Obstacle.
   *These Gfgs are created alone the line made by
   *insideCfg and outsideCfg. Algorithm is :
   * -#do{
   *   -# do {
   *       -# mid = midpoint between in and out
   *       -# if mid is in C-Free then out=mid
   *       -# else in=mid
   *   -# } while dist(in,out) > clearance
   * -#}while (not enough nodes gnerated && not enough tries tried)
   *Here dist is ditance between in and out.
   *clearance is clearanceFactor * PositionRes.
   *All midpoints which are in C-Free will be recorded.
   *@param rob index for robot in Environment.
   *@param obst index for obstacle in Environment.
   *@param insideCfg Cfg inside C-Obstacle
   *@param outsideCfg Cfg outside C-Obstacle.
   *@param clearanceFactor used to calculate cleanance between Cfg and Obstacle.
   *@return A list of midpoints which are in C-Free.
   *@see FirstFreeCfgs, GenerateInsideCfg, and GenerateOutsideCfg
   *@see called by BasicOBPRM, GenSurfaceCfgs4ObstVERTEX.
   */
  vector<CFG> GenerateSurfaceCfg(Environment* env, Stat_Class& Stats,
				 CollisionDetection* cd, DistanceMetric* dm,
				 int rob, int obst, 
				 CFG& insideCfg, CFG& outsideCfg, double clearanceFactor = 1.0);
 protected:
  /**Any two of these are valid values for collision and free pairs
   *@see PointsOnMultiBody
   */
  enum PairOptions {
    cM,   ///< Center of Mass
    rV,   ///< Random Vertex
    rT,   ///< point in Random Triangle
    rE,   ///< Random Extreme vertex
    rW,   ///< point in Random Weighted triangle 
    cM_rV,///< Center of Mass or Random Vertex
    rV_rT,///< random Vertex or point in random Triangle
    rV_rW,///< random Vertex or point in random Weighted triangle
    N_rT, ///< Normal of a random Triangle
    all,  ///< all of the basics
    INVALID_OPTION    ///< This means somthing "Bad"...
  };

  /**Get Cfg that is in side C-Obstacle.
   *This is done by following:
   *   -# Move the center point of robot to the
   *      center point of the obststacle.
   *   -# if first step does not generate
   *      Cfg in side C-Obs then, the center point of 
   *      robot will be moved to any point on the
   *      surface of obstacle.
   *@param rob index for robot in Environment.
   *@param obst index for obstacle in Environment.
   *@param inside The gernerated Cfg inside C-Obstacle.
   *@return Always return true
   *@see GenerateOutsideCfg
   */
  bool GenerateInsideCfg(Environment* _env, Stat_Class& Stats,
			 CollisionDetection* _cd,
			 int rob, int obst, CFG* insideNode);

  /**Get Cfg that is in CFree accroding to given
   *Cfg in C-Obs and incremental value.
   *This is done by increasing Cfg in C-Obs
   *in given ray.
   *
   *@param rob index for robot in Environment.
   *@param obst index for obstacle in Environment.
   *@param InsideNode The Cfg which is inside C-Obstacle.
   *@param incrCfg This a ray in CSpace. This method will
   *       inf Cfg in Cfree in this given direction.
   *
   *@return Cfg in free space if this method found one. Otherwise
   *InsideNode will be returned.
   *@note this method will try 500 times to find free Cfg in
   *given direction.
   *@see GenerateInsideCfg
   */
  
  CFG GenerateRandomDirection(Environment *env, const CFG &InsideNode);

  /**Returns a configuration representing a vector in a random 
   *direction for Obstacle Based node sampling.
   *
   *@param InsideNode is a configuration obtained with GenerateInsideCfg
   *@return randomDir is a configuration representing a vector starting
   *from InsideNode. If InsideNode is inside the bounding box, randomDir
   *is just a random ray, otherwise it goes toward a random configuration 
   *inside the bounding box. In any case it is normalized and scaled by 
   *EXPANSION_FACTOR.
   *@see GenerateRandomDirection
   */


  bool PushCfgToBoundingBox(Environment *env, CFG& FromCfg, const CFG& ToCfg);

  /**Moves FromCfg toward ToCfg until it is inside the Bounding Box
   *if FromCfg is already inside the BoundingBox, it leaves it where 
   *it already is
   *
   *@param FromCfg is the configuration to move
   *@param ToCfg is the configuration to move toward
   *@return true when FromCfg is left inside the bounding box, false
   *otherwise.
   */

  CFG GenerateOutsideCfg(Environment* env, Stat_Class& Stats,
			 CollisionDetection* cd, 
			 int rob, int obst,
			 CFG& InsideNode, CFG& incrCfg);

  /**Generate Free Cfgs which are near C-Obstacles.
   *This method is used to generate free Cfgs for
   *the Environment with only one MultiBody, which
   *is robot. Therefore, there is no obstacle in
   *the world. Due to self-collision, there are 
   *C-Obstacles in C-Space.
   *
   *Following Algorithm is used:
   *   -# randomly generate the orienatation of Robot.
   *   -# if this Cfg causes self-collision.
   *       -# save this Cfg to obs_seed list
   *   -# else save to (Free Cfg) return list.
   *   -# Repeat above nCfgs times.
   *   -# for each Cfg, seedCfg, in obs_seed
   *       -# Generate a ray emanating out from seedCfg
   *       -# Find a Cfg free in direction of this ray
   *       -# GenerateSurfaceCfg, binary search for free Cfgs
   *       -# Get info.numShells Cfgs as Shell
   *       -# insert sell to return list.
   *   -# end for
   *   -# return "return list"
   *
   *@param obstacle index for obstacle MultiBody in Envoriment.
   *@param nCfgs number of free Cfgs that will be generated.
   *@param clearanceFactor used to calculate cleanance between 
   *Cfg and CObstacle.
   *@return a list of Free Cfgs which are near C-Obstacles.
   */
  vector<CFG> GenCfgsFromCObst(Environment* env, Stat_Class& Stats,
			       CollisionDetection* cd,
			       DistanceMetric* dm, 
			       int obstacle, int nCfgs, 
			       double clearanceFactor = 1.0);

  /**Get nshells Cfgs from given Cfg list.
   *@param nshells number of Cfgs that are going to
   *bereturned.
   *@note if the size of given list is smaller than nshells
   *then all element in the list will be returned.
   *@note if the size of given list is larger than nshells,
   *then one elemet will be return in every (size/nshells)
   *elemets along the given list.
   */
  static vector<CFG> Shells(vector<CFG> cfgs, int nshells);

  /**Check if all Cfgs in cfgs are in bouding box of Environment.
   *@param cfgs a list of parameters.
   *@param env Provides bouding box information.
   *@return True if all Cfgs in cfgs are in side bouding box.
   *@see Cfg::InBoundingBox
   */
  static vector<CFG> InsideBoundingBox(Environment* env, vector<CFG> cfgs);

  /**Get a point on the Body accroding specified method.
   *@param body a pointer to Body where a point will be generated.
   *@param isFreeBody 1 is this body is free body.
   *@param select one of PairOptions values. Defines which way this
   *new point should be generated.
   *@note Accroding to select value following methods are called.
   *   -# cM Body::GetCenterOfMass()
   *   -# rV ChooseRandomVertexOnBody
   *   -# rT ChooseRandomTriangleOnBody
   *   -# rE ExtremeVertex
   *   -# rW ChooseRandomWeightedTriangleOnBody
   *   -# cM_rV GetCenterOfMass or ChooseRandomVertexOnBody
   *      (either on, randomly choosed)
   *   -# rV_rT ChooseRandomVertexOnBody or ChooseRandomTriangleOnBody
   *      (either on, randomly choosed)
   *   -# rV_rW ChooseRandomVertexOnBody or 
   *      ChooseRandomWeightedTriangleOnBody 
   *      (either on, randomly choosed)
   *   -# all One of above. Randomly choosed.
   *   -# none of above exit will be called.
   *@return a point which is generated by one of above method.
   */
  static Vector3D PointOnBody(Body* body, int select, bool isFreeBody);

  //@{
  /**Choose a random vertex on given Body.
   *
   *@param isFreeBody Check if this is a Body of robot.
   *
   *@note if this is a Body of robot, GetPolyhedron,
   *which returns vertex in local coordinate system, will
   *be called to get robot's geometry. Otherwise,
   *GetWorldPolyhedron will be called to get obstacle's
   *geometry in world coordinate system.
   *
   *@see Body::GetPolyhedron, Body::GetWorldPolyhedron,
   *and GMSPolyhedron::vertexList.
   */
  static Vector3D ChooseRandomVertexOnBody(Body* body, bool isFreeBody);
  /**Choose a random point inside a random trianagle on the
   *given Body.
   *
   *@param isFreeBody Check if this is a Body of robot.
   *
   *@note if this is a Body of robot, GetPolyhedron,
   *which returns vertex in local coordinate system, will
   *be called to get robot's geometry. Otherwise,
   *GetWorldPolyhedron will be called to get obstacle's
   *geometry in world coordinate system.
   *
   *@see Body::GetPolyhedron, Body::GetWorldPolyhedron,
   *GMSPolyhedron::vertexList, and GMSPolyhedron::polygonList.
   */
  static Vector3D ChooseRandomTriangleOnBody(Body*, bool);
  /**Choose a random extreme vertex on given Body.
   *A extreme vertex is a vertex has max/min coordinate
   *in X, Y, or Z direction.
   *One of extreme vertices on this body will be returned.
   *
   *@param isFreeBody Check if this is a Body of robot.
   *
   *@note if this is a Body of robot, GetPolyhedron,
   *which returns vertex in local coordinate system, will
   *be called to get robot's geometry. Otherwise,
   *GetWorldPolyhedron will be called to get obstacle's
   *geometry in world coordinate system.
   *
   *@see Body::GetPolyhedron, Body::GetWorldPolyhedron,
   *and GMSPolyhedron::vertexList.
   */
  static Vector3D ExtremeVertex(Body*, bool isFreeBody);
  /**Choose a random point inside a random weighted trianagle 
   *on the given Body.
   *Weighted trianagle is a trianagle weighted by its area. 
   *If the size of a trianagle triangle is big, this 
   *trianagle has big chance to be chose.
   *
   *@param isFreeBody Check if this is a Body of robot.
   *
   *@note if this is a Body of robot, GetPolyhedron,
   *which returns vertex in local coordinate system, will
   *be called to get robot's geometry. Otherwise,
   *GetWorldPolyhedron will be called to get obstacle's
   *geometry in world coordinate system.
   *
   *@see Body::GetPolyhedron, Body::GetWorldPolyhedron,
   *GMSPolyhedron::vertexList, and GMSPolyhedron::polygonList.
   */
  static Vector3D ChooseRandomWeightedTriangleOnBody(Body* body, bool isFreeBody);
  /**Choose a randome point inside a given triangle.
   *This triangle is made by 3 vertices, p, q, and r.
   *This mehotd randomly generates Barycentric coordinate
   *for this return point.
   *@see This funtion is called by ChooseRandomTriangleOnBody.
   */
  static Vector3D ChoosePointOnTriangle(Vector3D p, Vector3D q, Vector3D r);
  //@}

  /** returns the first n free cfgs from a given vector of cfgs.  
   * Returns all free cfgs if there are less than n. 
   *@param n how many free Cfg in cfgs will be returned.
   *@param cfgs a list of Cfgs which will be examed in this method
   *to eact Free Cfgs.
   */
  vector<CFG> FirstFreeCfgs(Environment* env, Stat_Class& Stats,
			    CollisionDetection* cd, 
			    vector<CFG> cfgs, int n);
  /**Return all free cfgs in the given vector of cfgs.
   *@see FirstFreeCfgs(Environment *,CollisionDetection *, vector<Cfg>, GNInfo&, int)
   */
  vector<CFG> FirstFreeCfgs(Environment* env, Stat_Class& Stats,
			    CollisionDetection* cd, 
			    vector<CFG> cfgs);

 public:
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**Number of free Cfgs will be generated along a sample ray.
   *A binary seacrh function, GenerateMapNodes::GenerateSurfaceCfg,
   *generates a list of free Cfgs along a sample ray
   *shooting from inside of C-Obstacle.
   *This variable defines how many Cfgs will be extracted 
   *from this list.
   *@see GenerateMapNodes::Shells 
   */
  num_param<int> numShells;
  /// max. # of attempts at surface convergence	
  static const int MAX_CONVERGE;
};


/////////////////////////////////////////////////////////////////////
//
//  definitions for class BasicOBPRM declarations
//
/////////////////////////////////////////////////////////////////////
/* //moved to main_obprm.cpp/main_query.cpp 
   //because hp needs explicit instantiation of static members
template <class CFG>
const int BasicOBPRM<CFG>::MAX_CONVERGE = 20;  
*/

template <class CFG>
BasicOBPRM<CFG>::
BasicOBPRM() : NodeGenerationMethod<CFG>(),      
  numShells        ("shells",            3,  1,   50) {
  numShells.PutDesc("INTEGER","(number of shells, default 3)");
}


template <class CFG>
BasicOBPRM<CFG>::
~BasicOBPRM() {
}


template <class CFG>
char*
BasicOBPRM<CFG>::
GetName() {
  return "BasicOBPRM";
}


template <class CFG>
void
BasicOBPRM<CFG>::
SetDefault() {
  NodeGenerationMethod<CFG>::SetDefault();
  numShells.PutValue(3);
}


template <class CFG>
void
BasicOBPRM<CFG>::
ParseCommandLine(int argc, char **argv) {
  for (int i =1; i < argc; ++i) {
    if( numNodes.AckCmdLine(&i, argc, argv) ) {
    }else if (exactNodes.AckCmdLine(&i, argc, argv) ) {
    }else if (numShells.AckCmdLine(&i, argc, argv) ) {
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
BasicOBPRM<CFG>::
PrintUsage(ostream& _os) {
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t"; numNodes.PrintUsage(_os);
  _os << "\n\t"; numShells.PrintUsage(_os);
  _os << "\n\t"; exactNodes.PrintUsage(_os);
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG>
void
BasicOBPRM<CFG>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << numNodes.GetFlag() << " " << numNodes.GetValue() << " ";
  _os << exactNodes.GetFlag() << " " << exactNodes.GetValue() << " ";
  _os << numShells.GetFlag() << " " << numShells.GetValue() << " ";
  _os << endl;
}


template <class CFG>
NodeGenerationMethod<CFG>* 
BasicOBPRM<CFG>::
CreateCopy() {
  NodeGenerationMethod<CFG> * _copy = new BasicOBPRM<CFG>(*this);
  return _copy;
}


template <class CFG>
void
BasicOBPRM<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats, 
	      CollisionDetection* cd, DistanceMetric* dm, 
	      vector<CFG>& nodes) {  
#ifndef QUIET
  cout << "(numNodes=" << numNodes.GetValue() << ", "<<flush;
  cout << "(exactNodes=" << exactNodes.GetValue() << ", "<<flush;
  cout << "numShells=" << numShells.GetValue() << ") "<<flush;
#endif
  
#if INTERMEDIATE_FILES
  vector<CFG> surface;
#endif
  
  vector<CFG> preshells, shells, tmp, obstSurface;
  int numMultiBody = _env->GetMultiBodyCount();
  int numExternalBody = _env->GetExternalBodyCount();

  std::string Callee(GetName());
  {std::string Method("-BasicOBPRM::GenerateNodes");Callee = Callee+Method;}
  
  int robot = _env->GetRobotIndex();
  
  CFG InsideNode, OutsideNode, low, high, mid;	
  
  int N; // # of nodes per each shell;
; // just to make sure its default behavior is the same as the original node generation method;
  bool bExact = exactNodes.GetValue() ? true : false;

  if (numExternalBody > 1) //more objects besides the robot
    N = numNodes.GetValue() / (numExternalBody-1)  // -1 for the robot
                  / numShells.GetValue();
  else //the only obstacle is the robot		
    N = numNodes.GetValue() / (numExternalBody)
                  / numShells.GetValue();

  
  if (N < 1) N = 1; //max(numNodes.GetValue(),numShells.GetValue());

  int nNodesGap = numNodes.GetValue() - nodes.size(); 
  int nNumTries = 0;
  do{ // while not enough nodes are generated
    for (int obstacle = 0 ; obstacle < numExternalBody ; obstacle++) {
      if (obstacle != robot) {  // && obstacle is "Passive" not "Active" robot
      
	for(int n = 0 ; n < N; n++) {
	
	  // Generate Inside cfg
	  CFG InsideNode;
	  if(!GenerateInsideCfg(_env, Stats, cd, robot, obstacle, &InsideNode)) {
	    cout << "\nError: cannot overlap COMs of robot & obstacle\n";
	    continue;
	  }
	  if(!InsideNode.isCollision(_env,Stats,cd,robot,obstacle,*cdInfo, 
				     true, &(Callee))){
	    cout << "\nError: Seed not in collision w/"
	      " obstacle[index="<<obstacle<<"]\n" << flush;
	    continue;
	  }	
	  // Generate Random direction
	  CFG incrCfg = GenerateRandomDirection(_env, InsideNode);
	  // Generate outside cfg
	  CFG OutsideNode = GenerateOutsideCfg(_env,Stats,cd,robot,obstacle,
					       InsideNode,incrCfg);
	  //move inside node to the bounding box if required
	  bool inBBox = PushCfgToBoundingBox(_env, InsideNode, OutsideNode);
	  if (inBBox) {
	    if (OutsideNode.AlmostEqual(InsideNode) ||
		!InsideNode.isCollision(_env,Stats,cd,robot,obstacle,*cdInfo))
	      continue; //no valid outside or inside node was found	
	  }
	  else
	    continue; // no valid inside node was found

	 
	  // Generate surface cfgs
	  tmp = GenerateSurfaceCfg(_env,Stats,cd,dm,
				   robot,obstacle, InsideNode,OutsideNode);
	
	  // Choose as many as nshells
	  preshells = Shells(tmp, numShells.GetValue());
	  shells = InsideBoundingBox(_env, preshells);
	  preshells.erase(preshells.begin(), preshells.end());
	
	  // Collect the cfgs for this obstacle
	  obstSurface.insert(obstSurface.end(),
			     shells.begin(),shells.end());
	
	} // endfor: n
      
	// Collect the generated surface nodes
	for (int i=0;i<obstSurface.size();i++){
	  obstSurface[i].obst = obstacle;
/* 	  nodes.push_back(obstSurface[i]); */
	}
      
#if INTERMEDIATE_FILES
	surface.insert(surface.end(),
		       obstSurface.begin(),obstSurface.end());
#endif
      
/* 	obstSurface.erase   (obstSurface.begin(),obstSurface.end()); */
      
      } // endif (obstacle != robot)
      else
	if(numExternalBody == 1) { //if robot is the only object
	  //		  if(numMultiBody == 1) {
	  vector<CFG> CobstNodes = GenCfgsFromCObst(_env, Stats, cd, dm, obstacle, 
						    numNodes.GetValue());
	  int i;
	  for(i=0; i<CobstNodes.size(); ++i) {
	    CobstNodes[i].obst = obstacle;
/* 	    nodes.push_back(CobstNodes[i]); */
	    obstSurface.push_back(CobstNodes[i]);
	  }
#if INTERMEDIATE_FILES
	  surface.insert(surface.end(),CobstNodes.begin(), CobstNodes.end());
#endif
	}
    
    } // endfor: obstacle


    if (bExact){ //exact nodes
      int nActualNodes = obstSurface.size();
      if ( nActualNodes < nNodesGap){
	nodes.insert(nodes.end(), obstSurface.begin(), obstSurface.end()); 	
	nNodesGap = nNodesGap - nActualNodes;
	obstSurface.erase(obstSurface.begin(),obstSurface.end());

	if (numExternalBody > 1) //more objects besides the robot
	  N = nNodesGap / (numExternalBody-1)  // -1 for the robot
	    / numShells.GetValue();
	else //t		he only obstacle is the robot		
	  N = nNodesGap / (numExternalBody)
	    / numShells.GetValue();
	if (N < 1) 
	  N = 1; 
      }else if (nActualNodes == nNodesGap){//Generated exact number of nodes
	nodes.insert(nodes.end(), obstSurface.begin(), obstSurface.end()); 	
	obstSurface.erase(obstSurface.begin(),obstSurface.end());
	nNodesGap = 0;
	N = 0;	
      }else{ // nActualNodes > nNodesGap ;
	//+++++++++++Need to sample here	
	vector <bool> indices(nActualNodes);
	int iSelect;
	for (int j = 0; j < nNodesGap; j++) //randomly sample #nNodesGap 
	  {
	    do { iSelect = rand() % nActualNodes; } while (indices[iSelect] == true);
	    indices[iSelect] = true;
	  }

	for (int j = 0; j < nActualNodes; j++) //push those selected one in to nodes
	  if (indices[j])
	    nodes.push_back(obstSurface[j]);

	obstSurface.erase(obstSurface.begin(),obstSurface.end());
	nNodesGap = 0;
	N = 0;
      }
    }else { // not asked for exact nodes
      nodes.insert(nodes.end(), obstSurface.begin(), obstSurface.end()); 
      obstSurface.erase(obstSurface.begin(),obstSurface.end());
      N = 0;
    }
    nNumTries++;
  }while (N>0 && nNumTries < MAX_NUM_NODES_TRIES); // while not enough nodes generated. 
  
  if (nNumTries >= MAX_NUM_NODES_TRIES)
    cerr << GetName() << ": Can\'t generate engough nodes! " << endl;
  
#if INTERMEDIATE_FILES
  WritePathConfigurations("surface.path", surface, _env);
#endif
};


template <class CFG>
bool
BasicOBPRM<CFG>::
GenerateInsideCfg(Environment* _env, Stat_Class& Stats, 
		  CollisionDetection* _cd,
		  int rob, int obst, CFG* insideNode) {
  
  std::string Callee(GetName());
  {std::string Method("-BasicOBPRM::GenerateInsideCfg");Callee=Callee+Method;}

  bool tmp = insideNode->GenerateOverlapCfg(_env, rob,
					    _env->GetMultiBody(rob)->GetCenterOfMass(),
					    _env->GetMultiBody(obst)->GetCenterOfMass(),
					    insideNode);
  
  // check the cfg obtained by center of mass overlapping if valid
  if (!insideNode->isCollision(_env, Stats, _cd, rob, obst,
			       *cdInfo, true, &(Callee))) {
    
    // if center of mass does not work in getting the cfg in collision,
    // use random vertex of an obstacle (J Kim)
    Vector3D vP;
    
    // code copied from GenerateMapNodes::PointsOnMultiBody()
    vP = PointOnBody(_env->GetMultiBody(obst)->GetFixedBody(0), rV, false);
    
    // get inside cfg again by using vP instead of center of mass
    bool tmp = insideNode->GenerateOverlapCfg(_env, rob,
					      _env->GetMultiBody(rob)->GetCenterOfMass(),
					      vP,
					      insideNode);
    tmp = true;
  }
  return tmp;
}

template <class CFG>
CFG
BasicOBPRM<CFG>::
GenerateRandomDirection(Environment* env, const CFG &InsideNode) {
  CFG randomDir;
  if (InsideNode.InBoundingBox(env))
    randomDir.GetRandomRay(EXPANSION_FACTOR * env->GetPositionRes());
  else { //ensure randomDir goes toward BBox
    CFG insideTmp;
    insideTmp.GetRandomCfg(env);
    randomDir.subtract(insideTmp, InsideNode); //vctr to add to InsideNode 
    randomDir.divide(randomDir, randomDir.PositionMagnitude());//normalize
    randomDir.multiply(randomDir, EXPANSION_FACTOR * env->GetPositionRes());
  }
  return randomDir;
}

template <class CFG>
CFG
BasicOBPRM<CFG>::
GenerateOutsideCfg(Environment* env, Stat_Class& Stats, 
		   CollisionDetection* cd, 
		   int rob, int obst,
                   CFG& InsideNode, CFG& incrCfg) {
  
  int count = 0;
  CFG OutsideNode;
  std::string Callee(GetName());
  {std::string Method("-BasicOBPRM::GenerateOutsideCfg");Callee=Callee+Method;}

  OutsideNode.add(InsideNode, incrCfg);
  while(OutsideNode.isCollision(env, Stats, cd, rob, obst, *cdInfo, 
                                true, &(Callee)) ) {
    OutsideNode.add(OutsideNode, incrCfg);
    if(count++ > 500)
      return InsideNode;
  }
  return OutsideNode;
}

template <class CFG>
bool
BasicOBPRM<CFG>::
PushCfgToBoundingBox(Environment *env, CFG& FromCfg, const CFG& ToCfg) {
  //cout << "Moving 1" << endl;
  if (FromCfg.InBoundingBox(env))
    return true;
  //cout << "Moving 2" << endl;
  //move inside node to the bounding box,
  //assumes that the line between (insideCfg,outsideCfg) crosses the BBox
  //can be reimplemented with binary search
  CFG tick = FromCfg;
  CFG incr;
  int n_ticks;
  incr.FindIncrement(FromCfg, ToCfg, &n_ticks, env->GetPositionRes(), env->GetOrientationRes());
  int tk = 0;
  while (!tick.InBoundingBox(env) && tk < n_ticks && tick != ToCfg) {
    tick.Increment(incr);
    tk++;
  }
  if (tick.InBoundingBox(env))
    FromCfg = tick;

  //check if move was succesful
  if (!FromCfg.InBoundingBox(env))
    return false;
  else
    return true;
}

template <class CFG>
vector<CFG>
BasicOBPRM<CFG>::
GenerateSurfaceCfg(Environment* env, Stat_Class& Stats,
		   CollisionDetection* cd, DistanceMetric* dm,
		   int rob, int obst, 
		   CFG& insideCfg, CFG& outsideCfg, double clearanceFactor) {
	
  const double PositionRes = env->GetPositionRes();
  vector<CFG> surface; 
  surface.reserve(MAX_CONVERGE);
  vector<CFG> tmp;     
  tmp.reserve(MAX_CONVERGE);
  
  CFG low, high, mid;
  double delta;
  int cnt;
  
  std::string Callee(GetName()), CallM("(mid)"),CallH("(High)");
  {std::string Method("-BasicOBPRM::GenerateSurfaceCfg"); Callee = Callee+Method;}

  low = insideCfg; 
  high = outsideCfg;
  mid.WeightedSum(low, high, 0.5);
  delta = dm->Distance(env, low, high);
  cnt = 0;
  
  // Do the Binary Search
  tmp.push_back(high);
  std::string tmpStr = Callee+CallM;
  while((delta >= clearanceFactor*PositionRes) && (cnt < MAX_CONVERGE)){
    if(mid.isCollision(env, Stats, cd , rob, obst, *cdInfo,
                       true, &tmpStr) ) {
      low = mid;
    } else {
      high = mid;
      tmp.push_back(high);
    }
    mid.WeightedSum(low, high, 0.5);
    delta = dm->Distance(env, low, high);
    cnt++;
  }
  
  // if converged save the cfgs that don't collide with the environment
  if(cnt < MAX_CONVERGE) {
    tmpStr = Callee+CallH;
    if(!high.isCollision(env, Stats, cd, *cdInfo, true, &tmpStr)) {
      surface = FirstFreeCfgs(env, Stats, cd,tmp);
    }
  }
  return surface;
}


template <class CFG>
vector<CFG>
BasicOBPRM<CFG>::
GenCfgsFromCObst(Environment* env, Stat_Class& Stats,
		 CollisionDetection* cd, DistanceMetric* dm, 
		 int obstacle, int nCfgs, double clearanceFactor) {
  
  int robot = env->GetRobotIndex();
  vector<CFG> surface, obstSeeds;
  Vector3D voidA, voidB;
  CFG gen;
  int i;
  std::string Callee(GetName());
  {std::string Method("-BasicOBPRM::GenCfgsFromCObst"); Callee=Callee+Method;}

  for(i=0; i<nCfgs; ++i) {
    ///random orientation....?
    gen.GenerateOverlapCfg(env, robot, voidA, voidB, &gen);  // voidA, voidB is not used.
    
    ///check collision
    if(gen.isCollision(env, Stats, cd, *cdInfo,true, &(Callee)))
      obstSeeds.push_back(gen);
    else
      surface.push_back(gen);
  }
  
  vector<CFG> tmp, preshells, shells;
  for(i = 0; i < obstSeeds.size(); i++) {
    
    CFG incrCfg;
    incrCfg.GetRandomRay(EXPANSION_FACTOR*env->GetPositionRes());
    
    CFG OutsideNode =
      GenerateOutsideCfg(env,Stats,cd,robot,obstacle,obstSeeds[i],incrCfg);
    if(OutsideNode.AlmostEqual(obstSeeds[i])) continue; // can not find outside node.
    
    tmp =
      GenerateSurfaceCfg(env,Stats,cd,dm,robot,obstacle,obstSeeds[i],OutsideNode,clearanceFactor);
    
    // Choose as many as nshells
    preshells = Shells(tmp, numShells.GetValue());
    shells = InsideBoundingBox(env, preshells);
    preshells.erase(preshells.begin(), preshells.end());
    
    // Collect the cfgs for this obstacle
    surface.insert(surface.end(),shells.begin(),shells.end());
  }
  return surface;
};


template <class CFG>
vector<CFG>
BasicOBPRM<CFG>::
Shells(vector<CFG> cfgs, int nshells) {
  int size = cfgs.size();
  int limit = min(nshells, size);
  vector<CFG> shells;
  shells.reserve(nshells);
  if(limit > 0) {
    int step = size/limit;
    for(int i = 0 ; i < limit ; i++){
      int k = (cfgs.size() - 1) - (i*step);
      shells.push_back(cfgs[k]);
    }
  }
  return shells;
}


template <class CFG>
vector<CFG>
BasicOBPRM<CFG>::
InsideBoundingBox(Environment* env, vector<CFG> cfgs) {
  vector<CFG> ncfgs;
  
  for(int i = 0 ; i < cfgs.size() ; i++)
    if(cfgs[i].InBoundingBox(env))
      ncfgs.push_back(cfgs[i]);
  return ncfgs;
};


template <class CFG>
Vector3D
BasicOBPRM<CFG>::
PointOnBody(Body* body, int select, bool isFreeBody) {
  Vector3D pt;
  int opt;
  
  switch( (PairOptions)select ){
  case cM:
    pt = body->GetCenterOfMass();
    break;
    
  case rV:
    pt = ChooseRandomVertexOnBody(body, isFreeBody);
    break;
    
  case rT:
    pt = ChooseRandomTriangleOnBody(body, isFreeBody);
    break;
    
  case rE:
    pt = ExtremeVertex(body, isFreeBody);
    break;
    
  case rW:
    pt = ChooseRandomWeightedTriangleOnBody(body, isFreeBody);
    break;
    
  case cM_rV:
    opt = rand() % 2;
    if(opt == 0){
      pt = body->GetCenterOfMass();
    } else {
      pt = ChooseRandomVertexOnBody(body, isFreeBody);
    }
    break;
    
  case rV_rT:
    opt = rand() % 2;
    if(opt == 0){
      pt = ChooseRandomVertexOnBody(body, isFreeBody);
    } else {
      pt = ChooseRandomTriangleOnBody(body, isFreeBody);
    }
    break;
    
  case rV_rW:
    opt = rand() % 2;
    if(opt == 0){
      pt = ChooseRandomVertexOnBody(body, isFreeBody);
    } else {
      pt = ChooseRandomWeightedTriangleOnBody(body, isFreeBody);
    }
    break;
    
  case all:
    opt = rand() % 5;
    if(opt == 0){
      pt = body->GetCenterOfMass();
    } else if(opt == 1){
      pt = ChooseRandomVertexOnBody(body, isFreeBody);
    } else if(opt == 2){
      pt = ChooseRandomTriangleOnBody(body, isFreeBody);
    } else if(opt == 3){
      pt = ExtremeVertex(body, isFreeBody);
    } else {
      pt = ChooseRandomWeightedTriangleOnBody(body, isFreeBody);
    }
    break;
    
  default:
    cout << "\n Unknown Option for PointOnBody \n";
    exit(0);
    break;
  }
  return pt;
}


template <class CFG>
Vector3D
BasicOBPRM<CFG>::
ChooseRandomVertexOnBody(Body* body, bool isFreeBody) {
  GMSPolyhedron polyhedron;
  
  // for robot, choose body frame; for obstacle, choose world frame
  if(isFreeBody) 
    polyhedron = body->GetPolyhedron();
  else           
    polyhedron = body->GetWorldPolyhedron();
  
  // We choose a vertex of the part at random
  return polyhedron.vertexList[(int)(drand48()*polyhedron.numVertices)];
};


template <class CFG>
Vector3D
BasicOBPRM<CFG>::
ChooseRandomTriangleOnBody(Body* body, bool isFreeBody) {
  GMSPolyhedron polyhedron;
  // for robot, choose body frame; for obstacle, choose world frame
  if(isFreeBody)
    polyhedron = body->GetPolyhedron();
  else
    polyhedron = body->GetWorldPolyhedron();
  
  // We choose a triangle of the body at random
  
  GMSPolygon *poly = &polyhedron.polygonList[(int)(drand48()*polyhedron.numPolygons)];
  Vector3D p, q, r;
  p = polyhedron.vertexList[poly->vertexList[0]];
  q = polyhedron.vertexList[poly->vertexList[1]];
  r = polyhedron.vertexList[poly->vertexList[2]];
  
  Vector3D u;
  u = ChoosePointOnTriangle(p, q, r);
  return u;
};


template <class CFG>
Vector3D
BasicOBPRM<CFG>::
ExtremeVertex(Body* body, bool isFreeBody) {
  GMSPolyhedron polyhedron;
  // for robot, choose body frame; for obstacle, choose world frame
  if(isFreeBody) 
    polyhedron = body->GetPolyhedron();
  else           
    polyhedron = body->GetWorldPolyhedron();
  
  int indexVert[6];
  for(int j = 0 ; j < 6 ; j++){
    indexVert[j] = 0;
  }
  
  for(int i = 1 ; i < polyhedron.numVertices ; i++){
    
    //MAX X
    if(polyhedron.vertexList[i][0] < polyhedron.vertexList[indexVert[0]][0])
      indexVert[0] = i;
    
    //MIN X
    if(polyhedron.vertexList[i][0] > polyhedron.vertexList[indexVert[1]][0])
      indexVert[1] = i;
    
    //MAX Y
    if(polyhedron.vertexList[i][1] < polyhedron.vertexList[indexVert[2]][1])
      indexVert[2] = i;
    
    //MIN Y
    if(polyhedron.vertexList[i][1] > polyhedron.vertexList[indexVert[3]][1])
      indexVert[3] = i;
    
    //MAX Z
    if(polyhedron.vertexList[i][2] < polyhedron.vertexList[indexVert[4]][2])
      indexVert[4] = i;
    
    //<MIN Z
    if(polyhedron.vertexList[i][2] > polyhedron.vertexList[indexVert[5]][2])
      indexVert[5] = i;
  }
  
  // Choose an extreme random vertex at random
  int index = rand() % 6;
  return polyhedron.vertexList[index];
};


template <class CFG>
Vector3D
BasicOBPRM<CFG>::
ChooseRandomWeightedTriangleOnBody(Body* body, bool isFreeBody) {
  GMSPolyhedron polyhedron;
  // for robot, choose body frame; for obstacle, choose world frame
  if(isFreeBody){
    polyhedron = body->GetPolyhedron();
  } else {
    polyhedron = body->GetWorldPolyhedron();
  }
  
  double area;
  area = body->GetPolyhedron().area;
  
  double targetArea = area * drand48();
  
  int index, i;
  double sum;
  index = 0; i = 1; sum = body->GetPolyhedron().polygonList[0].area;
  while(targetArea > sum){
    sum += body->GetPolyhedron().polygonList[i].area;
    index++;
    i++;
  }
  
  // We choose the triangle of the body with that index
  GMSPolygon *poly = &polyhedron.polygonList[index];
  
  // We choose a random point in that triangle
  Vector3D p, q, r;
  p = polyhedron.vertexList[poly->vertexList[0]];
  q = polyhedron.vertexList[poly->vertexList[1]];
  r = polyhedron.vertexList[poly->vertexList[2]];
  
  Vector3D u;
  u = ChoosePointOnTriangle(p, q, r);
  return u;
};


template <class CFG>
Vector3D
BasicOBPRM<CFG>::
ChoosePointOnTriangle(Vector3D p, Vector3D q, Vector3D r) {
  
  Vector3D u, v;
  u = q - p;
  v = r - p;
  
  double s = drand48(); double t = drand48();
  while(s + t > 1){
    t = drand48();
  }
  return (p + u*s + v*t);
};


template <class CFG>
vector<CFG>
BasicOBPRM<CFG>::
FirstFreeCfgs(Environment* env, Stat_Class& Stats, CollisionDetection* cd, 
	      vector<CFG> cfgs, int n) {
  
  int size = cfgs.size();
  n = min(n,size);
  std::string Callee(GetName());
  {std::string Method("-BasicOBPRM::FirstFreeCfgs"); Callee = Callee+Method;}
  
  vector<CFG> free;
  free.reserve(size);
  int i = 0; int cnt = 0;
  for (i = 0, cnt = 0; i < size && cnt < n; i++){
    if(!cfgs[i].isCollision(env, Stats, cd, *cdInfo,true,&(Callee))){
      free.push_back(cfgs[i]);
      cnt++;
    }
  }
  return free;
}


template <class CFG>
vector<CFG>
BasicOBPRM<CFG>::
FirstFreeCfgs(Environment* env, Stat_Class& Stats, CollisionDetection* cd, 
	      vector<CFG> cfgs) {
  int size = cfgs.size();
  return FirstFreeCfgs(env, Stats, cd, cfgs, size);
}

#endif


