#ifndef BasicOBPRM_h
#define BasicOBPRM_h

#include "NodeGeneratorMethod.h"

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
  BasicOBPRM(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  ///Destructor.
  ~BasicOBPRM();
  virtual void ParseXML(XMLNodeReader& in_Node);
  virtual void ParseXMLshells(XMLNodeReader& in_Node);
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
  ///Used in new MPProblem framework.
  virtual void PrintOptions(ostream& out_os);
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

  virtual void ComputeCSpaceShells(Environment* _env, Stat_Class& Stats,
			     CollisionDetection* cd, 
			     DistanceMetric *dm, CFG& _freeCfg, CFG& _incr, vector<CFG>& nodes);
          
  virtual void GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG > &outCfgs);

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
  
  CFG GenerateRandomDirection(Environment *env, DistanceMetric* dm,
			      const CFG &InsideNode);

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
  int numShells;
  /// max. # of attempts at surface convergence	
  static const int MAX_CONVERGE;

  int m_balanced;

  unsigned long int m_balColl, m_balFree;

  //Index for next node 
  //used in incremental map generation
  static int nextNodeIndex;
};


/////////////////////////////////////////////////////////////////////
//
//  definitions for class BasicOBPRM declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG>
const int BasicOBPRM<CFG>::MAX_CONVERGE = 20;  

template <class CFG>
int BasicOBPRM<CFG>::nextNodeIndex = 0;


template <class CFG>
BasicOBPRM<CFG>::
BasicOBPRM() : NodeGenerationMethod<CFG>() {
  
}

template <class CFG>
BasicOBPRM<CFG>::
    BasicOBPRM(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    NodeGenerationMethod<CFG>(in_Node, in_pProblem) {
  LOG_DEBUG_MSG("BasicOBPRM::BasicOBPRM()");
  ParseXML(in_Node);
  LOG_DEBUG_MSG("~BasicOBPRM::BasicOBPRM()");
}

template <class CFG>
void BasicOBPRM<CFG>::
ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("BasicOBPRM::ParseXML()");
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "shells") {
      ParseXMLshells(*citr);
    } else {
      //citr->warnUnknownNode();
    }
  }
  
  m_balanced  = in_Node.numberXMLParameter(string("Balanced"), false, 
                                                1,0,1, string("Balanced OBPRM"));
  in_Node.warnUnrequestedAttributes();

  LOG_DEBUG_MSG("~BasicOBPRM::ParseXML()");
}

template <class CFG>
void BasicOBPRM<CFG>::
ParseXMLshells(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("BasicOBPRM::ParseXMLshells()");

  in_Node.verifyName(string("shells"));
  numShells = in_Node.numberXMLParameter(string("number"),true, 3,0,10,
                                         string("Number of Shells"));
  
  in_Node.warnUnrequestedAttributes();
  LOG_DEBUG_MSG("~BasicOBPRM::ParseXMLshells()");
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
  numShells = 3;
  m_balanced = 0;

  m_balColl=0;
  m_balFree=0;
}

template <class CFG>
int
BasicOBPRM<CFG>::
GetNextNodeIndex() {
  return nextNodeIndex;
}

template <class CFG>
void
BasicOBPRM<CFG>::
SetNextNodeIndex(int index) {
  nextNodeIndex = index;
}

template <class CFG>
void
BasicOBPRM<CFG>::
IncreaseNextNodeIndex(int numIncrease) {
  nextNodeIndex += numIncrease;
}


template <class CFG>
void
BasicOBPRM<CFG>::
PrintOptions(ostream& out_os){
  out_os << "    " << GetName() << ":: ";
  out_os << " num nodes = " << this->numNodes << " ";
  out_os << " exact  = " << this->exactNodes << " ";
  out_os << " chunk size = " << this->chunkSize << " ";
  out_os << " num shells = " << numShells << " ";
  out_os << " Balanced = " << m_balanced << " ";
  out_os << endl;
}



template <class CFG>
NodeGenerationMethod<CFG>* 
BasicOBPRM<CFG>::
CreateCopy() {
  NodeGenerationMethod<CFG> * _copy = new BasicOBPRM<CFG>(*this);
  return _copy;
}

///\todo FIX BasicOBPRM::balanced implementation.  This one is 
///quick and dirty for ICRA06 submission.
template <class CFG>
void
BasicOBPRM<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats, 
	      CollisionDetection* cd, DistanceMetric* dm, 
	      vector<CFG>& nodes) {  
         
  
  m_balColl=0;
  m_balFree=0;


  LOG_DEBUG_MSG("BasicOBPRM::GenerateNodes()");
#ifndef QUIET
  cout << "(numNodes=" << this->numNodes << ", "<<flush;
  cout << "(exactNodes=" << this->exactNodes << ", "<<flush;
  cout << "(chunkSize=" << this->chunkSize << ", "<<flush;
  cout << "numShells=" << numShells << ") "<<flush;
#endif
  
#if INTERMEDIATE_FILES
  vector<CFG> path; 
  path.reserve(this->numNodes);
#endif

  std::string Callee(GetName()), CallCnt;
  {std::string Method("-BasicOBPRM::GenerateNodes"); Callee = Callee+Method;}
if(m_balanced == 0) {
  // generate in bounding box
  for (int attempts=0,newNodes=0,success_cntr=0;  success_cntr < this->numNodes ; attempts++) {
    //LOG_DEBUG_MSG("BasicOBPRM::GenerateNodes() attempts = " << attempts);
    CFG cfg1,last_free;
    cfg1.GetRandomCfg(_env);
    
    CallCnt=" 1st Sample"; 
    std::string tmpStr = Callee+CallCnt;
    
    bool cfg1_free = !cfg1.isCollision(_env,Stats,cd,*this->cdInfo, true, &tmpStr);
    if (!cfg1_free) {  //push out of Obs
      //LOG_DEBUG_MSG("BasicOBPRM::GenerateNodes() -- Push Out of Obs");
      CFG r_dir;
      r_dir.GetRandomRay(min(_env->GetPositionRes(),
                         _env->GetOrientationRes()), _env, dm);
      bool pushed_enough = false;
      bool outofbb = false;
      do {
        //last_free.equals(cfg1);
        cfg1.Increment(r_dir);
        if(!cfg1.InBoundingBox(_env) ) {outofbb=true;continue;} //out of bounding box
        CallCnt=" PushOutOfObs"; 
        std::string tmpStr = Callee+CallCnt;
        pushed_enough = !cfg1.isCollision(_env,Stats,cd,*this->cdInfo, true, &tmpStr);
        if(pushed_enough) {
         // nodes.push_back(CFG(cfg1));
         // newNodes++;
         ComputeCSpaceShells(_env, Stats, cd, dm, cfg1, r_dir, nodes);
        }
      } while (pushed_enough == false && outofbb == false);
    } else {   //Push to Obs
      //LOG_DEBUG_MSG("BasicOBPRM::GenerateNodes() -- Push to Obs");
      CFG r_dir;
      r_dir.GetRandomRay(min(_env->GetPositionRes(),
                         _env->GetOrientationRes()), _env, dm);
      bool pushed_enough = false;
      bool outofbb = false;
      do {
        
        CFG last_free(cfg1);
        cfg1.Increment(r_dir);
        if(!cfg1.InBoundingBox(_env) ) {outofbb=true;continue;} //out of bounding box
        CallCnt=" PushToObs"; 
        std::string tmpStr = Callee+CallCnt;
        pushed_enough = cfg1.isCollision(_env,Stats,cd,*this->cdInfo, true, &tmpStr);
        if(pushed_enough) {
          //nodes.push_back(CFG(last_free));
          //newNodes++;
          r_dir.multiply(r_dir,double(-1));
          ComputeCSpaceShells(_env, Stats, cd, dm, last_free, r_dir, nodes);
        }
      } while (pushed_enough == false && outofbb == false);
    }
    
    if (this->exactNodes)
      success_cntr = nodes.size();
    else
      success_cntr = attempts+1;
    //LOG_DEBUG_MSG("BasicOBPRM::GenerateNodes() -- success_cntr = " << success_cntr);
  } // endfor
} else {  //DO IT BALANCED!!!
  //m_balColl=0;
  //m_balFree=0;

  // generate in bounding box
  for (int attempts=0,newNodes=0,success_cntr=0;  success_cntr < this->numNodes ; attempts++) {
    //LOG_DEBUG_MSG("BasicOBPRM::GenerateNodes() attempts = " << attempts);
    CFG cfg1,last_free;
    cfg1.GetRandomCfg(_env);
    
    CallCnt=" 1st Sample"; 
    std::string tmpStr = Callee+CallCnt;
    
    bool cfg1_free = !cfg1.isCollision(_env,Stats,cd,*this->cdInfo, true, &tmpStr);
    if (!cfg1_free && (m_balColl <= m_balFree)) {  //push out of Obs
      ++m_balColl;
      //LOG_DEBUG_MSG("BasicOBPRM::GenerateNodes() -- Push Out of Obs");
      CFG r_dir;
      r_dir.GetRandomRay(min(_env->GetPositionRes(),
                         _env->GetOrientationRes()), _env, dm);
      bool pushed_enough = false;
      bool outofbb = false;
      do {
        //last_free.equals(cfg1);
        cfg1.Increment(r_dir);
        if(!cfg1.InBoundingBox(_env) ) {outofbb=true;continue;} //out of bounding box
        CallCnt=" PushOutOfObs"; 
        std::string tmpStr = Callee+CallCnt;
        ++m_balColl;
        pushed_enough = !cfg1.isCollision(_env,Stats,cd,*this->cdInfo, true, &tmpStr);
        if(pushed_enough) {
         // nodes.push_back(CFG(cfg1));
         // newNodes++;
         ComputeCSpaceShells(_env, Stats, cd, dm, cfg1, r_dir, nodes);
        }
      } while (pushed_enough == false && outofbb == false);
    } else if(cfg1_free && (m_balColl >= m_balFree)){   //Push to Obs
      ++m_balFree;
      //LOG_DEBUG_MSG("BasicOBPRM::GenerateNodes() -- Push to Obs");
      CFG r_dir;
      r_dir.GetRandomRay(min(_env->GetPositionRes(),
                         _env->GetOrientationRes()), _env, dm);
      bool pushed_enough = false;
      bool outofbb = false;
      do {
        
        CFG last_free(cfg1);
        cfg1.Increment(r_dir);
        if(!cfg1.InBoundingBox(_env) ) {outofbb=true;continue;} //out of bounding box
        CallCnt=" PushToObs"; 
        std::string tmpStr = Callee+CallCnt;
        ++m_balFree;
        pushed_enough = cfg1.isCollision(_env,Stats,cd,*this->cdInfo, true, &tmpStr);
        if(pushed_enough) {
          //nodes.push_back(CFG(last_free));
          //newNodes++;
          r_dir.multiply(r_dir,double(-1));
          ComputeCSpaceShells(_env, Stats, cd, dm, last_free, r_dir, nodes);
        }
      } while (pushed_enough == false && outofbb == false);
    }
    
    if (this->exactNodes)
      success_cntr = nodes.size();
    else
      success_cntr = attempts+1;
    //LOG_DEBUG_MSG("BasicOBPRM::GenerateNodes() -- success_cntr = " << success_cntr);
  } // endfor
}
#if INTERMEDIATE_FILES
  WritePathConfigurations("BridgeTestPRM.path", path, _env);
#endif

  LOG_DEBUG_MSG("~BasicOBPRM::GenerateNodes()"); 

};


///\todo this shell implementation is quick and dirty .. FIX after ICRA2006
template <class CFG>
void BasicOBPRM<CFG>::
ComputeCSpaceShells(Environment* _env, Stat_Class& Stats,
                    CollisionDetection* cd, 
                    DistanceMetric *dm, CFG& _freeCfg, CFG& _incr, vector<CFG>& nodes) {

  if(numShells < 1) {
    cout << "ERROR, SHELLS must be larger than 0" << endl; exit(-1);
  }
  if(numShells > 1) {
  std::string Callee(GetName()), CallCnt;
  {std::string Method("-BasicOBPRM::GenerateNodes"); Callee = Callee+Method;}
  CallCnt=" ComputeCSpaceShells"; 
  std::string tmpStr = Callee+CallCnt;
  for(int i=0; i<numShells; ++i) {
   if(!_freeCfg.isCollision(_env,Stats,cd,*this->cdInfo, true, &tmpStr) && _freeCfg.InBoundingBox(_env)) {
     nodes.push_back(CFG(_freeCfg));
   }
     _freeCfg.Increment(_incr);
  }
  } else { nodes.push_back(CFG(_freeCfg)); }
}
  



template <class CFG>
void
BasicOBPRM<CFG>::
GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG > &nodes) {
  
  Environment* _env = in_pRegion;
  Stat_Class& Stats = *(in_pRegion->GetStatClass());
  CollisionDetection* cd = this->GetMPProblem()->GetCollisionDetection();
  DistanceMetric* dm =  this->GetMPProblem()->GetDistanceMetric();
  
  GenerateNodes(_env,  Stats,  cd,  dm, nodes);
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
			       *this->cdInfo, true, &(Callee))) {
    
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
GenerateRandomDirection(Environment* env, DistanceMetric* dm, const CFG &InsideNode) {
  CFG randomDir;
  if (InsideNode.InBoundingBox(env))
    randomDir.GetRandomRay(EXPANSION_FACTOR * env->GetPositionRes(), env, dm);
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
  while(OutsideNode.isCollision(env, Stats, cd, rob, obst, *this->cdInfo, 
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
    if(mid.isCollision(env, Stats, cd , rob, obst, *this->cdInfo,
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
    if(!high.isCollision(env, Stats, cd, *this->cdInfo, true, &tmpStr)) {
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
    if(gen.isCollision(env, Stats, cd, *this->cdInfo,true, &(Callee)))
      obstSeeds.push_back(gen);
    else
      surface.push_back(gen);
  }
  
  vector<CFG> tmp, preshells, shells;
  for(i = 0; i < obstSeeds.size(); i++) {
    
    CFG incrCfg;
    incrCfg.GetRandomRay(EXPANSION_FACTOR*env->GetPositionRes(), env, dm);
    
    CFG OutsideNode =
      GenerateOutsideCfg(env,Stats,cd,robot,obstacle,obstSeeds[i],incrCfg);
    if(OutsideNode.AlmostEqual(obstSeeds[i])) continue; // can not find outside node.
    
    tmp =
      GenerateSurfaceCfg(env,Stats,cd,dm,robot,obstacle,obstSeeds[i],OutsideNode,clearanceFactor);
    
    // Choose as many as nshells
    preshells = Shells(tmp, numShells);
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
    opt = OBPRM_lrand() % 2;
    if(opt == 0){
      pt = body->GetCenterOfMass();
    } else {
      pt = ChooseRandomVertexOnBody(body, isFreeBody);
    }
    break;
    
  case rV_rT:
    opt = OBPRM_lrand() % 2;
    if(opt == 0){
      pt = ChooseRandomVertexOnBody(body, isFreeBody);
    } else {
      pt = ChooseRandomTriangleOnBody(body, isFreeBody);
    }
    break;
    
  case rV_rW:
    opt = OBPRM_lrand() % 2;
    if(opt == 0){
      pt = ChooseRandomVertexOnBody(body, isFreeBody);
    } else {
      pt = ChooseRandomWeightedTriangleOnBody(body, isFreeBody);
    }
    break;
    
  case all:
    opt = OBPRM_lrand() % 5;
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
  return polyhedron.vertexList[(int)(OBPRM_drand()*polyhedron.numVertices)];
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
  
  GMSPolygon *poly = &polyhedron.polygonList[(int)(OBPRM_drand()*polyhedron.numPolygons)];
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
  int index = OBPRM_lrand() % 6;
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
  
  double targetArea = area * OBPRM_drand();
  
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
  
  double s = OBPRM_drand(); double t = OBPRM_drand();
  while(s + t > 1){
    t = OBPRM_drand();
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
    if(!cfgs[i].isCollision(env, Stats, cd, *this->cdInfo,true,&(Callee))){
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


