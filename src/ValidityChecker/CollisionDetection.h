////////////////////////////////////////////////////////////////////////////////////////////
/**@file CollisionDetection.h

  @author Daniel Vallejo
  @date   8/11/98
*/   
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CollisionDetection_h
#define CollisionDetection_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include standard headers

///////////////////////////////////////////////////////////////////////////////
///CD libraries
#ifdef USE_VCLIP
#include <vclip.h>
#endif
#ifdef USE_RAPID
#include <RAPID.H>
#endif
#ifdef USE_PQP
#include <PQP.h>
#endif
#ifdef USE_SOLID
#include <SOLID.h>
#endif

//////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "DistanceMetricMethod.h"
#include "Cfg.h"
#include "util.h"

//////////////////////////////////////////////////////////////////////////////

class Environment;
class MultiBody;
class Transformation;
class CollisionDetectionMethod;

///////////////////////////////////////////////////////////////////////////////

const double MaxDist =  1e10;

/**
*Algo base information data structures.
*
*This was made into a class so I knew everything was
*initialized properly. I got tired of trying to track
*down where all the CDInfo variables were created - BD July 2000
*@author BD
*@date 07/01/00
*/
class CDInfo {
	
 public:
  
  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //    Constructors and Destructor
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Constructors and Destructor*/
  //@{
  
  ///Default Constrcutor. Intialize evry thing to invalid value.
  CDInfo();
  ///Destructor. Do nothing.
  ~CDInfo();
  
  //@}
  
  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //    Helper
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Helper Methods*/
  //@{
	
  ///Re-Init vars as done by constructor
  void ResetVars();
  //@}
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //    Data
  //
	//////////////////////////////////////////////////////////////////////////////////////////
  
  int colliding_obst_index;   ///< The index for fisrt discovered obstacle which collides with robot.
  bool ret_all_info;          ///< Is this instance contains all (following) infomation.
  int nearest_obst_index;     ///< The index for closest obstacle
  double min_dist;            ///< Distance between Robot and closet obstacle
  Vector3D robot_point;       ///< Cloest point on Robot to closet obstacle
  Vector3D object_point;      ///< Cloest point on closet obstacle to Robot
};


const int Out = 0;      ///<Type Out: no collision sure; collision unsure.
const int In = 1;       ///<Type In: no collision unsure; collision sure.
const int Exact = 2;    ///<Type Exact: no collision sure; collision sure.


class CollisionDetection : MPBaseObject {
 public:
  CollisionDetection();
  CollisionDetection(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  CollisionDetection(vector<CollisionDetectionMethod*>& _selected);
  virtual ~CollisionDetection();
  
  static vector<CollisionDetectionMethod*> GetDefault();
  
  void PrintUsage(ostream& _os) const;
  void PrintValues(ostream& _os) const;
  void PrintDefaults(ostream& _os) const;
  void PrintOptions(ostream& out_os);

  /**Ouput information about all CD instances to file.
   *@param _fname filename for data file.
   *@see WriteCDs(ostream& _myostream)
   */
  ///void WriteCDs(const char* _fname) const;
  
  /**Ouput information about all CD instances to ouputstream.
   *@note format: CD_NAME (a string) \n for each CD instance.
   *@see GetCDs
   */
  //void WriteCDs(ostream& _myostream) const;
  
  /**Read information about CD instances from file.
   *@param _fname filename for data file.
   *@see ReadCDs(istream& _myistream)
   */
  //void ReadCDs(const char* _fname);
  
  /**Read information about CD instances from input stream.
   *This methods read and add CD instances to universe.
   *@see AddCD for add CD instances to universe and WriteCDs 
   *for data format
   *@note if this method could not be able to understand
   *input file format, then error message will be post to 
   *standard output.
   */
  //void ReadCDs(istream& _myistream);

  /**Check collision for Robot with all Obstacles in Environment.
   *
   *@param lineRobot If this parameter is not NULL, then
   *this MultiBody will be used as robot. Otherwise, robot's
   *MultiBody will be retrived from Environment.
   *
   *@return true if Robot collides with Obstacle(s).
   *if returned value is true, post condition of _cdInfo depends on
   *_cdInfo.ret_all_info :
   *	-# if _cdInfo.ret_all_info is false, then _cdInfo.colliding_obst_index 
   *      is the index for the first obstacle colliding with robot.
   *	-# if _cdInfo.ret_all_info is true, then _cdInfo.colliding_obst_index
   *      is the index for the first obstacle colliding with robot, and 
   *      local_cd_info.nearest_obst_index is the index for cloeset Obstacle.
   *@note if self collision of Robot is found, _cdInfo will be set to odd value.
   *@see IsInCollision(Environment* , SID , CDInfo& , MultiBody* , MultiBody*)
   */
  bool IsInCollision(Environment* env, Stat_Class& Stats, CDInfo& _cdInfo, 
		     shared_ptr<MultiBody> lineRobot = shared_ptr<MultiBody>(), bool enablePenetration=true, std::string *pCallName=NULL);

  /**Check collision by index of robot and obstacle.
   *This method retrives MultiBody instances from Environment insntace,
   *then call IsInCollision(Environment* , SID , CDInfo& , MultiBody* , MultiBody* )
   *to detect collision.
   *@see IsInCollision(Environment* , SID , CDInfo& , MultiBody* , MultiBody*)
   */
  bool IsInCollision(Environment* env, Stat_Class& Stats, CDInfo& _cdInfo, 
		     int robot, int obstacle, std::string *pCallName=NULL);

  /**Check collision between MultiBody of robot and obstacle.
   *This method using collision detection information in _cdInfo to
   *check collision.
   *@return Following rules are used to determin collision:
   *
   *   -# if CDInfo::GetType = Out and CDInfo::GetCollisionDetection returns false
   *      then return false.
   *   -# if CDInfo::GetType = In and CDInfo::GetCollisionDetection returns true
   *      then return true.
   *   -# if CDInfo::GetType = Exact then return what CDInfo::GetCollisionDetection
   *      returned.
   *
   *@see Collision detection core functions, CDInfo::GetType, and 
   *CDInfo::GetCollisionDetection
   */
  bool IsInCollision(Environment* env, Stat_Class& Stats, CDInfo& _cdInfo, 
		     shared_ptr<MultiBody> rob, shared_ptr<MultiBody> obstacle, std::string *pCallName=NULL);
  
  bool isInsideObstacle(const Cfg& cfg, Environment* env, CDInfo& _cdInfo);

  /**Set penetration depth  
   * The parameter depth defines how many times the resolution 
   * Default value is -1, no penetration
   */
  void SetPenetration(double times);

  /** Check if there is a collision but it is in permissible range,
   * i.e., the penetration is within the penetration range
   */
  bool AcceptablePenetration(Cfg& c, Environment* env, Stat_Class& Stats,
			     CDInfo& cdInfo);
  
  /** Initialize n direction vectors  with the penetration length*/
  template <class CFG>
  void InitializePenetration(double times, int nCfgs,  Environment* env,
			     DistanceMetricMethod* dm, double ratio=0.5);
  
  double penetration; // Penetration distance  

  vector<cd_predefined> GetAllCDTypes() const;
  vector<cd_predefined> GetSelectedCDTypes() const;
  CollisionDetectionMethod* GetRAPID();
  CollisionDetectionMethod* GetPQP();
  CollisionDetectionMethod* GetVCLIP();
  CollisionDetectionMethod* GetSOLID();
 protected:
  bool ParseCommandLine(int argc, char** argv);

  vector<CollisionDetectionMethod*> all;
  vector<CollisionDetectionMethod*> selected;

  vector<Cfg*> directions;
  double acceptableRatio;
  std::string vcMethod;
};


class CollisionDetectionMethod {
 public:
  CollisionDetectionMethod();
  virtual ~CollisionDetectionMethod();

  string GetName() const {return name;}
  virtual void SetDefault();

  virtual bool operator==(const CollisionDetectionMethod& cd) const;

  int GetType();

  virtual void ParseCommandLine(int argc, char** argv);
  virtual void PrintUsage(ostream& _os) const;
  virtual void PrintValues(ostream& _os) const;
  virtual void PrintOptions(ostream& _os) const;
  virtual CollisionDetectionMethod* CreateCopy() = 0;

  /**
   * Check if robot in given cfg is complete inside or outside obstacle.
   * @warning The precondition is that robot is collision free
   * in this given cfg. (i.e no intersections among boundaries of robot and obs)
   * @return True, if robot is completely contained inside any obs.
   * otherwise, false will be returned.
   */
  virtual bool isInsideObstacle(const Cfg& cfg, Environment* env, CDInfo& _cdInfo);
  //@}
  
  /**Check collision between MultiBody of robot and obstacle.
   */
  virtual bool IsInCollision(shared_ptr<MultiBody> rob, shared_ptr<MultiBody> obstacle, Stat_Class& Stats, CDInfo& _cdInfo, std::string *pCallName=NULL, int ignore_i_adjacent_multibodies=1) = 0;

  CDInfo cdInfo;            ///<No one use this??!!

  cd_predefined GetCDType() const { return cdtype; }

 protected:
  int type; ///<Out, In, or Exact. Used to classify CD functions.
  cd_predefined cdtype;
  string name;
};


#ifdef USE_VCLIP
class Vclip : public CollisionDetectionMethod {
 public:

  Vclip();
  virtual ~Vclip();

  virtual CollisionDetectionMethod* CreateCopy();
  
  /**Using VCLIP to check collision between two MultiBodys.
   *Collision is checked in Body level between two MultiBodys,
   *if any of Body from Robot collides with any of Body from obstacle,
   *true will be returned.
   *
   *@note This method doesn't support "Return all info", if 
   *_cdInfo.ret_all_info is true, then error message will be post.
   *@note collision between two ajacent links will be ignore.
   *@return true if Collision found. Otherwise false will be returned.
   *
   *@see Body::GetVclipBody, and GetVclipPose.
   *@see IsInColl_AllInfo_vclip for get all info. 
   */
  virtual bool IsInCollision(shared_ptr<MultiBody> robot, shared_ptr<MultiBody> obstacle, 
			     Stat_Class& Stats, CDInfo& _cdInfo, std::string *pCallName=NULL, int ignore_i_adjacent_multibodies=1);
  
  /**Get VclipPose.
   *@todo I don't really know what this is....
   */
  VclipPose GetVclipPose(const Transformation&, const Transformation&);

  
  /**Get all collsion information for given MultiBody.
   *Collision is checked in Body level between two MultiBodys,
   *if any of Body from Robot collides with any of Body from obstacle,
   *true will be returned.
   *
   *More information about collision between two object, such as the closet points between
   *two object, closest distance... all of these information are stored in _cdInfo.
   *
   *@note each obstacle could change the results in _cdInfo
   *Trace back to general IsInCollision call to see how it all
   *gets updated correctly.
   *@see IsInCollision(Environment*, SID, CDInfo& , MultiBody*)
   */
  bool IsInColl_AllInfo_vclip(shared_ptr<MultiBody> robot, shared_ptr<MultiBody> obstacle, 
			      CDInfo& _cdInfo, int ignore_i_adjacent_multibodies=1);
};
#endif


#ifdef USE_RAPID
class Rapid: public CollisionDetectionMethod {
 public:

  Rapid();
  virtual ~Rapid();

  virtual CollisionDetectionMethod* CreateCopy();
  
  /**Using RAPID to check collision between two MultiBodys.
   *Collision is checked in Body level between two MultiBodys,
   *if any of Body from Robot collides with any of Body from obstacle,
   *true will be returned.
   *
   *@note This method doesn't support "Return all info", if 
   *_cdInfo.ret_all_info is true, then error message will be post.
   *@note if RAPID_Collide, the RAPID method, return false, process will 
   *be terminated.
   *@note collision between two ajacent links will be ignore.
   *@return true if Collision found. Otherwise false will be returned.
   *@see Body::GetRapidBody
   */
  virtual bool IsInCollision(shared_ptr<MultiBody> robot, shared_ptr<MultiBody> obstacle, 
			     Stat_Class& Stats, CDInfo& _cdInfo, std::string *pCallName=NULL, int ignore_i_adjacent_multibodies=1);
};
#endif


#ifdef USE_PQP
class Pqp : public CollisionDetectionMethod {
 public:
  Pqp();
  virtual ~Pqp();

  virtual CollisionDetectionMethod* CreateCopy();

  virtual bool IsInCollision(shared_ptr<MultiBody> robot, shared_ptr<MultiBody> obstacle, 
			     Stat_Class& Stats, CDInfo& _cdInfo,std::string *pCallName=NULL, int ignore_i_adjacent_multibodies=1);
};

class Pqp_Solid : public Pqp {
 public:
  Pqp_Solid() : Pqp() {name = "PQP_solid";}
  virtual ~Pqp_Solid() {}
  virtual CollisionDetectionMethod* CreateCopy();
  virtual bool IsInCollision(shared_ptr<MultiBody> robot, shared_ptr<MultiBody> obstacle,
			     Stat_Class& Stats, CDInfo& _cdInfo,std::string *pCallName=NULL, int ignore_i_adjacent_multibodies=1);
  virtual bool isInsideObstacle(const Cfg& cfg, Environment* env);
  virtual bool isInsideObstacle(Vector3D robot_pt, shared_ptr<MultiBody> obstacle);
  PQP_Model* BuildPQPSegment(PQP_REAL dX, PQP_REAL dY, PQP_REAL dZ) const;
};
#endif
  

#ifdef USE_SOLID
class Solid : public CollisionDetectionMethod {
 public:
  Solid();
  virtual ~Solid();

  virtual CollisionDetectionMethod* CreateCopy();

  virtual bool IsInCollision(shared_ptr<MultiBody> robot, shared_ptr<MultiBody> obstacle, 
                             Stat_Class& Stats, CDInfo& _cdInfo,std::string *pCallName=NULL, int ignore_i_adjacent_multibodies=1);




};
#endif




class BoundingSpheres : public CollisionDetectionMethod {
 public:

  BoundingSpheres();
  virtual ~BoundingSpheres();

  virtual CollisionDetectionMethod* CreateCopy();
  
  virtual bool IsInCollision(shared_ptr<MultiBody> robot, shared_ptr<MultiBody> obstacle, 
			     Stat_Class& Stats, CDInfo& _cdInfo, std::string *pCallName=NULL,  int ignore_i_adjacent_multibodies=1);
};


class InsideSpheres : public CollisionDetectionMethod {
 public:

  InsideSpheres();
  virtual ~InsideSpheres();

  virtual CollisionDetectionMethod* CreateCopy();

  virtual bool IsInCollision(shared_ptr<MultiBody> robot, shared_ptr<MultiBody> obstacle, 
			     Stat_Class& Stats, CDInfo& _cdInfo, std::string *pCallName=NULL,  int ignore_i_adjacent_multibodies=1);
};


class Naive : public CollisionDetectionMethod {
 public:

  Naive();
  virtual ~Naive();

  virtual CollisionDetectionMethod* CreateCopy();

  virtual bool IsInCollision(shared_ptr<MultiBody> robot, shared_ptr<MultiBody> obstacle, 
			     Stat_Class& Stats, CDInfo& _cdInfo,std::string *pCallName=NULL,  int ignore_i_adjacent_multibodies=1);
};


class Quinlan : public CollisionDetectionMethod {
 public:

  Quinlan();
  virtual ~Quinlan();

  virtual CollisionDetectionMethod* CreateCopy();

  virtual bool IsInCollision(shared_ptr<MultiBody> robot, shared_ptr<MultiBody> obstacle, 
                             Stat_Class& Stats, CDInfo& _cdInfo, std::string *pCallName=NULL,  int ignore_i_adjacent_multibodies=1);
};


//////////////////////////////////////////////////////////////////////////
// InitializePenetration
//
// Set the penetrationdepth and find n direction vectors
//
//
//////////////////////////////////////////////////////////////////////////
template <class CFG>
void 
CollisionDetection::
InitializePenetration(double times, int nCfgs, Environment* env,
		      DistanceMetricMethod* dm, double ratio) {
  CFG origin;
  acceptableRatio = ratio; 
  // first find the environment resolution
  CFG res;
  res.GetResolutionCfg(env);
  
  // now find the length of that resolution
  double length = dm->Distance(env, res, origin);
  
  // penetration is length*times
  penetration = times*length;
  
  cout << "Penetration is " << penetration << endl;
  
  CFG* tmp;
  for(int i=0; i < nCfgs; i++) {
    tmp = new CFG;
    tmp->GetRandomCfg(env, dm, penetration*OBPRM_drand());
    directions.push_back(tmp);
    cout <<"Added Cfg\n"<<flush;
  }
};

#endif
