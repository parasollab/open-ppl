/**@file Cfg.h
*
*General Description
* Configuration Data Class, it has all the interface needed
* by other Motion Planning classes. 
*
*@author  Guang Song
*@date 08/31/99
*/

////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Cfg_h
#define Cfg_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include standard headers
#ifdef HPUX
#include <sys/io.h>
#endif

#include <vector>
#include <map>


////////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "Vectors.h"
#include "BasicDefns.h"

#include "boost/shared_ptr.hpp"
using boost::shared_ptr;

class Stat_Class;

/////////////////////////////////////////////////////////////////////////////////////////////
//
/// Information about the clearance of a cfg.
//
/////////////////////////////////////////////////////////////////////////////////////////////
class Cfg;
class ClearanceInfo {

 private:
   
  /////////////////////////////////////////////////////////////////////////////
  //
  // Data
  //
  /////////////////////////////////////////////////////////////////////////////
  /**Clearance of this Cfg.
   * Closet distance from this Cfg to any Obstacle in C-Space
   */
  double clearance;
  
  /**Direction of clearance, or "witness pair".
   *This Cfg is closest to the c-obst.
   */
  Cfg* direction;
    
  int obstacle_id;

 public:
  
  /////////////////////////////////////////////////////////////////////////////
  //
  // Constructors and Destructor
  //
  /////////////////////////////////////////////////////////////////////////////  
  /**@name Constructors and Destructor*/
  //@{  
  ClearanceInfo(Cfg* _direction = NULL, double _clearance = -1e10) {
    clearance = _clearance;
    direction = _direction;
  }
  ~ClearanceInfo();
  //@}
  
  /////////////////////////////////////////////////////////////////////////////
  //
  // Access Methods
  //
  /////////////////////////////////////////////////////////////////////////////
  /**@name Access Method*/
  //@{
  double getClearance() {return clearance;};
  void setClearance(double _clearance) {clearance = _clearance;};
  
  Cfg* getDirection() {return direction;};
  void setDirection(Cfg* _direction) {direction = _direction;};

  int getObstacleId() { return obstacle_id;};
  void setObstacleId(int id) { obstacle_id = id;};
  //@}
    
};


/**
  *This class provides storage, tools, and operators for representing configuration.
  *
  *Client could constrcut Cfg instance in 3, 6 or more dimensions CSpace by choosing
  *relavant constructor. Comparasion, baisc operations (+ - * / ), or more advanced operations
  *(like weighted sum, AlmostEqual.. ) are also provided.
  *This class also provides input/output functions to read/write instances of this class
  *to many kinds of file (formats).
  *Morevoer, tools for create primitives of PRM, like random generation of Cfg for 
  *a given Workspace, and connections between Cfgs are also provided.
  *
  *However, most of methods for helping generation and connections call methods provide
  *by CfgManager or its offsprings, which are desinged for specific robots.
  *
  *@see CfgManager, Cfg_free, Cfg_free_serial, Cfg_fixed_tree, Cfg_fixed_PRR.
  */
class Body;
class Environment;
class CollisionDetection;
class DistanceMetric;
class DistanceMetricMethod;
class CDInfo;
class MultiBody;
class Cfg {
 public:
    
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Constructors and Destructor*/
  //@{
  Cfg() {};
  virtual ~Cfg() {};
  
  //@}
    
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Operator Ovverloading
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Operators*/
  //@{
    
  ///Return true if this and other Cfg instance have same configuration
  virtual bool operator== (const Cfg&) const;
  ///Return true if this and other Cfg instance have different configuration
  virtual bool operator!= (const Cfg&) const;
  virtual bool AlmostEqual(const Cfg& _c) const;

  virtual void add(const Cfg&, const Cfg&);
  virtual void subtract(const Cfg&, const Cfg&);
  virtual void negative(const Cfg&);
  virtual void multiply(const Cfg&, double);
  virtual void divide(const Cfg&, double);

  virtual void equals(const Cfg&) = 0;
    
  /**create a new Cfg instance whose configuration is weighted summation of the
   *first and the second Cfg.
   *The summation is done in every dimension in CSpace.
   *@param weight should between [0,1]. this weight is for thesecond Cfg. 
   * The weight for the first Cfg is (1-weight)
   */
  virtual void WeightedSum(const Cfg&, const Cfg&, double weight = 0.5);       
  virtual bool isWithinResolution(const Cfg &c, double positionRes, double orientationRes) const;
    
  //@}
  
  //===================================================================
  //  Other Methods
  //===================================================================
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name I/O*/
  //@{ 
  friend ostream& operator<< (ostream&, const Cfg&);
  friend istream& operator>> (istream&, Cfg&);
  ///Write configuration to output stream
  void Write(ostream& os) const;
  virtual void WriteInfo(ostream& os) const;
  ///Read configuration from input stream
  void Read(istream& is);
  virtual void ReadInfo(istream& is);
 
  virtual void printLinkConfigurations(Environment *env, vector<Vector6D> &cfigs) const;
  virtual void print_preamble_to_file(Environment *env, FILE *_fp, int numofCfg);
  //@}
    
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods : Retrive and set related information of this class
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods*/
  //@{
  ///Get internal storage of configuration
  const vector<double>& GetData() const;
  virtual Vector3D GetRobotCenterPosition() const = 0;
  virtual Vector3D GetRobotCenterofMass(Environment* env) const {
    return GetRobotCenterPosition();
  }
  /// Return the number of degrees of freedom for the configuration class
  int DOF() const;
  int posDOF() const;
  virtual const char* GetName() const = 0;
    
  /** Return the range of a single parameter of the configuration (i.e., range of x)
   * @param param the parameter to get the range for.
   *
   * @note In the future, this function should get the range for x,y,z by the bounding box
   * Currently it assumes the range for a position parameter to be -10000 to 10000
   * and the range for an orientation parameter to be 0 to 1, which should
   * also be changed to reflect any self collision in a linked robot
   */
  virtual pair<double,double> SingleParamRange(int param);  
  /** Set a single parameter in the configuration (i.e., x,y,z,roll...)
   * @param param the parameter number to set.
   * @param value the value to set the parameter as
   */
  int SetSingleParam(int param, double value);    
  /** Increment a single parameter in the configuration (i.e., x,y,z,roll...)
   * @param param the parameter number to set.
   * @param value the value to increment the parameter by.
   */
  int IncSingleParam(int param, double value);  
  /** Retreive a single parameter in the configuration (i.e., x,y,z,roll...)
   * @param param the parameter number to retreive
   */
  double GetSingleParam(int param) const;
    
  /// methods for Distance Metric.
  virtual vector<double> GetPosition() const;
  virtual vector<double> GetOrientation() const;  
  virtual double OrientationMagnitude() const;
  virtual double PositionMagnitude() const;
  //@}
  
  virtual void InvalidData();

  virtual bool ConfigEnvironment(Environment* env) const = 0;

  /** 
   * tests whether or not robot in this configuration has every vertex inside
   * the environment specified bounding box
   */    
  bool InBoundingBox(Environment* env) const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Connection Related Methods : These methods are related to local planner for connecting
  //    Cfgs
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Connection Related Methods*/
  //@{
  /**Return a Cfg whose cofiguration is d-resolution toward cfg2 from cfg1
   *in each dimension.
   */
  virtual void c1_towards_c2(const Cfg& cfg1, const Cfg& cfg2, double d);

  void GetResolutionCfg(Environment*);

  virtual void GetPositionOrientationFrom2Cfg(const Cfg&, const Cfg&);

  /// for rotate-at-s Local Planner. 
  virtual void GetMovingSequenceNodes(const Cfg& other, vector<double> s_value, vector<Cfg*>& cfgs) const;

 virtual void GetMovingSequenceNodes(const Cfg& other, double s, vector<Cfg*>& cfgs) const;

  /** pt1 & pt2 are two endpts of a line segment
   * find the closest point to the current cfg on that line segment
   * it could be one of the two endpoints of course
   */
  void ClosestPtOnLineSegment(const Cfg&, const Cfg&, const Cfg&);
               
  /// methods for nodes connection. 
  virtual void FindNeighbors(Environment* env, Stat_Class& Stats,
           const Cfg& increment,
           CollisionDetection*,
           int noNeighbors, 
           CDInfo& _cdInfo,
           vector<Cfg*>& cfgs);
  /// methods for nodes connection. 
  virtual void FindNeighbors(Environment* env, Stat_Class& Stats,
           const Cfg& goal, const Cfg& increment, 
           CollisionDetection*,
           int noNeighbors, 
           CDInfo& _cdInfo,
           vector<Cfg*>& cfgs);
    
  ///Increase every value in this instance in each dimention by the value in _increment
  virtual void Increment(const Cfg& _increment);
  virtual void IncrementTowardsGoal(const Cfg &goal, const Cfg &increment);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int* n_ticks, 
           double positionRes, double orientationRes);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int n_ticks);
  //@}
  
  
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Generation Related Methods : These methods are related to create Cfgs randomly
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Generation Related Methods*/
  //@{
  /** 
   * generates a random configuration without consideration of bounding box restrictions
   */
  virtual void GetRandomCfg(double R, double rStep) = 0;
  /** 
   * generates random configuration where workspace robot's EVERY VERTEX
   * is guaranteed to lie within the environment specified bounding box
   * @param maxTries Try maxTries time to rondomly generate Cfg and check if
   * every vertex is in environment specified bounding box. If
   * no this cfg could be found. The program will be abort.
   */
  virtual void GetRandomCfg(Environment* env, int maxTries);
  /// ditto, but with a default number of tries (10).
  virtual void GetRandomCfg(Environment* env);
  /// Generates a random configuration with approximate length
  /*
  virtual void BinarySearch(Environment* env, DistanceMetric* dm, double length,
			    const Cfg& low, const Cfg& high);
  */
  virtual void GetRandomCfg(Environment* env, DistanceMetric* _dm,
          double length);
  virtual void GetRandomRay(double incr, Environment* env, DistanceMetric* dm) = 0;
  virtual void GetRandomRay(double incr, Environment* env, DistanceMetricMethod* dm) = 0;

  /// generates random configuration that is in Free CSpace. 
  virtual void GetFreeRandomCfg(Environment* env, Stat_Class& Stats,
        CollisionDetection* cd, CDInfo& _cdInfo);
  /// generates N random configurations
  virtual void GetNFreeRandomCfgs(vector<Cfg*>& nodes, Environment* env,
          Stat_Class& Stats, CollisionDetection* cd,  
          CDInfo& _cdInfo, int num) const;

  /// generates random configuration and pushes it to the medial axis of the
  /// free c-space
  void GetMedialAxisCfg(Environment* _env, Stat_Class& Stats,
      CollisionDetection* _cd, CDInfo& _cdInfo, 
      DistanceMetric* _dm, 
      int clearnce_n, int penetration_n);
  /// pushes a node towards the medial axis
  void PushToMedialAxis(Environment* _env, Stat_Class& Stats,
      CollisionDetection* cd, CDInfo& cdInfo, 
      DistanceMetric* dm, 
      int clearance_n, int penetration_n);
  /// pushes a free node towards the medial axis
  virtual void MAPRMfree(Environment* _env, Stat_Class& Stats,
       CollisionDetection* cd, CDInfo& cdInfo, 
       DistanceMetric* dm, int n);
    

  virtual bool GenerateOverlapCfg(Environment* env, int robot,
          Vector3D robot_start, Vector3D robot_goal, 
          Cfg* resultCfg) = 0;  // OBPRM and BasicOBPRM
  virtual void GenSurfaceCfgs4ObstNORMAL(Environment* env, Stat_Class& Stats,
           CollisionDetection*,
           int obstacle, int nCfgs,
           CDInfo& _cdInfo,
           vector<Cfg*>& nodes) const = 0;
  //@}
    
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Helper Methods : Bounding Box, Clearance, Collision detection, Potential
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Helper Methods */
  //@{
  
  /**
   *Set Robot's Cfg as this Cfg instance and 
   *check Robot's clearance.
   *@see CollisionDetection::Clearance
   */
  double Clearance(Environment *env, Stat_Class& Stats, 
       CollisionDetection* cd) const;
  ///Approximate C-Space Clearance.
  /// returns clearance in c-space
  double ApproxCSpaceClearance(Environment* env, Stat_Class& Stats,
			       CollisionDetection* cd, CDInfo& cdInfo, 
			       DistanceMetric* dm, int n,
			       bool bComputePenetration,
			       int ignore_obstacle = -1) const;
  /// clearance and the direction set via ClearanceInfo
  void ApproxCSpaceClearance(Environment* env, Stat_Class& Stats,
			     CollisionDetection* cd, CDInfo& cdInfo,
			     DistanceMetric* dm, int n,
			     ClearanceInfo& clearInfo, 
			     bool bComputePenetration,
			     int ignore_obstacle = -1) const;

  ///Approximate C-Space Contact Points
  /// given an origin Cfg and a vector of directions
  /// returns the obstacle contact point for each direction from origin
  /// (contact points are in-collision)
  void ApproxCSpaceContactPoints(vector<Cfg*>& directions, Environment* _env,
         Stat_Class& Stats,
         CollisionDetection* cd, CDInfo &cdInfo,
         vector<Cfg*>& contact_points) const;    
  
  virtual bool isCollision(Environment* env, Stat_Class& Stats,
         CollisionDetection* cd, CDInfo& _cdInfo,
         bool enablePenetration=true, std::string *pCallName = NULL);
  virtual bool isCollision(Environment* env, Stat_Class& Stats,
         CollisionDetection* cd,
         int robot, int obs, 
         CDInfo& _cdInfo,
         bool enablePenetration=true, std::string *pCallName = NULL);
  virtual bool isCollision(Environment* env, Stat_Class& Stats,
         CollisionDetection* cd, CDInfo& _cdInfo,
         shared_ptr<MultiBody>, bool enablePenetration=true, std::string *pCallName = NULL);
    
  //@}

  virtual Cfg* CreateNewCfg() const = 0;
  virtual Cfg* CreateNewCfg(vector<double>&) const = 0;

  
 public:  
  /** Normalize the orientation to the some range.
   * call CfgManager's Normalize_orientation.
   */
  virtual void Normalize_orientation(int index = -1);
     
  /**
   * generates random configuration where workspace robot's CENTER OF MASS
   * is guaranteed to lie within the environment specified bounding box
   * Call CfgManager::GetRandomCfg_CenterOfMass
   */
  virtual void GetRandomCfg_CenterOfMass(Environment *env) = 0;

  static int  getNumofJoints();

  // setNumofJoints should be consistent in every class
  static void setNumofJoints(int _numofjoints);

  bool GetLabel(string in_strLabel);
  bool IsLabel(string in_strLabel);
  void SetLabel(string in_strLabel,bool in_bool);

  double GetStat(string in_strStat);
  bool IsStat(string in_strStat);
  void SetStat(string in_strStat,double in_dstat);
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
 protected:
  
  std::vector<double> v;   
  int dof;
  int posDof;

  std::map<string,bool> m_LabelMap;
  std::map<string,double> m_StatMap;
  
 public:
  //Info:
  /**From which Obstacle this Cfg is generated.
   *@see GenerateMapNodes::GausePRM, GenerateMapNodes::BasicOBPRM
   *GenerateMapNodes::OBPRM
   */
  int obst;

  double tag;

  /**Clearance of this Cfg.
   *Closest distance from this Cfg to any Obstacle in Environment
   */
  double clearance;

 protected:
  static int NumofJoints;

}; // class Cfg


#endif
