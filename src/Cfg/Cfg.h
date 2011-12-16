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
#ifdef _PARALLEL
#include "views/proxy.h"
#endif 
#include <vector>
#include <map>

/////////////////////////////////////////////////////////////////////////////////////////
//Include mathtool vec
#include "Vector.h"

#include "boost/shared_ptr.hpp"
#include "boost/serialization/map.hpp"
using boost::shared_ptr;

class Cfg;
class Stat_Class;
class Environment;
class CollisionDetection;
class DistanceMetricMethod;
class CDInfo;
class MultiBody;
class MPProblem;


/////////////////////////////////////////////////////////////////////////////////////////////
//
/// Information about the clearance of a cfg.
//
/////////////////////////////////////////////////////////////////////////////////////////////
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
  double m_clearance;
  
  /**Direction of clearance, or "witness pair".
   *This Cfg is closest to the c-obst.
   */
  Cfg* m_direction;
    
  int m_obstacleId;

 public:
  
  /////////////////////////////////////////////////////////////////////////////
  //
  // Constructors and Destructor
  //
  /////////////////////////////////////////////////////////////////////////////  
  /**@name Constructors and Destructor*/
  //@{  
  ClearanceInfo(Cfg* _direction = NULL, double _clearance = -1e10) {
    m_clearance = _clearance;
    m_direction = _direction;
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
  double GetClearance() {return m_clearance;};
  void SetClearance(double _clearance) {m_clearance = _clearance;};
  
  Cfg* GetDirection() {return m_direction;};
  void SetDirection(Cfg* _direction) {m_direction = _direction;};

  int GetObstacleId() { return m_obstacleId;};
  void SetObstacleId(int _id) { m_obstacleId = _id;};
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

  virtual Cfg& operator=(const Cfg&);
    
  /**create a new Cfg instance whose configuration is weighted summation of the
   *first and the second Cfg.
   *The summation is done in every dimension in CSpace.
   *@param weight should between [0,1]. this weight is for thesecond Cfg. 
   * The weight for the first Cfg is (1-weight)
   */
  virtual void WeightedSum(const Cfg&, const Cfg&, double _weight = 0.5);       
  virtual bool IsWithinResolution(const Cfg& _c, double _positionRes, double _orientationRes) const;
    
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
  virtual void Write(ostream& _os) const;
  virtual void WriteInfo(ostream& _os) const;
  ///Read configuration from input stream
  virtual void Read(istream& _is);
  virtual void ReadInfo(istream& _is);
 
  virtual void PrintLinkConfigurations(Environment* _env, vector<Vector6D>& _cfigs) const;
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

  /// Return the number of degrees of freedom for the configuration class
  int DOF() const;
  int PosDOF() const;
  virtual const char* GetName() const = 0;
    
  /** Set a single parameter in the configuration (i.e., x,y,z,roll...)
   * @param param the parameter number to set.
   * @param value the value to set the parameter as
   */
  virtual int SetSingleParam(int _param, double _value);    
  /** Increment a single parameter in the configuration (i.e., x,y,z,roll...)
   * @param param the parameter number to set.
   * @param value the value to increment the parameter by.
   */
  virtual int IncSingleParam(int _param, double _value);  
  /** Retreive a single parameter in the configuration (i.e., x,y,z,roll...)
   * @param param the parameter number to retreive
   */
  double GetSingleParam(int _param) const;
    
  /// methods for Distance Metric.
  virtual vector<double> GetPosition() const;
  virtual vector<double> GetOrientation() const;  
  virtual double OrientationMagnitude() const;
  virtual double PositionMagnitude() const;
  //@}
  

  virtual bool ConfigEnvironment(Environment* _env) const = 0;

  /** 
   * tests whether or not robot in this configuration has every vertex inside
   * the environment specified bounding box
   */    
  bool InBoundingBox(Environment* _env) const;

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
  void GetResolutionCfg(Environment*);

  virtual void GetPositionOrientationFrom2Cfg(const Cfg&, const Cfg&);

  /// for rotate-at-s Local Planner. 
  virtual void GetMovingSequenceNodes(const Cfg& _other, vector<double> _sValue, vector<Cfg*>& _cfgs) const;

  virtual void GetMovingSequenceNodes(const Cfg& _other, double _s, vector<Cfg*>& _cfgs) const;

  /** pt1 & pt2 are two endpts of a line segment
   * find the closest point to the current cfg on that line segment
   * it could be one of the two endpoints of course
   */
  void ClosestPtOnLineSegment(const Cfg&, const Cfg&, const Cfg&);

  /// methods for nodes connection. 
  virtual void FindNeighbors(MPProblem* _mp, Environment* _env, Stat_Class& _stats,
           const Cfg& _increment,
           string _vcMethod,
           int _noNeighbors, 
           CDInfo& _cdInfo,
           vector<Cfg*>& _cfgs);
  /// methods for nodes connection. 
  virtual void FindNeighbors(MPProblem* _mp, Environment* _env, Stat_Class& _stats,
           const Cfg& _goal, const Cfg& _increment, 
           string _vcMethod,
           int _noNeighbors, 
           CDInfo& _cdInfo,
           vector<Cfg*>& _cfgs);
               
  ///Increase every value in this instance in each dimention by the value in _increment
  virtual void Increment(const Cfg& _increment);
  virtual void IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks, double _positionRes, double _orientationRes);
  virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks);
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
  virtual void GetRandomCfg(double _r, double _rStep) = 0;
  /** 
   * generates random confiasking Jory and Shawna to oversee/advise him on this.
guration where workspace robot's EVERY VERTEX
   * is guaranteed to lie within the environment specified bounding box
   * @param _maxTries Try _maxTries time to rondomly generate Cfg and check if
   * every vertex is in environment specified bounding box. If
   * no this cfg could be found. The program will be abort.
   */
  virtual void GetRandomCfg(Environment* _env, int _maxTries);

  /// ditto, but with a default number of tries (10).
  virtual void GetRandomCfg(Environment* _env);

  /// Generates a random configuration with approximate length
  virtual void GetRandomCfg(Environment* _env, shared_ptr<DistanceMetricMethod> _dm, double _length);

  virtual void GetRandomRay(double _incr, Environment* _env,  shared_ptr<DistanceMetricMethod> _dm) = 0;
  virtual void GetRandomRayPos(double _incr, Environment* _env);
  virtual double GetSmoothingValue(MPProblem* _mp, Environment *_env,Stat_Class& _stats,
	    string _vc, CDInfo& _cdInfo, string _dm, int _n, bool _bl );

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

  //@}

  virtual Cfg* CreateNewCfg() const;
  virtual Cfg* CreateNewCfg(vector<double>&) const;

  
 public:  
  /** Normalize the orientation to the some range.
   * call CfgManager's NormalizeOrientation.
   */
  virtual void NormalizeOrientation(int _index = -1);
     
  /**
   * generates random configuration where workspace robot's CENTER OF MASS
   * is guaranteed to lie within the environment specified bounding box
   * Call CfgManager::GetRandomCfg_CenterOfMass
   */
  virtual void GetRandomCfg_CenterOfMass(Environment *_env) = 0;

  static int  GetNumOfJoints();

  // setNumofJoints should be consistent in every class
  static void SetNumOfJoints(int _numOfJoints);

  bool GetLabel(string _label);
  bool IsLabel(string _label);
  void SetLabel(string _label,bool _value);

  double GetStat(string _stat);
  bool IsStat(string _stat);
  void SetStat(string _stat, double _value);

  //polygonal approximation
  vector<Vector3D> PolyApprox (Environment* _env) const;
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
 protected:
  
  std::vector<double> m_v;   
  int m_dof;
  int m_posDof;
/** @To do- there is still problem with (un)packing map
   *uncomment the ifdef after fix
   */
  #ifndef _PARALLEL
  std::map<string,bool> m_labelMap;
  std::map<string,double> m_statMap;
  #endif

 protected:
  static int m_numOfJoints;
  
  public:
//changed local to member
  #ifdef _PARALLEL
    void define_type(stapl::typer &_t)  
    {
      _t.member(m_v);
      _t.member(m_dof);
      _t.member(m_posDof);
    }
  #endif


}; // class Cfg


#endif
