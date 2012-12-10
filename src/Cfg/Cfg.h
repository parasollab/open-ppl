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

#ifndef CFG_H_
#define CFG_H_

#ifdef _PARALLEL
#include "views/proxy.h"
#endif 
#include <vector>
#include <map>

#include "boost/shared_ptr.hpp"
#include "boost/serialization/map.hpp"
using boost::shared_ptr;

#include "Vector.h"

#include "MPProblem/Robot.h"
#include "Utilities/MPUtils.h"

class Cfg;
class StatClass;
class Environment;
class CDInfo;
class MultiBody;
//class MPProblem;
class BoundingBox;
class Boundary;
class ClearanceParams;

/////////////////////////////////////////////////////////////////////////////////////////////
//
/// Information about the clearance of a cfg.
//
/////////////////////////////////////////////////////////////////////////////////////////////
class ClearanceInfo {
  private:
    double m_clearance; // closest distance from this Cfg to any Obstacle in C-space
    Cfg* m_direction; //direct to Cfg that is closest to c-obst
    int m_obstacleId;

  public:

    ClearanceInfo(Cfg* _direction = NULL, double _clearance = -1e10) {
      m_clearance = _clearance;
      m_direction = _direction;
    }
    ~ClearanceInfo();

    double GetClearance() {return m_clearance;};
    void SetClearance(double _clearance) {m_clearance = _clearance;};

    Cfg* GetDirection() {return m_direction;};
    void SetDirection(Cfg* _direction) {m_direction = _direction;};

    int GetObstacleId() { return m_obstacleId;};
    void SetObstacleId(int _id) { m_obstacleId = _id;};
};


/**
 *This class provides storage, tools, and operators for representing configuration.
 *
 *Comparasion, baisc operations (+ - * / ), or more advanced operations
 *(like weighted sum, AlmostEqual.. ) are also provided.
 *This class also provides input/output functions to read/write instances of this class
 *to many kinds of file (formats).
 *Morevoer, tools for create primitives of PRM, like random generation of Cfg for 
 *a given Workspace, and connections between Cfgs are also provided.
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

    /**@name Copy Constructor*/

    Cfg(const Cfg& _other);

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Operator Ovverloading
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /*@name Operators*/
    //@{

    ///Return true if this and other Cfg instance have same configuration
    virtual bool operator== (const Cfg&) const;
    ///Return true if this and other Cfg instance have different configuration
    virtual bool operator!= (const Cfg&) const;
    virtual bool AlmostEqual(const Cfg& _c) const;
    virtual void add(const Cfg&, const Cfg&);
    virtual void subtract(const Cfg&, const Cfg&);
    virtual void negative(const Cfg&);
    virtual void multiply(const Cfg&, double, bool _norm=true);
    virtual void divide(const Cfg&, double);

    static void InitRobots(vector<Robot>& _robots);
    virtual vector<Robot> GetRobots(int) = 0;
    virtual Cfg& operator=(const Cfg&);

    /**create a new Cfg instance whose configuration is weighted summation of the
     *first and the second Cfg.
     *The summation is done in every dimension in CSpace.
     *@param weight should between [0,1]. this weight is for the second Cfg. 
     * The weight for the first Cfg is (1-weight)
     */
    virtual void WeightedSum(const Cfg&, const Cfg&, double _weight = 0.5);       
    virtual bool IsWithinResolution(const Cfg& _c, double _positionRes, double _orientationRes) const;

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
    friend ostream& operator<< (ostream&, const Cfg&);
    friend istream& operator>> (istream&, Cfg&);
    ///Write configuration to output stream
    virtual void Write(ostream& _os) const;
    ///Read configuration from input stream
    virtual void Read(istream& _is);

    virtual void PrintLinkConfigurations(Environment* _env, vector<Vector6D>& _cfigs) const;

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
    void SetData(vector<double>& _data);
    virtual Vector3D GetRobotCenterPosition() const = 0;
     virtual Vector3D GetRobotCenterofMass(Environment* _env) const=0;
    /// Return the number of degrees of freedom for the configuration class
    static size_t DOF();
    static size_t PosDOF();
    virtual const string GetName() const = 0;
    static size_t GetNumOfJoints() { return 0; }

    /** Set a single parameter in the configuration (i.e., x,y,z,roll...)
     * @param param the parameter number to set.
     * @param value the value to set the parameter as
     */
    virtual int SetSingleParam(size_t _param, double _value, bool _norm=true);    
    /** Increment a single parameter in the configuration (i.e., x,y,z,roll...)
     * @param param the parameter number to set.
     * @param value the value to increment the parameter by.
     */
    virtual int IncSingleParam(size_t _param, double _value);  
    /** Retreive a single parameter in the configuration (i.e., x,y,z,roll...)
     * @param param the parameter number to retreive
     */
    double GetSingleParam(size_t _param) const;

    /// methods for Distance Metric.
    virtual vector<double> GetPosition() const;
    virtual vector<double> GetOrientation() const;  
    virtual double OrientationMagnitude() const;
    virtual double PositionMagnitude() const;

    bool InBoundary(Environment* _env) const;
    bool InBoundary(Environment* _env, shared_ptr<Boundary> _bb) const;

    ///Increase every value in this instance in each dimention by the value in _increment
    virtual void Increment(const Cfg& _increment);
    virtual void IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment);
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks, double _positionRes, double _orientationRes);
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks);



    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Generation Related Methods : These methods are related to create Cfgs randomly
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /** 
     * Confguration where workspace robot's EVERY VERTEX
     * is guaranteed to lie within the environment specified bounding box
     * @param _maxTries Try _maxTries times to randomly generate Cfg and check if
     * every vertex is in environment-specified bounding box. If
     * not, a cfg couldn't be found in the bbx, and the program will abort.
     */
    virtual void GetRandomCfg(Environment* _env);
    virtual void GetRandomCfg(Environment* _env, shared_ptr<Boundary> _bb);
    virtual void GetRandomCfgCenterOfMass(Environment *_env, shared_ptr<Boundary> bb) = 0;
    
    template<class DistanceMetricPointer>
      void GetRandomRay(double _incr, Environment* _env,  DistanceMetricPointer _dm, bool _norm=true);

    template<template<class> class ClearanceUtility, class MPTraits>
      double GetSmoothingValue(ClearanceUtility<MPTraits>& _clearanceUtils, shared_ptr<Boundary> _bb);

    virtual bool ConfigEnvironment(Environment* _env) const = 0;

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

    /** pt1 & pt2 are two endpts of a line segment
     * find the closest point to the current cfg on that line segment
     * it could be one of the two endpoints of course
     */
    void ClosestPtOnLineSegment(const Cfg&, const Cfg&, const Cfg&);

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

    virtual Cfg* CreateNewCfg() const = 0;
    Cfg* CreateNewCfg(vector<double>&) const;

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
    /** generates random configuration where workspace robot's CENTER OF MASS
     * is guaranteed to lie within the environment specified bounding box
     */
    /** Normalize the orientation to the some range.
     * call CfgManager's NormalizeOrientation.
     */
    virtual void NormalizeOrientation(int _index = -1);

    vector<double> m_v;   
    static size_t m_dof;

    enum DofType {POS, ROT, JOINT};
    static vector<DofType> m_dofTypes;
    static vector<Robot> m_robots;

    /** TODO- there may still problem with (un)packing map
     */
    map<string,bool> m_labelMap;
    map<string,double> m_statMap;

  public:
#ifdef _PARALLEL
    void define_type(stapl::typer &_t)  
    {
      _t.member(m_v);
      _t.member(m_dof);
      _t.member(m_dofTypes);
      _t.member(m_robots);
      _t.member(m_labelMap);
      _t.member(m_statMap);
    }
#endif

}; // class Cfg

template<class DistanceMetricPointer>
void
Cfg::GetRandomRay(double _incr, Environment* _env,  DistanceMetricPointer _dm, bool _norm){
  cerr << "Error::Get Random Ray not implemented in Cfg derived class, exiting." << endl;
  exit(1);
}

template<template<class> class ClearanceUtility, class MPTraits>
double
Cfg::GetSmoothingValue(ClearanceUtility<MPTraits>& _clearanceUtils, shared_ptr<Boundary> _bb){
  CDInfo cdInfo;
  typename MPTraits::CfgType tmp;
  _clearanceUtils.CollisionInfo(*((typename MPTraits::CfgType*)this), tmp, _bb, cdInfo);
  return cdInfo.m_minDist;
}

#ifdef _PARALLEL
namespace stapl {
  template <typename Accessor>
    class proxy<Cfg, Accessor> 
    : public Accessor {
      private:
        friend class proxy_core_access;
        typedef Cfg target_t;

      public:
        //typedef target_t::parameter_type  parameter_type;
        explicit proxy(Accessor const& acc) : Accessor(acc) { }
        //operator target_t() const { return Accessor::read(); }
        proxy const& operator=(proxy const& rhs) { Accessor::write(rhs); return *this; }
        proxy const& operator=(target_t const& rhs) { Accessor::write(rhs); return *this;}
        int DOF() const { return Accessor::const_invoke(&target_t::DOF);}
        int PosDOF() const { return Accessor::const_invoke(&target_t::PosDOF);}
        void Write(ostream& _os) const { return Accessor::const_invoke(&target_t::Write, _os);}
        void Read(istream& _is){ return Accessor::const_invoke(&target_t::Read, _is);}
        const vector<double>& GetData() const { return Accessor::const_invoke(&target_t::GetData);}
        void SetData(vector<double>& _data) const { return Accessor::const_invoke(&target_t::SetData, _data);}
        bool GetLabel(string _label) const { return Accessor::const_invoke(&target_t::GetLabel, _label);}
        bool IsLabel(string _label) const { return Accessor::const_invoke(&target_t::IsLabel, _label);}
        bool SetLabel(string _label) const { return Accessor::const_invoke(&target_t::SetLabel, _label);}
        bool GetStat(string _stat) const { return Accessor::const_invoke(&target_t::GetStat, _stat);}
        bool IsStat(string _stat) const { return Accessor::const_invoke(&target_t::IsStat, _stat);}
        bool SetStat(string _stat) const { return Accessor::const_invoke(&target_t::SetStat, _stat);}
        static int GetNumOfJoints()  { return Accessor::const_invoke(&target_t::GetNumOfJoints);}
        // static void SetNumOfJoints(int _numOfJoints)  { return Accessor::const_invoke(&target_t::SetNumOfJoints, _numOfJoints);}
    }; //struct proxy
}
#endif

#endif
