/**@file Cfg.h
 *
 *General Description
 * Configuration Data Class, it has all the interface needed
 * by other Motion Planning classes. 
 */

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
#include "ValidityCheckers/CollisionDetection/CDInfo.h"

class Cfg;
class Environment;
class Boundary;

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
 *Comparison, basic operations (+ - * / ), or more advanced operations
 *(like weighted sum, AlmostEqual.. ) are also provided.
 *This class also provides input/output functions to read/write instances of this class
 *to many kinds of file (formats).
 *Morevoer, tools for create primitives of PRM, like random generation of Cfg for 
 *a given Workspace, and connections between Cfgs are also provided.
 */
class Cfg {
  public:

    Cfg();
    Cfg(const Cfg& _other);
    virtual ~Cfg() {};

    static void InitRobots(vector<Robot>& _robots, ostream& _os=std::cout);

    Cfg& operator=(const Cfg& _cfg);
    ///determines equality of this and other configuration
    bool operator== (const Cfg& _cfg) const;
    ///determines non-equality of this and other configuration
    bool operator!= (const Cfg& _cfg) const;
    //addition
    Cfg operator+(const Cfg& _cfg) const;
    Cfg& operator+=(const Cfg& _cfg);
    //subtraction
    Cfg operator-(const Cfg& _cfg) const;
    Cfg& operator-=(const Cfg& _cfg);
    //negate
    Cfg operator-() const;
    //scalar multiply
    Cfg operator*(double _d) const;
    Cfg& operator*=(double _d);
    //scalar divide
    Cfg operator/(double _d) const;
    Cfg& operator/=(double _d);
    //access dof values
    double& operator[](size_t _dof);
    const double& operator[](size_t _dof) const;


    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Access Methods : Retrieve and set related information of this class
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    ///Get internal storage of configuration
    const vector<double>& GetData() const {return m_v;};
    void SetData(const vector<double>& _data);

    //labeling of the Cfg and statistics
    bool GetLabel(string _label);
    bool IsLabel(string _label);
    void SetLabel(string _label,bool _value);

    double GetStat(string _stat);
    bool IsStat(string _stat);
    void SetStat(string _stat, double _value = 0.0);
    void IncStat(string _stat, double _value = 1.0);

    /// Return the number of degrees of freedom for the configuration class
    static size_t DOF() {return m_dof;};
    static size_t PosDOF() {return m_posdof;}
    static size_t GetNumOfJoints() {return m_numJoints;}
    virtual const string GetName() const {return "Cfg";};

    /// methods for Distance Metric.
    virtual vector<double> GetPosition() const;
    virtual vector<double> GetOrientation() const;  
    virtual double Magnitude() const;
    virtual double PositionMagnitude() const;
    virtual double OrientationMagnitude() const;

    //Calculate the center position and center of mass of the robot configures
    //at this Cfg
    virtual Vector3d GetRobotCenterPosition() const;
    virtual Vector3d GetRobotCenterofMass(Environment* _env) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Generation Related Methods : These methods are related to create Cfgs randomly
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /** 
     * Configuration where workspace robot's EVERY VERTEX
     * is guaranteed to lie within the environment specified bounding box If
     * not, a cfg couldn't be found in the bbx, and the program will abort.
     * The function will try a predefined number of times
     */
    virtual void GetRandomCfg(Environment* _env);
    virtual void GetRandomCfg(Environment* _env, shared_ptr<Boundary> _bb);
    
    template<class DistanceMetricPointer>
      void GetRandomRay(double _incr, Environment* _env,  DistanceMetricPointer _dm, bool _norm=true);

    virtual bool ConfigEnvironment(Environment* _env) const;

    void GetResolutionCfg(Environment*);

    ///Increase every value in this instance in each dimention by the value in _increment
    virtual void IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment);
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks, double _positionRes, double _orientationRes);
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks);

    /**create a new Cfg instance whose configuration is weighted summation of the
     *first and the second Cfg.
     *The summation is done in every dimension in CSpace.
     *@param weight should between [0,1]. this weight is for the second Cfg. 
     * The weight for the first Cfg is (1-weight)
     */
    virtual void WeightedSum(const Cfg&, const Cfg&, double _weight = 0.5);       

    virtual void GetPositionOrientationFrom2Cfg(const Cfg&, const Cfg&);
    
    template<template<class> class ClearanceUtility, class MPTraits>
      double GetSmoothingValue(ClearanceUtility<MPTraits>& _clearanceUtils, shared_ptr<Boundary> _bb);

    //polygonal approximation
    vector<Vector3d> PolyApprox (Environment* _env) const;

    size_t GetRobotIndex() const {return m_robotIndex;}
    void SetRobotIndex(size_t _newIndex){m_robotIndex = _newIndex;}

    //I/O Helper functions
    virtual void Read(istream& _is);
    virtual void Write(ostream& _os) const;
  
  protected:
    //Normalize the orientation to the range [-1, 1)
    virtual void NormalizeOrientation(int _index = -1);
    
    //generates random configuration within C-space
    virtual void GetRandomCfgImpl(Environment* _env, shared_ptr<Boundary> bb);

    vector<double> m_v;   
    size_t m_robotIndex; //which active body in the env this cfg refers to

    static size_t m_dof;
    static size_t m_posdof;
    static size_t m_numJoints;

    enum DofType {POS, ROT, JOINT};
    static vector<DofType> m_dofTypes;
    static vector<Robot> m_robots;

    /** TODO- there may still problem with (un)packing map
     */
    map<string,bool> m_labelMap;
    map<string,double> m_statMap;

  public:
    static const vector<Robot>& GetRobots() { return m_robots; }

    CDInfo m_clearanceInfo;
    shared_ptr<Cfg> m_witnessCfg;

#ifdef _PARALLEL
    void define_type(stapl::typer& _t)  
    {
      _t.member(m_v);
      _t.member(m_labelMap);
      _t.member(m_statMap);
      _t.member(m_robotIndex); 
    }
#endif
}; // class Cfg

//I/O for Cfg
ostream& operator<< (ostream& _os, const Cfg& _cfg);
istream& operator>> (istream& _is, Cfg& _cfg);

template<class DistanceMetricPointer>
void
Cfg::GetRandomRay(double _incr, Environment* _env,  DistanceMetricPointer _dm, bool _norm){
  //randomly sample params
  m_v.clear();
  for(size_t i = 0; i < DOF(); ++i)
    m_v.push_back(2.0*DRand() - 1.0);

  //scale to appropriate length
  Cfg origin;
  _dm->ScaleCfg(_env, _incr, origin, *this);
  if(_norm)
    NormalizeOrientation();
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
        operator target_t() const { return Accessor::read(); }
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
