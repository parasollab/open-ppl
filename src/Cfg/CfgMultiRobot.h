#ifndef CFG_MULTI_ROBOT_H
#define CFG_MULTI_ROBOT_H

#include "Cfg.h"
#include <vector>

#include "Utilities/MPUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Cfgs
/// @brief CfgMultiRobot is a container class for controlling multiple cfg.
///
/// The CfgMultiRobot class is a container class that contains multiple cfg and
/// propagate function call to its member cfgs. Since this class class is just a
/// container, there is no actually data stored in CfgMultiRobot; instead, they
/// are stored (operated) in member cfgs.
///
/// @todo template this class to not only Cfg, but more CfgType (CfgProtein,
///       CfgSurface)
////////////////////////////////////////////////////////////////////////////////
/* template<class T> */
class CfgMultiRobot : public Cfg {
public:

    //===================================================================
    //  Constructors and Destructor
    //===================================================================

    CfgMultiRobot();
    CfgMultiRobot(const Cfg& _c);
    CfgMultiRobot(const CfgMultiRobot& _c);
    ~CfgMultiRobot(){}

    CfgMultiRobot& operator=(const CfgMultiRobot& _cfg);
    bool operator==(const CfgMultiRobot& _cfg) const;
    bool operator!=(const CfgMultiRobot& _cfg) const;
    CfgMultiRobot operator+(const CfgMultiRobot& _cfg) const;
    CfgMultiRobot& operator+=(const CfgMultiRobot& _cfg);
    CfgMultiRobot operator-(const CfgMultiRobot& _cfg) const;
    CfgMultiRobot& operator-=(const CfgMultiRobot& _cfg);
    CfgMultiRobot operator-() const;
    CfgMultiRobot operator*(double _d) const;
    CfgMultiRobot& operator*=(double _d);
    CfgMultiRobot operator/(double _d) const;
    CfgMultiRobot& operator/=(double _d);
    double& operator[](size_t _dof);
    const double& operator[](size_t _dof) const;

    friend istream& operator>> (istream& _is, CfgMultiRobot& _cfg);

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Access Methods : Retrieve and set related information of this class
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    ///Get internal storage of configuration
    const vector<double>& GetData() const;
    void SetData(const vector<double>& _data);

    size_t DOF() const;
    virtual const string GetName() const {return "CfgMultiRobot";};

    virtual vector<double> GetPosition() const;
    virtual vector<double> GetOrientation() const;
    virtual double Magnitude() const;
    virtual double PositionMagnitude() const;
    virtual double OrientationMagnitude() const;

    //Calculate the center position and center of mass of the robot configures
    //at this Cfg
    virtual Vector3d GetRobotCenterPosition() const;
    virtual Vector3d GetRobotCenterofMass() const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Generation Related Methods : These methods are related to create Cfgs randomly
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    virtual void GetRandomCfg(Environment* _env);
    virtual void GetRandomCfg(Environment* _env, shared_ptr<Boundary> _bb);

    template<class DistanceMetricPointer>
      void GetRandomRay(double _incr, DistanceMetricPointer _dm, bool _norm=true);

    virtual void ConfigureRobot() const;

    void GetResolutionCfg(Environment*);

    ///Increase every value in this instance in each dimention by the value in _increment
    virtual void IncrementTowardsGoal(const CfgMultiRobot& _goal, const CfgMultiRobot& _increment);
    virtual void FindIncrement(const CfgMultiRobot& _start, const CfgMultiRobot& _goal, int* _nTicks, double _positionRes, double _orientationRes);
    virtual void FindIncrement(const CfgMultiRobot& _start, const CfgMultiRobot& _goal, int _nTicks);

    /**create a new Cfg instance whose configuration is weighted summation of the
     *first and the second Cfg.
     *The summation is done in every dimension in CSpace.
     *weight should between [0,1]. this weight is for the second Cfg.
     * The weight for the first Cfg is (1-weight)
     */
    virtual void WeightedSum(const CfgMultiRobot&, const CfgMultiRobot&, double _weight = 0.5);

    virtual void GetPositionOrientationFrom2Cfg(const CfgMultiRobot&, const CfgMultiRobot&);

    template<template<class> class ClearanceUtility, class MPTraits>
      double GetSmoothingValue(ClearanceUtility<MPTraits>& _clearanceUtils, shared_ptr<Boundary> _bb);

    //polygonal approximation
    vector<Vector3d> PolyApprox (Environment* _env) const;

    //I/O Helper functions
    virtual void Read(istream& _is);
    virtual void Write(ostream& _os) const;

    vector<Cfg> GetRobotsCollect() const {return m_robotsCollect;}

    static size_t m_numRobot;

  protected:
    vector<Cfg> m_robotsCollect;
}; // class CfgMultiRobot

template<class DistanceMetricPointer>
void
CfgMultiRobot::
GetRandomRay(double _incr, DistanceMetricPointer _dm, bool _norm) {
  for(auto& robot : m_robotsCollect)
    robot.GetRandomRay(_incr, _dm, _norm);
}

template<template<class> class ClearanceUtility, class MPTraits>
double
CfgMultiRobot::
GetSmoothingValue(ClearanceUtility<MPTraits>& _clearanceUtils, shared_ptr<Boundary> _bb){
  double result = 0.0;
  for(size_t i = 0; i < m_robotsCollect.size(); ++i)
    result += this->m_robotsCollect[i].GetSmoothingValue(_clearanceUtils, _bb);
  return result;
}

#endif
