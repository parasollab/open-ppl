#ifndef CFG_H_
#define CFG_H_

#include <cstddef>
#include <map>
#include <vector>

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "Utilities/MPUtils.h"

#include "Vector.h"

class ActiveMultiBody;
class Boundary;
class Cfg;
class Environment;
class Robot;

enum class DofType;


////////////////////////////////////////////////////////////////////////////////
/// Information about the clearance of a cfg.
/// @ingroup Cfgs
////////////////////////////////////////////////////////////////////////////////
class ClearanceInfo final {

  private:

    ///@name Internal State
    ///@{

    double m_clearance;        ///< Distance to nearest c-space obstacle.
    Cfg* m_direction{nullptr}; ///< Direction to nearest c-obstacle configuration.
    int m_obstacleId;          ///< The index of the nearest workspace obstacle?

    ///@}

  public:

    ///@name Construction
    ///@{

    ClearanceInfo(Cfg* _direction = nullptr, const double _clearance = -1e10);

    ~ClearanceInfo();

    ///@}
    ///@name Clearance Interface
    ///@{

    double GetClearance() const noexcept;
    void SetClearance(const double _clearance) noexcept;

    Cfg* GetDirection() const noexcept;
    void SetDirection(Cfg* _direction) noexcept;

    int GetObstacleId() const noexcept;
    void SetObstacleId(const int _id) noexcept;

    ///@}

};


////////////////////////////////////////////////////////////////////////////////
/// Default @cspace configuration definition.
///
/// @ingroup Cfgs
/// @details Cfg is the core class which defines a configuration, or a vector of
///          values representing all the degrees of freedom of a robot. This is
///          the abstraction of @cspace essentially, and thus Cfg is a point or
///          vector inside of @cspace. Most mathematical operations are defined
///          over this class, i.e., @c operator+ and @c operator-, reading and
///          writing to streams, accessing, random sampling, etc.
////////////////////////////////////////////////////////////////////////////////
class Cfg {

  public:

    ///@name Construction
    ///@{

    explicit Cfg(Robot* const _robot = nullptr);
    explicit Cfg(const Vector3d& _v, Robot* const _robot = nullptr);
    Cfg(const Cfg& _other);

    virtual ~Cfg() = default;

    ///@}
    ///@name Assignment
    ///@{

    Cfg& operator=(const Cfg& _cfg);
    Cfg& operator+=(const Cfg& _cfg);
    Cfg& operator-=(const Cfg& _cfg);
    Cfg& operator*=(double _d);
    Cfg& operator/=(double _d);

    ///@}
    ///@name Arithmetic
    ///@{

    Cfg operator+(const Cfg& _cfg) const;
    Cfg operator-(const Cfg& _cfg) const;
    Cfg operator-() const;
    Cfg operator*(double _d) const;
    Cfg operator/(double _d) const;

    virtual double Magnitude() const;
    virtual double PositionMagnitude() const;
    virtual double OrientationMagnitude() const;

    ///@}
    ///@name Equality
    ///@{

    bool operator==(const Cfg& _cfg) const;
    bool operator!=(const Cfg& _cfg) const;

    ///@}
    ///@name DOF Accessors
    ///@{

    double& operator[](size_t _dof);
    const double& operator[](size_t _dof) const;

    virtual vector<double> GetPosition() const;
    virtual vector<double> GetOrientation() const;

    virtual vector<double> GetNonJoints() const;
    virtual vector<double> GetJoints() const;
    virtual vector<double> GetRotation() const;

    /// Get the robot's reference point.
    Point3d GetPoint() const noexcept;

    /// Get the internal storage of DOF data.
    const vector<double>& GetData() const noexcept;

    /// Set the internal storage of DOF data.
    virtual void SetData(const vector<double>& _data);

    /// Set the internal storage of joint DOF data.
    /// @details Other DOFs will remain the same.
    void SetJointData(const vector<double>& _data);

    /// Compute the normalized representation relative to the environment bounds.
    /// @warning This is not a normalization to length 1!
    /// @param[in] _b The boundary to normalize against.
    /// @return  A copy of this with each DOF scaled from [_b.min, _b.max] to
    ///          [-1, 1].
    vector<double> GetNormalizedData(const Boundary* const _b) const;

    /// Compute the standard representation from a form normalized relative to
    /// the environment bounds. This is the reverse of GetNormalizedData.
    /// @param[in] _data The normalized DOF data relative to _b.
    /// @param[in] _b    The normalization boundary.
    void SetNormalizedData(const vector<double>& _data,
        const Boundary* const _b);

    ///@}
    ///@name Labels and Stats
    ///@{

    //labeling of the Cfg and statistics
    bool GetLabel(const std::string& _label) const;
    bool IsLabel(const std::string& _label) const noexcept;
    void SetLabel(const std::string& _label, const bool _value) noexcept;

    double GetStat(const std::string& _stat) const;
    bool IsStat(const std::string& _stat) const noexcept;
    void SetStat(const std::string& _stat, const double _value = 0.0) noexcept;
    void IncStat(const std::string& _stat, const double _value = 1.0) noexcept;

    ///@}
    ///@name Robot Info
    ///@{

    size_t DOF() const;
    size_t PosDOF() const;
    size_t GetNumOfJoints() const;

    /// Get the robot referenced by this configuration.
    Robot* GetRobot() const noexcept;

    /// Get the robot's multibody.
    ActiveMultiBody* GetMultiBody() const noexcept;

    //Calculate the center position and center of mass of the robot configures
    //at this Cfg
    virtual Vector3d GetRobotCenterPosition() const;
    virtual Vector3d GetRobotCenterofMass() const;

    ///@}

    void ResetRigidBodyCoordinates();

    ///@}
    ///@name Generation Methods
    ///@{

    /// Create a configuration where workspace robot's EVERY VERTEX
    /// is guaranteed to lie within the environment specified bounding box. If
    /// a cfg couldn't be found in the bbx, the program will abort.
    /// The function will try a predefined number of times
    virtual void GetRandomCfg(Environment* _env);
    virtual void GetRandomCfg(Environment* _env, const Boundary* const _b);

    template <typename DistanceMetricPointer>
    void GetRandomRay(double _incr, DistanceMetricPointer _dm, bool _norm = true);

    virtual void ConfigureRobot() const;

    void GetResolutionCfg(Environment*);

    // Increase every value in this instance in each dimention by the value in
    // _increment.
    virtual void IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment);
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks,
        double _positionRes, double _orientationRes);
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks);

    /**create a new Cfg instance whose configuration is weighted summation of the
     *first and the second Cfg.
     *The summation is done in every dimension in CSpace.
     *weight should between [0,1]. this weight is for the second Cfg.
     * The weight for the first Cfg is (1-weight)
     */
    virtual void WeightedSum(const Cfg&, const Cfg&, double _weight = 0.5);

    virtual void GetPositionOrientationFrom2Cfg(const Cfg&, const Cfg&);

    template<template<class> class ClearanceUtility, class MPTraits>
    double GetSmoothingValue(ClearanceUtility<MPTraits>& _clearanceUtils,
        const Boundary* const _b);

    //polygonal approximation
    vector<Vector3d> PolyApprox() const;

    ///@name I/O
    ///@{

    virtual void Read(istream& _is);
    virtual void Write(ostream& _os) const;

    ///@}
    ///@name Internal State with poor encapsulation
    ///@{
    /// @TODO Fix encapsulation issues.
    /// @TODO Witness should not be a shared_ptr.

    CDInfo m_clearanceInfo;
    shared_ptr<Cfg> m_witnessCfg;

    ///@}

  protected:

    /// Normalize an orientation DOF to the range [-1, 1).
    /// @param[in] _index The index of the DOF to normalize. If it is -1, all
    ///                   orientation DOFs will be normalized.
    virtual void NormalizeOrientation(int _index = -1);

    /// Set this configuration's DOFs to random values that lie within a given
    /// sampling boundary.
    /// @param[in] _env The environment to generate the configuration within.
    /// @param[in] _b The boundary to sample within.
    virtual void GetRandomCfgImpl(Environment* _env, const Boundary* const _b);

    ///@name Internal State
    ///@{

    vector<double> m_v;  ///< The DOF values.
    Robot* m_robot{nullptr}; ///< The robot this cfg refers to.

    map<string, bool> m_labelMap;  ///< A map of labels for this cfg.
    map<string, double> m_statMap; ///< A map of stats for this cfg.

    ///@}

};

//I/O for Cfg
ostream& operator<<(ostream& _os, const Cfg& _cfg);
istream& operator>>(istream& _is, Cfg& _cfg);


template <class DistanceMetricPointer>
void
Cfg::
GetRandomRay(double _incr, DistanceMetricPointer _dm, bool _norm) {
  //randomly sample params
  m_v.clear();
  for(size_t i = 0; i < DOF(); ++i)
    m_v.push_back(2. * DRand() - 1.);

  //scale to appropriate length
  _dm->ScaleCfg(_incr,
      static_cast<typename DistanceMetricPointer::element_type::CfgType&>(*this));
  if(_norm)
    NormalizeOrientation();
}


template <template <class> class ClearanceUtility, class MPTraits>
double
Cfg::
GetSmoothingValue(ClearanceUtility<MPTraits>& _clearanceUtils,
    const Boundary* const _b) {
  CDInfo cdInfo;
  typename MPTraits::CfgType tmp;
  _clearanceUtils.CollisionInfo(static_cast<typename MPTraits::CfgType&>(*this),
      tmp, _b, cdInfo);
  return cdInfo.m_minDist;
}


#endif
