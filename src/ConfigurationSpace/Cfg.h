#ifndef CFG_H_
#define CFG_H_

#include <cstddef>
#include <map>
#include <vector>

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "Utilities/MPUtils.h"

#include "Vector.h"

#ifdef _PARALLEL
#include "views/proxy.h"
#endif

enum class DofType;
class Cfg;
class Environment;
class ActiveMultiBody;
class Boundary;


////////////////////////////////////////////////////////////////////////////////
/// Information about the clearance of a cfg.
/// @ingroup Cfgs
////////////////////////////////////////////////////////////////////////////////
class ClearanceInfo final {

  private:

    ///@name Internal State
    ///@{

    double m_clearance; ///< Distance to nearest c-space obstacle.
    Cfg* m_direction;   ///< Direction to nearest c-obstacle configuration.
    int m_obstacleId;   ///< The index of the nearest workspace obstacle?

    ///@}

  public:

    ///@name Construction
    ///@{

    ClearanceInfo(Cfg* _direction = nullptr, double _clearance = -1e10) :
        m_clearance(_clearance), m_direction(_direction) { }

    ~ClearanceInfo();

    ///@}
    ///@name Clearance Interface
    ///@{

    double GetClearance() {return m_clearance;};
    void SetClearance(double _clearance) {m_clearance = _clearance;};

    Cfg* GetDirection() {return m_direction;};
    void SetDirection(Cfg* _direction) {m_direction = _direction;};

    int GetObstacleId() { return m_obstacleId;};
    void SetObstacleId(int _id) { m_obstacleId = _id;};

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

    explicit Cfg(size_t _index = 0);
    explicit Cfg(const Vector3d& _v, size_t _index = 0);
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
    Point3d GetPoint() const;

    /// Get the internal storage of DOF data.
    const vector<double>& GetData() const {return m_v;}

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
    vector<double> GetNormalizedData(const shared_ptr<const Boundary> _b) const;

    /// Compute the standard representation from a form normalized relative to
    /// the environment bounds. This is the reverse of GetNormalizedData.
    /// @param[in] _data The normalized DOF data relative to _b.
    /// @param[in] _b    The normalization boundary.
    void SetNormalizedData(const vector<double>& _data,
        const shared_ptr<const Boundary> _b);

    ///@}
    ///@name Labels and Stats
    ///@{

    //labeling of the Cfg and statistics
    bool GetLabel(string _label);
    bool IsLabel(string _label);
    void SetLabel(string _label, bool _value);

    double GetStat(string _stat) const;
    bool IsStat(string _stat) const;
    void SetStat(string _stat, double _value = 0.0);
    void IncStat(string _stat, double _value = 1.0);

    ///@}
    ///@name Robot Info
    ///@{

    size_t DOF() const;
    size_t PosDOF() const;
    size_t GetNumOfJoints() const;

    /// Get the robot referenced by this configuration.
    /// @param[in] _index The robot index of interest.
    ActiveMultiBody* GetRobot() const {
      return m_robotIndex == size_t(-1) ? m_pointRobot : m_robots[m_robotIndex];
    }

    size_t GetRobotIndex() const {return m_robotIndex;}
    void SetRobotIndex(size_t _i) {m_robotIndex = _i;}

    //Calculate the center position and center of mass of the robot configures
    //at this Cfg
    virtual Vector3d GetRobotCenterPosition() const;
    virtual Vector3d GetRobotCenterofMass() const;

    ///@}

    void ResetRigidBodyCoordinates();

    // Generation Related Methods : create Cfgs randomly
    /**
     * Configuration where workspace robot's EVERY VERTEX
     * is guaranteed to lie within the environment specified bounding box If
     * not, a cfg couldn't be found in the bbx, and the program will abort.
     * The function will try a predefined number of times
     */
    virtual void GetRandomCfg(Environment* _env);
    virtual void GetRandomCfg(Environment* _env, shared_ptr<Boundary> _bb);

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
        shared_ptr<Boundary> _bb);

    //polygonal approximation
    vector<Vector3d> PolyApprox() const;

    ///@name I/O
    ///@{

    virtual void Read(istream& _is);
    virtual void Write(ostream& _os) const;

    ///@}
    ///@name Ugly Static Hacks
    ///@{
    /// @TODO Remove this junk and replace with a more elegant, instance-based
    ///       solution. Storing an index is specifically less efficient than
    ///       storing a robot pointer since we require an additional look-up
    ///       (and also container) to get the desired object.

    static vector<ActiveMultiBody*> m_robots;
    static ActiveMultiBody* m_pointRobot;

    static void InitRobots(ActiveMultiBody* _robot, size_t _index);

    static void SetSize(size_t _size);

    static vector<ActiveMultiBody*> GetRobots() {return m_robots;}

    ///@}
    ///@name Internal State with poor encapsulation
    ///@{
    /// @TODO Fix encapsulation issues.

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
    virtual void GetRandomCfgImpl(Environment* _env, shared_ptr<Boundary> _b);

    ///@name Internal State
    ///@{

    vector<double> m_v;  ///< The DOF values.
    size_t m_robotIndex; ///< The ActiveBody this cfg refers to.

    map<string, bool> m_labelMap;  ///< A map of labels for this cfg.
    map<string, double> m_statMap; ///< A map of stats for this cfg.

    ///@}

#ifdef _PARALLEL

  public:

    //parallel connected component
    void active(bool _a) {m_active = _a;}
    bool active() const {return m_active;}
    void cc(size_t _c) {m_cc = _c;}
    size_t cc() const {return m_cc;}

    void define_type(stapl::typer& _t) {
      _t.member(m_v);
      _t.member(m_labelMap);
      _t.member(m_statMap);
      _t.member(m_robotIndex);
      _t.member(m_active);
      _t.member(m_cc);
    }

  private:

    bool m_active;
    size_t m_cc;

#endif
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
    shared_ptr<Boundary> _bb) {
  CDInfo cdInfo;
  typename MPTraits::CfgType tmp;
  _clearanceUtils.CollisionInfo(static_cast<typename MPTraits::CfgType&>(*this),
      tmp, _bb, cdInfo);
  return cdInfo.m_minDist;
}

#ifdef _PARALLEL
namespace stapl {

  //////////////////////////////////////////////////////////////////////////////
  /// @TODO
  //////////////////////////////////////////////////////////////////////////////
  template <typename Accessor>
  class proxy<Cfg, Accessor> : public Accessor {

    friend class proxy_core_access;
    typedef Cfg target_t;

    public:

      //typedef target_t::parameter_type  parameter_type;
      explicit proxy(Accessor const& acc) : Accessor(acc) { }
      operator target_t() const {
        return Accessor::read();
      }

      proxy const& operator=(proxy const& rhs) {
        Accessor::write(rhs);
        return *this;
      }

      proxy const& operator=(target_t const& rhs) {
        Accessor::write(rhs);
        return *this;
      }

      int DOF() const {
        return Accessor::const_invoke(&target_t::DOF);
      }

      int PosDOF() const {
        return Accessor::const_invoke(&target_t::PosDOF);
      }

      void Write(ostream& _os) const {
        return Accessor::const_invoke(&target_t::Write, _os);
      }

      void Read(istream& _is) {
        return Accessor::const_invoke(&target_t::Read, _is);
      }

      const vector<double>& GetData() const {
        return Accessor::const_invoke(&target_t::GetData);
      }

      void SetData(vector<double>& _data) const {
        return Accessor::const_invoke(&target_t::SetData, _data);
      }

      bool GetLabel(string _label) const {
        return Accessor::const_invoke(&target_t::GetLabel, _label);
      }

      bool IsLabel(string _label) const {
        return Accessor::const_invoke(&target_t::IsLabel, _label);
      }

      bool SetLabel(string _label) const {
        return Accessor::const_invoke(&target_t::SetLabel, _label);
      }

      double GetStat(string _stat) const {
        return Accessor::const_invoke(&target_t::GetStat, _stat);
      }

      bool IsStat(string _stat) const {
        return Accessor::const_invoke(&target_t::IsStat, _stat);
      }

      void SetStat(string _stat, double _val) const {
        return Accessor::const_invoke(&target_t::SetStat, _stat,_val);
      }

      void IncStat(string _stat, double _val) const {
        return Accessor::const_invoke(&target_t::IncStat, _stat,_val);
      }

      static int GetNumOfJoints() {
        return Accessor::const_invoke(&target_t::GetNumOfJoints);
      }

      void active(bool _a) {
        return Accessor::invoke(&target_t::active, _a);
      }

      bool active() const {
        return Accessor::const_invoke(&target_t::active);
      }

      void cc(size_t _c) {
        return Accessor::invoke(&target_t::cc, _c);
      }

      size_t cc() const {
        return Accessor::const_invoke(&target_t::cc);
      }

  };
}
#endif

#endif
