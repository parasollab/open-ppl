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
    Cfg(Cfg&& _other);

    virtual ~Cfg() = default;

    ///@}
    ///@name Assignment
    ///@{

    Cfg& operator=(const Cfg& _cfg);
    Cfg& operator=(Cfg&& _cfg);
    Cfg& operator+=(const Cfg& _cfg);
    Cfg& operator-=(const Cfg& _cfg);
    Cfg& operator*=(const double _d);
    Cfg& operator/=(const double _d);

    ///@}
    ///@name Arithmetic
    ///@{

    Cfg operator+(const Cfg& _cfg) const;
    Cfg operator-(const Cfg& _cfg) const;
    Cfg operator-() const;
    Cfg operator*(const double _d) const;
    Cfg operator/(const double _d) const;

    ///@}
    ///@name Equality
    ///@{

    bool operator==(const Cfg& _cfg) const;
    bool operator!=(const Cfg& _cfg) const;

    ///@}
    ///@name Robot Info
    ///@{

    /// Get the robot referenced by this configuration.
    Robot* GetRobot() const noexcept;

    /// Get the robot's multibody.
    ActiveMultiBody* GetMultiBody() const noexcept;

    size_t DOF() const;
    size_t PosDOF() const;
    size_t OriDOF() const;
    size_t JointDOF() const;

    ///@}
    ///@name DOF Accessors
    ///@{

    /// Access the data for a given DOF.
    double& operator[](const size_t _dof) noexcept;
    double operator[](const size_t _dof) const noexcept;

    /// Get the data for all DOFs.
    const vector<double>& GetData() const noexcept;

    /// Access the velocity data for a given DOF.
    double& Velocity(const size_t _dof) noexcept;
    double Velocity(const size_t _dof) const noexcept;

    /// Get the velocity data for all DOFs.
    const vector<double>& GetVelocity() const noexcept;

    /// Set the DOF data.
    virtual void SetData(const vector<double>& _data);
    virtual void SetData(vector<double>&& _data);

    /// Set the joint DOF data. Other DOFs will remain unchanged.
    void SetJointData(const vector<double>& _data);

    /// Compute the normalized representation relative to the environment bounds.
    /// @warning This is not a normalization to length 1!
    /// @param[in] _b The boundary to normalize against.
    /// @return  A copy of this with each DOF scaled from [_b.min, _b.max] to
    ///          [-1, 1].
    vector<double> GetNormalizedData(const Boundary* const _b) const;

    /// Compute the standard representation from a form normalized relative to
    /// the environment bounds. This is the inverse of GetNormalizedData.
    /// @param[in] _data The normalized DOF data relative to _b.
    /// @param[in] _b    The normalization boundary.
    void SetNormalizedData(const vector<double>& _data,
        const Boundary* const _b);

    /// Get the robot's reference point.
    Point3d GetPoint() const noexcept;

    virtual vector<double> GetPosition() const;
    virtual vector<double> GetRotation() const;
    virtual vector<double> GetJoints() const;
    virtual vector<double> GetNonJoints() const;
    virtual vector<double> GetOrientation() const;

    virtual double Magnitude() const;
    virtual double PositionMagnitude() const;
    virtual double OrientationMagnitude() const;

    Vector3d LinearPosition() const;
    Vector3d AngularPosition() const;
    Vector3d LinearVelocity() const;
    Vector3d AngularVelocity() const;

    ///@}
    ///@name Labels and Stats
    ///@{
    /// Each Cfg has a set of labels and stats. Label are boolean attributes,
    /// while stats are real-valued.

    bool GetLabel(const std::string& _label) const;
    bool IsLabel(const std::string& _label) const noexcept;
    void SetLabel(const std::string& _label, const bool _value) noexcept;

    double GetStat(const std::string& _stat) const;
    bool IsStat(const std::string& _stat) const noexcept;
    void SetStat(const std::string& _stat, const double _value = 0) noexcept;
    void IncrementStat(const std::string& _stat, const double _value = 1)
        noexcept;

    ///@}
    ///@name Generation Methods
    ///@{

    /// Create a configuration where workspace robot's EVERY VERTEX
    /// is guaranteed to lie within the environment specified bounding box. If
    /// a cfg couldn't be found in the bbx, the program will abort.
    /// The function will try a predefined number of times
    virtual void GetRandomCfg(Environment* _env, const Boundary* const _b);
    virtual void GetRandomCfg(Environment* _env);

    /// Randomly sample the velocity for this configuration.
    virtual void GetRandomVelocity();

    /// Generate a random configuration with a set length.
    /// @param _length The desired length.
    /// @param _dm The distance metric for checking length.
    /// @param _norm Normalize the orientation DOFs?
    template <typename DistanceMetricPointer>
    void GetRandomRay(const double _length, DistanceMetricPointer _dm,
        const bool _norm = true);

    /// Configure the robot with the DOF values of this configuration.
    virtual void ConfigureRobot() const;

    /// Move this configuration towards a goal by adding a fixed increment.
    /// @param _goal The desired goal configuration.
    /// @param _increment The fixed increment to add to this, moving towards
    ///                   goal.
    virtual void IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment);

    /// Find the c-space increment that moves from a start to a goal in a fixed
    /// number of steps.
    /// @param _start The start configuration.
    /// @param _goal The goal configuration.
    /// @param _nTicks The number of steps to take.
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal,
        const int _nTicks);

    /// Find the c-space increment and number of steps needed to move from a
    /// start to a goal, taking steps no larger than the designated resolutions.
    /// @param _start The start configuration.
    /// @param _goal The goal configuration.
    /// @param _nTicks The number of steps to take (computed by this method).
    /// @param _positionRes The position resolution to use.
    /// @param _orientationRes The orientation resolution to use.
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks,
        const double _positionRes, const double _orientationRes);

    /// Create a configuration from the weighted sum of two other cfgs.
    /// @param _c1 The first configuration.
    /// @param _c2 The second configuration.
    /// @param _weight The weight for the second configuration. The first will
    ///                have (1 - _weight).
    virtual void WeightedSum(const Cfg& _c1, const Cfg& _c2,
        const double _weight = .5);

    /// Extract the position and orientation for this configuration from two
    /// other configurations.
    /// @param _pos Copy the position from this configuration.
    /// @param _ori Copy the orientation from this configuration.
    virtual void GetPositionOrientationFrom2Cfg(const Cfg& _pos, const Cfg& _ori);

    ///@}
    ///@name I/O
    ///@{

    /// Read a configuration from an input stream.
    /// @param _is The input stream to read from.
    virtual void Read(istream& _is);

    /// Write a configuration to an output stream.
    /// @param _os The output stream to write to.
    virtual void Write(ostream& _os) const;

    ///@}
    ///@name Internal State with poor encapsulation
    ///@{
    /// @TODO Fix encapsulation issues.
    /// @TODO Witness should not be a shared_ptr.

    CDInfo m_clearanceInfo;
    shared_ptr<Cfg> m_witnessCfg;

    ///@}
    ///@name Helpers
    ///@{

    /// Normalize an orientation DOF to the range [-1, 1).
    /// @param[in] _index The index of the DOF to normalize. If it is -1, all
    ///                   orientation DOFs will be normalized.
    virtual void NormalizeOrientation(const int _index = -1) noexcept;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    vector<double> m_dofs;         ///< The DOF values.
    vector<double> m_vel;          ///< The velocities, if any.
    Robot* m_robot{nullptr};       ///< The robot this cfg refers to.

    map<string, bool> m_labelMap;  ///< A map of labels for this cfg.
    map<string, double> m_statMap; ///< A map of stats for this cfg.

    ///@}

};

/*--------------------------- Generation Methods -----------------------------*/

template <class DistanceMetricPointer>
void
Cfg::
GetRandomRay(const double _length, DistanceMetricPointer _dm, const bool _norm) {
  // Randomly sample DOFs.
  for(size_t i = 0; i < DOF(); ++i)
    m_dofs[i] = 2. * DRand() - 1.;

  // Scale to appropriate length.
  _dm->ScaleCfg(_length,
      static_cast<typename DistanceMetricPointer::element_type::CfgType&>(*this));

  // Normalize if requested.
  if(_norm)
    NormalizeOrientation();
}

/*----------------------------------------------------------------------------*/

ostream& operator<<(ostream& _os, const Cfg& _cfg);
istream& operator>>(istream& _is, Cfg& _cfg);

#endif
