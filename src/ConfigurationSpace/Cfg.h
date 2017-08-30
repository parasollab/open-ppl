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
/// A point in configuration space.
///
/// @details Each instance holds an array of values representing all the degrees
///          of freedom of a robot. Translational DOFs represent the position of
///          a reference point on the robot relative to the world origin. Angular
///          and joint DOFs are normalized to the range [-1, 1), so this object
///          can only reliably represent points in @cspace and not directions.
////////////////////////////////////////////////////////////////////////////////
class Cfg {

  public:

    ///@name Construction
    ///@{

    /// Construct a configuration for Cfg::inputRobot.
    /// @details This constructor is provided for two cases: allocating storage
    ///          for Cfgs without knowing the robot in advance, and reading in
    ///          roadmaps. The robot pointer will be Cfg::inputRobot, which can
    ///          be thought of as a variable default for the Cfg(Robot* const)
    ///          constructor.
    explicit Cfg();

    /// Construct a configuration for a given robot.
    /// @param[in] _robot The robot this represents.
    explicit Cfg(Robot* const _robot);

    /// Construct a configuration for a given robot at a specified point in
    /// workspace.
    /// @param[in] _v The workspace location of the robot's reference point.
    /// @param[in] _robot The robot to represent.
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
    Cfg& operator*=(const Cfg& _cfg);
    Cfg& operator/=(const Cfg& _cfg);

    Cfg& operator*=(const double _d);
    Cfg& operator/=(const double _d);

    ///@}
    ///@name Arithmetic
    ///@{

    Cfg operator-() const;

    Cfg operator+(const Cfg& _cfg) const;
    Cfg operator-(const Cfg& _cfg) const;
    Cfg operator*(const Cfg& _cfg) const;
    Cfg operator/(const Cfg& _cfg) const;

    Cfg operator*(const double _d) const;
    Cfg operator/(const double _d) const;

    ///@}
    ///@name Equality
    ///@{

    bool operator==(const Cfg& _cfg) const;
    bool operator!=(const Cfg& _cfg) const;

    ///@}
    ///@name Comparison
    ///@{

    bool operator<(const Cfg& _cfg) const;
    ///@}
    ///@name Robot Info
    ///@{

    /// Get the robot referenced by this configuration.
    Robot* GetRobot() const noexcept;

    /// Get the robot's multibody.
    ActiveMultiBody* GetMultiBody() const noexcept;

    size_t DOF() const noexcept;
    size_t PosDOF() const noexcept;
    size_t OriDOF() const noexcept;
    size_t JointDOF() const noexcept;

    bool IsNonholonomic() const noexcept;

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

    Vector3d GetLinearPosition() const;
    Vector3d GetAngularPosition() const;
    Vector3d GetLinearVelocity() const;
    Vector3d GetAngularVelocity() const;

    void SetLinearPosition(const Vector3d& _v);
    void SetAngularPosition(const Vector3d& _v);
    void SetLinearVelocity(const Vector3d& _v);
    void SetAngularVelocity(const Vector3d& _v);

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

    /// Set all DOFs to zero.
    void Zero() noexcept;

    /// Test if a configuration lies within a boundary and also within the
    /// robot's c-space limits.
    /// @param[in] _boundary The boundary to check.
    /// @return True if the configuration places the robot inside both the
    ///         boundary and its DOF limits.
    bool InBounds(const Boundary* const _b) const noexcept;
    /// @overload
    bool InBounds(const Environment* const _env) const noexcept;

    /// Create a configuration where workspace robot's EVERY VERTEX
    /// is guaranteed to lie within the specified boundary. If
    /// a cfg can't be found, the program will abort.
    /// The function will try a predefined number of times
    virtual void GetRandomCfg(const Boundary* const _b);
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
    ///@name C-Space Directions
    ///@{

    /// Compute the direction in C-Space between two Cfgs. This is equivalent
    /// to subtraction without normalization.
    /// @WARNING This function produces a non-normalized Cfg. Applying any Cfg
    ///          arithmetic operations to it will then normalize it. We should
    ///          eventually fix this by implementing Cfg's with a generic
    ///          n-vector object that represents directions. Then, our Cfg
    ///          arithmetic can normalize the values for point representation and
    ///          return the non-normalized vector objects for directions.
    /// @param[in] _target The destination point in C-Space.
    /// @return The direction (_target - *this).
    Cfg FindDirectionTo(const Cfg& _target) const;

    ///@}
    ///@name I/O
    ///@{

    // Static pointer for reading roadmaps. It should be set to the relevant
    // robot before reading in the map file, and nullptr otherwise.
    /// @TODO This is needed because we use stapl's graph reading function,
    ///       which does not allow us to set the robot pointers on construction.
    ///       Devise a better scheme for doing this that does not involve static
    ///       data.
    static Robot* inputRobot;

    /// Read a configuration from an input stream.
    /// @param _is The input stream to read from.
    virtual void Read(istream& _is);

    /// Write a configuration to an output stream.
    /// @param _os The output stream to write to.
    virtual void Write(ostream& _os) const;

    /// Writes very helpful info when comparing two cfgs, like their euclidean
    /// distance, their positions, velocities, and the difference. This is
    /// especially helpful for debugging nonholonomic control-following from
    /// a roadmap while simulating.
    /// @param _os The output stream to write to.
    /// @param _shouldBe The cfg that is where the cfg is supposed to be.
    /// @param _current The cfg that is where the cfg actually is.
    static void PrintRobotCfgComparisonInfo(
        std::ostream& _out, const Cfg& _shouldBe, const Cfg& _current);

    /// Print the Cfg's dofs and velocities with limited precision for terminal
    /// debugging.
    /// @param _precision The display precision.
    /// @return A string of [dof1, dof2, ..., dofn] for holonomic robots, or
    ///         {[dof1, ..., dofn], <vel1, ..., veln>} for nonholonomic robots.
    std::string PrettyPrint(const size_t _precision = 4) const;

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

    /// Ensure that this Cfg respects the robot's velocity limits.
    void EnforceVelocityLimits() noexcept;

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
