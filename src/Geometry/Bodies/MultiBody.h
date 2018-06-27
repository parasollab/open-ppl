#ifndef MULTI_BODY_H_
#define MULTI_BODY_H_

#include <memory>
#include <string>
#include <vector>

#include "Geometry/Bodies/Body.h"
#include "Geometry/Boundaries/Range.h"

class Boundary;
class Cfg;
class Connection;
class XMLNode;

#ifdef DEBUG_BULLET_PROBLEMS
class btMultiBody;
#endif


////////////////////////////////////////////////////////////////////////////////
/// Types of movement that are supported.
////////////////////////////////////////////////////////////////////////////////
enum class DofType {
  Positional, ///< Translational motion R = [min, max]
  Rotational, ///< Rotational motion in S = [-1, 1]
  Joint       ///< Rotational motion in R = [min, max]
};


////////////////////////////////////////////////////////////////////////////////
/// Information of DOF values: name, minimum value, and maximum value.
////////////////////////////////////////////////////////////////////////////////
struct DofInfo final {

  ///@name Construction
  ///@{

  /// Construct a DofInfo with a name and range of allowed values.
  /// @param _n Semantic name for this DOF.
  /// @param _type The type for this DOF.
  /// @param _min Minimum allowed value.
  /// @param _max Maximum allowed value.
  DofInfo(std::string&& _n, const DofType _type, const Range<double> _range) :
      name(_n), type(_type), range(_range) {}

  ///@}
  ///@name Internal State
  ///@{

  std::string name;    ///< DOF name.
  DofType type;        ///< DOF type.
  Range<double> range; ///< Range of allowed values.

  ///@}

};


////////////////////////////////////////////////////////////////////////////////
/// A geometric object in workspace (such as an obstacle or robot) with one or
/// more sub-components, referred to as Bodies.
/// @ingroup Geometry
////////////////////////////////////////////////////////////////////////////////
class MultiBody {

  public:

#ifdef DEBUG_BULLET_PROBLEMS
    /// @note This is a dirty hack for debugging the problems with linked robots
    ///       in the bullet simulation. Please do not use this elsewhere as I
    ///       will delete any code that does after I fix bullet and remove the
    ///       hack.
    btMultiBody* m_bullet{nullptr};
#endif

    ///@name Local Types
    ///@{

    /// The types of MultiBody that we can support.
    enum class Type {
      Active,       ///< Movable object.
      Passive,      ///< Static visible object.
      Internal      ///< Static invisible object.
    };


    ///@}
    ///@name Construction
    ///@{

    /// Construct a multibody.
    /// @param _type The multibody type.
    MultiBody(const MultiBody::Type _type);

    /// Parse a multibody from an XML node.
    MultiBody(XMLNode& _node);

    MultiBody(const MultiBody&);  ///< Copy.
    MultiBody(MultiBody&&);       ///< Move.

    ~MultiBody();

    /// Initialize the DOF information for this multibody.
    /// @param _b The Boundary for DOF ranges.
    void InitializeDOFs(const Boundary* const _b);

    ///@}
    ///@name Assignment
    ///@{

    MultiBody& operator=(const MultiBody&);  ///< Copy.
    MultiBody& operator=(MultiBody&&);       ///< Move.

    ///@}
    ///@name MultiBody Properties
    ///@{

    /// Get the type for this MultiBody.
    MultiBody::Type GetType() const noexcept;

    /// Is this MultiBody an active type?
    bool IsActive() const noexcept;

    /// Is this MultiBody a non-active type?
    bool IsPassive() const noexcept;

    /// Is this MultiBody an internal type?
    bool IsInternal() const noexcept;

    /// Is this MultiBody a composite body, i.e. having multiple decoupled
    /// bases? (used for assembly planning)
    bool IsComposite() const noexcept;

    /// Get the number of DOF for this multibody.
    size_t DOF() const noexcept;

    /// Get the number of positional DOF for this multibody's base.
    size_t PosDOF() const noexcept;

    /// Get the number of orientational DOF for this multibody's base.
    size_t OrientationDOF() const noexcept;

    /// Get the number of joint DOF for this multibody.
    size_t JointDOF() const noexcept;

    /// Get the current DOFs for this configuration, as set by Configure().
    const std::vector<double>& GetCurrentDOFs() const noexcept;

    /// Get the current configuration dofs (no velocity), as set by Configure().
    std::vector<double> GetCurrentCfg() noexcept;

    ///@}
    ///@name Body Accessors
    ///@{

    /// Get the number of bodies in this multibody.
    size_t GetNumBodies() const noexcept;

    /// Get an internal body.
    /// @param _i The index of the body.
    Body* GetBody(const size_t _i) noexcept;
    const Body* GetBody(const size_t _i) const noexcept;

    /// Add a body.
    /// @param _body The body to add.
    /// @return The index of the added body.
    size_t AddBody(Body&& _body);

    /// Get the base body.
    Body* GetBase() noexcept;
    const Body* GetBase() const noexcept;

    /// Set the base body.
    /// @param _index The index of the body to use as the base.
    void SetBaseBody(const size_t _index);

    /// Get the body type of the base body.
    Body::Type GetBaseType() const noexcept;

    /// Get the movement type of the base body.
    Body::MovementType GetBaseMovementType() const noexcept;

    ///@}
    ///@name Geometric Properties
    ///@{

    /// Get the center of mass.
    const Vector3d& GetCenterOfMass() const noexcept;

    /// Get the bounding sphere radius.
    double GetBoundingSphereRadius() const noexcept;

    /// Get the maximum distance in X, Y, or Z direction.
    double GetMaxAxisRange() const noexcept;

    /// Get the bounding box.
    const double* GetBoundingBox() const noexcept;

    /// Compute some kind of polygonal approximation of this multibody.
    /// @TODO This function is ancient and needs to be tested/cleaned up. It
    ///       should also return the result rather than using a c-style output
    ///       parameter.
    /// @param _result Polygonal Approximation
    void PolygonalApproximation(std::vector<Vector3d>& _result);

    ///@}
    ///@name Connections
    ///@{

    /// Get the Connections in this multibody.
    const std::vector<std::unique_ptr<Connection>>& GetJoints() const noexcept;

    /// Get a specific Connection.
    /// @param _i The connection index.
    Connection* GetJoint(const size_t _i) noexcept;

    /// Get the DOF type for a specific degree of freedom.
    const DofType& GetDOFType(const size_t _i) const noexcept;

    /// Get the DOF info for a specific degree of freedom.
    const std::vector<DofInfo>& GetDofInfo() const noexcept;

    /// Update the DOF info from the current values in the connection objects.
    /// @TODO Fix this so that we do not double-store the limits both here and in
    ///       connection.
    void UpdateJointLimits() noexcept;

    ///@}
    ///@name Configuration Methods
    ///@{

    /// Place a robot at a given configuration.
    /// @param _c The configuration to use.
    void Configure(const Cfg& _c);

    /// Place a robot at a given configuration.
    /// @param _v The DOF values to use.
    void Configure(const std::vector<double>& _v);

    /// Place a robot at a given configuration.
    /// @param _v The position DOF values to use.
    /// @param _t The orientation DOF values to use.
    ///
    /// This version is to support functionality in the folding code where
    /// we need to configure the theta values as well. Note that they are not
    /// stored in _v as full dofs since they are not treated as such.  Instead
    /// they are computed as a function of _v and other bio-specific things.
    void Configure(const std::vector<double>& _v, const std::vector<double>& _t);

    /// A helper function that generates the transformation for the body
    /// passed to it. given the dofs in _v and the _index offset.
    Transformation GenerateModelTransformation(const std::vector<double>& _v,
                            int& _index, const Body::MovementType _movementType,
                            const Body::Type _bodyType) const;

    /// Check if the DOF values are out-of-range. If so, push them to the nearest
    /// acceptable configuration and reconfigure the model.
    /// @note In this context, 'valid' means with respect to the joint limits
    ///       only. It does not consider collisions or any other validity.
    void PushToNearestValidConfiguration();

    ///@}
    ///@name I/O
    ///@{

    /// Read a MultiBody from an input stream and compute information.
    /// @param[in] _is The input stream to read from.
    /// @param[in] _cbs Counting stream buffer
    void Read(std::istream& _is, CountingStreamBuffer& _cbs);

    /// Write the MultiBody to an output stream.
    /// @param _os The output stream to write to.
    void Write(std::ostream& _os) const;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Sort the joints by body indexes.
    void SortJoints();

    /// Update the transforms of each link after changing the base link's
    /// transform.
    void UpdateLinks();

    /// Compute center of mass, boundaries, and range.
    /// @warning This function is wrong for COM and boundaries - the only thing
    ///          it computes correctly is the min and max bounding radii. The
    ///          problem is that it performs a one-time computation of the COM
    ///          and bbx, but these things should change as active bodies change
    ///          configuration.
    void FindMultiBodyInfo();

    ///@}
    ///@name Internal State
    ///@{

    Type m_multiBodyType{Type::Passive};  ///< The type of multibody.

    std::vector<Body> m_bodies;           ///< The sub-parts.

    Vector3d m_com;                       ///< Center of mass

    double m_radius{0};                   ///< Bounding Sphere
    std::array<double, 6> m_boundingBox;  ///< Bounding Box
    double m_maxAxisRange{0};             ///< Max axis range

    size_t m_baseIndex{0};                ///< Free body index for base
    Body* m_baseBody{nullptr};            ///< Body of base
    Body::Type m_baseType;                ///< Type of base
    Body::MovementType m_baseMovement;    ///< Type of movement for base

    std::vector<std::unique_ptr<Connection>> m_joints;  ///< All Connections

    std::vector<DofInfo> m_dofInfo;       ///< DofInfo for each motion
    std::vector<double> m_currentDofs;    ///< The current configuration DOFs

    ///@}
};

#endif
