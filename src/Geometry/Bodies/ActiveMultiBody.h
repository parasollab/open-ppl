#ifndef ACTIVE_MULTI_BODY_H_
#define ACTIVE_MULTI_BODY_H_

#include "FreeBody.h"
#include "MultiBody.h"
#include "Geometry/Boundaries/Range.h"

class Boundary;
class Cfg;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Types of movement that are supported.
////////////////////////////////////////////////////////////////////////////////
enum class DofType {
  Positional, ///< Translational motion R = [min, max]
  Rotational, ///< Rotational motion in S = [-1, 1]
  Joint       ///< Rotational motion in R = [min, max]
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Geometry
/// A collection of geometries in workspace reprenting robots or other movable
/// objects.
////////////////////////////////////////////////////////////////////////////////
class ActiveMultiBody : public MultiBody {

  public:

    ///@name Local Types
    ///@{

    typedef shared_ptr<Connection> Joint; ///< Joint of robot

    ////////////////////////////////////////////////////////////////////////////
    /// Information of DOF values: name, minimum value, and maximum value.
    ////////////////////////////////////////////////////////////////////////////
    struct DofInfo final {

      ///@name Construction
      ///@{

      /// Construct a DofInfo with a name and range of allowed values.
      /// @param _n Semantic name for this DOF.
      /// @param _min Minimum allowed value.
      /// @param _max Maximum allowed value.
      DofInfo(string&& _n, const double _min, const double _max) :
          name(_n), range(_min, _max) {}

      ///@}
      ///@name Internal State
      ///@{

      string name;         ///< DOF name
      Range<double> range; ///< Range of allowed DOF values.

      ///@}

    };

    ///@}
    ///@name Contruction
    ///@{

    ActiveMultiBody();

    ActiveMultiBody(XMLNode& _node);

    ActiveMultiBody(const ActiveMultiBody&) = delete;            ///< No copy
    ActiveMultiBody& operator=(const ActiveMultiBody&) = delete; ///< No assign

    virtual ~ActiveMultiBody() noexcept = default;

    ///@}
    ///@name MultiBody Info
    ///@{

    using MultiBody::MultiBodyType;

    virtual const Vector3d& GetCenterOfMass() const;

    /// Get the type for this MultiBody.
    virtual MultiBodyType GetType() const noexcept override {
      return MultiBodyType::Active;
    }

    /// Get the number of sub-bodies.
    size_t NumFreeBody() const;

    /// Get a sub-body by index.
    /// @param[in] _index The index of the free body.
    /// @return A pointer to the desired body.
    const FreeBody* GetFreeBody(const size_t _index) const;
    FreeBody* GetFreeBody(const size_t _index);

    ///@}
    ///@name Robot Information
    ///@{

    /// Initialize DOFTypes of robot
    /// @param _b Boundary for DOF ranges
    /// @param _os If not null, DOF type information will be output here as well
    void InitializeDOFs(const Boundary* _b, ostream* _os = nullptr);

    /// @return Base Body type
    FreeBody::BodyType GetBaseType() const {return m_baseType;}

    /// @return Base movement type
    FreeBody::MovementType GetBaseMovementType() const {return m_baseMovement;}

    /// @void set Base movement type
    void SetBaseMovementType(FreeBody::MovementType _mt) {m_baseMovement=_mt;}

    /// @return Number of Connection in this multibody
    size_t NumJoints() const {return m_joints.size();}

    /// @return const_iterator to begin of joints data
    vector<Joint>::const_iterator joints_begin() const {return m_joints.begin();}

    /// @return const_iterator to end of joints data
    vector<Joint>::const_iterator joints_end() const {return m_joints.end();}

    /// @return DOF type information of robot
    const DofType& GetDOFType(const size_t _i) const {return m_dofTypes[_i];}

    /// @return DOF range information of robot
    const vector<DofInfo>& GetDofInfo() const {return m_dofInfo;}

    /// @return Number of DOF for this robot
    size_t DOF() const noexcept;

    /// Get the number of positional DOF for this robot's base.
    size_t PosDOF() const noexcept;

    /// Get the number of orientational DOF for this robot's base.
    size_t OrientationDOF() const noexcept;

    /// Get the number of joint DOF for this robot.
    size_t JointDOF() const noexcept;

    /// Get the current DOFs for this configuration, as set by Configure().
    const std::vector<double>& GetCurrentDOFs() const noexcept;

    ///@}
    ///@name Configuration Methods
    ///@{

    /// Place a robot at a given configuration.
    /// @param _c The configuration to use.
    void Configure(const Cfg& _c);

    /// Place a robot at a given configuration.
    /// @param _v The DOF values to use.
    void Configure(const vector<double>& _v);

    /// Place a robot at a given configuration.
    /// @param _v The position DOF values to use.
    /// @param _t The orientation DOF values to use.
    ///
    /// This version is to support functionality in the folding code where
    /// we need to configure the theta values as well.  Note that they are not
    /// stored in _v as full dofs since they are not treated as such.  Instead
    /// they are computed as a function of _v and other bio specific things.
    void Configure(const vector<double>& _v, const vector<double>& _t);

    /// Compute rendering transforms for robot at @p _v
    /// @param _v Configuration DOF parameters
    void ConfigureRender(const vector<double>& _v);

    /// @param[out] _result Polygonal Approximation
    void PolygonalApproximation(vector<Vector3d>& _result);

    ///@}
    ///@name I/O
    ///@{

    virtual void ReadXML(XMLNode& _node);
    virtual void Read(istream& _is, CountingStreamBuffer& _cbs);
    virtual void Write(ostream& _os);

    /// Add a free body.
    /// @param _body The free body to add.
    void AddBody(const shared_ptr<FreeBody>& _body);

    /// Set the base body.
    /// @param _body The free body to use as the base.
    void SetBaseBody(const shared_ptr<FreeBody>& _body);

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Update the transforms of each link after changing the base link's
    /// transform.
    void UpdateLinks();

    ///@}
    ///@name Internal State
    ///@{

    vector<shared_ptr<FreeBody>> m_freeBody; ///< All free body

    vector<DofType> m_dofTypes;              ///< DOF type of robot motions
    vector<DofInfo> m_dofInfo;               ///< DofInfo for each motion

    size_t m_baseIndex;                      ///< Free body index for base
    shared_ptr<FreeBody> m_baseBody;         ///< Body of base
    FreeBody::BodyType m_baseType;           ///< Type of base
    FreeBody::MovementType m_baseMovement;   ///< Type of movement for base

    vector<Joint> m_joints;                  ///< All Connections

    vector<double> m_currentDofs;            ///< The current configuration DOFs

    ///@}

};

#endif
