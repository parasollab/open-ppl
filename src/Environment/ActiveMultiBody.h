#ifndef ACTIVE_MULTI_BODY_H_
#define ACTIVE_MULTI_BODY_H_

#include "FreeBody.h"
#include "MultiBody.h"

class Boundary;

////////////////////////////////////////////////////////////////////////////////
/// @brief Type of free movement
////////////////////////////////////////////////////////////////////////////////
enum class DofType {
  Positional, ///< Translational motion R = [min, max]
  Rotational, ///< Rotational motion in S = [-1, 1]
  Joint       ///< Rotational motion in R = [min, max]
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief A collection of geometries in workspace reprenting robots
///
/// A MultiBody representing a Robot in workspace.
////////////////////////////////////////////////////////////////////////////////
class ActiveMultiBody : public MultiBody {
  public:
    typedef shared_ptr<Connection> Joint; ///< Joint of robot

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Information of DOF values: name, minimum value, maximum value
    ////////////////////////////////////////////////////////////////////////////
    struct DOFInfo {
      //////////////////////////////////////////////////////////////////////////
      /// @param _n Name
      /// @param _min Minimum value
      /// @param _max Maximum value
      DOFInfo(string _n, double _min, double _max)
        : m_name(_n), m_minVal(_min), m_maxVal(_max) {}

      string m_name;   ///< DOF name
      double m_minVal; ///< DOF min val
      double m_maxVal; ///< DOF max val
    };

    ///@name Contruction
    ///@{

    ActiveMultiBody();

    ActiveMultiBody(const ActiveMultiBody&) = delete;            ///< No copy
    ActiveMultiBody& operator=(const ActiveMultiBody&) = delete; ///< No assign

    ///@}
    ///@name Bodies
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of free body
    size_t NumFreeBody() const;

    ////////////////////////////////////////////////////////////////////////////
    /// @return Free body accroding to the given index
    shared_ptr<FreeBody> GetFreeBody(size_t _index) const;

    ///@}
    ///@name Robot Information
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Initialize DOFTypes of robot
    /// @param _b Boundary for DOF ranges
    /// @param _os If not null, DOF type information will be output here as well
    void InitializeDOFs(shared_ptr<Boundary>& _b, ostream* _os = NULL);

    ////////////////////////////////////////////////////////////////////////////
    /// @return Base Body type
    FreeBody::BodyType GetBaseType() const {return m_baseType;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Base movement type
    FreeBody::MovementType GetBaseMovementType() const {return m_baseMovement;}

    ////////////////////////////////////////////////////////////////////////////
    /// @void set Base movement type
    void SetBaseMovementType(FreeBody::MovementType _mt) {m_baseMovement=_mt;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of Connection in this multibody
    size_t NumJoints() const {return m_joints.size();}

    ////////////////////////////////////////////////////////////////////////////
    /// @return const_iterator to begin of joints data
    vector<Joint>::const_iterator joints_begin() const {return m_joints.begin();}

    ////////////////////////////////////////////////////////////////////////////
    /// @return const_iterator to end of joints data
    vector<Joint>::const_iterator joints_end() const {return m_joints.end();}

    ////////////////////////////////////////////////////////////////////////////
    /// @return DOF type information of robot
    const vector<DofType>& GetDOFTypes() const {return m_dofTypes;}
    const DofType& GetDOFType(const size_t _i) const {return m_dofTypes[_i];}

    ////////////////////////////////////////////////////////////////////////////
    /// @return DOF range information of robot
    const vector<DOFInfo>& GetDOFInfo() const {return m_dofInfo;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of DOF for this robot
    size_t DOF() const {return m_dofTypes.size();}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of positional DOF for this robot
    size_t PosDOF() const;

    ///@}
    ///@name Configuration Methods
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Place robot at @p _v
    /// @param _v Configuration DOF parameters
    void Configure(const vector<double>& _v);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Place robot at @p _v for DOF and _t for theta values
    /// @param _v Configuration DOF parameters
    /// @param _t Configuration theta parameters
    ///
    /// This version is to support functionality in the folding code where
    /// we need to configure the theta values as well.  Note that they are not
    /// stored in _v as full dofs since they are not treated as such.  Instead
    /// they are computed as a function of _v and other bio specific things.
    void Configure(const vector<double>& _v, const vector<double>& _t);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute rendering transforms for robot at @p _v
    /// @param _v Configuration DOF parameters
    void ConfigureRender(const vector<double>& _v);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute the transformation for robot at @_index _v
    /// @param _v Configuration DOF parameters
    /// @param _index current index of _v
    /// @param _bodyType FreeBody::BodyType of the robot
    /// @param _movementType FreeBody::MovementType of the robot
    Transformation GenerateModelTransformation(const vector<double>& _v,
        int& _index, FreeBody::BodyType _bodyType,
        FreeBody::MovementType _movementType);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Sample random configuration in boundary
    /// @param _boundary Boundary
    vector<double> GetRandomCfg(shared_ptr<Boundary>& _boundary);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief   Get the DOF ranges for a given boundary.
    /// @param[in] _b The boundary in question.
    /// @return  A pair of configurations representing the minimum and maximum
    ///          DOF values allowed within the boundary.
    pair<vector<double>, vector<double>> GetCfgLimits(
        const shared_ptr<const Boundary>& _b) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @param _cfg Configuration dofs
    /// @param _b Workspace bounds
    /// @return True if @p _cfg is inside physical robot constraints
    bool InCSpace(const vector<double>& _cfg, shared_ptr<Boundary>& _b);

    ////////////////////////////////////////////////////////////////////////////
    /// @param[out] _result Polygonal Approximation
    void PolygonalApproximation(vector<Vector3d>& _result);

    ///@}
    ///@name I/O
    ///@{

    virtual void Read(istream& _is, CountingStreamBuffer& _cbs);
    virtual void Write(ostream& _os);

    ///@}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _body Body to add
    void AddBody(const shared_ptr<FreeBody>& _body);

    ////////////////////////////////////////////////////////////////////////////
    /// @param _body Body to set as base body
    /// @brief It is used to set some things that get set by default in the Read
    ///        function. Needed by group behaviors.
    void SetBaseBody(const shared_ptr<FreeBody>& _body);

  private:

    vector<shared_ptr<FreeBody>> m_freeBody; ///< All free body
    vector<DofType> m_dofTypes;              ///< DOF type of robot motions
    vector<DOFInfo> m_dofInfo;               ///< DOFInfo for each motion
    size_t m_baseIndex;                      ///< Free body index for base
    shared_ptr<FreeBody> m_baseBody;         ///< Body of base
    FreeBody::BodyType m_baseType;           ///< Type of base
    FreeBody::MovementType m_baseMovement;   ///< Type of movement for base
    vector<Joint> m_joints;                  ///< All Connections
};

#endif
