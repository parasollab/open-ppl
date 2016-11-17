#ifndef ACTIVE_MULTI_BODY_H_
#define ACTIVE_MULTI_BODY_H_

#include "FreeBody.h"
#include "MultiBody.h"

class Boundary;
class Cfg;

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

    typedef shared_ptr<Connection> Joint; ///< Joint of robot

    ////////////////////////////////////////////////////////////////////////////
    /// Information of DOF values: name, minimum value, maximum value
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
    ///@name MultiBody Info
    ///@{

    using MultiBody::MultiBodyType;

    /// Get the type for this MultiBody.
    virtual MultiBodyType GetType() const noexcept override {
      return MultiBodyType::Active;
    }

    ///@}
    ///@name Bodies
    ///@{

    /// Get the number of sub-bodies.
    size_t NumFreeBody() const;

    /// Get a sub-body by index.
    /// @param[in] _index The index of the free body.
    /// @return A pointer to the desired body.
    shared_ptr<FreeBody> GetFreeBody(size_t _index) const;

    ///@}
    ///@name Robot Information
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// Initialize DOFTypes of robot
    /// @param _b Boundary for DOF ranges
    /// @param _os If not null, DOF type information will be output here as well
    void InitializeDOFs(const Boundary* _b, ostream* _os = nullptr);

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
    /// Place a robot at a given configuration.
    /// @param _c The configuration to use.
    void Configure(const Cfg& _c);

    ////////////////////////////////////////////////////////////////////////////
    /// Place a robot at a given configuration.
    /// @param _v The DOF values to use.
    void Configure(const vector<double>& _v);

    ////////////////////////////////////////////////////////////////////////////
    /// Place a robot at a given configuration.
    /// @param _v The position DOF values to use.
    /// @param _t The orientation DOF values to use.
    ///
    /// This version is to support functionality in the folding code where
    /// we need to configure the theta values as well.  Note that they are not
    /// stored in _v as full dofs since they are not treated as such.  Instead
    /// they are computed as a function of _v and other bio specific things.
    void Configure(const vector<double>& _v, const vector<double>& _t);

    ////////////////////////////////////////////////////////////////////////////
    /// Compute rendering transforms for robot at @p _v
    /// @param _v Configuration DOF parameters
    void ConfigureRender(const vector<double>& _v);

    ////////////////////////////////////////////////////////////////////////////
    /// Sample random configuration in boundary
    /// @param _boundary Boundary
    vector<double> GetRandomCfg(shared_ptr<Boundary>& _boundary);

    ////////////////////////////////////////////////////////////////////////////
    /// Get the DOF ranges for a given boundary.
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
    /// It is used to set some things that get set by default in the Read
    /// function.
    /// @TODO This function was written for GB, see if we can remove it.
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
