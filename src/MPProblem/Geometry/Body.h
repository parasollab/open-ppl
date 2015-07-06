#ifndef BODY_H_
#define BODY_H_

#ifdef USE_VCLIP
#include <vclip.h>
#endif
#ifdef USE_PQP
#include <PQP.h>
#endif
#ifdef USE_RAPID
#include <RAPID.H>
#endif
#ifdef USE_SOLID
#include <SOLID.h>
#include "DT_Polyhedron.h"
#include "DT_Polytope.h"
#endif

#include "Utilities/MPUtils.h"
#include "MPProblem/Geometry/GMSPolyhedron.h"
#include "MPProblem/Geometry/Connection.h"
#include "MPProblem/Robot.h"

class MultiBody;
class DHparameters;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief Single geometric body in the workspace
///
/// Body is a high level representation of a workspace object.
/// Body s essentially encapsulate the geometry of the body (including boundary
/// information), collision detection models, and contain methods to modify
/// transformations of them. If this Body instance is a link in articulated
/// MultiBody, then Connection information is also provided.
////////////////////////////////////////////////////////////////////////////////
class Body {
  public:
    ////////////////////////////////////////////////////////////////////////////
    /// @name Constructors
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _owner Owner of this body
    Body(MultiBody* _owner);
    ////////////////////////////////////////////////////////////////////////////
    /// @param _owner Owner of this body
    /// @param _polyhedron Geometry of body
    Body(MultiBody* _owner, GMSPolyhedron& _polyhedron);

    Body(const Body& _other) = delete;
    Body& operator=(const Body& _other) = delete;

    virtual ~Body();

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    string GetFileName() { return m_filename; }

    ////////////////////////////////////////////////////////////////////////////
    /// @return Transformation of this body w.r.t. the world frame
    virtual Transformation& GetWorldTransformation() = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// @return Transformation of this body w.r.t. the world frame
    ///
    /// Return world transformation of this body. If worldTransformation has
    /// been calculated(updated), this method should be used to avoid redundant
    /// calculation.
    Transformation& WorldTransformation() {return m_worldTransformation;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Polyhedron in world coordinates
    GMSPolyhedron& GetWorldPolyhedron();
    ////////////////////////////////////////////////////////////////////////////
    /// @return Bounding box in world coordinates
    GMSPolyhedron& GetWorldBoundingBox();
    ////////////////////////////////////////////////////////////////////////////
    /// @return Polyhedron in model coordinates
    GMSPolyhedron& GetPolyhedron() {return m_polyhedron;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return BoundingBox in model coordinates
    GMSPolyhedron& GetBoundingBoxPolyhedron() {return m_bbPolyhedron;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Owner of this body
    MultiBody* GetMultiBody() {return m_multibody;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Boinding box
    double* GetBoundingBox() {return m_boundingBox;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Center of mass of body
    Vector3d GetCenterOfMass();
    ////////////////////////////////////////////////////////////////////////////
    /// @return Moment of Inertia of body
    const Matrix3x3& GetMoment() {return m_moment;}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _index Index of desired forward Connection
    /// @return Requested forward Connection
    Connection& GetForwardConnection(size_t _index);
    ////////////////////////////////////////////////////////////////////////////
    /// @param _index Index of desired backward Connection
    /// @return Requested backward Connection
    Connection& GetBackwardConnection(size_t _index);

#ifdef USE_VCLIP
    ////////////////////////////////////////////////////////////////////////////
    /// @return VClip model
    shared_ptr<PolyTree> GetVClipBody() {return vclipBody;}
#endif
#ifdef USE_RAPID
    ////////////////////////////////////////////////////////////////////////////
    /// @return RAPID model
    shared_ptr<RAPID_model> GetRapidBody() {return rapidBody;}
#endif
#ifdef USE_PQP
    ////////////////////////////////////////////////////////////////////////////
    /// @return PQP model
    shared_ptr<PQP_Model> GetPQPBody() {return pqpBody;}
#endif
#ifdef USE_SOLID
    ////////////////////////////////////////////////////////////////////////////
    /// @return Solid model
    shared_ptr<DT_ObjectHandle> GetSolidBody() {return solidBody;}
#endif

    ////////////////////////////////////////////////////////////////////////////
    /// @param _worldTransformation Transformation w.r.t. world frame
    void PutWorldTransformation(Transformation& _worldTransformation){m_worldTransformation = _worldTransformation;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Set the world polyhedron based upon world tranfromation
    void ChangeWorldPolyhedron();

    ////////////////////////////////////////////////////////////////////////////
    /// @return Is this body fixed?
    virtual bool IsFixedBody() const = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// @return Is this body a base?
    bool IsBase() { return m_isBase; };
    ////////////////////////////////////////////////////////////////////////////
    /// @return Base type of body
    Robot::Base GetBase() { return m_baseType; };
    ////////////////////////////////////////////////////////////////////////////
    /// @return Base movement type of body
    Robot::BaseMovement GetBaseMovement() { return m_baseMovementType; };
    ////////////////////////////////////////////////////////////////////////////
    /// @param _baseType Type of base of this body
    void SetBase(Robot::Base _baseType) { m_baseType = _baseType; };
    ////////////////////////////////////////////////////////////////////////////
    /// @param _baseMovementType Type of movement of the base of this body
    void SetBaseMovement(Robot::BaseMovement _baseMovementType) { m_baseMovementType = _baseMovementType; };

    ////////////////////////////////////////////////////////////////////////////
    /// @return Label of body
    int GetLabel() { return m_label; };
    ////////////////////////////////////////////////////////////////////////////
    /// @param _label New label for this body
    void SetLabel(const int _label) { m_label = _label; };

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read geometry information from file
    void Read();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Output information of body
    /// @param _os Output stream
    void Write(ostream& _os);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Calculate center of mass in world coordinates
    ///
    /// This function is automatically calld by GetCenterOfMass() if it has
    /// never been computed. After computing it, this function will not be
    /// called again: rigid body. This way of computing center of mass is
    /// physically not true. This assumes that each vertex carries the same
    /// mass, and edges are weightless. To be more accurate, we need to be
    /// modify this to consider the length of edges, which is still an
    /// approximation.
    void ComputeCenterOfMass();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Approximate moment of inertia
    void ComputeMomentOfInertia();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Determind bounding box
    void FindBoundingBox();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Determines if two bodies share the same joint
    /// @param _otherBody Second body
    /// @return True if adjacent
    bool IsAdjacent(shared_ptr<Body> _otherBody);
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Determines if two bodies are within \p _i joints of each other
    /// @param _otherBody Second body
    /// @param _i Number of joints
    /// @return True if within \p _i joints
    bool IsWithinI(shared_ptr<Body> _otherBody, int _i);
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Determines if two bodies are within \p _i joints of each other
    /// @param _body1 First Body
    /// @param _body2 Second Body
    /// @param _i Number of joints
    /// @param _prevBody Previous Body
    /// @return True if within \p _i joints
    bool IsWithinIHelper(Body* _body1, Body* _body2, int _i,Body* _prevBody);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Build appropriate collision detection models
    void BuildCDStructure(cd_predefined _cdtype);
#ifdef USE_SOLID
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Special function for SOLID
    void UpdateVertexBase();
#endif

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of forward Connection
    int ForwardConnectionCount() const {return m_forwardConnection.size();}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of backward Connection
    int BackwardConnectionCount() const {return m_backwardConnection.size();}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _connection Connection to add as forward Connection
    void AddForwardConnection(const Connection& _connection) {m_forwardConnection.push_back(_connection);}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _connection Connection to add as backward Connection
    void AddBackwardConnection(const Connection& _connection) {m_backwardConnection.push_back(_connection);}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Link two Body, i.e., add a Connection between them
    /// @param _otherBody Second body
    /// @param _transformationToBody2 Transformation to second body
    /// @param _dhparameters DH frame description
    /// @param _transformationToDHFrame Transformation to DH frame
    void Link(const shared_ptr<Body>& _otherBody,
        const Transformation& _transformationToBody2,
        const DHparameters& _dhparameters,
        const Transformation& _transformationToDHFrame);
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Link two Body, i.e., add a Connection between them
    /// @param _c Connection description
    void Link(const Connection& _c);

    ////////////////////////////////////////////////////////////////////////////
    /// @param _v Vertex
    /// @return True if \p _v is a convex hull vertex of Body
    bool IsConvexHullVertex(const Vector3d& _v);

    static string m_modelDataDir; ///< Directory of geometry files

  protected:

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute convex hull of body
    void ComputeConvexHull();

    string m_filename;                       ///< Geometry filename
    MultiBody* m_multibody;                  ///< Owner of Body
    Transformation m_worldTransformation;    ///< World Transformation
    bool m_isBase;                           ///< Base or Joint
    int m_label;                             ///< Body ID
    Robot::Base m_baseType;                  ///< Base type
    Robot::BaseMovement m_baseMovementType;  ///< Base movement

    GMSPolyhedron m_polyhedron;              ///< Model in model coordinates
    GMSPolyhedron m_worldPolyhedron;         ///< Model in world coordinates
    GMSPolyhedron m_convexHull;              ///< Convex hull of model
    bool m_convexHullAvailable;              ///< Is convex hull computed
    bool m_centerOfMassAvailable;            ///< Is center of mass computed
    Vector3d m_centerOfMass;                 ///< Center of mass
    bool m_worldPolyhedronAvailable;         ///< Is world polyhedron available
    double m_boundingBox[6];                 ///< Bounding box
    GMSPolyhedron m_bbPolyhedron;            ///< Bounding polyhedron
    GMSPolyhedron m_bbWorldPolyhedron;       ///< Bounding polyhedron in world

    vector<Connection> m_forwardConnection;  ///< Forward Connection s
    vector<Connection> m_backwardConnection; ///< Backward Connection s

#ifdef USE_VCLIP
    shared_ptr<PolyTree> vclipBody;          ///< VClip model
#endif
#ifdef USE_RAPID
    shared_ptr<RAPID_model> rapidBody;       ///< RAPID model
#endif
#ifdef USE_PQP
    shared_ptr<PQP_Model> pqpBody;           ///< PQP model
#endif
#ifdef USE_SOLID
    shared_ptr<DT_ObjectHandle> solidBody;   ///< Solid model
    DT_VertexBaseHandle base;                ///< Solid base
    MT_Point3* vertex;                       ///< Solid set of vertices
#endif

    Matrix3x3 m_moment;
};

#endif
