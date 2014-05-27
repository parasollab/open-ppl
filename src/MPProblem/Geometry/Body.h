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
#include "boost/shared_ptr.hpp"

using boost::shared_ptr;

class MultiBody;
class DHparameters;

/**class Body is a high level represented object in workspace.
 *Body is an abstract class with many basic functions provided.
 *Body instance could read (write) geometirc data from (to) specified file.
 *More information about bounding box and collision detection library internal models
 *could be computed and accessed from member methods.
 *if this Body instance is a link in articulate multibody,
 *connection information is also provided.
 */
class Body {
  public:
    Body(MultiBody* _owner);
    Body(MultiBody* _owner, GMSPolyhedron & _polyhedron);
    Body(const Body& b);

    virtual ~Body();

    bool operator==(const Body& b) const;
    bool operator!=(const Body& b) const { return !(*this == b); }

    string GetFileName() { return m_filename; }

    ///Return transformation of this body in world coordinate.
    virtual Transformation& GetWorldTransformation() = 0;

    /**Return world transformation of this body.
     *If worldTransformation has been calculated(updated), this method should be used
     *to avoid redundant calculation.
     */
    Transformation& WorldTransformation() {return m_worldTransformation;}

    /**This function returns a Polyhedron whose vertices and normals are represented in
     *world coordinate.
     */
    virtual GMSPolyhedron& GetWorldPolyhedron();
    virtual GMSPolyhedron& GetWorldBoundingBox();
    virtual GMSPolyhedron& GetPolyhedron() {return m_polyhedron;}
    virtual GMSPolyhedron& GetBoundingBoxPolyhedron() {return m_bbPolyhedron;}

    MultiBody* GetMultiBody() {return m_multibody;}
    double* GetBoundingBox() {return m_boundingBox;}
    Vector3d GetCenterOfMass();
    Connection& GetForwardConnection(size_t _index);
    Connection& GetBackwardConnection(size_t _index);

    //Get methods used in collision detection
#ifdef USE_VCLIP
    shared_ptr<PolyTree> GetVClipBody() {return vclipBody;}
#endif
#ifdef USE_RAPID
    shared_ptr<RAPID_model> GetRapidBody() {return rapidBody;}
#endif
#ifdef USE_PQP
    shared_ptr<PQP_Model> GetPQPBody() {return pqpBody;}
#endif
#ifdef USE_SOLID
    shared_ptr<DT_ObjectHandle> GetSolidBody() {return solidBody;}
#endif

    /**
      * PutWorldTransformation
      * Function: Assign the given transformation as a transformation
      * w.r.t the world for "this" body
      */
    void PutWorldTransformation(Transformation& _worldTransformation){m_worldTransformation = _worldTransformation;}

    virtual void ChangeWorldPolyhedron();

    virtual int IsFixedBody() = 0;

    bool IsBase() { return m_isBase; };
    Robot::Base GetBase() { return m_baseType; };
    Robot::BaseMovement GetBaseMovement() { return m_baseMovementType; };
    void SetBase(Robot::Base _baseType) { m_baseType = _baseType; };
    void SetBaseMovement(Robot::BaseMovement _baseMovementType) { m_baseMovementType = _baseMovementType; };

    void Read();

    virtual void Write(ostream& _os);

    /**Calculate center of mass of given polyhedron (in world coordinates).
     *
     *This function is automatically calld by GetCenterOfMass()
     *if it has never been computed. After computing it,
     *this function will not be called again: rigid body.
     *
     *This way of computing center of mass is physically not true.
     *This assumes that each vertex carries the same mass, and
     *edges are weightless. To be more accurate, we need to
     *be modify this to consider the length of edges, which is
     *still an approximation.
     *
     *if GetCenterOfMass() is called for the first time,
     *this function calculates it and set 'available' flag
     */
    void ComputeCenterOfMass();
    void FindBoundingBox();

    //to check if two Body share same joint (adjacent) for a robot.
    bool IsAdjacent(shared_ptr<Body> _otherBody);
    bool IsWithinI(shared_ptr<Body> _otherBody, int _i);
    bool IsWithinIHelper(Body* _body1, Body* _body2, int _i,Body* _prevBody);

    /////////////////////////////////////////////////////////////////////////////////
    //    Collision Detection Model methods
    ///////////////////////////////////////////////////////////////////////////////
    void BuildCDStructure(cd_predefined _cdtype);
#ifdef USE_SOLID
    void UpdateVertexBase();
#endif
    ////////////////////////////////////////////////////////////////////////////////
    //    Connection methods
    ////////////////////////////////////////////////////////////////////////////////
    int ForwardConnectionCount() const {return m_forwardConnection.size();}
    int BackwardConnectionCount() const {return m_backwardConnection.size();}
    void AddForwardConnection(Connection& _connection) 
      {m_forwardConnection.push_back(shared_ptr<Connection>(&_connection));}
    void AddBackwardConnection(Connection& _connection) 
      {m_backwardConnection.push_back(shared_ptr<Connection>(&_connection));}
    /**Link
     * Function: Link "this" body to the given other body by setting up a
     * connectionship between them using the given DH parameters and the
     * transformation (from the distal joint of "this" body to the center of
     * gravity of the other).
     */
    void Link(const shared_ptr<Body>& _otherBody, const Transformation& _transformationToBody2,
        const DHparameters& _dhparameters, const Transformation& _transformationToDHFrame);
    /**Link
     * Function: Link "this" body to the given body by using the given
     * connection.  Establish a forward and backward connectionship.
     */
    void Link(Connection& _c);

    bool IsConvexHullVertex(const Vector3d& _v);

    static string m_modelDataDir;

  protected:

    void ComputeConvexHull();

    string m_filename;
    MultiBody* m_multibody;
    Transformation m_worldTransformation;
    bool m_isBase;
    Robot::Base m_baseType;
    Robot::BaseMovement m_baseMovementType;

    GMSPolyhedron m_polyhedron;
    GMSPolyhedron m_worldPolyhedron;
    GMSPolyhedron m_convexHull;
    bool m_convexHullAvailable;
    bool m_centerOfMassAvailable;
    Vector3d m_centerOfMass;
    bool m_worldPolyhedronAvailable;
    double m_boundingBox[6];
    GMSPolyhedron m_bbPolyhedron;
    GMSPolyhedron m_bbWorldPolyhedron;

    vector<shared_ptr<Connection> > m_forwardConnection;
    vector<shared_ptr<Connection> > m_backwardConnection;

#ifdef USE_VCLIP
    shared_ptr<PolyTree> vclipBody;
#endif
#ifdef USE_RAPID
    shared_ptr<RAPID_model> rapidBody;
#endif
#ifdef USE_PQP
    shared_ptr<PQP_Model> pqpBody;
#endif
#ifdef USE_SOLID
    shared_ptr<DT_ObjectHandle> solidBody;
    DT_VertexBaseHandle base;
    MT_Point3* vertex;
#endif

    friend class MultiBody; //Owner
    friend class Connection;
};

#endif
