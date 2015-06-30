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
#include "MPProblem/Geometry/Connection.h"
#include "MPProblem/Geometry/GMSPolyhedron.h"

class CollisionDetectionMethod;
class DHparameters;
class MultiBody;

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
    /// @return Bounding sphere radius
    double GetBoundingSphereRadius() const;
    ////////////////////////////////////////////////////////////////////////////
    /// @return Inside sphere radius
    double GetInsideSphereRadius() const;

    ////////////////////////////////////////////////////////////////////////////
    /// @param _worldTransformation Transformation w.r.t. world frame
    void PutWorldTransformation(Transformation& _worldTransformation){m_worldTransformation = _worldTransformation;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Set the world polyhedron based upon world tranfromation
    void ChangeWorldPolyhedron();

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
    /// @brief Determind bounding box
    void FindBoundingBox();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Build appropriate collision detection models
    void BuildCDStructure(CollisionDetectionMethod* _cdMethod);

    ////////////////////////////////////////////////////////////////////////////
    /// @param _v Vertex
    /// @return True if \p _v is a convex hull vertex of Body
    bool IsConvexHullVertex(const Vector3d& _v);

    static string m_modelDataDir; ///< Directory of geometry files

#ifdef USE_VCLIP
    ////////////////////////////////////////////////////////////////////////////
    /// @return VClip model
    shared_ptr<PolyTree> GetVClipBody() {return vclipBody;}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _vclipBody VClip model
    void SetVClipBody(const shared_ptr<PolyTree>& _vclipBody) {vclipBody = _vclipBody;}
#endif
#ifdef USE_RAPID
    ////////////////////////////////////////////////////////////////////////////
    /// @return RAPID model
    shared_ptr<RAPID_model> GetRapidBody() {return rapidBody;}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _rapidBody Rapid model
    void SetRapidBody(const shared_ptr<RAPID_model>& _rapidBody) {rapidBody = _rapidBody;}
#endif
#ifdef USE_PQP
    ////////////////////////////////////////////////////////////////////////////
    /// @return PQP model
    shared_ptr<PQP_Model> GetPQPBody() {return pqpBody;}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _pqpBody PQP model
    void SetPQPBody(const shared_ptr<PQP_Model>& _pqpBody) {pqpBody = _pqpBody;}
#endif
#ifdef USE_SOLID
    ////////////////////////////////////////////////////////////////////////////
    /// @return Solid model
    shared_ptr<DT_ObjectHandle> GetSolidBody() {return solidBody;}
    ////////////////////////////////////////////////////////////////////////////
    /// @param _solidBody Solid model
    void SetSolidBody(const shared_ptr<DT_ObjectHandle>& _solidBody) {solidBody = _solidBody;}
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Special function for SOLID
    void UpdateVertexBase();
#endif

  protected:

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute convex hull of body
    void ComputeConvexHull();

    string m_filename;                       ///< Geometry filename
    MultiBody* m_multibody;                  ///< Owner of Body
    Transformation m_worldTransformation;    ///< World Transformation
    int m_label;                             ///< Body ID

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
};

#endif
