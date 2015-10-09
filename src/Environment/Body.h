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

#include "Transformation.h"
using namespace mathtool;

#include "GMSPolyhedron.h"
#include "Utilities/Color.h"
#include "Utilities/MPUtils.h"

class CollisionDetectionMethod;
class MultiBody;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief Single geometric body in the workspace
///
/// Body is a high level representation of a workspace object.
/// Body s essentially encapsulate the geometry of the body (including boundary
/// information), collision detection models, and contain methods to modify
/// transformations of them.
////////////////////////////////////////////////////////////////////////////////
class Body {
  public:

    ////////////////////////////////////////////////////////////////////////////
    /// @name Constructors
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _owner Owner of this body
    Body(MultiBody* _owner);

    Body(const Body& _other) = delete;            ///< No copy
    Body& operator=(const Body& _other) = delete; ///< No assign

    virtual ~Body();

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Body Information
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Owner of this body
    MultiBody* GetMultiBody() {return m_multibody;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Filename of Body
    const string& GetFileName() const { return m_filename; }

    ////////////////////////////////////////////////////////////////////////////
    /// @return Label of body
    int GetLabel() { return m_label; };
    ////////////////////////////////////////////////////////////////////////////
    /// @param _label New label for this body
    void SetLabel(const int _label) { m_label = _label; };

    ////////////////////////////////////////////////////////////////////////////
    /// @return Is color loaded as option?
    bool IsColorLoaded() const { return m_colorLoaded; };
    ////////////////////////////////////////////////////////////////////////////
    /// @return Color
    const Color4& GetColor() const { return m_color; };
    ////////////////////////////////////////////////////////////////////////////
    /// @return Is color loaded as option?
    bool IsTextureLoaded() const { return m_textureLoaded; };
    ////////////////////////////////////////////////////////////////////////////
    /// @return Texture filename
    const string& GetTexture() const { return m_textureFile; };

    ////////////////////////////////////////////////////////////////////////////
    /// @return Center of mass of body
    Vector3d GetCenterOfMass();
    ////////////////////////////////////////////////////////////////////////////
    /// @return Bounding sphere radius
    double GetBoundingSphereRadius() const;
    ////////////////////////////////////////////////////////////////////////////
    /// @return Inside sphere radius
    double GetInsideSphereRadius() const;

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Model
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _poly New polyhedron to define this body
    /// 
    /// This code is used in GB. If touched, someone in GB should verify change.
    void SetPolyhedron(GMSPolyhedron& _poly);
    ////////////////////////////////////////////////////////////////////////////
    /// @return Polyhedron in model coordinates
    GMSPolyhedron& GetPolyhedron() {return m_polyhedron;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Polyhedron in world coordinates
    GMSPolyhedron& GetWorldPolyhedron();
    ////////////////////////////////////////////////////////////////////////////
    /// @return BoundingBox in model coordinates
    GMSPolyhedron& GetBoundingBoxPolyhedron() {return m_bbPolyhedron;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Bounding box in world coordinates
    GMSPolyhedron& GetWorldBoundingBox();

    ////////////////////////////////////////////////////////////////////////////
    /// @return Bounding box
    double* GetBoundingBox() {return m_boundingBox;}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _v Vertex
    /// @return True if \p _v is a convex hull vertex of Body
    bool IsConvexHullVertex(const Vector3d& _v);

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Transformation
    /// @{

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

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Collision Detection Models
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Build appropriate collision detection models
    void BuildCDStructure(CollisionDetectionMethod* _cdMethod);

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

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name I/O
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read geometry information from file
    void Read();

    static string m_modelDataDir; ///< Directory of geometry files

    /// @}
    ////////////////////////////////////////////////////////////////////////////
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

  protected:
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read optional color or texture
    /// @param _is Input stream
    /// @param _cbs Counting buffer stream
    void ReadOptions(istream& _is, CountingStreamBuffer& _cbs);


    ////////////////////////////////////////////////////////////////////////////
    /// @brief Determine bounding box
    void FindBoundingBox();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute convex hull of body
    void ComputeConvexHull();

    MultiBody* m_multibody;                  ///< Owner of Body
    string m_filename;                       ///< Geometry filename
    int m_label;                             ///< Body ID

    bool m_colorLoaded;                      ///< Was color option set
    Color4 m_color;                          ///< Optionally specified color
    bool m_textureLoaded;                    ///< Was texture option set
    string m_textureFile;                    ///< Optionally specified texture

    Transformation m_worldTransformation;    ///< World Transformation
    GMSPolyhedron m_polyhedron;              ///< Model in model coordinates
    bool m_worldPolyhedronAvailable;         ///< Is world polyhedron available
    GMSPolyhedron m_worldPolyhedron;         ///< Model in world coordinates
    GMSPolyhedron m_bbPolyhedron;            ///< Bounding polyhedron
    GMSPolyhedron m_bbWorldPolyhedron;       ///< Bounding polyhedron in world

    bool m_convexHullAvailable;              ///< Is convex hull computed
    GMSPolyhedron m_convexHull;              ///< Convex hull of model

    bool m_centerOfMassAvailable;            ///< Is center of mass computed
    Vector3d m_centerOfMass;                 ///< Center of mass

    double m_boundingBox[6];                 ///< Bounding box

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
