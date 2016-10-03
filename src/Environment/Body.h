#ifndef BODY_H_
#define BODY_H_

#ifndef NO_PQP
#include <PQP.h>
#endif

#ifndef NO_RAPID
#include <RAPID.H>
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

    ///@name Constructors
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _owner Owner of this body
    Body(MultiBody* _owner) : m_multibody(_owner) { }


    Body(const Body& _other) = delete;            ///< No copy.
    Body& operator=(const Body& _other) = delete; ///< No assign.

    virtual ~Body() = default;

    ///@}
    ///@name Body Information Accessors
    ///@{

    MultiBody* GetMultiBody() {return m_multibody;}
    const string& GetFileName() const {return m_filename;}
    int GetLabel() const {return m_label;};
    void SetLabel(const int _label) {m_label = _label;};

    ///@}
    ///@name Rendering Property Accessors
    ///@{

    bool IsColorLoaded() const {return m_colorLoaded;}
    const Color4& GetColor() const {return m_color;}
    bool IsTextureLoaded() const {return m_textureLoaded;}
    const string& GetTexture() const {return m_textureFile;}

    ///@}
    ///@name Model Property Accessors
    ///@}

    Vector3d GetCenterOfMass();
    GMSPolyhedron::COMAdjust GetCOMAdjust() const {return m_comAdjust;}

    const double GetMass() const {return m_mass;}
    const Matrix3x3& GetMoment() const {return m_moment;}

    double GetBoundingSphereRadius() const {return m_polyhedron.m_maxRadius;}
    double GetInsideSphereRadius() const {return m_polyhedron.m_minRadius;}

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

    GMSPolyhedron& GetBoundingBoxPolyhedron() {return m_bbPolyhedron;}
    GMSPolyhedron& GetWorldBoundingBox();
    double* GetBoundingBox() {return m_boundingBox;}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _v A vertex in model coordinates.
    /// @return True if \p _v is a convex hull vertex of body.
    bool IsConvexHullVertex(const Vector3d& _v);

    ///@}
    ///@name Transformation
    ///@{

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

    ///@}
    ///@name Collision Detection Models
    ///@{

    static vector<CollisionDetectionMethod*> m_cdMethods; ///< All CD Methods

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Build appropriate collision detection models
    void BuildCDStructure();

#ifndef NO_RAPID
    shared_ptr<RAPID_model> GetRapidBody() {return m_rapidBody;}
    void SetRapidBody(const shared_ptr<RAPID_model>& _r) {m_rapidBody = _r;}
#endif

#ifndef NO_PQP
    shared_ptr<PQP_Model> GetPQPBody() {return m_pqpBody;}
    void SetPQPBody(const shared_ptr<PQP_Model>& _p) {m_pqpBody = _p;}
#endif

    ///@}
    ///@name I/O
    ///@{

    static string m_modelDataDir; ///< Directory of geometry files

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read geometry information from file
    /// @param _comAdjust Center of mass adjustment method
    void Read(GMSPolyhedron::COMAdjust _comAdjust);

  protected:

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read optional color or texture
    /// @param _is Input stream
    /// @param _cbs Counting buffer stream
    void ReadOptions(istream& _is, CountingStreamBuffer& _cbs);

    ///@}
    ///@name Computation Helpers
    ///@}

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
    /// @brief Compute a bounding box in world coordinates.
    void ComputeBoundingBox();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute a GMSPolyhedron representation of the bounding box in
    ///        model coordinates.
    void ComputeBoundingPolyhedron();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute convex hull of body
    void ComputeConvexHull();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute the world polyhedron from the model-coordinate version.
    void ComputeWorldPolyhedron();

    ///@}
    ///@name Internal State
    ///@{

    MultiBody* m_multibody;                  ///< Owner of Body
    string m_filename;                       ///< Geometry filename
    int m_label{0};                          ///< Body ID

    bool m_colorLoaded{false};               ///< Was color option set
    Color4 m_color;                          ///< Optionally specified color
    bool m_textureLoaded{false};             ///< Was texture option set
    string m_textureFile;                    ///< Optionally specified texture

    GMSPolyhedron m_polyhedron;              ///< Model in model coordinates
    GMSPolyhedron m_worldPolyhedron;         ///< Model in world coordinates

    bool m_worldPolyhedronAvailable{false};  ///< Is world polyhedron available
    Transformation m_worldTransformation;    ///< World Transformation

    GMSPolyhedron m_bbPolyhedron;            ///< Bounding box in model coords.
    GMSPolyhedron m_bbWorldPolyhedron;       ///< Bounding box in world coords.
    double m_boundingBox[6]{0,0,0,0,0,0};    ///< Another world bounding box.

    bool m_convexHullAvailable{false};       ///< Is convex hull computed
    GMSPolyhedron m_convexHull;              ///< Convex hull of model

    bool m_centerOfMassAvailable{false};     ///< Is center of mass computed
    Vector3d m_centerOfMass;                 ///< Center of mass
    GMSPolyhedron::COMAdjust m_comAdjust{GMSPolyhedron::COMAdjust::COM};
                                             ///< COM Adjustment option

    double m_mass{1};                        ///< Mass of Body
    Matrix3x3 m_moment;                      ///< Moment of Inertia

#ifndef NO_RAPID
    shared_ptr<RAPID_model> m_rapidBody;     ///< RAPID model
#endif

#ifndef NO_PQP
    shared_ptr<PQP_Model> m_pqpBody;         ///< PQP model
#endif

    ///@}

};

#endif
