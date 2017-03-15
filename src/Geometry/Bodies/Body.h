#ifndef BODY_H_
#define BODY_H_

#include <PQP.h>
#include <RAPID.H>

#include "Transformation.h"
using namespace mathtool;

#include "Geometry/GMSPolyhedron.h"
#include "Utilities/Color.h"
#include "Utilities/MPUtils.h"

class CollisionDetectionMethod;
class MultiBody;


////////////////////////////////////////////////////////////////////////////////
/// A single polyhedral body in workspace. One or more of these are composed to
/// form a MultiBody, which is PMPL's working representation of object geometries.
///
/// @details Each Body has two representations: one for the 'model frame' and one
///          for the 'world frame'. The model frame is the coordinate frame in
///          which the Body was originally described - this is usually centered
///          on the Body's center of mass. The world frame is the coordinate
///          frame in our current environment - this represents the Body after
///          it has been translated/oriented into its place in the planning
///          scene.
/// @ingroup Geometry
////////////////////////////////////////////////////////////////////////////////
class Body {

  public:

    ///@name Construction
    ///@{

    /// Construct an empty body.
    /// @param _owner The owning multibody.
    Body(MultiBody* _owner);

    Body(const Body& _other) = delete;            ///< No copy.
    Body& operator=(const Body& _other) = delete; ///< No assign.

    virtual ~Body() = default;

    ///@}
    ///@name Metadata Accessors
    ///@{

    /// Get the owning MultiBody.
    MultiBody* GetMultiBody();

    /// Get the file name from which this body was constructed.
    const string& GetFileName() const;

    /// Get the full path of the file from which this body was constructed.
    string GetFilePath() const;

    ///@}
    ///@name Rendering Property Accessors
    ///@{

    /// Check if a color was loaded.
    bool IsColorLoaded() const;

    /// Get the loaded color.
    const Color4& GetColor() const;

    /// Check if a texture was loaded.
    bool IsTextureLoaded() const;

    /// Get the loaded texture file name.
    const string& GetTexture() const;

    ///@}
    ///@name Model Property Accessors
    ///@}

    /// Get the center-of-mass adjustment option.
    GMSPolyhedron::COMAdjust GetCOMAdjust() const;

    /// Get the mass.
    double GetMass() const;

    /// Get the moment of inertia in the model frame.
    const Matrix3x3& GetMoment() const;

    /// Get the bounding radius as measured from the body's center point.
    double GetBoundingSphereRadius() const;

    /// Get the minimum radius as measured from the body's center point.
    double GetInsideSphereRadius() const;

    /// Set the polyhedron model for this body.
    void SetPolyhedron(GMSPolyhedron& _poly);

    /// Get the polyhedron in model coordinates.
    const GMSPolyhedron& GetPolyhedron() const;

    /// Get the polyhedron in world coordinates.
    const GMSPolyhedron& GetWorldPolyhedron() const;

    /// Get the bounding box in model coordinates.
    const GMSPolyhedron& GetBoundingBox() const;

    /// Compute the bounding box in world coordinates.
    GMSPolyhedron GetWorldBoundingBox() const;

    /// Test if a point is a convex hull vertex of this body's polyhedron in
    /// model coordinates.
    /// @param _v A vertex in model coordinates.
    /// @return True if \p _v is a convex hull vertex of body.
    bool IsConvexHullVertex(const Vector3d& _v) const;

    ///@}
    ///@name Transformation
    ///@{

    /// @return Transformation of this body w.r.t. the world frame
    virtual const Transformation& GetWorldTransformation() const = 0;

    /// Mark all cached objects as requiring an update.
    void MarkDirty() const;

    ///@}
    ///@name Collision Detection Models
    ///@{

    static vector<CollisionDetectionMethod*> m_cdMethods; ///< All CD Methods

    /// Build appropriate collision detection models.
    void BuildCDStructure();

    RAPID_model* GetRapidBody() const {return m_rapidBody.get();}
    void SetRapidBody(unique_ptr<RAPID_model>&& _r) {m_rapidBody = move(_r);}

    PQP_Model* GetPQPBody() const {return m_pqpBody.get();}
    void SetPQPBody(unique_ptr<PQP_Model>&& _p) {m_pqpBody = move(_p);}

    ///@}
    ///@name I/O
    ///@{

    static string m_modelDataDir; ///< Directory of geometry files

    /// Read geometry information from file.
    /// @param _comAdjust Center of mass adjustment method
    void Read(GMSPolyhedron::COMAdjust _comAdjust);

  protected:

    /// Read optional color or texture.
    /// @param _is Input stream
    /// @param _cbs Counting buffer stream
    void ReadOptions(istream& _is, CountingStreamBuffer& _cbs);

    ///@}
    ///@name Computation Helpers
    ///@}

    /// Approximate moment of inertia in model coordinates.
    void ComputeMomentOfInertia() const;

    /// Compute the axis-aligned bounding box in model coordinates.
    void ComputeBoundingBox() const;

    /// Compute the convex hull in model coordinates.
    void ComputeConvexHull() const;

    /// Compute the world-coordinate polyhedron by applying the world transform
    /// to the model-coordinate version.
    void ComputeWorldPolyhedron() const;

    ///@}
    ///@name Internal State
    ///@{

    MultiBody* m_multibody;                  ///< The owning MultBody.
    string m_filename;                       ///< Geometry filename.

    Color4 m_color;                          ///< Optionally specified color
    bool m_colorLoaded{false};               ///< Was color option set?

    string m_textureFile;                    ///< Optionally specified texture
    bool m_textureLoaded{false};             ///< Was texture option set?

    GMSPolyhedron m_polyhedron;              ///< Model in model coordinates.
    GMSPolyhedron m_boundingBox;             ///< AABB in model coordinates.
    GMSPolyhedron m_convexHull;              ///< Convex hull in model frame.
    mutable bool m_convexHullCached{false};  ///< Is convex hull cached?

    Transformation m_transform;         ///< Transform from model to world frame.
    mutable bool m_transformCached{false};    ///< Is the transform cached?

    GMSPolyhedron m_worldPolyhedron;         ///< Model in world coordinates.
    mutable bool m_worldPolyhedronCached{false}; ///< Is world polyhedron cached?

    double m_mass{1};                        ///< Mass of Body
    Matrix3x3 m_moment;                      ///< Moment of Inertia
    GMSPolyhedron::COMAdjust m_comAdjust{GMSPolyhedron::COMAdjust::COM};
                                             ///< COM Adjustment option

    unique_ptr<RAPID_model> m_rapidBody;     ///< RAPID model
    unique_ptr<PQP_Model> m_pqpBody;         ///< PQP model

    ///@}

};

#endif
