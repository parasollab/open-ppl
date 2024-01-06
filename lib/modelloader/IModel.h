#ifndef I_MODEL_H_
#define I_MODEL_H_

#include <utility>
#include <vector>

#include "Vector.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

////////////////////////////////////////////////////////////////////////////////
/// \brief A loading interface for all classes which contain polygonal data.
////////////////////////////////////////////////////////////////////////////////
class IModel {

  public:

    ///\name Local Types
    ///@{

    using Point3d = mathtool::Point3d;
    using Vector2d = mathtool::Vector2d;
    using Vector3d = mathtool::Vector3d;
    using CGALPoint = CGAL::Exact_predicates_exact_constructions_kernel::Point_3;

    typedef mathtool::Vector<int, 3>   Tri;

    typedef std::vector<Tri>           TriVector;
    typedef std::vector<Point3d>       PtVector;
    typedef std::vector<Vector2d>      PtVector2;
    typedef std::vector<CGALPoint>     CGALPtVector;

    typedef std::pair<Point3d,Point3d> Edge;
    typedef std::vector<Edge>          EdgeList;

    ///@}
    ///\name Construction
    ///@{

    virtual ~IModel() = default;

    ///@}
    ///\name Derived Class Interface
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Parse the model file.
    /// \param[in] _silent Display error messages?
    /// \return True on success, false otherwise.
    virtual bool ParseFile(bool _silent = false) = 0;

    ///@}
    ///\name Accessors
    ///@{

    virtual void SetDataFileName(const std::string& _fn) {m_filename = _fn;}

    const PtVector& GetVertices() const {return m_points;}
    const PtVector& GetNormals() const {return m_normals;}
    const PtVector2& GetTextureCoords() const {return m_textures;}
    const TriVector& GetTriP() const {return m_triP;}
    const TriVector& GetTriN() const {return m_triN;}
    const TriVector& GetTriT() const {return m_triT;}
    const CGALPtVector& GetCGALVertices() const {return m_cgalPoints;}

    PtVector& GetVertices() {return m_points;}
    PtVector& GetNormals() {return m_normals;}
    PtVector2& GetTextureCoords() {return m_textures;}
    TriVector& GetTriP() {return m_triP;}
    TriVector& GetTriN() {return m_triN;}
    TriVector& GetTriT() {return m_triT;}
    CGALPtVector& GetCGALVertices() {return m_cgalPoints;}

    PtVector& GetUniquePts() {return m_uniquePts;}
    EdgeList& GetEdgeList() {return m_edgeList;}

    void setRotRAD(double _rotR) {m_rot = _rotR;}
    double getRotRAD() const {return m_rot;}

    const Vector3d& GetDir() const {return m_dir;}
    Vector3d& GetDir() {return m_dir;}

    double getDistanceChange() const {return m_distanceChange;}

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Get the radius to a joint in the xz-plane.
    /// \param[in] _el The edge list holding the joint of interest.
    /// \param[in] _jointIndex The joint's index in the edge list.
    /// \param[in] _useFirst Use the first or second point on the edge?
    /// \return The distance from m_center to the checked endpoint.
    double getRadiusToJoint(const EdgeList& _el, int _jointIndex, bool _useFirst)
        const;

    double getBBXRadius() const; ///< Get the xz-plane bbx radius.

    ///@}
    ///\name Model Computations
    ///@{

    void resetBBX();                   ///< Initialize an empty bounding box.
    void updateBBX(const Point3d& _p); ///< Expand the bbx to contain _p.
    void skelBBX(const EdgeList& _el); ///< Create the bbx around points in _el.

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Compute m_uniquePts by filtering out vertices that are closer
    ///        than m_tolerance to eachother.
    void genUniquePts();

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Compute m_dir, m_directionChange, and m_distanceChange from the
    ///        _index-th elements of _el1 and _el2.
    void genDir(EdgeList& _el1, EdgeList& _el2, size_t _index);

    void ComputeFaceNormal();

    ///@}

    //void transform(EdgeList& el, double radius, double height);

  protected:

    ///\name Internal State
    ///@{

    PtVector m_uniquePts;       ///< A filtered point list, excluding nearby pts.
    EdgeList m_edgeList;        ///< The list of edges between points.
    std::vector<double> m_bbx;  ///< The maximum xyz ranges for this model.
    Point3d m_center;           ///< The model center.
    Vector3d m_dir;             ///< ?
    Vector3d m_directionChange; ///< ?
    double m_distanceChange;    ///< ?
    double m_rot;               ///< Rotation in radians.

    std::string m_filename;     ///< The filename from which the model was loaded.

    PtVector m_points;          ///< Model vertices.
    PtVector m_normals;         ///< Model normals.
    PtVector2 m_textures;       ///< Model textures.

    TriVector m_triP;           ///< Triangle points indexes.
    TriVector m_triN;           ///< Triangle normal indexes.
    TriVector m_triT;           ///< Triangle texture indexes.

    CGALPtVector m_cgalPoints;  ///< Exact model points using CGAL.

    static constexpr double m_tolerance{.1}; ///< Uniqueness tolerance.

    ///@}
};

#endif
