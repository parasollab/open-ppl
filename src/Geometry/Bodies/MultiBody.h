#ifndef MULTI_BODY_H_
#define MULTI_BODY_H_

#include "Body.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Geometry
/// @brief A collection of geometries in workspace reprenting, e.g., robots
///
/// A MultiBody represent a Obstacle or a Robot in workspace. MultiBody contain
/// one or more Body s, either FixedBody s or FreeBody s. Many access methods
/// are implemented to allow client access internal information about MultiBody
/// instance, like number of Body s, Fixed and Free, Bounding box, center of
/// mass, surface area size, bounding sphere radius, etc.
////////////////////////////////////////////////////////////////////////////////
class MultiBody {

  public:

    ////////////////////////////////////////////////////////////////////////////
    /// The types of MultiBody that we can support.
    ////////////////////////////////////////////////////////////////////////////
    enum class MultiBodyType {
      Active,       ///< Holonomic Robot
      NonHolonomic, ///< Nonholonomic Robot
      Passive,      ///< Visible Obstacle
      Internal      ///< Invisible Obstacle
    };

    ///@name Construction
    ///@{

    MultiBody();

    MultiBody(const MultiBody&) = delete;            ///< No copy
    MultiBody& operator=(const MultiBody&) = delete; ///< No assign

    virtual ~MultiBody();

    ///@}
    ///@name MultiBody Info
    ///@{

    /// Parse a string into a MultiBodyType.
    /// @param[in] _tag The string to parse.
    /// @param[in] _where File location info for error reporting.
    /// @return The MultiBodyType described by _tag.
    static MultiBodyType GetMultiBodyTypeFromTag(const string& _tag,
        const string& _where);

    /// Print a MultiBodyType to a string.
    /// @param[in] _b The type to print.
    /// @return A string representation of _b.
    static string GetTagFromMultiBodyType(MultiBodyType _b);

    /// Get the type for this MultiBody.
    virtual MultiBodyType GetType() const noexcept = 0;

    /// Get the center of mass.
    virtual const Vector3d& GetCenterOfMass() const {return m_com;}

    /// Get the bounding sphere radius.
    double GetBoundingSphereRadius() const {return m_radius;}

    /// Get the maximum distance in X, Y, or Z direction.
    double GetMaxAxisRange() const {return m_maxAxisRange;}

    /// Get the bounding box.
    const double* GetBoundingBox() const {return m_boundingBox.data();}

    ///@}
    ///@name Collision Detection
    ///@{

    /// Build collision detection representations.
    void BuildCDStructure();

    ///@}
    ///@name I/O
    ///@{

    /// Read a MultiBody from an input stream and compute information.
    /// @param[in] _is The input stream to read from.
    /// @param[in] _cbs Counting stream buffer
    virtual void Read(istream& _is, CountingStreamBuffer& _cbs) = 0;

    /// Write the MultiBody to an output stream.
    /// @param _os The output stream to write to.
    virtual void Write(ostream & _os) = 0;

    ///@}
    ///@name Modifiers
    ///@{

    /// Add a body.
    /// @param _body The body to add.
    void AddBody(const shared_ptr<Body>& _body);

    ///@}

    /// Compute center of mass, boundaries, and range.
    /// @warning This function is wrong for COM and boundaries - the only thing
    ///          it computes correctly is the min and max bounding radii. The
    ///          problem is that it performs a one-time computation of the COM
    ///          and bbx, but these things should change as active bodies change
    ///          configuration.
    void FindMultiBodyInfo();

  protected:

    MultiBodyType m_multiBodyType;     ///< MultiBody type

  private:

    vector<shared_ptr<Body>> m_bodies; ///< All bodies

    Vector3d m_com;                    ///< Center of mass
    double m_radius;                   ///< Bounding Sphere
    array<double, 6> m_boundingBox;    ///< Bounding Box
    double m_maxAxisRange;             ///< Max axis range

    string m_modelDataDir;             ///< Directory of environment file
};

#endif
