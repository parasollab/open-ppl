#ifndef MULTI_BODY_H_
#define MULTI_BODY_H_

#include "Body.h"

////////////////////////////////////////////////////////////////////////////////
/// A geometric object in workspace (such as an obstacle or robot) with one or
/// more sub-components, referred to as Bodies.
/// @ingroup Geometry
////////////////////////////////////////////////////////////////////////////////
class MultiBody {

  public:

    ///@name Local Types
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// The types of MultiBody that we can support.
    ////////////////////////////////////////////////////////////////////////////
    enum class MultiBodyType {
      Active,       ///< Movable object.
      Passive,      ///< Static visible object.
      Internal      ///< Static invisible object.
    };

    ///@}
    ///@name Construction
    ///@{

    MultiBody();

    MultiBody(const MultiBody&) = delete;            ///< No copy
    MultiBody& operator=(const MultiBody&) = delete; ///< No assign

    virtual ~MultiBody() = default;

    ///@}
    ///@name Body Accessors
    ///@{

    size_t GetNumBodies() const noexcept;

    /// Get an internal body.
    Body* GetBody(const size_t _i) noexcept;
    const Body* GetBody(const size_t _i) const noexcept;

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

    ///@name Internal State
    ///@{

    MultiBodyType m_multiBodyType;     ///< MultiBody type

  private:

    vector<shared_ptr<Body>> m_bodies; ///< All bodies

    Vector3d m_com;                    ///< Center of mass
    double m_radius;                   ///< Bounding Sphere
    array<double, 6> m_boundingBox;    ///< Bounding Box
    double m_maxAxisRange;             ///< Max axis range

    ///@}
};

#endif
