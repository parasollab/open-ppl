#ifndef MULTI_BODY_H_
#define MULTI_BODY_H_

#include "Body.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
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
    /// @brief MultiBody type
    ////////////////////////////////////////////////////////////////////////////
    enum class MultiBodyType {
      Active,       ///< Holonomic Robot
#ifdef PMPState
      NonHolonomic, ///< Nonholonomic Robot
#endif
      Passive,      ///< Visible Obstacle
      Internal,     ///< Invisible Obstacle
      Surface       ///< Surface
    };

    ////////////////////////////////////////////////////////////////////////////
    /// @name Constructors
    /// @{

    MultiBody();

    MultiBody(const MultiBody&) = delete;            ///< No copy
    MultiBody& operator=(const MultiBody&) = delete; ///< No assign

    virtual ~MultiBody();

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name MultiBody Info
    /// @{

    static MultiBodyType GetMultiBodyTypeFromTag(const string& _tag, const string& _where);
    static string GetTagFromMultiBodyType(MultiBodyType _b);

    ////////////////////////////////////////////////////////////////////////////
    /// @return Center of mass
    const Vector3d& GetCenterOfMass() const {return m_com;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Bounding sphere radius
    double GetBoundingSphereRadius() const {return m_radius;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Maximum distance in X, Y, or Z direction
    double GetMaxAxisRange() const {return m_maxAxisRange;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Bounding Box
    const double * GetBoundingBox() const {return m_boundingBox.data();}

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Collision Detection
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Build CD representation
    void BuildCDStructure();

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name I/O
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read MultiBody from stream and compute information
    /// @param _is Input stream
    /// @param _cbs Counting stream buffer
    virtual void Read(istream& _is, CountingStreamBuffer& _cbs) = 0;
    ////////////////////////////////////////////////////////////////////////////
    /// @param _os Output stream
    virtual void Write(ostream & _os) = 0;

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @param _body Body to add
    void AddBody(const shared_ptr<Body>& _body);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute information, including center of mass, sphere, box, and
    ///        range
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
