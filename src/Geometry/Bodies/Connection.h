#ifndef CONNECTION_H
#define CONNECTION_H

#include <memory>
using namespace std;

#include <Transformation.h>
using namespace mathtool;

#include "DHParameters.h"
#include "Utilities/IOUtils.h"
#include "Geometry/Boundaries/Range.h"

class FreeBody;
class MultiBody;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief Connection information between two FreeBody s in a robot
///
/// This class stores information about connection from one body to another one.
/// The information stored in this class includes:
///   - Connection type
///   - Two FreeBody instances
///   - Transform from frame of body1 to DH-Frame
///   - DHParameter
///   - Transform from DH-Frame to frame of body2
////////////////////////////////////////////////////////////////////////////////
class Connection final {

  public:

    ///@name Local Types
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// The supported connection types.
    ////////////////////////////////////////////////////////////////////////////
    enum class JointType {
      Revolute,   ///< 1 DOF
      Spherical,  ///< 2 DOF
      NonActuated ///< 0 DOF
    };

    ///@}
    ///@name Construction
    ///@{

    /// @param _owner MultiBody who owns this Connection
    Connection(MultiBody* _owner);

    ///@}
    ///@name I/O
    ///@{

    /// Parse the connection type from a string tag.
    /// @param _tag The string tag to parse.
    /// @param _where Location of \p _tag for error reporting.
    /// @return The joint type represented by _tag.
    static JointType GetJointTypeFromTag(const string& _tag,
        const string& _where);

    /// Create a string tag for a given connection type.
    /// @param _j Joint type
    /// @return String representation of _j.
    static string GetTagFromJointType(JointType _j);

    /// Parse connection info from an XML node.
    /// @param _node The input XML node to read.
    void ReadXML(XMLNode& _node);

    /// Parse connection info from an input stream.
    /// @param _is The input stream to read.
    /// @param _cbs The input counting stream buffer.
    void Read(istream& _is, CountingStreamBuffer& _cbs);

    friend ostream& operator<<(ostream& _os, const Connection& _c);

    ///@}
    ///@name Joint Information
    ///@{

    /// @return Global connection index
    size_t GetGlobalIndex() const;

    /// @return Connection type
    JointType GetConnectionType() const;

    /// @return Joint Limits
    const pair<double, double>& GetJointLimits(size_t _i) const;

    ///@}
    ///@name FreeBody information
    ///@{

    /// @return Pointer to first FreeBody
    const FreeBody* GetPreviousBody() const;
    FreeBody* GetPreviousBody();

    /// @return Index of first FreeBody
    size_t GetPreviousBodyIndex() const;

    /// @return Pointer to second FreeBody
    const FreeBody* GetNextBody() const;
    FreeBody* GetNextBody();

    /// @return Index of second FreeBody
    size_t GetNextBodyIndex() const;

    ///@}
    ///@name Transformation information
    ///@{

    /// @return DH frame description
    DHParameters& GetDHParameters();
    const DHParameters& GetDHParameters() const;

    /// @return DH frame description for rendering
    DHParameters& GetDHRenderParameters();

    /// @return Transformation to second body
    Transformation& GetTransformationToBody2();

    /// @return Transformation to second body
    const Transformation& GetTransformationToBody2() const;

    /// @return Transformation to DH frame
    Transformation& GetTransformationToDHFrame();

    /// @return Transformation to DH frame
    const Transformation& GetTransformationToDHFrame() const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    MultiBody* m_multibody;                   ///< Owner of this Connection
    FreeBody* m_bodies[2];                    ///< (previous body, next body)
    Transformation m_transformationToBody2;   ///< Transform to second body
    Transformation m_transformationToDHFrame; ///< Transform to DH frame
    DHParameters m_dhParameters;              ///< DH frame description
    DHParameters m_dhRenderParameters;        ///< DH Rendering parameters

    size_t m_globalIndex;                     ///< Global ID
    JointType m_jointType;                    ///< Type of connection
    pair<size_t, size_t> m_bodyIndices;       ///< (previous body, next body)
    pair<double, double> m_jointLimits[2];    ///< valid range within [-1,1)

    static size_t m_globalCounter;            ///< Global ID counter

    ///@}

};

#endif
