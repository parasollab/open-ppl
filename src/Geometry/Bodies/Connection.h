#ifndef CONNECTION_H
#define CONNECTION_H

#include <memory>
using namespace std;

#include <Transformation.h>
using namespace mathtool;

#include "DHParameters.h"
#include "Utilities/IOUtils.h"

class FreeBody;
class MultiBody;

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
class Connection {
  public:

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Connection type
    ////////////////////////////////////////////////////////////////////////////
    enum class JointType {
      Revolute,   ///< 1 DOF
      Spherical,  ///< 2 DOF
      NonActuated ///< 0 DOF
    };

    ////////////////////////////////////////////////////////////////////////////
    /// @name Constructors
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _owner MultiBody who owns this Connection
    Connection(MultiBody* _owner);

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @param _tag String to parse
    /// @param _where Location of \p _tag for error reporting
    /// @return Joint type
    static JointType GetJointTypeFromTag(const string& _tag,
        const string& _where);
    ////////////////////////////////////////////////////////////////////////////
    /// @param _jt Joint type
    /// @return String representing joint type
    static string GetTagFromJointType(JointType _jt);

    ////////////////////////////////////////////////////////////////////////////
    /// @name Joint Information
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Global connection index
    size_t GetGlobalIndex() const {return m_globalIndex;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Connection type
    JointType GetConnectionType() const {return m_jointType;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Connection type
    const pair<double, double>& GetJointLimits(size_t _i) const {return m_jointLimits[_i];}

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name FreeBody information
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Pointer to first FreeBody
    shared_ptr<FreeBody> GetPreviousBody() const {return m_bodies[0];}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Index of first FreeBody
    size_t GetPreviousBodyIndex() const {return m_bodyIndices.first;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Pointer to second FreeBody
    shared_ptr<FreeBody> GetNextBody() const {return m_bodies[1];}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Index of second FreeBody
    size_t GetNextBodyIndex() const {return m_bodyIndices.second;}

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Transformation information
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @return DH frame description
    DHParameters& GetDHParameters() {return m_dhParameters;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return DH frame description for rendering
    DHParameters& GetDHRenderParameters() {return m_dhRenderParameters;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Transformation to second body
    Transformation& GetTransformationToBody2() {return m_transformationToBody2;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Transformation to second body
    const Transformation& GetTransformationToBody2() const {return m_transformationToBody2;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Transformation to DH frame
    Transformation& GetTransformationToDHFrame() {return m_transformationToDHFrame;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Transformation to DH frame
    const Transformation& GetTransformationToDHFrame() const {return m_transformationToDHFrame;}

    ///@}
    ///@name I/O
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read in the connection info from an input stream.
    /// @param _is Stream
    /// @param _cbs Counting stream buffer
    void Read(istream& _is, CountingStreamBuffer& _cbs);

    friend ostream& operator<<(ostream& _os, const Connection& _c);

    ///@}

  private:

    MultiBody* m_multibody;                   ///< Owner of this Connection
    shared_ptr<FreeBody> m_bodies[2];         ///< (previous body, next body)
    Transformation m_transformationToBody2;   ///< Transform to second body
    Transformation m_transformationToDHFrame; ///< Transform to DH frame
    DHParameters m_dhParameters;              ///< DH frame description
    DHParameters m_dhRenderParameters;        ///< DH Rendering parameters

    size_t m_globalIndex;                     ///< Global ID
    JointType m_jointType;                    ///< Type of connection
    pair<size_t, size_t> m_bodyIndices;       ///< (previous body, next body)
    pair<double, double> m_jointLimits[2];    ///< valid range within [-1,1)

    static size_t m_globalCounter;            ///< Global ID counter
};

#endif


