#ifndef CONNECTION_H
#define CONNECTION_H

#include <Transformation.h>
using namespace mathtool;

#include "MPProblem/Geometry/DHparameters.h"
#include "MPProblem/Robot.h"
#include "Utilities/IOUtils.h"

class Body;
class MultiBody;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief Connection information between two Body s in a robot
///
/// This class stores information about connection from one body to another one.
/// The information stored in this class includes:
///   - Connection type
///   - Two Body instances
///   - Transform from frame of body1 to DH-Frame
///   - DHParameter
///   - Transform from DH-Frame to frame of body2
////////////////////////////////////////////////////////////////////////////////
class Connection {
  public:
    enum JointType {REVOLUTE, SPHERICAL, NONACTUATED};

    ////////////////////////////////////////////////////////////////////////////
    /// @name Constructors
    /// @{
    ////////////////////////////////////////////////////////////////////////////
    /// @param _owner MultiBody who owns this Connection
    Connection(MultiBody* _owner);
    ////////////////////////////////////////////////////////////////////////////
    /// @param _body1 First Body
    /// @param _body2 Second Body
    /// @param _transformationToBody2 Transformation to second Body
    /// @param _dhparameters DHFrame description
    /// @param _transformationToDHFrame Transformation to DHFrame
    Connection(const shared_ptr<Body>& _body1, const shared_ptr<Body>& _body2,
        const Transformation& _transformationToBody2,
        const DHparameters& _dhparameters,
        const Transformation& _transformationToDHFrame);
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
    static string GetTagFromJointType(const JointType& _jt);

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
    const pair<double, double>& GetJointLimits(int _i) const {return m_jointLimits[_i];}
    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Body information
    /// @{
    ////////////////////////////////////////////////////////////////////////////
    /// @return Pointer to first Body
    shared_ptr<Body> GetPreviousBody() const {return m_bodies[0];}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Index of first Body
    size_t GetPreviousBodyIndex() const {return m_bodyIndices.first;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Pointer to second Body
    shared_ptr<Body> GetNextBody() const {return m_bodies[1];}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Index of second Body
    size_t GetNextBodyIndex() const {return m_bodyIndices.second;}
    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name Transformation information
    /// @{
    ////////////////////////////////////////////////////////////////////////////
    /// @return DH frame description
    DHparameters& GetDHparameters() {return m_dhParameters;}
    const DHparameters& GetDHparameters() const {return m_dhParameters;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Transformation to second body
    Transformation& GetTransformationToBody2() {return m_transformationToBody2;}
    const Transformation& GetTransformationToBody2() const {return m_transformationToBody2;}
    ////////////////////////////////////////////////////////////////////////////
    /// @return Transformation to DH frame
    Transformation& GetTransformationToDHFrame() {return m_transformationToDHFrame;}
    const Transformation& GetTransformationToDHFrame() const {return m_transformationToDHFrame;}
    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name I/O
    /// @{
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Parse
    /// @param _is Stream
    /// @param _cbs Counting stream buffer
    void Read(istream& _is, CountingStreamBuffer& _cbs);

    friend ostream& operator<<(ostream& _os, const Connection& _c);
    /// @}
    ////////////////////////////////////////////////////////////////////////////

  private:
    MultiBody* m_multibody;                   ///< Owner of this Connection
    shared_ptr<Body> m_bodies[2];             ///< (previous body, next body)
    Transformation m_transformationToBody2;   ///< Transform to second body
    Transformation m_transformationToDHFrame; ///< Transform to DH frame
    DHparameters m_dhParameters;              ///< DH frame description

    size_t m_globalIndex;                     ///< Global ID
    JointType m_jointType;                    ///< Type of connection
    pair<size_t, size_t> m_bodyIndices;       ///< (previous body, next body)
    pair<double, double> m_jointLimits[2];    ///< valid range within [-1,1)

    static size_t m_globalCounter;            ///< Global ID counter
};

#endif


