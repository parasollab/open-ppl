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
///   - Two FreeBody instances
///   - Transform from frame of body1 to DH-Frame
///   - DHParameter
///   - Transform from DH-Frame to frame of body2
////////////////////////////////////////////////////////////////////////////////
class Connection {
  public:


    ////////////////////////////////////////////////////////////////////////////
    /// @name Constructors
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _owner MultiBody who owns this Connection
    Connection(MultiBody* _owner);

    /// @}
    ////////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////////
    /// @name Joint Information
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Global connection index
    size_t GetGlobalIndex() const {return m_globalIndex;}

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
    /// @return DH frame description
    const DHParameters& GetDHParameters() const {return m_dhParameters;}
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

    /// @}
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    /// @name I/O
    /// @{

    ////////////////////////////////////////////////////////////////////////////
    /// @param _is Stream
    /// @param _cbs Counting stream buffer
    void Read(istream& _is, CountingStreamBuffer& _cbs);

    friend ostream& operator<<(ostream& _os, const Connection& _c);

    /// @}
    ////////////////////////////////////////////////////////////////////////////

  private:
    MultiBody* m_multibody;                   ///< Owner of this Connection
    shared_ptr<FreeBody> m_bodies[2];         ///< (previous body, next body)
    Transformation m_transformationToBody2;   ///< Transform to second body
    Transformation m_transformationToDHFrame; ///< Transform to DH frame
    DHParameters m_dhParameters;              ///< DH frame description
    DHParameters m_dhRenderParameters;        ///< DH Rendering parameters

    size_t m_globalIndex;                     ///< Global ID
    pair<size_t, size_t> m_bodyIndices;       ///< (previous body, next body)

    static size_t m_globalCounter;            ///< Global ID counter
};

#endif


