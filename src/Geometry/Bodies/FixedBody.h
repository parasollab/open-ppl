#ifndef FIXED_BODY_H_
#define FIXED_BODY_H_

#include "Body.h"
#include "Utilities/XMLNode.h"

////////////////////////////////////////////////////////////////////////////////
/// A stationary object in workspace.
/// @ingroup Geometry
////////////////////////////////////////////////////////////////////////////////
class FixedBody : public Body {

  public:

    ///@name Construction
    ///@{

    /// @param _owner The owning multibody.
    /// @param _filename Filename
    FixedBody(MultiBody* _owner, const string& _filename = "");

    /// Construct base class from XML node.
    /// @param _owner The owning multibody.
    /// @param _node The XML node containing the body information.
    FixedBody(MultiBody* _owner, XMLNode& _node);

    FixedBody(const FixedBody&) = delete;            ///< No copy
    FixedBody& operator=(const FixedBody&) = delete; ///< No assign

    virtual ~FixedBody() = default;

    ///@}
    ///@name Transformation
    ///@{

    virtual const Transformation& GetWorldTransformation() const override;

    /// Set the transform for this object.
    /// @param _worldTransformation Transformation w.r.t. world frame
    void PutWorldTransformation(const Transformation& _worldTransformation);

    ///@}
    ///@name I/O
    ///@{

    using Body::Read;

    /// @brief Parse XML node.
    /// @param _node XMLNode to parse.
    void ReadXML(XMLNode& _node);

    /// @brief Parse
    /// @param _is Stream
    /// @param _cbs Counting stream buffer
    void Read(istream& _is, CountingStreamBuffer& _cbs);

    friend ostream& operator<<(ostream& _os, const FixedBody& _fb);

    /// @}
};

#endif
