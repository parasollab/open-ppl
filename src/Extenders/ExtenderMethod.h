#ifndef EXTENDER_METHOD_H_
#define EXTENDER_METHOD_H_

#include <string>
#include <iostream>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Base algorithm abstraction for \ref Extenders.
///
/// ExtenderMethod has one main method, @c Extend, to grow a simple path from a
/// starting node in some input direction - note that not all expansion
/// methods go in straight lines through @cspace.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ExtenderMethod : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ExtenderMethod() {}
    ExtenderMethod(MPProblemType* _problem, XMLNodeReader& _node)
      : MPBaseObject<MPTraits>(_problem, _node) {}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Extends a path from an input configuration towards a given
    ///        direction
    /// @param _nearest Initial configuration to grow from
    /// @param _dir Direction configuration to grow to
    /// @param _new Placeholder for resulting configuration
    /// @param _innerNodes Placeholder for polygonal chain configurations for
    ///        non-straight-line extention operations
    /// @return Success/fail for extention operation
    ///
    /// @usage
    /// @code
    /// ExtenderPointer e = this->GetMPProblem()->GetExtender(m_eLabel);
    /// CfgType c, cDir, cNew;
    /// vector<CfgType> intermediates;
    /// bool pass = e->Extend(c, cDir, cNew, intermediates);
    /// @endcode
    ////////////////////////////////////////////////////////////////////////////
    virtual bool Extend(const CfgType& _nearest, const CfgType& _dir,
        CfgType& _new, vector<CfgType>& _innerNodes) =0;
};

#endif
