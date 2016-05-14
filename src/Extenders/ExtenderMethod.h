#ifndef EXTENDER_METHOD_H_
#define EXTENDER_METHOD_H_

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Base algorithm abstraction for \ref Extenders.
/// @tparam MPTraits Motion planning universe
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
    ExtenderMethod(MPProblemType* _problem, XMLNode& _node)
      : MPBaseObject<MPTraits>(_problem, _node) {}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Maximum extension distance
    virtual double GetDelta() const = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Extends a path from an input configuration towards a given
    ///        direction
    /// @param _nearest Initial configuration to grow from
    /// @param _dir Direction configuration to grow to
    /// @param _new Placeholder for resulting configuration
    /// @param _lpOutput Placeholder for polygonal chain configurations for
    ///        non-straight-line extention operations and associated weight
    /// @return Success/fail for extention operation
    ///
    /// @usage
    /// @code
    /// ExtenderPointer e = this->GetExtender(m_eLabel);
    /// CfgType c, cDir, cNew;
    /// LPOutput<MPTraits> lpOutput;
    /// bool pass = e->Extend(c, cDir, cNew, lpOutput);
    /// @endcode
    ////////////////////////////////////////////////////////////////////////////
    virtual bool Extend(const CfgType& _nearest, const CfgType& _dir,
        CfgType& _new, LPOutput<MPTraits>& _lpOutput) = 0;
};

#endif
