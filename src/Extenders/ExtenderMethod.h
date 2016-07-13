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
template <class MPTraits>
class ExtenderMethod : public MPBaseObject<MPTraits> {

  protected:

    ///\name Extender Properties
    ///@{

    double m_minDist{.001};
    double m_maxDist{10};

    ///@}

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ///@}
    ///\name Construction
    ///@{

    ExtenderMethod() = default;
    ExtenderMethod(MPProblemType* _problem, XMLNode& _node) :
        MPBaseObject<MPTraits>(_problem, _node) {
      ParseXML(_node);
    }
    virtual ~ExtenderMethod() = default;

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override {
      MPBaseObject<MPTraits>::Print(_os);
      _os << "\tMin distance: " << m_minDist << endl
          << "\tMax distance: " << m_maxDist << endl;
    }

    void ParseXML(XMLNode& _node) {
      m_maxDist = _node.Read("maxDist", false, 10., 0., MAX_DBL, "Maximum "
          "extension distance");
      m_minDist = _node.Read("minDist", false, .001, 0., MAX_DBL, "Minimum "
          "extension distance.");
    }

    ///@}
    ///\name Required Interface
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Maximum extension distance
    virtual double GetMinDistance() const {return m_minDist;}

    ////////////////////////////////////////////////////////////////////////////
    /// @return Maximum extension distance
    virtual double GetMaxDistance() const {return m_maxDist;}

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

    ///@}
};

#endif
