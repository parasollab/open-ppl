#ifndef VALIDITYCHECKERFUNCTOR_H
#define VALIDITYCHECKERFUNCTOR_H

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ValidityCheckerFunctor {
  public:
    typedef typename MPTraits::CfgType CfgType;
    ValidityCheckerFunctor(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName) :
      m_cfg(_cfg), m_cdInfo(_cdInfo), m_callName(_callName) {}

    ~ValidityCheckerFunctor() {}

    template<class ValidityCheckerPointer>
      bool operator()(ValidityCheckerPointer _vcMethodPtr){
        return _vcMethodPtr->IsValid(m_cfg, m_cdInfo, m_callName);
      }

  private:
    CfgType& m_cfg;
    CDInfo& m_cdInfo;
    const string& m_callName;
};

#endif
