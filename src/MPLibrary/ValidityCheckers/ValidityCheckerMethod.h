#ifndef VALIDITY_CHECKER_METHOD_H
#define VALIDITY_CHECKER_METHOD_H

#include "MPLibrary/MPBaseObject.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "Utilities/MPUtils.h"

#include <string>


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref ValidityCheckers.
///
/// ValidityCheckerMethod has two important methods: @c IsValid and
/// @c IsInsideObstacle.
///
/// @c IsValid takes as input a configuration @c c and a @c CDInfo and returns
/// whether or not @c c is within @cfree.
///
/// @c IsInsideObstacle is meant mostly for medial axis related functions, but
/// it takes as input a configuration @c c and determines whether the robot
/// configured at @c lies entirely within an obstacle.
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ValidityCheckerMethod : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::GroupCfgType  GroupCfgType;
    typedef typename GroupCfgType::Formation Formation;

    ///@}
    ///@name Construction
    ///@{

    ValidityCheckerMethod() = default;

    ValidityCheckerMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {}

    virtual ~ValidityCheckerMethod() = default;

    ///@}
    ///@name ValidityChecker Interface
    ///@{

    /// Classify a configuration to either @cfree or @cobst. No extra collision
    /// information is returned.
    /// @overload
    bool IsValid(CfgType& _cfg, const std::string& _callName) {
      CDInfo cdInfo;
      return IsValid(_cfg, cdInfo, _callName);
    }

    /// Classify a GroupCfg in the same manner.
    /// @overload
    bool IsValid(GroupCfgType& _cfg, const std::string& _callName,
                 const Formation& _robotIndexes = Formation()) {
      CDInfo cdInfo;
      return IsValid(_cfg, cdInfo, _callName, _robotIndexes);
    }

    /// Classify a configuration to either @cfree or @cobst.
    /// @param _cfg Configuration
    /// @param _cdInfo Extra computed information, e.g., minimum dist to
    ///        obstacle
    /// @param _callName Function caller for statistics tracking
    /// @return boolean valid/invalid. Valid (true) implies @cfree.
    ///
    /// @usage
    /// @code
    /// ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);
    /// CfgType c;
    /// CDInfo cdInfo;
    /// string callee("SomeFunc");
    /// bool valid = vc->IsValid(c, cdInfo, callee);
    /// @endcode
    bool IsValid(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
      if(m_validity)
        return IsValidImpl(_cfg, _cdInfo, _callName);
      else
        return !IsValidImpl(_cfg, _cdInfo, _callName);
    }

    /// Classify a GroupCfg in the same manner.
    /// @overload
    bool IsValid(GroupCfgType& _cfg, CDInfo& _cdInfo,
                 const std::string& _callName,
                 const Formation& _robotIndexes = Formation()) {
      if(m_validity)
        return IsValidImpl(_cfg, _cdInfo, _callName, _robotIndexes);
      else
        return !IsValidImpl(_cfg, _cdInfo, _callName, _robotIndexes);
    }

    /// Determine if a configuration lies entirely in a workspace obstacle.
    /// @param _cfg Configuration
    /// @return boolean inside/outside of workspace obstacle.
    ///
    /// @usage
    /// @code
    /// ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);
    /// CfgType c;
    /// bool valid = vc->IsInsideObstacle(c);
    /// @endcode
    virtual bool IsInsideObstacle(const CfgType& _cfg) {
      throw RunTimeException(WHERE, "IsInsideObstacle() not defined.");
    }


    /// TODO: Confirm this is ideal behavior.
    virtual bool IsInsideObstacle(const GroupCfgType& _cfg,
                                  const Formation& _robotIndexes = Formation()){
      for(size_t i : _robotIndexes)
        if(!IsInsideObstacle(_cfg.GetRobotCfg(i)))
          return false; // If any robots are not inside, we say it's false.
      return true;
    }

    /// Switches the meaning of "valid" to "invalid" and vice versa
    void ToggleValidity() {m_validity = !m_validity;}

    bool GetValidity() const {return m_validity;}

  protected:

    /// Implementation of the classification of a configuration to either @cfree
    /// or @cobst
    /// @param _cfg Configuration
    /// @param _cdInfo Extra computed information, e.g., minimum dist to
    ///        obstacle
    /// @param _callName Function caller for statistics tracking
    /// @return boolean valid/invalid. Valid (true) implies @cfree.
    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
                             const std::string& _callName) = 0;

    /// We don't want to require VCs implement group behavior, but want the
    /// default to be an exception thrown.
    virtual bool IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo,
                             const std::string& _callName,
                             const Formation& _robotIndexes = 0) {
      throw RunTimeException(WHERE, "Not Implemented.");
    }

    bool m_validity{true}; ///< Use standard validity? False indicates negation.

};

#endif
