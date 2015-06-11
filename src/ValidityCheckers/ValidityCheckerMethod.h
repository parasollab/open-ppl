#ifndef VALIDITYCHECKERMETHOD_H
#define VALIDITYCHECKERMETHOD_H

#include <string>
#include "Utilities/MPUtils.h"
#include "ValidityCheckers/CollisionDetection/CDInfo.h"
#include "MPProblem/MPBaseObject.h"
////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief Base algorithm abstraction for \ref ValidityCheckers.
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
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ValidityCheckerMethod : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;

    ValidityCheckerMethod() : MPBaseObject<MPTraits>(), m_validity(true) {}
    ValidityCheckerMethod(typename MPTraits::MPProblemType* _problem, XMLNode& _node) :
      MPBaseObject<MPTraits>(_problem, _node), m_validity(true) {}
    virtual ~ValidityCheckerMethod(){}

    virtual void Print(ostream& _os) const {
      _os << this->GetNameAndLabel() << endl;
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Classify a configuration to either @cfree or @cobst
    ///
    /// @overload
    /// No extra collision information is returned.
    ////////////////////////////////////////////////////////////////////////////
    bool IsValid(CfgType& _cfg, const string& _callName) {
      CDInfo cdInfo;
      return IsValid(_cfg, cdInfo, _callName);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Classify a configuration to either @cfree or @cobst
    /// @param _cfg Configuration
    /// @param _cdInfo Extra computed information, e.g., minimum dist to
    ///        obstacle
    /// @param _callName Function caller for statistics tracking
    /// @return boolean valid/invalid. Valid (true) implies @cfree.
    ///
    /// @usage
    /// @code
    /// ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
    /// CfgType c;
    /// CDInfo cdInfo;
    /// string callee("SomeFunc");
    /// bool valid = vc->IsValid(c, cdInfo, callee);
    /// @endcode
    ////////////////////////////////////////////////////////////////////////////
    bool IsValid(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName) {
      if(m_validity)
        return IsValidImpl(_cfg, _cdInfo, _callName);
      else
        return !IsValidImpl(_cfg, _cdInfo, _callName);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Determines if a configuration lies entirely in a workspace obstacle
    /// @param _cfg Configuration
    /// @return boolean inside/outside of workspace obstacle.
    ///
    /// @usage
    /// @code
    /// ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
    /// CfgType c;
    /// bool valid = vc->IsInsideObstacle(c);
    /// @endcode
    ////////////////////////////////////////////////////////////////////////////
    virtual bool IsInsideObstacle(const CfgType& _cfg){
      cerr << "error: IsInsideObstacle() not defined." << endl;
      exit(-1);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Switches the meaning of "valid" to "invalid" and vice versa
    ////////////////////////////////////////////////////////////////////////////
    void ToggleValidity() { m_validity = !m_validity; }

    bool GetValidity() const { return m_validity; }

  protected:
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Implementation of the classification of a configuration to either
    ///        @cfree or @cobst
    /// @param _cfg Configuration
    /// @param _cdInfo Extra computed information, e.g., minimum dist to
    ///        obstacle
    /// @param _callName Function caller for statistics tracking
    /// @return boolean valid/invalid. Valid (true) implies @cfree.
    ////////////////////////////////////////////////////////////////////////////
    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName) =0;

    bool m_validity; ///< True: valid implies true validity. False: valid implies invalid and vice verse.
};

#endif
