#ifndef LOCAL_PLANNER_METHOD_H_
#define LOCAL_PLANNER_METHOD_H_

#include "Environment/Environment.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"

template<class MPTraits> struct LPOutput;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup LocalPlanners
/// @brief Base algorithm abstraction for \ref LocalPlanners.
/// @tparam MPTraits Motion planning universe
///
/// LocalPlannerMethod has two main functions: @c IsConnected and
/// @c ReconstructPath.
///
/// @c IsConnected takes as input two configurations \f$c_1\f$, \f$c_2\f$, an
/// LPOutput, validation resolutions, and optional booleans dictating whether to
/// check collision and save the path. The function both returns true or false
/// to validate the simple path between \f$c_1\f$ and \f$c_2\f$, but also
/// populates the LPOutput structure with useful information.
///
/// @c ReconstructPath is used to reconstruct a specific polygonal chain from a
/// WeightType object's intermediate configurations. The function takes as input
/// two configurations, a set of intermediate configurations, and validity
/// resolutions.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class LocalPlannerMethod : public MPBaseObject<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ///@}
    ///\name Construction
    ///@{

    LocalPlannerMethod(bool _saveIntermediates = false) :
        m_saveIntermediates(_saveIntermediates) { }

    LocalPlannerMethod(MPProblemType* _problem, XMLNode& _node) :
        MPBaseObject<MPTraits>(_problem, _node) {
      m_saveIntermediates = _node.Read("saveIntermediates", false,
          false, "Save intermediate nodes");
    }

    virtual ~LocalPlannerMethod() = default;

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const;

    ///@}
    ///\name Local Planner Interface
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Validate a simple path between two nodes.
    /// @param _start The starting configuration.
    /// @param _end The ending configuration.
    /// @param _col The witness configuration on failure.
    /// @param _lpOutput Weight and path computed from local plan.
    /// @param _posRes Positional DOF resolution.
    /// @param _oriRes Rotational DOF resolution.
    /// @param _checkCollision Use validity checking?
    /// @param _savePath Save all configurations along the path?
    /// @return A boolean indicating whether the connection succeeded.
    ///
    /// @usage
    /// @code
    /// LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);
    /// Environment* env = this->GetMPProblem()->GetEnvironment();
    /// CfgType c1, c2, col;
    /// LPOutput<MPTraits> lpOut;
    /// lp->IsConnected(c1, c2, col, &lpOut, env->GetPositionRes(),
    ///     env->GetOrientationRes());
    /// @endcode
    virtual bool IsConnected(const CfgType& _start, const CfgType& _end,
        CfgType& _col, LPOutput<MPTraits>* _lpOutput, double _posRes,
        double _oriRes, bool _checkCollision = true, bool _savePath = false) = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Validate a simple path between two nodes without returning a
    ///        witness node on failure.
    /// @overload
    virtual bool IsConnected(const CfgType& _start, const CfgType& _end,
        LPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
        bool _checkCollision = true, bool _savePath = false);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Reconstruct a previously computed simple path between two nodes
    /// @param _start Configuration 1
    /// @param _end Configuration 2
    /// @param _intermediates Intermediate configurations along simple path's
    ///        polygonal chain
    /// @param _posRes Positional DOF resolution
    /// @param _oriRes Rotational DOF resolution
    /// @return Configurations along path from _start to _end up to a resolution
    ///         (_posRes, _oriRes)
    ///
    /// @usage
    /// @code
    /// LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);
    /// Environment* env = this->GetMPProblem()->GetEnvironment();
    /// CfgType c1, c2;
    /// vector<CfgType> intermediates;
    /// lp->ReconstructPath(c1, c2, intermediates, env->GetPositionRes(),
    ///     env->GetOrientationRes());
    /// @endcode
    virtual vector<CfgType> ReconstructPath(const CfgType& _start,
        const CfgType& _end, const vector<CfgType>& _intermediates,
        double _posRes, double _oriRes);

    ///@}

  protected:

    ///\name Internal State
    ///@{

    bool m_saveIntermediates; ///< Save the intermediates in the roadmap?

    ///@}
};

/*------------------------- MPBaseObject Overrides ---------------------------*/

template<class MPTraits>
void
LocalPlannerMethod<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel()
      << "\n\tSave intermediates: " << m_saveIntermediates << endl;
}

/*------------------------ LocalPlanner Interface ----------------------------*/

template<class MPTraits>
bool
LocalPlannerMethod<MPTraits>::
IsConnected(const CfgType& _start, const CfgType& _end,
    LPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath) {
  CfgType col;
  return IsConnected(_start, _end, col, _lpOutput, _posRes,
      _oriRes, _checkCollision, _savePath);
}


template<class MPTraits>
vector<typename MPTraits::CfgType>
LocalPlannerMethod<MPTraits>::
ReconstructPath(const CfgType& _start, const CfgType& _end,
    const vector<CfgType>& _intermediates, double _posRes, double _oriRes) {
  LPOutput<MPTraits> lpOutput;
  IsConnected(_start, _end, &lpOutput, _posRes, _oriRes, false, true);
  return lpOutput.m_path;
}

/*----------------------------------------------------------------------------*/

#endif
