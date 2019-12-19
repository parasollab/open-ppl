#ifndef PMPL_TRANSFORM_AT_S_H_
#define PMPL_TRANSFORM_AT_S_H_

#include "StraightLine.h"


////////////////////////////////////////////////////////////////////////////////
/// Translates to the location \f$s\f$ percent along the straight line path,
/// changes all orientation DoFs one by one, then translates to the goal.
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class TransformAtS : public StraightLine<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    TransformAtS(double _s = 0.5, const std::string& _vcLabel = "",
        bool _evalation = false, bool _saveIntermediates = false);

    TransformAtS(XMLNode& _node);

    virtual ~TransformAtS() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name LocalPlannerMethod Overrides
    ///@{

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false) override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Generate a sequence of intermediate nodes which can be connected via
    /// straight-line planning to implement transform at s.
    /// @param _c1 The start node.
    /// @param _c2 The end node.
    /// @param _reverse Generate the reverse sequence?
    /// @return The generated sequence of intermediate nodes.
    virtual std::vector<CfgType> GetSequenceNodes(const CfgType& _c1,
        const CfgType& _c2, const bool _reverse = true);

    ///@}
    ///@name Internal State
    ///@{

    /// Transform should happen at this fraction of the original straight-line
    /// local plan.
    double m_s{.5};

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <class MPTraits>
TransformAtS<MPTraits>::
TransformAtS(double _s, const std::string& _vcLabel, bool _evalation,
    bool _saveIntermediates) :
    StraightLine<MPTraits>(_vcLabel, _evalation, _saveIntermediates),
    m_s(_s) {
  this->SetName("TransformAtS");
}


template <class MPTraits>
TransformAtS<MPTraits>::
TransformAtS(XMLNode& _node) : StraightLine<MPTraits>(_node) {
  this->SetName("TransformAtS");
  m_s = _node.Read("s", true, m_s, 0.0, 1.0, "Transform at s value");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
TransformAtS<MPTraits>::
Initialize() {
  // This doesn't work for composite c-space.
  auto task = this->GetTask();
  if(task and task->GetRobot()->GetMultiBody()->IsComposite())
    throw RunTimeException(WHERE) << "Does not work for composite c-space.";
}


template <class MPTraits>
void
TransformAtS<MPTraits>::
Print(std::ostream& _os) const {
  StraightLine<MPTraits>::Print(_os);
  _os << "\tbinary evaluation = " << this->m_binaryEvaluation
      << "\n\ts = " << m_s
      << std::endl;
}

/*----------------------- LocalPlannerMethod Overrides -----------------------*/

template <class MPTraits>
bool
TransformAtS<MPTraits>::
IsConnected(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath) {
  const std::string id     = this->GetNameAndLabel(),
                    callee = id + "::IsConnected";
  StatClass* const stats = this->GetStatClass();
  MethodTimer mt(stats, callee);

  // Initialize the LPOutput object.
  _lpOutput->Clear();
  _lpOutput->SetLPLabel(this->GetLabel());

  // Generate the sequence of intermediate nodes, including the start and goal.
  std::vector<CfgType> sequence = GetSequenceNodes(_c1, _c2, true);

  // Check sequence nodes (skip start/end as they are already validated).
  bool connected = true;
  if(_checkCollision) {
    size_t cdCounter = 0;
    auto vc = this->GetValidityChecker(this->m_vcLabel);

    for(auto iter = sequence.begin() + 1; iter != sequence.end() - 1; ++iter) {
      ++cdCounter;
      if(vc->IsValid(*iter, callee))
        continue;

      _col = *iter;
      connected = false;
      break;
    }
    stats->IncLPCollDetCalls(id, cdCounter);
  }

  auto& forward = _lpOutput->m_edge.first;

  // Plan between sequence nodes.
  for(auto iter = sequence.begin(); connected and iter != sequence.end() - 1;
      ++iter) {
    LPOutput<MPTraits> lpo;
    const CfgType& c1 = *iter,
                 & c2 = *(iter + 1);

    // Plan between these intermediates.
    connected = this->IsConnectedFunc(c1, c2, _col, &lpo, _posRes, _oriRes,
                                      _checkCollision, _savePath);
    if(!connected)
      break;

    // Add this weight to the lpOutput.
    auto& f = lpo.m_edge.first;
    forward.SetWeight(forward.GetWeight() + f.GetWeight());
    forward.SetTimeSteps(forward.GetTimeSteps() + f.GetTimeSteps());

    // Add the sequence node as an intermediate of the path.
    const bool first = iter == sequence.begin();
    if(!first)
      _lpOutput->m_intermediates.push_back(c1);

    // Save the resolution-level path if needed.
    if(_savePath) {
      auto& p1       = _lpOutput->m_path;
      const auto& p2 = lpo.m_path;
      if(!first)
        p1.push_back(c1);
      p1.insert(p1.end(), p2.begin(), p2.end());
    }
  }

  // If the plan is good, set lpoutput data.
  if(connected) {
    stats->IncLPConnections(id);
    auto& backward = _lpOutput->m_edge.second;
    backward.SetWeight(forward.GetWeight());
    backward.SetTimeSteps(forward.GetTimeSteps());

    if(this->m_saveIntermediates)
      _lpOutput->AddIntermediatesToWeights(true);
  }

  return connected;
}

/*--------------------------------- Helpers ----------------------------------*/

template <class MPTraits>
std::vector<typename MPTraits::CfgType>
TransformAtS<MPTraits>::
GetSequenceNodes(const CfgType& _c1, const CfgType& _c2, const bool _reverse) {
  // Assert the robot makes sense for this LP.
  if(_c1.PosDOF() == 0)
    throw RunTimeException(WHERE) << "This LP doesn't make sense for robots "
                                  << "that can't translate.";

  // Push the start node into the sequence.
  std::vector<CfgType> sequence;
  sequence.push_back(_c1);

  // Translate the robot base s way between start and goal, keeping
  // orientation fixed.
  CfgType transformPoint = _c1;
  transformPoint.SetLinearPosition(_c2.GetLinearPosition());
  transformPoint.WeightedSum(_c1, transformPoint, m_s);
  sequence.push_back(transformPoint);

  // Create intermediate configurations by moving each remaining DOF one at a
  // time in index order. Use reverse order for reverse direction.
  if(!_reverse) {
    for(size_t i = _c1.PosDOF(); i < _c1.DOF(); ++i) {
      transformPoint[i] = _c2[i];
      sequence.push_back(transformPoint);
    }
  }
  else {
    for(size_t i = _c1.DOF() - 1; i >= _c1.PosDOF(); --i) {
      transformPoint[i] = _c2[i];
      sequence.push_back(transformPoint);
    }
  }

  // Push the end node into the sequence.
  sequence.push_back(_c2);
  return sequence;
}

/*----------------------------------------------------------------------------*/

#endif
