#ifndef TRANSFORM_AT_S_H_
#define TRANSFORM_AT_S_H_

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

    virtual std::vector<CfgType> ReconstructPath(
        const CfgType& _c1, const CfgType& _c2,
        const std::vector<CfgType>& _intermediates,
        double _posRes, double _oriRes) override;

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
  if(this->GetTask()->GetRobot()->GetMultiBody()->IsComposite())
    throw RunTimeException(WHERE, "Does not work for composite c-space.");
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
  StatClass* stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetName() + "::IsConnected");

  _lpOutput->Clear();

  // Generate the sequence of intermediate nodes.
  std::vector<CfgType> sequence = GetSequenceNodes(_c1, _c2, true);
  if(this->m_debug) {
    std::cout << "Start CFG positional DOF: " << _c1.PosDOF() << std::endl;
    for(auto iter = sequence.begin(); iter != sequence.end(); iter++)
      std::cout << "C" << std::distance(sequence.begin(), iter) << ": "
                << iter->PrettyPrint()
                << std::endl;
  }

  bool connected = true;

  // Check sequence nodes (skip start/end as they are already validated).
  if(_checkCollision) {
    size_t cdCounter = 0;
    auto vc = this->GetValidityChecker(this->m_vcLabel);
    const std::string callee = this->GetNameAndLabel() + "::IsConnected";

    for(auto iter = sequence.begin() + 1; iter != sequence.end() - 1; ++iter) {
      ++cdCounter;
      if(!vc->IsValid(*iter, callee)) {
        _col = *iter;
        connected = false;
        break;
      }
    }
    stats->IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  }

  // Plan between sequence nodes.
  for(auto iter = sequence.begin(); connected and iter != sequence.end() - 1;
      ++iter) {
    LPOutput<MPTraits> lpo;
    connected = this->IsConnectedFunc(*iter, *(iter + 1), _col, &lpo,
        _posRes, _oriRes, _checkCollision, _savePath);
    // Add this weight to the lpOutput.
    if(connected) {
      auto& w = _lpOutput->m_edge.first;
      w.SetWeight(w.GetWeight() + lpo.m_edge.first.GetWeight());
    }
  }

  // If the plan is good, set lpoutput data.
  if(connected) {
    stats->IncLPConnections(this->GetNameAndLabel());
    _lpOutput->m_edge.second.SetWeight(_lpOutput->m_edge.first.GetWeight());

    // Save intermediates if needed.
    if(_savePath)
      for(auto iter = sequence.begin() + 1; iter != sequence.end() - 1; ++iter)
        _lpOutput->m_intermediates.push_back(*iter);

    _lpOutput->SetLPLabel(this->GetLabel());
    _lpOutput->AddIntermediatesToWeights(this->m_saveIntermediates);
  }

  return connected;
}


template <class MPTraits>
std::vector<typename MPTraits::CfgType>
TransformAtS<MPTraits>::
ReconstructPath(
    const CfgType& _c1, const CfgType& _c2,
    const std::vector<CfgType>& _intermediates,
    double _posRes, double _oriRes) {

  int dummyCntr;
  LPOutput<MPTraits> lpOutput;
  CfgType col;

  // Generate path between start, intermediates, and goal
  std::vector<CfgType> cfgList;
  cfgList.push_back(_c1);
  cfgList.insert(cfgList.end(), _intermediates.begin(), _intermediates.end());
  cfgList.push_back(_c2);

  for(auto iter = cfgList.begin(); iter != cfgList.end() - 1; iter++) {
    if(this->m_binaryEvaluation)
      this->IsConnectedSLBinary(*iter, *(iter + 1), col, &lpOutput,
          dummyCntr, _posRes, _oriRes, false, true);
    else
      this->IsConnectedSLSequential(*iter, *(iter + 1), col, &lpOutput,
          dummyCntr, _posRes, _oriRes, false, true);
    if(distance(cfgList.begin(), iter) != (int)cfgList.size() - 2)
      lpOutput.m_intermediates.push_back(*(iter + 1));
  }

  // Return final path
  return lpOutput.m_intermediates;
}

/*--------------------------------- Helpers ----------------------------------*/

template <class MPTraits>
std::vector<typename MPTraits::CfgType>
TransformAtS<MPTraits>::
GetSequenceNodes(const CfgType& _c1, const CfgType& _c2, const bool _reverse) {
  // Assert the robot makes sense for this LP.
  if(_c1.PosDOF() == 0)
    throw RunTimeException(WHERE, "This LP doesn't make sense for robots that "
        "can't translate.");

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
    for(size_t i = _c1.DOF() - 1; i > _c1.PosDOF() - 1; --i) {
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
