#ifndef ACTIVE_BODY_STRAIGHT_LINE_H_
#define ACTIVE_BODY_STRAIGHT_LINE_H_

#include "LocalPlannerMethod.h"

#include "LPOutput.h"
#include "MPLibrary/ValidityCheckers/SpecificBodyCollisionValidity.h"
#include "MPProblem/RobotGroup/GroupUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup LocalPlanners
/// @brief Validate straight-line path between two configurations,
///
/// This is a straight line local planner specially designed to handle the case
/// of subassemblies needing rotation, as used in disassembly planning.
/// This method is currently BROKEN!
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class ActiveBodyStraightLine : public LocalPlannerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;

    ActiveBodyStraightLine(const string& _vcLabel = "", bool _evalation = false,
        bool _saveIntermediates = false);
    ActiveBodyStraightLine(XMLNode& _node);
    virtual ~ActiveBodyStraightLine() = default;

    virtual void Print(ostream& _os) const;

    virtual void Initialize() const;

    /**
     * Check if two Cfgs could be connected by straight line.
     */
    //Wrapper function to call appropriate impl IsConnectedFunc based on CfgType
    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    bool IsConnectedFunc(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    void SetActiveBodies(const std::vector<unsigned int>& _bodies) {
      m_activeBodies = _bodies;
    }

  protected:

    string m_vcLabel;

    std::vector<unsigned int> m_activeBodies;
};

template<class MPTraits>
ActiveBodyStraightLine<MPTraits>::ActiveBodyStraightLine(const string& _vcLabel, bool _evalation,
    bool _saveIntermediates) : LocalPlannerMethod<MPTraits>(_saveIntermediates),
  m_vcLabel(_vcLabel) {
  this->SetName("ActiveBodyStraightLine");
}

template<class MPTraits>
ActiveBodyStraightLine<MPTraits>::ActiveBodyStraightLine(XMLNode& _node) :
    LocalPlannerMethod<MPTraits>(_node) {
  this->SetName("ActiveBodyStraightLine");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Label");
}


template<class MPTraits>
void
ActiveBodyStraightLine<MPTraits>::Print(ostream& _os) const {
  LocalPlannerMethod<MPTraits>::Print(_os);
  _os << "\tvc label = " << m_vcLabel << endl;
}


template <typename MPTraits>
void
ActiveBodyStraightLine<MPTraits>::
Initialize() const {
  throw RunTimeException(WHERE, "This method is broken and should only be used"
                                " in the interest of fixing/testing it.");
}


template<class MPTraits>
bool
ActiveBodyStraightLine<MPTraits>::
IsConnected(const CfgType& _c1, const CfgType& _c2, CfgType& _col,
            LPOutput<MPTraits>* _lpOutput, double _positionRes,
            double _orientationRes, bool _checkCollision,
            bool _savePath) {
  if(m_activeBodies.empty())
    throw RunTimeException(WHERE, "Need active bodies if using disassembly lp!");

  if(this->m_debug)
    std::cout << "Performing local plan with active bodies " << m_activeBodies
              << std::endl;

  _lpOutput->Clear();
  const bool connected = IsConnectedFunc(_c1, _c2, _col, _lpOutput,
                                         _positionRes, _orientationRes,
                                         _checkCollision, _savePath);
  _lpOutput->SetActiveBodies(m_activeBodies);
  if(connected)
    _lpOutput->SetLPLabel(this->GetLabel());

  m_activeBodies.clear(); // Require that bodies are set EVERY time
  return connected;
}

template<class MPTraits>
bool
ActiveBodyStraightLine<MPTraits>::
IsConnectedFunc(const CfgType& _c1, const CfgType& _c2, CfgType& _col,
                LPOutput<MPTraits>* _lpOutput, double _positionRes,
                double _orientationRes, bool _checkCollision, bool _savePath) {
  StatClass* stats = this->GetStatClass();
  stats->IncLPAttempts(this->GetNameAndLabel());
  int cdCounter = 0;

  bool connected = true;

  Environment* env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();
  auto vc = this->GetValidityChecker(m_vcLabel);

  int nTicks;
  CfgType tick = _c1;
  CfgType incr(robot), previous(robot);
  const string callee = this->GetNameAndLabel() + "::IsConnectedFunc";

  const unsigned int refBodyNum = m_activeBodies[0]; // The body rotated about.
  const unsigned int posDofsPerBody = incr.PosDOF();
  const unsigned int dofsPerBody = posDofsPerBody + incr.OriDOF();

  ///@TODO Set this to true to have single parts treated (less
  ///      efficiently) as multiple parts, which SHOULD be identical.
  const bool multipleParts = m_activeBodies.size() > 1;
//  const bool multipleParts = true;

  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes);
  mathtool::Orientation rotation;

  const CfgType incrUntouched = incr;
  CfgType oneStep = _c1;// + incr;


  if(multipleParts) {
    //Now remove the rotational bits, as incr should only do the
    // translation and then RotateCfgAboutBody() will handle all rotations:
    for(const unsigned int body : m_activeBodies) {
      //Copy translation of main body to all components for other bodies
      for(unsigned int i = 0; i < posDofsPerBody; ++i)
        incr[body*dofsPerBody + i] = incr[refBodyNum*dofsPerBody + i];

      //Zero out all rotations:
      for(unsigned int i = posDofsPerBody; i < dofsPerBody; ++i)
        incr[body*dofsPerBody + i] = 0.;
    }
  }

  int nIter = 0;
  for(int i = 1; i < nTicks; ++i) { //don't need to check the ends, _c1 and _c2
    previous = tick;
    tick += incr;

    oneStep += incrUntouched;

    // New attempt:
    previous.ConfigureRobot();
    mathtool::Transformation initialTransform = previous.GetMultiBody()->
        GetBody(refBodyNum)->GetWorldTransformation();

    oneStep.ConfigureRobot();
    mathtool::Transformation finalTransform = oneStep.GetMultiBody()->
        GetBody(refBodyNum)->GetWorldTransformation();

    mathtool::Transformation delta = -initialTransform * finalTransform;

    //Note that m_activeBodies[0] is the body to rotate about.
    rotation = delta.rotation();


    if(multipleParts) //Handle subassembly rotation correctly.
      tick = RotateCfgAboutBody<MPTraits>(m_activeBodies, tick, rotation,
                                          dofsPerBody, this->m_debug);
    ++cdCounter;
    if(_checkCollision){
      if(!tick.InBounds(env->GetBoundary()) || !vc->IsValid(tick, callee)) {
        if(tick.InBounds(env->GetBoundary()))
          _col = tick;
        _lpOutput->m_edge.first.SetWeight(_lpOutput->m_edge.first.GetWeight() + nIter);
        _lpOutput->m_edge.second.SetWeight(_lpOutput->m_edge.second.GetWeight() + nIter);
        connected = false;
        break;
      }
    }

    if(_savePath)
      _lpOutput->m_path.push_back(tick);
    ++nIter;
  }
  _lpOutput->m_edge.first.SetWeight(_lpOutput->m_edge.first.GetWeight() + nIter);
  _lpOutput->m_edge.second.SetWeight(_lpOutput->m_edge.second.GetWeight() + nIter);

  if(connected)
    stats->IncLPConnections(this->GetNameAndLabel());

  stats->IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  return connected;
}


#endif
