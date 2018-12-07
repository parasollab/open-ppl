#ifndef RESAMPLE_PATH_MODIFIER_H_
#define RESAMPLE_PATH_MODIFIER_H_

#include "PathModifierMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup PathModifiers
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ResamplePathModifier : public PathModifierMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Construction
    ///@{

    ResamplePathModifier(const string _dmLabel = "", const string _lpLabel = "",
        const size_t _numResamples = 0, const double _stepSize = -1,
        const double _userValue = -1, const string _typeName = "");

    ResamplePathModifier(XMLNode& _node);

    virtual ~ResamplePathModifier() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const;

    ///@}
    ///@name PathModifierMethod Overrides
    ///@{

    virtual bool ModifyImpl(RoadmapType* _graph, vector<CfgType>& _originalPath,
        vector<CfgType>& _newPath) override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// @TODO
    vector<pair<CfgType, double>> FindNeighbors(CfgType& _previous,
        CfgType& _current, CfgType& _next, size_t _maxAttempts,
        RoadmapType* _graph);

    /// Get the minimum clearance for a given configuration.
    /// @param _c The configuration of interest.
    /// @return The minimum clearance of _c.
    double GetClearance(CfgType& _c);

    ///@}
    ///@name Internal State
    ///@{

    string m_dmLabel;           // Distance metric
    string m_lpLabel;           // Local planner

    size_t m_numResamples;
    double m_stepSize;
    double m_userValue;
    string m_typeName;

    ClearanceUtility<MPTraits> m_clearanceUtils;

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
ResamplePathModifier<MPTraits>::
ResamplePathModifier(const string _dmLabel, const string _lpLabel,
    const size_t _numResamples, const double _stepSize,
    const double _userValue, const string _typeName) :
    PathModifierMethod<MPTraits>(), m_dmLabel(_dmLabel), m_lpLabel(_lpLabel),
    m_numResamples(_numResamples), m_stepSize(_stepSize), m_userValue(_userValue),
    m_typeName(_typeName) {
  this->SetName("ResamplePathModifier");
}


template <typename MPTraits>
ResamplePathModifier<MPTraits>::
ResamplePathModifier(XMLNode& _node) : PathModifierMethod<MPTraits>(_node) {
  this->SetName("ResamplePathModifier");

  m_dmLabel = _node.Read("dmLabel", false, "", "Distance metric method");
  m_lpLabel = _node.Read("lpLabel", true, "", "Local planner method");
  m_numResamples = _node.Read("numResamples", false, 5, 1, MAX_INT,
      "number of samples to generate in each resample attempt");
  m_stepSize= _node.Read("stepSize", false, 0.0, 0.0, MAX_DBL,
      "distance of a resampled node from existing one");
  m_userValue= _node.Read("userValue", false, 0.3, 0.0, MAX_DBL,
      "distance of a resampled node from existing one");
  m_typeName = _node.Read("typeName", true, "", "type of the CFG task");

  if(m_typeName == "MAX_CLEARANCE")
    m_clearanceUtils = ClearanceUtility<MPTraits>(_node);
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
ResamplePathModifier<MPTraits>::
Print(ostream& _os) const {
  PathModifierMethod<MPTraits>::Print(_os);
  _os << "\tdistance metric = \"" << m_dmLabel << "\"" << endl;
  _os << "\tlocal planner = \"" << m_lpLabel << "\"" << endl;
  _os << "\tnumber of resamples = " << m_numResamples << endl;
  if(m_typeName == "MAX_CLEARANCE") {
    _os << "\tclearance utils::" << endl;
    m_clearanceUtils.Print(_os);
  }
}


template <typename MPTraits>
bool
ResamplePathModifier<MPTraits>::
ModifyImpl(RoadmapType* _graph, vector<CfgType>& _originalPath, vector<CfgType>& _newPath) {
  if(this->m_debug)
    cout << "\n*R* Executing ResampleSmoother::Modifier" << endl;

  double currentConfigurationWeight = 0.0;
  double temp = 0.0;
  size_t maxAttempts = 1000;

  if(this->m_debug) {
    cout << "\ttypeName::" << m_typeName << endl;
    cout << "\tnumResamples::" << m_numResamples << endl;
    cout << "\tstepSize::" << m_stepSize << endl;
    cout << "\tuserValue::" << m_userValue << endl;
  }

  StatClass* stats = this->GetStatClass();
  stats->StartClock("Resampling");

  vector<double> smoothingValues;

  // Make room for the path
  _newPath.clear();

  auto cit = _originalPath.begin();
  _newPath.push_back(*cit);

  if(this->m_debug) {
    temp = GetClearance(*cit);
    cout << "\toldsmoothingvalues:" << endl << temp << ";";
    smoothingValues.push_back(temp);
  }

  ++cit;

  for(; cit + 1 != _originalPath.end(); ++cit) {
    currentConfigurationWeight = GetClearance(*cit);

    if(this->m_debug)
      cout << currentConfigurationWeight << ";";

    if((currentConfigurationWeight > m_userValue
        && m_typeName == "PROTEIN_ENERGY") ||
        (currentConfigurationWeight < m_userValue
        && m_typeName == "MAX_CLEARANCE")) {

      CfgType previous = *(cit-1);
      CfgType current = *cit;
      CfgType next = *(cit+1);

      vector<pair<CfgType, double> > sampledNeighbors =
          FindNeighbors(previous, current, next, maxAttempts, _graph);

      if(sampledNeighbors.size() > 0) {
        auto nit = sampledNeighbors.begin();
        _newPath.push_back(nit->first);
        *cit = nit->first;
        if(this->m_debug) {
          temp = GetClearance(nit->first);
          smoothingValues.push_back(temp);
        }
      }
      else{
        _newPath.push_back(*cit);
        if(this->m_debug) {
          temp = GetClearance(*cit);
          smoothingValues.push_back(temp);
        }
      }
    }
    else {
      _newPath.push_back(*cit);
      if(this->m_debug) {
        temp = GetClearance(*cit);
        smoothingValues.push_back(temp);
      }
    }
  }

  _newPath.push_back(*cit);
  if(this->m_debug) {
    temp = GetClearance(*cit);
    smoothingValues.push_back(temp);
  }

  stats->StopClock("Resampling");

  if(this->m_debug) {
    stats->PrintClock("Resampling", cout);
    cout << endl;
    cout << "\tnewsmoothingvalues:" << endl;
    for(auto dit = smoothingValues.begin(); dit != smoothingValues.end(); ++dit)
      cout << *dit << ";";
    cout << endl;
  }

  return false;
}


template <typename MPTraits>
vector<pair<typename MPTraits::CfgType, double>>
ResamplePathModifier<MPTraits>::
FindNeighbors(CfgType& _previous, CfgType& _current, CfgType& _next,
    size_t _maxAttempts, RoadmapType* _graph) {
  size_t numOfSamples = m_numResamples;
  vector<pair<CfgType, double>> result;
  double newConfigurationWeight, oldConfigurationWeight;
  bool firstConnectFlag, secondConnectFlag;

  LPOutput<MPTraits> lpOutput;
  auto dm = this->GetDistanceMetric(this->m_dmLabel);
  auto lp = this->GetLocalPlanner(this->m_lpLabel);
  Environment* env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();

  oldConfigurationWeight = GetClearance(_current);

  for(size_t k=0; k < _maxAttempts && numOfSamples > 0; ++k) {
    CfgType r(robot), c(robot);
    r.GetRandomRay(m_stepSize, dm);
    c = r + _current;
    newConfigurationWeight = GetClearance(c);
    if((newConfigurationWeight > oldConfigurationWeight
        && m_typeName == "MAX_CLEARANCE") ||
        (newConfigurationWeight < oldConfigurationWeight
        && m_typeName == "PROTEIN_ENERGY")) {
      firstConnectFlag = lp->IsConnected(_previous, c, &lpOutput,
            env->GetPositionRes(), env->GetOrientationRes());
      secondConnectFlag = lp->IsConnected(c, _next, &lpOutput,
            env->GetPositionRes(), env->GetOrientationRes());

      VID cvid = _graph->GetVID(c);
      if(cvid == INVALID_VID)
        cvid = _graph->AddVertex(c);// the vertex did not exist

      if(firstConnectFlag && secondConnectFlag) {
        _graph->AddEdge(_graph->GetVID(_previous), cvid, lpOutput.m_edge);
        _graph->AddEdge(cvid, _graph->GetVID(_next), lpOutput.m_edge);
        result.push_back(pair<CfgType,double>(c, newConfigurationWeight));
        numOfSamples--;
      }
    }
  }

  if(m_typeName == "MAX_CLEARANCE") {
    // Sort results with highest clearance first.
    auto decreasing = [](const std::pair<CfgType, double>& _left,
                         const std::pair<CfgType, double>& _right) {
      return _left.second > _right.second;
    };
    sort(result.begin(), result.end(), decreasing);
  }
  else if(m_typeName == "PROTEIN_ENERGY") {
    // Sort results with lowest energy first.
    auto increasing = [](const std::pair<CfgType, double>& _left,
                         const std::pair<CfgType, double>& _right) {
        return _left.second < _right.second;
    };
    sort(result.begin(), result.end(), increasing);
  }

  return result;
}


template <typename MPTraits>
double
ResamplePathModifier<MPTraits>::
GetClearance(CfgType& _c) {
  CDInfo cdInfo;
  CfgType null(_c.GetRobot());
  m_clearanceUtils.CollisionInfo(_c, null,
      this->GetEnvironment()->GetBoundary(), cdInfo);
  return cdInfo.m_minDist;
}

/*----------------------------------------------------------------------------*/

#endif
