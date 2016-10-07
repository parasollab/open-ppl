#ifndef RESAMPLEPATHMODIFIER_H_
#define RESAMPLEPATHMODIFIER_H_

#include "PathModifierMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup PathModifiers
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ResamplePathModifier : public PathModifierMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename vector<CfgType>::iterator CfgIter;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    ResamplePathModifier(const string _dmLabel = "", const string _lpLabel = "",
        const size_t _numResamples = 0, const double _stepSize = -1,
        const double _userValue = -1, const string _typeName = "");
    ResamplePathModifier(MPProblemType* _problem, XMLNode& _node);

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    bool ModifyImpl(vector<CfgType>& _originalPath, vector<CfgType>& _newPath);

  private:
    string m_dmLabel;           // Distance metric
    string m_lpLabel;           // Local planner
    shared_ptr<Boundary> m_boundary;

    vector<pair<CfgType, double> > FindNeighbors(CfgType& _previous,
        CfgType& _current, CfgType& _next, size_t _maxAttempts);// use m_boundary which is initialize in Modifier method

    size_t m_numResamples;
    double m_stepSize;
    double m_userValue;
    string m_typeName;
    ClearanceUtility<MPTraits> m_clearanceUtils;


};

// Non-XML Constructor
template<class MPTraits>
ResamplePathModifier<MPTraits>::ResamplePathModifier(const string _dmLabel,
    const string _lpLabel, const size_t _numResamples, const double _stepSize,
    const double _userValue, const string _typeName) :
  PathModifierMethod<MPTraits>(), m_dmLabel(_dmLabel), m_lpLabel(_lpLabel),
  m_numResamples(_numResamples), m_stepSize(_stepSize), m_userValue(_userValue),
  m_typeName(_typeName) {
    this->SetName("ResamplePathModifier");
  }

// XML Constructor
template<class MPTraits>
ResamplePathModifier<MPTraits>::ResamplePathModifier(MPProblemType* _problem, XMLNode& _node) :
  PathModifierMethod<MPTraits>(_problem, _node) {
    this->SetName("ResamplePathModifier");
    ParseXML(_node);
  }

template<class MPTraits>
void
ResamplePathModifier<MPTraits>::ParseXML(XMLNode& _node) {
  m_dmLabel = _node.Read("dmLabel", false, "", "Distance metric method");
  m_lpLabel = _node.Read("lpLabel", true, "", "Local planner method");
  m_numResamples = _node.Read("numResamples", false, 5, 1, MAX_INT, "number of samples to generate in each resample attempt");
  m_stepSize= _node.Read("stepSize", false, 0.0, 0.0, MAX_DBL, "distance of a resampled node from existing one");
  m_userValue= _node.Read("userValue", false, 0.3, 0.0, MAX_DBL, "distance of a resampled node from existing one");
  m_typeName = _node.Read("typeName", true, "", "type of the CFG task");

  if(m_typeName == "MAX_CLEARANCE")
    m_clearanceUtils = ClearanceUtility<MPTraits>(this->GetMPProblem(), _node);
}

template<class MPTraits>
void
ResamplePathModifier<MPTraits>::Print(ostream& _os) const {
  PathModifierMethod<MPTraits>::Print(_os);
  _os << "\tdistance metric = \"" << m_dmLabel << "\"" << endl;
  _os << "\tlocal planner = \"" << m_lpLabel << "\"" << endl;
  _os << "\tnumber of resamples = " << m_numResamples << endl;
  if(m_typeName == "MAX_CLEARANCE") {
    _os << "\tclearance utils::" << endl;
    m_clearanceUtils.Print(_os);
  }
}

struct SortAscend {
  template<typename T>
  bool operator()(const std::pair<T, double>& _left, const std::pair<T, double>& _right) {
    return _left.second < _right.second;
  }
};

struct SortDescend {
  template<typename T>
  bool operator()(const std::pair<T, double>& _left, const std::pair<T, double>& _right) {
    return _left.second > _right.second;
  }
};

template<class MPTraits>
bool
ResamplePathModifier<MPTraits>::ModifyImpl(vector<CfgType>& _originalPath, vector<CfgType>& _newPath) {
  if(this->m_debug)
    cout << "\n*R* Executing ResampleSmoother::Modifier" << endl;

  double currentConfigurationWeight = 0.0;
  double temp = 0.0;
  size_t maxAttempts = 1000;

  if(m_boundary == NULL)
    m_boundary = this->GetMPProblem()->GetEnvironment()->GetBoundary();

  if(this->m_debug) {
    cout << "\ttypeName::" << m_typeName << endl;
    cout << "\tnumResamples::" << m_numResamples << endl;
    cout << "\tstepSize::" << m_stepSize << endl;
    cout << "\tuserValue::" << m_userValue << endl;
  }

  StatClass* stats = this->GetMPProblem()->GetStatClass();
  stats->StartClock("Resampling");

  vector<double> smoothingValues;

  // Make room for the path
  _newPath.clear();

  CfgIter cit = _originalPath.begin();
  _newPath.push_back(*cit);

  if(this->m_debug) {
    temp = cit->GetSmoothingValue(m_clearanceUtils, m_boundary);
    cout << "\toldsmoothingvalues:" << endl << temp << ";";
    smoothingValues.push_back(temp);
  }

  ++cit;

  for(; cit+1 != _originalPath.end(); ++cit) {

    currentConfigurationWeight = cit->GetSmoothingValue(m_clearanceUtils, m_boundary);
    if(this->m_debug) cout << currentConfigurationWeight << ";";
    if((currentConfigurationWeight > m_userValue && m_typeName == "PROTEIN_ENERGY") ||
        (currentConfigurationWeight < m_userValue && m_typeName == "MAX_CLEARANCE")) {

      CfgType previous = *(cit-1);
      CfgType current = *cit;
      CfgType next = *(cit+1);

      vector<pair<CfgType, double> > sampledNeighbors =
        FindNeighbors(previous, current, next, maxAttempts);

      if(sampledNeighbors.size() > 0) {
        typename vector<pair<CfgType, double> >::iterator nit = sampledNeighbors.begin();
        _newPath.push_back(nit->first);
        *cit = nit->first;
        if(this->m_debug) {
          temp = nit->first.GetSmoothingValue(m_clearanceUtils, m_boundary);
          smoothingValues.push_back(temp);
        }
      }
      else{
        _newPath.push_back(*cit);
        if(this->m_debug) {
          temp = cit->GetSmoothingValue(m_clearanceUtils, m_boundary);
          smoothingValues.push_back(temp);
        }
      }
    }
    else {
      _newPath.push_back(*cit);
      if(this->m_debug) {
        temp = cit->GetSmoothingValue(m_clearanceUtils, m_boundary);
        smoothingValues.push_back(temp);
      }
    }
  }//for

  _newPath.push_back(*cit);
  if(this->m_debug) {
    temp = cit->GetSmoothingValue(m_clearanceUtils, m_boundary);
    smoothingValues.push_back(temp);
  }

  stats->StopClock("Resampling");

  if(this->m_debug) {
    stats->PrintClock("Resampling", cout);
    cout << endl;
    cout << "\tnewsmoothingvalues:" << endl;
    for(vector<double>::iterator dit = smoothingValues.begin(); dit != smoothingValues.end(); ++dit)
      cout << *dit << ";";
    cout<<endl;
  }

  return false;
}

template<class MPTraits>
vector<pair<typename MPTraits::CfgType, double> >
ResamplePathModifier<MPTraits>::FindNeighbors(CfgType& _previous, CfgType& _current, CfgType& _next, size_t _maxAttempts) {
  size_t numOfSamples = m_numResamples;
  vector<pair<CfgType, double> > result;
  double newConfigurationWeight, oldConfigurationWeight;
  bool firstConnectFlag, secondConnectFlag;

  LPOutput<MPTraits> lpOutput;
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(this->m_lpLabel);
  GraphType* graph = this->GetMPProblem()->GetRoadmap()->GetGraph();
  Environment* env = this->GetMPProblem()->GetEnvironment();

  oldConfigurationWeight = _current.GetSmoothingValue(m_clearanceUtils, m_boundary);

  for(size_t k=0; k < _maxAttempts && numOfSamples > 0; ++k) {
    CfgType r, c;
    r.GetRandomRay(m_stepSize, dm);
    c = r + _current;
    newConfigurationWeight = c.GetSmoothingValue(m_clearanceUtils, m_boundary);
    if((newConfigurationWeight > oldConfigurationWeight && m_typeName == "MAX_CLEARANCE") ||
        (newConfigurationWeight < oldConfigurationWeight && m_typeName == "PROTEIN_ENERGY")) {
      firstConnectFlag = lp->IsConnected(_previous, c, &lpOutput,
            env->GetPositionRes(), env->GetOrientationRes());
      secondConnectFlag = lp->IsConnected(c, _next, &lpOutput,
            env->GetPositionRes(), env->GetOrientationRes());

      VID cvid = graph->GetVID(c);
      if(cvid == INVALID_VID)
        cvid = graph->AddVertex(c);// the vertex did not exist

      if(firstConnectFlag && secondConnectFlag) {
        graph->AddEdge(graph->GetVID(_previous), cvid, lpOutput.m_edge);
        graph->AddEdge(cvid, graph->GetVID(_next), lpOutput.m_edge);
        result.push_back(pair<CfgType,double>(c, newConfigurationWeight));
        numOfSamples--;
      }
    }
  }//for

  if(m_typeName == "MAX_CLEARANCE")
    sort(result.begin(), result.end(), SortDescend());
  else if(m_typeName == "PROTEIN_ENERGY")
    sort(result.begin(), result.end(), SortAscend());

  return result;
}

#endif

