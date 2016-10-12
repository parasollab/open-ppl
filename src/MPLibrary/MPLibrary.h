#ifndef MP_LIBRARY_H_
#define MP_LIBRARY_H_

#include "MPProblem/MPProblem.h"

#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"

#include "MPLibrary/Connectors/ConnectorMethod.h"
#include "MPLibrary/DistanceMetrics/DistanceMetricMethod.h"
#include "MPLibrary/Extenders/ExtenderMethod.h"
#include "MPLibrary/LocalPlanners/LocalPlannerMethod.h"
#include "MPLibrary/MapEvaluators/MapEvaluatorMethod.h"
#include "MPLibrary/Metrics/MetricMethod.h"
#include "MPLibrary/MPStrategies/MPStrategyMethod.h"
#include "MPLibrary/NeighborhoodFinders/NeighborhoodFinderMethod.h"
#include "MPLibrary/PathModifiers/PathModifierMethod.h"
#include "MPLibrary/Samplers/SamplerMethod.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "MPLibrary/ValidityCheckers/ValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @brief The set of all algorithms in PMPL for solving motion planning problems.
///
/// The MPLibrary represents a collection of planning algorithms that can
/// operate on a specific MPProblem to solve its queries.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
#ifdef _PARALLEL
class MPLibrary final : public stapl::p_object
#else
class MPLibrary final
#endif
{

  public:

    ///@name General Abstraction Types
    ///@{

    typedef typename MPTraits::MPProblemType      MPProblemType;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Solver represents an input set to MPLibrary. It includes an
    ///        MPStrategy label, seed, base file name, and vizmo debug option.
    typedef tuple<string, long, string, bool> Solver;

    ///@}
    ///@name Method Set Types
    ///@{

    typedef MethodSet<MPTraits, DistanceMetricMethod<MPTraits>> DistanceMetricSet;
    typedef MethodSet<MPTraits, ValidityCheckerMethod<MPTraits>>
                                                            ValidityCheckerSet;
    typedef MethodSet<MPTraits, NeighborhoodFinderMethod<MPTraits>>
                                                            NeighborhoodFinderSet;
    typedef MethodSet<MPTraits, SamplerMethod<MPTraits>>        SamplerSet;
    typedef MethodSet<MPTraits, LocalPlannerMethod<MPTraits>>   LocalPlannerSet;
    typedef MethodSet<MPTraits, ExtenderMethod<MPTraits>>       ExtenderSet;
    typedef MethodSet<MPTraits, PathModifierMethod<MPTraits>>   PathModifierSet;
    typedef MethodSet<MPTraits, ConnectorMethod<MPTraits>>      ConnectorSet;
    typedef MethodSet<MPTraits, MetricMethod<MPTraits>>         MetricSet;
    typedef MethodSet<MPTraits, MapEvaluatorMethod<MPTraits>>   MapEvaluatorSet;
    typedef MethodSet<MPTraits, MPStrategyMethod<MPTraits>>     MPStrategySet;

    ///@}
    ///@name Method Pointer Types
    ///@{

    typedef typename ValidityCheckerSet::MethodPointer ValidityCheckerPointer;
    typedef typename DistanceMetricSet::MethodPointer  DistanceMetricPointer;
    typedef typename NeighborhoodFinderSet::MethodPointer
                                                       NeighborhoodFinderPointer;
    typedef typename SamplerSet::MethodPointer         SamplerPointer;
    typedef typename LocalPlannerSet::MethodPointer    LocalPlannerPointer;
    typedef typename ExtenderSet::MethodPointer        ExtenderPointer;
    typedef typename PathModifierSet::MethodPointer    PathModifierPointer;
    typedef typename ConnectorSet::MethodPointer       ConnectorPointer;
    typedef typename MetricSet::MethodPointer          MetricPointer;
    typedef typename MapEvaluatorSet::MethodPointer    MapEvaluatorPointer;
    typedef typename MPStrategySet::MethodPointer      MPStrategyPointer;

    ///@}
    ///@name Construction
    ///@{

    MPLibrary();
    MPLibrary(const string& _filename);
    ~MPLibrary();

    ///@}
    ///@name Configuration
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read an XML file to set the algorithms and parameters in this
    ///        instance.
    /// @param[in] _filename The XML file name.
    void ReadXMLFile(const string& _filename);

    ///@}
    ///@name Base Filename Accessors
    ///@{

    const string& GetBaseFilename() const {return m_problem->GetBaseFilename();}
    void SetBaseFilename(const string& _s) {m_problem->SetBaseFilename(_s);}

    ///@}
    ///@name Stat Class Accessor
    ///@{

    StatClass* GetStatClass() {return m_problem->GetStatClass();}

    ///@}
    ///@name Distance Metric Accessors
    ///@{

    DistanceMetricPointer GetDistanceMetric(const string& _l) {
      return m_distanceMetrics->GetMethod(_l);
    }
    void AddDistanceMetric(DistanceMetricPointer _dm, const string& _l) {
      m_distanceMetrics->AddMethod(_dm, _l);
    }

    ///@}
    ///@name Validity Checker Accessors
    ///@{

    ValidityCheckerPointer GetValidityChecker(const string& _l) {
      return m_validityCheckers->GetMethod(_l);
    }
    void AddValidityChecker(ValidityCheckerPointer _vc, const string& _l) {
      m_validityCheckers->AddMethod(_vc, _l);
    }
    void ToggleValidity() {
      for(auto& vc : *m_validityCheckers) vc.second->ToggleValidity();
    }

    ///@}
    ///@name Neighborhood Finder Accessors
    ///@{

    NeighborhoodFinderPointer GetNeighborhoodFinder(const string& _l) {
      return m_neighborhoodFinders->GetMethod(_l);
    }
    void AddNeighborhoodFinder(NeighborhoodFinderPointer _nf, const string& _l) {
      m_neighborhoodFinders->AddMethod(_nf, _l);
    }

    ///@}
    ///@name Sampler Accessors
    ///@{

    const SamplerSet* const GetSamplers() const {return m_samplers;}
    SamplerPointer GetSampler(const string& _l) {
      return m_samplers->GetMethod(_l);
    }
    void AddSampler(SamplerPointer _s, const string& _l) {
      m_samplers->AddMethod(_s, _l);
    }

    ///@}
    ///@name Local Planner Accessors
    ///@{

    LocalPlannerPointer GetLocalPlanner(const string& _l) {
      return m_localPlanners->GetMethod(_l);
    }
    void AddLocalPlanner(LocalPlannerPointer _lp, const string& _l) {
      m_localPlanners->AddMethod(_lp, _l);
    }

    ///@}
    ///@name Extender Accessors
    ///@{

    ExtenderPointer GetExtender(const string& _l) {
      return m_extenders->GetMethod(_l);
    }
    void AddExtender(ExtenderPointer _mps, const string& _l) {
      m_extenders->AddMethod(_mps, _l);
    }

    ///@}
    ///@name Path Modifier Accessors
    ///@{

    PathModifierPointer GetPathModifier(const string& _l) {
      return m_pathModifiers->GetMethod(_l);
    }
    void AddPathModifier(PathModifierPointer _ps, const string& _l) {
      m_pathModifiers->AddMethod(_ps, _l);
    }

    ///@}
    ///@name Connector Accessors
    ///@{

    ConnectorPointer GetConnector(const string& _l) {
      return m_connectors->GetMethod(_l);
    }
    void AddConnector(ConnectorPointer _c, const string& _l) {
      m_connectors->AddMethod(_c, _l);
    }

    ///@}
    ///@name Metric Accessors
    ///@{

    MetricPointer GetMetric(const string& _l) {return m_metrics->GetMethod(_l);}
    void AddMetric(MetricPointer _m, const string& _l) {
      m_metrics->AddMethod(_m, _l);
    }

    ///@}
    ///@name Map Evaluator Accessors
    ///@{

    string GetQueryFilename() const {return m_problem->GetQueryFilename();}
    void SetQueryFilename(const string& _s) {m_problem->SetQueryFilename(_s);}

    MapEvaluatorPointer GetMapEvaluator(const string& _l) {
      return m_mapEvaluators->GetMethod(_l);
    }
    void AddMapEvaluator(MapEvaluatorPointer _me, const string& _l) {
      m_mapEvaluators->AddMethod(_me, _l);
    }

    ///@}
    ///@name MPStrategy Accessors
    ///@{

    const MPStrategySet* const GetMPStrategies() const {return m_mpStrategies;}
    MPStrategyPointer GetMPStrategy(const string& _l) {
      return m_mpStrategies->GetMethod(_l);
    }
    void AddMPStrategy(MPStrategyPointer _mps, const string& _l) {
      m_mpStrategies->AddMethod(_mps, _l);
    }

    ///@}
    ///@name Execution Interface
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Get the current MPProblem.
    MPProblemType* GetMPProblem() {return m_problem;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Set the current MPProblem of all methods.
    void SetMPProblem(MPProblemType* _p);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add an input set to this MPLibrary.
    /// @param[in] _label        The MPStrategy label to use.
    /// @param[in] _seed         The random seed to use.
    /// @param[in] _baseFileName The base name of the XML file to use.
    /// @param[in] _vizmoDebug   Enable/disable vizmo debug.
    void AddSolver(const string& _label, long _seed,
        const string& _baseFileName, bool _vizmoDebug = false) {
      m_solvers.push_back(Solver(_label, _seed, _baseFileName, _vizmoDebug));
    }

    void Solve(); ///< Run each input (solver) in sequence.

    ///@}
    ///@name Debugging
    ///@{

    void Print(ostream& _os) const; ///< Print each method set.

    ///@}

  private:

    ///@name Construction Helpers
    ///@{

    void Initialize(); ///< Initialize all local method sets and data.

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Helper for parsing XML nodes.
    /// @param[in] _node The child node to be parsed.
    bool ParseChild(XMLNode& _node);

    ///@}
    ///@name Inputs
    ///@{

    MPProblemType* m_problem;   ///< The current MPProblem.
    vector<Solver> m_solvers;   ///< The set of inputs to execute.

    ///@}
    ///@name Method Sets
    ///@{
    /// Method sets hold and offer access to the motion planning objects of the
    /// corresponding type.

    DistanceMetricSet*     m_distanceMetrics{nullptr};
    ValidityCheckerSet*    m_validityCheckers{nullptr};
    NeighborhoodFinderSet* m_neighborhoodFinders{nullptr};
    SamplerSet*            m_samplers{nullptr};
    LocalPlannerSet*       m_localPlanners{nullptr};
    ExtenderSet*           m_extenders{nullptr};
    PathModifierSet*       m_pathModifiers{nullptr};
    ConnectorSet*          m_connectors{nullptr};
    MetricSet*             m_metrics{nullptr};
    MapEvaluatorSet*       m_mapEvaluators{nullptr};
    MPStrategySet*         m_mpStrategies{nullptr};

    ///@}

};

/*---------------------------- Construction ----------------------------------*/

template <typename MPTraits>
MPLibrary<MPTraits>::
MPLibrary() {
  Initialize();
};


template <typename MPTraits>
MPLibrary<MPTraits>::
MPLibrary(const string& _filename) {
  Initialize();
  ReadXMLFile(_filename);
}


template <typename MPTraits>
MPLibrary<MPTraits>::
~MPLibrary() {
  delete m_distanceMetrics;
  delete m_validityCheckers;
  delete m_neighborhoodFinders;
  delete m_samplers;
  delete m_localPlanners;
  delete m_extenders;
  delete m_pathModifiers;
  delete m_connectors;
  delete m_metrics;
  delete m_mapEvaluators;
  delete m_mpStrategies;
}


template <typename MPTraits>
void
MPLibrary<MPTraits>::
SetMPProblem(MPProblemType* _p) {
  m_problem = _p;

  // Set the current MPProblem for all algorithms.
  m_distanceMetrics->SetMPProblem(_p);
  m_validityCheckers->SetMPProblem(_p);
  m_neighborhoodFinders->SetMPProblem(_p);
  m_samplers->SetMPProblem(_p);
  m_localPlanners->SetMPProblem(_p);
  m_extenders->SetMPProblem(_p);
  m_pathModifiers->SetMPProblem(_p);
  m_connectors->SetMPProblem(_p);
  m_metrics->SetMPProblem(_p);
  m_mapEvaluators->SetMPProblem(_p);
  m_mpStrategies->SetMPProblem(_p);

  // Ensure CD structures have been built.
  auto& cdMethods = Body::m_cdMethods;
  for(auto& vc : *m_validityCheckers) {
    // Try to cast each validity checker to the collision detection base class.
    auto method =
        dynamic_pointer_cast<CollisionDetectionValidity<MPTraits>>(vc.second);

    // If the cast failed, then this isn't a CD method and we can move on.
    if(!method) continue;

    // Check if the method was already added.
    const auto& cd = method->GetCDMethod();
    const bool alreadyAdded = find(cdMethods.begin(), cdMethods.end(), cd) !=
        cdMethods.end();
    if(!alreadyAdded)
      cdMethods.push_back(cd);
  }
  m_problem->GetEnvironment()->BuildCDStructure();
}


template <typename MPTraits>
void
MPLibrary<MPTraits>::
Initialize() {
  m_distanceMetrics = new DistanceMetricSet(this,
      typename MPTraits::DistanceMetricMethodList(), "DistanceMetrics");
  m_validityCheckers = new ValidityCheckerSet(this,
      typename MPTraits::ValidityCheckerMethodList(), "ValidityCheckers");
  m_neighborhoodFinders = new NeighborhoodFinderSet(this,
      typename MPTraits::NeighborhoodFinderMethodList(), "NeighborhoodFinders");
  m_samplers = new SamplerSet(this,
      typename MPTraits::SamplerMethodList(), "Samplers");
  m_localPlanners = new LocalPlannerSet(this,
      typename MPTraits::LocalPlannerMethodList(), "LocalPlanners");
  m_extenders = new ExtenderSet(this,
      typename MPTraits::ExtenderMethodList(), "Extenders");
  m_pathModifiers = new PathModifierSet(this,
      typename MPTraits::PathModifierMethodList(), "PathModifiers");
  m_connectors = new ConnectorSet(this,
      typename MPTraits::ConnectorMethodList(), "Connectors");
  m_metrics = new MetricSet(this,
      typename MPTraits::MetricMethodList(), "Metrics");
  m_mapEvaluators = new MapEvaluatorSet(this,
      typename MPTraits::MapEvaluatorMethodList(), "MapEvaluators");
  m_mpStrategies = new MPStrategySet(this,
      typename MPTraits::MPStrategyMethodList(), "MPStrategies");
}

/*---------------------------- XML Helpers -----------------------------------*/

template <typename MPTraits>
void
MPLibrary<MPTraits>::
ReadXMLFile(const string& _filename) {
  // Open the XML and get the root node.
  XMLNode mpNode(_filename, "MotionPlanning");

  // Find the 'MPLibrary' node.
  XMLNode* planningLibrary = nullptr;
  for(auto& child : mpNode)
    if(child.Name() == "MPProblem")
      planningLibrary = &child;

  // Throw exception if we can't find it.
  if(!planningLibrary)
    throw ParseException(WHERE, "Cannot find MPLibrary node in XML file '"
        + _filename + "'.");

  // Parse the library node to set algorithms and parameters.
  for(auto& child : *planningLibrary)
    ParseChild(child);

  // Ensure we have at least one solver.
  if(m_solvers.empty())
    throw ParseException(WHERE, "Cannot find Solver node in XML file '" +
        _filename + "'.");

  // Print XML details if requested.
  bool print = mpNode.Read("print", false, false, "Print all XML input");
  if(print)
    Print(cout);

  // Handle XML warnings/errors.
  bool warnings = mpNode.Read("warnings", false, false, "Report warnings");
  if(warnings) {
    bool warningsAsErrors = mpNode.Read("warningsAsErrors", false, false,
        "XML warnings considered errors");
    planningLibrary->WarnAll(warningsAsErrors);
  }
}


template <typename MPTraits>
bool
MPLibrary<MPTraits>::
ParseChild(XMLNode& _node) {
  if(_node.Name() == "DistanceMetrics") {
    m_distanceMetrics->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "ValidityCheckers") {
    m_validityCheckers->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "NeighborhoodFinders") {
    m_neighborhoodFinders->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Samplers") {
    m_samplers->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "LocalPlanners") {
    m_localPlanners->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Extenders") {
    m_extenders->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "PathModifiers") {
    m_pathModifiers->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Connectors") {
    m_connectors->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Metrics") {
    m_metrics->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "MapEvaluators") {
    m_mapEvaluators->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "MPStrategies") {
    m_mpStrategies->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Solver") {
    string label = _node.Read("mpStrategyLabel", true, "", "The strategy pointed"
        " to by this label will be used to solve the problem");
    long seed = _node.Read("seed", true, 1, 0, MAX_INT,
        "The random number generator seed for the solver.");
    string baseFilename = _node.Read("baseFilename", true, "",
        "BaseFilename for the solver.");
    ostringstream oss;
    oss << baseFilename << "." << seed;
    baseFilename = oss.str();
    bool vdOutput = _node.Read("vizmoDebug", false, false,
        "True yields VizmoDebug output for the solver.");
    m_solvers.push_back(Solver(label, seed, baseFilename, vdOutput));
    return true;
  }
  else
    return false;
}

/*-------------------------------- Debugging ---------------------------------*/

template <typename MPTraits>
void
MPLibrary<MPTraits>::
Print(ostream& _os) const {
  _os << "MPLibrary" << endl;
  m_distanceMetrics->Print(_os);
  m_validityCheckers->Print(_os);
  m_neighborhoodFinders->Print(_os);
  m_samplers->Print(_os);
  m_localPlanners->Print(_os);
  m_extenders->Print(_os);
  m_pathModifiers->Print(_os);
  m_connectors->Print(_os);
  m_metrics->Print(_os);
  m_mapEvaluators->Print(_os);
  m_mpStrategies->Print(_os);
}

/*--------------------------- Execution Interface ----------------------------*/

template <typename MPTraits>
void
MPLibrary<MPTraits>::
Solve() {
  for(auto& sit : m_solvers) {

    // Call solver
    cout << "\n\nMPLibrary is solving with MPStrategyMethod labeled "
         << get<0>(sit) << " using seed " << get<1>(sit) << "." << endl;

#ifdef _PARALLEL
    SRand(get<1>(sit) + get_location_id());
#else
    SRand(get<1>(sit));
#endif

    SetBaseFilename(GetMPProblem()->GetPath(get<2>(sit)));
    GetStatClass()->SetAuxDest(GetBaseFilename());

    // Initialize vizmo debug if there is a valid filename
    if(get<3>(sit))
        VDInit(GetBaseFilename() + ".vd");

    GetMPStrategy(get<0>(sit))->operator()();

    // Close vizmo debug if necessary
    if(get<3>(sit))
      VDClose();
  }
}

/*----------------------------------------------------------------------------*/

#endif
