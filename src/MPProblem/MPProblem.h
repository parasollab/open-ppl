#ifndef MP_PROBLEM_H_
#define MP_PROBLEM_H_

#include "MPProblemBase.h"
#include "Roadmap.h"

#include "Environment/Environment.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"

#include "Connectors/ConnectorMethod.h"
#include "DistanceMetrics/DistanceMetricMethod.h"
#include "Extenders/ExtenderMethod.h"
#include "LocalPlanners/LocalPlannerMethod.h"
#include "MapEvaluators/MapEvaluatorMethod.h"
#include "Metrics/MetricMethod.h"
#include "MPStrategies/MPStrategyMethod.h"
#include "NeighborhoodFinders/NeighborhoodFinderMethod.h"
#include "PathModifiers/PathModifierMethod.h"
#include "Samplers/SamplerMethod.h"
#include "ValidityCheckers/CollisionDetectionValidity.h"
#include "ValidityCheckers/ValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief Central access hub of PMPL.
///
/// The MPProblem is a central class to PMPL. It "stores everything". It is a
/// hub of accessing all algorithmic abstractions, roadmaps, statistics
/// classes, the environment, etc.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
#ifdef _PARALLEL
class MPProblem : public stapl::p_object, public MPProblemBase
#else
class MPProblem : public MPProblemBase
#endif
{

  public:

    ///\name General Abstraction Types
    ///@{

    typedef typename MPTraits::MPProblemType      MPProblemType;
    typedef Roadmap<MPTraits>                     RoadmapType;
    typedef typename RoadmapType::GraphType       GraphType;
    typedef typename GraphType::vertex_descriptor VID;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Solver represents an input set to MPProblem. It includes an
    ///        MPStrategy label, seed, base file name, and vizmo debug option.
    typedef tuple<string, long, string, bool> Solver;

    ///@}
    ///\name Method Set Types
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
    ///\name Method Pointer Types
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
    ///\name Construction
    ///@{

    MPProblem();
    MPProblem(const string& _filename);
    MPProblem(const string& _filename, MPProblemType* _problem);
    virtual ~MPProblem();

    void SetMPProblem(); ///< Set the MPProblem of all methods to this.

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Read an XML file for parsing into an MPProblem.
    /// \param[in] _filename    The XML file name.
    /// \param[in,out] _problem The MPProblem to parse into.
    void ReadXMLFile(const string& _filename, MPProblemType* _problem);

    ///@}
    ///\name Base Filename Accessors
    ///@{

    const string& GetBaseFilename() const {return m_baseFilename;}
    void SetBaseFilename(const string& _s) {m_baseFilename = _s;}

    ///@}
    ///\name Environment Accessors
    ///@{

    Environment* GetEnvironment() {return m_environment;}
    void SetEnvironment(Environment* _e) {m_environment = _e;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add an obstacle to the Environment
    /// @param _modelFileName .obj file to be added as an obstacle
    /// @param _where Obstacle placement
    /// @return the obstacle's index in the Environment's m_otherMultiBodies on
    ///         success, -1 on failure
    pair<size_t, shared_ptr<StaticMultiBody>> AddObstacle(
        const string& _modelFileName,
        const Transformation& _where = Transformation());

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Remove an obstacle from the Environment.
    /// \param[in] _index The obstalce's index.
    void RemoveObstacle(size_t _index);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Initialize collision detectors for this environment.
    void BuildCDStructures();

    ///@}
    ///\name Roadmap Accessors
    ///@{

    RoadmapType* GetRoadmap() {return m_roadmap;}
    void SetRoadmap(RoadmapType* _roadmap) {m_roadmap = _roadmap;}
    RoadmapType* GetBlockRoadmap() {return m_blockRoadmap;}

    ///@}
    ///\name Stat Class Accessor
    ///@{

    StatClass* GetStatClass() {return m_stats;}

    ///@}
    ///\name Distance Metric Accessors
    ///@{

    DistanceMetricPointer GetDistanceMetric(const string& _l) {
      return m_distanceMetrics->GetMethod(_l);
    }
    void AddDistanceMetric(DistanceMetricPointer _dm, const string& _l) {
      m_distanceMetrics->AddMethod(_dm, _l);
    }

    ///@}
    ///\name Validity Checker Accessors
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
    ///\name Neighborhood Finder Accessors
    ///@{

    NeighborhoodFinderPointer GetNeighborhoodFinder(const string& _l) {
      return m_neighborhoodFinders->GetMethod(_l);
    }
    void AddNeighborhoodFinder(NeighborhoodFinderPointer _nf, const string& _l) {
      m_neighborhoodFinders->AddMethod(_nf, _l);
    }

    ///@}
    ///\name Sampler Accessors
    ///@{

    const SamplerSet* const GetSamplers() const {return m_samplers;}
    SamplerPointer GetSampler(const string& _l) {
      return m_samplers->GetMethod(_l);
    }
    void AddSampler(SamplerPointer _s, const string& _l) {
      m_samplers->AddMethod(_s, _l);
    }

    ///@}
    ///\name Local Planner Accessors
    ///@{

    LocalPlannerPointer GetLocalPlanner(const string& _l) {
      return m_localPlanners->GetMethod(_l);
    }
    void AddLocalPlanner(LocalPlannerPointer _lp, const string& _l) {
      m_localPlanners->AddMethod(_lp, _l);
    }

    ///@}
    ///\name Extender Accessors
    ///@{

    ExtenderPointer GetExtender(const string& _l) {
      return m_extenders->GetMethod(_l);
    }
    void AddExtender(ExtenderPointer _mps, const string& _l) {
      m_extenders->AddMethod(_mps, _l);
    }

    ///@}
    ///\name Path Modifier Accessors
    ///@{

    PathModifierPointer GetPathModifier(const string& _l) {
      return m_pathModifiers->GetMethod(_l);
    }
    void AddPathModifier(PathModifierPointer _ps, const string& _l) {
      m_pathModifiers->AddMethod(_ps, _l);
    }

    ///@}
    ///\name Connector Accessors
    ///@{

    ConnectorPointer GetConnector(const string& _l) {
      return m_connectors->GetMethod(_l);
    }
    void AddConnector(ConnectorPointer _c, const string& _l) {
      m_connectors->AddMethod(_c, _l);
    }

    ///@}
    ///\name Metric Accessors
    ///@{

    MetricPointer GetMetric(const string& _l) {return m_metrics->GetMethod(_l);}
    void AddMetric(MetricPointer _m, const string& _l) {
      m_metrics->AddMethod(_m, _l);
    }

    ///@}
    ///\name Map Evaluator Accessors
    ///@{

    string GetQueryFilename() const {return m_queryFilename;}
    void SetQueryFilename(const string& _s) {m_queryFilename = _s;}

    MapEvaluatorPointer GetMapEvaluator(const string& _l) {
      return m_mapEvaluators->GetMethod(_l);
    }
    void AddMapEvaluator(MapEvaluatorPointer _me, const string& _l) {
      m_mapEvaluators->AddMethod(_me, _l);
    }

    ///@}
    ///\name MPStrategy Accessors
    ///@{

    const MPStrategySet* const GetMPStrategies() const {return m_mpStrategies;}
    MPStrategyPointer GetMPStrategy(const string& _l) {
      return m_mpStrategies->GetMethod(_l);
    }
    void AddMPStrategy(MPStrategyPointer _mps, const string& _l) {
      m_mpStrategies->AddMethod(_mps, _l);
    }

    ///@}
    ///\name Execution Interface
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Add an input set to this MPProblem.
    /// \param[in] _label        The MPStrategy label to use.
    /// \param[in] _seed         The random seed to use.
    /// \param[in] _baseFileName The base name of the XML file to use.
    /// \param[in] _vizmoDebug   Enable/disable vizmo debug.
    void AddSolver(const string& _label, long _seed,
        const string& _baseFileName, bool _vizmoDebug = false) {
      m_solvers.push_back(Solver(_label, _seed, _baseFileName, _vizmoDebug));
    }
    void Solve(); ///< Run each input (solver) in sequence.

    ///@}
    ///\name Debugging
    ///@{

    virtual void Print(ostream& _os) const; ///< Print each method set.

  protected:

    ///@}
    ///\name Construction Helpers
    ///@{

    virtual void Initialize(); ///< Initialize all local method sets and data.

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Helper for parsing XML nodes.
    /// \param[in] _node The child node to be parsed.
    /// \param[in,out]   The MPProblem to parse into.
    bool ParseChild(XMLNode& _node, MPProblemType* _problem);

    ///@}
    ///\name Core Properties
    ///@{

    string       m_baseFilename; ///< The base name for output files.
    Environment* m_environment;  ///< The environment to plan in.
    RoadmapType* m_roadmap;      ///< The free-space roadmap.
    RoadmapType* m_blockRoadmap; ///< The obstacle-space roadmap.
    StatClass*   m_stats;        ///< Performance tracking object.

    ///@}
    ///\name Inputs
    ///@{

    string         m_queryFilename; ///< The query file name.
    vector<Solver> m_solvers;       ///< The set of inputs to execute.

    ///@}
    ///\name Method Sets
    ///@{
    /// \brief Method sets hold and offer access to the motion planning objects
    ///        of the corresponding type.

    DistanceMetricSet*     m_distanceMetrics;
    ValidityCheckerSet*    m_validityCheckers;
    NeighborhoodFinderSet* m_neighborhoodFinders;
    SamplerSet*            m_samplers;
    LocalPlannerSet*       m_localPlanners;
    ExtenderSet*           m_extenders;
    PathModifierSet*       m_pathModifiers;
    ConnectorSet*          m_connectors;
    MetricSet*             m_metrics;
    MapEvaluatorSet*       m_mapEvaluators;
    MPStrategySet*         m_mpStrategies;

    ///@}

  private:

    bool m_cdBuilt; ///< Indicates whether collision detectors are initialized.
};

/*---------------------------- Construction ----------------------------------*/

template<class MPTraits>
MPProblem<MPTraits>::
MPProblem() {
  Initialize();
};


template<class MPTraits>
MPProblem<MPTraits>::
MPProblem(const string& _filename) {
  Initialize();
  ReadXMLFile(_filename, this);
}


template<class MPTraits>
MPProblem<MPTraits>::
MPProblem(const string& _filename, MPProblemType* _problem) {
  Initialize();
  ReadXMLFile(_filename, _problem);
}


//template<class MPTraits>
//MPProblem<MPTraits>::
//MPProblem(XMLNode& _node, bool _parse) {
//  Initialize();
//  if(_parse)
//    ParseXML(_node, this);
//}
//
//
//template<class MPTraits>
//MPProblem<MPTraits>::
//MPProblem(XMLNode& _node, MPProblemType* _problem, bool _parse) {
//  Initialize();
//  if(_parse)
//    ParseXML(_node, _problem);
//}


template<class MPTraits>
MPProblem<MPTraits>::
~MPProblem() {
  delete m_roadmap;
  delete m_blockRoadmap;
  delete m_stats;
  delete m_environment;
  delete m_distanceMetrics;
  delete m_validityCheckers;
  delete m_neighborhoodFinders;
  delete m_samplers;
  delete m_localPlanners;
  delete m_pathModifiers;
  delete m_connectors;
  delete m_metrics;
  delete m_mapEvaluators;
  delete m_mpStrategies;
}


template<class MPTraits>
void
MPProblem<MPTraits>::
SetMPProblem(){
  m_distanceMetrics->SetMPProblem(this);
  m_validityCheckers->SetMPProblem(this);
  m_neighborhoodFinders->SetMPProblem(this);
  m_samplers->SetMPProblem(this);
  m_localPlanners->SetMPProblem(this);
  m_extenders->SetMPProblem(this);
  m_pathModifiers->SetMPProblem(this);
  m_connectors->SetMPProblem(this);
  m_metrics->SetMPProblem(this);
  m_mapEvaluators->SetMPProblem(this);
  m_mpStrategies->SetMPProblem(this);
}


template<class MPTraits>
void
MPProblem<MPTraits>::
Initialize() {
  m_environment = nullptr;
  m_roadmap = new RoadmapType();
  m_blockRoadmap = new RoadmapType();
  m_stats = new StatClass();

  m_distanceMetrics = new DistanceMetricSet(
      typename MPTraits::DistanceMetricMethodList(), "DistanceMetrics");
  m_validityCheckers = new ValidityCheckerSet(
      typename MPTraits::ValidityCheckerMethodList(), "ValidityCheckers");
  m_neighborhoodFinders = new NeighborhoodFinderSet(
      typename MPTraits::NeighborhoodFinderMethodList(), "NeighborhoodFinders");
  m_samplers = new SamplerSet(
      typename MPTraits::SamplerMethodList(), "Samplers");
  m_localPlanners = new LocalPlannerSet(
      typename MPTraits::LocalPlannerMethodList(), "LocalPlanners");
  m_extenders = new ExtenderSet(
      typename MPTraits::ExtenderMethodList(), "Extenders");
  m_pathModifiers = new PathModifierSet(
      typename MPTraits::PathModifierMethodList(), "PathModifiers");
  m_connectors = new ConnectorSet(
      typename MPTraits::ConnectorMethodList(), "Connectors");
  m_metrics = new MetricSet(
      typename MPTraits::MetricMethodList(), "Metrics");
  m_mapEvaluators = new MapEvaluatorSet(
      typename MPTraits::MapEvaluatorMethodList(), "MapEvaluators");
  m_mpStrategies = new MPStrategySet(
      typename MPTraits::MPStrategyMethodList(), "MPStrategies");

  m_cdBuilt = false;
}

/*---------------------------- XML Helpers -----------------------------------*/

template<class MPTraits>
void
MPProblem<MPTraits>::
ReadXMLFile(const string& _filename, MPProblemType* _problem) {
  bool envIsSet = m_environment;

  size_t sl = _filename.rfind("/");
  m_filePath = _filename.substr(0, sl == string::npos ? 0 : sl + 1);

  // Open the XML and get the root node.
  XMLNode mpNode(_filename, "MotionPlanning");

  // Find the 'Input' and 'MPProblem' nodes.
  XMLNode* input = nullptr;
  XMLNode* mp    = nullptr;
  for(auto& child : mpNode) {
    if(child.Name() == "Input")
      input = &child;
    else if(child.Name() == "MPProblem")
      mp = &child;
  }

  // Throw exceptions if we can't find those nodes.
  if(!envIsSet && !input)
    throw ParseException(WHERE, "Cannot find Input node in XML file '" +
        _filename + "'.");
  if(!mp)
    throw ParseException(WHERE, "Cannot find MPProblem node in XML file '" +
        _filename + "'.");

  // Parse the input node first to set the environment, then parse the MP node
  // to set algorithms and parameters.
  if(!envIsSet)
    for(auto& child : *input)
      ParseChild(child, _problem);
  for(auto& child : *mp)
    ParseChild(child, _problem);

  // Ensure we have at least one solver.
  if(m_solvers.empty())
    throw ParseException(WHERE, "Cannot find Solver node in XML file '" +
        _filename + "'.");

  BuildCDStructures();

  // Print XML details if requested.
  bool print = mpNode.Read("print", false, false, "Print all XML input");
  if(print)
    Print(cout);

  // Handle XML warnings/errors.
  bool warnings = mpNode.Read("warnings", false, false, "Report warnings");
  if(warnings) {
    bool warningsAsErrors = mpNode.Read("warningsAsErrors", false, false,
        "XML warnings considered errors");
    if(envIsSet)
      mp->WarnAll(warningsAsErrors);
    else
      mpNode.WarnAll(warningsAsErrors);
  }
}


template<class MPTraits>
bool
MPProblem<MPTraits>::
ParseChild(XMLNode& _node, MPProblemType* _problem) {
  if(_node.Name() == "Environment") {
    // Ignore this node if we already have an environment.
    if(!m_environment)
      m_environment = new Environment(_node);
    return true;
  }
  if(_node.Name() == "Query") {
    // Ignore this node if we already have a query file.
    if(m_queryFilename.empty())
      m_queryFilename = _node.Read("filename", false, "", "Query file name");
    return true;
  }
  else if(_node.Name() == "DistanceMetrics") {
    m_distanceMetrics->ParseXML(_problem, _node);
    return true;
  }
  else if(_node.Name() == "ValidityCheckers") {
    m_validityCheckers->ParseXML(_problem, _node);
    return true;
  }
  else if(_node.Name() == "NeighborhoodFinders") {
    m_neighborhoodFinders->ParseXML(_problem, _node);
    return true;
  }
  else if(_node.Name() == "Samplers") {
    m_samplers->ParseXML(_problem, _node);
    return true;
  }
  else if(_node.Name() == "LocalPlanners") {
    m_localPlanners->ParseXML(_problem, _node);
    return true;
  }
  else if(_node.Name() == "Extenders") {
    m_extenders->ParseXML(_problem, _node);
    return true;
  }
  else if(_node.Name() == "PathModifiers") {
    m_pathModifiers->ParseXML(_problem, _node);
    return true;
  }
  else if(_node.Name() == "Connectors") {
    m_connectors->ParseXML(_problem, _node);
    return true;
  }
  else if(_node.Name() == "Metrics") {
    m_metrics->ParseXML(_problem, _node);
    return true;
  }
  else if(_node.Name() == "MapEvaluators") {
    m_mapEvaluators->ParseXML(_problem, _node);
    return true;
  }
  else if(_node.Name() == "MPStrategies") {
    m_mpStrategies->ParseXML(_problem, _node);
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

template<class MPTraits>
void
MPProblem<MPTraits>::
Print(ostream& _os) const {
  _os << "MPProblem" << endl;
  m_environment->Print(_os);
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

template<class MPTraits>
void
MPProblem<MPTraits>::
Solve() {
  if(!m_cdBuilt)
    BuildCDStructures();

  for(auto& sit : m_solvers) {
    // Clear roadmap structures
    delete m_roadmap;
    delete m_blockRoadmap;
    delete m_stats;
    m_roadmap = new RoadmapType();
    m_blockRoadmap = new RoadmapType();
    m_stats = new StatClass();

    // Call solver
    cout << "\n\nMPProblem is solving with MPStrategyMethod labeled "
         << get<0>(sit) << " using seed " << get<1>(sit) << "." << endl;

#ifdef _PARALLEL
    SRand(get<1>(sit) + get_location_id());
#else
    SRand(get<1>(sit));
#endif

    SetBaseFilename(GetPath(get<2>(sit)));
    m_stats->SetAuxDest(GetBaseFilename());

    // Initialize vizmo debug if there is a valid filename
    if(get<3>(sit))
        VDInit(m_baseFilename + ".vd");

    GetMPStrategy(get<0>(sit))->operator()();

    // Close vizmo debug if necessary
    if(get<3>(sit))
      VDClose();
  }
}

/*------------------------ Environment Accessors -----------------------------*/

template<class MPTraits>
pair<size_t, shared_ptr<StaticMultiBody>>
MPProblem<MPTraits>::
AddObstacle(const string& _modelFileName, const Transformation& _where) {
  if(m_environment) {
    return m_environment->AddObstacle("", _modelFileName, _where);
  }
  else {
    cerr << "MPProblem::AddObstacle Warning: "
         << "Attempted to add obstacle to a NULL environment" << endl;
    return make_pair(-1, nullptr);
  }
}


template<class MPTraits>
void
MPProblem<MPTraits>::
RemoveObstacle(size_t _index) {
  if(m_environment)
    m_environment->RemoveObstacle(_index);
  else
    cerr << "MPProblem::RemoveObstacleAt Warning: Attempted to remove an "
         << "obstacle from a NULL environment" << endl;
}


template<class MPTraits>
void
MPProblem<MPTraits>::
BuildCDStructures() {
  if(m_environment) {
    Body::m_cdMethods.clear();
    for(auto& vc : *m_validityCheckers)
      if(shared_ptr<CollisionDetectionValidity<MPTraits>> method =
          dynamic_pointer_cast<CollisionDetectionValidity<MPTraits>>(vc.second))
        Body::m_cdMethods.push_back(method->GetCDMethod());
    m_environment->BuildCDStructure();
  }
  else
    throw RunTimeException(WHERE,
        "Cannot Build CD Structures. Must define an Environment.");
  m_cdBuilt = true;
}

/*----------------------------------------------------------------------------*/

#endif
