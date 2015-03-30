#ifndef MPPROBLEM_H_
#define MPPROBLEM_H_

#include "Utilities/MPUtils.h"
#include "Utilities/MetricUtils.h"
#include "MPProblem/Environment.h"
#include "MPProblem/Roadmap.h"

#include "DistanceMetrics/DistanceMetricMethod.h"
#include "ValidityCheckers/ValidityCheckerMethod.h"
#include "NeighborhoodFinders/NeighborhoodFinderMethod.h"
#include "Samplers/SamplerMethod.h"
#include "LocalPlanners/LocalPlannerMethod.h"
#include "Extenders/ExtenderMethod.h"
#include "PathModifiers/PathModifierMethod.h"
#include "Connectors/ConnectorMethod.h"
#include "Metrics/MetricMethod.h"
#include "MapEvaluators/MapEvaluatorMethod.h"
#include "MPStrategies/MPStrategyMethod.h"

#include "ValidityCheckers/CollisionDetectionValidity.h"

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
class MPProblem : public stapl::p_object
#else
class MPProblem
#endif
{
  public:
    typedef Roadmap<MPTraits> RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename GraphType::vertex_descriptor VID;

    MPProblem();
    MPProblem(const string& _filename);
    MPProblem(const string& _filename, typename MPTraits::MPProblemType* _problem);
    MPProblem(XMLNodeReader& _node, bool _parse = true);
    MPProblem(XMLNodeReader& _node, typename MPTraits::MPProblemType* _problem, bool _parse = true);
    virtual ~MPProblem();

    Environment* GetEnvironment() {return m_environment;};
    void SetEnvironment(Environment* _e) {m_environment = _e;};

    RoadmapType* GetRoadmap() { return m_roadmap; }
    void SetRoadmap(RoadmapType* _roadmap){m_roadmap = _roadmap;}
    RoadmapType* GetBlockRoadmap() { return m_blockRoadmap; }
    RoadmapType* GetColRoadmap() { return m_colRoadmap; }

    StatClass* GetStatClass() { return m_stats; }

    typedef MethodSet<MPTraits, DistanceMetricMethod<MPTraits> > DistanceMetricSet;
    typedef typename DistanceMetricSet::MethodPointer DistanceMetricPointer;
    DistanceMetricPointer GetDistanceMetric(const string& _l){return m_distanceMetrics->GetMethod(_l);}
    void AddDistanceMetric(DistanceMetricPointer _dm, const string& _l){m_distanceMetrics->AddMethod(_dm, _l);}

    typedef MethodSet<MPTraits, ValidityCheckerMethod<MPTraits> > ValidityCheckerSet;
    typedef typename ValidityCheckerSet::MethodPointer ValidityCheckerPointer;
    ValidityCheckerPointer GetValidityChecker(const string& _l){return m_validityCheckers->GetMethod(_l);}
    void AddValidityChecker(ValidityCheckerPointer _vc, const string& _l){m_validityCheckers->AddMethod(_vc, _l);}
    void ToggleValidity();

    typedef MethodSet<MPTraits, NeighborhoodFinderMethod<MPTraits> > NeighborhoodFinderSet;
    typedef typename NeighborhoodFinderSet::MethodPointer NeighborhoodFinderPointer;
    NeighborhoodFinderPointer GetNeighborhoodFinder(const string& _l){return m_neighborhoodFinders->GetMethod(_l);}
    void AddNeighborhoodFinder(NeighborhoodFinderPointer _nf, const string& _l){m_neighborhoodFinders->AddMethod(_nf, _l);}

    typedef MethodSet<MPTraits, SamplerMethod<MPTraits> > SamplerSet;
    typedef typename SamplerSet::MethodPointer SamplerPointer;
    SamplerPointer GetSampler(const string& _l){return m_samplers->GetMethod(_l);}
    void AddSampler(SamplerPointer _s, const string& _l){m_samplers->AddMethod(_s, _l);}

    typedef MethodSet<MPTraits, LocalPlannerMethod<MPTraits> > LocalPlannerSet;
    typedef typename LocalPlannerSet::MethodPointer LocalPlannerPointer;
    LocalPlannerPointer GetLocalPlanner(const string& _l){return m_localPlanners->GetMethod(_l);}
    void AddLocalPlanner(LocalPlannerPointer _lp, const string& _l){m_localPlanners->AddMethod(_lp, _l);}

    typedef MethodSet<MPTraits, ExtenderMethod<MPTraits> > ExtenderSet;
    typedef typename ExtenderSet::MethodPointer ExtenderPointer;
    ExtenderPointer GetExtender(const string& _l){return m_extenders->GetMethod(_l);}
    void AddExtender(ExtenderPointer _mps, const string& _l){m_extenders->AddMethod(_mps, _l);}

    typedef MethodSet<MPTraits, PathModifierMethod<MPTraits> > PathModifierSet;
    typedef typename PathModifierSet::MethodPointer PathModifierPointer;
    PathModifierPointer GetPathModifier(const string& _l){return m_pathModifiers->GetMethod(_l);}
    void AddPathModifier(PathModifierPointer _ps, const string& _l){m_pathModifiers->AddMethod(_ps, _l);}

    typedef MethodSet<MPTraits, ConnectorMethod<MPTraits> > ConnectorSet;
    typedef typename ConnectorSet::MethodPointer ConnectorPointer;
    ConnectorPointer GetConnector(const string& _l){return m_connectors->GetMethod(_l);}
    void AddConnector(ConnectorPointer _c, const string& _l){m_connectors->AddMethod(_c, _l);}

    typedef MethodSet<MPTraits, MetricMethod<MPTraits> > MetricSet;
    typedef typename MetricSet::MethodPointer MetricPointer;
    MetricPointer GetMetric(const string& _l){return m_metrics->GetMethod(_l);}
    void AddMetric(MetricPointer _m, const string& _l){m_metrics->AddMethod(_m, _l);}

    typedef MethodSet<MPTraits, MapEvaluatorMethod<MPTraits> > MapEvaluatorSet;
    typedef typename MapEvaluatorSet::MethodPointer MapEvaluatorPointer;
    MapEvaluatorPointer GetMapEvaluator(const string& _l){return m_mapEvaluators->GetMethod(_l);}
    void AddMapEvaluator(MapEvaluatorPointer _me, const string& _l){m_mapEvaluators->AddMethod(_me, _l);}

    //#ifndef _PARALLEL
    //MPCharacterizer<CfgType, WeightType>* GetCharacterizer(){return m_pCharacterizer;};

    //PartitioningMethods* GetPartitioningMethods(){return m_PartitioningMethods;}

    //PartitioningEvaluators* GetPartitioningEvaluators(){return m_PartitioningEvaluators;}

    //Features* GetFeatures(){return m_Features;}
    //#endif

    typedef MethodSet<MPTraits, MPStrategyMethod<MPTraits> > MPStrategySet;
    typedef typename MPStrategySet::MethodPointer MPStrategyPointer;
    MPStrategyPointer GetMPStrategy(const string& _l){return m_mpStrategies->GetMethod(_l);}
    void AddMPStrategy(MPStrategyPointer _mps, const string& _l){m_mpStrategies->AddMethod(_mps, _l);}

    virtual void Print(ostream& _os) const;

    void SetMPProblem();

    //solver, seed, baseName, vizmoDebugName
    typedef tuple<string, long, string, string> Solver;
    void AddSolver(const string& _label, long _seed,
        const string& _baseFileName, const string& _vizmoDebugName) {
      m_solvers.push_back(Solver(_label, _seed, _baseFileName, _vizmoDebugName));
    }
    void Solve();

    void BuildCDStructures();

    //AddObstacle, will add an obstacle to the environment
    //_modelFileName: a string specifying the path to the .obj file to be added as an obstacle
    //_where: 6 dofs for obstacle placement: (x, y, z, rotation about X, rotation about Y, rotation about Z)
    //return value: the obstacle's index in the Environment's m_otherMultiBodies on success, -1 on failure
    int AddObstacle(string _modelFileName, const Transformation& _where);

    //RemoveObstacleAt
    //a wrapper call to remove a multibody from the environment
    void RemoveObstacleAt(size_t _index);

  protected:
    virtual void Initialize();
    void ReadXMLFile(const string& _filename, typename MPTraits::MPProblemType* _problem);
    bool ParseChild(XMLNodeReader::childiterator citr, typename MPTraits::MPProblemType* _problem);
    virtual void ParseXML(XMLNodeReader& _node, typename MPTraits::MPProblemType* _problem);

    vector<cd_predefined> GetSelectedCDTypes() const;

    Environment* m_environment;
    RoadmapType* m_roadmap, * m_blockRoadmap, * m_colRoadmap;
    StatClass* m_stats;

    DistanceMetricSet* m_distanceMetrics;
    ValidityCheckerSet* m_validityCheckers;
    NeighborhoodFinderSet* m_neighborhoodFinders;
    SamplerSet* m_samplers;
    LocalPlannerSet* m_localPlanners;
    ExtenderSet* m_extenders;
    PathModifierSet* m_pathModifiers;
    ConnectorSet* m_connectors;
    MetricSet* m_metrics;
    MapEvaluatorSet* m_mapEvaluators;
//    //Characterization and Filtering
//#ifndef _PARALLEL
//    MPCharacterizer<CfgType, WeightType>* m_pCharacterizer;
//
//    //UAS items
//    PartitioningMethods* m_PartitioningMethods;
//    PartitioningEvaluators* m_PartitioningEvaluators;
//    Features* m_Features;
//#endif
    MPStrategySet* m_mpStrategies;

    vector<Solver> m_solvers;

  private:
    bool m_cdBuilt;
};

template<class MPTraits>
MPProblem<MPTraits>::MPProblem() {
  Initialize();
};

template<class MPTraits>
MPProblem<MPTraits>::MPProblem(const string& _filename) {
  Initialize();

  ReadXMLFile(_filename, this);
}

template<class MPTraits>
MPProblem<MPTraits>::MPProblem(const string& _filename, typename MPTraits::MPProblemType* _problem) {
  Initialize();

  ReadXMLFile(_filename, _problem);
}

template<class MPTraits>
MPProblem<MPTraits>::MPProblem(XMLNodeReader& _node, bool _parse) {
  Initialize();

  if(_parse)
    ParseXML(_node, this);
}

template<class MPTraits>
MPProblem<MPTraits>::MPProblem(XMLNodeReader& _node, typename MPTraits::MPProblemType* _problem, bool _parse) {
  Initialize();

  if(_parse)
    ParseXML(_node, _problem);
}

template<class MPTraits>
MPProblem<MPTraits>::~MPProblem() {
  delete m_roadmap;
  delete m_blockRoadmap;
  delete m_colRoadmap;
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
MPProblem<MPTraits>::Initialize(){
  m_environment = NULL;
  m_roadmap = new RoadmapType();
  m_blockRoadmap = new RoadmapType();
  m_colRoadmap = new RoadmapType();
  m_stats = new StatClass();

  m_distanceMetrics = new DistanceMetricSet(typename MPTraits::DistanceMetricMethodList(), "DistanceMetrics");
  m_validityCheckers = new ValidityCheckerSet(typename MPTraits::ValidityCheckerMethodList(), "ValidityCheckers");
  m_neighborhoodFinders = new NeighborhoodFinderSet(typename MPTraits::NeighborhoodFinderMethodList(), "NeighborhoodFinders");
  m_samplers = new SamplerSet(typename MPTraits::SamplerMethodList(), "Samplers");
  m_localPlanners = new LocalPlannerSet(typename MPTraits::LocalPlannerMethodList(), "LocalPlanners");
  m_extenders = new ExtenderSet(typename MPTraits::ExtenderMethodList(), "Extenders");
  m_pathModifiers = new PathModifierSet(typename MPTraits::PathModifierMethodList(), "PathModifiers");
  m_connectors = new ConnectorSet(typename MPTraits::ConnectorMethodList(), "Connectors");
  m_metrics = new MetricSet(typename MPTraits::MetricMethodList(), "Metrics");
  m_mapEvaluators = new MapEvaluatorSet(typename MPTraits::MapEvaluatorMethodList(), "MapEvaluators");
  m_mpStrategies = new MPStrategySet(typename MPTraits::MPStrategyMethodList(), "MPStrategies");

  m_cdBuilt = false;
}

template<class MPTraits>
void
MPProblem<MPTraits>::ReadXMLFile(const string& _filename, typename MPTraits::MPProblemType* _problem) {
  TiXmlDocument doc(_filename);
  bool loadOkay = doc.LoadFile();

  if (!loadOkay){
    cerr << "Error::Could not load test file " << _filename << ". XMLError=" << doc.ErrorDesc() << ". Exiting." << endl;
    exit(1);
  }

  XMLNodeReader mpNode(_filename, doc, "MotionPlanning");

  //Iterate over child nodes in search of MPProblem node within MotionPlanning
  //MotionPlanning should be removed in the future
  bool found = false;
  for(XMLNodeReader::childiterator citr = mpNode.children_begin(); citr != mpNode.children_end(); ++citr){
    if(citr->getName() == "MPProblem"){
      ParseXML(*citr, _problem);
      found = true;
      break;
    }
  }

  if(!found){
    cerr << "Error::Cannot find MPProblem XML node. Exiting." << endl;
    exit(1);
  }
}

template<class MPTraits>
bool
MPProblem<MPTraits>::ParseChild(XMLNodeReader::childiterator citr, typename MPTraits::MPProblemType* _problem) {
  if(citr->getName() == "Environment") {
    m_environment = new Environment(*citr);
    return true;
  }
  else if(citr->getName() == "DistanceMetrics") {
    m_distanceMetrics->ParseXML(_problem, *citr);
    return true;
  }
  else if(citr->getName() == "ValidityCheckers") {
    m_validityCheckers->ParseXML(_problem, *citr);
    return true;
  }
  else if(citr->getName() == "NeighborhoodFinders") {
    m_neighborhoodFinders->ParseXML(_problem, *citr);
    return true;
  }
  else if(citr->getName() == "Samplers") {
    m_samplers->ParseXML(_problem, *citr);
    return true;
  }
  else if(citr->getName() == "LocalPlanners") {
    m_localPlanners->ParseXML(_problem, *citr);
    return true;
  }
  else if(citr->getName() == "Extenders") {
    m_extenders->ParseXML(_problem, *citr);
    return true;
  }
  else if(citr->getName() == "PathModifiers") {
    m_pathModifiers->ParseXML(_problem, *citr);
    return true;
  }
  else if(citr->getName() == "Connectors") {
    m_connectors->ParseXML(_problem, *citr);
    return true;
  }
  else if(citr->getName() == "Metrics") {
    m_metrics->ParseXML(_problem, *citr);
    return true;
  }
  else if(citr->getName() == "MapEvaluators") {
    m_mapEvaluators->ParseXML(_problem, *citr);
    return true;
  }
  else if(citr->getName() == "MPStrategies") {
    m_mpStrategies->ParseXML(_problem, *citr);
    return true;
  }
  else if(citr->getName() == "Solver") {
    string m_label = citr->stringXMLParameter("mpStrategyLabel", true, "", "The strategy pointed to by this label will be used to solve the problem");
    long m_seed = citr->numberXMLParameter("seed", true, 1, 0, MAX_INT, "The random number generator seed for the solver.");
    string m_baseFilename = citr->stringXMLParameter("baseFilename", true, "", "BaseFilename for the solver.");
    ostringstream oss;
    oss << m_baseFilename << "." << m_seed;
    m_baseFilename = oss.str();
    bool m_vdOutput = citr->boolXMLParameter("vizmoDebug", false, false, "True yields VizmoDebug output for the solver.");
    string m_vizmoDebugName = "";
    if(m_vdOutput)
        m_vizmoDebugName = m_baseFilename + ".vd";
    m_solvers.push_back(Solver(m_label, m_seed, m_baseFilename, m_vizmoDebugName));
    return true;
  }
  else
    return false;
}

template<class MPTraits>
void
MPProblem<MPTraits>::ParseXML(XMLNodeReader& _node, typename MPTraits::MPProblemType* _problem) {
  _node.verifyName("MPProblem");

  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
    if(!ParseChild(citr, _problem))
      citr->warnUnknownNode();
  }

  BuildCDStructures();

  if(m_solvers.empty()){
    cerr << "Error::Must define a solver within the XML. Exiting." << endl;
    exit(1);
  }
}

template<class MPTraits>
void
MPProblem<MPTraits>::ToggleValidity(){
  typedef typename ValidityCheckerSet::MIT MIT;
  for(MIT mit = m_validityCheckers->Begin(); mit!= m_validityCheckers->End(); ++mit){
    mit->second->ToggleValidity();
  }
}

template<class MPTraits>
void
MPProblem<MPTraits>::Print(ostream& _os) const {
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

template<class MPTraits>
void
MPProblem<MPTraits>::SetMPProblem(){
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
MPProblem<MPTraits>::Solve() {
  if(!m_cdBuilt)
    BuildCDStructures();


  //Solver is tuple<MPStrategyMethod label, seed, base filename, vizmo debug filename>
  typedef vector<Solver>::iterator SIT;
  for(SIT sit = m_solvers.begin(); sit != m_solvers.end(); ++sit) {
    //clear roadmap structures
    delete m_roadmap;
    delete m_blockRoadmap;
    delete m_colRoadmap;
    delete m_stats;
    m_roadmap = new RoadmapType();
    m_blockRoadmap = new RoadmapType();
    m_colRoadmap = new RoadmapType();
    m_stats = new StatClass();

    //initialize vizmo debug if there is a valid filename
    string vdfilename = get<3>(*sit);
    if(vdfilename != "")
      VDInit(vdfilename);

    //call solver
    cout << "\n\nMPProblem is solving with MPStrategyMethod labeled "
      << get<0>(*sit)
      << " using seed " << get<1>(*sit) << "." << endl;
#ifdef _PARALLEL
    SRand(get<1>(*sit) + get_location_id());
#else
    SRand(get<1>(*sit));
#endif
    GetMPStrategy(get<0>(*sit))->SetBaseFilename(get<2>(*sit));
    GetMPStrategy(get<0>(*sit))->operator()();

    //close vizmo debug if necessary
    if(vdfilename != "")
      VDClose();
  }
};

template<class MPTraits>
void
MPProblem<MPTraits>::BuildCDStructures(){
  if(m_environment != NULL){
    vector<cd_predefined> cdtypes = GetSelectedCDTypes();
    for(vector<cd_predefined>::iterator C = cdtypes.begin(); C != cdtypes.end(); ++C)
      m_environment->BuildCDstructure(*C);
  }
  else{
    cerr << "Error::Cannot BuildCDStructures. Must define an environment. Exiting." << endl;
    exit(1);
  }
  m_cdBuilt = true;
}

template<class MPTraits>
vector<cd_predefined>
MPProblem<MPTraits>::GetSelectedCDTypes() const{
  vector<cd_predefined> cdTypes;
  typedef typename ValidityCheckerSet::MIT MIT;
  for(MIT mit = m_validityCheckers->Begin(); mit != m_validityCheckers->End(); ++mit)
    if(CollisionDetectionValidity<MPTraits>* method = dynamic_cast<CollisionDetectionValidity<MPTraits>*>(mit->second.get()))
      cdTypes.push_back(method->GetCDType());
  return cdTypes;
}

template<class MPTraits>
int
MPProblem<MPTraits>::AddObstacle(string _modelFileName, const Transformation& _where){
  if(m_environment != NULL){
    vector<cd_predefined> cdtypes = GetSelectedCDTypes();
    int index =  m_environment->AddObstacle(_modelFileName, _where, cdtypes);
    return index;
  }
  else{
    cerr << "MPProblem::AddObstacle Warning: Attempted to add obstacle to a NULL environment" << endl;
    return -1;
  }
}

template<class MPTraits>
void
MPProblem<MPTraits>::RemoveObstacleAt(size_t _index){
  if(m_environment != NULL){
    m_environment->RemoveObstacleAt(_index);
  }
  else{
    cerr << "MPProblem::RemoveObstacleAt Warning: Attempted to remove an obstacle from a NULL environment" << endl;
  }
}

#endif
