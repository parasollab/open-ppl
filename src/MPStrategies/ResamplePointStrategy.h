#ifndef RESAMPLEPOINTSTRATEGY_H_ 
#define RESAMPLEPOINTSTRATEGY_H_ 

#include "MPStrategyMethod.h"

template<class MPTraits>
class ResamplePointStrategy : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    
    ResamplePointStrategy(); 
    ResamplePointStrategy(MPProblemType* _problem, XMLNodeReader& _node); 
    virtual ~ResamplePointStrategy();

    virtual void PrintOptions(ostream& _os);
    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

  protected:
    vector<pair<CfgType, double> > FindNeighbors(CfgType& _previous, CfgType& _current, CfgType& _next, size_t _maxAttempts);

    string m_inputPathFilename;
    string m_outputPathFilename;
    string m_inputMapFilename;
    string m_outputMapFilename;
    string m_typeName;
    string m_dmLabel;
    string m_lpLabel;
    size_t m_numResamples;
    double m_stepSize;
    double m_userValue;
    ClearanceUtility<MPTraits> m_clearanceUtils;

    vector<CfgType> m_pathCfgs, m_outputPathCfgs;
};

template<class MPTraits>
ResamplePointStrategy<MPTraits>::ResamplePointStrategy() {
  this->m_name = "ResamplePointStrategy";
}

template<class MPTraits>
ResamplePointStrategy<MPTraits>::ResamplePointStrategy(MPProblemType* _problem, XMLNodeReader& _node) : MPStrategyMethod<MPTraits>(_problem, _node) {
  this->m_name = "ResamplePointStrategy";
  ParseXML(_node); 
}

template<class MPTraits>
ResamplePointStrategy<MPTraits>::~ResamplePointStrategy(){}

template<class MPTraits>
void
ResamplePointStrategy<MPTraits>::PrintOptions(ostream& _os){
  _os << "ResamplePointStrategy::\n";
  _os << "\tinputPathFilename = \"" << m_inputPathFilename << "\"\n";
  _os << "\toutputPathFilename = \"" << m_outputPathFilename << "\"\n";
  _os << "\tinputMapFilename = \"" << m_inputMapFilename << "\"\n";
  _os << "\toutputMapFilename = \"" << m_outputMapFilename << "\"\n";
  _os << "\tnumResamples = " << m_numResamples << endl;
  if(m_typeName == "MAX_CLEARANCE"){
    _os << "\tclearanceUtils::" << endl;
    m_clearanceUtils.PrintOptions(_os);
  }
}

template<class MPTraits>
void
ResamplePointStrategy<MPTraits>::ParseXML(XMLNodeReader& _node) {
  m_inputPathFilename = _node.stringXMLParameter("inputPath", true, "", "filename containing path to be resampled");
  m_outputPathFilename = _node.stringXMLParameter("outputPath", false, "", "filename to output resampled path to, default = input_path");
  if(m_outputPathFilename == "") m_outputPathFilename = m_inputPathFilename;
  m_inputMapFilename = _node.stringXMLParameter("inputMap", true, "", "filename containing map to be resampled");
  m_outputMapFilename = _node.stringXMLParameter("outputMap", false, "", "filename to output resampled map to, default = input_map");
  if(m_outputMapFilename == "") m_outputMapFilename = m_inputMapFilename;
  m_numResamples = _node.numberXMLParameter("numResamples", false, 5, 1, MAX_INT, "number of samples to generate in each resample attempt");
  m_stepSize= _node.numberXMLParameter("stepSize", false, 0.0, 0.0, MAX_DBL, "distance of a resampled node from existing one");
  m_userValue= _node.numberXMLParameter("userValue", false, 0.3, 0.0, MAX_DBL, "distance of a resampled node from existing one");
  m_typeName = _node.stringXMLParameter("typeName", true, "", "type of the CFG task");
  m_dmLabel = _node.stringXMLParameter("dmLabel", true, "", "Distance metric");
  m_lpLabel = _node.stringXMLParameter("lpLabel", true, "", "Local Planner");
 
  if(m_typeName == "MAX_CLEARANCE"){
    m_clearanceUtils = ClearanceUtility<MPTraits>(this->GetMPProblem(), _node);
  }

  _node.warnUnrequestedAttributes();
}

struct SortAscend {
  template<typename T>
  bool operator()(const std::pair<T, double> &left, const std::pair<T, double> &right) {
    return left.second < right.second;
  }
};

struct SortDescend {
  template<typename T>
  bool operator()(const std::pair<T, double> &left, const std::pair<T, double> &right) {
    return left.second > right.second;
  }
};

template<class MPTraits>
void
ResamplePointStrategy<MPTraits>::Initialize() {
  //load input map
  this->GetMPProblem()->GetRoadmap()->Read(m_inputMapFilename.c_str());
  
  //load input path
  ifstream inPath(m_inputPathFilename.c_str());
  
  if(this->m_debug)
    cout<<m_inputPathFilename.c_str();
  
  size_t numCfgs = 0;
  if(m_typeName == "MAX_CLEARANCE"){//max clearance
    string line;
    getline(inPath, line); //skip header line
    getline(inPath, line); //skip header line  
  }
  inPath >> numCfgs;
  for(size_t i=0; i<numCfgs; ++i) {
    CfgType c;
    c.Read(inPath);
    m_pathCfgs.push_back(c);
  }
  inPath.close();
}

template<class MPTraits>
void
ResamplePointStrategy<MPTraits>::Run() {
  double currentConfigurationWeight = 0.0;
  double temp = 0.0;
  size_t maxAttempts = 1000;
  
  if(this->m_debug){
    cout << "ResamplePointStrategy::Run" << endl;
    cout << "\ntypeName::" << m_typeName << endl;
    cout << "numResamples::" << m_numResamples << endl;
    cout << "stepSize::" << m_stepSize << endl;
    cout << "userValue::" << m_userValue << endl;
  }

  StatClass* stats = this->GetMPProblem()->GetStatClass();
  stats->StartClock("Resampling");
  
  vector<double> smoothingValues;
  
  typename vector<CfgType>::iterator cit = m_pathCfgs.begin();
  m_outputPathCfgs.push_back(*cit);
  
  
  if(this->m_debug){ 
    temp = cit->GetSmoothingValue(m_clearanceUtils, this->m_boundary);
    cout << "oldsmoothingvalues:" << endl << temp << ";";
    smoothingValues.push_back(temp);
  }

  ++cit;
  
  for(; cit+1 != m_pathCfgs.end(); ++cit) {

    currentConfigurationWeight = cit->GetSmoothingValue(m_clearanceUtils, this->m_boundary);
    if(this->m_debug) cout << currentConfigurationWeight << ";";
    if((currentConfigurationWeight > m_userValue && m_typeName == "PROTEIN_ENERGY") ||
        (currentConfigurationWeight < m_userValue && m_typeName == "MAX_CLEARANCE")) {

      CfgType previous = *(cit-1);
      CfgType current = *cit;
      CfgType next = *(cit+1);
      
      vector<pair<CfgType, double> > sampledNeighbors = FindNeighbors(previous, current, next, maxAttempts); 

      if(sampledNeighbors.size() > 0) {
        typename vector<pair<CfgType, double> >::iterator nit = sampledNeighbors.begin();
        m_outputPathCfgs.push_back(nit->first);
        *cit = nit->first;
        if(this->m_debug){ 
          temp = nit->first.GetSmoothingValue(m_clearanceUtils, this->m_boundary);
          smoothingValues.push_back(temp);
        }
      }
      else{
        m_outputPathCfgs.push_back(*cit);
        if(this->m_debug){
          temp = cit->GetSmoothingValue(m_clearanceUtils, this->m_boundary);
          smoothingValues.push_back(temp);
        }
      }
    }
    else {
      m_outputPathCfgs.push_back(*cit);
      if(this->m_debug){
        temp = cit->GetSmoothingValue(m_clearanceUtils, this->m_boundary);
        smoothingValues.push_back(temp);
      }
    }
  }//for

  m_outputPathCfgs.push_back(*cit);
  if(this->m_debug){
    temp = cit->GetSmoothingValue(m_clearanceUtils, this->m_boundary);
    smoothingValues.push_back(temp);
  }

  stats->StopClock("Resampling");

  if(this->m_debug){
    stats->PrintClock("Resampling", cout); 
    cout << endl;
    cout << "newsmoothingvalues:" << endl;
    for(vector<double>::iterator dit = smoothingValues.begin(); dit!=smoothingValues.end(); ++dit)  {
      cout << *dit << ";";
    }
    cout<<endl;
  }
}

template<class MPTraits>
void
ResamplePointStrategy<MPTraits>::Finalize() {
  WritePathConfigurations(m_outputPathFilename.c_str(), m_outputPathCfgs, this->GetMPProblem()->GetEnvironment()); 
  //write output map
  ofstream osMap(m_outputMapFilename.c_str());
  this->GetMPProblem()->GetRoadmap()->Write(osMap, this->GetMPProblem()->GetEnvironment());
  osMap.close();
}

template<class MPTraits>
vector<pair<typename MPTraits::CfgType, double> >
ResamplePointStrategy<MPTraits>::FindNeighbors(CfgType& _previous, CfgType& _current, CfgType& _next, size_t _maxAttempts){
  size_t numOfSamples = m_numResamples;
  vector<pair<CfgType, double> > result;
  double newConfigurationWeight, oldConfigurationWeight;
  bool firstConnectFlag, secondConnectFlag;
  LPOutput<MPTraits> lpOutput;
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
  LocalPlannerPointer lpp = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  Environment* env = this->GetMPProblem()->GetEnvironment();
  
  oldConfigurationWeight = _current.GetSmoothingValue(m_clearanceUtils, this->m_boundary);
  
  for(size_t k=0; k < _maxAttempts && numOfSamples > 0; ++k) {  
    CfgType r, c;
    r.GetRandomRay(m_stepSize, env, dm);
    c.add(r, _current); 
    newConfigurationWeight = c.GetSmoothingValue(m_clearanceUtils, this->m_boundary);
    if((newConfigurationWeight > oldConfigurationWeight && m_typeName == "MAX_CLEARANCE") ||
        (newConfigurationWeight < oldConfigurationWeight && m_typeName == "PROTEIN_ENERGY")) {
      firstConnectFlag = lpp->IsConnected(env, *stats, dm, _previous, c, &lpOutput, 
            env->GetPositionRes(), env->GetOrientationRes());
      secondConnectFlag = lpp->IsConnected(env, *stats, dm, c, _next, &lpOutput, 
            env->GetPositionRes(), env->GetOrientationRes());

      if(rdmp->GetGraph()->GetVID(c) == INVALID_VID) {
        rdmp->GetGraph()->AddVertex(c);// the vertex did not exist 
      } 
      if(firstConnectFlag && secondConnectFlag) {
        rdmp->GetGraph()->AddEdge(_previous, c, lpOutput.edge); 
        rdmp->GetGraph()->AddEdge(c, _next, lpOutput.edge);  
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

