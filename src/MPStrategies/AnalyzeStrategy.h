#ifndef AnalyzeStrategy_h
#define AnalyzeStrategy_h

#include "MPStrategyMethod.h"
#include "Environment/Body.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup FoldingStrategies
/// @brief Analyze approach
///
/// Analyze finds the energy and distances between each sampled node and a
/// protein and displays them in the format: vid com_dist energy/
///
/// \internal This strategy was modified for the binding pockets summer project
////////////////////////////////////////////////////////////////////////////////

template<class MPTraits>
class AnalyzeStrategy : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType              MPProblemType;
    typedef typename MPTraits::CfgType                    CfgType;
    typedef typename MPProblemType::VID                   VID;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    AnalyzeStrategy();
    AnalyzeStrategy(MPProblemType* _problem, XMLNode& _node);
    virtual ~AnalyzeStrategy() {}

    virtual void ParseXML(XMLNode& _node);
    virtual void PrintOptions(ostream& _os);

    virtual void Initialize(){}
    virtual void Run();
    virtual void Finalize();

  protected:
    string m_inputRoadmapLabel;
    string m_inputPositionLabel;  // input file for protein atoms
    double m_x;                   //top x% of node to report
    string m_outputLabel;         //output file name
    string m_vcLabel;            //to check validity with the convex hull
};

template<class MPTraits>
AnalyzeStrategy<MPTraits>::
AnalyzeStrategy() {
  this->m_name = "AnalyzeStrategy";
}

template<class MPTraits>
AnalyzeStrategy<MPTraits>::
AnalyzeStrategy(MPProblemType* _problem, XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node) {
    this->m_name = "AnalyzeStrategy";
    ParseXML(_node);
  }


template<class MPTraits>
void
AnalyzeStrategy<MPTraits>::
ParseXML(XMLNode& _node) {
  m_inputRoadmapLabel = _node.Read("inputMap", false, "", "filename of the input ligand map");
  m_inputPositionLabel = _node.Read("inputPosition", false, "", "filename of the protein position");
  m_x = _node.Read("x", true, 0.0, 0.0, MAX_DBL, "percentage of the nodes for calculating average");
  m_outputLabel = _node.Read("output", false, "", "output filename");
  m_vcLabel = _node.Read("vcLabel", false, "cd4", "validity checker");
}

template<class MPTraits>
void
AnalyzeStrategy<MPTraits>::
PrintOptions(ostream& _os) {
  _os << "AnalyzeStrategy :: " << endl;
  _os << "input map name = " << m_inputRoadmapLabel << endl;
  _os << "input position name = " << m_inputPositionLabel << endl;
  _os << "x% = " << m_x << endl;
  _os << "output filename = " << m_outputLabel << endl;
  _os << "vc label = " << m_vcLabel << endl;
}

template<class MPTraits>
void
AnalyzeStrategy<MPTraits>::
Run() {
  PrintOptions(cout);

  vector<CfgType> outNodes;  //vector to store the ligand samples

  this->GetMPProblem()->GetRoadmap()->Read(m_inputRoadmapLabel.c_str());

  for(size_t i=0; i<this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_vertices(); i++) {
    outNodes.push_back(this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVertex(i));
  }

  //compute distance between com(ligand) and com(protein)
  vector<pair<size_t, double> > dist;  //store id and distance
  Vector3d obst;

  //get center of mass of the protein
  Environment* env = this->GetMPProblem()->GetEnvironment();
  for(size_t i=0; i < env->NumObstacles(); i++) {
    obst = env->GetObstacle(i)->GetCenterOfMass();
  }

  //get protein composition info
  ifstream fin;
  fin.open(m_inputPositionLabel.c_str(), std::ifstream::in);
  char buffer[300];
  double x, y, z;
  vector<Vector3d> proteinAtoms;  //store the protein information

  while(!fin.eof()) {
    fin.getline(buffer, 500);
    stringstream str(buffer);
    str >> x >> y >> z;
    proteinAtoms.push_back(Vector3d(x, y, z));  //read in protein
  }

  // Check collision with the convex hull of the protein
  // build the convex hull of the protein
  shared_ptr<Body> obstacle(this->GetEnvironment()->GetObstacle(0)->GetFixedBody(0));
  obstacle->m_buildConvex = true;
  ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);
  string sampler = "UniformObstacleBasedSampler::SampleImpl()";

  //output metrics
  ofstream fout;
  fout.open(m_outputLabel.c_str());

  fout<<"VID" << "\t" << "com distance "<< "\t" << "Energy" << "\t" << "convex col." << endl;
  fout<<"======================================================================="<< endl << endl;

  size_t id = 0;
  for(typename vector<CfgType>::iterator I = outNodes.begin(); I != outNodes.end(); I++) {
    //get distance to the center of mass
    Vector3d ligand = I->GetRobotCenterPosition();  //com(ligand)
    double distance_i = (ligand - obst).norm();
    dist.push_back(pair<size_t, double>(id, distance_i));

    //check collision with convex hull
    bool convexCollision = !(vc->IsValid(*I, sampler)) ||
        (vc->IsInsideObstacle(*I));
    vector<double> energy_i;
    int n = 0;
    double totalE_i = 0;
    for(auto& atom : proteinAtoms) {
      switch(n%3) {
        case 0:  //N
          energy_i.push_back((3952580/(pow((ligand - atom).norm(), 12))) - (2556/(pow((ligand - atom).norm(), 6))));
          break;
        case 1:  //Ca
          energy_i.push_back((3075695/(pow((ligand - atom).norm(), 12))) - (953/(pow((ligand - atom).norm(), 6))));
          break;
        case 2:  //C
          energy_i.push_back((1200965/(pow((ligand - atom).norm(), 12))) - (425/(pow((ligand - atom).norm(), 6))));
          break;
      }
      n++;
      totalE_i += energy_i.back();
    }
    fout<< id << "\t" << distance_i << "\t" << totalE_i << "\t" << convexCollision << endl;
    id++;
  }


  sort(dist.begin(), dist.end());
  double tempD = 0.0;
  for(size_t i=0; i<(dist.size()*m_x/100); i++) {
    tempD += dist[i].second;
  }
  fout << "top " << m_x << "% has " << dist.size()*m_x/100 << " nodes." << endl;
  fout << "top " << m_x << "% distance average: " <<  tempD/((dist.size())*m_x/100) << endl;

  fout << "min distance: " << dist[0].second << endl;

  //find the minimum among all the distances in dist
  double minD = dist[0].second;
  for(vector<pair<size_t, double> >::iterator I = dist.begin(); I != dist.end(); I++) {
    if(I->second < minD)
      minD = I->second;
  }
  cout << "min distance: " << minD << endl;
}

template<class MPTraits>
void
AnalyzeStrategy<MPTraits>::Finalize() {
  string str;
  str = this->GetBaseFilename() + ".map";
  this->GetMPProblem()->GetRoadmap()->Write(str, this->GetMPProblem()->GetEnvironment());
}
#endif
