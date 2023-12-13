#include "WrenchAccessibilityTool.h"

#include "MPLibrary/MPLibrary.h"

/*------------------------------- Construction -------------------------------*/

WrenchAccessibilityTool::
WrenchAccessibilityTool() {
  this->SetName("WrenchAccessibilityTool");
}


WrenchAccessibilityTool::
WrenchAccessibilityTool(XMLNode& _node) : MPBaseObject(_node) {
  this->SetName("WrenchAccessibilityTool");

    m_radius = _node.Read("wrenchRadius", false, m_radius, 0., MAX_DBL, "The radius of the wrench");

    m_inputFile = _node.Read("inputFile", true, "", "Path to the file containing the disks");
    m_outputFile = _node.Read("outputFile", false, m_outputFile, "Path to the file containing the disks");

    m_numSamples = _node.Read("sampleNum", false, m_numSamples, size_t(1), size_t(MAX_INT), "The number of samples for each disk");
}

/*--------------------------------- Functions --------------------------------*/

void
WrenchAccessibilityTool::
Initialize(){
  m_cd = dynamic_cast<CollisionDetectionValidityMethod*>(this->GetMPLibrary()->GetValidityChecker("pqp_solid"));

  m_disks = new vector<Transformation>();
  m_obstIdxs = new vector<vector<size_t>*>();

  if (m_inputFile != "")
    ReadDisks(m_inputFile);

  return;
}


void
WrenchAccessibilityTool::
ComputeWrenchAccessibility(){
  m_outputStream.open(m_outputFile);
  
  auto start = std::chrono::high_resolution_clock::now();

  Vector3d s;

  vector<Vector3d> samples;

  vector<double> results;

  double res;

  for (size_t i = 0; i < m_disks->size(); i++) {

    samples.clear();

    for (size_t iter = 0; iter < m_numSamples; iter ++) {
      
      s = SampleInDisk(m_radius);

      samples.push_back(s);

    }

    sort(samples.begin(), samples.end(), AngleCompare);

    res = FindLargestFreeAngle(samples, i);

    results.push_back(res);

  }
  if (this->m_debug)
    cout << "the results of wrench accessibility are: "<< results << endl;
  
  m_outputStream.close();
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  if (this->m_debug)
    cout << endl << duration.count() << " micro-seconds" << endl << endl;
  return;
}


Vector3d 
WrenchAccessibilityTool::
SampleInDisk(double _r){

  Vector3d res;

  double x = GRand();
  double y = GRand();

  double normaliztionFactor = 1. / pow(x*x + y*y, 0.5);
  
  double r = _r * pow(DRand(), 0.5);

  res[0] = r * x * normaliztionFactor;
  res[1] = r * y * normaliztionFactor;
  res[2] = 0;
  
  return res;
}


double
WrenchAccessibilityTool::
Angle(Vector3d& _v){

  double res;

  /// <v1, v2> / |v1| * |v2| = cos(theta)
  /// here v1 = (1,0) and v2 = _v
  res =  _v[0] / _v.norm();

  res = acos(res);

  res = (_v[1] > 0) ? res : 2*PI - res;

  return res;
}


bool
WrenchAccessibilityTool::
AngleCompare(Vector3d& _v1, Vector3d& _v2){

  return Angle(_v1) < Angle(_v2);
}


double
WrenchAccessibilityTool::
FindLargestFreeAngle(vector<Vector3d>& _v, size_t _idx){

  double res = 0;

  double currAngle;

  double currFreeAngle = 0;

  double startFreeAngle = 0;

  double freeSequenceStartAngle = 0;

  bool blocked;

  bool foundFirstBlocked = false;

  bool onFreeSequence = false;

  Vector3d v;

  for (size_t s = 0; s < _v.size(); s++) {

    v = m_disks->at(_idx) * _v[s];

    blocked =  m_cd->IsInsideObstacle(v, m_obstIdxs->at(_idx));

    for (int i = 0; i < 3; i++) {
      m_outputStream << v[i] << " ";
    }

    blocked ? m_outputStream << "0" : m_outputStream << "1";
    m_outputStream << std::endl;

    currAngle = Angle(_v[s]);

    if (blocked) {
      if (!foundFirstBlocked) 
        foundFirstBlocked = true;
    }
    else {
      /// Advancing the starting free angle for later cyclic check
      if (!foundFirstBlocked)
        startFreeAngle = currAngle;
      
      if (onFreeSequence) {

        currFreeAngle = currAngle - freeSequenceStartAngle;

        if (currFreeAngle > res) 
          res = currFreeAngle;
      }
      /// Starting a free sequence
      else
        freeSequenceStartAngle = currAngle;
    }

    onFreeSequence = !blocked;
  }

  /// If we end on a free sequence try adding the start free sequence
  /// since this is a cyclic set of points.
  if (onFreeSequence) {

    currFreeAngle = (2 * PI - freeSequenceStartAngle) + startFreeAngle;

    if (currFreeAngle > res) 
          res = currFreeAngle;
  }

  /// If there are no blocked points
  if (!foundFirstBlocked)
    res = 2 * PI;

  /// Normalize to a value in [0,1];
  return res / (2 * PI);
}


void
WrenchAccessibilityTool::
ReadDisks(const string& _filename) {

  ifstream file(_filename);

  if (file.is_open()) {
    string line;
    while (std::getline(file, line)) {

      /// Reading the transformation
      Transformation transformation;

      std::istringstream buffer(line);

      buffer >> transformation;

      m_disks->push_back(transformation);

      /// Reading the obstacle subset
      std::getline(file, line);

      buffer= std::istringstream(line);

      vector<size_t>* obstIdxs = new vector<size_t>();
      size_t idx;
      
      while (buffer >> idx) {
        obstIdxs->push_back(idx);
      }

      m_obstIdxs->push_back(obstIdxs);

    }
  }
}
