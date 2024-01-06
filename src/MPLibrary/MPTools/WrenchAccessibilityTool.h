#ifndef PMPL_WRENCH_ACCESSIBILITY_TOOL_H_
#define PMPL_WRENCH_ACCESSIBILITY_TOOL_H_

#include "MPLibrary/MPBaseObject.h"
#include "MPLibrary/MPLibrary.h"
#include "Utilities/MPUtils.h"
#include <iostream>

////////////////////////////////////////////////////////////////////////////////
/// Heuristic tool for determining accessibility of a system realization for a 
/// wrench. Takes in 6DOF points (x,y,z,roll,pitch,yaw), samlpes disks around
/// these points, and returns an approximation for the largest possible wrench
/// swing. 
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class WrenchAccessibilityTool : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

      typedef typename MPTraits::RoadmapType            RoadmapType;
      typedef typename RoadmapType::VID                 VID;
      typedef typename MPTraits::CfgType                CfgType;

    ///@}
    ///@name Local Types
    ///@{



    ///@}
    ///@name Construction
    ///@{

    WrenchAccessibilityTool();

    WrenchAccessibilityTool(XMLNode& _node);

    virtual ~WrenchAccessibilityTool() = default;

    ///@}
    ///@name Problem Interface
    ///@{

    /// A template function
    void Initialize();

    /// Compute the wrench accessibility of the set of disks (read from file)
    /// by sampling points, performing collision detection on the points, and
    /// computing the largest free angle within the disk as a fraction of 2*pi
    /// (so in the range [0,1]).
    void ComputeWrenchAccessibility();

    ///@}
    ///@name Setters
    ///@{



    ///@}

  private:

    ///@name helpers
    ///@{
    /// Read a set of disks from a file.
    /// @param _filename The path to the file
    /// @note The file should have the following structure:
    ///       
    ///       <disk1 transformation>
    ///       <disk1 obstacle indices>
    ///       <disk2 transformation>
    ///       <disk2 obstacle indices>
    ///       <disk3 transformation>
    ///       <disk3 obstacle indices>       
    ///       ...
    ///       
    ///       transformations are 6 DOFs, first three are a point in the 
    ///       bounding box, and last three are rotational DOFs in degrees
    ///       values are single space separated ("1.2 2.1 -12.13 45 60 90")
    ///       Obstacle indices are non negative integers single space
    ///       separated ("0 4 17") 
    void ReadDisks(const string& _filename);

    /// Samples a point uniformly at random from a disk with a given radius
    /// centered around the origin (0,0) of the (x,y)-plane. Returns a cfg
    /// with 3 DOFS (0 in the last coordinate).
    /// @param _r The radius of the disk from which to sample
    /// @return A vector of the form (x,y,0) sampled u.a.r from the disk 
    Vector3d SampleInDisk(double _r);


    /// Measures the angle of between a given 2d vector in the (x,y)-plane
    /// and the x-axis.
    /// @param _c The vector (given as a cfg)
    /// @return The angle between _c and the x-axis 
    static double Angle(Vector3d& _v);


    /// Compares the angular value of two 2d vectors.
    /// @param _v1 The first vector
    /// @param _v2 The second vector
    /// @return True if _v1 < _v2 (angularly)
    static bool AngleCompare(Vector3d& _v1, Vector3d& _v2);

    /// Computes the largest free angle in a disk using a set of sample points/
    /// The points are sampled from a disk on the (x,y)-plane, and points on
    /// the actual disk are found using a transformation given as an input.
    /// The largest angle is the largest angle between two free samples that
    /// does not contain a blocked sample.  
    /// @param _v The samples from the disk on the (x,y)-plane
    /// @param _idx The index of the transformation and obstacle subset
    /// @return The maximum free angle in the disk described by _t 
    double FindLargestFreeAngle(vector<Vector3d>& _v, size_t _idx);
    

    ///@}
    ///@name internal state
    ///@{

    vector<Transformation>* m_disks;                  //< The set of disks
    vector<vector<size_t>*>* m_obstIdxs;              //< The subsets of obstacles to check for collisions

    double m_radius{1};                               //< The radius of the wrench for which
                                                      //< Accessibility is computed

    size_t m_numSamples{30};                          //< The number of samples for each disk

    CollisionDetectionValidityMethod<MPTraits>* m_cd; //< Collision detection

    string m_inputFile;                                //< Path to the file containing the disks
    string m_outputFile{"Disks.txt"};                               //< Path to the output file (will be created if doesnt exist)

    ofstream m_outputStream;                    //< Output stream

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
WrenchAccessibilityTool<MPTraits>::
WrenchAccessibilityTool() {
  this->SetName("WrenchAccessibilityTool");
}


template <typename MPTraits>
WrenchAccessibilityTool<MPTraits>::
WrenchAccessibilityTool(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  this->SetName("WrenchAccessibilityTool");

    m_radius = _node.Read("wrenchRadius", false, m_radius, 0., MAX_DBL, "The radius of the wrench");

    m_inputFile = _node.Read("inputFile", true, "", "Path to the file containing the disks");
    m_outputFile = _node.Read("outputFile", false, m_outputFile, "Path to the file containing the disks");

    m_numSamples = _node.Read("sampleNum", false, m_numSamples, size_t(1), size_t(MAX_INT), "The number of samples for each disk");

    

}


/*--------------------------------- Functions --------------------------------*/

template <typename MPTraits>
void
WrenchAccessibilityTool<MPTraits>::
Initialize(){
  m_cd = dynamic_cast<CollisionDetectionValidityMethod<MPTraits>*>(this->GetValidityChecker("pqp_solid"));

  m_disks = new vector<Transformation>();
  m_obstIdxs = new vector<vector<size_t>*>();

  if (m_inputFile != "")
    ReadDisks(m_inputFile);



  return;
}

template <typename MPTraits>
void
WrenchAccessibilityTool<MPTraits>::
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


template <typename MPTraits>
Vector3d 
WrenchAccessibilityTool<MPTraits>::
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


template <typename MPTraits>
double
WrenchAccessibilityTool<MPTraits>::
Angle(Vector3d& _v){

  double res;

  /// <v1, v2> / |v1| * |v2| = cos(theta)
  /// here v1 = (1,0) and v2 = _v
  res =  _v[0] / _v.norm();

  res = acos(res);

  res = (_v[1] > 0) ? res : 2*PI - res;

  return res;
}

template <typename MPTraits>
bool
WrenchAccessibilityTool<MPTraits>::
AngleCompare(Vector3d& _v1, Vector3d& _v2){

  return Angle(_v1) < Angle(_v2);

}


template <typename MPTraits>
double
WrenchAccessibilityTool<MPTraits>::
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



template <typename MPTraits>
void
WrenchAccessibilityTool<MPTraits>::
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
 

#endif
