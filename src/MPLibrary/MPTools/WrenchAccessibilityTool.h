#ifndef PMPL_WRENCH_ACCESSIBILITY_TOOL_H_
#define PMPL_WRENCH_ACCESSIBILITY_TOOL_H_

#include "MPLibrary/MPBaseObject.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidityMethod.h"
#include "Utilities/MPUtils.h"
#include <iostream>

////////////////////////////////////////////////////////////////////////////////
/// Heuristic tool for determining accessibility of a system realization for a 
/// wrench. Takes in 6DOF points (x,y,z,roll,pitch,yaw), samlpes disks around
/// these points, and returns an approximation for the largest possible wrench
/// swing. 
////////////////////////////////////////////////////////////////////////////////
class WrenchAccessibilityTool : public MPBaseObject {

  public:

    ///@name Motion Planning Types
    ///@{

      typedef typename MPBaseObject::RoadmapType        RoadmapType;
      typedef typename RoadmapType::VID                 VID;

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

    CollisionDetectionValidityMethod* m_cd; //< Collision detection

    string m_inputFile;                                //< Path to the file containing the disks
    string m_outputFile{"Disks.txt"};                               //< Path to the output file (will be created if doesnt exist)

    ofstream m_outputStream;                    //< Output stream

    ///@}

};

#endif
