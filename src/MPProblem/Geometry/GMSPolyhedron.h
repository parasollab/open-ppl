#ifndef GMSPOLYHEDRON_H_
#define GMSPOLYHEDRON_H_

#include "Vector.h"
#include "Point.h"
using namespace mathtool;

class IModel;

///////////////////////////////////////////////////////////////////////////////////////////
//  Class GMSPolygon
//
// Data structure for Polygon.
// This class Contains vertices, normal of this polygon, and size (area).
//////////////////////////////////////////////////////////////////////////////////////////

class GMSPolygon {
  public:
    GMSPolygon();
    GMSPolygon(const GMSPolygon& _p);
    ~GMSPolygon();

    bool operator==(const GMSPolygon& _p) const;

    vector<int> m_vertexList; // A list of index, which points to vertex in Polyhedron
    Vector3D m_normal; // The normal vector of this polygon 
    double m_area; // Size of this polygon
};


///////////////////////////////////////////////////////////////////////////////////////////
//  Class GMSPolyhedron
//
//  Data structure for Polyhedron.
//  This class Contains vertices, normal of this polygon, and size (area).
//////////////////////////////////////////////////////////////////////////////////////////

class GMSPolyhedron {
  public:
    GMSPolyhedron();
    GMSPolyhedron(const GMSPolyhedron& _p);
    ~GMSPolyhedron();

    bool operator==(const GMSPolyhedron& _p) const;

    /**Calculate Normal for every Polygons. (The areas is computed as well)
     *Remember: at this point, every poly must already be a tri.
     */
    void ComputeNormals();

    /**This Read distuishes format and call other read methods.
     *This version of the "Read" method will distinguish which file
     *file format body should request polyhedron to read. If format is not recognized
     *, exit will be called.
     */
    Vector3D Read(string _fileName);

    /// read GMS format and caluate maxRadius and minRadius
    Vector3D Read(istream& _is);

    /// read BYU format and caluate maxRadius and minRadius
    Vector3D ReadBYU(istream& _is);

    /// load vertices and triangles from the imodel which loads all types of models
    void LoadFromIModel(IModel* _im, Vector3D& _com); 

    /// read BYU/OBJ format and caluate maxRadius and minRadius
    /// calls model loader lib
    Vector3D ReadModel(string _fileName);

    /// Write in "original" GMS format
    void Write(ostream& _os);

    /// Write in BYU format
    void WriteBYU(ostream& _os);

    /// get a point on the surface of the polyhedron
    Point3d GetRandPtOnSurface(); 

    /// is the specified point on the surface of the polyhedron
    bool IsOnSurface(Point2d& _pt, double _h);
    double HeightAtPt(Point2d _pt, bool& _valid); 


    vector<Vector3D> m_vertexList; // 3D Vector stores vertex location info.
    vector<GMSPolygon> m_polygonList; // An array of GMSPolygon

    double m_area; //The summation of all area of polygons in this polyhedron.
    double m_maxRadius; // the maximum distance from a vertex to com.
    double m_minRadius; // the minimum distance from a vertex to com.
};

#endif
