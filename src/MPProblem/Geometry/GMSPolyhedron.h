#ifndef GMSPOLYHEDRON_H_
#define GMSPOLYHEDRON_H_

#include <string>
#include <iostream>
#include <vector>
using namespace std;

#include "Vector.h"
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
    Vector3d m_normal; // The normal vector of this polygon 
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
    Vector3d Read(string _fileName);

    /// read GMS format and caluate maxRadius and minRadius
    Vector3d Read(istream& _is);

    /// read BYU format and caluate maxRadius and minRadius
    Vector3d ReadBYU(istream& _is);

    /// load vertices and triangles from the imodel which loads all types of models
    void LoadFromIModel(IModel* _im, Vector3d& _com); 

    /// read BYU/OBJ format and caluate maxRadius and minRadius
    /// calls model loader lib
    Vector3d ReadModel(string _fileName);

    /// Write in "original" GMS format
    void Write(ostream& _os);

    /// Write in BYU format
    void WriteBYU(ostream& _os);

    /// get a point on the surface of the polyhedron
    Point3d GetRandPtOnSurface(); 

    vector<Vector3d>& GetVertexList() { return m_vertexList; }
    vector<GMSPolygon>& GetPolygonList() { return m_polygonList; }

    void BuildBoundary();
    void BuildBoundary2D();
    vector< pair<int,int> >& GetBoundaryLines() { 
       BuildBoundary();
       return m_boundaryLines; 
    }; 
    double GetClearance(Point3d pt, Point3d& closest, int numRays); 
    double PushToMedialAxis(Point3d& pt); 
    

    /// is the specified point on the surface of the polyhedron
    bool IsOnSurface(Point2d& _pt, double _h);
    double HeightAtPt(Point2d _pt, bool& _valid); 


    vector<Vector3d> m_vertexList; // 3D Vector stores vertex location info.
    vector<GMSPolygon> m_polygonList; // An array of GMSPolygon

    double m_area; //The summation of all area of polygons in this polyhedron.
    double m_maxRadius; // the maximum distance from a vertex to com.
    double m_minRadius; // the minimum distance from a vertex to com.
    
    ///////////////////////////////////////////////////////////////////////////
    vector< pair<int,int> > m_boundaryLines;//store the ids to vertices stored in surface model 
    bool m_boundaryBuilt;
    bool m_force2DBoundary;
};

#endif
