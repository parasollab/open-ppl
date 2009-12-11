// $Id$

/**This file defines data structures for polygon and polyhedron.
  *
  *@file GMSPolyhedron.h
  *@date 3/6/98
  *@author Aaron Michalk
  */

/////////////////////////////////////////////////////////////////////////////////////////
#ifndef GMSPolyhedron_h
#define GMSPolyhedron_h

/////////////////////////////////////////////////////////////////////////////////////////
//Include mathtool vec
#include "Vector.h"
/////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "BasicDefns.h"

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//  Class GMSPolygon
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

/** Data structure for Polygon.
  * This class Contains vertices, normal of this polygon, and size (area).
  */
class GMSPolygon {
public:
  GMSPolygon();
  GMSPolygon(const GMSPolygon& _p);
  ~GMSPolygon();

  bool operator==(const GMSPolygon& _p) const;

  vector<int> vertexList; ///< A list of index, which points to vertex in Polyhedron
  Vector3D normal; ///< The normal vector of this polygon (??)
  double area; ///< Size of this polygon
};


///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//  Class GMSPolyhedron
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

/** Data structure for Polyhedron.
  * This class Contains vertices, normal of this polygon, and size (area).
  */
class GMSPolyhedron {
public:
  GMSPolyhedron();
  GMSPolyhedron(const GMSPolyhedron & _p);
  ~GMSPolyhedron();

  bool operator==(const GMSPolyhedron& _p) const;

 /**Calculate Normal for every Polygons. (The areas is computed as well)
    *@warning this seems assumed that every polygon is a triangular
    */
  void ComputeNormals();

  /**This Read distuishes format and call other read methods.
    *This version of the "Read" method will distinguish which file
    *file format body should request polyhedron to read. If format is not recognized
    *, exit will be called.
    */
  Vector3D Read(char* fileName);

  /// read GMS format and caluate maxRadius and minRadius
  Vector3D Read(istream & _is);

  /// read BYU format and caluate maxRadius and minRadius
  Vector3D ReadBYU(istream & _is);

  /// Write in "original" GMS format
  void Write(ostream & _os);

  /// Write in BYU format
  void WriteBYU(ostream & _os);

  vector<Vector3D> vertexList; ///< 3D Vector stores vertex location info.
  vector<GMSPolygon> polygonList; ///< An array of GMSPolygon

  double area; ///<The summation of all area of polygons in this polyhedron.
  double maxRadius; /// the maximum distance from a vertex to com.
  double minRadius; /// the minimum distance from a vertex to com.
};

#endif
