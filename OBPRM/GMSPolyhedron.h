// $Id$

/**@file GMSPolyhedron.h
   @date 3/6/98
   @author Aaron Michalk
*/

#ifndef GMSPolyhedron_h
#define GMSPolyhedron_h

#include <fstream.h>
#include "Vectors.h"

class GMSPolygon {
public:
    //---------------------------------------------------------------------
    //  Constructor(s) and Destructor
    //---------------------------------------------------------------------
    ~GMSPolygon() { delete[] vertexList; }

    //---------------------------------------------------------------------
    //  Data
    //---------------------------------------------------------------------
    int *  vertexList;
    int    numVertices;
    Vector3D normal;
    double   area;

     /**It is slow but otherwis if any function use a local
     polygon which is assigned to another polygon variable, the
     descructor would free not only the local variables vertexlist
     but the vertexlist of the original polygon as well. so with
     a = operator each polygon will have its own vertex list
     */
     GMSPolygon    &operator=(GMSPolygon  _p);
     GMSPolygon getCopy();
};

class GMSPolyhedron {
public:
    //---------------------------------------------------------------------
    //  Data
    //---------------------------------------------------------------------
    Vector3D  * vertexList;
    int       numVertices;
    GMSPolygon * polygonList;
    int       numPolygons;
    double    area;
    /// the maximum distance from a vertex to com.
    double    maxRadius;

    //---------------------------------------------------------------------
    //  Constructor(s) and Destructor
    //---------------------------------------------------------------------
    GMSPolyhedron();
    /// copy constructor
    GMSPolyhedron(GMSPolyhedron & _p);
    ~GMSPolyhedron();

    //---------------------------------------------------------------------
    //  Operators
    //---------------------------------------------------------------------
    GMSPolyhedron & operator=(GMSPolyhedron & _p);
    //---------------------------------------------------------------------
    //  Methods
    //---------------------------------------------------------------------
    void ComputeNormals();
    /// distuishes format
    Vector3D Read(char* fileName);
    /// read GMS format
    Vector3D Read(istream & _is);
    /// read BYU format
    Vector3D ReadBYU(istream & _is);
    void Write(ostream & _os);
};

#endif
