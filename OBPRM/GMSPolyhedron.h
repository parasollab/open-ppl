// $Id$
///////////////////////////////////////////////////////////////////////////
//  GMSPolyhedron.h
//
//  Created   3/ 6/98 Aaron Michalk
//  Last modified:
//            06/16/99 Guang Song:  add constructors and destructors.
///////////////////////////////////////////////////////////////////////////

#ifndef GMSPolyhedron_h
#define GMSPolyhedron_h

#include <fstream.h>
#include "Vectors.h"

class Polygon {
public:
    //---------------------------------------------------------------------
    //  Constructor(s) and Destructor
    //---------------------------------------------------------------------
    ~Polygon() { delete[] vertexList; } 

    //---------------------------------------------------------------------
    //  Data
    //---------------------------------------------------------------------
    int *  vertexList;
    int    numVertices;
    Vector3D normal;
    double   area;
};

class GMSPolyhedron {
public:
    //---------------------------------------------------------------------
    //  Data
    //---------------------------------------------------------------------
    Vector3D  * vertexList;
    int       numVertices;
    Polygon * polygonList;
    int       numPolygons;
    double    area;
    double    maxRadius; // the maximum distance from a vertex to com.

    //---------------------------------------------------------------------
    //  Constructor(s) and Destructor
    //---------------------------------------------------------------------
    GMSPolyhedron();
    GMSPolyhedron(GMSPolyhedron & _p); // copy constructor
    ~GMSPolyhedron();

    //---------------------------------------------------------------------
    //  Operators
    //---------------------------------------------------------------------
    GMSPolyhedron & operator=(GMSPolyhedron & _p);
    //---------------------------------------------------------------------
    //  Methods
    //---------------------------------------------------------------------
    void ComputeNormals();
    Vector3D Read(char* fileName);          // distuishes format
    Vector3D Read(istream & _is);           // read GMS format
    Vector3D ReadBYU(istream & _is);        // read BYU format
    void Write(ostream & _os);
};

#endif




















