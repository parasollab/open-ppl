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
	/*It is slow but otherwis if any function 
	   use a local 
	 polygon which is assigned to another polygon variable, the
	 descructor would free not only the local variables vertexlist 
	 but the vertexlist of the original polygon as well. so with
	 a = operator each polygon will have its own vertex list */
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




















