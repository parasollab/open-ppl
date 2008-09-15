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
//Include OBPRM headers
#include "Vectors.h"


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

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------
    /**@name Constructors and Destructor.*/
    //-----------------------------------------------------------
    //@{

    ///Destructor. Free memory for storing vertex.
    ~GMSPolygon() { delete[] vertexList; }

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Operator Overloadings
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    //-----------------------------------------------------------
    /**@name Operator Overloadings.*/
    //-----------------------------------------------------------
    //@{

     /**Copy _p's values to this instance.
       *It is slow but otherwis if any function use a local
       *polygon which is assigned to another polygon variable, the
       *descructor would free not only the local variables vertexlist
       *but the vertexlist of the original polygon as well. so with
       *@param a operator each polygon will have its own vertex list
       */
     GMSPolygon & operator=(GMSPolygon  _p);

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Helper Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    //-----------------------------------------------------------
    /**@name Helper Methods.*/
    //-----------------------------------------------------------
    //@{

     ///Return a copy of this instance. Clone!!
     GMSPolygon getCopy();

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data Members
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    int * vertexList;///< Vector list. A list of index, which points to vetex in Polyhedron
    int numVertices; ///< Number of vectex in this polygon
    Vector3D normal; ///< The normal vector of this polygon (??)
    double   area;   ///< Size of this polygon
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

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------
    /**@name Constructors and Destructor.*/
    //-----------------------------------------------------------
    //@{

    ///Init every thing to zero and NULL
    GMSPolyhedron();

    ///Copy constructor
    GMSPolyhedron(const GMSPolyhedron & _p);

    ///Destructor. delete vertex and polygon array.
    ~GMSPolyhedron();

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Operator Overloadings
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    //-----------------------------------------------------------
    /**@name Operator Overloadings.*/
    //-----------------------------------------------------------
    //@{

    ///Copy values in _p to this instance and return this instance as a reference
    GMSPolyhedron & operator=(GMSPolyhedron & _p);

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Helper Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    //-----------------------------------------------------------
    /**@name Helper Methods.*/
    //-----------------------------------------------------------
    //@{

    /**Calculate Normal for every Polygons. (The areas is computed as well)
      *@warning this seems assumed that every polygon is a triangular
      */
    void ComputeNormals();

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    //-----------------------------------------------------------
    /**@name I/O Methods.*/
    //-----------------------------------------------------------
    //@{
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
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data Members
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    Vector3D  * vertexList; ///< 3D Vector stores vertex location info.
    int       numVertices;  ///< Number of vertex in this polyhedron
    GMSPolygon * polygonList; ///< An array of GMSPolygon
    int       numPolygons;    ///< Number of polygon in this polyhedron

    double    area; ///<The summation of all area of polygons in this polyhedron.
    /// the maximum distance from a vertex to com.
    double    maxRadius;
    /// the minimum distance from a vertex to com.
    double minRadius;
};

#endif
