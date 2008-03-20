// $Id$
///////////////////////////////////////////////////////////////////////////
//  GMSPolyhedron.c
//
//  Created   3/ 6/98 Aaron Michalk
///////////////////////////////////////////////////////////////////////////

#include "GMSPolyhedron.h"
using namespace std;

#include <fstream>
#include <istream>

//=========================================================================
//  Class GMSPolyhedron
//=========================================================================
//---------------------------------------------------------------------
//  Constructor(s) and Destructor
//---------------------------------------------------------------------
GMSPolyhedron::GMSPolyhedron() {
	vertexList = NULL;
	polygonList = NULL;
	numVertices = 0;
	numPolygons = 0;
	area = 0.0;
	maxRadius = 0.0;
	minRadius = 0.0;
}

GMSPolyhedron::GMSPolyhedron(GMSPolyhedron & _p) {

	//Copy vertex size
    numVertices = _p.numVertices;

	//Copy vertices
    vertexList = new Vector3D[numVertices];
    int i;
    for (i = 0; i < numVertices; i++) {
        vertexList[i] = _p.vertexList[i];
    }

	//Copy polygon size
    numPolygons = _p.numPolygons;

	//Copy polygons (why not using polygon's operator?)
    polygonList = new GMSPolygon[numPolygons];
    for (i = 0; i < numPolygons; i++) {
        polygonList[i].numVertices = _p.polygonList[i].numVertices;
        polygonList[i].vertexList = new int[polygonList[i].numVertices];
        for (int j = 0; j < polygonList[i].numVertices; j++) {
            polygonList[i].vertexList[j] = _p.polygonList[i].vertexList[j];
        }
        polygonList[i].normal = _p.polygonList[i].normal;
    }

	//copy other info
    area = _p.area;
    maxRadius = _p.maxRadius;
    minRadius = _p.minRadius;
}

GMSPolyhedron::~GMSPolyhedron() {
	delete[] vertexList;
	delete[] polygonList;
}



//=========================================================================
//  Operators
//=========================================================================
GMSPolyhedron & GMSPolyhedron::operator=(GMSPolyhedron & _p) {
  
  int i,j,aptal;
  int *trick;

  if(this != &_p) // protect against self assignment. p = p
  {
    delete[] vertexList;
    delete[] polygonList;

	//Copy vertex size
    numVertices = _p.numVertices;

	//Copy vertices
    vertexList = new Vector3D[numVertices];
    for (i = 0; i < numVertices; i++) {
        vertexList[i] = _p.vertexList[i];
    }

	//Copy polygon size
    numPolygons = _p.numPolygons;

	//Copy polygons
    polygonList = new GMSPolygon[numPolygons];
    for (i = 0; i < numPolygons; i++) 
	{
		polygonList[i].numVertices = _p.polygonList[i].numVertices;
        aptal=polygonList[i].numVertices;
        trick=new int[aptal];
        if(trick==NULL) 
		{
			printf("Not enough memory\n");
            exit(5);
        }

		polygonList[i].vertexList = trick;
		for (j = 0; j < polygonList[i].numVertices; j++) 
		{
		    polygonList[i].vertexList[j] = _p.polygonList[i].vertexList[j];
		}
		polygonList[i].normal = _p.polygonList[i].normal;
	}

	//Copy other info
    area = _p.area;
    maxRadius = _p.maxRadius;
    minRadius = _p.minRadius;
  }

  aptal++; //<--What is this??
  return *this;
}

//=========================================================================
//  ComputeNormals
//=========================================================================
void GMSPolyhedron::ComputeNormals() {
    int i;
    GMSPolygon * p;
    Vector3D v1, v2;

    double sum = 0;
    for (i=0; i < numPolygons; i++)
	{
      p = &(polygonList[i]);
      v1 = vertexList[p->vertexList[1]] - vertexList[p->vertexList[0]];
      v2 = vertexList[p->vertexList[2]] - vertexList[p->vertexList[0]];
      p->normal=v1.crossProduct(v2);
      polygonList[i].area = (0.5) * p->normal.magnitude();
      sum += polygonList[i].area;
      p->normal.normalize();
    }

    this->area = sum;
}



//=========================================================================
//  Read
//      this version of the "Read" method will distinguish which file
//      file format body should request polyhedron to read
//=========================================================================
Vector3D GMSPolyhedron::Read(char* fileName) {

    Vector3D com;	//Center of Mass

    //---------------------------------------------------------------
    // Get polyhedron file name and try to open the file
    //---------------------------------------------------------------
   std::ifstream _is(fileName);

    if (!_is) {
      std:: cout << "Can't open \"" << fileName << "\"." << endl;
        exit(1);
    }

    //---------------------------------------------------------------
    // Read polyhedron
    //---------------------------------------------------------------

    int fileLength = strlen(fileName);

    if (!strncmp(fileName+fileLength-4,".dat",4)) 
	{
        com = Read(_is);
    } else if (!strncmp(fileName+fileLength-2,".g",2)) 
	{
        com = ReadBYU(_is);
    } else 
	{
        cout<<"ERROR: \""<<fileName<<"\" format is unrecognized.";
        cout<<"\n\n       Formats are recognized by file suffixes:"
                        "\n\t    GMS(*.dat)"
                        "\n\t    BYU(*.g)";
    }

    //---------------------------------------------------------------
    // Close file
    //---------------------------------------------------------------
    _is.close();
    return com;
}

//=========================================================================
//  Read
//      reads "original" GMS format
//=========================================================================
Vector3D GMSPolyhedron::Read(istream & _is) {

    Vector3D sum(0,0,0), com;

    int i, j;
    _is >> numVertices;
    vertexList = new Vector3D[numVertices];
    for (i = 0; i < numVertices; i++) {
        vertexList[i].Read(_is);
	sum = sum + vertexList[i];
    }
    com = sum/numVertices;

    maxRadius = 0.0; // shift center to origin and find maximum radius.
    for (i = 0; i < numVertices; i++) {
  	vertexList[i] = vertexList[i] - com;
        if(vertexList[i].magnitude() > maxRadius) 
	  maxRadius = vertexList[i].magnitude();
    }

    _is >> numPolygons;
    polygonList = new GMSPolygon[numPolygons];
    for (i = 0; i < numPolygons; i++) {
        _is >> polygonList[i].numVertices;
	polygonList[i].vertexList = new int[polygonList[i].numVertices];
	for (j = 0; j < polygonList[i].numVertices; j++) {
	    _is >> polygonList[i].vertexList[j];
	}
    }
    ComputeNormals();
    return com;
}

//=========================================================================
//  Read
//      reads BYU format
//=========================================================================
Vector3D GMSPolyhedron::ReadBYU(istream & _is) {

    Vector3D sum(0,0,0), com;

//    int nVerts,nPolys;
    int nParts, nEdges;
    _is >> nParts;              // throwaway for now
    _is >> numVertices;
    _is >> numPolygons;
    _is >> nEdges;              // throwaway for now

    int startPartPolys,nPartPolys;
    _is >> startPartPolys;      // throwaway for now
    _is >> nPartPolys;          // throwaway for now

    int i, j, cnt, tmp;
    vertexList = new Vector3D[numVertices];
    for (i = 0; i < numVertices; i++) {
        vertexList[i].Read(_is);
	sum = sum + vertexList[i];
    }
    com = sum/numVertices;

    maxRadius = 0.0; // shift center to origin and find maximum radius.
    for (i = 0; i < numVertices; i++) {
        vertexList[i] = vertexList[i] - com;
        if(vertexList[i].magnitude() > maxRadius) 
	    maxRadius = vertexList[i].magnitude();
    }
    com = Vector3D(0.0, 0.0, 0.0);

    

    polygonList = new GMSPolygon[numPolygons];
    for (i = 0; i < numPolygons; i++) {

        cnt =0;                                 // don't know count
        int indices[20];                        // ahead of time so need
        do{                                     // to count them up
                _is >> tmp;
                indices[cnt++]=tmp;
        }while (indices[cnt-1]>0);
        indices[cnt-1]= - indices[cnt-1];       // last one is negative
                                                // so change the sign

        polygonList[i].numVertices = cnt;
        polygonList[i].vertexList = new int[polygonList[i].numVertices];
        for (j = 0; j < polygonList[i].numVertices; j++) {
                                        // BYU starts numbering from
                                        // 1 instead of 0 so decr by 1
            polygonList[i].vertexList[j] = indices[j]-1;
        }

    }

    ComputeNormals();
    return com;
}

//=========================================================================
//  Write
//=========================================================================
void GMSPolyhedron::Write(ostream & _os) {
    int i, j;
    _os << numVertices << " " << endl;
    for (i = 0; i < numVertices; i++) {
        vertexList[i].Write(_os);
        _os <<endl;
    }
    _os << numPolygons << " " << endl;
    for (i = 0; i < numPolygons; i++) {
        _os << polygonList[i].numVertices << " ";
        for (j = 0; j < polygonList[i].numVertices; j++) {
            _os << polygonList[i].vertexList[j] << " ";
        }
        _os<<endl;
    }
    _os<<endl;
}

//=========================================================================
//  WriteBYU
//=========================================================================
void GMSPolyhedron::WriteBYU(ostream & _os) {
  int i;
  _os << "1 " << numVertices << " " << numPolygons << " 1 1 1" << endl;
  for(i=0; i<numVertices; i++) {
    vertexList[i].Write(_os);
    _os << endl;
  }
  for(i=0; i<numPolygons; i++) {
    for(int j=0; j<polygonList[i].numVertices; j++) {
      if(j == polygonList[i].numVertices-1) { //last one
	_os << "-" << polygonList[i].vertexList[j]+1 << endl;
      } else {
	_os << polygonList[i].vertexList[j]+1 << " ";
      }
    }
  }
  _os << endl;
}

/***********************************************************************************
 *
 *
 *
 *
 *	Implementation of GMSPolygon methods
 *
 *
 *
 *
 ***********************************************************************************/
GMSPolygon & GMSPolygon::operator=(GMSPolygon  _p) {
    
	//Check if these two are the same
	if(this == &_p) return *this;

	//Copy vertex nunmber
	numVertices=_p.numVertices;

	//Copy vertex
	vertexList=new int[numVertices];
	if( vertexList==NULL) 
	{ 
		fprintf(stderr,"Can't allocate memory for GMSPolygon\n"); 
		exit(4);
	}

	for(int i=0;i<numVertices;i++)
	{
		vertexList[i]=_p.vertexList[i];
	}

	//copy normal
	normal=_p.normal;

	//copy area
	area=area;	//<--- This is wierd

	return *this;
}

GMSPolygon    GMSPolygon::getCopy(){

	GMSPolygon *cp;
	cp=new GMSPolygon;
	cp->numVertices=numVertices;
	cp->vertexList=new int[numVertices];
	if( cp->vertexList==NULL) { fprintf(stderr,"Can't allocate memory for GMSPolygon\n"); exit(4);}
	cp->normal=normal;
	cp->area=area;
	for(int i=0;i<numVertices;i++)
	{
		cp->vertexList[i]=vertexList[i];
	}

   return *cp;
}
