// $Id$
///////////////////////////////////////////////////////////////////////////
//  GMSPolyhedron.c
//
//  Created   3/ 6/98 Aaron Michalk
///////////////////////////////////////////////////////////////////////////

#include "GMSPolyhedron.h"
#include <fstream>
#include <string.h>

using namespace std;

GMSPolygon::GMSPolygon() 
  : area(0) 
{}


GMSPolygon::GMSPolygon(const GMSPolygon& _p) 
  : vertexList(_p.vertexList), normal(_p.normal), area(_p.area) 
{}


GMSPolygon::~GMSPolygon() 
{}


bool GMSPolygon::operator==(const GMSPolygon& _p) const 
{
  return (vertexList == _p.vertexList) &&
         (normal == _p.normal) &&
         (area == _p.area);
}


GMSPolyhedron::GMSPolyhedron() 
  : area(0), maxRadius(0), minRadius(0)
{}


GMSPolyhedron::GMSPolyhedron(const GMSPolyhedron & _p) 
  : vertexList(_p.vertexList), polygonList(_p.polygonList), 
    area(_p.area), maxRadius(_p.maxRadius), minRadius(_p.minRadius)
{}


GMSPolyhedron::~GMSPolyhedron() 
{}


bool GMSPolyhedron::operator==(const GMSPolyhedron& _p) const
{
  return (vertexList == _p.vertexList) &&
         (polygonList == _p.polygonList) &&
         (area == _p.area) &&
         (maxRadius == _p.maxRadius) &&
         (minRadius == _p.minRadius);
}


//=========================================================================
//  ComputeNormals
//=========================================================================
void GMSPolyhedron::ComputeNormals() 
{
  double sum = 0;
  for(vector<GMSPolygon>::iterator P = polygonList.begin(); P != polygonList.end(); ++P)
  {
    Vector3D v1 = vertexList[P->vertexList[1]] - vertexList[P->vertexList[0]];
    Vector3D v2 = vertexList[P->vertexList[2]] - vertexList[P->vertexList[0]];
    P->normal = v1.crossProduct(v2);
    P->area = (0.5) * P->normal.magnitude();
    sum += P->area;
    P->normal = P->normal.normalize();
  }
  area = sum;
}


//=========================================================================
//  Read
//      this version of the "Read" method will distinguish which file
//      file format body should request polyhedron to read
//=========================================================================
Vector3D GMSPolyhedron::Read(char* fileName) 
{
  Vector3D com;	//Center of Mass

  //---------------------------------------------------------------
  // Get polyhedron file name and try to open the file
  //---------------------------------------------------------------
  ifstream _is(fileName);
  if (!_is) {
      cout << "Can't open \"" << fileName << "\"." << endl;
      exit(1);
  }

  //---------------------------------------------------------------
  // Read polyhedron
  //---------------------------------------------------------------
  int fileLength = strlen(fileName);
  if (!strncmp(fileName+fileLength-4,".dat",4)) 
  {
    com = Read(_is);
  } 
  else if (!strncmp(fileName+fileLength-2,".g",2)) 
  {
    com = ReadBYU(_is);
  } 
  else 
  {
    cout << "ERROR: \"" << fileName << "\" format is unrecognized.";
    cout << "\n\n       Formats are recognized by file suffixes:"
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
Vector3D GMSPolyhedron::Read(istream & _is) 
{
  Vector3D sum(0,0,0), com;

  int numVertices;
  _is >> numVertices;
  for(int i=0; i<numVertices; ++i)
  {
    Vector3D v;
    v.Read(_is);
    vertexList.push_back(v);
    sum = sum + v;
  }
  com = sum / vertexList.size();

  maxRadius = 0.0; // shift center to origin and find maximum radius.
  for(vector<Vector3D>::iterator V = vertexList.begin(); V != vertexList.end(); ++V)
  {
    *V = *V - com;
    if(V->magnitude() > maxRadius)
      maxRadius = V->magnitude();
  }

  int numPolygons;
  _is >> numPolygons;
  for(int i=0; i<numPolygons; ++i)
  {
    int numPolyVertices;
    _is >> numPolyVertices;
    GMSPolygon p;
    p.vertexList = vector<int>(numPolyVertices, -1);
    for(int j=0; j<numPolyVertices; ++j) 
      _is >> p.vertexList[j];
    polygonList.push_back(p);
  }

  ComputeNormals();
  return com;
}


//=========================================================================
//  Read
//      reads BYU format
//=========================================================================
Vector3D GMSPolyhedron::ReadBYU(istream & _is) 
{
    int nParts, numVertices, numPolygons, nEdges;
    _is >> nParts;              // throwaway for now
    _is >> numVertices;
    _is >> numPolygons;
    _is >> nEdges;              // throwaway for now

    int startPartPolys, nPartPolys;
    _is >> startPartPolys;      // throwaway for now
    _is >> nPartPolys;          // throwaway for now

    Vector3D sum(0,0,0), com;
    for(int i=0; i<numVertices; ++i)
    {
      Vector3D v;
      v.Read(_is);
      vertexList.push_back(v);
      sum = sum + v;
    }
    com = sum / vertexList.size();

    maxRadius = 0.0; // shift center to origin and find maximum radius.
    for(vector<Vector3D>::iterator V = vertexList.begin(); V != vertexList.end(); ++V)
    {
      *V = *V - com;
      if(V->magnitude() > maxRadius)
        maxRadius = V->magnitude();
    }
    com = Vector3D(0.0, 0.0, 0.0);

    for(int i=0; i<numPolygons; ++i)
    {
      GMSPolygon p;
      do 
      {
        int tmp;
        _is >> tmp;
        p.vertexList.push_back(tmp);
      } while(p.vertexList.back() > 0);
      p.vertexList.back() *= -1; //last one is negative, so change sign

      for(vector<int>::iterator I = p.vertexList.begin(); I != p.vertexList.end(); ++I)
        *I = *I-1; //BYU starts numbering from 1 instead of 0, so decrement by 1

      polygonList.push_back(p);
    }

    ComputeNormals();
    return com;
}


//=========================================================================
//  Write
//=========================================================================
void GMSPolyhedron::Write(ostream & _os) 
{
    _os << vertexList.size() << " " << endl;
    for(vector<Vector3D>::const_iterator V = vertexList.begin(); V != vertexList.end(); ++V)
    {
      V->Write(_os);
      _os << endl;
    }
    _os << polygonList.size() << " " << endl;
    for(vector<GMSPolygon>::const_iterator P = polygonList.begin(); P != polygonList.end(); ++P)
    {
       _os << P->vertexList.size() << " ";
       for(vector<int>::const_iterator I = P->vertexList.begin(); I != P->vertexList.end(); ++P)
         _os << *I << " ";
       _os << endl;
    }
    _os << endl;
}


//=========================================================================
//  WriteBYU
//=========================================================================
void GMSPolyhedron::WriteBYU(ostream & _os) 
{
  _os << "1 " << vertexList.size() << " " << polygonList.size() << " 1 1 1\n";
  for(vector<Vector3D>::const_iterator V = vertexList.begin(); V != vertexList.end(); ++V)
  {
    V->Write(_os);
    _os << endl;
  }
  for(vector<GMSPolygon>::const_iterator P = polygonList.begin(); P != polygonList.end(); ++P)
  {
    for(vector<int>::const_iterator I = P->vertexList.begin(); (I+1) != P->vertexList.end(); ++I)
      _os << *I+1 << " ";
    _os << "-" << P->vertexList.back()+1 << endl;
  }
  _os << endl;
}

