// dist.cpp : Defines the entry point for the console application.
// [exe file] [env file]  [the output file for parsemap.cpp]  [output min distance file]

/********************** env file format *************************
 *
 *  [obstacle .g file] [x coordinate of the obstacle] [y] [z]
 *
 ***************************************************************/

#include <fstream>
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <limits>
#include <string>
#include <sstream>
//#include "stdafx.h"
#define MAX_DIST  1e35
using namespace std;
struct vertex
{
  int id;
  double pos[3];
};

struct face
{
  int id;
  vertex* v[3];
  double normal[3];
  int x,y;
  int vertex_contains;
  double distribution_error;
  double ratio;
  face()
  {
    vertex_contains = 0;
  }
  void compute_normal()
  {
    double a[3], b[3];
    for(int i =0;i<3;i++)
    {
      a[i] = v[1]->pos[i] - v[0]->pos[i];
      b[i] = v[2]->pos[i] - v[0]->pos[i];
    }
    double mag = 0;
    for(int i =0;i<3;i++){
      normal[i] = (a[(i+1)%3]*b[(i+2)%3])- (a[(i+2)%3]*b[(i+1)%3]);
      mag += (normal[i]*normal[i]);
    }
    mag = sqrt(mag);
    if(mag > 0.0f)
    {
      for(int i =0;i<3;i++)
        normal[i] /= (double)mag;
    }
    int longest;
    if(fabs(normal[0]) > fabs(normal[1])) longest =0;
    else
      longest = 1;
    if(fabs(normal[2]) > fabs(normal[longest]))
      longest = 2;
    x=(longest+1)%3;
    y=(longest+2)%3;

  }
  double area()
  {
    double surface_area=0.0;
    for(int i =0;i<3; i++){
      double temp =0.0;
      for(int j=0;j<3;j++)
        temp += (v[j]->pos[i]*(v[(j+1)%3]->pos[(i+1)%3]- v[(j+2)%3]->pos[(i+1)%3]));
      surface_area += pow(temp,2);
    }
    /*double a[3], b[3], c[3];
      for(int i=0; i<3; i++)
      {
      a[i] = v[1]->pos[i] - v[0]->pos[i];
      b[i] = v[0]->pos[i] - v[2]->pos[i];
      }
      for(int i=0; i<3; i++)
      c[i] = (a[(i+1)%3]*b[(i+2)%3]) - (a[(i+2)%3]*b[(i+1)%3]);

      for(int i=0; i<3; i++)
      surface_area += (c[i]*c[i]);*/

    surface_area = sqrt(surface_area);
    surface_area = 0.5*surface_area;
    return surface_area;
  }
  double distance(vertex *ver)
  {
    double dist=0;
    for(int i =0;i<3;i++)
    {
      dist += ((ver->pos[i] - v[0]->pos[i]) * normal[i]);
    }
    if(dist<0.0f) return MAX_DIST;
    return fabs(dist);

  }
  //Check this part- for the edges
  int inside_outside(vertex *p)
  {
    int i, j, c = 0;
    for (i = 0, j = 2; i < 3; j = i++) {
      if(fabs(v[i]->pos[y]-p->pos[y])<0 && fabs(v[i]->pos[x]-p->pos[x])<0) return 1;
      if ( ((v[i]->pos[y]>p->pos[y]) != (v[j]->pos[y]>p->pos[y])) &&
          (p->pos[x] <= (v[j]->pos[x]-v[i]->pos[x]) * (p->pos[y]-v[i]->pos[y]) / (v[j]->pos[y]-v[i]->pos[y]) + v[i]->pos[x]) )
        c = !c;
      else if((v[i]->pos[y]==p->pos[y])&& (v[j]->pos[y]== p->pos[y])
          && ((v[j]->pos[x]- v[i]->pos[x]) == (v[j]->pos[x]-p->pos[x])+(p->pos[x]-v[i]->pos[x])))
        c = !c;
      else if((v[i]->pos[x]==p->pos[x])&& (v[j]->pos[x]== p->pos[x])
          && ((v[j]->pos[y]- v[i]->pos[y]) == (v[j]->pos[y]-p->pos[y])+(p->pos[y]-v[i]->pos[y])))
        c = !c;
    }
    return c;

  }
};

struct compare {
  bool operator()(face* a, face* b)
  {
    return a->area() < b->area();
  }
} areacompare;
double distance(vertex* node1, vertex* node2)
{
  double dist = 0;
  for(int i =0; i<3;i++)
    dist += pow((node1->pos[i]-node2->pos[i]),2);
  dist = sqrt(dist);
  return dist;
}
int main(int argc, char* argv[])
{
  for (int index = 2; index < argc; ++index) {
    ifstream fin(argv[1], ios::in);  //Read in object env file
    ifstream fintxt(argv[index],ios::in);  //Read in map file containing x, y, z info for each vertex
    vector<vertex*> model_vertices;
    vector<face*> model_faces;

    ofstream fouth((string(argv[index]) + ".out").c_str(), ios::out);  //Output file for minimum distance
    if( !fin.good() )
    {	//Not good. File not found
      cerr<<"env File  not found"<<endl;
      return false;
    }
    if( !fintxt.good() )
    {	//Not good. File not found
      cerr<<"map File  not found"<<endl;
      return false;
    }
    if( !fouth.good() ){
      cerr<<"Can't Open file : "<<endl;
      return false;
    }


    int parts, vertices, polygons, edges; 
    int temp1, temp2;
    int number_nodes;
    fintxt>>number_nodes;
    vector<vertex*> configurations;

    vertices =0;
    polygons =0;
    while(!fin.eof()){
      string fname;
      double cox, coy, coz;

      fin>>fname;
      fin>>cox>>coy>>coz;
      int nov =0, nop=0;
      if(fname.find(".g") != std::string::npos){
        ifstream fing(fname.c_str(), ios::in);  
        if( !fing.good() )
        {	
          cerr<<"File  not found"<<endl;
          return false;
        }
        int nov,nop;
        fing>>parts>>nov>>nop>>edges;
        fing>>temp1>>temp2;

        polygons+= nop;
        double minx, maxx, miny, maxy, minz, maxz;

        //Get the vertices and find the max and  min in the three dimension
        for(int i=0;i<nov;i++)
        {
          vertex * newv = new vertex();
          fing>>newv->pos[0]>>newv->pos[1]>>newv->pos[2];
          if(i==0 ||newv->pos[0] < minx) minx = newv->pos[0];
          if(i==0 ||newv->pos[0] > maxx) maxx = newv->pos[0];
          if(i==0 ||newv->pos[1] < miny) miny = newv->pos[1];
          if(i==0 ||newv->pos[1] > maxy) maxy = newv->pos[1];
          if(i==0 ||newv->pos[2] < minz) minz = newv->pos[2];
          if(i==0 ||newv->pos[2] > maxz) maxz = newv->pos[2];
          newv->id = model_vertices.size();
          model_vertices.push_back(newv);
        }

        //Model centroid
        double cenx = 0.5*(minx+maxx);
        double ceny = 0.5*(miny+maxy);
        double cenz = 0.5*(minz+maxz);

        //Translate to the given centroid
        for(int i=0;i<nov;i++){
          vertex *curv = model_vertices[vertices+i];
          curv->pos[0] = curv->pos[0] +(cox - cenx);
          curv->pos[1] = curv->pos[1] +(coy - ceny);
          curv->pos[2] = curv->pos[2] +(coz - cenz);
        }

        //Read the faces
        for(int i=0;i<nop;i++)
        {
          int t1,t2,t3;
          fing>>t1>>t2>>t3;
          t3 = -t3;
          face * f = new face();
          f->id = model_faces.size();
          f->v[0] = model_vertices[vertices+t1-1];
          f->v[1] = model_vertices[vertices+t2-1];
          f->v[2] = model_vertices[vertices+t3-1];
          model_faces.push_back(f);
          f->compute_normal();
        }

        vertices+= nov;
        fing.close();
      }
    }

    //Reading g files end

    for(int i=0;i<number_nodes;i++)
    {

      vertex * newv = new vertex();
      fintxt>>newv->pos[0]>>newv->pos[1]>>newv->pos[2];
      newv->id = configurations.size();
      configurations.push_back(newv);

    }

    double * distances = new double [number_nodes];
    double * mindist = new double [number_nodes];
    face** maxface = new face* [number_nodes];
    double **interdistance = new double* [number_nodes];
    for(int i=0;i<number_nodes;i++)
      interdistance[i] = new double [number_nodes];

    /*
    //Calculate individula distance
    double sumsurface =0;
    for(int i=0;i<number_nodes;i++)
    {
      distances[i] = MAX_DIST;
      for(int j=0;j<polygons;j++)
      {
        double temp = model_faces[j]->distance(configurations[i]);
        if(temp < distances[i]) {
          distances[i] = temp;
          maxface[i] = model_faces[j];
        }
      }
      sumsurface+= distances[i];
    }

    double averagesurface = sumsurface/(double)number_nodes;
    double varsurface =0;
    double minsurface, maxsurface;
    for(int i=0;i<number_nodes;i++){
      varsurface += pow((distances[i] - averagesurface),2);
      if(i==0 || minsurface > distances[i]) minsurface = distances[i];
      if(i==0 || maxsurface < distances[i]) maxsurface = distances[i];
    } 
    varsurface = varsurface/(double)number_nodes;


    //Calculates the number of vertex in each face
    for(int i=0;i<number_nodes;i++)
      maxface[i]->vertex_contains++;
    */

    //Calculate inter-node distance 
    for(int i=0;i<number_nodes;i++){
      for(int j=i;j<number_nodes;j++){
        interdistance[i][j] = distance(configurations[i],configurations[j]);
        interdistance[j][i] = interdistance[i][j];
      }
    }

    double intersum =0;
    double minmin, minmax;
    double interavg;
    double intervar =0;
    for(int i=0;i<number_nodes;i++){
      mindist[i] = interdistance[i][0];
      if(i== 0) mindist[i] = interdistance[i][1];
      for(int j=0;j<number_nodes;j++){
        if(i!=j && interdistance[i][j] < mindist[i]) mindist[i] = interdistance[i][j];

      }
      if(i ==0 || minmin > mindist[i]) minmin = mindist[i];
      if(i ==0 || minmax < mindist[i]) minmax = mindist[i];
      intersum += mindist[i];
      fouth<<mindist[i];
      fouth<<endl;
    }
    interavg = intersum/(double)number_nodes;
    for(int i=0;i<number_nodes;i++){ intervar += pow((mindist[i]-interavg),2);}
    intervar= intervar/(double)number_nodes;

    double total_area = 0.0;
    for(int i=0; i<polygons; i++)
    {
      total_area += model_faces[i]->area();
    }

    double error_area = 0.0;
    double error1 = 0.0;
    double positiveerror =0.0; double negativeerror = 0.0;
    double error2 = 0.0;
    int errorpoly =0;
    double weighted_average=0.0;
    for(int i=0; i<polygons; i++){
      error1 += fabs(((double)number_nodes/total_area)-((double)model_faces[i]->vertex_contains/model_faces[i]->area()));
      if(model_faces[i]->vertex_contains != 0){
        weighted_average += (model_faces[i]->vertex_contains*model_faces[i]->area());
        error2 += fabs((total_area/(double)number_nodes)-(model_faces[i]->area()/(double)model_faces[i]->vertex_contains)); 
        errorpoly++;
        model_faces[i]->distribution_error = ((double)(model_faces[i]->area()/(double)total_area)- ((double)model_faces[i]->vertex_contains/(double)number_nodes));
        error_area += model_faces[i]->area();

      }
      else
      {
        error1 += (double)number_nodes/total_area;
        error2 += fabs(total_area/(double)number_nodes);
        model_faces[i]->distribution_error = ((double) model_faces[i]->area()/(double)total_area);
      }
      if((((double)number_nodes/total_area)-((double)model_faces[i]->vertex_contains/model_faces[i]->area())) < 0.0)
        negativeerror += fabs(((double)number_nodes/total_area)-((double)model_faces[i]->vertex_contains/model_faces[i]->area()));
      else
        positiveerror += (((double)number_nodes/total_area)-((double)model_faces[i]->vertex_contains/model_faces[i]->area()));

    }

    double sum=0.0, avg, var=0.0;
    for(int i=0; i<polygons; i++){

      sum += fabs(model_faces[i]->distribution_error);
    }

    avg = (double)(sum/(double)polygons);
    for(int i=0; i<polygons; i++)
      var = var + pow((model_faces[i]->distribution_error-avg), 2);
    var = var/((double)polygons);

    weighted_average = weighted_average/(number_nodes * total_area);

    ofstream ofs((string(argv[index]) + ".dist").c_str());

    /*
    ofs << "--------To Surface------------------------" << endl;
    ofs << "sum: "<< sumsurface << " average : "<< averagesurface << endl;
    ofs << "variance: "<<varsurface << " standard deviation: "<<sqrt(varsurface)<< endl;
    ofs << "minimum : "<< minsurface << " maximum: " << maxsurface <<endl;
    ofs << endl;
    */

    ofs << "--------Interconfiguration----------------" << endl;
    ofs << "sum : "<< intersum << " average : "<< interavg << endl;
    ofs << "variance: "<<intervar << " standard deviation: "<<sqrt(intervar)<<endl;
    ofs << "minimum : "<< minmin << " maximum: " << minmax <<endl;
    ofs << endl;

    /*
    ofs << "-----------ERROR--------------------------" << endl;
    ofs << "average: " << avg << endl;
    ofs << "sum: "<<sum<<" number: "<<polygons<<endl;
    ofs << "variance: " << var << endl;
    ofs << "standard deviation: " << sqrt(var) << endl;
    ofs << "positive error: " << positiveerror/(double)polygons << " negative error : " << negativeerror/(double)polygons << endl;

    ofs << "number of nodes: "<< number_nodes << endl;
    ofs << "error  in area: "<< error_area<<endl;
    ofs << "error  in %vertex: "<<error1/(double)polygons<<endl;
    ofs << "error  in %area: "<< error2 << endl;
    ofs << "weighted average: "<< weighted_average << endl;
    ofs << "-------------------------------------" << endl;   
    */

    ofs.close();
    fin.close();
    fintxt.close();
    fouth.close();
  }
  return 0;
}

