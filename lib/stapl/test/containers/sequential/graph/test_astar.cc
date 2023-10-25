/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "test_util.h"
#include <limits>

#include <stapl/containers/sequential/graph/algorithms/dijkstra.h>
#include <stapl/containers/sequential/graph/algorithms/astar.h>
#include <stapl/runtime/counter/default_counters.hpp>

using namespace stapl;
using namespace stapl::sequential;
using namespace std;

//Euclidean point in 2-space as vertex of metric graph
class point{
  public:
    point(double a = 1E10, double b = 1E10):x(a), y(b){}
    ~point()= default;

    double get_x(){return x;}
    double get_y(){return y;}

    friend ostream& operator<<(ostream& os, const point& p){
      return os << "[" << p.x << ", " << p.y << "]";
    }

  private:
    double x, y;
};

//Euclidean euclidean_distance function to calulate weights for edges
class euclidean_distance{
  public:
    euclidean_distance()= default;

    double operator()(point& x, point& y){
      return sqrt(
          (x.get_x()-y.get_x())*(x.get_x()-y.get_x())
          +(x.get_y()-y.get_y())*(x.get_y()-y.get_y())
          );
    }
  private:
};

//A* Heuristic function for metric graph in 2-space
class euclidean_distance_from{
  public:
    euclidean_distance_from(point& a) : p(a) {}

    double operator()(point& x){
      return euclidean_distance()(p, x);
    }
  private:
    point p;
};

//Global Graph Typedefs
typedef graph<DIRECTED, NONMULTIEDGES, point, double> random_graph;
typedef random_graph::vertex_descriptor VID;
typedef random_graph::vertex_iterator VI;

//Build random graph with n ranomd vertices and n*ln(n) random edges
void build_random_graph(random_graph& g, size_t n){
  size_t ne = log(n);
  vector<VID> nodes, shuffle;
  for(size_t i = 0; i<n; i++){
    point p((double)rand()/(double)RAND_MAX - 0.5,
        (double)rand()/(double)RAND_MAX - 0.5);
    nodes.push_back(g.add_vertex(p));
  }

  copy(nodes.begin(), nodes.end(), back_inserter(shuffle));
  typedef vector<VID>::iterator VIT;
  for(VIT vit = nodes.begin(); vit!=nodes.end(); ++vit){
    point& x = g.find_vertex(*vit)->property();
    random_shuffle(shuffle.begin(), shuffle.end());
    for(VIT vit2 = shuffle.begin(); vit2!=shuffle.begin()+ne; ++vit2){
      if(*vit != *vit2){
        point& y = g.find_vertex(*vit2)->property();
        g.add_edge(*vit, *vit2, euclidean_distance()(x, y));
      }
    }
  }

  if(n <= 10)
    write_graph(g, cout);
}


int main(int argc, char** argv){
  size_t n = 10;
  cout<<"AStar Metric Graph Tests\n";
  if(argc < 2) 
    cout<<"by default n is 10\n";
  else 
    n = atoi(argv[1]);

  srand(n);

  cout<<"test astar on graph<DIRECTED, NONMULTIEDGES,point,double>...\n";

  //build the graph
  random_graph graph;
  build_random_graph(graph, n);

  double dijkstraTime = 0, astarTime = 0;

  //test all to all connection of paths
  for(VI vi = graph.begin(); vi!=graph.end(); ++vi){
    for(VI vi2 = graph.begin(); vi2!=graph.end(); ++vi2){
      vector<VID> pathDijkstra, pathAStar;

      //run Dijkstra separate
      counter<default_timer> dijkstraTimer;
      dijkstraTimer.start();
      find_path_dijkstra(graph, vi->descriptor(), vi2->descriptor(), pathDijkstra, 1E10);
      dijkstraTime += dijkstraTimer.stop();

      //run AStar
      euclidean_distance_from dToGoal(vi2->property());
      counter<default_timer> astarTimer;
      astarTimer.start();
      astar(graph, vi->descriptor(), vi2->descriptor(),
          pathAStar, dToGoal);
      astarTime += astarTimer.stop();

      //Compare paths
      if(pathDijkstra.size() != pathAStar.size() ||
          !equal(pathDijkstra.begin(), pathDijkstra.end(), pathAStar.begin())){
        cout << "Failed" << endl;
        return 1;
      }
    }
  }

  cout << "Passed" << endl;
  
  cout << endl << "Time Dijkstra::" << dijkstraTime << endl;
  cout << endl << "Time AStar::" << astarTime << endl;

  return 0;
}
