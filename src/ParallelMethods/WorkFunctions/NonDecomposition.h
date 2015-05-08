#ifndef NonDecomposition_h
#define NonDecomposition_h
//#include "DecompositionMethod.h"

#include <sstream>
using namespace std;

class Environment;
class Body;
class BoundingBox;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <typename CFG>
class NonDecomposition {
  //public DecompositionMethod { //<CFG> {
  //TO DO: Move some of these to base class
 private:
  Environment* env;
  std::string strLabel;
  int dims;
  int n_partitions;
  double epsilon;
  int posdof;
  int dof;

 public:
  NonDecomposition(BoundingBox &c_boundary, int m_partitions=2){
    n_partitions = m_partitions;
  }
  NonDecomposition() {};
  ~NonDecomposition() {}


  const char* Name() const { return "NonDecomposition"; }

  void Print(ostream& os) const {
    os << Name()
       << " (# of partition = " << n_partitions
       << ", overlap btw partition = " << epsilon
       << ", # dims = " << dims << ")";
  }

  template <typename OutputIterator>
    void
    Decompose( Environment* _env, BoundingBox &c_boundary,int n_partitions,OutputIterator _out,vector<BoundingBox>& vbox);
  template <typename OutputIterator>
    void
    DecomposeWS( Environment* _env, BoundingBox &c_boundary,int nx, int ny,int nz, OutputIterator _out);
};

template<typename CFG>
template<typename OutputIterator>
void
NonDecomposition<CFG>::
Decompose(Environment* _env, BoundingBox &c_boundary,int n_partitions,OutputIterator _out, vector<BoundingBox>& vbox){
	///@todo same as DecomposeWS but fix parameter
}

template<typename CFG>
template<typename OutputIterator>
void
NonDecomposition<CFG>::
DecomposeWS(Environment* _env, BoundingBox &c_boundary,int nx, int ny, int nz, OutputIterator _out){
  for(int i=1; i<nx+1; ++i){
    double Xmin = c_boundary.GetRange(0).first;
    double Xmax = c_boundary.GetRange(0).second;
    for(int i=1; i<ny+1; ++i){
      double Ymin = c_boundary.GetRange(1).first;
      double Ymax = c_boundary.GetRange(1).second;
      for(int i=1; i<nz+1; ++i){
	BoundingBox tmp(c_boundary.GetDOFs(),c_boundary.GetPosDOFs());
	double Zmin = c_boundary.GetRange(2).first;
	double Zmax = c_boundary.GetRange(2).second;
	tmp.SetParameter(0,Xmin, Xmax);
	tmp.SetParameter(1,Ymin, Ymax);
	tmp.SetParameter(2,Zmin,Zmax);
	*_out++ = tmp;
      }
    }
  }
}

#endif
