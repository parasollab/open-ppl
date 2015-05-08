#ifndef _MPNN_WRAPPER_H_
#define _MPNN_WRAPPER_H_

#include <vector>
#include <map>
#include "DNN/ANN.h"
#include "DNN/multiann.h"

#ifndef PI
#define PI 3.1415926535897932385
#endif

using namespace std;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinderUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class MPNNWrapper
{
  public:
    MPNNWrapper();
    ~MPNNWrapper();

    // here, scale refers to how much weight a given DOF carries...
    //   rotational DOFs should have a higher scale than regular DOFs
    MPNNWrapper(vector<int> _topology, int maxPts, int maxNeighbors, double _epsilon);
    MPNNWrapper(vector<int> _topology, vector<double> _scale,
                int maxPts, int maxNeighbors, double _epsilon);

    // return type = MPNN's index for this point
    //   this point is also stored in vidMapping
    int add_node(vector<double> point, int vid);

    // this returns the VID associated with the given point
    int get_vid(vector<double> point);

    // get the k-closest VIDs to the input point (not necessarily in tree)
    template <typename OutputIterator>
    OutputIterator KClosest(vector<double> point, int k, OutputIterator _out);

    // get the k-closest VIDs to the input vid (already in tree)
    template <typename OutputIterator>
    OutputIterator KClosest(int vid, int k, OutputIterator _out);

    int current_size;

  private:
    int dim;
    int max_pts;
    int max_neighbors;
    double epsilon;

    ANNpointArray data_pts;

    ANNpoint scale;
    int *topology;

    MultiANN *MAG;

    // maps a MultiANN index (int) to a Roadmap VID
    map<int, int> indexToVIDMapping;
    map<int, int> vidToIndexMapping;
};

/*
  Returns the k-closest VIDs to a given input CFG, and returns the results in
  the OutputIterator _out.  Some pre- and post-processing is required, because
  The default k-closest results include the input point.  We must return one
  extra neighbor, and remove the input VID from the results.
*/
template <typename OutputIterator>
OutputIterator
MPNNWrapper::
KClosest(vector<double> point, int k, OutputIterator _out)
{
  // return one extra node, then filter
  k += 1;

  int length = point.size();
  // scale input point to match MPNN input (2*PI)
  for (int i = 0; i < length; i++)
  {
    if (topology[i] == 1)
      point.at(i) = 1 * point.at(i);
    if (topology[i] == 2)
      point.at(i) = 2*PI * point.at(i);
    else
      point.at(i) = 1 * point.at(i);
  }

  ANNpoint query_pt = annAllocPt(point.size());
  for (size_t i = 0; i < point.size(); i++)
    query_pt[i] = point.at(i);

  double *d_best_list = new double[max_neighbors];
  int *idx_best_list = new int[max_neighbors];
  double **n_best_list = new double*[max_neighbors];

  /*
  cout << "Query point: \t(";
  for (int i = 0; i < dim - 1; i++) cout << query_pt[i] << ", ";
  cout << query_pt[dim - 1] << ")" << endl;
  */

  MAG->NearestNeighbor(query_pt, k, d_best_list, idx_best_list, (void **)n_best_list);

  //cout << "Nearest Neighbor results: " << endl;
  for (int i = 0; i < k; i++)
  {
    int resultIndex = idx_best_list[i];
    int resultVID = indexToVIDMapping[resultIndex];

    // check to see if this result is equal to the input point... filter
    //   this point out, if that is the case
    bool foundQueryPoint = true;
    ANNpoint resultPoint = data_pts[resultIndex];
    for (size_t j = 0; j < point.size(); j++) {
      if (resultPoint[j] != query_pt[j]) {
        foundQueryPoint = false;
        break;
      }
    }
    if (foundQueryPoint == true) {
      // ignore this point
      continue;
    }

    //cout << d_best_list[i] << " - index = " << resultVID << endl;
    *_out = pair<int, double>(resultVID, d_best_list[i]);
    ++_out;
  }

  return _out;
}

/*
  Returns the k-closest VIDs to a given input VID, and returns the results in
  the OutputIterator _out.  Some pre- and post-processing is required, because
  The default k-closest results include the input point.  We must return one
  extra neighbor, and remove the input VID from the results.
*/
template <typename OutputIterator>
OutputIterator
MPNNWrapper::
KClosest(int vid, int k, OutputIterator _out)
{
  int inputIndex = vidToIndexMapping[vid];

  // return one extra neighbor
  k += 1;

  ANNpoint query_pt = data_pts[inputIndex];
  double *d_best_list = new double[max_neighbors];
  int *idx_best_list = new int[max_neighbors];
  double **n_best_list = new double*[max_neighbors];

  /*
  cout << "Query point: " << vid << "\t(";
  for (int i = 0; i < dim - 1; i++) cout << query_pt[i] << ", ";
  cout << query_pt[dim - 1] << ")" << endl;
  */

  MAG->NearestNeighbor(query_pt, k, d_best_list, idx_best_list, (void **)n_best_list);

  //cout << "Nearest Neighbor results: " << endl;
  for (int i = 0; i < k; i++)
  {
    int resultIndex = idx_best_list[i];
    int resultVID = indexToVIDMapping[resultIndex];

    // filter input VID
    if (resultVID == vid)
      continue;

    //cout << d_best_list[i] << " - index = " << resultVID << endl;
    *_out = pair<int, double>(resultVID, d_best_list[i]);
    ++_out;
  }

  return _out;
}

#endif //end #ifndef _MPNN_WRAPPER_H_
