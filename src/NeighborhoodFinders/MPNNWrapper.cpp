#include "MPNNWrapper.h"

MPNNWrapper::MPNNWrapper() { }

MPNNWrapper::~MPNNWrapper()
{
  MAG->MultiANN::~MultiANN();
}

MPNNWrapper::MPNNWrapper(vector<int> _topology, int maxPts, int maxNeighbors, double _epsilon)
{

  dim = _topology.size();
  max_pts = maxPts;
  max_neighbors = maxNeighbors;
  epsilon = _epsilon;

  data_pts = annAllocPts(max_pts, dim);
  topology = new int[dim];

  scale = new double[dim];

  for (int i = 0; i < dim; i++)
  {
    topology[i] = _topology.at(i);

    // topology for normal positional DOF
    if (_topology.at(i) == 1)
      scale[i] = 1.0;
    // topology for rotational DOF
    else if (_topology.at(i) == 2)
      scale[i] = 1.0;
    // topology for the other type of coordinate
    else if (_topology.at(i) == 3)
      scale[i] = 1.0;//50.0;
    else
      scale[i] = 1.0;

  }

  current_size = 0;

  MAG = new MultiANN(dim, topology, scale, epsilon);
  //
}

MPNNWrapper::MPNNWrapper(vector<int> _topology, vector<double> _scale,
                            int maxPts, int maxNeighbors, double _epsilon)
{
  dim = _topology.size();
  max_pts = maxPts;
  max_neighbors = maxNeighbors;
  epsilon = _epsilon;

  data_pts = annAllocPts(max_pts, dim);
  topology = new int[dim];
  scale = new double[dim];

  for (int i = 0; i < dim; i++)
  {
    topology[i] = _topology.at(i);
    scale[i] = _scale.at(i);
  }

  current_size = 0;

  MAG = new MultiANN(dim, topology, scale, epsilon);
}

int
MPNNWrapper::
add_node(vector<double> point, int vid)
{
  ANNpoint data_point = annAllocPt(dim);
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

  for (int i = 0; i < length; i++)
    data_point[i] = point.at(i);

  int index = current_size;
  data_pts[index] = data_point;

  MAG->AddPoint(data_pts[index], data_pts[index]);
  indexToVIDMapping[index] = vid;
  vidToIndexMapping[vid] = index;

  current_size++;

  return index;
}

int
MPNNWrapper::
get_vid(vector<double> point)
{
  int vid = -1;

  for (int i = 0; i < current_size; i++)
  {
    vid = indexToVIDMapping[i];
    for (int j = 0; j < dim; j++) {
      if (data_pts[i][j] != point.at(j)) {
        vid = -1;
        break;
      }

      if (vid != -1)
        break;
    }
  }

  return vid;
}

