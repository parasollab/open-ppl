#ifndef GRID_SAMPLER_H_
#define GRID_SAMPLER_H_

#include "SamplerMethod.h"

#include <algorithm>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Generate configurations along a grid through @cspace.
/// @tparam MPTraits Motion planning universe
///
/// This samplers generates randomly a cfg \f$c\f$. Then for the dimensions
/// established in the xml file it generates a grid of points changing the
/// values of \f$c\f$s dimensions and validate them.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class GridSampler : public SamplerMethod<MPTraits> {

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    GridSampler(string _vcm = "",
        map<size_t, size_t> _numPoints = map<size_t, size_t>(),
        bool _useBoundary  = true);

    GridSampler(MPProblemType* _problem, XMLNode& _node);

    // Reads XML
    void ParseXML(XMLNode& _node);

    // Prints options
    virtual void Print(ostream& _os) const;

    // Attempts to sample, bool value is not working, it just return true at the end.
    virtual bool Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision);

  private:
    void GetCoordLocation(int _iter, size_t _totalCell,
        map<size_t,size_t>& _coords, map<size_t,int> _tempSize);

    void GetRealLocation(map<size_t,double>& _locations,
        map<size_t,size_t> _coordinates, shared_ptr<Boundary>  _boundary);

    string m_vcLabel; // Validity checker method
    map<size_t, size_t> m_numPoints; // Map of dimension to number of grid points
};

template<class MPTraits>
GridSampler<MPTraits>::
GridSampler(string _vcm, map<size_t, size_t> _numPoints, bool _useBoundary)
  : m_vcLabel(_vcm), m_numPoints(_numPoints) {
    this->SetName("GridSampler");
  }

template<class MPTraits>
GridSampler<MPTraits>::
GridSampler(MPProblemType* _problem, XMLNode& _node) :
  SamplerMethod<MPTraits>(_problem, _node) {
    this->SetName("GridSampler");
    ParseXML(_node);
  }

// Reads XML
template<class MPTraits>
void
GridSampler<MPTraits>::
ParseXML(XMLNode& _node) {

  // Read grid data and store in m_numPoints
  for(auto& child : _node) {
    if(child.Name() == "Dimension") {
      size_t points = child.Read("points", true, 10, 0,
          MAX_INT, "Number of grid points, excluding min and max");
      size_t index = child.Read("index", true, 0, 0, MAX_INT,
          "Index in bounding box");
      m_numPoints[index] = points;
    }
  }

  m_vcLabel = _node.Read("vcLabel", true, "", "Validity test method");
}

// Prints options
template<class MPTraits>
void
GridSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tnumPoints (index, points):" << endl;
  for(auto& dim : m_numPoints)
    _os << "\t\t" << dim.first << ", " << dim.second << endl;
}

// Attempts to sample, bool value is not working, it just return true at the end.
template<class MPTraits>
bool
GridSampler<MPTraits>::
Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  string callee = this->GetNameAndLabel() + "::Sampler()";
  ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);

  //Calculate total number of cfg to be created
  size_t totalCell = 1;
  for(auto&  num : m_numPoints) {
    totalCell = totalCell * num.second;
  }
  //Generate all the points in the grid for the asked dimensions
  for(size_t i = 0; i < totalCell; ++i) {
    map<size_t,size_t> coordinates;
    map<size_t,int> tempSize;
    int dimSize = 1;
    //Get the cummulative sizes of the dimensions required.
    //Multiply the size of the n first dimensions and each time use one more dim.
    for(auto&  num : m_numPoints) {
      int index = num.first;
      int sizee = num.second;
      dimSize = dimSize * sizee;
      tempSize[index] = dimSize;
    }
    //Returns coordinates in the grid as int value (uses m_numPoints)
    GetCoordLocation(i, totalCell, coordinates, tempSize);

    //Turn the grid's coordinates into real values (uses m_numPoints, _boundary)
    map<size_t,double> locations;
    GetRealLocation(locations, coordinates, _boundary);

    //Update the DoF values of the point generated.
    for(auto&  location : locations)
      _cfg[location.first] = location.second;

    // Is _cfg a valid configuration?
    if(this->GetEnvironment()->InBounds(_cfg, _boundary) &&
        vc->IsValid(_cfg, callee)) {
      // Yes (sampler successful)
      _result.push_back(_cfg);
    }
  }
  return true;
}

/*
 * Calculate the grid's coordinates for a cfg specified as an int number (use m_numPoints)
 * Each point in the grid is numerated. This function transforms the int number
 * into coordinates of the dimensions required.
 */
template<class MPTraits>
void
GridSampler<MPTraits>::
GetCoordLocation(int _iter, size_t _totalCell, map<size_t,size_t>& _coords,
    map<size_t,int> _tempSize) {

  int iterVal = _iter; //Assigning variable value as _iter.
  int divResult= 0;
  int mod = 0;
  int lastIt= 0;

  //Check if it is just one point's dimension required
  //If the grid is defined in 1 dim, then return _iter as the coordinate.
  if(_tempSize.size() == 1) {
    map<size_t, size_t>::iterator iterat = m_numPoints.begin();
    int inde = iterat->first;
    _coords[inde] = _iter;
    return;
  }

  //If there more than 1 dim, for each dim is needed to take the values of the coordinates
  //by dividing the int value that represent the grid point by the tempSize related to
  //each dim.
  //Note: It starts at the end of the map going backwards saving the mod value.

  for(map<size_t, int>::reverse_iterator it = _tempSize.rbegin();
      it != _tempSize.rend();it++) {
    int ind = it->first;
    int siz = it->second; //Saving the cummulative size of ind dim

    if(it == _tempSize.rbegin()) { //If this is the bigger dimension just safe 0
      _coords[ind] = 0;
      lastIt = ind; //Saving the reference value to the location asigned with 0.
    }
    else {
      divResult = iterVal/siz; //Divide iterVal by siz
      mod = iterVal%siz; //Calculate mod of iterVal mod siz.
      _coords[lastIt] = divResult; //Saving divResult but using the preceding iter
      _coords[ind] = mod; //Saving the mod value using the present iter
      lastIt = ind; //Update auxiliaries
      iterVal = mod;
    }
  }
}

//Calculate the real point in the workspace for a cfg having the number of grid's point
//of each dimension.
template<class MPTraits>
void
GridSampler<MPTraits>::
GetRealLocation(map<size_t,double>& _locations, map<size_t,size_t> _coordinates,
    shared_ptr<Boundary>  _boundary) {

  for(auto&  num : m_numPoints) {
    int index = num.first;
    int numPoints = num.second;

    // Get bounding box min and max (for this dimension)
    pair<double, double> range = _boundary->GetRange(index);

    // Resolution of grid
    double auxPoints = (double) numPoints;
    double delta;
    if(numPoints == 0)
      delta = (range.second - range.first);
    else
      delta = (range.second - range.first) / auxPoints;

    // Get the real place in the workspace .
    double coor = (double) _coordinates[index];
    double gridVal = (coor * delta) + range.first;

    //Push gridVal inside bounding box (if necessary)
    gridVal = min(range.second, gridVal);
    gridVal = max(range.first, gridVal);
    _locations[index] =(double) gridVal;
  }
}

#endif
