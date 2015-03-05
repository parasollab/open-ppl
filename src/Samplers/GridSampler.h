/*
 * GridSampler.h
 * This samplers generates randomly a cfg c. Then for the dimensions stablished in the xml file
 * it generates a grid of points changing the values of c's dimensions and validate them.
 *
 */

#ifndef GRID_SAMPLER_H_
#define GRID_SAMPLER_H_

#include "SamplerMethod.h"

#include <algorithm>

template <class MPTraits>
class GridSampler : public SamplerMethod<MPTraits> {

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    GridSampler(string _vcm = "",
        map<size_t, size_t> _numPoints = (map<size_t, size_t>()),
        bool _useBoundary  = true);

    GridSampler(MPProblemType* _problem, XMLNodeReader& _node);

    // Reads XML
    void ParseXML(XMLNodeReader& _node);

    // Prints options
    virtual void Print(ostream& _os) const;

    // Attempts to sample, bool value is not working, it just return true at the end.
    virtual bool Sampler(Environment* _env,
        shared_ptr<Boundary> _bb, StatClass& _stats,
        CfgType& _cfgIn, vector<CfgType>& _cfgOut,
        vector<CfgType>& _cfgCol);

  private:
    void GetCoordLocation(int _iter, size_t _totalCell,
        map<size_t,size_t>& _coords, map<size_t,int> tempSize);

    void GetRealLocation(map<size_t,double>& _locations,
        map<size_t,size_t> _coordinates,
        Environment*  _env, shared_ptr<Boundary>  _bb);

    string m_vcLabel; // Validity checker method
    map<size_t, size_t> m_numPoints; // Map of dimension to number of grid points
    bool m_useBoundary; // Is the bounding box an obstacle?
};

template<class MPTraits>
GridSampler<MPTraits>::
GridSampler(string _vcm, map<size_t, size_t> _numPoints, bool _useBoundary)
  : m_vcLabel(_vcm), m_numPoints(_numPoints),
  m_useBoundary(_useBoundary) {
    this->SetName("GridSampler");
  }

template<class MPTraits>
GridSampler<MPTraits>::
GridSampler(MPProblemType* _problem, XMLNodeReader& _node) :
  SamplerMethod<MPTraits>(_problem, _node) {
    this->SetName("GridSampler");
    ParseXML(_node);
  }

// Reads XML
template<class MPTraits>
void
GridSampler<MPTraits>::
ParseXML(XMLNodeReader& _node) {

  // Read grid data and store in m_numPoints
  for(XMLNodeReader::childiterator citr =_node.children_begin();
      citr != _node.children_end(); citr++) {
    if(citr->getName() == "Dimension") {
      size_t points = citr->numberXMLParameter("points", true, 10, 0,
          MAX_INT, "Number of grid points, excluding min and max");
      size_t index = citr->numberXMLParameter("index", true, 0, 0, MAX_INT,
          "Index in bounding box");
      m_numPoints[index] = points;

      citr->warnUnrequestedAttributes();
    }
    else
      citr->warnUnknownNode();
  }

  // Read data to m_vcLabel and m_useBBX
  m_vcLabel = _node.stringXMLParameter("vcLabel", true, "",
      "Validity test method");
  m_useBoundary = _node.boolXMLParameter("useBBX", true, false,
      "Use bounding box as obstacle");

  _node.warnUnrequestedAttributes();
}

// Prints options
template<class MPTraits>
void
GridSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tuseBoundary = " << m_useBoundary << endl;
  _os << "\tnumPoints (index, points):" << endl;
  for(map<size_t, size_t>::const_iterator it = m_numPoints.begin();
      it != m_numPoints.end(); it++)
    _os << "\t\t" << it->first << ", " << it->second << endl;
}

// Attempts to sample, bool value is not working, it just return true at the end.
template<class MPTraits>
bool
GridSampler<MPTraits>::
Sampler(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats,
    CfgType& _cfgIn, vector<CfgType>& _cfgOut, vector<CfgType>& _cfgCol) {

  string callee = this->GetNameAndLabel() + "::Sampler()";
  ValidityCheckerPointer vc = this->GetMPProblem()->
    GetValidityChecker(m_vcLabel);

  // Set tmp to _cfgIn or a random configuration
  CfgType tmp = _cfgIn;
  if(tmp == CfgType())
    tmp.GetRandomCfg(_env,_bb);

  //Calculate total number of cfg to be created
  size_t totalCell = 1;
  for(map<size_t, size_t>::iterator it = m_numPoints.begin();
      it != m_numPoints.end(); it++) {
    totalCell = totalCell* it->second;
  }
  //Generate all the points in the grid for the asked dimensions
  for(size_t iter=0; iter<totalCell; iter++) {
    map<size_t,size_t> coordinates;
    map<size_t,int> tempSize;
    int dimSize = 1;
    //Get the cummulative sizes of the dimensions required.
    //Multiply the size of the n first dimensions and each time use one more dim.
    for(map<size_t, size_t>::iterator it = m_numPoints.begin();
        it != m_numPoints.end(); it++) {
      int index = it->first;
      int sizee = it->second;
      dimSize = dimSize * sizee;
      tempSize[index] = dimSize;
    }
    //Returns coordinates in the grid as int value (uses m_numPoints)
    GetCoordLocation(iter,totalCell,coordinates,tempSize);

    //Turn the grid's coordinates into real values (uses m_numPoints, _env, _bb)
    map<size_t,double> locations;
    GetRealLocation(locations,coordinates, _env, _bb);

    //Update the DoF values of the point generated.
    for (map<size_t, double>::iterator it = locations.begin();
        it != locations.end(); it++) {
      int index = it->first;
      double sizee = it->second;
      tmp[index] = sizee;
    }

    // Is tmp a valid configuration?
    if(_env->InBounds(tmp, _bb) && vc->IsValid(tmp, callee)) {
      // Yes (sampler successful)
      _cfgOut.push_back(tmp);
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
    map<size_t,int> tempSize) {

  int iterVal = _iter; //Assigning variable value as _iter.
  int divResult= 0;
  int mod = 0;
  int lastIt= 0;

  //Check if it is just one point's dimension required
  //If the grid is defined in 1 dim, then return _iter as the coordinate.
  if(tempSize.size() == 1) {
    map<size_t, size_t>::iterator iterat = m_numPoints.begin();
    int inde = iterat->first;
    _coords[inde] = _iter;
    return;
  }

  //If there more than 1 dim, for each dim is needed to take the values of the coordinates
  //by dividing the int value that represent the grid point by the tempSize related to
  //each dim.
  //Note: It starts at the end of the map going backwards saving the mod value.

  for(map<size_t, int>::reverse_iterator it = tempSize.rbegin();
      it != tempSize.rend();it++) {
    int ind = it->first;
    int siz = it->second; //Saving the cummulative size of ind dim

    if(it == tempSize.rbegin()) { //If this is the bigger dimension just safe 0
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
    Environment*  _env, shared_ptr<Boundary>  _bb) {

  for(map<size_t, size_t>::iterator it = m_numPoints.begin();
      it != m_numPoints.end(); it++) {
    int index = it->first;
    int numPoints = it->second;

    // Get bounding box min and max (for this dimension)
    pair<double, double> range = _env->GetRange(index, _bb);

    // Resolution of grid
    double auxPoints = (double) numPoints;
    double delta;
    if(numPoints == 0) {
      delta = (range.second - range.first);
    }
    else {
      delta = (range.second - range.first) / auxPoints;
    }

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
