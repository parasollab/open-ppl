// Samples by "snapping" random configurations to lattice points in a grid

#ifndef GRIDSAMPLER_H_
#define GRIDSAMPLER_H_

#include "SamplerMethod.h"
#include <algorithm>

template <typename CFG>
class GridSampler : public SamplerMethod<CFG> {

  string m_vcm; // Validity checker method 
  map<size_t, size_t> m_numPoints; // Map of dimension to number of grid points
  bool m_useBoundary; // Is the bounding box an obstacle?

  public:

  ////////////////////////////
  //Constructors
  ////////////////////////////
  
  GridSampler(string _vcm = "", map<size_t, size_t> _numPoints = map<size_t, size_t>(), bool _useBoundary = false) 
    : m_vcm(_vcm), m_numPoints(_numPoints), m_useBoundary(_useBoundary) {
      this->SetName("GridSampler");
    }

  GridSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
    this->SetName("GridSampler");
    ParseXML(_node);

    if(this->m_debug) 
      PrintOptions(cout);
  }

  ~GridSampler() {}

  // Reads XML
  void ParseXML(XMLNodeReader& _node) {

    // Read grid data and store in m_numPoints
    for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); citr++) {
      if(citr->getName() == "dimension") {
        size_t points = citr->numberXMLParameter("points", true, 10, 0, MAXINT, "Number of grid points, excluding min and max");
        size_t index = citr->numberXMLParameter("index", true, 0, 0, MAXINT, "Index in bounding box");
        m_numPoints[index] = points;

        citr->warnUnrequestedAttributes();
      }
      else
        citr->warnUnknownNode();
    }

    // Read data to m_vcm and m_useBBX
    m_vcm = _node.stringXMLParameter("vcMethod", true, "", "Validity test method");
    m_useBoundary = _node.boolXMLParameter("useBBX", true, false, "Use bounding box as obstacle");

    _node.warnUnrequestedAttributes();
  }

  // Prints options
  virtual void PrintOptions(ostream& _os) const {
    SamplerMethod<CFG>::PrintOptions(_os);
    _os << "\tm_vcm = " << m_vcm << endl;
    _os << "\tm_useBoundary = " << m_useBoundary << endl;
    _os << "\tm_numPoints (index, points):" << endl;
    for(map<size_t, size_t>::const_iterator it = m_numPoints.begin(); it != m_numPoints.end(); it++) 
      _os << "\t\t" << it->first << ", " << it->second << endl;
  }

  // Attempts to sample, returns true if successful

 virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats, 
     CFG& _cfgIn, vector<CFG>& _cfgOut, vector<CFG>& _cfgCol) {

   // When using grid sampler, set the TestEval to a number 
   // that is a no more than 70-80% of the total number of 
   // possible grid points. Otherwise a very long time may pass
   // before enough unique points are generated

   string callee = this->GetNameAndLabel() + "::Sampler()";
   ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
   CDInfo cdInfo;

   // Set tmp to _cfgIn or a random configuration
   CFG tmp = _cfgIn;
   if(tmp == CFG())
     tmp.GetRandomCfg(_env,_bb);
   
   if(this->m_debug){
     VDClearAll();  
     VDAddTempCfg(tmp, true);
   }

   // Loop through each dimension index
   for(map<size_t, size_t>::iterator it = m_numPoints.begin(); it != m_numPoints.end(); it++) {
     int index = it->first;
     int numPoints = it->second;

     // Get bounding box min and max (for this dimension)
     double bbMin = _bb->GetRange(index).first;
     double bbMax = _bb->GetRange(index).second;

     // Get starting point (for this dimension)
     double dofVal = tmp.GetSingleParam(index);

     // Resolution of grid
     double delta = (bbMax - bbMin) / (numPoints + 1);

     // Number of grid cells from bounding box min
     double steps = floor(((dofVal - bbMin) / delta) + 0.5);

     // The grid location closest to dofVal
     double gridVal = steps*delta + bbMin;

     // Push gridVal inside bounding box (if necessary)
     gridVal = min(bbMax, gridVal);
     gridVal = max(bbMin, gridVal);

     // If the bounding box is an obstacle, move gridVal further inside bounding box (if necessary) 
     if(m_useBoundary) {
       if(fabs(bbMax - gridVal) <= delta/10)
         gridVal = bbMax - delta;
       if(fabs(bbMin - gridVal) <= delta/10)
         gridVal = bbMin + delta;
     }

     // Set the parameter at index
     tmp.SetSingleParam(index, gridVal);
     if(this->m_debug){
       ostringstream oss; 
       oss << "GridSampler::Dim::" << dofVal << "::grid value::" << gridVal; 
       VDComment(oss.str());
       VDClearLastTemp();
       VDAddTempCfg(tmp, true);
     }
   }

   // Is tmp a valid configuration?
   if(tmp.InBoundary(_env,_bb) && 
       vc->IsValid(vc->GetVCMethod(m_vcm), tmp, _env, _stats, cdInfo, true, &callee)) {
     // Yes (sampler successful)
     _cfgOut.push_back(tmp);
     
     if(this->m_debug){
       VDClearLastTemp();
       VDAddTempCfg(tmp, true);
       ostringstream oss; 
       oss << "GridSampler::Successful";
       VDComment(oss.str()); 
     } 
     return true;
   }
   else if(this->m_debug){
     VDClearLastTemp();
     VDAddTempCfg(tmp, false);
     ostringstream oss; 
     oss << "GridSampler::failure";
     VDComment(oss.str()); 
   }

   return false;
 }
};

#endif

