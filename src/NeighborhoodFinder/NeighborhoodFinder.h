#ifndef NEIGHBORHOODFINDER_H_
#define NEIGHBORHOODFINDER_H_

#include "MPUtils.h"
#include "MetricUtils.h"

#include "BFNF.hpp"
#include "BFFNF.hpp"
#include "RadiusNF.hpp"
//#include "DPESNF.hpp"
//#include "MPNNNF.hpp"
#include "CGALNF.hpp"
//#include "STNF.hpp"
//#include "MTNF.hpp"
#include "BandsNF.hpp"

/**This is the main distance metric class.  It contains two vectors: all 
  *and selected.  all contains all of the different types of distance 
  *metric methods.  selected contains only those selected by the user.
  */
typedef ElementSet<NeighborhoodFinderMethod> NeighborhoodFinderSet;

class NeighborhoodFinder : private NeighborhoodFinderSet, public MPBaseObject {
  public:
    typedef NeighborhoodFinderSet::MethodPointer NeighborhoodFinderPointer;

    NeighborhoodFinder(): NeighborhoodFinderSet(pmpl_detail::NeighborhoodFinderMethodList()) {};

    NeighborhoodFinder(XMLNodeReader& _node, MPProblem* _problem)
      : NeighborhoodFinderSet(pmpl_detail::NeighborhoodFinderMethodList()), MPBaseObject(_problem) {
        NeighborhoodFinderSet::ParseXML(_node, _problem);
        PrintOptions(cout);
      }
    
    ~NeighborhoodFinder() { }

    NeighborhoodFinderPointer GetMethod(const string& _label){
      return GetElement(_label);
    }

    void AddMethod(string const& _label, NeighborhoodFinderPointer _nfp){
      AddElement(_label, _nfp);
    }

    virtual void SetMPProblem(MPProblem* _mp){
      MPBaseObject::SetMPProblem(_mp);
      NeighborhoodFinderSet::SetMPProblem(_mp);
    }
    
    virtual void PrintOptions(ostream& _os) {
      _os << "  Neighborhood Finder Methods" << endl;
      map<string, NeighborhoodFinderPointer>::const_iterator NFI;
      for(NFI = this->ElementsBegin(); NFI != this->ElementsEnd(); ++NFI){
        _os << "  " << NFI->first << "::\t";
        NFI->second->PrintOptions(_os);
      }
    }
};

#endif
