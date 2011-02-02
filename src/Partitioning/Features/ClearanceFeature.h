#ifndef CLEARANCEFEATURE_H_
#define CLEARANCEFEATURE_H_

#include "MPFeature.h"
#include "MPProblem.h"

class ClearanceFeature : public MPFeature {
public:
	ClearanceFeature();
	ClearanceFeature(string _vc);
	ClearanceFeature(XMLNodeReader& in_Node, MPProblem* in_PProblem);
	virtual ~ClearanceFeature(){}

	virtual void ParseXML(XMLNodeReader& in_Node);

	virtual vector<double> Collect(vector<VID>& vids);
private:
	string m_vc;
};

#endif

	
