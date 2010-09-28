#ifndef CLEARANCEFEATURE_H_
#define CLEARANCEFEATURE_H_

#include "MPFeature.h"
#include "MPProblem.h"

class ClearanceFeature : public MPFeature {
public:
	ClearanceFeature();
	ClearanceFeature(XMLNodeReader& in_Node, MPProblem* in_PProblem);
	~ClearanceFeature(){}

	virtual void ParseXML(XMLNodeReader& in_Node);

	virtual vector<double> Collect(vector<VID>& vids);
private:
	string m_vc;
};

#endif

	
