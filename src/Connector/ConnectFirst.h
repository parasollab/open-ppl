#ifndef ConnectFirst_h
#define ConnectFirst_h

#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"


template <class CFG, class WEIGHT>
class ConnectFirst : public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  ConnectFirst();
  ConnectFirst(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~ConnectFirst();
 
  //////////////////////
  // Access
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void ParseXML(XMLNodeReader& in_Node);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);  
  virtual void PrintOptions(ostream& _os);  
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy();

  //////////////////////
  // Core: Connection method
  void Connect(Roadmap<CFG, WEIGHT>* rm, Stat_Class& Stats, 
	       DistanceMetric* dm,
	       LocalPlanners<CFG,WEIGHT>* lp,
	       bool addPartialEdge, bool addAllEdges);  
  void Connect(Roadmap<CFG, WEIGHT>* rm, Stat_Class& Stats,
	       DistanceMetric* dm,
	       LocalPlanners<CFG,WEIGHT>* lp,
	       bool addPartialEdge, bool addAllEdges,
	       vector<VID>& cfgs1, vector<VID>& cfgs2);
  
 private:
  //////////////////////
  // Data
  
  int kfirst;
};


template <class CFG, class WEIGHT>
ConnectFirst<CFG,WEIGHT>::
ConnectFirst() : NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "connectfirst"; 
  SetDefault();
}


template <class CFG, class WEIGHT>
ConnectFirst<CFG,WEIGHT>::
ConnectFirst(XMLNodeReader& in_Node, MPProblem* in_pProblem) : NodeConnectionMethod<CFG,WEIGHT>(in_Node, in_pProblem)
{
  LOG_DEBUG_MSG("ConnectFirst::ConnectFirst()");
  this->element_name = "connectfirst";
  SetDefault();
  this->ParseXML(in_Node);
  LOG_DEBUG_MSG("~ConnectFirst::ConnectFirst()");
}


template <class CFG, class WEIGHT>
ConnectFirst<CFG,WEIGHT>::
~ConnectFirst() { 
}


template <class CFG, class WEIGHT>
void ConnectFirst<CFG,WEIGHT>::
SetDefault() {
  kfirst = 1;
}


template <class CFG, class WEIGHT>
void ConnectFirst<CFG,WEIGHT>::
ParseXML(XMLNodeReader& in_Node)
{
  NodeConnectionMethod<CFG,WEIGHT>::ParseXML(in_Node);
  kfirst = in_Node.numberXMLParameter(string("k"), true, 1, 1, 1000, string("connect first k value"));
}

template <class CFG, class WEIGHT>
void
ConnectFirst<CFG, WEIGHT>::
PrintUsage(ostream& _os) {
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << this->GetName() << " ";
  _os << "\tINTEGER (default " << 1 << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
ConnectFirst<CFG, WEIGHT>::
PrintValues(ostream& _os) {
  _os << "\n" << this->GetName() << " kfirst = ";
  _os << kfirst;
  _os << endl;
}


template <class CFG, class WEIGHT>
void
ConnectFirst<CFG, WEIGHT>::
PrintOptions(ostream& out_os)
{
  NodeConnectionMethod<CFG,WEIGHT>::PrintOptions(out_os);
  out_os << "      kfirst = " << kfirst << endl;
}

template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
ConnectFirst<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = 
           new ConnectFirst<CFG,WEIGHT>(*this);
  return _copy;
}


template <class CFG, class WEIGHT>
void 
ConnectFirst<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        DistanceMetric* dm,
	LocalPlanners<CFG,WEIGHT>* lp,
	bool addPartialEdge, bool addAllEdges) {
  vector<VID> cfgs;
  _rm->m_pRoadmap->GetVerticesVID(cfgs);
  return Connect(_rm, Stats, dm, lp, addPartialEdge, addAllEdges,
		 cfgs, cfgs);
}


template <class CFG, class WEIGHT>
void 
ConnectFirst<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
	DistanceMetric* dm,
	LocalPlanners<CFG,WEIGHT>* lp,
	bool addPartialEdge, bool addAllEdges,
	vector<VID>& cfgs1, vector<VID>& cfgs2) {
#ifndef QUIET
  cout << "connectfirst(k=" << kfirst << "): " << flush;
#endif

  typename vector<VID>::iterator C1, C2;
  typename vector<pair<VID,double> >::iterator D;
  for(C1 = cfgs1.begin(); C1 != cfgs1.end(); ++C1) {
    // sort cfgs2 in by distance from C
    vector<pair<VID,double> > distances;
    for(C2 = cfgs2.begin(); C2 != cfgs2.end(); ++C2)
      distances.push_back(make_pair(*C2, dm->Distance(_rm->GetEnvironment(), 
						      _rm->m_pRoadmap->GetData(*C1), 
						      _rm->m_pRoadmap->GetData(*C2))));
    sort(distances.begin(), distances.end(), CfgDist_Compare<VID>());

    //try to connect, return after failure or connect first k
    int numFound = 0;
    for(D = distances.begin(); (D != distances.end()) && (numFound < kfirst); 
	++D) {
      if(_rm->m_pRoadmap->IsEdge(*C1, D->first))
        continue;
      if(this->m_CheckIfSameCC && IsSameCC(*(_rm->m_pRoadmap), *C1, D->first))
        continue;
      LPOutput<CFG,WEIGHT> _ci;
      if(lp->IsConnected(_rm->GetEnvironment(), Stats, dm, 
			 _rm->m_pRoadmap->GetData(*C1), 
			 _rm->m_pRoadmap->GetData(D->first), 
			 &_ci, _rm->GetEnvironment()->GetPositionRes(), 
			 _rm->GetEnvironment()->GetOrientationRes(), 
			 true, true)) {
	_rm->m_pRoadmap->AddEdge(*C1, D->first, _ci.edge);
	numFound++;
      } 
    }
  }
}


#endif
