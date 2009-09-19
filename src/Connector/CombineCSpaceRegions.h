/**@file CombineCSpaceRegions.h
 * Classes to combine C-space regions after being solved separately
 * Marco Morales
 */

#ifndef COMBINECSPACEREGIONS_H
#define COMBINECSPACEREGIONS_H



template <class CFG>
class CombinationMethod {
 public:
  CombinationMethod();
  virtual ~CombinationMethod();

  // Access
  virtual char* GetName() = 0;
  virtual void SetDefault();
  //I/O methods
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;
  virtual CombinationMethod<CFG>* CreateCopy() = 0;

  virtual void CombineRegions(Environment* _env, 
			      Stat_Class& Stats,
			      BoundingBox &c_boundary,
			      vector<CFG>& free_nodes,
			      vector<CFG>& coll_nodes) = 0;


 protected:

};


template <class CFG>
CombinationMethod<CFG>::
CombinationMethod() {
  SetDefault();
}

template <class CFG>
CombinationMethod<CFG>::
~CombinationMethod() {
}

template <class CFG>
void CombinationMethod<CFG>::
SetDefault() {
  //set default values for partitioning parameters
}

template <class CFG>
class BruteForceCombination: public CombinationMethod<CFG> {
 public:
  BruteForceCombination();
  ~BruteForceCombination();
  char* GetName();
  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual CombinationMethod<CFG>* CreateCopy();

  virtual void CombineRegions(Environment* _env, 
					      Stat_Class& Stats,
					      BoundingBox &c_boundary,
					      vector<CFG>& free_nodes, 
					      vector<CFG>& coll_nodes);


};


template <class CFG>
BruteForceCombination<CFG>::
BruteForceCombination() : CombinationMethod<CFG>() {
}

template <class CFG>
BruteForceCombination<CFG>::
~BruteForceCombination() {
}

template <class CFG>
char*
BruteForceCombination<CFG>::
GetName() {
  return "BruteForceCombination";
}


template <class CFG>
void
BruteForceCombination<CFG>::
PrintUsage(ostream& _os) {
  _os.setf(ios::left,ios::adjustfield);
  _os << "\n" << GetName() << " ";
  _os.setf(ios::right,ios::adjustfield);
}

template <class CFG>
void 
BruteForceCombination<CFG>::
PrintValues(ostream& _os) {
  _os << "\n" << GetName() << " ";
  _os << endl;
}

template <class CFG>
PartitioningMethod<CFG>*
BruteForceCombination<CFG>::
CreateCopy() {
  CombinationMethod<CFG> * _copy = new BruteForceCombination<CFG>(*this);
  return _copy;
}

template <class CFG>
void BruteForceCombination<CFG>::
CombineRegions(Environment* _env, Stat_Class& Stats,
	       MPRegion<> l_region, MPRegion<> r_region) {

  cout << "get selected vertices from left map " << endl;
  cout << "get selected vertices from right map " << endl;


  cout << "make connections (as in RRTexpand:ModifyRoadmap) " << endl;
  cout << "result is left in inputMap1 after calling ConnectComponents" << endl;
  cout << "Basic (BruteForce): main_integrate_2_maps.serial.cpp:307-326" << endl;

  int DOF_split_point = l_boundary.FindSplitParameter(r_boundary);
  bool anglewrap = l_boundary.IfWrap(DOF_split_point) || r_boundary.IfWrap(DOF_split_point);
  cout << "Split parameter: " << DOF_split_point << "; Wrap?: " << anglewrap << endl;


/*   vector<VID> inputMap1_vertices; */
/*   vector<CfgType> inputMap1_selectedVertices; */
/*   vector<CfgType> inputMap1_selectedVertices2; */
/*   inputMap1.m_pRoadmap->GetVerticesVID( inputMap1_vertices ); */
  
/*   vector<VID> inputMap2_vertices; */
/*   vector<CfgType> inputMap2_selectedVertices; */
/*   vector<CfgType> inputMap2_selectedVertices2; */
/*   inputMap2.m_pRoadmap->GetVerticesVID( inputMap2_vertices ); */
  
/*   vector<VID> inputMap2_newVids; */
/*   RRTexpand<CfgType,WeightType>* rrtexp = new RRTexpand<CfgType, WeightType> (); */
  
/*   inputMap2_newVids = rrtexp->ModifyRoadMap(&inputMap1, &inputMap2,  */
/* 						inputMap2_vertices); */
    

  cout << "Overlap: main_integrate_2_maps.serial.cpp:210-306" << endl;
  cout << "probability (Overlap+Random): main_integrate_2_maps.serial.cpp:327-570?" << endl;


}

template <class CFG>
class CombineCSpaceRegions {
 public:
  CombineCSpaceRegions();
  ~CombineCSpaceRegions();

  static vector<CombinationMethod<CFG>*> GetDefault();

  void PrintUsage(ostream& _os);
  void PrintValues(ostream& _os);
  void PrintDefaults(ostream& _os);

  /**Generate nodes according to those in selected vector.
   *@param _rm New created nodes will be added to this roadmap if addNodes@Map=true.
   *@param nodes New created nodes are stored here.
   */
  template <class WEIGHT>
  void CombineRegions(Roadmap<CFG, WEIGHT>* _rm, 
				      Stat_Class& Stats,
				      BoundingBox &c_boundary,
				      vector< vector<CFG> >& nodes);

 protected:
  //////////////////////
  // Data
  vector<CombinationMethod<CFG>*> all;

 public:
  vector<CombinationMethod<CFG>*> selected;


};



template <class CFG>
CombineCSpaceRegions<CFG>::
CombineCSpaceRegions() {
  BruteForceCombination<CFG>* brute_force_combination = new BruteForceCombination<CFG>();
  all.push_back(brute_force_combination);

/*   GapPartitioning<CFG>* gap_partitioning = new GapPartitioning<CFG>(); */
/*   all.push_back(gap_partitioning); */

/*   InformationGainPartitioning<CFG>* information_gain_partitioning = new InformationGainPartitioning<CFG>(); */
/*   all.push_back(information_gain_partitioning); */
}

template <class CFG>
CombineCSpaceRegions<CFG>::
~CombineCSpaceRegions() {
  typename vector<CombinationMethod<CFG>*>::iterator I;
  for (I=selected.begin(); I!=selected.end(); I++)
    delete *I;

  for (I=all.begin(); I!=all.end(); I++)
    delete *I;
}

template <class CFG>
vector<CombinationMethod<CFG>*> 
CombineCSpaceRegions<CFG>::
GetDefault() {
  vector<CombinationMethod<CFG>*> Default;
  BruteForceCombination<CFG>* brute_force_combination= new BruteForceCombination<CFG>();
  Default.push_back(brute_force_combination);
  return Default;
}


template <class CFG>
void 
CombineCSpaceRegions<CFG>::
PrintUsage(ostream& _os) {
  typename vector<CombinationMethod<CFG>*>::iterator I;
  for(I=all.begin(); I!=all.end(); I++)
    (*I)->PrintUsage(_os);
}


template <class CFG>
void 
CombineCSpaceRegions<CFG>::
PrintValues(ostream& _os) {
  typename vector<CombinationMethod<CFG>*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    (*I)->PrintValues(_os);
}


template <class CFG>
void
CombineCSpaceRegions<CFG>::
PrintDefaults(ostream& _os) {
  vector<CombinationMethod<CFG>*> Default;
  Default = GetDefault();
  typename vector<CombinationMethod<CFG>*>::iterator I;
  for(I=Default.begin(); I!=Default.end(); I++)
    (*I)->PrintValues(_os);
}


template <class CFG>
template <class WEIGHT>
void
CombineCSpaceRegions<CFG>::
CombineRegions(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		BoundingBox &c_boundary,
		vector< vector<CFG> >& nodes) {

  cout << "------Starting Combinations-------" << endl;
 
  typename vector<PartitioningMethod<CFG>*>::iterator itr;


  cout << " Find splitting points: Internal BBox " << endl;


  for ( itr = selected.begin(); itr != selected.end(); itr++ ) {
#ifndef QUIET	
    Clock_Class clock;
    clock.StartClock((*itr)->GetName());
    cout<<"\n  "; clock.PrintName(); cout << " " << flush;
#endif	
    
    (*itr)->CombineRegions(_rm->GetEnvironment(), Stats, c_boundary, free_nodes, coll_nodes);
#ifndef QUIET
    clock.StopClock();
    cout << clock.GetClock_SEC() << " sec  \n" << flush;
#endif
  }

  cout << "------Stopping Combinations-------" << endl;

};


/* ORIGINAL METHOD FROM MPREGION, NEED TO ADAPT IN ABOVE
/* // Combines Roadmaps assuming the subregions in vectors are ordered so that  */
/* // neighboring regions are contiguous in the vector */
/* template <class CFG, class WEIGHT> */
/* 				  //Roadmap<CFG,WEIGHT>  */
/* void MPRegion<CFG,WEIGHT>:: */
/* CombineRegions(vector< MPRegion<CFG,WEIGHT>*> &subregions) { */

/*   // let's start with the simplest method, later we will need to make it better. */
/*   cout << "MPRegion::CombineRoadmaps(). TODO ALL " << endl; */
  
/*   Roadmap<CFG,WEIGHT> combined_roadmap; */
/*   CombineCSpaceRegions<CFG,WEIGHT> integrator; */
/*   integrator.ReadCommandLine(input.integrationType); */

/*   // Combine roadmaps */
/*   cout << "\t combining regions" << endl; */
/*   vector< MPRegion<CFG,WEIGHT>* >::iterator sbrg_itr; */
/*   MPRegion<CFG,WEIGHT>* merged_region = *(subregions.begin()); */
/*   for (sbrg_itr = subregions.begin()+1; sbrg_itr < subregions.end(); sbrg_itr++) { */
/*     //merged_region =  */
/*       integrator.CombineRegions(*(merged_region), *(*sbrg_itr),combine_stats, */
/* 				cm_combine, */
/* 				cd,dm,lp_map, */
/* 				false/\*input.addPartialEdge.GetValue()*\/, */
/* 				false/\*input.addAllEdges.GetValue()*\/); */
/*   } */
  
/*   // Combine merged region to feature roadmap of parent */
/*   cout << "\t combine this with parent roadmap " << endl; */
/*   //  return merged_region.roadmap; */
/* } */

#endif /* COMBINECSPACEREGIONS_H */
