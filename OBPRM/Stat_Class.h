#ifndef StatClass_h
#define StatClass_h

#include "Roadmap.h"


// Maximum number of connected components to keep track of
const int MaxCC=    100;

// Maximum number of local planners to keeps stats on
const int MaxLP=    10;

// Maximum number of collision detection algorithms to keep stats on
const int MaxCD=    10;

/**@file Stat_Class.h
   This class lets you keep statistics on various aspects of
   the program (ie. number of collision detection calls,
   number of calls to each local planner, etc).

   @date 1/27/99
   @author Chris Jones 
*/

class Stat_Class {
public:
  Stat_Class();
  ~Stat_Class();

  void ClearStats();
  int IncNumNodes();
  int SetNumNodes( int Nodes );
  int IncNumEdges();
  int SetNumEdges( int Edges );

  int IncNumCC();
  int SetNumCC( int CC );
  int IncSizeCC( int CC );
  int SetSizeCC( int CC, int Size );

  int IncNumCollDetCalls( char *CDName , std::string *pCallName = NULL);

  void IncCfgIsColl( std::string *pCallName = NULL);


  int FindCD( char *CDName );

  int FindLP( char *LPName );
  int IncLPConnections( char *LPName , int);
  int IncLPConnections( char *LPName );
  int DecLPConnections( char *LPName , int);
  int DecLPConnections( char *LPName );
  int SetLPConnections( char *LPName, int Connections );
  int IncLPAttempts( char *LPName ,int );
  int IncLPAttempts( char *LPName );
  int DecLPAttempts( char *LPName ,int );
  int DecLPAttempts( char *LPName );
  int SetLPAttempts( char *LPName, int Attempts );
  int IncLPCollDetCalls( char *LPName );
  int IncLPCollDetCalls( char *LPName ,int);
  int DecLPCollDetCalls( char *LPName ); 
  int DecLPCollDetCalls( char *LPName ,int); 


  static const int ALL;
  template <class CFG, class WEIGHT>
  void PrintAllStats( Roadmap<CFG, WEIGHT> *rmap);
  template <class CFG, class WEIGHT>
  void PrintAllStats( Roadmap<CFG, WEIGHT> *rmap, int numCCs);

  template <class CFG, class WEIGHT>
  void PrintDataLine(ostream&, Roadmap<CFG, WEIGHT>* , int show_column_headers=0);
  void PrintParams();

  template <class CFG, class WEIGHT>
  void ComputeIntraCCFeatures(ConnectMap<CFG, WEIGHT>* cm, Roadmap<CFG,WEIGHT> *rdmp, DistanceMetric * dm);

  template <class CFG, class WEIGHT>
  void ComputeInterCCFeatures(ConnectMap<CFG, WEIGHT>* cm, Roadmap<CFG,WEIGHT> *rdmp, DistanceMetric * dm);
  void PrintFeatures();
  void IncNodes_Generated();
  void IncNodes_Attempted();
  void IncConnections_Attempted();
  void IncConnections_Made();


  //features
  int Connections_Attempted;
  int Connections_Made;
  int Nodes_Attempted;
  int Nodes_Generated;
  int cc_number;

  //Intra-CC features:
  double avg_min_intracc_dist;
  double avg_max_intracc_dist;
  double avg_mean_intracc_dist;
  double avg_sigma_intracc_dist;

  double avg_min_intracc_edge_s;
  double avg_max_intracc_edge_s;
  double avg_mean_intracc_edge_s;
  double avg_sigma_intracc_edge_s;

  double avg_max_intracc_dist_to_cm;
  double avg_min_intracc_dist_to_cm;
  double avg_mean_intracc_dist_to_cm;
  double avg_sigma_intracc_dist_to_cm;

  //Inter-CC features:
  double max_intercc_dist;
  double avg_intercc_dist; 
  double sigma_intercc_dist;
  double min_intercc_dist;

  double max_cc_size;
  double min_cc_size;
  double avg_cc_size;
  double sigma_cc_size;


protected:
  int NumNodes;
  int NumEdges;
  int NumCC;

  unsigned long int NumCollDetCalls[MaxCD];
  char CDNameList[MaxCD][50];

  int SizeCC[MaxCC];

  char LPNameList[MaxLP][50];
  unsigned long int LPConnections[MaxLP];
  unsigned long int LPAttempts[MaxLP];
  unsigned long int LPCollDetCalls[MaxLP];

  std::map<std::string, unsigned long int> CollDetCountByName;

  ///IsColl simply counts the number of times a Cfg is tested for Collision.
  ///\see Cfg::isCollision
  std::map<std::string, unsigned long int> IsCollByName;
  unsigned long int IsCollTotal;
};

//definitions of templated functions
template <class CFG, class WEIGHT>
void
Stat_Class::
PrintAllStats( Roadmap<CFG, WEIGHT>* rmap) {
  PrintAllStats(rmap, ALL);
}

template <class CFG, class WEIGHT>
void
Stat_Class::
PrintAllStats( Roadmap<CFG, WEIGHT>* rmap, int numCCs) {
  int i;
  std::map<std::string, unsigned long int>::const_iterator iter;
  int total=0;

  cout << endl << endl << "Local Planners:" << endl;
  cout << setw(20) << "Name"
	<<setw(15) << "Connections"
	<<setw(15) << "Attempts"
	<<setw(15) << "Coll Det Calls" << endl;
 
  for(i=0;i<MaxLP;i++)
    if (strcmp(LPNameList[i],"empty")!=0) {
      cout << setw(20) << LPNameList[i];
      cout << setw(15) << LPConnections[i];
      cout << setw(15) << LPAttempts[i];
      cout << setw(15) << LPCollDetCalls[i] << endl;
  }

  cout << endl << endl;
  cout << "Number of Nodes: " << rmap->m_pRoadmap->GetVertexCount() << endl;
  cout << "Number of Edges: " << rmap->m_pRoadmap->GetEdgeCount() << endl;
/*  Removed by Roger for reasons given below
  cout << "Number of Collision Detection Calls: " << endl;
  for(i=0;i<MaxCD;i++)
    if (strcmp(CDNameList[i],"empty")!=0)
      cout << setw(20) << CDNameList[i] 
	   << setw(15) << NumCollDetCalls[i] << endl;
*/

  #if VERBOSE
  #endif

  cout << endl;

  if (numCCs==ALL)    {DisplayCCStats(*(rmap->m_pRoadmap));      }
  else if (numCCs==0) {DisplayCCStats(*(rmap->m_pRoadmap),0);     }
  else                {DisplayCCStats(*(rmap->m_pRoadmap),numCCs);}


  ///Below removed b/c it counts Coll Detection too fine grained.  We have decided 
  ///to only keep Total times Cfg::isCollision is called.  This makes the 'price' for a 
  ///free node the same as a collision node.  Will be added back after collision detection
  ///counting is properly fixed     --Roger 9/17/2005
/*
  cout << endl << endl << "Collision Detection Exact Counts:" << endl;
  for (i=0, iter=CollDetCountByName.begin(); iter != CollDetCountByName.end(); iter++, i++) 
  {
    total+=iter->second;
    cout << i << ") " << iter->second << " ";
    cout << flush;
    printf("%s\n", iter->first.data()); //K2 does not have the << operator defined for string
    fflush(stdout);
    }

  cout << "total " << total << endl;
  */

  cout << endl << endl << "Cfg::isCollision() Exact Counts:" << endl;
  for (i=0, iter=IsCollByName.begin(); iter != IsCollByName.end(); iter++, i++) 
  {
    cout << i << ") " << iter->second << " " << iter->first << endl;;

  }
  cout << "Total Cfg::isCollision() = " << IsCollTotal << endl << endl;

}

template <class CFG, class WEIGHT>
void
Stat_Class::
PrintDataLine(ostream& _myostream, Roadmap<CFG, WEIGHT> *rmap, int show_column_headers) {

   // Default is to NOT print out column headers
   if (show_column_headers){
        _myostream <<"\nV  E #CC 1 2 3 4 5 6 7 8 9 10 #iso CD  LPattSum LPcdSum\n";
   }//endif

   _myostream << rmap->m_pRoadmap->GetVertexCount() << " ";
   _myostream << rmap->m_pRoadmap->GetEdgeCount()   << " ";

   vector< pair<int,VID> > ccstats;
   GetCCStats(*(rmap->m_pRoadmap),ccstats);
   _myostream << ccstats.size() << "  ";
   int i;
   for (i=0;i<10;++i)
      if (ccstats.size() > i)
        _myostream << ccstats[i].first << " ";
      else
        _myostream << 0                << " ";

   int sumIsolatedNodes=0;
   for (i=0;i<ccstats.size();++i)
      if (ccstats[i].first == 1) ++sumIsolatedNodes;
   _myostream << sumIsolatedNodes << " ";

   int sumCDcalls=0;
   for(i=0;i<MaxCD;i++)
     if (strcmp(CDNameList[i],"empty")!=0)
        sumCDcalls += NumCollDetCalls[i];
   _myostream << sumCDcalls << " ";

   int sumAtt=0;
   int sumCD =0;
   for(i=0;i<MaxLP;i++){
     if (strcmp(LPNameList[i],"empty")!=0) {
       sumAtt += LPAttempts[i];
       sumCD  += LPCollDetCalls[i];
     }//endif
   }//endfor
   _myostream << sumAtt << " ";
   _myostream << sumCD  << " ";
   ccstats.clear();
}


// Compute intra-connected-component statistics
template <class CFG, class WEIGHT>
void
Stat_Class::
ComputeIntraCCFeatures(ConnectMap<CFG, WEIGHT>* cm, Roadmap<CFG,WEIGHT> * rdmp, DistanceMetric * dm) {
  avg_min_intracc_dist = 0;
  avg_max_intracc_dist = 0;
  avg_mean_intracc_dist = 0;
  avg_sigma_intracc_dist = 0;

  avg_min_intracc_edge_s = 0;
  avg_max_intracc_edge_s = 0;
  avg_mean_intracc_edge_s = 0;
  avg_sigma_intracc_edge_s = 0;

  avg_mean_intracc_dist_to_cm = 0;

  vector< pair<int,VID> > ccs; //vector of connected components in the roadmap
  GetCCStats(*(rdmp->m_pRoadmap), ccs);//fill ccs from the roadmap
  cout << "in intra ccs portion" << endl;

  vector< pair<int,VID> >::iterator cci; // cci is CC[i] hereafter
  for (cci = ccs.begin(); cci < ccs.end(); cci++) {
    vector<CFG> cci_cfgs; //configurations in cci
    CFG cci_tmp = rdmp->m_pRoadmap->GetData(cci->second);//cci->second: vertex ID of first node in cci
    GetCC(*(rdmp->m_pRoadmap), cci_tmp, cci_cfgs); //fill cci_cfgs

    if (cci_cfgs.size() > 1) {
      //compute shortest, longest, mean, and std-dev (sigma) distances 
      //between nodes for cci
    
      double cci_min_intracc_dist = 0;
      double cci_max_intracc_dist = 0;
      double cci_mean_intracc_dist = 0;
      double n_pairs = 0;
      typename vector<CFG>::iterator cci_i;
      typename vector<CFG>::iterator cci_j;
      for (cci_i=cci_cfgs.begin(); cci_i < cci_cfgs.end(); cci_i++) {
	for (cci_j = cci_i+1; cci_j < cci_cfgs.end(); cci_j++) {
	  double c_dist = dm->Distance(rdmp->GetEnvironment(), *cci_i, *cci_j);
	  if (c_dist > cci_max_intracc_dist)
	    cci_max_intracc_dist = c_dist;
	  if (cci_min_intracc_dist == 0 || c_dist < cci_min_intracc_dist)
	    cci_min_intracc_dist = c_dist;
	  cci_mean_intracc_dist += c_dist;
	  n_pairs++;
	}
      }
      if (n_pairs > 0)
	cci_mean_intracc_dist /= n_pairs;

      double cci_sigma_intracc_dist = 0;
      for (cci_i=cci_cfgs.begin(); cci_i < cci_cfgs.end(); cci_i++) {
	for (cci_j = cci_i+1; cci_j < cci_cfgs.end(); cci_j++) {
	  cci_sigma_intracc_dist += pow(dm->Distance(rdmp->GetEnvironment(),
		     *cci_i, *cci_j)-cci_mean_intracc_dist, 2);
	}
      }
      if (n_pairs > 1)
	cci_sigma_intracc_dist /= (n_pairs-1);
      cci_sigma_intracc_dist = sqrt(cci_sigma_intracc_dist);

      //compute shortest, longest, mean and std-dev (sigma) edge sizes 
      //in the component
      vector<pair<VID,VID> > cci_edges;
      GetCCEdges(*(rdmp->m_pRoadmap), cci_edges, cci->second);
      double cci_max_intracc_edge_s = 0;
      double cci_min_intracc_edge_s = 0;
      double cci_mean_intracc_edge_s = 0;
      double cci_sigma_intracc_edge_s = 0;
      for (vector< pair<VID,VID> >::iterator e_iter = cci_edges.begin();
	   e_iter < cci_edges.end(); e_iter++) {
	CFG e_v1 = rdmp->m_pRoadmap->GetData(e_iter->first);
	CFG e_v2 = rdmp->m_pRoadmap->GetData(e_iter->second);
	double e_dist = dm->Distance(rdmp->GetEnvironment(), e_v1, e_v2);
	if (e_dist > cci_max_intracc_edge_s)
	  cci_max_intracc_edge_s = e_dist;
	if (cci_min_intracc_edge_s == 0 || e_dist < cci_min_intracc_edge_s)
	  cci_min_intracc_edge_s = e_dist;
	cci_mean_intracc_edge_s += e_dist;
      }
      if (cci_edges.size() > 0)
	cci_mean_intracc_edge_s /= cci_edges.size();
      for (vector< pair<VID,VID> >::iterator e_iter = cci_edges.begin();
	   e_iter < cci_edges.end(); e_iter++) {
	CFG e_v1 = rdmp->m_pRoadmap->GetData(e_iter->first);
	CFG e_v2 = rdmp->m_pRoadmap->GetData(e_iter->second);
	double e_dist = dm->Distance(rdmp->GetEnvironment(), e_v1, e_v2);
	cci_sigma_intracc_edge_s += pow(e_dist-cci_mean_intracc_edge_s, 2);
      }
      if (cci_edges.size() > 1)
	cci_sigma_intracc_edge_s /= cci_edges.size() - 1;
      cci_sigma_intracc_edge_s = sqrt(cci_sigma_intracc_edge_s);

      //get center of mass of all Cfgs in current CC
      CFG center_of_mass = cci_cfgs[0];
      for (int j = 1; j < cci_cfgs.size(); ++j)
	center_of_mass.add(center_of_mass, cci_cfgs[j]);
      center_of_mass.divide(center_of_mass, cci_cfgs.size());
    
      //compute average distance of the nodes to the center of mass
      double cci_min_intracc_dist_to_cm = 0;
      double cci_max_intracc_dist_to_cm = 0;
      double cci_mean_intracc_dist_to_cm = 0;
      double cci_sigma_intracc_dist_to_cm = 0;
      for (int j = 0; j < cci_cfgs.size(); ++j) {
	CFG tmp = cci_cfgs[j];
	double e_dist = dm->Distance(rdmp->GetEnvironment(), 
				     center_of_mass, tmp);
	if (e_dist > cci_max_intracc_dist_to_cm)
	  cci_max_intracc_dist_to_cm = e_dist;
	if (cci_min_intracc_dist_to_cm == 0 || e_dist < cci_min_intracc_dist_to_cm)
	  cci_min_intracc_dist_to_cm = e_dist;
	cci_mean_intracc_dist_to_cm += e_dist;
      }
      if (cci_cfgs.size() > 0 )
	cci_mean_intracc_dist_to_cm /= cci_cfgs.size();
      for (int j = 0; j < cci_cfgs.size(); ++j) {
	CFG tmp = cci_cfgs[j];
	double e_dist = dm->Distance(rdmp->GetEnvironment(), 
				     center_of_mass, tmp);
	cci_sigma_intracc_dist_to_cm += pow(e_dist-cci_mean_intracc_dist_to_cm, 2);
      }
      if (cci_cfgs.size() > 1)
	cci_sigma_intracc_dist_to_cm /= cci_cfgs.size() - 1;
      cci_sigma_intracc_dist_to_cm = sqrt(cci_sigma_intracc_dist_to_cm);

      avg_min_intracc_dist += cci_min_intracc_dist;
      avg_max_intracc_dist += cci_max_intracc_dist;
      avg_mean_intracc_dist += cci_mean_intracc_dist;
      avg_sigma_intracc_dist += cci_sigma_intracc_dist;
    
      avg_min_intracc_edge_s += cci_min_intracc_edge_s;
      avg_max_intracc_edge_s += cci_max_intracc_edge_s;
      avg_mean_intracc_edge_s += cci_mean_intracc_edge_s;
      avg_sigma_intracc_edge_s += cci_sigma_intracc_edge_s;

      avg_max_intracc_dist_to_cm += cci_max_intracc_dist_to_cm;
      avg_min_intracc_dist_to_cm += cci_min_intracc_dist_to_cm;
      avg_mean_intracc_dist_to_cm += cci_mean_intracc_dist_to_cm;
      avg_sigma_intracc_dist_to_cm += cci_sigma_intracc_dist_to_cm;
    }
  }
  if (ccs.size() > 0) {
    //cout << "averaging" << endl;
    avg_min_intracc_dist /= ccs.size();
    avg_max_intracc_dist /= ccs.size();
    avg_mean_intracc_dist /= ccs.size();
    avg_sigma_intracc_dist /= ccs.size();
    
    avg_min_intracc_edge_s /= ccs.size();
    avg_max_intracc_edge_s /= ccs.size();
    avg_mean_intracc_edge_s /= ccs.size();
    avg_sigma_intracc_edge_s /= ccs.size(); 

    avg_max_intracc_dist_to_cm /= ccs.size();
    avg_min_intracc_dist_to_cm /= ccs.size();
    avg_mean_intracc_dist_to_cm /= ccs.size();
    avg_sigma_intracc_dist_to_cm /= ccs.size();

  }
  CFG tcfg;
  double norm = rdmp->GetEnvironment()->Getminmax_BodyAxisRange()*tcfg.DOF();
  avg_min_intracc_dist /= norm;
  avg_max_intracc_dist /= norm;
  avg_mean_intracc_dist /= norm;
  avg_sigma_intracc_dist /= norm;
  
  avg_min_intracc_edge_s /= norm;
  avg_max_intracc_edge_s /= norm;
  avg_mean_intracc_edge_s /= norm;
  avg_sigma_intracc_edge_s /= norm;
  
  avg_max_intracc_dist_to_cm /= norm;
  avg_min_intracc_dist_to_cm /= norm;
  avg_mean_intracc_dist_to_cm /= norm;
  avg_sigma_intracc_dist_to_cm /= norm;
}

// Compute inter-connected-component statistics
// Find Closest nodes between two ccs and find distance 
// between the closest pairs of ccs
template <class CFG, class WEIGHT>
void
Stat_Class::
ComputeInterCCFeatures(ConnectMap<CFG, WEIGHT>* cm, Roadmap<CFG,WEIGHT> * rdmp, DistanceMetric * dm) {
  vector< pair<int,VID> > ccs; //connected components in the roadmap
  cout << "in inter ccs portion" << endl;
  GetCCStats(*(rdmp->m_pRoadmap), ccs);//fill ccs

  max_intercc_dist = 0.0;
  avg_intercc_dist = 0.0; 
  sigma_intercc_dist = 0.0;
  min_intercc_dist = 0.0;

  int pairs_checked = 0;
  vector<double> min_cc_distance_between_closest_pairs;
  vector<pair<VID,VID> > ccedges;
  vector<double> ccsizes;

  max_cc_size = 0;
  min_cc_size = 0;
  avg_cc_size = 0;
  sigma_cc_size = 0;

  cc_number = ccs.size();

  double total_components_dist = 0;
  for(vector< pair<int,VID> >::iterator cce=ccs.begin(); cce < ccs.end(); cce++){
    GetCCEdges(*(rdmp->m_pRoadmap),ccedges,cce->second);
    double total_size = 0.0;
    cout << "size of edge list for cc:" << cce->second 
	 << " "<< ccedges.size() << endl;
    for(vector< pair<VID,VID> >::iterator cciter=ccedges.begin(); cciter<ccedges.end();cciter++) {
      CFG cciter_a = rdmp->m_pRoadmap->GetData(cciter->first);
      CFG cciter_b = rdmp->m_pRoadmap->GetData(cciter->second);
      double dist =dm->Distance(rdmp->GetEnvironment(),
				cciter_a, cciter_b);
      total_size += dist;
    }     
    ccedges.clear();
    total_size /= 2;
    ccsizes.push_back(total_size);
    if(total_size > max_cc_size) 
      max_cc_size=total_size;
    if(min_cc_size == 0 || total_size < min_cc_size) 
      min_cc_size=total_size;
    avg_cc_size+=total_size;
    total_components_dist+=total_size;
  }
  avg_cc_size /= ccs.size();

  sigma_cc_size = 0;
  for (int j = 0; j < ccsizes.size(); j++)
    sigma_cc_size  += pow(ccsizes[j]-avg_cc_size, 2);
  if (ccsizes.size() > 1)
    sigma_cc_size /= (ccsizes.size()-1);
  sigma_cc_size = sqrt(sigma_cc_size);
  if (total_components_dist>0) {
    min_cc_size/=total_components_dist;
    max_cc_size/=total_components_dist;
    avg_cc_size/=total_components_dist;
    sigma_cc_size/=total_components_dist;
  }


  vector< pair<int,VID> >::iterator cci; // cci is CC[i] hereafter
  if(ccs.size()>1)
    for (cci = ccs.begin(); cci < ccs.end(); cci++) {
      vector< pair<int,VID> >::iterator ccj;//ccj will be the rest of the ccs that cci has not
      //checked against
      ccj = cci;
      //ccj++;
      for(ccj++;ccj<ccs.end();ccj++) {
	
	vector<CFG> cci_cfgs; //configurations in cci
	vector<CFG> ccj_cfgs; //configurations in ccj
	CFG cci_tmp = rdmp->m_pRoadmap->GetData(cci->second);//cci->second: vertex ID of first node in cci
	CFG ccj_tmp = rdmp->m_pRoadmap->GetData(ccj->second);//ccj->second: vertex ID of first node in ccj

	GetCC(*(rdmp->m_pRoadmap), cci_tmp, cci_cfgs); //fill cci_cfgs
	GetCC(*(rdmp->m_pRoadmap), ccj_tmp, ccj_cfgs); //fill ccj_cfgs
	
	vector< pair<CFG,CFG> > pairs;
	pairs = dm->FindKClosestPairs(rdmp->GetEnvironment(), 
				      cci_cfgs, ccj_cfgs, 1); 
	double tmp_dist = dm->Distance(rdmp->GetEnvironment(),
				       pairs[0].first, pairs[0].second);
	if(tmp_dist > max_intercc_dist) 
	  max_intercc_dist = tmp_dist;
	if(min_intercc_dist == 0.0 || tmp_dist < min_intercc_dist) 
	  min_intercc_dist = tmp_dist;
	avg_intercc_dist += tmp_dist;
	min_cc_distance_between_closest_pairs.push_back( tmp_dist );
	pairs_checked++;
      }
    }
  if (pairs_checked > 0)
    avg_intercc_dist /= pairs_checked;
  for (int j = 0; j < min_cc_distance_between_closest_pairs.size(); j++)
    sigma_intercc_dist += pow(min_cc_distance_between_closest_pairs[j]-
				avg_intercc_dist , 2);
  if (min_cc_distance_between_closest_pairs.size() > 1)
    sigma_intercc_dist /= min_cc_distance_between_closest_pairs.size() - 1;
  sigma_intercc_dist = sqrt(sigma_intercc_dist);

  CFG tcfg;
  double norm = rdmp->GetEnvironment()->Getminmax_BodyAxisRange()*tcfg.DOF();
  max_intercc_dist /= norm;
  min_intercc_dist /= norm;
  avg_intercc_dist /= norm;
  sigma_intercc_dist /= norm;
}
#endif

