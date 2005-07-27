#if !defined(_MAP_GENERATOR_H_)
#define _MAP_GENERATOR_H_

#include "Clock_Class.h"
#include "GenerateMapNodes.h"
#include "ConnectMap.h"
#include "MapEvaluator.h"

/////////////////////////////////////////////////////////////
//
//    Map Generator
//
/////////////////////////////////////////////////////////////

template <class CFG, class WEIGHT>
class MapGenerator {
 public:
 
  /////////////////////////////////
  // Constructor and Destructor
  /////////////////////////////////
  
  MapGenerator(int num_rep = 5);
  ~MapGenerator();

  void SetEvaluator(const MapEvaluator<CFG,WEIGHT>& eval) {
    stop = eval;
  }

  //main wrapper, called by main_obprm.cpp
  //if input.seedByChunk, calls GenerateIncrementalMap()
  //otherwise, calls GenerateNormalMap
  void GenerateMap(Roadmap<CFG, WEIGHT>* rmap, Stat_Class& Stats,
		     CollisionDetection* cd, 
		     DistanceMetric* dm, vector<CFG>& nodes, 
		     LocalPlanners<CFG,WEIGHT>* lp,
		     Input* input); 
  protected:
  
  virtual void GenerateIncrementalMap(Roadmap<CFG, WEIGHT>* rmap, Stat_Class& Stats,
		     CollisionDetection* cd, 
		     DistanceMetric* dm, vector<CFG>& nodes, 
		     LocalPlanners<CFG,WEIGHT>* lp,
		     Input* input,
		     bool addPartialEdge,
		     bool addAllEdges); 

  void GenerateNormalMap(Roadmap<CFG, WEIGHT>* rmap, Stat_Class& Stats,
		     CollisionDetection* cd, 
		     DistanceMetric* dm, vector<CFG>& nodes, 
		     LocalPlanners<CFG,WEIGHT>* lp,
		     Input* input,
		     bool addPartialEdge,
		     bool addAllEdges); 
  //Update NextNodeIndex from a map
  //It goes through all methods recorded in the map and update nextNodeIndex for each method
  //it also returns RNGseed
  void SetupFromMap(Input* input, Roadmap<CFG, WEIGHT>* rmap);
		     
  //////////////////////
  //Data
  //////////////////////
 
  public:

  int numRepNodesPerCC;          //number of representative nodes per connectted component
  GenerateMapNodes<CFG> gn;     //used to generate map nodes
  ConnectMap<CFG, WEIGHT> cm;   //used to connect map nodes
  GenerateMapNodes<CFG> old_gn; //used to parse command line from the existing map
  MapEvaluator<CFG,WEIGHT> stop;
};

template <class CFG, class WEIGHT>
MapGenerator<CFG, WEIGHT>::
MapGenerator(int num_rep){
  if(num_rep < 0) {
    cout<<"in MapGenerator<CFG, WEIGHT>::MapGenerator(int), the argument should not be negative."<<endl;
    exit(-1);
  }
  numRepNodesPerCC = num_rep;
}

template <class CFG, class WEIGHT>
MapGenerator<CFG, WEIGHT>::
~MapGenerator(){}


/*
 Set NextNodeIndex for each method we already used
 Set RNGseed for rmap
 Seed RNG using RNGseed, call OBPRM_srand()
 Update current command line 
 Backup the old map
*/
template <class CFG, class WEIGHT>
void
MapGenerator<CFG, WEIGHT>::
SetupFromMap(Input* input, Roadmap<CFG, WEIGHT>* rmap) 
{
    //get info from the existing map
    ifstream  myifstream(input->inmapFile.GetValue());
    long RNGseed;
    std::string tagstring;
    std::string oldCommandLine;

    getline(myifstream, tagstring);  //skip the first line
    getline(myifstream, tagstring);  //read in the second line 
    if (tagstring.find("PREAMBLESTART") == string::npos ) {
      cout << endl << "In ReadMapFile: didn't read PREAMBLESTART tag right";
      exit (-1);
    }
    getline(myifstream, oldCommandLine);  //read in the third line: old command line
    getline(myifstream, tagstring);       //read in the fourth line to double check
    if (tagstring.find("PREAMBLESTOP") == string::npos ) {
      cout << endl << "In ReadMapFile: didn't read PREAMBLESTOP tag right";
      exit (-1);
    }

    // skip over stuff up to DMSTOP, next line should contain RNGSEEDSTART
    bool moreFile=true;
    while(moreFile) {
      getline(myifstream, tagstring); 
      if (tagstring.find("DMSTOP") != string::npos ) 
        moreFile = false;
    } 
    
    getline(myifstream, tagstring); //Get the line after DMSTOP
    if (tagstring.find("RNGSEEDSTART") == string::npos ) {
      cout << endl << "WARNNING: In ReadMapFile, didn't read RNGSEEDSTART tag right";
      RNGseed = INVALID_RNGSEED;               //set default value for old version map
    } 
    else {
      myifstream >> RNGseed;        //read in RNGseed from the map
      cout<<"Read RNGseed from existing map file and it is: "<<RNGseed<<endl;
    }
    myifstream.close();
   
    
    //update data member in rmap
    rmap->SetRNGseed(RNGseed);
    //reset seed for OBPRM_srand
    if(RNGseed == INVALID_RNGSEED)
      OBPRM_srand(); //seed RNG using the default value
    else
      OBPRM_srand(RNGseed); //seed RNG using this seed

    //set NextNodeIndex for each method used in the existing map
    vector<std::string> gens;  //including "-gNodes"
     
    std::string genStr = "-gNodes";
  
    int current = oldCommandLine.find(genStr, 0);
    int next;
    while (current != std::string::npos) {
      next = oldCommandLine.find('-', current +1);
      if (next == std::string::npos) {
        gens.push_back(oldCommandLine.substr(current,oldCommandLine.size()));
        break;
      }
      else
       gens.push_back(oldCommandLine.substr(current, next-current));
       current = oldCommandLine.find(genStr, next);

    }
     
     int numOldGen = gens.size();
     n_str_param      *GNstrings[numOldGen];
     
     for(int i=0; i<numOldGen; i++) { 
       std::string tmpString = gens[i].substr(8,gens[i].size()) ; //get rid of "-gNodes "
       char *tmp = (char*) tmpString.c_str() ;
       GNstrings[i]=new n_str_param("-gNodes", tmp);
     }
     
     old_gn.ReadCommandLine(GNstrings, gens.size());
     int totalNumNodes = 0;   
     typename vector<NodeGenerationMethod<CFG>*>::iterator itr;
     
     cout<<"Information for nodes in the existing roadmap:"<<endl;
     for ( itr = old_gn.selected.begin(); itr != old_gn.selected.end(); itr++ ) {
       cout<<"  The node generation method was: "<<(*itr)->GetName()<<endl;
       //cout<<"The nextNodeIndex is:"<<(*itr)->GetNextNodeIndex()<<endl;
       cout<<"  Number of nodes were generated: "<<(*itr)->numNodes.GetValue()<<endl;
       totalNumNodes += ((*itr)->numNodes.GetValue());
       (*itr)->IncreaseNextNodeIndex((*itr)->numNodes.GetValue());
       cout<<"  The nextNodeIndex is set to:"<<(*itr)->GetNextNodeIndex()<<endl;
     }

     //update commandLine in the input
     for(int i=0; i<gens.size(); i++) {
      std::string tmpString = gens[i];
      char *tmp = (char*) tmpString.c_str() ;
      strcat(input->commandLine, tmp);
    }

    //backup the existing map
    //file name is: "oldfileName.XXX" where XXX is totalNumNodes
    char outfile[200];
    sprintf(outfile, "%s%s%d", input->inmapFile.GetValue(), ".", totalNumNodes);

    // open infile and outfile
    ifstream  myifstream2(input->inmapFile.GetValue());
    if (!myifstream2) {
      cout << endl << "In SaveCurrentVersion: cannot open infile: " << input->inmapFile.GetValue();
      return;
    }
    ofstream  myofstream(outfile);
    if (!myofstream) {
      cout << endl << "In SaveCurrentVersion: cannot open outfile: " << outfile;
      return;
    }
  
    while ( getline(myifstream2, tagstring) ){
      myofstream << tagstring << endl;
    }
 
}

/*main interface/wrapper to generate a map
  if seedByChunk is set, generate/connect nodes chunk by chunk
    call GenerateIncrementalMap (this is the default option)
  otherwise, use the regular method to generate a map
    call GenerateNormalMap
 */
template <class CFG, class WEIGHT>
void 
MapGenerator<CFG, WEIGHT>::
GenerateMap(Roadmap<CFG, WEIGHT>* rmap, Stat_Class& Stats,
	     CollisionDetection* cd, 
	     DistanceMetric* dm, vector<CFG>& nodes, 
	     LocalPlanners<CFG,WEIGHT>* lp,
	     Input* input) {

  if(input->seedByChunk.GetValue()) 
    GenerateIncrementalMap(rmap,Stats,cd,dm,nodes,lp,input, 
			   input->addPartialEdge.GetValue(), 
			   input->addAllEdges.GetValue() );
  else
    GenerateNormalMap(rmap,Stats,cd,dm,nodes,lp,input,
		      input->addPartialEdge.GetValue(), 
		      input->addAllEdges.GetValue() );
} 


/*Incremental Map Generator
 1. if inmapFile is active,
      read in the existing map
 2. set up nextNodeIndex for node generation methods
 3. generate/connect new nodes chunk by chunk
 4. update command line before we output the final map
 */
template <class CFG, class WEIGHT>
void 
MapGenerator<CFG, WEIGHT>::
GenerateIncrementalMap(Roadmap<CFG, WEIGHT>* rmap, Stat_Class& Stats,
            CollisionDetection* cd, 
            DistanceMetric* dm, vector<CFG>& nodes, 
	    LocalPlanners<CFG,WEIGHT>* lp,
	    Input* input,
	    bool addPartialEdge,
	    bool addAllEdges){

  double totalGenTime = 0.0;  //all gen time for this map
  double totalConTime = 0.0;  //all con time for this map
  double totalEvalTime = 0.0; //all evaluation time for this map

  nodes.erase(nodes.begin(),nodes.end());

  /*
  If inmapFile.IsActivated {
    rmap->ReadRoadmapGRAPHONLY(input->inmapFile.GetValue()) to:
    SetupFromMap()
  }
  Generate/connect nodes according to command line
  */
   
  if ( input->inmapFile.IsActivated() ){
     
    //---------------------------
    // Read roadmap nodes
    //---------------------------
    rmap->ReadRoadmapGRAPHONLY(input->inmapFile.GetValue());

    SetupFromMap(input, rmap);
  } 
  
  bool firstChunk = true; // the first chunk for this map (no matter which gen method we use)

  bool finished;
  do {
    double expandGenTime = 0.0;
    double expandConTime = 0.0;

    typename vector<NodeGenerationMethod<CFG>*>::iterator itr;
    for ( itr = gn.selected.begin(); itr != gn.selected.end(); itr++ ) {

#ifndef QUIET
      cout<<endl<< (*itr)->GetName()<<endl;
#endif

      double genTime = 0.0;    //this selected method
      double conTime = 0.0;    //this selected method
      
      int oriNumNodes = (*itr)->numNodes.GetValue();
      int chunkSize =   (*itr)->chunkSize.GetValue();
      int numChunks = oriNumNodes/chunkSize;
      if( oriNumNodes > chunkSize * numChunks)
	numChunks ++;
      vector<CFG> sub_nodes;
      cout << "the chunk size is: " << chunkSize
	   << ", and the number of chunks is: " << numChunks << endl;
      vector<CFG> rep_cfgs;  //representative cfgs for existing nodes
      
      //generate nodes chunk by chunk and seed for each chunk
      //perform connection chunk by chunk:
      //  we use newly generated nodes and some prepresentativ nodes from the existing map.
      //  if numRepNodePerCC is set, we pick numRepNodePerCC nodes from each CC;
      //  if numRepNodePerCC is 0, we only use newly nodes.
      //  Note: the connection method should provide function to take a vector of nodes as an argument
      for (int i=0; i<numChunks; i++) {
	Clock_Class genClock;
	Clock_Class conClock;
	char genClockName[100], conClockName[100];      
	sprintf(genClockName, "%s%d%s", "Node generation time in chunk ", i, ": ");
	sprintf(conClockName, "%s%d%s", "Node connection time in chunk ", i, ": ");
	
	(*itr)->numNodes.PutValue (chunkSize);
	sub_nodes.clear();
	int nextNodeIndex = (*itr)->GetNextNodeIndex();
	cout<<endl<<"Chunk "<<i<<endl;
	OBPRM_srand((*itr)->GetName(), (*itr)->GetNextNodeIndex());
	nextNodeIndex += chunkSize;
	
	genClock.StartClock(genClockName);
	(*itr)->GenerateNodes(rmap->GetEnvironment(), Stats, cd, dm, sub_nodes);  //this is the original GenerateNodes function
	genClock.StopClock();
	
	
	//Select up to numRepNodesPerCC nodes from each CC of the existing roadmap and store them in rep_cfgs
	if (rmap->m_pRoadmap->GetVertexCount()  > 0 && numRepNodesPerCC > 0) //starts from 2nd chunk
	  {
	    firstChunk = false;
	    vector< pair<int,VID> > ccs;
	    GetCCStats(*(rmap->m_pRoadmap),ccs);
	    rep_cfgs.clear(); 
	    
	    for (int num_cc = 0; num_cc< ccs.size(); num_cc++) {
	      VID cc_vid = ccs[num_cc].second;                //VID that represents this cc
	      CFG cc_cfg = rmap->m_pRoadmap->GetData(cc_vid);  //CFG for this vertex
	      vector<CFG> cc_cfgs;                          //get CFGs in this cc
	      GetCC(*(rmap->m_pRoadmap), cc_cfg, cc_cfgs);
	      int cc_size = cc_cfgs.size();
	      if(cc_size < numRepNodesPerCC )  //select all nodes from this cc if its side is smaller than numRepNodesPerCC
		{
		  for(int j=0; j< cc_size;j++){
		    rep_cfgs.push_back(cc_cfgs[j]);
		  }
		  
		} 
	      else {  //only select numRepNodesPerCC nodes from this cc
		
		vector <bool> indices(cc_size);
		int iSelect;
		for (int j = 0; j < numRepNodesPerCC; j++) //randomly pick #num_pick 
		  {
		    do { iSelect = OBPRM_lrand() % cc_size; } while (indices[iSelect] == true);
		    indices[iSelect] = true;
		    rep_cfgs.push_back(cc_cfgs[iSelect]);
		  }
		
	      }//end else
	    }  //end for num_cc
	  } //end select rep_cfgs
	
	if (gn.addNodes2Map) {
	  rmap->m_pRoadmap->AddVertex(sub_nodes); //add sub_nodes to graph
	}
	
	//also keep these nodes in nodes
	nodes.insert(nodes.end(), sub_nodes.begin(), sub_nodes.end());
	
	//add rep_cfgs from existing roadmap to sub_nodes
	//we do this from the 2nd chunk && numRepNodesPerCC > 0
	if (!firstChunk && numRepNodesPerCC > 0) {
	  for (int j=0; j<rep_cfgs.size();j++) 
	    sub_nodes.push_back(rep_cfgs[j]);
	} 
	
	vector<vector<CFG> > verticesList;
	verticesList.push_back(sub_nodes);
	verticesList.push_back(sub_nodes);
	
	//If the connection method does not take vector<vector<CFG> > as argument,
	//we will use all the nodes in the map to perform connection
	//see: ConnectionMethod.h
	conClock.StartClock(conClockName);
	cm.ConnectComponents(rmap, Stats, cd, dm, lp,
			     addPartialEdge, addAllEdges, verticesList);
	conClock.StopClock();
	
	//set the next node index for this generation method
	(*itr)->SetNextNodeIndex(nextNodeIndex);
	
	//keep track of gen/con time
	genTime += genClock.GetClock_SEC();
	conTime += conClock.GetClock_SEC();
#ifndef QUIET
        cout<<"\n "; genClock.PrintName(); cout << " ";
        cout << genClock.GetClock_SEC() << " sec  ";
	cout<<"\n "; conClock.PrintName(); cout << " ";
        cout << conClock.GetClock_SEC() << " sec  \n";
#endif
	
      } //end for numChunks
      
#ifndef QUIET
      cout<<endl<< (*itr)->GetName()<<endl;
      cout <<" Subtotal node generation time: "<<genTime << " sec  \n";
      cout <<" Subtotal node connection time: "<<conTime << " sec  \n";
#endif

      (*itr)->numNodes.PutValue(oriNumNodes);
      
      expandGenTime += genTime;
      expandConTime += conTime;

      totalGenTime += genTime;
      totalConTime += conTime;
      
    } //end of for itr = gn.selected

#ifndef QUIET
    cout <<endl<<"Total node generation time for expansion round: "<<expandGenTime << " sec  \n";
    cout <<"Total node connection time for expansion round: "<<expandConTime << " sec  \n";
#endif

    Clock_Class evaluationClock;
    evaluationClock.StartClock("Evaluation time");
    finished = stop(rmap);
    evaluationClock.StopClock();
    totalEvalTime += evaluationClock.GetClock_SEC();

#ifndef QUIET
    cout << "Evaluation time: " << evaluationClock.GetClock_SEC() << endl;

    if(!finished)
      cout << "\nMap failed evaluation test\n";
#endif
  } while(!finished);

#ifndef QUIET
  cout << "\nMap passed evaluation test\n";

  cout << endl << "Total node generation time: " << totalGenTime << " sec  \n";
  cout << "Total node connection time: " << totalConTime << " sec  \n";
  cout << "Total evaluation time: " << totalEvalTime << " sec  \n";
#endif
} 


//This is the regular method we use: 
// 1. read/generate all nodes, 
// 2. connect them to form a roadmap
template <class CFG, class WEIGHT>
void 
MapGenerator<CFG, WEIGHT>::
GenerateNormalMap(Roadmap<CFG, WEIGHT>* rmap, Stat_Class& Stats,
            CollisionDetection* cd, 
            DistanceMetric* dm, vector<CFG>& nodes, 
	    LocalPlanners<CFG,WEIGHT>* lp,
	    Input* input,
	    bool addPartialEdge,
	    bool addAllEdges){

  Clock_Class        NodeGenClock;
  Clock_Class        ConnectionClock;
  nodes.erase(nodes.begin(),nodes.end());

   
  if ( input->inmapFile.IsActivated() ){
    //---------------------------
    // Read roadmap nodes
    //---------------------------
    rmap->ReadRoadmapGRAPHONLY(input->inmapFile.GetValue());
  } else {
    //---------------------------
    // Generate roadmap nodes
    //---------------------------
    NodeGenClock.StartClock("Node Generation");
    vector<CFG> nodes;
    gn.GenerateNodes(rmap,Stats,cd,dm,nodes);
    NodeGenClock.StopClock();
  }


  #ifdef QUIET
  #else
    cout << "\n";
    if ( input->inmapFile.IsActivated() ){
      cout << "Read nodes from the existing map:";
    } else{
      cout << "Node Generation: " << NodeGenClock.GetClock_SEC()
           << " sec (ie, " << NodeGenClock.GetClock_USEC() << " usec),";
    }
    
    cout << " "<<rmap->m_pRoadmap->GetVertexCount()<<" nodes\n"<< flush;
  #endif


  //---------------------------
  // Connect roadmap nodes
  //---------------------------
  ConnectionClock.StartClock("Node Connection");
  cm.ConnectComponents(rmap, Stats, cd, dm, lp,
	       addPartialEdge, addAllEdges);
  ConnectionClock.StopClock();

} 


#endif // !defined(_MAP_GENERATOR_H_)
