#if !defined(_MAP_GENERATOR_H_)
#define _MAP_GENERATOR_H_

#include "MetricUtils.h"
#include "GenerateMapNodes.h"
#include "ConnectMap.h"
#include "MapEvaluator.h"

/////////////////////////////////////////////////////////////
//
//    Map Generator
//
/////////////////////////////////////////////////////////////

template <class CFG, class WEIGHT, 
          class GMN = GenerateMapNodes<CFG>, class CM = ConnectMap<CFG,WEIGHT>, 
          class ME = MapEvaluator<CFG,WEIGHT> >
class MapGenerator {
 public:
 
  /////////////////////////////////
  // Constructor and Destructor
  /////////////////////////////////
  
  MapGenerator();
  virtual ~MapGenerator();

  void SetEvaluator(const ME& eval) {
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

  GMN gn;     //used to generate map nodes
  CM cm;   //used to connect map nodes
  GMN old_gn; //used to parse command line from the existing map
  ME stop;
};

template <class CFG, class WEIGHT, class GMN, class CM, class ME>
MapGenerator<CFG, WEIGHT, GMN, CM, ME>::
MapGenerator(){
}

template <class CFG, class WEIGHT, class GMN, class CM, class ME>
MapGenerator<CFG, WEIGHT, GMN, CM, ME>::
~MapGenerator(){}


/*
 Set NextNodeIndex for each method we already used
 Set RNGseed for rmap
 Seed RNG using RNGseed, call SRand()
 Update current command line 
 Backup the old map
*/
template <class CFG, class WEIGHT, class GMN, class CM, class ME>
void
MapGenerator<CFG, WEIGHT, GMN, CM, ME>::
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
    //reset seed for SRand
    if(RNGseed == INVALID_RNGSEED)
      SRand(); //seed RNG using the default value
    else
      SRand(RNGseed); //seed RNG using this seed

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
template <class CFG, class WEIGHT, class GMN, class CM, class ME>
void 
MapGenerator<CFG, WEIGHT, GMN, CM, ME>::
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
template <class CFG, class WEIGHT, class GMN, class CM, class ME>
void 
MapGenerator<CFG, WEIGHT, GMN, CM, ME>::
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
  
  bool isFirstChunk = true; // the first chunk for this map (no matter which gen method we use)
  vector<VID> all_nodesVID; // keep track of all VIDs we already have

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
      vector<VID> sub_nodesVID;
      cout << "current map has "<< rmap->m_pRoadmap->get_num_vertices()<<" nodes"<<endl;
      cout << "the chunk size is: " << chunkSize
	   << ", and the number of chunks is: " << numChunks << endl;
      
      //generate nodes chunk by chunk and seed for each chunk
      //perform connection chunk by chunk:
      for (int i=0; i<numChunks; i++) {
	ClockClass genClock;
	ClockClass conClock;
	char genClockName[100], conClockName[100];      
	sprintf(genClockName, "%s%d%s", "Node generation time in chunk ", i, ": ");
	sprintf(conClockName, "%s%d%s", "Node connection time in chunk ", i, ": ");
	
	cout<<endl<<"Chunk "<<i<<endl;
	sub_nodes.clear();
	sub_nodesVID.clear();
	int nextNodeIndex = (*itr)->GetNextNodeIndex();
	nextNodeIndex += chunkSize;
	
	(*itr)->numNodes.PutValue (chunkSize);
        SRand((*itr)->GetName(), (*itr)->GetNextNodeIndex());
	
	genClock.StartClock(genClockName);
	(*itr)->GenerateNodes(rmap->GetEnvironment(), Stats, cd, dm, sub_nodes);  //this is the original GenerateNodes function
	genClock.StopClock();
	
        if (rmap->m_pRoadmap->get_num_vertices()  > 0 ) 
          isFirstChunk = false;
	
	
	if (gn.addNodes2Map) {
	  for(typename vector<CFG>::iterator S = sub_nodes.begin(); S != sub_nodes.end(); ++S)
	    sub_nodesVID.push_back(rmap->m_pRoadmap->AddVertex(*S)); //add sub_nodes to graph
	}

	//also keep these nodes in nodes
	nodes.insert(nodes.end(), sub_nodes.begin(), sub_nodes.end());
        all_nodesVID.insert(all_nodesVID.end(), sub_nodesVID.begin(), sub_nodesVID.end());	

	//do connection only using nodes in this chunk
	conClock.StartClock(conClockName);
	cm.ConnectNodes(rmap, Stats, cd, dm, lp,
			//addPartialEdge, addAllEdges, sub_nodesVID, sub_nodesVID);
			addPartialEdge, addAllEdges, sub_nodesVID, all_nodesVID);
			     
	//component connection
	//if it is the first chunk, we do component connection among all CCs
	//otherwise, we group CCs into two sets and do component connection between them
        if(isFirstChunk) {
	  cm.ConnectComponents(rmap, Stats, cd, dm, lp,
		     addPartialEdge, addAllEdges);
	}
	else {
	  //get VID sets that represent CCs in the roadmap
	  //all newly generated VID will be put to vids2
	  int numVertex = rmap->m_pRoadmap->get_num_vertices();
	  stapl::vector_property_map< GRAPH,size_t > cmap;
	  vector< pair<size_t,VID> > ccs;
	  cmap.reset();
          get_cc_stats(*(rmap->m_pRoadmap),cmap,ccs);
	  vector<VID> vids1, vids2;
	  for(int j=0; j< ccs.size(); j++){
	    if(ccs[j].second < numVertex - chunkSize)
	      vids1.push_back(ccs[j].second);           
	    else
	      vids2.push_back(ccs[j].second);
	  }

	  cm.ConnectComponents(rmap, Stats, cd, dm, lp,
		     addPartialEdge, addAllEdges, vids1, vids2);
	}
	conClock.StopClock();
	
	//set the next node index for this generation method
	(*itr)->SetNextNodeIndex(nextNodeIndex);
	
	//keep track of gen/con time
	genTime += genClock.GetSeconds();
	conTime += conClock.GetSeconds();
#ifndef QUIET
        cout<<"\n "; genClock.PrintName(); cout << " ";
        cout << genClock.GetSeconds() << " sec  ";
	cout<<"\n "; conClock.PrintName(); cout << " ";
        cout << conClock.GetSeconds() << " sec  \n";
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

    //output map
/*
    char mapname[256];
    sprintf(mapname, "%s.%d.%d.map", input->defaultFile.GetValue(),
            rmap->m_pRoadmap->Get VertexCount(),
            rmap->m_pRoadmap->Get EdgeCount());
    rmap->WriteRoadmap(input, cd, dm, lp, mapname);
*/

    ClockClass evaluationClock;
    evaluationClock.StartClock("Evaluation time");
    finished = stop(rmap);
    evaluationClock.StopClock();
    totalEvalTime += evaluationClock.GetSeconds();

#ifndef QUIET
    cout << "Evaluation time: " << evaluationClock.GetSeconds() << endl;

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
template <class CFG, class WEIGHT, class GMN, class CM, class ME>
void 
MapGenerator<CFG, WEIGHT, GMN, CM, ME>::
GenerateNormalMap(Roadmap<CFG, WEIGHT>* rmap, Stat_Class& Stats,
            CollisionDetection* cd, 
            DistanceMetric* dm, vector<CFG>& nodes, 
	    LocalPlanners<CFG,WEIGHT>* lp,
	    Input* input,
	    bool addPartialEdge,
	    bool addAllEdges){

  ClockClass        NodeGenClock;
  ClockClass        ConnectionClock;
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
      cout << "Node Generation: " << NodeGenClock.GetSeconds()
           << " sec (ie, " << NodeGenClock.GetUSeconds() << " usec),";
    }
    
    cout << " "<<rmap->m_pRoadmap->get_num_vertices()<<" nodes\n"<< flush;
  #endif


  //---------------------------
  // Connect roadmap nodes
  //---------------------------
  ConnectionClock.StartClock("Node Connection");
  cm.Connect(rmap, Stats, cd, dm, lp,
	       addPartialEdge, addAllEdges);
  ConnectionClock.StopClock();

} 


#endif // !defined(_MAP_GENERATOR_H_)
