#include "Graph.h"

//===================================================================
//===================================================================
class Task {
public:

//===================================================================
//  Data
//===================================================================

int taskwt;

//===================================================================

//===================================================================
//  Constructors and Destructor
//===================================================================
        Task() {};
 
		
        Task(int _vwt) {
        	taskwt = _vwt;
       	};  
        ~Task() {};

//===================================================================
// Operators
//===================================================================

   inline bool operator== (const Task &task)
                {  return  (taskwt == task.taskwt);
};

   inline Task& operator= (const Task &task)
                {       taskwt = task.taskwt;
                return *this;
};

//===================================================================
//  Other Methods
//===================================================================

        //Getting Data information

	int GetTaskWeight() {return taskwt;}; 

	//Modify data
	void SetTaskWeight(int _wt) {taskwt = _wt;};

	static Task InvalidData() {
		Task c(-1);
		return c;
	};

protected:
private:
};
//===================================================================
//===================================================================
class Weight {
public:

//===================================================================
//  Data
//===================================================================

int edgewt;

//===================================================================

//===================================================================
//  Constructors and Destructor
//===================================================================
        Weight() {};
 
		
        Weight(int _ewt) {
        	edgewt = _ewt;
       	};  
        ~Weight() {};

//===================================================================
// Operators
//===================================================================

   inline bool operator== (const Weight &weight)
                {  return  (edgewt == weight.edgewt);
};

   inline Weight& operator= (const Weight &weight)
                {       edgewt = weight.edgewt;
                return *this;
};

//===================================================================
//  Other Methods
//===================================================================

        //Getting Data information

   int GetEdgeWeight() {return edgewt;}; 

	//Modify data
	void SetEdgeWeight(int _wt) {edgewt = _wt;};

	static Weight InvalidWeight() {
		Weight c(-1);
		return c;
	};

protected:
private:
};

//---------------------------------------------
// Input/Output operators for Task
//---------------------------------------------
  inline istream& operator >> (istream& s, Task& t) { 
     s >> t.taskwt;
	return s;
  };

  inline ostream& operator << (ostream& s, const Task& t) {
  return s << t.taskwt;
  };

//---------------------------------------------
// Input/Output operators for Weight
//---------------------------------------------
  inline istream& operator >> (istream& s, Weight& w) { 
    s >> w.edgewt;
  	return s;
  };

  inline ostream& operator << (ostream& s, const Weight& w) {
    return s << w.edgewt;
  };


int main(int argc, char* argv[]) {

	typedef WeightedGraph<Task,Weight> WtGraph;
	typedef vector<WtVertexType<Task,Weight> >::iterator VI;
	typedef vector<WtVertexType<Task,Weight> >::const_iterator CVI;
	typedef vector<WtEdgeType<Task,Weight> >::iterator EGI;

	WtGraph graph;		//the undirected graph
	

	int n = 4;	       	//n vertices in the graph
	int m = 3;	        //m edges in the graph
			       	//the cell id in the graph
	VID vid,v1id,v2id;     	//vertex id defined as VID (typedefed as int) 
	int tmp,i,j;

	//Add Vertices to Graph
	for(i = 0; i < n; i++) {	//there are n vertices in the graph
		Task task(i*i);
		vid = i;
		graph.AddVertex(task,vid);
		//add a vertex to the graph, with
		//VID vid = i, and VERTEX data = task;
	}

	//Add Edges to Graph
	for(j =0; j < m; j++) {	//there are m edges in the graph
		v1id = j; v2id = j+1;
		Weight edge_wt1(j*j);
		Weight edge_wt2(j*j+1);
		pair<Weight, Weight> wtpair(edge_wt1,edge_wt2);
		tmp = graph.AddEdge(v1id,v2id,wtpair);	
		//add edges between vertex 1 and 2 
		//the edge weight from 1 to 2 is aed1
		//the edge weight from 2 to 1 is aed2
		if(tmp) cout<<"\nEdge from "<<v1id<<" to "<<v2id<<
			  " is not added right."<<endl;
	}

	//Access Vertices in Graph
	int task_wt[10];
	VID task_id[10];
	VID vertex2_id[10];
	int edge_wt[10];
	i=0;
	for(VI vi = graph.v.begin(); vi != graph.v.end(); vi++,i++) {
	   task_wt[i] = vi->data.taskwt;
	   task_id[i] = vi->vid;
	   cout<<"\ntaskwt["<<i<<"]="<<task_wt[i]<<"; task_id["<<i<<"]="<<
	     task_id[i]<<endl;
	   //Access Edges in Graph
	   j=0;
	   for(EGI ei = vi->edgelist.begin(); ei!=vi->edgelist.end(); ei++,j++) {
	     vertex2_id[j]=ei->vertex2id;
	     edge_wt[j] = ei->weight.edgewt;
	     cout<<"\nvertex2_id["<<j<<"]="<<vertex2_id[j]<<"; edge_wt["<<j<<"]="
	       <<edge_wt[j]<<endl;
	   }
	}
	Task* tt;
	tt=graph.GetReferenceofData(0);
	tt->taskwt = 1000;

	graph.DisplayGraph(); 

	CVI v1=graph.v.begin();
	CVI v2=graph.v.begin()+1;
	
	pair<int,VID> p1(1,1);
	pair<int,VID> p2(2,2);
	pair<VID,int> pp1(1,1);
	pair<VID,int> pp2(2,2);

	graph.VID_Compare(*v1,*v2);
	graph.CCVID_Compare(p1,p2); 
	graph. FinishLate(pp1,pp2);

}


