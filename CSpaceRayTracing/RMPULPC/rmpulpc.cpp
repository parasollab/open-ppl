#include <iostream.h>
#include "RayTracer.h"
#include "rmpulpc.h"

#include <Input.h>
#include <Query.h>
#include <GenerateMapNodes.h>
#include <ConnectMapNodes.h>
#include <LocalPlanners.h>
#include <DistanceMetrics.h>
#include <CollisionDetection.h>

#include "Stat_Class.h"

Input input; //figure out why the input has to be a global variable
QueryCmds Qinput;
extern Stat_Class Stats;

int main (int argc, char **argv) {
  GenerateMapNodes   gn;
  ConnectMapNodes    cn;
  LocalPlanners      lp;
  DistanceMetric     dm;
  CollisionDetection cd;

  CDInfo info; //stores info about collision detection results, I have to come back
               // to this after checking how to do collision detection
  SID cdsetid;
  SID dmsetid;
  bool path_found=false; //true if a path has been found, false otherwise

  cout << "Reading the environment\n";
  input.ReadCommandLine(argc,argv);
  Roadmap rmap(&input, &cd, &dm, &lp);

  //Initialization of cd, lp, gn, cn, dm, lp
  cd.UserInit(&input,   &gn, &cn );
  lp.UserInit(&input,        &cn );
  gn.UserInit(&input, rmap.GetEnvironment() );
  cn.UserInit(&input, rmap.GetEnvironment() );
  dm.UserInit(&input,   &gn, &lp );

  //these two lines are necessary to be able to use distance metrics correctly.
  //initialization of dmsetids
  dmsetid = cn.cnInfo.dmsetid;
  cdsetid = cn.cnInfo.cdsetid;
  // To set a random free configuration I need an _info variable,
  // I've noticed that _info is a GN variable.
  // I have to find out how to use it
  // Guang says that info is used to define some specifications
  // for the collision detection and generation of the nodes, such
  // as resolution and so on.

  //Get a Free Random Configuration and set the source there
  //Eventually I'll change this for some parameter or for a query
  //read from a file
  Cfg source;
  Cfg target;
  Environment *environment;
  environment = rmap.GetEnvironment();
  //Read the source and target from a file
  Cfg tempCfg;
  
  ifstream  myifstream("narrow.query");
  if (!myifstream) {
    cout << endl << "In ReadQuery: can't open infile\n";
    return 1;
  }
  
  
  tempCfg.Read(myifstream);
  source=tempCfg;
  tempCfg.Read(myifstream);
  target=tempCfg;
  myifstream.close();
  
  cout << "source: " << source << ", target: " << target << "\n";
  
  //source=source.GetFreeRandomCfg(environment,&cd, cdsetid, info);
  //target=target.GetFreeRandomCfg(environment,&cd, cdsetid, info);
  //creates a ray tracer
  RayTracer tracer(environment, source, target);


  //lp.UserInit(&input,      &cn);
  //dm.UserInit(&input, &gn,      &lp);

  //gn.UserInit(&input,                query.rdmp.GetEnvironment());
  //cn.UserInit(&input,                query.rdmp.GetEnvironment());

  //query.initDefaultSetIDs(&cn);
  //Qinput.PrintValues(cout);
  //lp.planners.DisplayLPs();
  //lp.planners.DisplayLPSets();
  //if ( query.PerformQuery(&cd,&cn,&lp,&dm) )
  //  query.WritePath();
  //else
  //  cout << endl << "UNSUCCESSFUL query";


  tracer.setDirection(RT_TARGET_ORIENTED);
  
  while (!path_found && !tracer.exhausted()) {
    //Trace the ray
    cout<< "Trying new direction for ray"<<endl;
    path_found=tracer.trace(&cd, cdsetid, info, &dm, dmsetid);
    tracer.newDirection();
  }
  if (!path_found)
    cout << "Path not found\n";
  else
  tracer.printPath();
}
