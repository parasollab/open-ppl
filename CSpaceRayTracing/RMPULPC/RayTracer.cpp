///////////////////////////////////////////
// RayTracer.cpp
//    Implementation of RayTracer class
///////////////////////////////////////////

#include "RayTracer.h"
#include <iostream.h>

//  RayTracer::RayTracer(Environment *environment, Cfg source, Cfg target) {
//    this->environment = environment;
//    this->source = source;
//    this->target = target;
//    all_explored = false; //Used by this->exhausted to decide if there are no more rays to trace
//    rays_tested = 0;
//  }

RayTracer::RayTracer(Roadmap *rdmp, CollisionDetection *cd, SID cdsetid, DistanceMetric * dm, SID dmsetid) {
  this->rdmp = rdmp;
  this->cd = cd;
  this->cdsetid = cdsetid;
  this->dm = dm;
  this->dmsetid = dmsetid;
  environment = rdmp->GetEnvironment();
  all_explored = false;
  rays_tested = 0;
  setOptions(string("targetOriented"), 1, 10000, 10000);
}

RayTracer::~RayTracer() {
  rdmp = NULL;
  environment = NULL;
  cd = NULL;
  dm = NULL;
}

void RayTracer::setOptions(string bouncing_mode, int max_rays, int max_bounces, int max_ray_length) {
  this->max_rays = max_rays;
  this->max_bounces = max_bounces;
  this->max_ray_length = max_ray_length;
  //set the bouncing policy
  if (bouncing_mode == string("targetOriented"))
    bouncing_policy =TARGET_ORIENTED;
  else if (bouncing_mode == string("random"))
    bouncing_policy = RANDOM;
  else if (bouncing_mode == string("heuristic"))
    bouncing_policy = HEURISTIC;
  else if (bouncing_mode == string("normal"))
    bouncing_policy = NORMAL;
  else
    bouncing_policy = TARGET_ORIENTED;

}

void RayTracer::connectCCs() {
  unsigned int num_attempts = 0;
  unsigned int max_attempts = 10;

  vector< pair<int,VID> > ccs; //list of CCs in the roadmap
  GetCCStats(*(rdmp->m_pRoadmap), ccs);//get the CCs
  
  while (num_attempts < max_attempts && ccs.size() > 1) {
    Roadmap target_rdmp = Roadmap::Roadmap();
    target_rdmp.environment = rdmp->GetEnvironment();
    bool path_found = false;
    
    vector< pair<int,VID> >::iterator cci = ccs.begin(); //cci is the i'th cc
    unsigned int k = 10;
    
    vector<Cfg> cci_cfgs;
    Cfg cci_tmp = rdmp->m_pRoadmap->GetData(cci->second);
    GetCC(*(rdmp->m_pRoadmap), cci_tmp, cci_cfgs);
    vector<Cfg> rep_cci_cfgs;
    getBoundaryCfgs(cci_cfgs, rep_cci_cfgs, k);
    
    vector< pair<int,VID> >::iterator ccj = cci+1; //ccj is the cc after cci
    //    //Order CCs to attempt to connect them
    //    OrderCCByCloseness(rdmp, dm, info, ccs);
    
    while (ccj <= ccs.end() && !path_found) {
      cout << "one pass"<<endl;
      vector<Cfg> ccj_cfgs;
      Cfg ccj_tmp = rdmp->m_pRoadmap->GetData(ccj->second);
      GetCC(*(rdmp->m_pRoadmap), ccj_tmp, ccj_cfgs);
      vector<Cfg> rep_ccj_cfgs;
      getBoundaryCfgs(ccj_cfgs, rep_ccj_cfgs, k);
      
      if (connectCCs(cci->second, rep_cci_cfgs, ccj->second, rep_ccj_cfgs, target_rdmp)) {
	path_found = true;
      }
      ccj++; 
    }
    
    ccs.clear();
    GetCCStats(*(rdmp->m_pRoadmap), ccs);//get the CCs
    num_attempts++;
    //update	 the roadmap
    vector<VID> target_rdmp_vertices;
    target_rdmp.m_pRoadmap->GetVerticesVID(target_rdmp_vertices);
    ModifyRoadMap(rdmp, &target_rdmp, target_rdmp_vertices);
    
    target_rdmp.environment = NULL;
    target_rdmp.m_pRoadmap = NULL;
  }
}

bool RayTracer::connectCCs(VID cci_id, vector<Cfg> &rep_cci_cfgs, VID ccj_id, vector<Cfg> &rep_ccj_cfgs, Roadmap &target_rdmp) {
  bool path_found = false;

  cout << "connecting CC " << cci_id << " To CC " << ccj_id << endl;
  //first get cfgs representative of cci and ccj
  cout << "\trep_cci_cfgs size: " << rep_cci_cfgs.size();
  cout << "\trep_ccj_cfgs size: " << rep_ccj_cfgs.size() << endl;

  //find paths from cci to ccj
  for (unsigned int i = 0; i < rep_cci_cfgs.size(); ++i) {
    Roadmap ray_rdmp = Roadmap::Roadmap();
    ray_rdmp.environment = rdmp->GetEnvironment();
  
    if (findPath(rep_cci_cfgs[i],  rep_ccj_cfgs[0], &rep_ccj_cfgs, ray_rdmp)) {
      cout << "A path was found" << endl;
      vector<VID> ray_vertices;
      ray_rdmp.m_pRoadmap->GetVerticesVID(ray_vertices);
      //ConnectMapNodes::ModifyRoadMap(&cci, &ray_rdmp, ray_vertices);
      ModifyRoadMap(&target_rdmp, &ray_rdmp, ray_vertices);
      path_found = true;
      ray_rdmp.environment = NULL;
      ray_rdmp.m_pRoadmap = NULL;
      break;
    }
    else
      cout << "No path was found"<< endl;
    ray_rdmp.environment = NULL;
    ray_rdmp.m_pRoadmap = NULL;
  }

  //now from ccj to cci
//    for (unsigned int i = 0; i < rep_ccj_cfgs.size(); ++i)
//      if (findPath(rep_ccj_cfgs[i],  rep_cci_cfgs[0], &rep_cci_cfgs))
//        break;


//    Cfg source = cci.m_pRoadmap->GetData(cci_id);
//    Cfg target = ccj.m_pRoadmap->GetData(ccj_id);

//    findPath(source,target);
  return path_found;
}

bool RayTracer::findPath(Cfg &source, Cfg &target, vector<Cfg> *target_cfgs, Roadmap &ray_rdmp) {
  setTargetCfgs(target_cfgs);
  return findPath(source,target, ray_rdmp);
}

bool RayTracer::findPath(Cfg &source, Cfg &target, Roadmap &ray_rdmp) {
  bool path_found = false;
  setSource(source);
  setTarget(target);  
  setInitialDirection();
  
  do {
    //Trace the ray
    //cout<< "Trying new direction for ray"<<endl;
    path_found= trace(ray_rdmp);
    newDirection();
  } while (!path_found && !exhausted());
  if (path_found)
    return true;
  return false;
}

// Set source of the ray (it is a configuration)
void RayTracer::setSource(Cfg configuration) {
  //cout << "Setting the source of the ray\n";
  source = configuration;
}

// Set target of the ray (it is a configuration)
void  RayTracer::setTarget(Cfg configuration) {
  //cout << "Setting the target (goal)\n";
  target = configuration;
}

void RayTracer::setTargetCfgs(vector<Cfg> *target_cfgs) {
  using_target_vector = true;
  this->target_cfgs = target_cfgs;
}

// Set environment
//  void  RayTracer::setEnvironment(Environment *environment) {
//    cout << "Setting the environment\n";
//    this->environment = environment;
//  }

// Set direction in which the ray to be traced is going to be
// shot for the first time, a policy can be defined to select
// how the direction is generated. In this first implementation
// of the ray tracer I am going to shut rays in the direction
// of the target and when a ray hits an object it is going to 
// bounce in a random direction from it (of course this has to
// change if I want to have a coherent tracer).
void RayTracer::setInitialDirection() {
  //  cout << "Setting direction of the first ray: ";
  switch (bouncing_policy) {
  case TARGET_ORIENTED: direction = target;
    //    cout << "Direction of the target\n";
    break;
  case HEURISTIC:
    cout << "Heuristic method.\n";
    break;
  case RANDOM:
    cout << "Random,\n";
    break;
  case NORMAL:
    cout << "Normal\n";
    break;
  default: direction = target;
    cout << "Direction of the target\n";
    break;
  }
  all_explored = false;
  rays_tested = 0;
  
}

//  bool RayTracer::trace(CollisionDetection *cd, SID cdsetid, CDInfo& cdinfo, 
//  		      DistanceMetric * dm, SID dmsetid) {
bool RayTracer::trace(Roadmap &ray_rdmp) {
  bool path_found = false; //true if a path has been found, false otherwise
  long number_bouncings = 0; //number of bouncings of the ray
  double ray_length = 0; //length of the ray (depending on metrics, I suppose)

  ray.init(source, direction, target); //initialize the ray
  if (using_target_vector)
    ray.setTargetVector(target_cfgs);
  //cout << "looking for a path with " << max_bounces << " max_bounces and " << max_ray_length << " max_ray_length " << endl;
  while (!path_found && number_bouncings < max_bounces &&
	 ray.length() < max_ray_length) {
    if (ray.connectTarget(ray_rdmp.GetEnvironment(), cd, cdsetid, cd->cdInfo, dm, dmsetid)) {
      ray.finish();
      path_found = true;
    }
    else if (ray.collide(ray_rdmp.GetEnvironment(), cd, cdsetid, cd->cdInfo, dm, dmsetid,max_ray_length)) { // if there is a collision
      //cout<< "\tif the collided object is the target's screen\n";
      //cout << "\t\tpath_found=true;\n";
      //cout << "\telse\n";
      //cout << "\t\tray.bounce(collisionPoint, newdirection);\n";
      //cout << "there was a collision, the ray is going to bounce \n";
      ray.bounce(Cfg::GetRandomCfg(ray_rdmp.GetEnvironment()));//consider the two cases commented out
      //cout << "the ray has bounced \n";
      ++number_bouncings;
    }
    else { //the ray has gone further than the max_ray_length without colliding;
      path_found = false;
    }
  }
  if (path_found) {    
    //ray.save(environment);
    //ray.addRoadmapNodes(ray_rdmp);
    //ray.writePath(environment);
  }
  ray.addRoadmapNodes(ray_rdmp);
  return path_found;
}

//This function sets a direction to trace a new ray. This direction
//is not related with bouncings at all!.
void RayTracer::newDirection() {
  //cout << "sets a new Direction according to the policy defined in setDirection\n";
  rays_tested++;
  if (rays_tested>=max_rays)
    all_explored = true;
  //by the momment this is going to be just a random direction
  direction = direction.GetRandomCfg(environment);
  //cout << "new direction set" << endl;
}

bool RayTracer::exhausted() {
  //  cout << "all_explored status:" << all_explored << endl;
  return all_explored;
}

void RayTracer::printPath() {
  cout << "print the path found, remember that it is stored in the roadmap"<<endl;
}

void RayTracer::getBoundaryCfgs(const vector<Cfg> &input, vector<Cfg> &output, unsigned int k) {
  //get center of mass of all Cfgs in input
  Cfg center_mass;
  int i=0;
//    for (i = 1, center_mass = input_rdmp.m_pRoadmap->GetData(input[0]); i < input.size(); ++i) {
//      center_mass = center_mass + input_rdmp.m_pRoadmap->GetData(input[i]);
//    }
  for (i = 1, center_mass = input[0]; i < input.size(); ++i) {
    center_mass = center_mass + input[i];
  }
  center_mass = center_mass / i;

  //  ConnectMapNodes::SortByDistFromCfg(input_rdmp.GetEnvironment(), dm, 
//    cout << "Number of cfgs processed: " << i << ", center of mass: " << center_mass << endl;
  //copy input to distances (pair of distances to center of mass, VID)
  vector <ConnectMapNodes::CfgDistType> distances;
  for (i = 0; i < input.size(); ++i) {
//      double dist = dm->Distance(input_rdmp.GetEnvironment(), center_mass, input_rdmp.m_pRoadmap->GetData(input[i]), dmsetid);
    double dist = dm->Distance(environment, center_mass, input[i], dmsetid);
    //    Cfg current_cfg = input_rdmp.m_pRoadmap->GetData(input[i]);
    //    distances.push_back(VE_DIST_TYPE(Cfg_VE_Type(center_mass, current_cfg), dist));
    //distances.push_back(ConnectMapNodes::CfgDistType(input_rdmp.m_pRoadmap->GetData(input[i]), dist));
    distances.push_back(ConnectMapNodes::CfgDistType(input[i], dist));
  }

  //order distances by distance to center of mass (first field of pair)
  sort(distances.begin(), distances.end(), ptr_fun(ConnectMapNodes::CfgDist_Compare));

  //clear output
  output.clear();
  int lim_inf = distances.size() - 1 - k;

//    cout << "for (i="<< distances.size()-1<<";i>"<<lim_inf<< " && i>=0; --i) k: "<< k <<" liminf:"<<lim_inf<<endl;


  //copy the first k to output
  for (i = distances.size() - 1; (i > lim_inf) && (i >= 0) ; --i) {
    output.push_back(distances[i].first);
//      cout << "processing distances["<<i<<"]"<<endl;
  }
  
}

void RayTracer::ModifyRoadMap(Roadmap *toMap, Roadmap *fromMap, vector<VID> vids){

  Cfg t;
  int i;

  //Add vertex
  for (i=0;i<vids.size();++i) {
    t=fromMap->m_pRoadmap->GetData(vids[i]);
    
    toMap->m_pRoadmap->AddVertex(t);
  } //endfor i


  //-- get edges from _rm connected component and add to submap
  for (i=0;i<vids.size();++i) {
     vector< pair<pair<VID,VID>,WEIGHT> > edges; 
     fromMap->m_pRoadmap->GetIncidentEdges(vids[i], edges);
     
     for (int j=0;j<edges.size();++j) {
       Cfg t1=fromMap->m_pRoadmap->GetData(edges[j].first.first),
           t2=fromMap->m_pRoadmap->GetData(edges[j].first.second);

       toMap->m_pRoadmap->AddEdge(t1,t2, edges[j].second);
     } //endfor j

  } //endfor i

}

