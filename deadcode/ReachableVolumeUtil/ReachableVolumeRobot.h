
#ifndef rv_robot
#define rv_robot

#include "ReachableVolumeLinkage.h"
#include "deque"
#include "queue"
class ReachableVolumeLinkage;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class ReachableVolumeRobot{

 public:
  int m_nAttempts;
  static const bool m_debug = false;
  vector<shared_ptr<ReachableVolumeLinkage> > m_RVLinkages;
  int m_nLinks;
  vector<int> m_earLinks;
  vector<int> m_earParentLinks;
  map<int,shared_ptr<ReachableVolumeJoint> > m_RVSpaceJointConstraints;
  Vector3d m_baseJointPos;
  bool m_fixed;

  ReachableVolumeRobot(int _nAttempts = 10000){
    m_nAttempts = _nAttempts;
    m_fixed=true;
  }

  ReachableVolumeRobot(char* _filename, int _nAttempts = 250){
    m_nAttempts = _nAttempts;
    m_fixed=true;
  }

  void printSizeOf(){
    cout<<"Size of m_RVLinkages = "<<sizeof(m_RVLinkages)<<endl;
    cout<<"Size of m_earLinks = "<<sizeof(m_earLinks)<<endl;
    cout<<"Size of m_earParentLinks = "<<sizeof(m_earParentLinks)<<endl;
    cout<<"Size of m_RVSpaceJointConstraints = "<<sizeof(m_RVSpaceJointConstraints)<<endl;
  }

  //assumptions made about tree file
  //ear joints are listed with link of ear chain first, followed by link it attaches to in parent chain
  //ear joints are ordered be ear link (in ascending order)
  //assumes that there is only one loop
  //if there is a loop it assumes the loop is formed by the first chain (i.e. the chain that corosponds to m_RVLinkages[0]
  //loop must be specified before ears
  //_treeStructure denotes the structure of the tree that will be built.  Current options are "EndEffectorFirst", "RootFirst" and "Balanced"
  void loadTree(const char* _filename="Chain.tree", string _treeStructure = "EndEffectorFirst"){
    cout<<"loading file for tree"<<endl;
    ifstream ifs(_filename);
    char strData[256];
    if(!(ifs >> strData)) {
      cerr << "Error while reading link values: can't read num_links\n";
      exit(-1);
    }
    if(strcmp(strData, "numLinks") != 0) {
      cerr << "Error while reading link values: can't read num_links\n";
      exit(-1);
    }
    m_nLinks = 0;
    if(!(ifs >> m_nLinks)) {
      cerr << "Error while reading link values: can't read num_links\n";
      exit(-1);
    }
    if(!(ifs >> strData)) {
      cerr << "Error while reading robot type from chain file\n";
      exit(-1);
    }
    if(strcmp(strData, "Fixed") == 0) {
      m_fixed=true;
    }else{
      m_fixed=false;
    }
    vector<pair<double, double> > linkLengths;
    linkLengths.empty();
    bool loop=false;

    for(int i=0; i<m_nLinks; ++i) {
      if(!(ifs >> strData)) {
        cerr << "Error while reading link " << i << " lengths\n";
        exit(-1);
      }
      if(strcmp(strData, "RealLink") != 0) {
        cerr << "Error while reading link " << i << " lengths\n";
        exit(-1);
      }
      int link_id = -1;
      if(!(ifs >> link_id)) {
        cerr << "Error while reading link " << i << " id\n";
        exit(-1);
      }
      double length1 = -1;
      if(!(ifs >> length1)) {
        cerr << "Error while reading link " << i << " min length\n";
        exit(-1);
      }
      double length2 = -1;
      if(!(ifs >> length2)) {
	cerr << "Error while reading link " << i << " max length\n";
	exit(-1);
      }
      linkLengths.push_back(pair<double,double>(length1,length2));
      if(m_debug){
	cout<<"adding linkage length = "<<length1<<"-"<<length2<<endl;
      }
    }
    m_earLinks.clear();
    m_earParentLinks.clear();
    int lastEar=0;
    int nextJointID=1;


    while(ifs >> strData){
      if(strcmp(strData, "Loop") == 0){
	if(m_debug) cout<<"Loop encountered"<<endl;
        loop=true;
      }
      if(strcmp(strData, "EarJoints") == 0){
	//read all ear joints + parse
	if(m_debug) cout<<"encountered ears"<<endl;
	int nEars;
	ifs >> nEars;
	for(int i=0; i<nEars; ++i) {
	  int l_ear,l_base,l_other;
	  ifs>>l_ear>>l_base>>l_other;
	  if(m_debug) cout<<"read ear "<<l_ear<<endl;
	  m_earLinks.push_back(l_ear);
	  m_earParentLinks.push_back(l_base);

	  vector<shared_ptr<ReachableVolumeJoint> > rvJoints;

	  shared_ptr<ReachableVolumeLinkage> rv =
	    shared_ptr<ReachableVolumeLinkage>(new ReachableVolumeLinkage(new vector<pair<double,double> >(linkLengths.begin() + lastEar, linkLengths.begin() + l_ear ), &m_RVSpaceJointConstraints, m_nAttempts, loop));

	  rv->m_baseJointID=nextJointID;
	  nextJointID=l_ear+1;
	  if(m_debug) cout<<"adding subchain of length"<<rv->m_linkLengths->size()<<endl;
	  m_RVLinkages.push_back(rv);
 	  lastEar=l_ear;
	  loop=false;
	}
      }
      if(strcmp(strData, "Constraint") == 0){
        int jointID;
        double min,max;
        double x,y,z;
	string type;
        ifs>>type>>jointID>>x>>y>>z>>min>>max;
	if(m_debug) cout<<"Encountered constraint "<<jointID<<", ("<<x<<","<<y<<","<<z<<"), "<<min<<", "<<max<<endl;
        ReachableVolume *rv = new ReachableVolume(min,max,1);
	if(type.compare("RV")==0){
	  rv->m_base[0]=x;
	  rv->m_base[1]=y;
	  rv->m_base[2]=z;
	  map<int, shared_ptr<ReachableVolumeJoint> >::iterator iter = m_RVSpaceJointConstraints.find(jointID);
	  if(iter==m_RVSpaceJointConstraints.end()){
            m_RVSpaceJointConstraints[jointID]=shared_ptr<ReachableVolumeJoint>(new ReachableVolumeJoint());
	    m_RVSpaceJointConstraints[jointID]->addConstraint(shared_ptr<ReachableVolume>(rv));
	  }else{
	    iter->second->addConstraint(shared_ptr<ReachableVolume>(rv));
	  }
	}else if(type.compare("Workspace")==0){
	  rv->m_baseWorkspace[0]=x;
	  rv->m_baseWorkspace[1]=y;
	  rv->m_baseWorkspace[2]=z;

	}
	if(m_debug) cout<<"jointID="<<jointID<<", type="<<type<<", size="<<m_RVSpaceJointConstraints[jointID]->m_reachableVolumesJoint->size()<<endl;
      }
    }

    //create last linkage in robot
    vector<pair<double,double> > *links_for_linkage = new vector<pair<double,double> >(linkLengths.begin() + lastEar, linkLengths.end());
    shared_ptr<ReachableVolumeLinkage> rv(new ReachableVolumeLinkage(links_for_linkage, &m_RVSpaceJointConstraints, m_nAttempts, loop));
    rv->m_baseJointID=nextJointID;
    if(m_debug) cout<<"adding final subchain of length"<<rv->m_linkLengths->size()<<endl;
    m_RVLinkages.push_back(rv);
    m_RVLinkages.front()->setReachableVolumes(m_earParentLinks, m_RVLinkages, _treeStructure);
  }




  void getEarAngles(shared_ptr<ReachableVolumeLinkage> &_rvNode, double &_theta, double &_psi){
    //for now just set them randomly
    //later when imlimenting multiple loops make it compute angle with respect to parent
    _theta=(float)rand()/float(RAND_MAX);
    _psi=(float)rand()/float(RAND_MAX);
  }




  //in process of replacing deque function with this function
  vector<Vector3d>  *computeReachableVolumeSample(vector<Vector3d> *_sample){
    std::queue<shared_ptr<ReachableVolumeJointTreeNode> > childrenQueue;
    (*_sample)[m_RVLinkages.front()->m_root->m_jointID]=m_RVLinkages.front()->m_root->getJointPos(m_nAttempts);
    for(vector<shared_ptr<ReachableVolumeJointTreeNode> >::iterator i = m_RVLinkages.front()->m_root->m_children.begin(); i!=m_RVLinkages.front()->m_root->m_children.end(); i++){
      childrenQueue.push(*i);
    }
    while(!childrenQueue.empty()){
      ReachableVolumeJoint tmp;
      childrenQueue.front()->m_ReachableVolumeToParent->m_base=(*_sample)[childrenQueue.front()->m_parent->m_jointID];
      tmp.addConstraint(childrenQueue.front()->m_ReachableVolumeToParent);
      tmp.addConstraint(childrenQueue.front()->m_reachableVolumesJoint);
      (*_sample)[childrenQueue.front()->m_jointID]=tmp.getJointPos(m_nAttempts);
      for(vector<shared_ptr<ReachableVolumeJointTreeNode> >::iterator i = childrenQueue.front()->m_children.begin(); i!=childrenQueue.front()->m_children.end(); i++){
        childrenQueue.push(*i);
      }

      childrenQueue.pop();
    }
    //(*_sample)[0]=(0,0,0);
    Vector3d origin; //(0,0,0)
    (*_sample)[0]=origin;
    if(m_debug){
      cout<<endl<<"*************************************"<<endl<<"reachable volume sample = "<<endl;
      for (vector<Vector3d>::iterator i = _sample->begin(); i != _sample->end(); i++){
        cout<<"("<<*i<<")"<<endl;
      }
      cout<<"*************************************"<<endl;
      cout<<endl<<"*************************************"<<endl<<"Angles between nodes = "<<endl;
      for (unsigned int i = 1; i < _sample->size()-1; i++){
        double angle_between_links = ReachableVolumeJoint::CosineAngle(((*_sample)[i-1]),((*_sample)[i]),((*_sample)[i+1]));
        cout<<"angle between links "<<i-1<<" and" <<i<<" = "<<angle_between_links<<", "<<endl;
        cout<<"theta=PI-angle = "<<PI-angle_between_links<<", "<<endl;
        cout<<"theta/2PI = "<<(PI-angle_between_links)/(2*PI)<<endl;
        cout<<endl;
        cout<<"distance between sample = "<<ReachableVolume::distance((*_sample)[i],(*_sample)[i+1])<<endl;
      }
      cout<<"*************************************"<<endl;
    }
    return _sample;
  }


  shared_ptr<deque<Vector3d> >  computeReachableVolumeSample(shared_ptr<deque<Vector3d> > &_sample){
    std::queue<shared_ptr<ReachableVolumeJointTreeNode> > childrenQueue;
    (*_sample)[m_RVLinkages.front()->m_root->m_jointID]=m_RVLinkages.front()->m_root->getJointPos(m_nAttempts);
    for(vector<shared_ptr<ReachableVolumeJointTreeNode> >::iterator i = m_RVLinkages.front()->m_root->m_children.begin(); i!=m_RVLinkages.front()->m_root->m_children.end(); i++){
      childrenQueue.push(*i);
    }
    while(!childrenQueue.empty()){
      ReachableVolumeJoint tmp;
      childrenQueue.front()->m_ReachableVolumeToParent->m_base=(*_sample)[childrenQueue.front()->m_parent->m_jointID];
      tmp.addConstraint(childrenQueue.front()->m_ReachableVolumeToParent);
      tmp.addConstraint(childrenQueue.front()->m_reachableVolumesJoint);
      (*_sample)[childrenQueue.front()->m_jointID]=tmp.getJointPos(m_nAttempts);
      for(vector<shared_ptr<ReachableVolumeJointTreeNode> >::iterator i = childrenQueue.front()->m_children.begin(); i!=childrenQueue.front()->m_children.end(); i++){
	childrenQueue.push(*i);
      }
      childrenQueue.pop();
    }

    //(*_sample)[0]=(0,0,0);
    Vector3d origin; //(0,0,0)
    (*_sample)[0]=origin;

    if(m_debug){
      cout<<endl<<"*************************************"<<endl<<"reachable volume sample = "<<endl;
      for (deque<Vector3d>::iterator i = _sample->begin(); i != _sample->end(); i++){
        cout<<"("<<*i<<")"<<endl;
      }
      cout<<"*************************************"<<endl;
      cout<<endl<<"*************************************"<<endl<<"Angles between nodes = "<<endl;
      for (unsigned int i = 1; i < _sample->size()-1; i++){
        double angle_between_links = ReachableVolumeJoint::CosineAngle(((*_sample)[i-1]),((*_sample)[i]),((*_sample)[i+1]));
        cout<<"angle between links "<<i-1<<" and" <<i<<" = "<<angle_between_links<<", "<<endl;
        cout<<"theta=PI-angle = "<<PI-angle_between_links<<", "<<endl;
        cout<<"theta/2PI = "<<(PI-angle_between_links)/(2*PI)<<endl;
        cout<<endl;
        cout<<"distance between sample = "<<ReachableVolume::distance((*_sample)[i],(*_sample)[i+1])<<endl;
      }
      cout<<"*************************************"<<endl;
    }
    return _sample;
  }


  shared_ptr<vector<double> > getInternalCFGCoordinates(){
    shared_ptr<deque<Vector3d> > sample(new deque<Vector3d>(m_nLinks+1));
    computeReachableVolumeSample(sample);
    return convertToJointAngleSample(sample);
  }


  inline vector<double> *convertToJointAngleSample(const vector<Vector3d> &_sample){
    vector<double> *cfg_data = new vector<double>;
    int linkageRoot;
    vector<Vector3d>::const_iterator front;
    vector<Vector3d>::const_iterator back;
    vector<Vector3d> subset;
    Vector3d refVectorRobot, refVectorLinkage;
    refVectorRobot[0]=-m_RVLinkages.front()->m_linkLengths->front().second;
    refVectorRobot[1]=0;
    refVectorRobot[2]=0;
    int nextFromSample=0;
    for(unsigned int i=0; i<=m_earParentLinks.size(); i++){
      if(i!=0){
        linkageRoot = m_earParentLinks[i-1]+1;
	back = front + m_RVLinkages[i]->m_linkLengths->size();
        subset.clear();
        if(linkageRoot==0){
          refVectorLinkage=refVectorRobot;
        }else{
          refVectorLinkage=_sample[linkageRoot-1];
        }
        subset.push_back(refVectorLinkage);
        subset.push_back(_sample[linkageRoot]);
        for(unsigned int k=0; k<m_RVLinkages[i]->m_linkLengths->size(); k++){
          subset.push_back(_sample[nextFromSample]);
          nextFromSample++;
        }
      }else{
        front = _sample.begin();
        back = front + m_RVLinkages[i]->m_linkLengths->size();
        refVectorLinkage=refVectorRobot;
        subset.clear();
	subset.push_back(refVectorLinkage);
        for(unsigned int k=0; k<=m_RVLinkages[i]->m_linkLengths->size(); k++){
          subset.push_back(_sample[nextFromSample]);
          nextFromSample++;
        }
      }
      vector<double> *cfg_data_linkage = m_RVLinkages[i]->getInternalCFGCoordinatesHelper(subset);
      if(m_RVLinkages[i]->m_loop){
        //angle between last and first link in loop.
        //not used while computing body positions so setting them to 0 (should go back and set them to values for angles later)
        cfg_data_linkage->push_back(0);
        cfg_data_linkage->push_back(0);
      }
      if(i==0){
        cfg_data->insert(cfg_data->end(),  cfg_data_linkage->begin()+2, cfg_data_linkage->end());
      }else{
        cfg_data->insert(cfg_data->end(),  cfg_data_linkage->begin(), cfg_data_linkage->end());
      }
      delete(cfg_data_linkage);
      front=back+1;
    }
    return cfg_data;
  }



  //to be gotten rid of and replaced with function above
  inline shared_ptr<vector<double> > convertToJointAngleSample(shared_ptr<deque<Vector3d> > sample){
    shared_ptr<vector<double> > cfg_data = shared_ptr<vector<double> >(new vector<double>);
    int linkageRoot;
    deque<Vector3d>::iterator front;
    deque<Vector3d>::iterator back;
    shared_ptr<deque<Vector3d> > subset(new deque<Vector3d>);
    Vector3d refVectorRobot, refVectorLinkage;
    refVectorRobot[0]=-m_RVLinkages.front()->m_linkLengths->front().second;
    refVectorRobot[1]=0;
    refVectorRobot[2]=0;
    int nextFromSample=0;
    for(unsigned int i=0; i<=m_earParentLinks.size(); i++){
      if(i!=0){
	linkageRoot = m_earParentLinks[i-1]+1;
	back = front + m_RVLinkages[i]->m_linkLengths->size();
	subset->clear();
        for(unsigned int k=0; k<m_RVLinkages[i]->m_linkLengths->size(); k++){
          subset->push_back((*sample)[nextFromSample]);
          nextFromSample++;
        }
      }else{
	front = sample->begin();
	back = front + m_RVLinkages[i]->m_linkLengths->size();
	refVectorLinkage=refVectorRobot;
	subset->clear();
	for(unsigned int k=0; k<=m_RVLinkages[i]->m_linkLengths->size(); k++){
	  subset->push_back((*sample)[nextFromSample]);
	  nextFromSample++;
	}
      }
      if(i!=0){
	subset->push_front((*sample)[linkageRoot]);
	if(linkageRoot==0){
	  refVectorLinkage=refVectorRobot;
	}else{
          refVectorLinkage=(*sample)[linkageRoot-1];
	}
      }
      shared_ptr<vector<double> > cfg_data_linkage = m_RVLinkages[i]->getInternalCFGCoordinates(subset,refVectorLinkage);
      if(m_RVLinkages[i]->m_loop){
	//angle between last and first link in loop.
	//not used while computing body positions so setting them to 0 (should go back and set them to values for angles later)
	cfg_data_linkage->push_back(0);
	cfg_data_linkage->push_back(0);
      }
      if(i==0){
	cfg_data->insert(cfg_data->end(),  cfg_data_linkage->begin()+2, cfg_data_linkage->end());
      }else{
        cfg_data->insert(cfg_data->end(),  cfg_data_linkage->begin(), cfg_data_linkage->end());
      }
      front=back+1;
    }
    return cfg_data;
  }

  shared_ptr<vector<shared_ptr<ReachableVolumeJoint> > > computeReachableVolumesAllPoints(){
    vector<shared_ptr<ReachableVolumeJoint> > rv_joints;
    shared_ptr<vector<shared_ptr<ReachableVolumeJoint> > > rv_joint_robot(new vector<shared_ptr<ReachableVolumeJoint> >);  //reachable volumes to be returned
    bool changed = true;
    int jointID=0;
    for(unsigned int j=0; j<m_RVLinkages.size(); j++){
      shared_ptr<ReachableVolumeLinkage> rvl = m_RVLinkages[j];
      int iterations = rvl->m_linkLengths->size();
      if(j==0) iterations++;
      for(int i=0; i<iterations; i++){
	//set rv_joints and rv_joint_robot based on map
	rv_joint_robot->push_back(shared_ptr<ReachableVolumeJoint>(new ReachableVolumeJoint));
	map<int, shared_ptr<ReachableVolumeJoint> >::iterator iter=m_RVSpaceJointConstraints.find(jointID);
	if(iter!=m_RVSpaceJointConstraints.end()){
	  if(m_debug) cout<<"encountered constraint for joint "<<jointID<<endl;
	  rv_joint_robot->back()->addConstraint(iter->second->m_reachableVolumesJoint);
	}
	jointID++;
      }
    }
    while(changed){
      changed = false;
      int jointID=0;
      for(unsigned int j=0; j<m_RVLinkages.size(); j++){
	shared_ptr<ReachableVolumeLinkage> rvl = m_RVLinkages[j];
	int iterations = rvl->m_linkLengths->size();
	if(j==0) iterations++;
	for(int i=0; i<iterations; i++){
	  if(i!=0){
	    double first = (*(rvl->m_linkLengths))[i].first;
	    double second = (*(rvl->m_linkLengths))[i].second;
	    ReachableVolumeJoint *tmp = (*rv_joint_robot)[jointID-1]->mkSum(first,second);
	    if((*rv_joint_robot)[jointID]->addConstraint(tmp->m_reachableVolumesJoint)){
	      changed=true;
	    }
	    delete tmp;
	  }

	  if(i+1!=iterations){
            double first = (*(rvl->m_linkLengths))[i+1].first;
            double second = (*(rvl->m_linkLengths))[i+1].second;
            ReachableVolumeJoint *tmp = (*rv_joint_robot)[jointID+1]->mkSum(first,second);
            if((*rv_joint_robot)[jointID]->addConstraint(tmp->m_reachableVolumesJoint)){
              changed=true;
            }
            delete tmp;
	  }

	  //check for linkages rooted at
	  vector<shared_ptr<ReachableVolumeLinkage> > *linkagesRootedAt = ReachableVolumeLinkage::getLinkagesRootedAt(m_earParentLinks, m_RVLinkages, jointID);
	  for(unsigned int k=0; k<linkagesRootedAt->size(); k++){
	    if(m_debug)cout<<"processing linkage rooted at "<<jointID<<endl;
	    int rootedLinkageFirst=(*linkagesRootedAt)[k]->m_baseJointID;
            double first = ((*linkagesRootedAt)[k]->m_linkLengths)->front().first;
            double second = ((*linkagesRootedAt)[k]->m_linkLengths)->front().second;
            ReachableVolumeJoint *tmp = (*rv_joint_robot)[rootedLinkageFirst]->mkSum(first,second);
            if((*rv_joint_robot)[jointID]->addConstraint(tmp->m_reachableVolumesJoint)){
              changed=true;
            }
            delete tmp;
	    tmp = (*rv_joint_robot)[jointID]->mkSum(first,second);
	    if((*rv_joint_robot)[rootedLinkageFirst]->addConstraint(tmp->m_reachableVolumesJoint)){
              changed=true;
            }
            delete tmp;
	  }
	  if(m_debug){
            cout<<endl<<"Constraint "<<jointID<<endl;
            (*rv_joint_robot)[jointID]->print();
            cout<<"changed ="<<changed<<endl;
          }
	  jointID++;
	}

	if(rvl->m_loop){
	  if((*rv_joint_robot)[rvl->m_baseJointID-1]->addConstraint((*rv_joint_robot)[rvl->m_baseJointID+iterations-2])){
	    changed = true;
	  }
	  if((*rv_joint_robot)[rvl->m_baseJointID+iterations-2]->addConstraint((*rv_joint_robot)[rvl->m_baseJointID-1])){
	    changed = true;
	  }
	}
      }
    }
    return rv_joint_robot;
  }



  shared_ptr<ReachableVolumeJointTreeNode> findRVJ(int _jointID){
    if(m_debug)cout<<"finding rvj for "<<_jointID<<endl;
    deque<shared_ptr<ReachableVolumeJointTreeNode> > rvj_queue;
    rvj_queue.push_back(m_RVLinkages.front()->m_root);
    while(!rvj_queue.empty()){
      if(rvj_queue.front()->m_jointID==_jointID)
	return rvj_queue.front();
      for(vector<shared_ptr<ReachableVolumeJointTreeNode> >::iterator i = rvj_queue.front()->m_children.begin(); i!= rvj_queue.front()->m_children.end(); i++){
	rvj_queue.push_back(*i);
      }
      rvj_queue.pop_front();
    }
    cout<<"failed to find rvj, returning null"<<endl;
    shared_ptr<ReachableVolumeJointTreeNode> returnValue;
    return returnValue;
  }


  int getNumberTreeLevelsHelper(shared_ptr<ReachableVolumeJointTreeNode> _rvj){
    int maxDepthOfChildren=0;
    for(vector<shared_ptr<ReachableVolumeJointTreeNode> >::iterator i = _rvj->m_children.begin(); i!= _rvj->m_children.end(); i++){
      maxDepthOfChildren=max(getNumberTreeLevelsHelper(*i),maxDepthOfChildren);
    }
    return maxDepthOfChildren+1;
  }

  //gets number of levels in rv tree
  int getNumberTreeLevels(){
    return getNumberTreeLevelsHelper(m_RVLinkages.front()->m_root);
  }


  //sets the rv _rvj to be the reachable volume for that joint given the positioning of its parents in _sample
  //assumes _sample is a workspace sample
  inline void setRVFromSample(ReachableVolumeJointTreeNode &_rvj, ReachableVolumeJointTreeNode &rvjJointId , vector<Vector3d> &_sample){

    _rvj.m_reachableVolumesJoint->clear();
    _rvj.m_jointID = rvjJointId.m_jointID;
    _rvj.m_nLinks = rvjJointId.m_nLinks;
    _rvj.m_ReachableVolumeToParent=rvjJointId.m_ReachableVolumeToParent;
    _rvj.m_parent=rvjJointId.m_parent;

    //add constraints from rvjJointId to _rvj
    _rvj.addConstraint(*(rvjJointId.m_reachableVolumesJoint));
    if(rvjJointId.m_parent!=NULL && rvjJointId.m_jointID != m_RVLinkages.front()->m_root->m_jointID && rvjJointId.m_parent->m_jointID!=-1){
      shared_ptr<ReachableVolume> rvParent = shared_ptr<ReachableVolume>(new ReachableVolume(0,0,rvjJointId.m_ReachableVolumeToParent->m_nLinks));
      rvParent->m_base=_sample[rvjJointId.m_parent->m_jointID]-_sample[0];  //convert to reachable volume space
      ReachableVolumeJointTreeNode rvJointParent;
      rvJointParent.m_reachableVolumesJoint->push_back(rvParent);
      _rvj.addConstraint(rvJointParent.mkSum(_rvj.m_ReachableVolumeToParent->m_rmin,_rvj.m_ReachableVolumeToParent->m_rmax)->m_reachableVolumesJoint);
    }
  }


  //Petrurb _node then check if decendants of joint are still fesiable (and resample if they are not)
  //note that _node and _perturbTowards are workspace coordinates not rvspace coordinates
  bool perturbJoint(vector<Vector3d> &_node, const int _jointID, const Vector3d &_perturbTowards, double _delta, string _repositionPolicy = "Random"){

    shared_ptr<ReachableVolumeJointTreeNode> rvj(new ReachableVolumeJointTreeNode);
    shared_ptr<ReachableVolumeJointTreeNode> rvjJointId = findRVJ(_jointID);

    setRVFromSample(*rvj, *rvjJointId, _node);
    Vector3d repositionedJoint;
    double d = max(ReachableVolume::distance(_perturbTowards,_node[_jointID]),.00001);

    repositionedJoint[0]=(_perturbTowards[0]-(_node[_jointID])[0])*_delta/d+(_node[_jointID])[0];
    repositionedJoint[1]=(_perturbTowards[1]-(_node[_jointID])[1])*_delta/d+(_node[_jointID])[1];
    repositionedJoint[2]=(_perturbTowards[2]-(_node[_jointID])[2])*_delta/d+(_node[_jointID])[2];

    for(vector<shared_ptr<ReachableVolume> >::iterator i=rvj->m_reachableVolumesJoint->begin(); i!=rvj->m_reachableVolumesJoint->end();i++){
      if((*i)->m_rmin==(*i)->m_rmax){
        repositionedJoint=(*i)->getNearestPointInRV(repositionedJoint-_node[0])+_node[0];
      }
    }

    //set reachable volume of rvj to be constrained rv of rvj + mk sums of rvjs of parents from _node
    if(rvj->IsInReachableVolume(repositionedJoint-_node[0])){
      _node[_jointID]=repositionedJoint;

      //for each child of rvj call resampleIfOutOfRv
      for(vector<shared_ptr<ReachableVolumeJointTreeNode> >::iterator i = rvjJointId->m_children.begin(); i!= rvjJointId->m_children.end(); i++){
        bool success = resampleIfOutOfRv(_node, *i, _repositionPolicy);
        if(!success)
          return false;
      }
      //_node[0]=(0,0,0);
      Vector3d origin; //(0,0,0)
      _node[0]=origin;
      if(m_debug) cout <<"returning 2"<<endl;

      return true;
    }

    if(m_debug) cout <<"returning 3"<<endl;

    return false;
  }




  bool resampleIfOutOfRv(vector<Vector3d> &_sample, shared_ptr<ReachableVolumeJointTreeNode> &_rvjJointId, string _repositionPolicy="Random"){
    if(_rvjJointId->m_jointID==-1)
      return true;
    m_nAttempts=100;
    shared_ptr<ReachableVolumeJointTreeNode> _rvj(new ReachableVolumeJointTreeNode);
    setRVFromSample(*_rvj,*_rvjJointId, _sample);
    if(!_rvj->IsInReachableVolume(_sample[_rvjJointId->m_jointID]-_sample[0])){
      if(m_debug) _rvj->print();
      if(_repositionPolicy.compare("Random")==0){
        if(m_debug)cout<<"using policy Random"<<endl;
        _sample[_rvj->m_jointID]=_rvj->getJointPos(m_nAttempts)+_sample[0];
      }else if(_repositionPolicy.compare("Closest")==0){
        if(m_debug)cout<<"using policy Closest"<<endl;
        _sample[_rvj->m_jointID]=_rvj->getNearestPointInRV(_sample[_rvj->m_jointID]-_sample[0])+_sample[0];
      }else{
        cout<<"reposition policy "<<_repositionPolicy<<" not found, defaulting to Random"<<endl;
        _sample[_rvj->m_jointID]=_rvj->getJointPos(m_nAttempts)+_sample[0];
      }
      for(vector<shared_ptr<ReachableVolumeJointTreeNode> >::iterator i = _rvjJointId->m_children.begin(); i!= _rvjJointId->m_children.end(); i++){
        if((*i)->m_jointID!=-1){
          bool success = resampleIfOutOfRv(_sample, *i, _repositionPolicy);
          if(!success)
            return false;
        }
      }
    }
    if(m_debug) cout <<"returning 1"<<endl;
    return true;
  }

};

#endif
