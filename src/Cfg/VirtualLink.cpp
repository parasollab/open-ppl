#include "math.h"
#include <iostream> 
#include "VirtualLink.h"
#include "util.h"

// #define XINYU_DEBUG 1

int Link::IDCount = 0;

Link *FindLCA(Link *left, Link *right)
{
  list <Link*> leftAncestors, rightAncestors;
  leftAncestors.clear();
  rightAncestors.clear();
  
  Link *leftParent = left->parent;
  while (leftParent != NULL){
    leftAncestors.push_back(leftParent);
    leftParent = leftParent->parent;
  }

  Link *rightParent = right->parent;
  while (rightParent != NULL){
    rightAncestors.push_back(rightParent);
    rightParent = rightParent->parent;
  }

  Link *CA = NULL;
  leftParent = leftAncestors.back();
  rightParent = rightAncestors.back();
  while (!leftAncestors.empty() &&
         !rightAncestors.empty() &&
         leftParent == rightParent){
    CA = leftParent;
    leftAncestors.pop_back();
    leftParent = leftAncestors.back();
    rightAncestors.pop_back();
    rightParent = rightAncestors.back();
  }

  return CA;
}

double CalculateJointAngle(Link *prev, Link *next)
{
  Link *root = FindLCA(prev, next);
  Link *left = prev;
  Link *right = next;
  double leftAngle = 0;
  double rightAngle = 0;
  while (left->parent != root){
    leftAngle += left->parent->convexity * left->AngleToParent();
    left = left->parent;
  }

  while (right->parent != root){
    rightAngle += right->parent->convexity * right->AngleToParent();
    right = right->parent;    
  }

  double topJointAngle = left->AngleToSibling();
  if (root->convexity < 0)
    topJointAngle = TWO_PI - topJointAngle;
  
  double bottomJointAngle = topJointAngle + leftAngle + rightAngle;
  return bottomJointAngle;
  
}

Link::Link(double rmin, double rmax){

  ID =   IDCount ++ ;
  length = -1;
  coord = pair<double, double> (0,0);
  reachableRange.min = rmin;
  reachableRange.max = rmax;
  availableRange = reachableRange;

  leftChild = NULL;  
  rightChild = NULL;
  parent = NULL;
  sibling = NULL;
  convexity = 1;
}

Link::Link(Link* c0, Link* c1){
  ID =   IDCount ++ ;
  length = -1;
  coord = pair<double,double>(-1, -1);
  parent = NULL;
  sibling = NULL;  
  

  leftChild = c0;
  c0->parent = this;
  c0->sibling = c1;

  rightChild = c1;
  c1->parent = this;
  c1->sibling = c0;  

  reachableRange = RangeUnion(c0->reachableRange, c1->reachableRange);
  availableRange = reachableRange;
  convexity = 1;
}

Link::~Link() {}

inline bool Link::SetLength(double l){
  
  if ((l >= availableRange.min - EPS_ZERO) && (l <= availableRange.max + EPS_ZERO) ){
    length = availableRange.min = availableRange.max = l;
    return true;
  }else{
    cerr << " Can't set length " << l << endl;
    cerr << "available range:" << availableRange << endl;
    cerr << "reachable range:" << reachableRange << endl;
    return false;
  }
}

//Randomly select a length within availableRange;
double Link::SampleLength(bool bSampleConvex, double gama){
  if (parent != NULL){
    if (!bSampleConvex && parent->convexity == 0){ //it's enforced to be a flat configuration

       Range toReachRange = RangeMinus(parent->availableRange, sibling->availableRange);
       availableRange = RangeIntersection(availableRange, toReachRange);
#ifdef XINYU_DEBUG
      cout << "parent->availableRange:" << parent->availableRange << endl;
      cout << "sibling->availableRange:" << sibling->availableRange << endl;
      cout << "toReachRange:" << toReachRange << endl;
      cout << "availableRange:" << availableRange << endl;
#endif 
    }else{
       Range toReachRange = RangeUnion(parent->availableRange, sibling->availableRange);
       availableRange = RangeIntersection(availableRange, toReachRange);
#ifdef XINYU_DEBUG
      cout << "parent->availableRange:" << parent->availableRange << endl;
      cout << "sibling->availableRange:" << sibling->availableRange << endl;
      cout << "toReachRange:" << toReachRange << endl;
      cout << "availableRange:" << availableRange << endl;
#endif 
    }
  }
  if (availableRange.Size() < -EPS_ZERO){
    cerr << "available Range is empty! " << endl;
    return -1;
  }
  
  SetLength(availableRange.min + (availableRange.max - availableRange.min)*OBPRM_drand());
#ifdef XINYU_DEBUG
  cout << "length:" << length << endl;
#endif
  if (bSampleConvex){
    double dSample = OBPRM_drand();
    convexity = dSample < gama ? 1: -1;
  }
  return length;
  
}

double Link::SampleLength(boost::variate_generator<boost::rand48&, 
			                           boost::uniform_real<> 
			                          >& rand) {
  if (parent != NULL) {
    if (parent->convexity == 0) { //it's enforced to be a flat configuration
      Range toReachRange = RangeMinus(parent->availableRange, sibling->availableRange);
      availableRange = RangeIntersection(availableRange, toReachRange);
    } else {
      Range toReachRange = RangeUnion(parent->availableRange, sibling->availableRange);
      availableRange = RangeIntersection(availableRange, toReachRange);
    }
  }
  if (availableRange.Size() < -EPS_ZERO) {
    cerr << "available Range is empty! " << endl;
    return -1;
  }
  
  SetLength(availableRange.min + (availableRange.max - availableRange.min)*rand());

  return length;
}


bool Link::CanRecursiveClose(){
   if (!leftChild && !rightChild)
       return true;

   if (CanClose() && leftChild->CanRecursiveClose() && rightChild->CanRecursiveClose())
      return true;
   else 
      return false;
}


void Link::FindBreachesRecursive(list<Link*> &breaches){
   if (!leftChild && !rightChild)
       return;

   if (!CanClose())
     breaches.push_back(this);
   leftChild->FindBreachesRecursive(breaches);
   rightChild->FindBreachesRecursive(breaches);
}


bool Link::CanClose() {
  if (!leftChild && !rightChild)
     return true;
  Range r = RangeUnion(leftChild->availableRange, rightChild->availableRange);
  if (availableRange.HasIntersection(r))
    return true;
  else 
    return false;
  
}

void Link::CalculateAngles(){
  AngleToLeft();
  AngleToRight();
  
//   cout << "Angle1 ("<< ID<<"," << leftChild->ID<< "): " << angLeft * 180/PI << endl;//debuginfo
//   cout << "Angle2 ("<< ID<<"," << rightChild->ID<< "): " << angRight * 180/PI << endl;//debuginfo

}

double Link::AngleToParent()
{
  return CosineAngle(length, parent->GetLength(), sibling->GetLength());
}

double Link::AngleToSibling()
{
  return CosineAngle(length, sibling->GetLength(), parent->GetLength());
}

double Link::AngleToLeft()
{
  angLeft =  CosineAngle(length, leftChild->GetLength(), rightChild->GetLength());
  return angLeft;
}

double Link::AngleToRight()
{
  angRight = CosineAngle(length, rightChild->GetLength(), leftChild->GetLength());
  return angRight;
}

bool Link::RecursiveSample(double l, bool bSampleConvex, double gama){
  if (l < -EPS_ZERO){
    if ( SampleLength(bSampleConvex, gama) < 0)
      return false;
  }else{
    if (!SetLength(l)){
      cerr << "Could not close for length " << l  << endl;
      return false;
    }
  }
  if (leftChild != NULL && rightChild != NULL){
    if ( !leftChild->RecursiveSample(-1, bSampleConvex, gama))
      return false;
    if ( !rightChild->RecursiveSample(-1, bSampleConvex, gama))
      return false;
  }
  return true;
}

bool Link::RecursiveSample(boost::variate_generator<boost::rand48&,
			                            boost::uniform_real<>
			                           >& rand, 
			   double l) {
  if (l < -EPS_ZERO){
    if ( SampleLength(rand) < 0)
      return false;
  }else{
    if (!SetLength(l)){
      cerr << "Could not close for length " << l  << endl;
      return false;
    }
  }
  if (leftChild != NULL && rightChild != NULL){
    if ( !leftChild->RecursiveSample(rand, -1))
      return false;
    if ( !rightChild->RecursiveSample(rand, -1))
      return false;
  }
  return true;
}

void Link::ResetTree()
{
  availableRange = reachableRange;
  convexity = 1;
  
  if (leftChild)
    leftChild->ResetTree();
  if (rightChild)
    rightChild->ResetTree();
}

void
Link::PrintLink(ostream& os)
{
  os << "("<<ID;
  if (leftChild != NULL)
    os << "(" << leftChild->ID << "," << rightChild->ID << ")";
  os <<")" << endl;
}

void
Link::ExportTreeLinkLength(vector<double> &lengths, vector<int> &convexities)
{
  lengths.push_back(length);
  convexities.push_back(convexity);
  
  if ( leftChild)
    leftChild->ExportTreeLinkLength(lengths, convexities);
  
  if ( rightChild)
     rightChild->ExportTreeLinkLength(lengths, convexities);
}


void
Link::ExportTreeLinkLength(vector<double> &lengths)
{
  lengths.push_back(length);
  
  if ( leftChild)
    leftChild->ExportTreeLinkLength(lengths);
  
  if ( rightChild)
     rightChild->ExportTreeLinkLength(lengths);
}

void
Link::ExportTreeLinkReachableRange(vector<Range>& ranges) const 
{
  ranges.push_back(reachableRange);
  if(leftChild)
    leftChild->ExportTreeLinkReachableRange(ranges);
  if(rightChild)
    rightChild->ExportTreeLinkReachableRange(ranges);
}

void
Link::ExportTreeLinkAvailableRange(vector<Range> &ranges)
{
  ranges.push_back(availableRange);
  
  if ( leftChild)
    leftChild->ExportTreeLinkAvailableRange(ranges);
  
  if ( rightChild)
     rightChild->ExportTreeLinkAvailableRange(ranges);
}


int
Link::ImportTreeLinkLength(const vector<double> &lengths, const vector<int> &convexities, int index)
{

  availableRange.min = availableRange.max = length = lengths[index];
  convexity = convexities[index++];
  
  if ( leftChild)
    index = leftChild->ImportTreeLinkLength(lengths, convexities, index);
  
  if ( rightChild)
     index = rightChild->ImportTreeLinkLength(lengths, convexities, index);
  return index;  
}

int
Link::ImportTreeLinkLength(const vector<double> &lengths, int index)
{

  length = lengths[index++];
  
  if ( leftChild)
    index = leftChild->ImportTreeLinkLength(lengths, index);
  
  if ( rightChild)
     index = rightChild->ImportTreeLinkLength(lengths, index);
  return index;  
}

int
Link::ImportTreeLinkAvailableRange(const vector<Range> &ranges, int index)
{

  availableRange = ranges[index++];
  
  if ( leftChild)
    index = leftChild->ImportTreeLinkAvailableRange(ranges, index);
  
  if ( rightChild)
     index = rightChild->ImportTreeLinkAvailableRange(ranges, index);
  return index;  
}

void
Link::InterpolateLinkLength(const vector<double> &start, 
                            const vector<double> &increment, 
                            vector<double> &intermediate, 
                            int index)
{
  intermediate.resize(start.size());
  for (int i = 0; i < start.size(); i++){
     intermediate[i] = start[i]+increment[i]*index; 
  }
}

void
Link::InterpolateLinkLength(const vector<double> &start, 
                            const vector<double> &goal, 
                            vector<double> &intermediate, 
                            int nSteps,
                            int index)
{
  intermediate.resize(start.size());
  for (int i = 0; i < start.size(); i++){
     intermediate[i] = start[i] + (goal[i]-start[i])*index/nSteps; 
  }
}

void
Link::FindIncrement(const vector<double> &start, 
              const vector<double> &goal, 
              vector<double> &increment, 
              int nSteps)
{
  increment.resize(start.size());
  for (int i = 0; i < start.size(); i++){
     increment[i] = (goal[i]-start[i])/nSteps; 
  }
}

void
Link::RecursiveBuildAvailableRange(bool bFlat)
{
  if (leftChild == NULL && rightChild == NULL){
      availableRange = reachableRange;
      return;
  }
  
  leftChild->RecursiveBuildAvailableRange(bFlat);
  rightChild->RecursiveBuildAvailableRange(bFlat);

  if (bFlat){
    if (convexity == 0){
      Range sumRange = RangePlus(leftChild->availableRange, rightChild->availableRange);
      availableRange  = RangeIntersection(reachableRange, sumRange);
    }else{
      Range unionRange = RangeUnion(leftChild->availableRange, rightChild->availableRange);
      availableRange = RangeIntersection(reachableRange, unionRange);
    }
  }else{
      availableRange = reachableRange;
  }
}


void
Link::PrintTree(ostream& os)
{
  PrintLink(os);
  
  if (leftChild != NULL)
     leftChild->PrintTree(os);
  if (rightChild != NULL)
     rightChild->PrintTree(os);
  
}

void
Link::ExportTreeLinkConvexity(vector<int> &convexities)
{
  convexities.push_back(convexity);
  
  if ( leftChild)
    leftChild->ExportTreeLinkConvexity(convexities);
  
  if ( rightChild)
     rightChild->ExportTreeLinkConvexity(convexities);
  
}

int
Link::ImportTreeLinkConvexity(const vector<int> &convexities, int index)
{
  convexity = convexities[index++];
  
  if ( leftChild)
    index = leftChild->ImportTreeLinkConvexity(convexities, index);
  
  if ( rightChild)
     index = rightChild->ImportTreeLinkConvexity(convexities, index);
  return index;  
}


Link *BuildTree(int i, int j, vector<Link *>& baseLinks)
{
  Link *root;
  int numLinks = j-i+1;
  
  if (numLinks >=2){
    int numLeftLinks = numLinks/2;
    Link *leftChild = BuildTree(i, i+numLeftLinks-1, baseLinks);
    Link *rightChild = BuildTree(i+numLeftLinks, j, baseLinks);
    root = new Link(leftChild, rightChild);    
  }else if (numLinks == 1){
    root = baseLinks[i];
  }else {
    root = NULL;
  }
  
  return root;
}



Range RangeIntersection(const Range &r1, const Range &r2 )
{
  Range r3;

  r3.min = r1.min > r2.min? r1.min: r2.min;
  r3.max = r1.max < r2.max? r1.max: r2.max;
  
  return r3;
}

Range RangeUnion(const Range &r1, const Range &r2 )
{
  Range r3;
  r3. max = r1.max + r2.max;
  if (r2.max < r1.min){
    r3.min = r1.min - r2.max;
  }else if (r1.max < r2.min){
    r3.min = r2.min - r1.max;
  }else {
    r3.min = 0;
  }

  return r3;
};

Range RangeMinus(const Range &r1, const Range &r2 )
{
  Range r3;
  double l1 = r1.min - r2.max;
  double l2 = r1.max - r2.min;
  r3.min = l1;
  r3.max = l2;
  return r3;
}

Range RangePlus(const Range &r1, const Range &r2 )
{
  Range r3;
  r3. max = r1.max + r2.max;
  r3. min = r1.min + r2.min;
  return r3;
};



double CosineAngle(double a, double b, double c)
{
  //cout << " a: " << a << " b:" << b << " c:" << c << endl;//debuginfo
  double retAngle;
  if ((fabs(a) < EPS_ZERO) || (fabs(b) < EPS_ZERO))
    retAngle =  PI/2;
  else
    retAngle =  acos((a*a+b*b-c*c)/(2*a*b));
    return retAngle;
};


//ostream& operator <<(ostream &o, Loop l){
/*   list<double> angles =  l.GetAllAngles(); */
/*   o << "("; */
/*   for (list<double>::iterator itrJointAngle = l.begin(); itrJointAngle != l.end(); itrJointAngle ++){ */
/*     o << *itrJointAngle << "\t"; */
/*   } */
/*   o << ")" << flush; */
//}  


ostream& operator <<(ostream &o, const Range& r){
  o << "(" << r.min << ", " << r.max << ")";

}

