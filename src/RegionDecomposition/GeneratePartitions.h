#ifndef PARTITIONSCHEMA_H
#define PARTITIONSCHEMA_H

#include "MetricUtils.h"
#include <functional>
#include "Environment.h"
#include <string>

//////////////////////////////////////////////////////////////////////////////////////////
class Body;
//template <class CFG, class WEIGHT> class Roadmap;

template <class CFG, class WEIGHT> class MPRegion;


class BoundingBox;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG>
class FSPartitioningMethod {
 public:
  FSPartitioningMethod();
  virtual ~FSPartitioningMethod();

  // Access
  virtual char* GetName() = 0;
  virtual void SetDefault();
  //I/O methods
  virtual void ParseCommandLine(int argc, char **argv) = 0;
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;
  virtual FSPartitioningMethod<CFG>* CreateCopy() = 0;

  virtual vector<BoundingBox>  PlaceBoundaries(Environment* _env,
					       StatClass& Stats,
					       BoundingBox &c_boundary,
					       vector<CFG>& free_nodes,
					       vector<CFG>& coll_nodes) = 0;

  void DisplayDetails(string Strategy, int BoxSize, int DOF, int NumPartition);


  int dims; //number of dimensions in bbox, not really needed, get rid of it.
  int n_partitions; //number of partitions per region, by default 2.

  double epsilon; //overlap to leave between partitions
  double percentage_p; //
 protected:
  double bNormalizeRanges;
};

template <class CFG>
FSPartitioningMethod<CFG>::
FSPartitioningMethod():
  dims (-1), n_partitions(2), epsilon(.15), percentage_p(0.66), bNormalizeRanges(true) {
  SetDefault();
}

template <class CFG>
FSPartitioningMethod<CFG>::
~FSPartitioningMethod() {
}

template <class CFG>
void FSPartitioningMethod<CFG>::
SetDefault() {
  //set default values for partitioning parameters
}

template <class CFG>
void FSPartitioningMethod<CFG>::
DisplayDetails( string Strategy, int BoxSize, int DOF, int NumPartition) {
  cout << "Subdivision Strategy: " << Strategy << endl;
  cout << "Bounding Box size(" << BoxSize << ")" << endl;
  cout << "DOFs: " << DOF << endl;
  cout << "Subregions: " << NumPartition << endl;
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG>
class RandomPartitioning: public FSPartitioningMethod<CFG> {
 public:
  RandomPartitioning();
  ~RandomPartitioning();
  char* GetName();
  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual FSPartitioningMethod<CFG>* CreateCopy();

  virtual vector<BoundingBox> PlaceBoundaries(Environment* _env,
					      StatClass& Stats,
					      BoundingBox &c_boundary,
					      vector<CFG>& free_nodes,
					      vector<CFG>& coll_nodes);

};

template <class CFG>
RandomPartitioning<CFG>::
RandomPartitioning() : FSPartitioningMethod<CFG>() {
}

template <class CFG>
RandomPartitioning<CFG>::
~RandomPartitioning() {
}

template <class CFG>
char*
RandomPartitioning<CFG>::
GetName() {
  return "RandomPartitioning";
}

template <class CFG>
void
RandomPartitioning<CFG>::
ParseCommandLine(int argc, char **argv) {

}

template <class CFG>
void
RandomPartitioning<CFG>::
PrintUsage(ostream& _os) {
  _os.setf(ios::left,ios::adjustfield);
  _os << "\n" << GetName() << " ";
  _os.setf(ios::right,ios::adjustfield);
}

template <class CFG>
void
RandomPartitioning<CFG>::
PrintValues(ostream& _os) {
  _os << "\n" << GetName() << " ";
  _os << endl;
}

template <class CFG>
FSPartitioningMethod<CFG>*
RandomPartitioning<CFG>::
CreateCopy() {
  FSPartitioningMethod<CFG> * _copy = new RandomPartitioning<CFG>(*this);
  return _copy;
}

template <class CFG>
vector<BoundingBox> RandomPartitioning<CFG>::
PlaceBoundaries(Environment* _env, StatClass& Stats,
		BoundingBox &c_boundary,
		vector<CFG>& free_nodes,
		vector<CFG>& coll_nodes) {
  c_boundary.Print(cout);

/*   string p_type = "Basic"; */
  if( this->dims == -1 ) {
    this->dims = c_boundary.GetDOFs();
/*     p_type = "BasicND"; */
  }
/*   cout << "Subdivision Strategy: Basic (random)" << endl; */
/*   //int dims = 3; */

  double epsilon;
  int tries = this->dims*5;
  int partition_par;
  double partition_point;
  vector<BoundingBox> subregions;
  double r_radius = _env->GetMultiBody( _env->GetRobotIndex() )->GetBody(0)->GetPolyhedron().m_maxRadius;
  bool done = false;
  for (int i = 0; !done && i < tries; i++) {
    partition_par = 2;//lrand48()%dims; Just for tests (fix later)
    partition_point = c_boundary.GetRandomValueInParameter(partition_par);
    if( partition_par >= 3 ) // an orientation that should be scaled
      epsilon = 0.15;      // later ... right now set it to something small
    else
      epsilon = r_radius; // robot radius
/*     cout << " partition par: " << partition_par << endl; */
/*     cout << " partition point: " << partition_point << endl; */
/*     cout << " epsilon: " << epsilon << endl; */

    subregions = c_boundary.Partition(partition_par,partition_point,epsilon);
    vector<BoundingBox>::iterator itr;
    for (itr = subregions.begin(), done=true; itr < subregions.end(); ++itr) {
      if (!(itr->IfEnoughRoom(partition_par,2*epsilon))) {
	done = false;
	break;
      }
    }
  }//end for

/*   if (done) { // output error	 */
/*     DisplayDetails(p_type, ndboundingBox.size(), this->dims, 2); */
/*     subregions[0].Print(cout); */
/*     subregions[1].Print(cout); */
/*   } */
  return subregions;

}


////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG>
class GapPartitioning: public FSPartitioningMethod<CFG> {
 public:
  GapPartitioning();
  ~GapPartitioning();
  char* GetName();
  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual FSPartitioningMethod<CFG>* CreateCopy();

  virtual vector<BoundingBox> PlaceBoundaries(Environment* _env,
					      StatClass& Stats,
					      BoundingBox &c_boundary,
					      vector<CFG>& free_nodes,
					      vector<CFG>& coll_nodes);

  double FindMaxGapForDim(int Dim, vector<CFG>& nodes, double &gap_width);


};

template <class CFG>
GapPartitioning<CFG>::
GapPartitioning() : FSPartitioningMethod<CFG>() {
}

template <class CFG>
GapPartitioning<CFG>::
~GapPartitioning() {
}

template <class CFG>
char*
GapPartitioning<CFG>::
GetName() {
  return "GapPartitioning";
}

template <class CFG>
void
GapPartitioning<CFG>::
ParseCommandLine(int argc, char **argv) {

}

template <class CFG>
void
GapPartitioning<CFG>::
PrintUsage(ostream& _os) {
  _os.setf(ios::left,ios::adjustfield);
  _os << "\n" << GetName() << " ";
  _os.setf(ios::right,ios::adjustfield);
}

template <class CFG>
void
GapPartitioning<CFG>::
PrintValues(ostream& _os) {
  _os << "\n" << GetName() << " ";
  _os << endl;
}

template <class CFG>
FSPartitioningMethod<CFG>*
GapPartitioning<CFG>::
CreateCopy() {
  FSPartitioningMethod<CFG> * _copy = new GapPartitioning<CFG>(*this);
  return _copy;
}

template <class CFG>
vector<BoundingBox> GapPartitioning<CFG>::
PlaceBoundaries(Environment* _env, StatClass& Stats,
		BoundingBox &c_boundary,
		vector<CFG>& free_nodes,
		vector<CFG>& coll_nodes) {
  BoundingBox *boundingBox = _env->GetBoundary();
  double r_radius =
    _env->GetMultiBody( _env->GetRobotIndex() )->GetBody(0)->GetPolyhedron().m_maxRadius;


  if(this->dims==-1)
    this->dims = boundingBox->GetDOFs();

  /* [ 3 bounding boxes divided on the dimention with the largest
       gap between values.
        left box will be from start of dim to (gap start - eps).
        center box will be from (gap start - 2eps) to (gap start + gap + 2eps)
        right box will be from (gap start + gap + eps) to end of dim.
     ]
   */

  //cout << "Subdivision Strategy: Gap " << endl;

  /*[ best_dim, gap_start, gap_width, i, cur_gap_start, cur_gap_width =
      dimention on which to partition (with the largest gap),
      starting point for the largest gap (of all dim),
      width of the largest gap found (of all dim),
      ?,?,?
    ]
   */
  int best_dim  = -1;

  double cur_val,
	 max_val,
	 gap_start = 0.0,
         gap_width = 0.0,
         cur_gap_start,
         cur_gap_width;

  //compute max gap in x,y,z
  std::pair<double,double> range = boundingBox->GetRange(0);
  double normalizationVal = range.second-range.first;
  range = boundingBox->GetRange(1);
  if (range.second-range.first > normalizationVal)
    normalizationVal = range.second-range.first;
  range = boundingBox->GetRange(2);
  if (range.second-range.first > normalizationVal)
    normalizationVal = range.second-range.first;

  for (int i=0; i < this->dims; i++)
  {
      // Find the largest gap for dim 'i'
      //
      cur_gap_start = FindMaxGapForDim(i, free_nodes, cur_gap_width);


      if( this->bNormalizeRanges ){

	// We are normalizing, so make the largest gap
	// value a percentage of the total space for
	// the current dimention (ie make the range 0.0 - 1.0)
	//
        if(i>2)
        {
          normalizationVal=1;
        }


	//cur_val = cur_gap_width / (dimMax-dimMin);
        cur_val = cur_gap_width / normalizationVal;

      }
      else{
        cur_val = cur_gap_width;
      }

      // If the current gap is wider than all previous,
      // retain the current for partitioning.
      //
      if( cur_val > max_val )
      {
        max_val   = cur_val;
        gap_width = cur_gap_width;
	gap_start = cur_gap_start;
	best_dim  = i;
      }

      //cout << i << ") " << cur_val << "-" << max_val;
      //cout << "  (" << cur_gap_width << "/ (" << normalizationVal << ") ((" << dimMax << "-" << dimMin;
      //cout << " )) " << endl;
  }

  // Find dimentions index in the boundry vector
  //
  bool bDimOk = true;

  // if the best dim is x,y,or z, then
  // set epsilon to the robot's radius.
  // otherwise...set epsilon to the default.
  //
  if( best_dim < 3 ){
    this->epsilon = r_radius;

    // Dimention is ok if the width of the box is greater than 2 * robot
    //
    bDimOk = ((boundingBox->GetRange(best_dim).second - boundingBox->GetRange(best_dim).first) > (2*r_radius))
                     ? true : false;
  }


  // Print out the ndx of the best_dim's starting
  // boundry (in the bounding box vector)
  //

  //cout << " partition : " ;

  int BoundNdx,
    CurBoxNdx,
    TotalBoxes= this->n_partitions;

  double lb[2], cb[2], rb[2];
  double gap_end = gap_start + gap_width;

  //
  // Grab and validate the boundries of the left box.
  //
  lb[0] = boundingBox->GetRange(best_dim).first;
  if( (best_dim >= 0) && (best_dim < 3) ){

    lb[1] = (gap_start - this->epsilon);

    //
    // Using x, y, or z
    //
    // Is the left partition wide enough for robot?
    // Is the boundry valid?
    //
    if( ((lb[1] - lb[0]) < r_radius) ||
	(lb[1] < boundingBox->GetRange(best_dim).first) ){
      //
      // There was a problem..
      // Reset the width of left box to (2 * robot radius)
      //
      lb[1] = lb[0] + (2*r_radius);
    }
  }
  else{

    //
    // Not using x, y, or z
    //
    lb[1] = lb[0] + ((gap_start - lb[0]) * this->percentage_p );
  }

  //
  // Grab and validate the boundries of the right box.
  //
  rb[1] = boundingBox->GetRange(best_dim).second;
  if( (best_dim >= 0) && (best_dim < 3) ){

    rb[0] = (gap_end + this->epsilon);

    //
    // Using x, y, or z
    //
    // Is the right partition wide enough for robot?
    // Is the boundry valid?
    //
    if( ((rb[1] - rb[0]) < r_radius) ||
	(rb[0] > boundingBox->GetRange(best_dim).second) ){

      // There was a problem..
      // Reset the width of left box to (2 * robot radius)
      //
      rb[0] = rb[1] - (2*r_radius);
    }
  }
  else{

    //
    // Not using x, y, or z
    //
    rb[0] = rb[1] - ((rb[1] - gap_end) * this->percentage_p );
  }

  //
  // Grab and validate the boundries of the left box.
  //
  if( (best_dim >= 0) && (best_dim < 3) ){

    cb[0] = (gap_start - (2*this->epsilon));
    cb[1] = (gap_start + gap_width + (2*this->epsilon));

  }
  else{

    //
    // Not using x, y, or z
    //
    cb[0] = (gap_start - ((gap_start - lb[0]) * this->percentage_p ));
    cb[1] = (gap_end   + ((rb[1] - gap_end ) * this->percentage_p ));
  }

  // Verify the center box does not overlap existing boundries.
  //
  if( cb[0] <= boundingBox->GetRange(best_dim).first ){
    cb[0] = (gap_start - ((gap_start - lb[0]) * this->percentage_p ));
  }
  if( cb[1] >= boundingBox->GetRange(best_dim).second ){
    cb[1] = (gap_end   + ((rb[1] - gap_end ) * this->percentage_p ));
  }

  if( (best_dim >= 0) && (best_dim < 3) )
    {
      //
      // Using x, y, or z
      //
      // Is the center partition wide enough for robot?
      //
      if( ((cb[1] - cb[0]) < r_radius) ){

        // There was a problem..
	// Reset the width of center box to (2 * robot radius)
	//
        cb[0] = (gap_start + (gap_width/2)) - r_radius;
        cb[1] = (gap_start + (gap_width/2)) + r_radius;
      }
    }


  BoundingBox& leftbox = *boundingBox;
  BoundingBox& centerbox = *boundingBox;
  BoundingBox& rightbox = *boundingBox;

  this->DisplayDetails("Gap", boundingBox->GetDOFs(), boundingBox->GetDOFs(), TotalBoxes);

  if(this->n_partitions==3)
    {
      leftbox.SetParameter(best_dim,lb[0],lb[1]);
      centerbox.SetParameter(best_dim,cb[0],cb[1]);
      rightbox.SetParameter(best_dim,rb[0],rb[1]);

      cout << "Left box " ;
      leftbox.Print(cout);
      cout << "Center box " ;
      centerbox.Print(cout);
      cout << "Right box " ;
      rightbox.Print(cout);
    }
  else
    {
      leftbox.SetParameter(best_dim,lb[0],cb[1]);
      rightbox.SetParameter(best_dim,rb[0],rb[1]);

      cout << "Left box " ;
      leftbox.Print(cout);
      cout << "Center box " ;
      centerbox.Print(cout);
      cout << "Right box " ;
      rightbox.Print(cout);
    }
  vector<BoundingBox> vb;
  vb.push_back(leftbox);
  vb.push_back(centerbox);
  vb.push_back(rightbox);
  return vb;
}


template <class CFG>
double GapPartitioning<CFG>::
FindMaxGapForDim(int Dim, vector<CFG>& nodes, double &gap_width)
{
  int CurDim;
  vector<double> tmpset,
                 NodeData;
  typename vector<CFG>::const_iterator Node_itr;
  vector<double>::const_iterator Dim_itr;
  vector<double>::iterator itr;
  vector<double>::iterator itr_2;
  double tmpDbl;

  // Loop through all of the nodes, collecting the value of the
  // requested dim. from each.
  //
  for( Node_itr = nodes.begin(); Node_itr < nodes.end(); ++Node_itr )
  {
    NodeData = (*Node_itr).GetData();

    // Loop through the dims until we find the requested..
    //
    for( CurDim = 0, Dim_itr = NodeData.begin();
         CurDim < Dim && Dim_itr < NodeData.end(); CurDim++, ++Dim_itr ){ }

    // if all is valid, save the value..
    //
    if( CurDim == Dim && Dim_itr != NodeData.end() )
    {
      tmpset.push_back( *Dim_itr );
    }
  }

  // Sort the vector of values (simple bubble sort)
  //
  for( itr = tmpset.begin(); itr < (tmpset.end()-1); ++itr )
  {
    for( itr_2 = itr+1; itr_2 < tmpset.end(); ++itr_2 )
    {
      if( *itr > *itr_2 ){
        tmpDbl = *itr_2;
	*itr_2 = *itr;
	*itr   = tmpDbl;
      }
    }
  }

  double curdist;
  double gap_start,
         hold_gap_width = 0;

  // Loop through the values..and find the biggest gap between
  // values.
  //
  itr = tmpset.begin();
  itr_2 = itr;
  for(itr_2++; itr_2 < tmpset.end(); itr++, itr_2++ )
  {
    if( itr_2 < tmpset.end() )
    {
      curdist = fabs(*itr_2 - *itr);

      if( curdist > hold_gap_width )
      {
        hold_gap_width = curdist;
        gap_start = *itr;
      }
    }
  }

  gap_width = hold_gap_width;
  return( gap_start );
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG>
class InformationGainPartitioning: public FSPartitioningMethod<CFG> {
 public:
  InformationGainPartitioning();
  ~InformationGainPartitioning();
  char* GetName();
  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual FSPartitioningMethod<CFG>* CreateCopy();

  virtual vector<BoundingBox> PlaceBoundaries(Environment* _env,
					      StatClass& Stats,
					      BoundingBox &c_boundary,
					      vector<CFG>& free_nodes,
					      vector<CFG>& coll_nodes);

  vector<double> ProjectToAxis( vector<CFG> nodes, int param );
  vector<double> FindPartitionPoints( vector<double> proj_free, vector<double> proj_coll);
  double logFunc( double p_f );
  int NumLessThan( vector<double> points, double v);
  // first partition point second = information gain
  pair<double,double> EvaluatePartitionPoint( vector<double> partition_points, vector<double> proj_free, vector<double> proj_coll );
  double Entropy( int Pf, int Pc);


};

template <class CFG>
InformationGainPartitioning<CFG>::
InformationGainPartitioning() : FSPartitioningMethod<CFG>() {
}

template <class CFG>
InformationGainPartitioning<CFG>::
~InformationGainPartitioning() {
}

template <class CFG>
char*
InformationGainPartitioning<CFG>::
GetName() {
  return "InformationGainPartitioning";
}

template <class CFG>
void
InformationGainPartitioning<CFG>::
ParseCommandLine(int argc, char **argv) {

}

template <class CFG>
void
InformationGainPartitioning<CFG>::
PrintUsage(ostream& _os) {
  _os.setf(ios::left,ios::adjustfield);
  _os << "\n" << GetName() << " ";
  _os.setf(ios::right,ios::adjustfield);
}

template <class CFG>
void
InformationGainPartitioning<CFG>::
PrintValues(ostream& _os) {
  _os << "\n" << GetName() << " ";
  _os << endl;
}

template <class CFG>
FSPartitioningMethod<CFG>*
InformationGainPartitioning<CFG>::
CreateCopy() {
  FSPartitioningMethod<CFG> * _copy = new InformationGainPartitioning<CFG>(*this);
  return _copy;
}

template <class CFG>
vector<BoundingBox> InformationGainPartitioning<CFG>::
PlaceBoundaries(Environment* _env, StatClass& Stats,
		BoundingBox &c_boundary,
		vector<CFG>& free_nodes, vector<CFG>& coll_nodes) {
  BoundingBox * boundingBox = _env->GetBoundary();
  vector< pair<double,double> > ppoints_info_gain;
  string p_type = "InformationGain";
  if( this->dims == -1 ) {
    this->dims = boundingBox->GetDOFs();
    p_type = "InformationGainND";
  }
  for(int i=0; i<this->dims; i++) {
    cout << " checking dimension " << i << endl << flush;
    vector<double> proj_free = ProjectToAxis( free_nodes, i );
    //cout << endl << endl;
    vector<double> proj_coll = ProjectToAxis( coll_nodes, i );
    vector<double> partition_points = FindPartitionPoints( proj_free, proj_coll  );
    pair<double,double> partition_point = EvaluatePartitionPoint( partition_points, proj_free, proj_coll );
    ppoints_info_gain.push_back( partition_point );
  }//end for i<dims



  double largest_gain = -998;
  double largest_tried = -998;
  int index = 0;
  bool found = false;
  int tries = 0;
  int partition_on_axis;
  double partition_point;
  double epsilon_save;
  double r_radius =
    _env->GetMultiBody( _env->GetRobotIndex() )->GetBody(0)->GetPolyhedron().m_maxRadius;
  for(int i=0; i< ppoints_info_gain.size(); i++) {
    if( i < 3 )
      this->epsilon = r_radius;
    else
      this->epsilon = 0.15; //orientation angle...not sure what it should be
    bool attempt = true;
    double v1 = boundingBox->GetRange(i).first;
    double v2 = boundingBox->GetRange(i).second;
    if( fabs( v1 - ppoints_info_gain[i].first)<2*this->epsilon ||
	fabs( v2 - ppoints_info_gain[i].first)<2*this->epsilon )
      attempt = false;
    if (attempt)
    if( ppoints_info_gain[i].second > largest_gain ) {
      largest_gain = ppoints_info_gain[i].second;
      partition_point = ppoints_info_gain[i].first;
      partition_on_axis = i;
      index = i;
      found = true;
      epsilon_save = this->epsilon;
    }
  }//end for i<ppoints


  vector< BoundingBox* > partitions;

  if( !found ) {
    // output error

    partitions.push_back( boundingBox );
    this->DisplayDetails(p_type, boundingBox->GetDOFs(), this->dims, 1);

    partitions[0]->Print(cout);
  }
  else {
    //int partition_on_axis =
    //Environment * env = _rm->GetEnvironment();
    //vector<double> ndboundingBox = env->GetNDBoundingBox(0);
    r_radius =
      _env->GetMultiBody( _env->GetRobotIndex() )->GetBody(0)->GetPolyhedron().m_maxRadius;
    int x;
    x = boundingBox->GetDOFs();
    cout << " radius of robot: " << r_radius << endl;
    cout << " partition point: " << partition_point << endl;
    cout << " partition_on_axis: " << partition_on_axis << endl;
/*     if( partition_on_axis < 3 ) */
/*       this->epsilon = r_radius; */
/*     else */
/*       this->epsilon = 0.15; //orientation angle...not sure what it should be */
    for(int z=0; z<2; z++) {
      //cout << "Bounding Box " << z << ": ";
      vector<double> pee;
      for(int i=0; i < x; i++) {
       	if( z==0 && (i == partition_on_axis) ) {
	  //cout << ndboundingBox[2*i]<<", "<<partition_point+this->epsilon;
	  pee.push_back( boundingBox->GetRange(i).first );
	  pee.push_back( partition_point+epsilon_save );
	}
	else if( z==1 && (i == partition_on_axis) ) {
	  //cout << partition_point-this->epsilon << ", "<< ndboundingBox[2*i+1] ;
	   pee.push_back(partition_point-epsilon_save);
	   pee.push_back(boundingBox->GetRange(i).second);
	}
	else {
	  pee.push_back(boundingBox->GetRange(i).first);
	  pee.push_back(boundingBox->GetRange(i).second);
	}

      }//end for
      BoundingBox * p_bbox = new BoundingBox(*boundingBox);
      p_bbox->SetRange(pee);
      partitions.push_back( p_bbox );
    }//end for z
    this->DisplayDetails(p_type, boundingBox->GetDOFs(), this->dims, 2);
    partitions[0]->Print(cout);
    partitions[1]->Print(cout);
    return partitions;
    //DisplayBoundBox( rightbox , 2);
  }

  cout << "Done with Information GainPartitioning " << endl;
}//end InformationGaidPartitioning


template <class CFG>
vector<double> InformationGainPartitioning<CFG>::
ProjectToAxis( vector<CFG> nodes, int param ){
  vector<double> proj;
  cout << " Projecting to axis " << param << endl << flush;
  for(typename vector<CFG>::iterator iter=nodes.begin(); iter<nodes.end(); iter++) {
    proj.push_back( (*iter).GetSingleParam( param ) );
  }
  //vector<double>::iterator ITR;
  sort( proj.begin(), proj.end() );
  cout << " Done Projecting to axis " << param << endl << flush;
  /*
  for (int i=0; i<proj.size(); i++) {
    cout << i << " \t" << proj[i] << endl;
  }
  */
  return proj;
}


template <class CFG>
vector<double> InformationGainPartitioning<CFG>::
FindPartitionPoints( vector<double> proj_free, vector<double> proj_coll) {
  cout << " Finding Partitions Points " << endl;
  vector<double> partition_points;
  if( proj_free.size()==0 || proj_coll.size()==0 ) {
    cout << " Size of free " << proj_free.size() << " Size of Coll. " << proj_coll.size() << endl <<flush;
    return partition_points;
  }

  int i_free = 0;
  int i_coll = 0;
  // Assume vectors come in sorted
  double min_free = proj_free[i_free];
  double min_coll = proj_coll[i_coll];
  double max_coll = proj_coll[ proj_coll.size()-1 ];
  double max_free = proj_free[ proj_free.size()-1 ];
  bool checking_free;
  bool last = false;
  if( min_free < min_coll ) checking_free = true;
  else checking_free = false;
  cout << " Size of free " << proj_free.size() << " Size of Coll. " << proj_coll.size() << endl <<flush;
  while( ( (i_free<proj_free.size()-1) || (i_coll<proj_coll.size()-1) ) && !last ) {
    double next_free, next_coll;
    if( i_free >= proj_free.size()-1 ) {
      next_free = max_free;
      min_free = max_free;
      last = true;
    }
    else next_free= proj_free[i_free+1];
    if( i_coll >= proj_coll.size()-1 ) {
      next_coll = max_coll;
      min_coll = max_coll;
      last = true;
    }
    else next_coll = proj_coll[i_coll+1];
    //cout << " in while, checking i_free: " << i_free << " i_coll " << i_coll << endl;
    if( checking_free ) { // free is minimum
      if( next_free > min_coll ) {// switch
	partition_points.push_back( (min_free+min_coll)/2 );
	checking_free = false;
	min_free = next_free;
	i_free++;
	if( i_free == proj_free.size()-1) {
	  if( min_free < min_coll ) {
	    partition_points.push_back( (max_free+min_coll)/2 );
	  }
	}
      }
      else { // state doesn't change increment i_free
	i_free++;
	min_free = next_free;
	if( i_free == proj_free.size()-1) {
	  if( min_free < min_coll ) {
	    partition_points.push_back( (max_free+min_coll)/2 );
	  }
	}
      }
    }
    else { //min_coll is minimum
      if( next_coll > min_free ) {
	partition_points.push_back( (min_free+min_coll)/2 );
	checking_free = true;
	min_coll = next_coll;
	i_coll++;
	if( i_coll == proj_coll.size()-1 ) {
	  if( min_coll < max_free ) {
	    partition_points.push_back( (max_free+min_coll)/2 );
	  }
	}
      }
      else {
	i_coll++;
	min_coll = next_coll;
	if( i_coll == proj_coll.size()-1 ) {
	  if( min_coll < max_free ) {
	    partition_points.push_back( (max_free+min_coll)/2 );
	  }
	}
      }
    }

  }//end while
  return partition_points;

}


template <class CFG>
double InformationGainPartitioning<CFG>::
logFunc(double p) {
  if(p == 0) return 0;
  else return (p*log(p)/log(2.0));
}


template <class CFG>
double InformationGainPartitioning<CFG>::
Entropy(int Pf, int Pc) {
  double p_f = (1.0*Pf)/(1.0*Pf + Pc);
  double p_c = 1-p_f;
  double entropy_l = logFunc( p_f ) + logFunc( p_c );
  entropy_l *= -1;
  return entropy_l;
}


template <class CFG>
int InformationGainPartitioning<CFG>::
NumLessThan(vector<double> points, double v) {
  int num_less_than = 0;
  for(int i=0; i< points.size(); i++) {
    if( points[i] < v )
      num_less_than++;
  }
  return num_less_than;
}

template <class CFG>
pair<double,double> InformationGainPartitioning<CFG>::
EvaluatePartitionPoint(vector<double> partition_points, vector<double> proj_free, vector<double> proj_coll) {
  if( partition_points.size() <= 2) {
    pair<double, double> result = pair<double,double>(-999,-999);
    return result;
  }
  double smallest_entropy = 100;
  double information_gain = Entropy( proj_free.size(), proj_coll.size() );
  double partition_point = proj_free[0];
  cout << " total proj free: " << proj_free.size() << ", total coll " << proj_coll.size() << endl;
  for(int i=1; i<partition_points.size()-1; i++) {
    int num_free_for_points = NumLessThan( proj_free, partition_points[i] );
    int num_coll_for_points = NumLessThan( proj_coll, partition_points[i] );
    double e_lt = Entropy( num_free_for_points, num_coll_for_points );
    //cout << " entropy for piece 1: " << e_lt << " free point " << num_free_for_points << " coll points " <<num_coll_for_points << " at point " << partition_points[i] << endl;
    double e_gt = Entropy( proj_free.size() - num_free_for_points,
			   proj_coll.size() - num_coll_for_points );
    //cout << " entropy for piece 2: " << e_gt << " free point " << proj_free.size() -num_free_for_points << " coll points " <<proj_coll.size() -num_coll_for_points << " at point " << partition_points[i] << endl;
    if( (e_lt + e_gt) < smallest_entropy ) {
      smallest_entropy = e_lt + e_gt;
      partition_point = partition_points[i];

    }
  }
  information_gain -= smallest_entropy;
  pair<double, double> result = pair<double,double>(partition_point,information_gain);
  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG>
class PartitionCSpaceRegion {
 public:
  PartitionCSpaceRegion();
  ~PartitionCSpaceRegion();

  static vector<FSPartitioningMethod<CFG>*> GetDefault();

  //int ReadCommandLine(str_param<char *> &partitionType);
  void PrintUsage(ostream& _os);
  void PrintValues(ostream& _os);
  void PrintDefaults(ostream& _os);

  /**Generate nodes according to those in selected vector.
   *_rm New created nodes will be added to this roadmap if addNodes in Map=true.
   *nodes New created nodes are stored here.
   */
  template <class WEIGHT>
    vector<BoundingBox> PlaceBoundaries(MPRegion<CFG, WEIGHT> *region,
					vector< vector<CFG> >& nodes);

  template <class WEIGHT>
    bool isSubdivide(MPRegion<CFG, WEIGHT>* region, int tree_node_height, int maximum_tree_height);

 protected:
  //////////////////////
  // Data
  vector<FSPartitioningMethod<CFG>*> all;

 public:
  vector<FSPartitioningMethod<CFG>*> selected;

};

template <class CFG>
PartitionCSpaceRegion<CFG>::
PartitionCSpaceRegion() {
  RandomPartitioning<CFG>* random_partitioning = new RandomPartitioning<CFG>();
  all.push_back(random_partitioning);

  GapPartitioning<CFG>* gap_partitioning = new GapPartitioning<CFG>();
  all.push_back(gap_partitioning);

  InformationGainPartitioning<CFG>* information_gain_partitioning = new InformationGainPartitioning<CFG>();
  all.push_back(information_gain_partitioning);
}

template <class CFG>
PartitionCSpaceRegion<CFG>::
~PartitionCSpaceRegion() {
  typename vector<FSPartitioningMethod<CFG>*>::iterator I;
  for (I=selected.begin(); I!=selected.end(); I++)
    delete *I;

  for (I=all.begin(); I!=all.end(); I++)
    delete *I;
}

template <class CFG>
vector<FSPartitioningMethod<CFG>*>
PartitionCSpaceRegion<CFG>::
GetDefault() {
  vector<FSPartitioningMethod<CFG>*> Default;
  RandomPartitioning<CFG>* random_partitioning = new RandomPartitioning<CFG>();
  Default.push_back(random_partitioning);
  return Default;
}


/*
template <class CFG>
int
PartitionCSpaceRegion<CFG>::
ReadCommandLine(str_param<char *> &partitionType) {
  typename vector<FSPartitioningMethod<CFG>*>::iterator I;

  for(I=selected.begin(); I!=selected.end(); I++)
    delete *I;
  selected.clear();



  typename vector<FSPartitioningMethod<CFG>*>::iterator itr;


  if (partitionType.IsActivated()) {
    std::istringstream _myistream(partitionType.GetValue());

    int argc = 0;
    char* argv[50];
    char cmdFields[50][100];
    while ( _myistream >> cmdFields[argc] ) {
      argv[argc] = (char*)(&cmdFields[argc]);
      ++argc;
    }

    bool found = false;
    try {
      int cmd_begin = 0;
      int cmd_argc = 0;
      char* cmd_argv[50];
      do {
	//go through the command line looking for method names
	for (itr = all.begin(); itr != all.end(); itr++) {
	  //If the method matches any of the supported methods ...
	  if ( !strcmp( argv[cmd_begin], (*itr)->GetName()) ) {
	    cmd_argc = 0;
	    bool is_method_name = false;
	    do {
	      cmd_argv[cmd_argc] = &(*(argv[cmd_begin+cmd_argc]));
	      cmd_argc++;

	      typename vector<FSPartitioningMethod<CFG>*>::iterator itr_names;
	      is_method_name = false;
	      for (itr_names = all.begin(); itr_names != all.end() &&cmd_begin+cmd_argc < argc; itr_names++)
		if (!strcmp(argv[cmd_begin+cmd_argc],(*itr_names)->GetName())) {
		  is_method_name = true;
		  break;
		}
	    } while (! is_method_name && cmd_begin+cmd_argc < argc);

	    // .. use the parser of the matching method
	    (*itr)->ParseCommandLine(cmd_argc, cmd_argv);
	    // .., set their parameters
/ * 	    (*itr)->cdInfo = &cdInfo; * /
	    //  and push it back into the list of selected methods.
	    selected.push_back((*itr)->CreateCopy());
	    (*itr)->SetDefault();
	    found = true;
	    break;
	  }
	}
	if(!found)
	  break;
	cmd_begin = cmd_begin + cmd_argc;
      } while (cmd_begin < argc);
      if (!found)
	throw BadUsage();
    } catch (BadUsage) {
      cerr << "Command line error" << endl;
      PrintUsage(cerr);
      exit(-1);
    }
  }

  //when there was no method selected, use the default
  if(selected.size() == 0) {
    selected = PartitionCSpaceRegion<CFG>::GetDefault();
/ *     for (itr = selected.begin(); itr != selected.end(); itr++) { * /
/ *       (*itr)->cdInfo = &cdInfo; * /
/ *     } * /
  }
  //cout << "selected:\n";
  //for(int j=0; j<selected.size(); j++)
  //  selected[j]->PrintValues(cout);
}
*/

template <class CFG>
void
PartitionCSpaceRegion<CFG>::
PrintUsage(ostream& _os) {
  typename vector<FSPartitioningMethod<CFG>*>::iterator I;
  for(I=all.begin(); I!=all.end(); I++)
    (*I)->PrintUsage(_os);
}


template <class CFG>
void
PartitionCSpaceRegion<CFG>::
PrintValues(ostream& _os) {
  typename vector<FSPartitioningMethod<CFG>*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    (*I)->PrintValues(_os);
}


template <class CFG>
void
PartitionCSpaceRegion<CFG>::
PrintDefaults(ostream& _os) {
  vector<FSPartitioningMethod<CFG>*> Default;
  Default = GetDefault();
  typename vector<FSPartitioningMethod<CFG>*>::iterator I;
  for(I=Default.begin(); I!=Default.end(); I++)
    (*I)->PrintValues(_os);
}


template <class CFG>
template <class WEIGHT>
vector<BoundingBox>
PartitionCSpaceRegion<CFG>::
PlaceBoundaries(MPRegion<CFG, WEIGHT>* region,
		vector< vector<CFG> >& nodes) {

  cout << "------Starting Partitions-------" << endl;
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  vector<BoundingBox> subregion_boundaries;
  // Get free nodes
  vector< pair<size_t,VID> > ccs1;
  stapl::sequential::vector_property_map< RoadmapGraph<CFG, WEIGHT>,size_t > cmap;
  get_cc_stats(*(region->roadmap.m_pRoadmap), cmap, ccs1);
  cout << " CCS1 (size) " << ccs1.size() << endl;
  vector<CFG> free_nodes;
  for(typename vector< pair<size_t,VID> >::iterator itr=ccs1.begin(); itr<ccs1.end(); itr++) {
    //free_nodes.push_back( region->m_pRoadmap->find_vertex((*itr).second).property() );
    //vector<CFG> tCfg;
    vector<VID> tCfg;
    CFG cc1Cfg = (*(region->roadmap.m_pRoadmap->find_vertex((*itr).second))).property();
    cmap.reset();
    get_cc(*(region->roadmap.m_pRoadmap), cmap, cc1Cfg,tCfg);
    for(typename vector<VID>::iterator itr2=tCfg.begin();itr2 != tCfg.end(); itr2++)
      (*(free_nodes.push_back( region->roadmap.m_pRoadmap->find_vertex(*itr2))).property() );
  }

  vector<CFG> coll_nodes;
  for(int i=0; i< nodes.size(); i++) {
    for(int j=0; j< nodes[i].size(); j++) {
      coll_nodes.push_back( nodes[i][j] );
    }
  }
  cout << " free_nodes " << free_nodes.size() << " coll_nodes " << coll_nodes.size();

  typename vector<FSPartitioningMethod<CFG>*>::iterator itr;

  for ( itr = selected.begin(); itr != selected.end(); itr++ ) {
    StatClass Stats;
    Stats.StartClock((*itr)->GetName());
    cout<<"\n  "; Stats.PrintClock((*itr)->GetName()); cout << " " << flush;

    subregion_boundaries = (*itr)->PlaceBoundaries(region->roadmap.GetEnvironment(), region->feature_stats, *(region->GetBoundingBox()), free_nodes, coll_nodes);
    Stats.StopClock((*itr)->GetName());
    cout << Stats.GetSeconds((*itr)->GetName()) << " sec  \n" << flush;
  }
  cout << "------Stopping Partitions-------" << endl;

  return subregion_boundaries;
};

template <class CFG>
template <class WEIGHT>
bool
PartitionCSpaceRegion<CFG>::isSubdivide(MPRegion<CFG, WEIGHT> *region, int tree_node_height, int maximum_tree_height) {
  cout << " PartitionCSpaceRegion::isSubdivide(maximum_tree_height). TODO Most of the tests to check feasibility" << endl;
  // 	if maximum tree height has been reached, stop
  if (tree_node_height >= maximum_tree_height)
    return false;

  // if a planner has been assigned already, stop
  if (false)
    return false;

  // if the class of this region is the same as the class of the parent (and class is not "nonhomogeneous", stop
  if (false)
    return false;

  // if the region is too small so that there is no room to vary any parameter, stop
  if (false)
    return false;

  // otherwise, it is feasible to subdivide
  return true;
}


#endif //PARTITIONSCHEMA_H
