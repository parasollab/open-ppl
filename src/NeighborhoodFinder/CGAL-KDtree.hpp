#ifndef CGAL_CGALNeighborhoodFinder_H
#define CGAL_CGALNeighborhoodFinder_H
//#include <CGAL/Cartesian_d.h>
//#include <CGAL/Search_traits.h>
//#include <CGAL/Orthogonal_k_neighbor_search.h>
//#include <vector>




typedef CGAL::Cartesian_d<double> K;
typedef int VID;

class CGALNeighborhoodFinder{

public:

CGALNeighborhoodFinder() { };
CGALNeighborhoodFinder(int k, int D) { kclosest = k; dim = D; } 
virtual ~CGALNeighborhoodFinder() { };	
	
class PMPL_Point_d : public K::Point_d {
 public:
  
  template <class InputIterator>
    PMPL_Point_d (int d, InputIterator first, InputIterator last)
    : K::Point_d (d, first, last) {vid=-1;}
  template <class InputIterator>
    PMPL_Point_d (int d, int _vid, InputIterator first, InputIterator last)
    : K::Point_d (d, first, last) {vid=_vid;}

    int vid;
}; //class PMPL_Point_d


typedef CGAL::Search_traits<K::FT, PMPL_Point_d, K::Cartesian_const_iterator_d, K::Construct_cartesian_const_iterator_d> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
typedef Neighbor_search::Tree Tree;
typedef PMPL_Point_d Point_d;

public:

  virtual const std::string GetName () const {
    return CGALNeighborhoodFinder::GetClassName();
  }
  
  static std::string GetClassName()  {
	return "CGALNF";	  
  }
  
  virtual void PrintOptions(std::ostream& out_os) const {
    out_os << this->GetClassName() << std::endl;
  }


// this may end up being a private function used to create an internal
// CGAL kdtree that will be populated by a roadmap in the future
void AddNode( std::vector<double> v, VID id){
	tree.insert(Point_d(dim,id, v.begin(),v.end()));
}


// Find the k closest to _v that is in the set represented by the iterators
// You should create a temporary tree based on the iterators to search
// not optimal, but should still do it...
template <typename InputIterator, typename OutputIterator>
OutputIterator
KClosest(  InputIterator _input_first, InputIterator _input_last, VID _v,
           OutputIterator _out);


// Find the k closest to _v that is in the set represented by the iterators
// You should create a temporary tree based on the iterators to search
// not optimal, but should still do it...
template <typename InputIterator, typename OutputIterator>
OutputIterator
KClosest( InputIterator _input_first, InputIterator _input_last,
	  std::vector<double> _cfg, OutputIterator _out);


// Find the kclosest to _v that is in the internal CGal kdtree
// created by  the "AddPoint" function
template <typename OutputIterator>
OutputIterator
KClosest( VID _v, OutputIterator _out);

// Find the kclosest to _cfg that is in the internal CGal kdtree
// created by  the "AddPoint" function
template <typename OutputIterator>
OutputIterator
KClosest( std::vector<double> _cfg, OutputIterator _out);
  
  
public:



//private:
int kclosest;
Tree tree;
int dim;
  }; // class CGALNeighborhoodFinder 

  
// Find the k closest to _v that is in the set represented by the iterators
// You should create a temporary tree based on the iterators to search
// not optimal, but should still do it...
template <typename InputIterator, typename OutputIterator>
OutputIterator
CGALNeighborhoodFinder::KClosest(  InputIterator _input_first, InputIterator _input_last, 
VID _v, OutputIterator _out){
	
}


// Find the k closest to _v that is in the set represented by the iterators
// You should create a temporary tree based on the iterators to search
// not optimal, but should still do it...
template <typename InputIterator, typename OutputIterator>
OutputIterator
CGALNeighborhoodFinder::KClosest( InputIterator _input_first, InputIterator _input_last,
std::vector<double> _cfg, OutputIterator _out){
	Tree temptree;
	InputIterator V1;
	VID _v = 0;
	for(V1 = _input_first; V1 != _input_last; ++V1)
	{temptree.insert(Point_d(dim, _v, (*V1).begin(),(*V1).end()));
	 ++_v;
	}
	Point_d query(Point_d(dim, _cfg.begin(),_cfg.end()));
	Neighbor_search search(temptree, query, kclosest);
	for(Neighbor_search::iterator it = search.begin(); it != search.end(); ++it){
	  *_out++ = it->first.vid;
	}
	return _out;
}


// Find the kclosest to _v that is in the internal CGal kdtree
// created by  the "AddPoint" function
template <typename OutputIterator>
OutputIterator
CGALNeighborhoodFinder::KClosest( VID _v, OutputIterator _out){
	std::vector<double> _cfg;
	for(Tree::iterator te = tree.begin(); te != tree.end(); ++te){
		if (te->vid == _v){		   
		Point_d::Cartesian_const_iterator itq;
		for(itq=(*te).cartesian_begin ();itq!=(*te).cartesian_end ();++itq) 
			_cfg.push_back(*itq);  	
		break;
		}
	}
	KClosest(_cfg, _out);	
	return _out;	
}


// Find the kclosest to _cfg that is in the internal CGal kdtree
// created by  the "AddPoint" function  
template <typename OutputIterator>
OutputIterator
CGALNeighborhoodFinder::KClosest( std::vector <double> _cfg, OutputIterator _out){
	Point_d query(Point_d(dim, _cfg.begin(),_cfg.end()));
	Neighbor_search search(tree, query, kclosest);
	for(Neighbor_search::iterator it = search.begin(); it != search.end(); ++it){
	  *_out++ = it->first.vid;
	}
	return _out;
}

#endif  // CGAL_CGALNeighborhoodFinder_H
