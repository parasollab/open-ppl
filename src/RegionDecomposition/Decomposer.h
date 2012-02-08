
////////////////////////////////////////////////////////////////////////////////////////////
#ifndef DECOMPOSER_H
#define DECOMPOSER_H



//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class Decomposer
//
//
//////////////////////////////////////////////////////////////////////////////////////////


namespace pmpl_detail { 
  typedef boost::mpl::list<
    WorkSpaceDecomposer,
    ClusteringDecomposer,
    > DecompositionMethodList;
}


template <class CFG>
class Decomposer : private ElementSet<DecompositionMethod >, public MPBaseObject
{			 
 public:
  typedef typename ElementSet<DecompositionMethod >::MethodPointer DecompositionPointer;
  
  template <typename MethodList>
  Decomposer() : ElementSet<DecompositionMethod >(MethodList()) {}
  
  Decomposer() : ElementSet<DecompositionMethod >(pmpl_detail::DecomposerMethodList()) {}    
  
  template <typename MethodList>
  Decomposer(XMLNodeReader& _Node, MPProblem* _pProblem, MethodList const&)
    : ElementSet<DecompositionMethod >(MethodList()), MPBaseObject(_pProblem) ;
  

  Decomposer(XMLNodeReader& _Node, MPProblem* _pProblem)
    : ElementSet<DecompositionMethod<CFG> >(pmpl_detail::DecompositionMethodList()), MPBaseObject(_pProblem); 
  
  
  ~Decomposer() {}
  
  DecompositionPointer GetMethod(string _strLabel);
  
 
  virtual void PrintOptions(ostream& _os) { }

  RegionGraph Decompose(DecompositionPointer _dp, shared_ptr<BoundingBox> _bb, vector<VID> _vec);  
  

};

#endif

