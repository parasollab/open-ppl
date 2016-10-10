//////////////////////////////////////////////////////////////////////////////////////////
//  class Decomposer
//
//  This class is the ElementSet container for DecompositionMethods. All known
//  Decomposers are defined in the DecompositionMethodList. To access, use
//  GetMethod.
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef DECOMPOSER_H_
#define DECOMPOSER_H_

namespace pmpl_detail {
  typedef boost::mpl::list<
    WorkSpaceDecomposer,
    ClusteringDecomposer
      > DecompositionMethodList;
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
class Decomposer : private ElementSet<DecompositionMethod>, public MPBaseObject {
  public:
    typedef typename ElementSet<DecompositionMethod>::MethodPointer DecompositionPointer;

    //////////////////////////////
    // Constructors
    //////////////////////////////
    template <typename MethodList>
      Decomposer() : ElementSet<DecompositionMethod>(MethodList()) {}

    Decomposer() : ElementSet<DecompositionMethod>(pmpl_detail::DecomposerMethodList()) {}

    template <typename MethodList>
      Decomposer(XMLNode& _node, MPProblem* _problem, MethodList const&)
      : ElementSet<DecompositionMethod>(MethodList()), MPBaseObject(_problem){}

    Decomposer(XMLNode& _node, MPProblem* _problem)
      : ElementSet<DecompositionMethod<CFG> >(pmpl_detail::DecompositionMethodList()),
      MPBaseObject(_problem) {}

    ~Decomposer() {}

    ///////////////////////////////
    // Accessors
    ///////////////////////////////
    DecompositionPointer GetMethod(string _label){
      return ElementSet<DecompositionMethod>::GetMethod(_label);
    }

    virtual void Print(ostream& _os) const {}
};

#endif

