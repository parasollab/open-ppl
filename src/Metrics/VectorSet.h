#ifndef VECTORSET_H
#define VECTORSET_H

#include <fstream>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class VectorSet {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename vector<CfgType>::iterator Iterator;
    typedef typename vector<CfgType>::const_iterator ConstIterator;

    VectorSet() {}

    VectorSet(vector<CfgType>& _cfgs) : m_cfgs(_cfgs) {}

    VectorSet(XMLNode& _node) {
      //reads in a list of cfgs from a file
      string filename = _node.Read("filename", true, "", "filename containing witness samples");
      ifstream is(filename.c_str());
      copy(istream_iterator<CfgType>(is), istream_iterator<CfgType>(),
          back_insert_iterator<vector<CfgType> >(m_cfgs));
      is.close();
    }

    size_t size() const { return m_cfgs.size(); }

    Iterator begin() { return m_cfgs.begin(); }

    Iterator end() { return m_cfgs.end(); }

    ConstIterator begin() const { return m_cfgs.begin(); }

    ConstIterator end() const { return m_cfgs.end(); }

    static string GetName() { return "VectorSet"; }

  private:

    vector<CfgType> m_cfgs;
};

#endif
