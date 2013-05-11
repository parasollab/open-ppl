#ifndef VECTORSET_H
#define VECTORSET_H

#include <fstream>

template<class MPTraits>
class VectorSet {
  public:

    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename vector<CfgType>::iterator iterator;
    typedef typename vector<CfgType>::const_iterator const_iterator;

    VectorSet() {}

    VectorSet(vector<CfgType>& _cfgs) : m_cfgs(_cfgs) {}

    VectorSet(MPProblemType* _problem, XMLNodeReader& _node) {
      //reads in a list of cfgs from a file
      string filename = _node.stringXMLParameter("filename", true, "", "filename containing witness samples");
      ifstream is(filename.c_str());
      copy(istream_iterator<CfgType>(is), istream_iterator<CfgType>(),
          back_insert_iterator<vector<CfgType> >(m_cfgs));
      is.close();
    }

    size_t size() const { return m_cfgs.size(); }

    iterator begin() { return m_cfgs.begin(); }

    iterator end() { return m_cfgs.end(); }

    const_iterator begin() const { return m_cfgs.begin(); }

    const_iterator end() const { return m_cfgs.end(); }

    static string GetName() { return "VectorSet"; }

  private:

    vector<CfgType> m_cfgs;
};

#endif
